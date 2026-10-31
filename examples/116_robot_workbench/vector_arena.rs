//! Independent CPU physics worlds with ordered batch stepping and selective reset.
use crate::arena::{Action, Arena, Config, Observation};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::{
    io::{BufRead, Read, Write},
    path::{Path, PathBuf},
};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct BatchConfig {
    pub environments: Vec<Config>,
    pub workers: usize,
}
#[derive(Debug)]
struct Lane {
    arena: Arena,
    failed: bool,
}
#[derive(Debug)]
pub(crate) struct Batch {
    config: BatchConfig,
    root: PathBuf,
    lanes: Vec<Lane>,
}
#[derive(Debug, Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub(crate) enum Outcome {
    Stepped { observation: Observation },
    Failed { error: String },
}
impl Batch {
    /// Create independent worlds in stable environment-index order.
    pub(crate) fn new(config: BatchConfig, root: &Path) -> Result<Self> {
        ensure!(
            (1..=16).contains(&config.environments.len()),
            "batch needs 1..=16 environments"
        );
        ensure!(
            (1..=8).contains(&config.workers),
            "workers must be in 1..=8"
        );
        let lanes = config
            .environments
            .iter()
            .map(|c| {
                Ok(Lane {
                    arena: Arena::new(c, root)?,
                    failed: false,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        Ok(Self {
            config,
            root: root.to_path_buf(),
            lanes,
        })
    }
    /// Read ordered observations without advancing any environment clock.
    pub(crate) fn observe(&self) -> Result<Vec<Observation>> {
        self.lanes
            .iter()
            .map(|lane| {
                ensure!(!lane.failed, "reset failed environments before observing");
                lane.arena.observe()
            })
            .collect()
    }
    /// Validate all commands first, then step disjoint worlds on CPU workers.
    /// Backend errors are per-environment; successful peers retain their steps.
    pub(crate) fn step(&mut self, actions: &[Action]) -> Result<Vec<Outcome>> {
        ensure!(
            actions.len() == self.lanes.len(),
            "one action per environment is required"
        );
        for (lane, action) in self.lanes.iter().zip(actions) {
            ensure!(!lane.failed, "reset failed environments before stepping");
            lane.arena.validate_action(action)?;
        }
        let chunk_size = self.lanes.len().div_ceil(self.config.workers);
        let run = |lanes: &mut [Lane], actions: &[Action]| {
            lanes
                .iter_mut()
                .zip(actions)
                .map(|(lane, action)| match lane.arena.step(action) {
                    Ok(observation) => Outcome::Stepped { observation },
                    Err(error) => {
                        lane.failed = true;
                        Outcome::Failed {
                            error: format!("{error:#}"),
                        }
                    }
                })
                .collect::<Vec<_>>()
        };
        if self.config.workers == 1 {
            return Ok(run(&mut self.lanes, actions));
        }
        Ok(std::thread::scope(|scope| {
            let workers = self
                .lanes
                .chunks_mut(chunk_size)
                .zip(actions.chunks(chunk_size))
                .map(|(lanes, actions)| scope.spawn(move || run(lanes, actions)))
                .collect::<Vec<_>>();
            workers
                .into_iter()
                .flat_map(|w| w.join().expect("physics worker panicked"))
                .collect()
        }))
    }
    /// Recreate only selected worlds. Invalid indices or construction errors change none.
    pub(crate) fn reset(&mut self, indices: &[usize]) -> Result<Vec<Observation>> {
        let mut unique = std::collections::BTreeSet::new();
        for &index in indices {
            ensure!(
                index < self.lanes.len() && unique.insert(index),
                "invalid or duplicate reset index"
            );
        }
        let replacements = indices
            .iter()
            .map(|&i| Arena::new(&self.config.environments[i], &self.root))
            .collect::<Result<Vec<_>>>()?;
        let observations = replacements
            .iter()
            .map(Arena::observe)
            .collect::<Result<Vec<_>>>()?;
        for (&index, arena) in indices.iter().zip(replacements) {
            self.lanes[index] = Lane {
                arena,
                failed: false,
            };
        }
        Ok(observations)
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case", deny_unknown_fields)]
enum Request {
    Step { actions: Vec<Action> },
    Reset { indices: Vec<usize> },
    Observe,
}

/// Persistent JSON-lines interface for an external learner, with flushed replies.
pub(crate) fn serve(config: BatchConfig, root: &Path) -> Result<()> {
    let mut batch = Batch::new(config, root)?;
    let mut input = std::io::stdin().lock();
    let mut output = std::io::stdout().lock();
    writeln!(
        output,
        "{}",
        serde_json::json!({"kind":"ready", "observations":batch.observe()?})
    )?;
    output.flush()?;
    loop {
        let mut line = Vec::new();
        let read = (&mut input)
            .take(4 * 1024 * 1024 + 1)
            .read_until(b'\n', &mut line)?;
        if read == 0 {
            break;
        }
        ensure!(line.len() <= 4 * 1024 * 1024, "request line exceeds 4 MiB");
        let reply = (|| -> Result<serde_json::Value> {
            let request: Request =
                serde_json::from_slice(&line).context("invalid vector request")?;
            Ok(match request {
                Request::Step { actions } => {
                    serde_json::json!({"kind":"step", "results":batch.step(&actions)?})
                }
                Request::Reset { indices } => {
                    let observations = batch.reset(&indices)?;
                    serde_json::json!({"kind":"reset", "indices":indices, "observations":observations})
                }
                Request::Observe => {
                    serde_json::json!({"kind":"observe", "observations":batch.observe()?})
                }
            })
        })();
        let value =
            reply.unwrap_or_else(|e| serde_json::json!({"kind":"error", "error":format!("{e:#}")}));
        writeln!(output, "{value}")?;
        output.flush()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;
    fn config(workers: usize) -> BatchConfig {
        let mut config: BatchConfig =
            serde_json::from_str(include_str!("scenarios/vector_arena.json")).unwrap();
        config.workers = workers;
        config.environments.push(config.environments[0].clone());
        config
    }
    fn root() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots")
    }
    fn action(steps: u32) -> Action {
        Action {
            steps,
            forces_n: BTreeMap::new(),
            torques_nm: BTreeMap::new(),
        }
    }
    fn json(value: impl Serialize) -> serde_json::Value {
        serde_json::to_value(value).unwrap()
    }

    #[test]
    fn force_batches_validate_before_advancing_and_match_worker_counts() {
        let mut serial = Batch::new(config(1), &root()).unwrap();
        let mut parallel = Batch::new(config(2), &root()).unwrap();
        let mut actions = vec![action(12), action(12), action(12)];
        actions[0].forces_n.insert(
            "alice".into(),
            BTreeMap::from([("wrist_flex".into(), [0.0, 0.0, 1.0])]),
        );
        actions[2].forces_n.insert(
            "alice".into(),
            BTreeMap::from([("unknown".into(), [0.0, 0.0, 1.0])]),
        );
        let before = json(parallel.observe().unwrap());
        assert!(parallel.step(&actions).is_err());
        assert_eq!(before, json(parallel.observe().unwrap()));
        actions[2].forces_n.clear();
        assert_eq!(
            json(serial.step(&actions).unwrap()),
            json(parallel.step(&actions).unwrap())
        );
        assert_ne!(
            json(parallel.observe().unwrap())[0],
            json(parallel.observe().unwrap())[1]
        );
    }

    #[test]
    fn independent_worlds_match_sequential_execution_and_selectively_reset() {
        let mut sequential = Batch::new(config(1), &root()).unwrap();
        let mut parallel = Batch::new(config(2), &root()).unwrap();
        let initial = json(parallel.observe().unwrap());
        let mut actions = vec![action(48), action(24), action(12)];
        actions[0].torques_nm.insert(
            "alice".into(),
            BTreeMap::from([("shoulder_pan".into(), 0.03)]),
        );
        assert_eq!(
            json(sequential.step(&actions).unwrap()),
            json(parallel.step(&actions).unwrap())
        );
        // The second lane sees no command or collision from the overlapping first lane.
        let mut isolated = Arena::new(&config(1).environments[1], &root()).unwrap();
        isolated.step(&actions[1]).unwrap();
        let before = json(parallel.observe().unwrap());
        assert_eq!(before[1], json(isolated.observe().unwrap()));
        parallel.reset(&[0]).unwrap();
        let after = json(parallel.observe().unwrap());
        assert_eq!(after[0], initial[0]);
        assert_eq!(after[1], before[1]);
        assert_eq!(after[2], before[2]);
        let mut fresh = Arena::new(&config(1).environments[0], &root()).unwrap();
        let expected = json(fresh.step(&actions[0]).unwrap());
        let step = json(parallel.step(&actions).unwrap());
        assert_eq!(step[0]["observation"], expected);
        assert_eq!(
            step[1]["observation"]["sim_time_ticks"],
            48 * crate::arena::STEP_TICKS
        );
    }

    #[test]
    fn invalid_batch_and_reset_do_not_partially_advance_worlds() {
        let mut batch = Batch::new(config(2), &root()).unwrap();
        let initial = json(batch.observe().unwrap());
        let mut actions = vec![action(24), action(24), action(24)];
        actions[2]
            .torques_nm
            .insert("missing".into(), BTreeMap::new());
        assert!(batch.step(&actions).is_err());
        assert_eq!(json(batch.observe().unwrap()), initial);
        assert!(batch.step(&actions[..2]).is_err());
        assert!(batch.reset(&[0, 3]).is_err());
        assert!(batch.reset(&[0, 0]).is_err());
        assert_eq!(json(batch.observe().unwrap()), initial);
    }

    #[test]
    fn roller_worlds_are_also_worker_count_invariant() {
        let mut one = config(1);
        one.environments.truncate(2);
        for env in &mut one.environments {
            env.idealized_omni = true;
        }
        let mut two = config(2);
        two.environments = one.environments.clone();
        let mut sequential = Batch::new(one, &root()).unwrap();
        let mut parallel = Batch::new(two, &root()).unwrap();
        let actions = [action(48), action(48)];
        assert_eq!(
            json(sequential.step(&actions).unwrap()),
            json(parallel.step(&actions).unwrap())
        );
    }
}
