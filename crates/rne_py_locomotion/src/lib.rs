//! Python bindings for `rne_locomotion`'s learning tasks, as the
//! `rne_py_locomotion` extension module.
//!
//! `rne_locomotion` is not published, so these bindings live apart from
//! `rne_py`, whose crate is: build them with
//! `maturin develop -m crates/rne_py_locomotion/Cargo.toml --release`.
//! The module holds example 147's Go2 walking task as a native parallel
//! batch, [`PyGo2LearningBatch`].

#![deny(missing_docs)]

use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use rne_locomotion::go2_learning::{
    go2_learning_task_spec, Floor, Gaps, Go2LearningBatch, Go2LearningBatchStep, Go2LearningConfig,
    Relief, EPISODE_STEPS,
};

/// Result of a [`PyGo2LearningBatch`] reset or step, in environment order.
#[pyclass(name = "Go2LearningBatchStep", skip_from_py_object)]
#[derive(Clone, Debug)]
pub struct PyGo2LearningBatchStep {
    inner: Go2LearningBatchStep,
}

#[pymethods]
impl PyGo2LearningBatchStep {
    /// Each environment's observation; after a step that ended its episode,
    /// the first observation of the next one.
    #[getter]
    fn observations(&self) -> Vec<Vec<f64>> {
        self.inner.observations.clone()
    }

    /// Each environment's reward; zero at a reset.
    #[getter]
    fn rewards(&self) -> Vec<f64> {
        self.inner.rewards.clone()
    }

    /// Whether the step ended the environment's episode with a fall.
    #[getter]
    fn terminated(&self) -> Vec<bool> {
        self.inner.terminated.clone()
    }

    /// Whether the step ended it at the episode's length.
    #[getter]
    fn truncated(&self) -> Vec<bool> {
        self.inner.truncated.clone()
    }

    /// The last observation of each episode the step ended, before its
    /// reset; `None` where the episode goes on.
    #[getter]
    fn terminal_observations(&self) -> Vec<Option<Vec<f64>>> {
        self.inner.terminal_observations.clone()
    }
}

/// Native parallel batch of example 147's Go2 walking task.
///
/// Each environment walks the Go2 for 250 control periods (5 s): from the
/// origin, or from a random start on a terrain; with or without the 15-ray
/// height scan. Actions are 12 joint offsets in [-1, 1], scaled by 0.5 rad
/// around the default pose. A finished environment is reset at once; its
/// last observation is in `terminal_observations`. Environment `i` draws its
/// starts from seed `seed + i`, and the results do not depend on `threads`.
#[pyclass(name = "Go2LearningBatch")]
#[derive(Debug)]
pub struct PyGo2LearningBatch {
    inner: Go2LearningBatch,
    config: Go2LearningConfig,
}

#[pymethods]
impl PyGo2LearningBatch {
    /// Creates a seeded batch. `terrain_seed=None` is a flat floor; a seed
    /// builds a terrain of `terrain_size` m sampled every `terrain_cell` m,
    /// with `terrain_amplitude` m of relief at `terrain_frequency` cycles per
    /// meter and gaps `gap_depth` m deep (none at 0), each between
    /// `gap_width[0]` and `gap_width[1]` m wide.
    #[new]
    #[pyo3(signature = (num_envs=8, seed=1, threads=0, terrain_seed=None, height_scan=false, random_starts=false, floor_friction=1.0, terrain_size=24.0, terrain_cell=0.1, terrain_amplitude=0.04, terrain_frequency=0.5, gap_depth=0.0, gap_width=(0.25, 0.35)))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        num_envs: usize,
        seed: u64,
        threads: usize,
        terrain_seed: Option<u64>,
        height_scan: bool,
        random_starts: bool,
        floor_friction: f64,
        terrain_size: f64,
        terrain_cell: f64,
        terrain_amplitude: f64,
        terrain_frequency: f64,
        gap_depth: f64,
        gap_width: (f64, f64),
    ) -> PyResult<Self> {
        if num_envs == 0 {
            return Err(PyValueError::new_err("num_envs must be greater than zero"));
        }
        let (narrowest_m, widest_m) = gap_width;
        if !(narrowest_m.is_finite() && narrowest_m > 0.0 && widest_m >= narrowest_m) {
            return Err(PyValueError::new_err(
                "gap_width must be (narrowest, widest) with 0 < narrowest <= widest",
            ));
        }
        let floor = match terrain_seed {
            None => Floor::Flat,
            Some(seed) => Floor::Terrain {
                seed,
                relief: Relief {
                    frequency_per_m: terrain_frequency,
                    amplitude_m: terrain_amplitude,
                    size_m: terrain_size,
                    cell_m: terrain_cell,
                    gaps: Gaps {
                        depth_m: gap_depth,
                        width_m: [narrowest_m, widest_m],
                        ..Gaps::default()
                    },
                },
            },
        };
        let config = Go2LearningConfig {
            floor_friction,
            floor,
            height_scan,
            random_starts,
            seed,
        };
        let threads = if threads == 0 {
            std::thread::available_parallelism().map_or(1, |n| n.get())
        } else {
            threads
        };
        let inner = Go2LearningBatch::new(config, num_envs, seed, threads)
            .map_err(|error| PyValueError::new_err(format!("Go2 learning batch: {error}")))?;
        Ok(Self { inner, config })
    }

    /// Returns the portable `TaskSpec` JSON of the batch's task.
    fn task_spec_json(&self) -> PyResult<String> {
        serde_json::to_string(&go2_learning_task_spec(&self.config)).map_err(|error| {
            PyRuntimeError::new_err(format!("failed to serialize the Go2 TaskSpec: {error}"))
        })
    }

    /// Resets every environment and returns the first observations.
    fn reset(&mut self) -> PyGo2LearningBatchStep {
        PyGo2LearningBatchStep {
            inner: self.inner.reset(),
        }
    }

    /// Applies one 12-element action per environment for one control
    /// period, resetting the environments whose episode ended.
    fn step(&mut self, actions: Vec<Vec<f64>>) -> PyResult<PyGo2LearningBatchStep> {
        if actions.len() != self.inner.num_envs() {
            return Err(PyValueError::new_err(format!(
                "expected {} actions, got {}",
                self.inner.num_envs(),
                actions.len()
            )));
        }
        let mut parsed = Vec::with_capacity(actions.len());
        for action in &actions {
            let action: [f64; 12] = action.as_slice().try_into().map_err(|_| {
                PyValueError::new_err(format!(
                    "each action must have 12 entries, got {}",
                    action.len()
                ))
            })?;
            if action.iter().any(|value| !value.is_finite()) {
                return Err(PyValueError::new_err("actions must be finite"));
            }
            parsed.push(action);
        }
        Ok(PyGo2LearningBatchStep {
            inner: self.inner.step(&parsed),
        })
    }

    /// Number of parallel environments.
    #[getter]
    fn num_envs(&self) -> usize {
        self.inner.num_envs()
    }

    /// The observation's width: 44, or 59 with the height scan.
    #[getter]
    fn observation_dim(&self) -> usize {
        self.config.observation_dim()
    }

    /// Control periods in an episode.
    #[getter]
    fn episode_steps(&self) -> usize {
        EPISODE_STEPS
    }

    /// Each environment's base position (x, y, z) in meters, Z-up
    /// (evaluation only).
    fn base_positions(&self) -> Vec<[f64; 3]> {
        (0..self.inner.num_envs())
            .map(|index| {
                let (position, _) = self.inner.episode(index).robot().base_pose_z_up();
                [position.x, position.y, position.z]
            })
            .collect()
    }

    /// Each environment's four foot sphere centers (x, y, z) in meters,
    /// Z-up, in front-left, front-right, rear-left, rear-right order
    /// (evaluation only).
    fn foot_positions(&self) -> PyResult<Vec<Vec<[f64; 3]>>> {
        (0..self.inner.num_envs())
            .map(|index| {
                let centers = self
                    .inner
                    .episode(index)
                    .robot()
                    .foot_centers_z_up_m()
                    .map_err(|error| PyValueError::new_err(format!("foot positions: {error}")))?;
                Ok(centers.iter().map(|c| [c.x, c.y, c.z]).collect())
            })
            .collect()
    }

    /// The gaps across the batch's floor, as (start, end) along x in
    /// meters (none on a flat floor or with `gap_depth=0`).
    fn gap_intervals(&self) -> Vec<[f64; 2]> {
        self.config.gaps()
    }
}

/// The `rne_py_locomotion` Python module.
#[pymodule]
fn rne_py_locomotion(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_class::<PyGo2LearningBatch>()?;
    m.add_class::<PyGo2LearningBatchStep>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const WIDTH_M: (f64, f64) = (0.25, 0.35);

    #[test]
    fn the_batch_steps_and_rejects_malformed_actions() {
        let mut batch = PyGo2LearningBatch::new(
            2,
            3,
            2,
            Some(7),
            true,
            true,
            1.0,
            24.0,
            0.1,
            0.04,
            0.5,
            0.0,
            WIDTH_M,
        )
        .unwrap();
        assert_eq!(batch.observation_dim(), 59);
        let reset = batch.reset();
        assert_eq!(reset.observations().len(), 2);
        assert!(reset.observations().iter().all(|o| o.len() == 59));
        let step = batch.step(vec![vec![0.0; 12]; 2]).unwrap();
        assert_eq!(step.rewards().len(), 2);
        assert_eq!(step.terminal_observations(), vec![None, None]);
        assert!(batch.step(vec![vec![0.0; 12]]).is_err());
        assert!(batch.step(vec![vec![0.0; 11]; 2]).is_err());
        assert!(batch.step(vec![vec![f64::NAN; 12]; 2]).is_err());
        let spec: serde_json::Value =
            serde_json::from_str(&batch.task_spec_json().unwrap()).unwrap();
        assert_eq!(spec["task_id"], "rne.unitree_go2.learning.v1");
        assert!(PyGo2LearningBatch::new(
            0, 3, 1, None, false, false, 1.0, 24.0, 0.1, 0.04, 0.5, 0.0, WIDTH_M
        )
        .is_err());
        // Random starts need a terrain.
        assert!(PyGo2LearningBatch::new(
            1, 3, 1, None, false, true, 1.0, 24.0, 0.1, 0.04, 0.5, 0.0, WIDTH_M
        )
        .is_err());
    }

    #[test]
    fn the_batch_lays_gaps_of_the_widths_asked_for() {
        let narrow = (0.05, 0.1);
        let batch = PyGo2LearningBatch::new(
            1,
            3,
            1,
            Some(7),
            true,
            true,
            1.0,
            24.0,
            0.05,
            0.0,
            0.5,
            0.3,
            narrow,
        )
        .unwrap();
        let gaps = batch.gap_intervals();
        assert!(gaps.len() > 5);
        let feet = batch.foot_positions().unwrap();
        assert_eq!(feet.len(), 1);
        assert_eq!(feet[0].len(), 4);
        assert!(
            gaps.iter()
                .all(|[start, end]| end - start >= narrow.0 - 1e-12
                    && end - start <= narrow.1 + 1e-12)
        );
        let flat = PyGo2LearningBatch::new(
            1, 3, 1, None, false, false, 1.0, 24.0, 0.1, 0.04, 0.5, 0.0, WIDTH_M,
        )
        .unwrap();
        assert!(flat.gap_intervals().is_empty());
        for bad in [(0.0, 0.1), (0.2, 0.1), (f64::NAN, 0.1)] {
            assert!(PyGo2LearningBatch::new(
                1,
                3,
                1,
                Some(7),
                false,
                true,
                1.0,
                24.0,
                0.05,
                0.0,
                0.5,
                0.3,
                bad
            )
            .is_err());
        }
    }
}
