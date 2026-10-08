//! Runs a batch of Go2 terrain-locomotion episodes in parallel, the way
//! raisimGym batches RaiSim environments for reinforcement learning.
//!
//! Each [`rne_locomotion::QuadrupedTerrainEpisode`] owns its own native
//! hard-contact simulation, draws a new fractal terrain, heading, and
//! velocity command at every reset, and takes a residual joint action on top
//! of the feedback trot. [`rne_ai::VectorizedEpisode::step_parallel`] steps
//! the batch on every available core. The example rolls out two policies — the
//! zero residual (the trot alone) and seeded random residuals — reports their
//! returns and the simulation throughput, and checks that the parallel batch
//! reproduces the serial one bit for bit.
//!
//! Run with `cargo run --release -p go2_parallel_rl --example 136_go2_parallel_rl`.

use rne_ai::{VectorizedEpisode, VectorizedEpisodeConfig, VectorizedEpisodeStep};
use rne_locomotion::{
    quadruped_terrain_task_spec, Quadruped, QuadrupedJointAction, QuadrupedObservation,
    QuadrupedTerrainConfig, QuadrupedTerrainEpisode, QUADRUPED_ACTION_DIM,
    QUADRUPED_OBSERVATION_DIM,
};
use std::sync::Arc;
use std::time::Instant;

const NUM_ENVS: usize = 32;
const SEED: u64 = 2026;
const CONTROL_STEPS: usize = 300;
/// Amplitude of the random residual policy, as a fraction of the action scale.
const RANDOM_AMPLITUDE: f64 = 0.6;

#[derive(Debug, Default)]
struct Rollout {
    mean_reward: f64,
    falls: usize,
    episodes_finished: usize,
    digest: u64,
    seconds: f64,
}

fn batch(
    go2: &Arc<Quadruped>,
    config: QuadrupedTerrainConfig,
) -> VectorizedEpisode<QuadrupedTerrainEpisode> {
    VectorizedEpisode::from_seeded(
        VectorizedEpisodeConfig {
            num_envs: NUM_ENVS,
            seed: SEED,
            // One episode per environment: with automatic resets the batch
            // would report the reset instead of the terminal transition.
            auto_reset: false,
        },
        |seed| QuadrupedTerrainEpisode::new(go2.clone(), config, seed).expect("episode"),
    )
}

/// SplitMix64 for the random policy.
fn next_unit(state: &mut u64) -> f64 {
    *state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
}

fn rollout(
    go2: &Arc<Quadruped>,
    config: QuadrupedTerrainConfig,
    threads: usize,
    random: bool,
) -> Rollout {
    let mut envs = batch(go2, config);
    let mut policy_rng = SEED ^ 0x5eed;
    let start = Instant::now();
    let first: VectorizedEpisodeStep<QuadrupedObservation> = envs.reset_parallel(threads);
    assert!(first
        .observations
        .iter()
        .all(|observation| observation.to_vec().len() == QUADRUPED_OBSERVATION_DIM));
    let mut rollout = Rollout::default();
    let mut total_reward = 0.0;
    let mut fell = [false; NUM_ENVS];
    let mut finished = [false; NUM_ENVS];
    for _ in 0..CONTROL_STEPS {
        let actions: Vec<QuadrupedJointAction> = (0..NUM_ENVS)
            .map(|_| {
                let mut action = QuadrupedJointAction::default();
                if random {
                    for value in &mut action.joint_action {
                        *value = RANDOM_AMPLITUDE * (2.0 * next_unit(&mut policy_rng) - 1.0);
                    }
                }
                action
            })
            .collect();
        let step = envs.step_parallel(&actions, threads);
        total_reward += step.rewards.iter().sum::<f64>();
        for env in 0..NUM_ENVS {
            fell[env] |= step.terminated[env];
            finished[env] |= step.terminated[env] || step.truncated[env];
        }
    }
    rollout.falls = fell.iter().filter(|fell| **fell).count();
    rollout.episodes_finished = finished.iter().filter(|done| **done).count();
    rollout.seconds = start.elapsed().as_secs_f64();
    rollout.mean_reward = total_reward / (NUM_ENVS * CONTROL_STEPS) as f64;
    rollout.digest = envs.replay_digest();
    rollout
}

fn main() {
    let threads = std::thread::available_parallelism()
        .map(|threads| threads.get())
        .unwrap_or(1);
    let config = QuadrupedTerrainConfig {
        max_steps: CONTROL_STEPS as u64,
        ..QuadrupedTerrainConfig::default()
    };
    let task = quadruped_terrain_task_spec(&config);
    println!(
        "task {}: {} observations, {} actions, control step {:.3} s ({} simulation steps)",
        task.task_id,
        QUADRUPED_OBSERVATION_DIM,
        QUADRUPED_ACTION_DIM,
        task.control_step_s,
        config.control_decimation
    );
    let go2 = Arc::new(Quadruped::go2().expect("Go2"));

    let serial = rollout(&go2, config, 1, false);
    let parallel = rollout(&go2, config, threads, false);
    let random = rollout(&go2, config, threads, true);
    let env_steps = (NUM_ENVS * CONTROL_STEPS) as f64;
    let sim_steps = env_steps * f64::from(config.control_decimation);
    for (name, run) in [
        ("trot (zero residual), 1 thread", &serial),
        (
            &format!("trot (zero residual), {threads} threads") as &str,
            &parallel,
        ),
        (&format!("random residual, {threads} threads"), &random),
    ] {
        println!(
            "{name}: mean reward {:.3}/step, {} of {NUM_ENVS} fell, {} finished, {:.0} env steps/s ({:.0} sim steps/s)",
            run.mean_reward,
            run.falls,
            run.episodes_finished,
            env_steps / run.seconds,
            sim_steps / run.seconds
        );
    }
    let identical = serial.digest == parallel.digest && serial.mean_reward == parallel.mean_reward;
    println!(
        "parallel speedup {:.1}x on {threads} threads; parallel replay identical to serial: {identical}",
        serial.seconds / parallel.seconds
    );

    let ok = identical
        && serial.falls == 0
        && serial.mean_reward > 0.8
        && random.mean_reward < serial.mean_reward
        && serial.episodes_finished == NUM_ENVS;
    if !ok {
        eprintln!("go2 parallel rl: failed");
        std::process::exit(1);
    }
    println!("go2 parallel rl: ok");
}
