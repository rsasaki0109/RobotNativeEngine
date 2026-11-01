//! Physics walking with unitree_rl_gym's pretrained G1 policy, in the browser.
//!
//! Unlike the kinematic pose playground, this mode simulates the 12-DoF G1
//! with RNE's Rapier backend and drives its legs with the pretrained LSTM
//! policy through the same 500 Hz PD / 50 Hz policy loop as example 144. The
//! scene, URDF and weights are embedded, so nothing is fetched at runtime.
//! The caller supplies elapsed wall time; this module paces whole policy
//! periods against it and never reads a clock itself.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use rne_ai::{LstmPolicy, PolicyWalkPlant, UnitreeRlGymWalker, UrdfSceneSim};
use rne_assets::{parse_scene_bundle_with_sources, UrdfSourceMap};
use rne_core::SimDuration;
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_render::{load_stl_bytes, PbrMaterial, RenderScene, TriangleMesh, VisualShape};
use rne_robot::Link;
use rne_urdf_import::{parse_urdf, rpy_to_quat, UrdfGeometry};
use rne_world::{world_transform_of, Transform3};

const SCENE: &str = include_str!("../../../assets/scenes/unitree_g1_12dof_walk.rne.scene.toml");
const ROBOT: &str = include_str!("../../../assets/robots/unitree_g1_12dof_walk.rne.robot.toml");
const URDF: &str = include_str!("../../../assets/robots/g1_description/g1_12dof.rne.urdf");
const POLICY_MANIFEST: &str =
    include_str!("../../../assets/policies/unitree_g1_walk/g1_walk_lstm.policy.json");
const POLICY_WEIGHTS: &[u8] =
    include_bytes!("../../../assets/policies/unitree_g1_walk/g1_walk_lstm.f32");

/// Constraint-solver iterations: the native timing sweep walks at every
/// setting from 1 to 32, and 4 keeps the most headroom for rendering.
const SOLVER_ITERATIONS: usize = 4;
/// Most policy periods simulated per animation frame (0.12 s). A slower
/// device drops the rest instead of falling ever further behind.
const MAX_PERIODS_PER_FRAME: usize = 6;
/// Longest wall-clock gap credited to one frame, so a background tab does not
/// return to a burst of catch-up steps.
const MAX_FRAME_GAP_S: f64 = 0.25;
/// Pelvis height below which the robot counts as fallen, in metres.
const FALL_HEIGHT_M: f64 = 0.45;
/// Command limits inside unitree_rl_gym's ±1 training range.
const FORWARD_M_S: f64 = 0.8;
const BACKWARD_M_S: f64 = 0.4;
const YAW_RAD_S: f64 = 0.8;
/// Command slew limits, so key presses ramp instead of stepping.
const LINEAR_SLEW_M_S2: f64 = 1.5;
const YAW_SLEW_RAD_S2: f64 = 3.0;
/// Horizontal push applied while the push button pulse lasts, in newtons.
const PUSH_N: f64 = 120.0;
const PUSH_S: f64 = 0.1;

struct Part {
    link: Entity,
    offset: Transform3,
    shape: VisualShape,
    color: [f32; 4],
    material: PbrMaterial,
    mesh: Arc<TriangleMesh>,
}

/// Pace and cost measured over the last completed second of wall time.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WalkPace {
    /// Simulated seconds per wall second.
    pub realtime_factor: f64,
    /// Compute seconds spent stepping per simulated second (below 1 keeps up).
    pub compute_per_sim_s: f64,
}

/// G1 walking under physics and the pretrained policy.
pub struct PhysicsWalk {
    walker: UnitreeRlGymWalker,
    parts: Vec<Part>,
    laboratory: RenderScene,
    command: [f64; 3],
    accumulator_s: f64,
    push_remaining_s: f64,
    fallen: bool,
    window_wall_s: f64,
    window_sim_s: f64,
    window_compute_s: f64,
    pace: WalkPace,
}

fn virtual_path(relative: &str) -> PathBuf {
    Path::new("/embedded/assets").join(relative)
}

fn slew(current: f64, target: f64, limit_per_s: f64, dt_s: f64) -> f64 {
    current + (target - current).clamp(-limit_per_s * dt_s, limit_per_s * dt_s)
}

impl PhysicsWalk {
    /// Builds the embedded scene, plant and policy.
    ///
    /// `shared_mesh` returns meshes another view has already loaded, so the
    /// renderer reuses their GPU buffers; missing ones are decoded here.
    pub fn new(shared_mesh: &dyn Fn(&str) -> Option<Arc<TriangleMesh>>) -> Result<Self, String> {
        let scene_path = virtual_path("scenes/unitree_g1_12dof_walk.rne.scene.toml");
        let robot_path = virtual_path("scenes/../robots/unitree_g1_12dof_walk.rne.robot.toml");
        let urdf_path = virtual_path("scenes/../robots/g1_description/g1_12dof.rne.urdf");
        let mut sources = UrdfSourceMap::new();
        sources.insert(urdf_path, URDF);
        sources.insert(robot_path.clone(), ROBOT);
        let bundle = parse_scene_bundle_with_sources(
            &scene_path,
            SCENE,
            &[(robot_path, ROBOT)],
            Some(&sources),
        )
        .map_err(|error| error.to_string())?;
        let sim = UrdfSceneSim::from_scene_bundle(
            &scene_path,
            &bundle,
            &sources,
            SOLVER_ITERATIONS,
            SimDuration::from_ticks(20_000_000),
        )
        .map_err(|error| error.to_string())?;
        let policy = LstmPolicy::from_manifest(POLICY_MANIFEST, POLICY_WEIGHTS)
            .map_err(|error| error.to_string())?;
        let walker = UnitreeRlGymWalker::new(sim, policy, PolicyWalkPlant::default())
            .map_err(|error| error.to_string())?;

        let urdf = parse_urdf(URDF).map_err(|error| error.to_string())?;
        let world = walker.sim().world();
        let mut parts = Vec::new();
        for link in &urdf.links {
            let entity = world
                .iter_entities()
                .find(|entity| entity.get::<Link>().is_some_and(|l| l.name == link.name))
                .map(|entity| entity.id())
                .ok_or_else(|| format!("simulated scene has no link {}", link.name))?;
            for visual in &link.visuals {
                let UrdfGeometry::Mesh { path, scale } = &visual.geometry else {
                    continue;
                };
                let mesh = match shared_mesh(path) {
                    Some(mesh) => mesh,
                    None => Arc::new(
                        load_stl_bytes(path, crate::humanoid::mesh_bytes(path)?)
                            .map_err(|error| error.to_string())?,
                    ),
                };
                let color = visual.material_rgba.unwrap_or([0.7, 0.7, 0.75, 1.0]);
                parts.push(Part {
                    link: entity,
                    offset: Transform3::from_translation_rotation(
                        visual.origin_xyz,
                        rpy_to_quat(visual.origin_rpy),
                    ),
                    shape: VisualShape::Mesh {
                        path: path.clone(),
                        scale: *scale,
                    },
                    color,
                    material: crate::humanoid::shell_material(color),
                    mesh,
                });
            }
        }
        Ok(Self {
            walker,
            parts,
            laboratory: crate::lab::scene(),
            command: [0.0; 3],
            accumulator_s: 0.0,
            push_remaining_s: 0.0,
            fallen: false,
            window_wall_s: 0.0,
            window_sim_s: 0.0,
            window_compute_s: 0.0,
            pace: WalkPace::default(),
        })
    }

    /// Starts a sideways push on the pelvis (+1 left, −1 right of its heading).
    pub fn push(&mut self, side: f64) {
        let (_, rotation) = self.walker.base_pose_z_up();
        let left_z_up = rotation * Vec3::Y;
        // Policy Z-up (x, y, z) is RNE Y-up (x, z, -y).
        let left = Vec3::new(left_z_up.x, 0.0, -left_z_up.y).normalize_or_zero();
        self.walker
            .set_base_push_n(Some(left * (PUSH_N * side.signum())));
        self.push_remaining_s = PUSH_S;
    }

    /// Advances by whole policy periods to keep pace with `wall_dt_s`.
    ///
    /// `forward` and `turn` are the drive axes in [−1, 1]. `elapsed_s` reads a
    /// monotonic clock and is used only to measure compute cost. Returns the
    /// number of policy periods simulated.
    pub fn advance(
        &mut self,
        wall_dt_s: f64,
        forward: f64,
        turn: f64,
        elapsed_s: &mut dyn FnMut() -> f64,
    ) -> Result<usize, String> {
        if !wall_dt_s.is_finite() || wall_dt_s < 0.0 {
            return Ok(0);
        }
        let wall_dt_s = wall_dt_s.min(MAX_FRAME_GAP_S);
        let period_s = self.walker.policy().deployment().control_dt_s();
        if !self.fallen {
            self.accumulator_s += wall_dt_s;
        }
        let forward = forward.clamp(-1.0, 1.0);
        let target = [
            forward
                * if forward >= 0.0 {
                    FORWARD_M_S
                } else {
                    BACKWARD_M_S
                },
            0.0,
            turn.clamp(-1.0, 1.0) * YAW_RAD_S,
        ];
        let start_s = elapsed_s();
        let mut periods = 0;
        while self.accumulator_s >= period_s && periods < MAX_PERIODS_PER_FRAME {
            self.command[0] = slew(self.command[0], target[0], LINEAR_SLEW_M_S2, period_s);
            self.command[1] = slew(self.command[1], target[1], LINEAR_SLEW_M_S2, period_s);
            self.command[2] = slew(self.command[2], target[2], YAW_SLEW_RAD_S2, period_s);
            let step = self
                .walker
                .step(self.command)
                .map_err(|error| error.to_string())?;
            self.accumulator_s -= period_s;
            periods += 1;
            if self.push_remaining_s > 0.0 {
                self.push_remaining_s -= period_s;
                if self.push_remaining_s <= 0.0 {
                    self.walker.set_base_push_n(None);
                }
            }
            if step.base_position_m[2] < FALL_HEIGHT_M {
                self.fallen = true;
                self.accumulator_s = 0.0;
                break;
            }
        }
        if periods == MAX_PERIODS_PER_FRAME {
            // This device cannot keep up: drop the backlog rather than spiral.
            self.accumulator_s = self.accumulator_s.min(period_s);
        }
        self.window_compute_s += (elapsed_s() - start_s).max(0.0);
        self.window_wall_s += wall_dt_s;
        self.window_sim_s += periods as f64 * period_s;
        if self.window_wall_s >= 1.0 {
            self.pace = WalkPace {
                realtime_factor: self.window_sim_s / self.window_wall_s,
                compute_per_sim_s: if self.window_sim_s > 0.0 {
                    self.window_compute_s / self.window_sim_s
                } else {
                    0.0
                },
            };
            self.window_wall_s = 0.0;
            self.window_sim_s = 0.0;
            self.window_compute_s = 0.0;
        }
        Ok(periods)
    }

    /// Whether the pelvis dropped below the fall height.
    pub fn fallen(&self) -> bool {
        self.fallen
    }

    /// Last measured pace.
    pub fn pace(&self) -> WalkPace {
        self.pace
    }

    /// Current (forward m/s, lateral m/s, yaw rad/s) command after slewing.
    pub fn command(&self) -> [f64; 3] {
        self.command
    }

    /// Simulated time in seconds.
    pub fn time_s(&self) -> f64 {
        self.walker.sim().sim_time().as_seconds().value()
    }

    /// Camera focus at the pelvis in RNE's Y-up world.
    pub fn focus(&self) -> Vec3 {
        let (position, _) = self.walker.base_pose_z_up();
        Vec3::new(position.x, position.z + 0.15, -position.y)
    }

    /// Laboratory plus the simulated robot.
    pub fn render(&self) -> RenderScene {
        let mut scene = self.laboratory.clone();
        let world = self.walker.sim().world();
        for part in &self.parts {
            let mut item = RenderScene::item_from_visual(
                world_transform_of(world, part.link),
                part.shape.clone(),
                part.color,
                part.offset,
            );
            item.mesh = Some(Arc::clone(&part.mesh));
            item.material = part.material.clone();
            scene.items.push(item);
        }
        scene
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_g1_walks_forward_under_physics() {
        let mut walk = PhysicsWalk::new(&|_| None).expect("embedded walk scene");
        let mut clock = 0.0;
        let mut elapsed = || {
            clock += 1e-4;
            clock
        };
        let mut periods = 0;
        // Three seconds of 60 Hz frames holding W.
        for _ in 0..180 {
            periods += walk.advance(1.0 / 60.0, 1.0, 0.0, &mut elapsed).unwrap();
        }
        // 180 frames of 1/60 s are 3 s up to rounding of the accumulator.
        assert!((149..=150).contains(&periods), "{periods} periods");
        assert!(!walk.fallen());
        assert!((walk.time_s() - 0.02 * periods as f64).abs() < 1e-9);
        let focus = walk.focus();
        assert!(focus.x > 0.8, "walked to {focus:?}");
        assert!((walk.command()[0] - FORWARD_M_S).abs() < 1e-12);
        assert!(walk.pace().realtime_factor > 0.99);
        assert_eq!(
            walk.render().items.len() - crate::lab::scene().items.len(),
            walk.parts.len()
        );
    }

    #[test]
    fn a_long_frame_gap_is_capped_and_pushes_end() {
        let mut walk = PhysicsWalk::new(&|_| None).unwrap();
        let mut elapsed = || 0.0;
        assert_eq!(
            walk.advance(5.0, 0.0, 0.0, &mut elapsed).unwrap(),
            MAX_PERIODS_PER_FRAME
        );
        walk.push(1.0);
        for _ in 0..30 {
            walk.advance(1.0 / 60.0, 0.0, 0.0, &mut elapsed).unwrap();
        }
        assert!(walk.push_remaining_s <= 0.0);
        assert!(!walk.fallen(), "a 120 N, 0.1 s push should be recoverable");
    }
}
