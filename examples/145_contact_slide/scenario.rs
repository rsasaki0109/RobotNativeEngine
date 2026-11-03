//! The block, the scenarios and one RNE run of a scenario.

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Seconds, Vec3};
use rne_physics::{
    Collider, ColliderPart, ColliderShape, CompoundCollider, ExternalBodyWrench, PhysicsBackend,
    PhysicsMaterial, PhysicsWorldDesc, PhysicsWorldId, RigidBody, RigidBodyInertia, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use serde_json::{json, Value};

use crate::Result;

/// Block mass in kilograms.
pub(crate) const MASS_KG: f64 = 10.0;
/// Principal moments of inertia about x, y and z (Z-up), those of a
/// 0.24 × 0.12 × 0.08 m solid box.
pub(crate) const INERTIA_KG_M2: [f64; 3] = [
    MASS_KG / 12.0 * (0.12 * 0.12 + 0.08 * 0.08),
    MASS_KG / 12.0 * (0.24 * 0.24 + 0.08 * 0.08),
    MASS_KG / 12.0 * (0.24 * 0.24 + 0.12 * 0.12),
];
/// Support sphere radius, as the G1's foot spheres.
pub(crate) const SPHERE_RADIUS_M: f64 = 0.005;
/// Support sphere centres relative to the centre of mass (Z-up): ±x, ±y, −z.
pub(crate) const SPHERE_OFFSET_M: [f64; 3] = [0.1, 0.05, 0.04];
/// Physics step, as the walking loop's.
pub(crate) const DT_S: f64 = 0.002;
/// Standard gravity.
pub(crate) const G_M_S2: f64 = 9.81;
/// Seconds a resting block settles before it is pushed.
pub(crate) const SETTLE_S: f64 = 0.5;
/// Seconds a push lasts.
pub(crate) const PUSH_S: f64 = 0.1;
/// Seconds simulated after the push, or after the drop starts.
pub(crate) const OBSERVE_S: f64 = 1.0;
/// Physics steps between samples in the report (10 ms).
const SAMPLE_EVERY: usize = 5;

/// How a scenario starts.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Start {
    /// Resting on its spheres, then pushed.
    Rest,
    /// Released with its lowest sphere `drop_m` above the floor, moving
    /// forward and pitched.
    Drop {
        drop_m: f64,
        forward_m_s: f64,
        pitch_rad: f64,
    },
}

/// One scenario.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Scenario {
    pub(crate) name: &'static str,
    pub(crate) what: &'static str,
    /// Push along +x, held for [`PUSH_S`].
    pub(crate) force_n: f64,
    /// Height of the push point above the centre of mass, fixed in the block.
    pub(crate) force_height_m: f64,
    /// Floor and sphere friction coefficient.
    pub(crate) friction: f64,
    pub(crate) start: Start,
}

const WEIGHT_N: f64 = MASS_KG * G_M_S2;

/// The scenarios, in report order.
pub(crate) const SCENARIOS: [Scenario; 5] = [
    Scenario {
        name: "stick",
        what: "push of half the friction limit at the centre of mass",
        force_n: 0.5 * WEIGHT_N,
        force_height_m: 0.0,
        friction: 1.0,
        start: Start::Rest,
    },
    Scenario {
        name: "slide",
        what: "push of twice the friction limit at the centre of mass",
        force_n: 2.0 * WEIGHT_N,
        force_height_m: 0.0,
        friction: 1.0,
        start: Start::Rest,
    },
    Scenario {
        name: "slide_mu_0.5",
        what: "the same on friction 0.5",
        force_n: WEIGHT_N,
        force_height_m: 0.0,
        friction: 0.5,
        start: Start::Rest,
    },
    Scenario {
        name: "rock",
        what: "push of 0.9 times the weight 0.1 m above the centre of mass: it rocks onto its front spheres",
        force_n: 0.9 * WEIGHT_N,
        force_height_m: 0.1,
        friction: 1.0,
        start: Start::Rest,
    },
    Scenario {
        name: "landing",
        what: "dropped 2 cm, heel first (pitched 0.2 rad nose up), moving at 1 m/s",
        force_n: 0.0,
        force_height_m: 0.0,
        friction: 1.0,
        start: Start::Drop {
            drop_m: 0.02,
            forward_m_s: 1.0,
            pitch_rad: -0.2,
        },
    },
];

impl Scenario {
    pub(crate) fn pushed(&self) -> bool {
        matches!(self.start, Start::Rest)
    }

    /// Simulated time at which the push starts and the record ends.
    fn times_s(&self) -> (f64, f64) {
        match self.start {
            Start::Rest => (SETTLE_S, SETTLE_S + PUSH_S + OBSERVE_S),
            Start::Drop { .. } => (0.0, OBSERVE_S),
        }
    }

    pub(crate) fn to_json(self) -> Value {
        let (push_start_s, end_s) = self.times_s();
        let start = match self.start {
            Start::Rest => json!({"kind": "rest"}),
            Start::Drop {
                drop_m,
                forward_m_s,
                pitch_rad,
            } => json!({"kind": "drop", "drop_m": drop_m, "forward_m_s": forward_m_s,
                         "pitch_rad": pitch_rad}),
        };
        json!({
            "name": self.name, "what": self.what, "force_n": self.force_n,
            "force_height_m": self.force_height_m, "friction": self.friction, "start": start,
            "push_start_s": if self.pushed() { Some(push_start_s) } else { None },
            "push_duration_s": if self.pushed() { PUSH_S } else { 0.0 },
            "end_s": end_s, "dt_s": DT_S,
        })
    }
}

/// The block's definition for the report (Z-up).
pub(crate) fn block_json() -> Value {
    let [x, y, z] = SPHERE_OFFSET_M;
    json!({
        "mass_kg": MASS_KG,
        "principal_inertia_kg_m2": INERTIA_KG_M2,
        "sphere_radius_m": SPHERE_RADIUS_M,
        "sphere_centres_m": [[x, y, -z], [x, -y, -z], [-x, y, -z], [-x, -y, -z]],
        "gravity_m_s2": G_M_S2,
    })
}

/// RNE solver settings.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Settings {
    /// Rapier solver iterations; 0 keeps the walking examples' 4.
    pub(crate) solver_iterations: usize,
    /// Contact natural frequency (Hz) and damping ratio, or Rapier's.
    pub(crate) contact_softness: Option<(f64, f64)>,
}

impl Settings {
    fn iterations(self) -> usize {
        if self.solver_iterations == 0 {
            4
        } else {
            self.solver_iterations
        }
    }

    pub(crate) fn to_json(self) -> Value {
        json!({
            "solver_iterations": self.iterations(),
            "contact_softness": self.contact_softness.map(|(hz, ratio)| json!({
                "natural_frequency_hz": hz, "damping_ratio": ratio,
            })),
        })
    }
}

/// A run's samples and summary.
pub(crate) struct RunResult {
    pub(crate) samples: Value,
    pub(crate) summary: Value,
}

/// Z-up (x, y, z) from RNE's Y-up.
fn zup(v: Vec3) -> [f64; 3] {
    [v.x, -v.z, v.y]
}

/// RNE's Y-up from Z-up.
fn yup(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, z, -y)
}

/// Rotation about Z-up +y (positive nose down), as an RNE rotation.
fn pitch_rotation(pitch_rad: f64) -> Quat {
    Quat::from_axis_angle(yup(0.0, 1.0, 0.0), pitch_rad)
}

/// Pitch about Z-up +y of an RNE rotation: the angle by which the block's
/// forward axis dips below the horizontal.
fn pitch_of(rotation: Quat) -> f64 {
    let [x, _, z] = zup(rotation * Vec3::X);
    (-z).atan2(x)
}

fn spawn(world: &mut World, scenario: &Scenario) -> (Entity, Entity) {
    let material = PhysicsMaterial {
        friction: scenario.friction as f32,
        restitution: 0.0,
    };
    let ground = spawn_named(world, "ground");
    world.entity_mut(ground).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: Vec3::new(20.0, 0.5, 20.0),
            },
            material,
            ..Collider::default()
        },
        Transform3::from_translation_rotation(Vec3::new(0.0, -0.5, 0.0), Quat::IDENTITY),
    ));
    let [ox, oy, oz] = SPHERE_OFFSET_M;
    let sphere = ColliderShape::Sphere {
        radius_m: SPHERE_RADIUS_M,
    };
    let parts = [(ox, oy), (ox, -oy), (-ox, oy), (-ox, -oy)]
        .map(|(x, y)| ColliderPart {
            shape: sphere.clone(),
            local_offset: Transform3::from_translation_rotation(yup(x, y, -oz), Quat::IDENTITY),
        })
        .to_vec();
    let rest_height_m = oz + SPHERE_RADIUS_M;
    let (translation, rotation, velocity) = match scenario.start {
        Start::Rest => (yup(0.0, 0.0, rest_height_m), Quat::IDENTITY, Vec3::ZERO),
        Start::Drop {
            drop_m,
            forward_m_s,
            pitch_rad,
        } => {
            // Lowest sphere bottom `drop_m` above the floor.
            let rotation = pitch_rotation(pitch_rad);
            let lowest = parts
                .iter()
                .map(|part| (rotation * part.local_offset.translation).y)
                .fold(f64::INFINITY, f64::min);
            (
                yup(0.0, 0.0, drop_m + SPHERE_RADIUS_M - lowest),
                rotation,
                yup(forward_m_s, 0.0, 0.0),
            )
        }
    };
    let [ixx, iyy, izz] = INERTIA_KG_M2;
    let block = spawn_named(world, "block");
    world.entity_mut(block).insert((
        RigidBody {
            mass_kg: MASS_KG,
            linear_velocity_m_s: velocity,
            ..RigidBody::default()
        },
        // Z-up x, y, z are RNE x, -z, y: the diagonal permutes.
        RigidBodyInertia {
            center_of_mass_local_m: Vec3::ZERO,
            ixx_kg_m2: ixx,
            ixy_kg_m2: 0.0,
            ixz_kg_m2: 0.0,
            iyy_kg_m2: izz,
            iyz_kg_m2: 0.0,
            izz_kg_m2: iyy,
        },
        Collider {
            shape: sphere,
            material,
            ..Collider::default()
        },
        CompoundCollider { parts },
        Transform3::from_translation_rotation(translation, rotation),
    ));
    (ground, block)
}

struct Sample {
    time_s: f64,
    position_m: [f64; 3],
    pitch_rad: f64,
    forward_m_s: f64,
    front_n: f64,
    back_n: f64,
}

/// Normal force on the front (x ahead of the centre of mass) and back spheres.
fn normal_loads(
    backend: &RapierBackend,
    id: PhysicsWorldId,
    ground: Entity,
    centre_x: f64,
) -> Result<(f64, f64)> {
    let (mut front, mut back) = (0.0, 0.0);
    for point in backend.contact_points(id)? {
        if point.entity_a != ground && point.entity_b != ground {
            continue;
        }
        if point.point_world_m.x > centre_x {
            front += point.normal_force_n;
        } else {
            back += point.normal_force_n;
        }
    }
    Ok((front, back))
}

/// Runs `scenario` in RNE.
pub(crate) fn run(scenario: Scenario, settings: Settings) -> Result<RunResult> {
    let mut backend = RapierBackend::new();
    let id = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: Vec3::new(0.0, -G_M_S2, 0.0),
        solver_iterations: settings.iterations(),
    })?;
    if let Some((hz, ratio)) = settings.contact_softness {
        if !backend.set_contact_softness(id, hz, ratio) {
            return Err("contact softness needs a positive frequency and damping ratio".into());
        }
    }
    let mut world = World::new();
    let (ground, block) = spawn(&mut world, &scenario);
    let (push_start_s, end_s) = scenario.times_s();
    let steps = (end_s / DT_S).round() as usize;
    let push_steps = {
        let first = (push_start_s / DT_S).round() as usize;
        first..first + (PUSH_S / DT_S).round() as usize
    };
    let dt = SimDuration::from_seconds(Seconds::new(DT_S));
    let mut steps_record = Vec::with_capacity(steps);
    for step in 0..steps {
        backend.sync_from_ecs(&mut world, id)?;
        if scenario.pushed() && push_steps.contains(&step) {
            let pose = *world.get::<Transform3>(block).ok_or("block has no pose")?;
            let point = pose.translation + pose.rotation * yup(0.0, 0.0, scenario.force_height_m);
            backend.apply_external_body_wrench(
                id,
                ExternalBodyWrench {
                    entity: block,
                    point_world_m: point,
                    force_world_n: yup(scenario.force_n, 0.0, 0.0),
                    torque_world_nm: Vec3::ZERO,
                },
            )?;
        }
        backend.step(id, dt)?;
        backend.sync_to_ecs(&mut world, id)?;
        let pose = *world.get::<Transform3>(block).ok_or("block has no pose")?;
        let body = world.get::<RigidBody>(block).ok_or("block has no body")?;
        let (front_n, back_n) = normal_loads(&backend, id, ground, pose.translation.x)?;
        steps_record.push(Sample {
            time_s: (step + 1) as f64 * DT_S,
            position_m: zup(pose.translation),
            pitch_rad: pitch_of(pose.rotation),
            forward_m_s: body.linear_velocity_m_s.x,
            front_n,
            back_n,
        });
    }
    let summary = summarize(&scenario, &steps_record);
    let samples = steps_record
        .iter()
        .skip(SAMPLE_EVERY - 1)
        .step_by(SAMPLE_EVERY)
        .map(|s| {
            json!({"time_s": s.time_s, "position_m": s.position_m, "pitch_rad": s.pitch_rad,
                   "forward_m_s": s.forward_m_s, "front_normal_n": s.front_n,
                   "back_normal_n": s.back_n})
        })
        .collect();
    Ok(RunResult { samples, summary })
}

/// Seconds after a push starts, or after a block that pitched first lies flat
/// again, over which contact loss and hops are counted.
const WINDOW_S: f64 = 0.3;

/// The summary both engines' reports carry, from every physics step
/// (`mujoco_contact_slide.py` computes the same).
fn summarize(scenario: &Scenario, steps: &[Sample]) -> Value {
    let (push_start_s, _) = scenario.times_s();
    let push_end_s = push_start_s + if scenario.pushed() { PUSH_S } else { 0.0 };
    let at = |time_s: f64| {
        steps
            .iter()
            .min_by(|a, b| {
                (a.time_s - time_s)
                    .abs()
                    .total_cmp(&(b.time_s - time_s).abs())
            })
            .expect("steps")
    };
    let last = steps.last().expect("steps");
    let after_push = || steps.iter().filter(move |s| s.time_s > push_end_s + 1e-9);
    let stop_s = after_push()
        .find(|s| s.forward_m_s.abs() < 0.01)
        .map(|s| s.time_s - push_end_s);
    // Loads over the push, after its first 20 ms, averaged over steps.
    let during: Vec<&Sample> = steps
        .iter()
        .filter(|s| s.time_s > push_start_s + 0.02 + 1e-9 && s.time_s <= push_end_s + 1e-9)
        .collect();
    let mean = |f: fn(&Sample) -> f64| {
        (!during.is_empty()).then(|| during.iter().map(|s| f(s)).sum::<f64>() / during.len() as f64)
    };
    // When a block that pitched visibly first lies flat again.
    let peak = steps
        .iter()
        .max_by(|a, b| a.pitch_rad.abs().total_cmp(&b.pitch_rad.abs()))
        .expect("steps");
    let flat_s = (peak.pitch_rad.abs() > 0.05)
        .then(|| {
            steps
                .iter()
                .find(|s| s.time_s > peak.time_s && s.pitch_rad.abs() < 0.01)
                .map(|s| s.time_s)
        })
        .flatten();
    // A block that pitched is watched from when it lies flat again.
    let window_start_s = if peak.pitch_rad.abs() > 0.05 {
        flat_s
    } else {
        Some(push_start_s)
    };
    let window: Vec<&Sample> = window_start_s.map_or(Vec::new(), |start| {
        steps
            .iter()
            .filter(|s| s.time_s > start + 1e-9 && s.time_s <= start + WINDOW_S + 1e-9)
            .collect()
    });
    let airborne = (!window.is_empty()).then(|| {
        window
            .iter()
            .filter(|s| s.front_n + s.back_n == 0.0)
            .count() as f64
            / window.len() as f64
    });
    // The largest rise of the centre of mass above its lowest point so far.
    let hop = (!window.is_empty()).then(|| {
        let mut lowest = f64::INFINITY;
        window.iter().fold(0.0, |hop: f64, s| {
            lowest = lowest.min(s.position_m[2]);
            hop.max(s.position_m[2] - lowest)
        })
    });
    let pitch = steps.iter().map(|s| s.pitch_rad);
    json!({
        // Dropped blocks start at x = 0.
        "slide_m": last.position_m[0]
            - if scenario.pushed() { at(push_start_s).position_m[0] } else { 0.0 },
        "speed_after_push_m_s": at(push_end_s).forward_m_s,
        "stop_after_s": stop_s,
        "max_pitch_rad": pitch.clone().fold(f64::NEG_INFINITY, f64::max),
        "min_pitch_rad": pitch.fold(f64::INFINITY, f64::min),
        "final_pitch_rad": last.pitch_rad,
        "flat_at_s": flat_s,
        "push_front_normal_n": mean(|s| s.front_n),
        "push_back_normal_n": mean(|s| s.back_n),
        "airborne_share": airborne,
        "hop_m": hop,
    })
}

/// One line of the console summary.
pub(crate) fn summary_line(scenario: Scenario, result: &RunResult) -> String {
    let s = &result.summary;
    let number = |key: &str| {
        s[key]
            .as_f64()
            .map_or("—".to_string(), |v| format!("{v:.4}"))
    };
    format!(
        "{:13} slide {} m, speed after push {} m/s, stop after {} s, pitch {}..{} rad (final {}), \
         flat at {} s, push loads front {} N / back {} N, airborne {}, hop {} m",
        scenario.name,
        number("slide_m"),
        number("speed_after_push_m_s"),
        number("stop_after_s"),
        number("min_pitch_rad"),
        number("max_pitch_rad"),
        number("final_pitch_rad"),
        number("flat_at_s"),
        number("push_front_normal_n"),
        number("push_back_normal_n"),
        number("airborne_share"),
        number("hop_m"),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn summary(name: &str, settings: Settings) -> Value {
        let scenario = SCENARIOS.iter().find(|s| s.name == name).unwrap();
        run(*scenario, settings).unwrap().summary
    }

    fn close(value: &Value, expected: f64, tolerance: f64) -> bool {
        (value.as_f64().unwrap() - expected).abs() <= tolerance
    }

    #[test]
    fn pitch_and_frames_round_trip() {
        let rotation = pitch_rotation(0.3);
        assert!((pitch_of(rotation) - 0.3).abs() < 1e-12);
        let [x, y, z] = zup(yup(1.0, 2.0, 3.0));
        assert_eq!([x, y, z], [1.0, 2.0, 3.0]);
        // Positive pitch takes the front (+x) down.
        assert!(zup(rotation * Vec3::X)[2] < 0.0);
    }

    #[test]
    fn a_pushed_block_slides_as_coulomb_friction_predicts() {
        let s = summary("slide", Settings::default());
        // (F/m - mu g) T, its slide, its stop and the load moved forward.
        let speed = (2.0 * G_M_S2 - G_M_S2) * PUSH_S;
        assert!(
            close(&s["speed_after_push_m_s"], speed, 0.002 * speed),
            "{s}"
        );
        let slide = speed * PUSH_S / 2.0 + speed * speed / (2.0 * G_M_S2);
        assert!(close(&s["slide_m"], slide, 0.005 * slide), "{s}");
        assert!(close(&s["stop_after_s"], speed / G_M_S2, 2.0 * DT_S), "{s}");
        // Rapier reports (n + 1) / n of the applied contact force at n = 4
        // solver iterations (README); scaled back, the loads match.
        let applied = |key: &str| Value::from(s[key].as_f64().unwrap() * 4.0 / 5.0);
        let shift = WEIGHT_N * (SPHERE_OFFSET_M[2] + SPHERE_RADIUS_M) / SPHERE_OFFSET_M[0];
        assert!(
            close(
                &applied("push_front_normal_n"),
                (WEIGHT_N + shift) / 2.0,
                1.0
            ),
            "{s}"
        );
        assert!(
            close(
                &applied("push_back_normal_n"),
                (WEIGHT_N - shift) / 2.0,
                1.0
            ),
            "{s}"
        );
        assert_eq!(s["airborne_share"], 0.0);
    }

    #[test]
    fn static_friction_holds_and_rapier_reports_n_plus_one_over_n_of_the_weight() {
        for iterations in [1, 4] {
            let scenario = SCENARIOS[0];
            let result = run(
                scenario,
                Settings {
                    solver_iterations: iterations,
                    ..Settings::default()
                },
            )
            .unwrap();
            let before_push = result.samples.as_array().unwrap()[..(SETTLE_S / 0.01) as usize]
                .last()
                .unwrap()
                .clone();
            let load = before_push["front_normal_n"].as_f64().unwrap()
                + before_push["back_normal_n"].as_f64().unwrap();
            // Rapier counts the warm start carried in from the previous step:
            // twice the weight at one iteration, 1.25 times at four.
            let reported = WEIGHT_N * (iterations as f64 + 1.0) / iterations as f64;
            assert!(
                (load - reported).abs() < 0.005 * reported,
                "{iterations}: {load} N"
            );
            // Static friction holds: 0.03 mm of creep at 4 iterations, 0.2 at 1.
            assert!(result.summary["slide_m"].as_f64().unwrap().abs() < 5e-4);
        }
    }

    #[test]
    fn rocking_and_landing_blocks_settle_without_rebound() {
        let rock = summary("rock", Settings::default());
        // The rigid block's rotation about its front contact line peaks at
        // 0.1274 rad (mujoco_contact_slide.py integrates it).
        assert!(close(&rock["max_pitch_rad"], 0.1274, 0.004), "{rock}");
        assert!(rock["min_pitch_rad"].as_f64().unwrap() > -0.005, "{rock}");
        let landing = summary("landing", Settings::default());
        assert!(
            landing["max_pitch_rad"].as_f64().unwrap() < 0.02,
            "{landing}"
        );
        assert!(landing["hop_m"].as_f64().unwrap() < 0.001, "{landing}");
        assert_eq!(landing["airborne_share"], 0.0);
    }
}
