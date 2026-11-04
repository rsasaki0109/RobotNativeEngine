//! The scenarios, their closed-form answers and one RNE run of a scenario.

use std::f64::consts::PI;

use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Seconds, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsMaterial, PhysicsWorldDesc, PhysicsWorldId,
    RigidBody, RigidBodyInertia, RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_world::Transform3;
use serde_json::{json, Map, Value};

use crate::Result;

/// Physics step, as the walking loop's.
pub(crate) const DT_S: f64 = 0.002;
/// Standard gravity.
pub(crate) const G_M_S2: f64 = 9.81;
/// Half extents of every box (Z-up): 0.2 × 0.2 × 0.1 m.
pub(crate) const BOX_HALF_M: [f64; 3] = [0.1, 0.1, 0.05];
/// Radius of every ball.
pub(crate) const BALL_RADIUS_M: f64 = 0.05;
/// Physics steps between samples in the report (10 ms).
const SAMPLE_EVERY: usize = 5;
/// Seconds at the end of a stack run over which the contact loads are averaged.
const LOAD_WINDOW_S: f64 = 0.5;
/// Speed below which a sliding box counts as stopped.
pub(crate) const STOP_SPEED_M_S: f64 = 0.01;

/// What a scenario checks, and its parameters.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Check {
    /// A 10 kg box launched across a level floor at `speed_m_s`, `direction_rad`
    /// from +x: it stops after v² / (2 μ g) along a straight line, whatever the
    /// direction.
    Slide { speed_m_s: f64, direction_rad: f64 },
    /// A 10 kg box released on an incline of `angle_rad` (gravity tilted toward
    /// +x): it stays if tan θ ≤ μ and otherwise slides at g (sin θ − μ cos θ).
    Incline { angle_rad: f64 },
    /// A 1 kg solid ball released on an incline: it rolls at 5/7 g sin θ if
    /// μ ≥ 2/7 tan θ, and otherwise slides at g (sin θ − μ cos θ) while its spin
    /// grows at 5 μ g cos θ / (2 r).
    Roll { angle_rad: f64 },
    /// A 1 kg ball dropped with its bottom `drop_m` above the floor: it rebounds
    /// to e² of that.
    Bounce { drop_m: f64 },
    /// A box of `top_kg` resting on a 1 kg box: neither moves, and the contacts
    /// carry the weights above them.
    Stack { top_kg: f64 },
}

/// One scenario.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Scenario {
    pub(crate) name: &'static str,
    pub(crate) what: &'static str,
    /// Friction coefficient of the floor and every body.
    pub(crate) friction: f64,
    /// Restitution coefficient of the floor and every body.
    pub(crate) restitution: f64,
    pub(crate) duration_s: f64,
    pub(crate) check: Check,
}

/// The scenarios, in report order.
pub(crate) const SCENARIOS: [Scenario; 12] = [
    Scenario {
        name: "slide_0",
        what: "box launched at 2 m/s along +x on friction 0.5",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 0.8,
        check: Check::Slide {
            speed_m_s: 2.0,
            direction_rad: 0.0,
        },
    },
    Scenario {
        name: "slide_22.5",
        what: "the same, 22.5 degrees from +x",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 0.8,
        check: Check::Slide {
            speed_m_s: 2.0,
            direction_rad: PI / 8.0,
        },
    },
    Scenario {
        name: "slide_45",
        what: "the same, 45 degrees from +x",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 0.8,
        check: Check::Slide {
            speed_m_s: 2.0,
            direction_rad: PI / 4.0,
        },
    },
    Scenario {
        name: "incline_hold_20",
        what: "box at rest on a 20 degree incline, friction 0.5 (holds: tan 20° = 0.36)",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 2.0,
        check: Check::Incline {
            angle_rad: 20.0 * PI / 180.0,
        },
    },
    Scenario {
        name: "incline_slide_35",
        what: "box at rest on a 35 degree incline, friction 0.5 (slides: tan 35° = 0.70)",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 1.0,
        check: Check::Incline {
            angle_rad: 35.0 * PI / 180.0,
        },
    },
    Scenario {
        name: "roll_20",
        what: "ball at rest on a 20 degree incline, friction 0.5 (rolls without slipping)",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 1.0,
        check: Check::Roll {
            angle_rad: 20.0 * PI / 180.0,
        },
    },
    Scenario {
        name: "roll_slip_20",
        what: "the same on friction 0.05 (slides while it spins up)",
        friction: 0.05,
        restitution: 0.0,
        duration_s: 1.0,
        check: Check::Roll {
            angle_rad: 20.0 * PI / 180.0,
        },
    },
    Scenario {
        name: "bounce_e0",
        what: "ball dropped from 0.2 m, restitution 0",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 1.0,
        check: Check::Bounce { drop_m: 0.2 },
    },
    Scenario {
        name: "bounce_e0.5",
        what: "ball dropped from 0.2 m, restitution 0.5",
        friction: 0.5,
        restitution: 0.5,
        duration_s: 1.0,
        check: Check::Bounce { drop_m: 0.2 },
    },
    Scenario {
        name: "stack_1",
        what: "1 kg box resting on a 1 kg box",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 2.0,
        check: Check::Stack { top_kg: 1.0 },
    },
    Scenario {
        name: "stack_10",
        what: "10 kg box resting on a 1 kg box",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 2.0,
        check: Check::Stack { top_kg: 10.0 },
    },
    Scenario {
        name: "stack_100",
        what: "100 kg box resting on a 1 kg box",
        friction: 0.5,
        restitution: 0.0,
        duration_s: 2.0,
        check: Check::Stack { top_kg: 100.0 },
    },
];

/// A body's shape.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum Shape {
    Box,
    Ball,
}

/// A body's initial state (Z-up).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Body {
    pub(crate) shape: Shape,
    pub(crate) mass_kg: f64,
    pub(crate) position_m: [f64; 3],
    pub(crate) velocity_m_s: [f64; 3],
}

impl Body {
    /// Principal moments of inertia about x, y and z (Z-up) of the solid shape.
    pub(crate) fn inertia_kg_m2(&self) -> [f64; 3] {
        match self.shape {
            Shape::Box => {
                let [a, b, c] = BOX_HALF_M.map(|h| 2.0 * h);
                let m = self.mass_kg / 12.0;
                [
                    m * (b * b + c * c),
                    m * (a * a + c * c),
                    m * (a * a + b * b),
                ]
            }
            Shape::Ball => [0.4 * self.mass_kg * BALL_RADIUS_M * BALL_RADIUS_M; 3],
        }
    }

    fn to_json(self) -> Value {
        let shape = match self.shape {
            Shape::Box => json!({"kind": "box", "half_extents_m": BOX_HALF_M}),
            Shape::Ball => json!({"kind": "ball", "radius_m": BALL_RADIUS_M}),
        };
        json!({
            "shape": shape, "mass_kg": self.mass_kg,
            "principal_inertia_kg_m2": self.inertia_kg_m2(),
            "position_m": self.position_m, "velocity_m_s": self.velocity_m_s,
        })
    }
}

impl Scenario {
    /// Gravity (Z-up); an incline tilts it toward +x instead of tilting the floor.
    pub(crate) fn gravity_m_s2(&self) -> [f64; 3] {
        match self.check {
            Check::Incline { angle_rad } | Check::Roll { angle_rad } => {
                [G_M_S2 * angle_rad.sin(), 0.0, -G_M_S2 * angle_rad.cos()]
            }
            _ => [0.0, 0.0, -G_M_S2],
        }
    }

    /// The bodies, the subject first.
    pub(crate) fn bodies(&self) -> Vec<Body> {
        let resting_box = |mass_kg: f64, z: f64| Body {
            shape: Shape::Box,
            mass_kg,
            position_m: [0.0, 0.0, z],
            velocity_m_s: [0.0; 3],
        };
        let ball = |z: f64| Body {
            shape: Shape::Ball,
            mass_kg: 1.0,
            position_m: [0.0, 0.0, z],
            velocity_m_s: [0.0; 3],
        };
        match self.check {
            Check::Slide {
                speed_m_s,
                direction_rad,
            } => vec![Body {
                velocity_m_s: [
                    speed_m_s * direction_rad.cos(),
                    speed_m_s * direction_rad.sin(),
                    0.0,
                ],
                ..resting_box(10.0, BOX_HALF_M[2])
            }],
            Check::Incline { .. } => vec![resting_box(10.0, BOX_HALF_M[2])],
            Check::Roll { .. } => vec![ball(BALL_RADIUS_M)],
            Check::Bounce { drop_m } => vec![ball(drop_m + BALL_RADIUS_M)],
            Check::Stack { top_kg } => vec![
                resting_box(top_kg, 3.0 * BOX_HALF_M[2]),
                resting_box(1.0, BOX_HALF_M[2]),
            ],
        }
    }

    pub(crate) fn to_json(self) -> Value {
        let check = match self.check {
            Check::Slide {
                speed_m_s,
                direction_rad,
            } => json!({"kind": "slide", "speed_m_s": speed_m_s, "direction_rad": direction_rad}),
            Check::Incline { angle_rad } => json!({"kind": "incline", "angle_rad": angle_rad}),
            Check::Roll { angle_rad } => json!({"kind": "roll", "angle_rad": angle_rad}),
            Check::Bounce { drop_m } => json!({"kind": "bounce", "drop_m": drop_m}),
            Check::Stack { top_kg } => json!({"kind": "stack", "top_kg": top_kg}),
        };
        json!({
            "name": self.name, "what": self.what, "friction": self.friction,
            "restitution": self.restitution, "duration_s": self.duration_s, "dt_s": DT_S,
            "gravity_m_s2": self.gravity_m_s2(), "check": check,
            "bodies": self.bodies().into_iter().map(Body::to_json).collect::<Vec<_>>(),
        })
    }

    /// The closed-form value of every summary metric this scenario has one for.
    pub(crate) fn closed_form(&self) -> Value {
        let mu = self.friction;
        let t = self.duration_s;
        match self.check {
            Check::Slide { speed_m_s: v, .. } => json!({
                "distance_m": v * v / (2.0 * mu * G_M_S2),
                "lateral_m": 0.0,
                "stop_s": (v - STOP_SPEED_M_S) / (mu * G_M_S2),
            }),
            Check::Incline { angle_rad } => {
                let a = if angle_rad.tan() <= mu {
                    0.0
                } else {
                    G_M_S2 * (angle_rad.sin() - mu * angle_rad.cos())
                };
                json!({"displacement_m": a * t * t / 2.0})
            }
            Check::Roll { angle_rad } => {
                let (a, alpha) = if mu >= 2.0 / 7.0 * angle_rad.tan() {
                    let a = 5.0 / 7.0 * G_M_S2 * angle_rad.sin();
                    (a, a / BALL_RADIUS_M)
                } else {
                    (
                        G_M_S2 * (angle_rad.sin() - mu * angle_rad.cos()),
                        2.5 * mu * G_M_S2 * angle_rad.cos() / BALL_RADIUS_M,
                    )
                };
                json!({
                    "displacement_m": a * t * t / 2.0, "speed_m_s": a * t,
                    "spin_rad_s": alpha * t, "slip_m_s": (a - alpha * BALL_RADIUS_M) * t,
                })
            }
            Check::Bounce { drop_m } => {
                json!({"apex_m": self.restitution * self.restitution * drop_m})
            }
            Check::Stack { top_kg } => json!({
                "top_sink_m": 0.0, "top_drift_m": 0.0,
                "ground_load_n": (top_kg + 1.0) * G_M_S2, "stack_load_n": top_kg * G_M_S2,
            }),
        }
    }
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

/// Z-up (x, y, z) from RNE's Y-up.
fn zup(v: Vec3) -> [f64; 3] {
    [v.x, -v.z, v.y]
}

/// RNE's Y-up from Z-up.
fn yup([x, y, z]: [f64; 3]) -> Vec3 {
    Vec3::new(x, z, -y)
}

fn spawn(world: &mut World, scenario: &Scenario) -> (Entity, Vec<Entity>) {
    let material = PhysicsMaterial {
        friction: scenario.friction as f32,
        restitution: scenario.restitution as f32,
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
    let bodies = scenario
        .bodies()
        .into_iter()
        .enumerate()
        .map(|(index, body)| {
            let shape = match body.shape {
                Shape::Box => {
                    let [x, y, z] = BOX_HALF_M;
                    ColliderShape::Cuboid {
                        half_extents_m: Vec3::new(x, z, y),
                    }
                }
                Shape::Ball => ColliderShape::Sphere {
                    radius_m: BALL_RADIUS_M,
                },
            };
            let [ixx, iyy, izz] = body.inertia_kg_m2();
            let entity = spawn_named(world, format!("body{index}"));
            world.entity_mut(entity).insert((
                RigidBody {
                    mass_kg: body.mass_kg,
                    linear_velocity_m_s: yup(body.velocity_m_s),
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
                    shape,
                    material,
                    ..Collider::default()
                },
                Transform3::from_translation_rotation(yup(body.position_m), Quat::IDENTITY),
            ));
            entity
        })
        .collect();
    (ground, bodies)
}

/// One physics step's record (Z-up): every body's state and the contact loads.
#[derive(Clone, Debug)]
pub(crate) struct Step {
    pub(crate) time_s: f64,
    pub(crate) position_m: Vec<[f64; 3]>,
    pub(crate) velocity_m_s: Vec<[f64; 3]>,
    pub(crate) spin_rad_s: Vec<[f64; 3]>,
    /// Normal force through every contact with the floor.
    pub(crate) ground_load_n: f64,
    /// Normal force through every contact between the first two bodies.
    pub(crate) stack_load_n: f64,
}

/// Normal force on the floor, and between the first two bodies.
fn loads(
    backend: &RapierBackend,
    id: PhysicsWorldId,
    ground: Entity,
    bodies: &[Entity],
) -> Result<(f64, f64)> {
    let (mut on_ground, mut between) = (0.0, 0.0);
    for point in backend.contact_points(id)? {
        let pair = [point.entity_a, point.entity_b];
        if pair.contains(&ground) {
            on_ground += point.normal_force_n;
        } else if bodies.len() > 1 && pair.contains(&bodies[0]) && pair.contains(&bodies[1]) {
            between += point.normal_force_n;
        }
    }
    Ok((on_ground, between))
}

/// A run's summary and its samples (every 10 ms).
pub(crate) struct RunResult {
    pub(crate) summary: Value,
    pub(crate) samples: Value,
}

/// Runs `scenario` in RNE.
pub(crate) fn run(scenario: Scenario, settings: Settings) -> Result<RunResult> {
    let mut backend = RapierBackend::new();
    let id = backend.create_world(PhysicsWorldDesc {
        gravity_m_s2: yup(scenario.gravity_m_s2()),
        solver_iterations: settings.iterations(),
    })?;
    if let Some((hz, ratio)) = settings.contact_softness {
        if !backend.set_contact_softness(id, hz, ratio) {
            return Err("contact softness needs a positive frequency and damping ratio".into());
        }
    }
    let mut world = World::new();
    let (ground, bodies) = spawn(&mut world, &scenario);
    let initial = Step {
        time_s: 0.0,
        position_m: scenario.bodies().iter().map(|b| b.position_m).collect(),
        velocity_m_s: scenario.bodies().iter().map(|b| b.velocity_m_s).collect(),
        spin_rad_s: vec![[0.0; 3]; bodies.len()],
        ground_load_n: 0.0,
        stack_load_n: 0.0,
    };
    let steps = (scenario.duration_s / DT_S).round() as usize;
    let dt = SimDuration::from_seconds(Seconds::new(DT_S));
    let mut record = vec![initial];
    for step in 0..steps {
        backend.sync_from_ecs(&mut world, id)?;
        backend.step(id, dt)?;
        backend.sync_to_ecs(&mut world, id)?;
        let (ground_load_n, stack_load_n) = loads(&backend, id, ground, &bodies)?;
        let mut state = Step {
            time_s: (step + 1) as f64 * DT_S,
            position_m: Vec::new(),
            velocity_m_s: Vec::new(),
            spin_rad_s: Vec::new(),
            ground_load_n,
            stack_load_n,
        };
        for &entity in &bodies {
            let pose = world.get::<Transform3>(entity).ok_or("body has no pose")?;
            let body = world.get::<RigidBody>(entity).ok_or("body has no state")?;
            state.position_m.push(zup(pose.translation));
            state.velocity_m_s.push(zup(body.linear_velocity_m_s));
            state.spin_rad_s.push(zup(body.angular_velocity_rad_s));
        }
        record.push(state);
    }
    let samples = record
        .iter()
        .step_by(SAMPLE_EVERY)
        .map(|s| {
            json!({"time_s": s.time_s, "position_m": s.position_m,
                   "velocity_m_s": s.velocity_m_s, "spin_rad_s": s.spin_rad_s,
                   "ground_load_n": s.ground_load_n, "stack_load_n": s.stack_load_n})
        })
        .collect();
    Ok(RunResult {
        summary: summarize(&scenario, &record),
        samples,
    })
}

/// The summary both engines' reports carry, from every physics step, the
/// initial state first (`mujoco_contact_closed_form.py` computes the same).
pub(crate) fn summarize(scenario: &Scenario, steps: &[Step]) -> Value {
    let first = &steps[0];
    let last = steps.last().expect("steps");
    let moved = |axis: usize| last.position_m[0][axis] - first.position_m[0][axis];
    let mut summary = Map::new();
    match scenario.check {
        Check::Slide { direction_rad, .. } => {
            let (sin, cos) = direction_rad.sin_cos();
            let (dx, dy) = (moved(0), moved(1));
            let stop_s = steps.iter().skip(1).find_map(|s| {
                let [vx, vy, _] = s.velocity_m_s[0];
                (vx.hypot(vy) < STOP_SPEED_M_S).then_some(s.time_s)
            });
            summary.insert("distance_m".into(), json!(dx * cos + dy * sin));
            summary.insert("lateral_m".into(), json!((dy * cos - dx * sin).abs()));
            summary.insert("stop_s".into(), json!(stop_s));
        }
        Check::Incline { .. } => {
            summary.insert("displacement_m".into(), json!(moved(0)));
        }
        Check::Roll { .. } => {
            let speed = last.velocity_m_s[0][0];
            let spin = last.spin_rad_s[0][1];
            summary.insert("displacement_m".into(), json!(moved(0)));
            summary.insert("speed_m_s".into(), json!(speed));
            summary.insert("spin_rad_s".into(), json!(spin));
            // The contact point's speed: the centre's height above the floor
            // is the rolling radius, smaller than the ball's by the contact's
            // penetration.
            summary.insert(
                "slip_m_s".into(),
                json!(speed - spin * last.position_m[0][2]),
            );
        }
        Check::Bounce { .. } => {
            // From the first step the falling ball stops falling, the highest
            // its bottom gets: later rebounds are lower.
            let falling = steps
                .iter()
                .position(|s| s.velocity_m_s[0][2] < -0.1)
                .unwrap_or(steps.len());
            let apex = steps[falling..]
                .iter()
                .position(|s| s.velocity_m_s[0][2] >= 0.0)
                .map(|impact| {
                    let impact = falling + impact;
                    steps[impact..]
                        .iter()
                        .map(|s| s.position_m[0][2] - BALL_RADIUS_M)
                        .fold(f64::NEG_INFINITY, f64::max)
                });
            summary.insert("apex_m".into(), json!(apex));
        }
        Check::Stack { .. } => {
            let window: Vec<&Step> = steps
                .iter()
                .filter(|s| s.time_s > scenario.duration_s - LOAD_WINDOW_S + 1e-9)
                .collect();
            let mean = |f: fn(&Step) -> f64| {
                window.iter().map(|s| f(s)).sum::<f64>() / window.len() as f64
            };
            summary.insert("top_sink_m".into(), json!(-moved(2)));
            summary.insert("top_drift_m".into(), json!(moved(0).hypot(moved(1))));
            summary.insert("ground_load_n".into(), json!(mean(|s| s.ground_load_n)));
            summary.insert("stack_load_n".into(), json!(mean(|s| s.stack_load_n)));
        }
    }
    Value::Object(summary)
}

/// One line per metric: RNE against the closed form.
pub(crate) fn summary_lines(scenario: &Scenario, summary: &Value) -> Vec<String> {
    let exact = scenario.closed_form();
    let number = |v: &Value| v.as_f64().map_or("—".to_string(), |v| format!("{v:.4}"));
    let mut lines = vec![format!("{}: {}", scenario.name, scenario.what)];
    for (metric, value) in summary.as_object().expect("summary is an object") {
        lines.push(format!(
            "  {metric:16} closed form {:>9}  RNE {:>9}",
            number(&exact[metric]),
            number(value)
        ));
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scenario(name: &str) -> Scenario {
        *SCENARIOS.iter().find(|s| s.name == name).unwrap()
    }

    fn summary(name: &str, settings: Settings) -> Value {
        run(scenario(name), settings).unwrap().summary
    }

    /// RNE's value of `metric` minus the closed form's.
    fn error(name: &str, summary: &Value, metric: &str) -> f64 {
        summary[metric].as_f64().unwrap() - scenario(name).closed_form()[metric].as_f64().unwrap()
    }

    #[test]
    fn frames_round_trip_and_inertias_are_the_solid_shapes() {
        assert_eq!(zup(yup([1.0, 2.0, 3.0])), [1.0, 2.0, 3.0]);
        let [ball, ..] = scenario("roll_20").bodies()[0].inertia_kg_m2();
        assert!((ball - 0.4 * 0.05 * 0.05).abs() < 1e-15);
        let [ixx, iyy, izz] = scenario("slide_0").bodies()[0].inertia_kg_m2();
        assert!((ixx - 10.0 / 12.0 * 0.05).abs() < 1e-12 && ixx == iyy);
        assert!((izz - 10.0 / 12.0 * 0.08).abs() < 1e-12);
    }

    #[test]
    fn a_sliding_box_stops_where_coulomb_friction_predicts_in_every_direction() {
        for name in ["slide_0", "slide_22.5", "slide_45"] {
            let s = summary(name, Settings::default());
            // 0.4077 m along a straight line, whatever the direction: Rapier's
            // friction is a cone, not a pyramid.
            assert!(error(name, &s, "distance_m").abs() < 0.002, "{name}: {s}");
            assert!(s["lateral_m"].as_f64().unwrap() < 0.001, "{name}: {s}");
            assert!(error(name, &s, "stop_s").abs() <= DT_S, "{name}: {s}");
        }
    }

    #[test]
    fn a_box_holds_below_the_friction_angle_and_slides_above_it() {
        let hold = summary("incline_hold_20", Settings::default());
        assert!(
            hold["displacement_m"].as_f64().unwrap().abs() < 0.001,
            "{hold}"
        );
        let slide = summary("incline_slide_35", Settings::default());
        assert!(
            error("incline_slide_35", &slide, "displacement_m").abs() < 0.01 * 0.8044,
            "{slide}"
        );
    }

    #[test]
    fn a_ball_rolls_or_slips_as_rigid_body_dynamics_predict() {
        let roll = summary("roll_20", Settings::default());
        for metric in ["displacement_m", "speed_m_s", "spin_rad_s"] {
            let relative = error("roll_20", &roll, metric)
                / scenario("roll_20").closed_form()[metric].as_f64().unwrap();
            assert!(relative.abs() < 0.01, "{metric}: {roll}");
        }
        // The contact point slips 0.03 m/s against the 2.4 m/s the ball rolls at.
        assert!(roll["slip_m_s"].as_f64().unwrap().abs() < 0.05, "{roll}");
        let slip = summary("roll_slip_20", Settings::default());
        for metric in ["displacement_m", "speed_m_s", "spin_rad_s", "slip_m_s"] {
            let relative = error("roll_slip_20", &slip, metric)
                / scenario("roll_slip_20").closed_form()[metric]
                    .as_f64()
                    .unwrap();
            assert!(relative.abs() < 0.03, "{metric}: {slip}");
        }
    }

    #[test]
    fn a_dropped_ball_rebounds_to_restitution_squared() {
        let dead = summary("bounce_e0", Settings::default());
        assert!(dead["apex_m"].as_f64().unwrap().abs() < 0.002, "{dead}");
        let bouncy = summary("bounce_e0.5", Settings::default());
        assert!(
            error("bounce_e0.5", &bouncy, "apex_m").abs() < 0.03 * 0.05,
            "{bouncy}"
        );
    }

    #[test]
    fn stacks_carry_their_weights_and_stiffer_contacts_stop_them_sinking() {
        for name in ["stack_1", "stack_10", "stack_100"] {
            let s = summary(name, Settings::default());
            for metric in ["ground_load_n", "stack_load_n"] {
                let relative = error(name, &s, metric)
                    / scenario(name).closed_form()[metric].as_f64().unwrap();
                assert!(relative.abs() < 0.005, "{name} {metric}: {s}");
            }
            assert!(s["top_drift_m"].as_f64().unwrap() < 0.002, "{name}: {s}");
        }
        // Rapier's contact stiffness scales with the effective mass at the
        // contact, so a heavy box on a light one sinks: 83 mm at the default
        // 30 Hz, 3.3 mm at 240 Hz.
        let stiff = Settings {
            contact_softness: Some((240.0, 5.0)),
            ..Settings::default()
        };
        let sink = summary("stack_100", stiff)["top_sink_m"].as_f64().unwrap();
        assert!(sink < 0.005, "{sink}");
    }
}
