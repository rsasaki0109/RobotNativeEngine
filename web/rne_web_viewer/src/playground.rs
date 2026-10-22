//! Deterministic pose animation and ordered checkpoint course, without physics.
use rne_math::Vec3;

pub(crate) const GOALS: [(f64, f64); 4] = [(1.5, 0.0), (1.5, -1.5), (0.0, -1.5), (0.0, 0.0)];
pub(crate) const GOAL_RADIUS_M: f64 = 0.35;
const WAVE_TICKS: u32 = 180;

#[derive(Debug, Default)]
pub(crate) struct Playground {
    phase_rad: f64,
    stride_blend: f64,
    wave_tick: Option<u32>,
    pub(crate) course: Course,
}

impl Playground {
    pub(crate) fn tick(
        &mut self,
        travel_m: f64,
        turn: f64,
        enabled: bool,
        wave: bool,
        position: Vec3,
    ) {
        let moving = enabled && (travel_m > 1e-8 || turn.abs() > 1e-8);
        self.stride_blend = (self.stride_blend + if moving { 0.12 } else { -0.12 }).clamp(0.0, 1.0);
        if moving {
            self.phase_rad =
                (self.phase_rad + travel_m * std::f64::consts::TAU / 0.7 + turn.abs() * 0.10)
                    .rem_euclid(std::f64::consts::TAU);
        } else if self.stride_blend == 0.0 {
            self.phase_rad = 0.0;
        }
        if wave {
            self.wave_tick = Some(0);
        }
        self.wave_tick = self
            .wave_tick
            .and_then(|tick| (tick + 1 < WAVE_TICKS).then_some(tick + 1));
        self.course.tick(position);
    }

    pub(crate) fn angle(&self, name: &str, manual_rad: f64) -> f64 {
        let sine = self.phase_rad.sin();
        let (left, kind) = if let Some(kind) = name.strip_prefix("left_") {
            (true, kind)
        } else {
            (false, name.strip_prefix("right_").unwrap_or(name))
        };
        let leg = if left { sine } else { -sine };
        let offset = match kind {
            "hip_pitch_joint" => 0.30 * leg,
            "knee_joint" => 0.55 * (-leg).max(0.0),
            "ankle_pitch_joint" => -0.22 * (-leg).max(0.0),
            "shoulder_pitch_joint" => -0.22 * leg,
            _ => 0.0,
        } * self.stride_blend;
        let angle = manual_rad + offset;
        let Some(tick) = self.wave_tick else {
            return angle;
        };
        let weight = (f64::from(tick) / 20.0)
            .min(f64::from(WAVE_TICKS - tick) / 20.0)
            .min(1.0);
        let target = match name {
            "left_shoulder_pitch_joint" => -0.7,
            "left_shoulder_roll_joint" => 1.65,
            "left_elbow_joint" => 1.2 + 0.20 * (f64::from(tick) * 0.18).sin(),
            "left_wrist_roll_joint" => 0.55 * (f64::from(tick) * 0.18).sin(),
            _ => return angle,
        };
        angle + (target - angle) * weight
    }

    pub(crate) fn motion_status(&self) -> &'static str {
        if self.wave_tick.is_some() {
            "手振り中"
        } else if self.stride_blend > 0.0 {
            "歩行モーション中"
        } else {
            "立ち姿"
        }
    }
}

#[derive(Debug, Default)]
pub(crate) struct Course {
    pub(crate) reached: usize,
    ticks: u64,
    active: bool,
}

impl Course {
    pub(crate) fn start(&mut self) {
        *self = Self {
            active: true,
            ..Self::default()
        };
    }

    fn tick(&mut self, position: Vec3) {
        if !self.active {
            return;
        }
        self.ticks = self.ticks.saturating_add(1);
        let (x, z) = GOALS[self.reached];
        if (position.x - x).hypot(position.z - z) <= GOAL_RADIUS_M {
            self.reached += 1;
            self.active = self.reached < GOALS.len();
        }
    }

    pub(crate) fn status(&self, position: Vec3) -> String {
        if self.reached == GOALS.len() {
            return format!(
                "クリア！ 4/4 · {:.2} 秒（プレビュー時間）",
                self.ticks as f64 / 60.0
            );
        }
        if !self.active {
            return "未開始 · 金色の輪を順番に4つ通過しよう".into();
        }
        let (x, z) = GOALS[self.reached];
        format!(
            "{}/4 通過 · 次 X {x:.1} / Z {z:.1} m · 距離 {:.2} m · {:.2} 秒",
            self.reached,
            (position.x - x).hypot(position.z - z),
            self.ticks as f64 / 60.0
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gait_changes_limbs_and_settles_exactly_back_to_manual_pose() {
        let mut play = Playground::default();
        for _ in 0..12 {
            play.tick(0.015, 0.0, true, false, Vec3::ZERO);
        }
        let left = play.angle("left_hip_pitch_joint", 0.0);
        assert!(left.abs() > 0.01);
        assert!((left + play.angle("right_hip_pitch_joint", 0.0)).abs() < 1e-12);
        for _ in 0..12 {
            play.tick(0.0, 0.0, true, false, Vec3::ZERO);
        }
        assert_eq!(play.angle("left_hip_pitch_joint", 0.2), 0.2);
        assert_eq!(play.motion_status(), "立ち姿");
    }

    #[test]
    fn wave_is_repeatable_finishes_and_does_not_change_legs() {
        let mut a = Playground::default();
        let mut b = Playground::default();
        for tick in 0..WAVE_TICKS + 5 {
            a.tick(0.0, 0.0, true, tick == 0, Vec3::ZERO);
            b.tick(0.0, 0.0, true, tick == 0, Vec3::ZERO);
            assert_eq!(
                a.angle("left_wrist_roll_joint", 0.0),
                b.angle("left_wrist_roll_joint", 0.0)
            );
            assert_eq!(a.angle("left_knee_joint", 0.3), 0.3);
            if tick == 40 {
                assert!(a.angle("left_shoulder_roll_joint", 0.0) > 1.0);
            }
        }
        assert_eq!(a.motion_status(), "立ち姿");
        assert_eq!(a.angle("left_elbow_joint", 0.2), 0.2);
    }

    #[test]
    fn course_requires_start_and_order_and_freezes_after_finish() {
        let mut course = Course::default();
        course.tick(Vec3::new(1.5, 0.0, 0.0));
        assert_eq!(course.reached, 0);
        course.start();
        course.tick(Vec3::new(0.0, 0.0, -1.5));
        assert_eq!(course.reached, 0, "out-of-order targets do not count");
        for (x, z) in GOALS {
            course.tick(Vec3::new(x, 0.0, z));
        }
        assert_eq!(course.reached, 4);
        let ticks = course.ticks;
        course.tick(Vec3::ZERO);
        assert_eq!(course.ticks, ticks);
        assert!(course.status(Vec3::ZERO).starts_with("クリア"));
        course.start();
        assert_eq!(course.reached, 0);
        assert_eq!(course.ticks, 0);
    }
}
