//! Validated, version-independent geometry for editable navigation scenes.
use rne_math::{Transform3, Vec3};
use rne_render::Camera;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub(crate) struct Point {
    pub(crate) x_m: f64,
    pub(crate) z_m: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub(crate) struct BoxSpec {
    pub(crate) x_m: f64,
    pub(crate) z_m: f64,
    pub(crate) width_m: f64,
    pub(crate) depth_m: f64,
    #[serde(default)]
    pub(crate) appear_step: u32,
}
impl BoxSpec {
    fn clearance(&self, p: Point) -> f64 {
        ((p.x_m - self.x_m).abs() - self.width_m * 0.5)
            .max(0.0)
            .hypot(((p.z_m - self.z_m).abs() - self.depth_m * 0.5).max(0.0))
            - 0.28
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub(crate) struct SceneSpec {
    pub(crate) goal: Point,
    pub(crate) obstacles: Vec<BoxSpec>,
}
impl Default for SceneSpec {
    fn default() -> Self {
        Self {
            goal: Point {
                x_m: 2.8,
                z_m: -0.55,
            },
            obstacles: vec![],
        }
    }
}
impl SceneSpec {
    pub(crate) fn parse(json: &str) -> Result<Self, String> {
        if json.len() > 65_536 {
            return Err("シーンは64 KiB以下にしてください".into());
        }
        let scene: Self =
            serde_json::from_str(json).map_err(|_| "シーンJSONの形式を確認してください")?;
        scene.validate()?;
        Ok(scene)
    }
    pub(crate) fn validate(&self) -> Result<(), String> {
        let p = self.goal;
        if !p.x_m.is_finite()
            || !p.z_m.is_finite()
            || p.x_m.abs() > 3.4
            || p.z_m.abs() > 3.4
            || p.x_m.hypot(p.z_m) < 0.55
        {
            return Err("目標はX/Z ±3.4m以内、開始点から0.55m以上離してください".into());
        }
        if self.obstacles.len() > 24 {
            return Err("障害物は24個までです".into());
        }
        for o in &self.obstacles {
            if ![o.x_m, o.z_m, o.width_m, o.depth_m]
                .iter()
                .all(|v| v.is_finite())
                || !(0.2..=3.0).contains(&o.width_m)
                || !(0.2..=3.0).contains(&o.depth_m)
                || o.x_m.abs() + o.width_m * 0.5 > 3.65
                || o.z_m.abs() + o.depth_m * 0.5 > 3.65
                || o.appear_step > 900
            {
                return Err(
                    "障害物の寸法は0.2〜3m、外縁は±3.65m以内、出現は900ステップ以内です".into(),
                );
            }
            if o.appear_step == 0
                && (o.clearance(Point { x_m: 0.0, z_m: 0.0 }) < 0.15 || o.clearance(p) < 0.15)
            {
                return Err(
                    "開始点・目標と障害物の間に、ロボット半径を含め0.43m以上の余裕が必要です"
                        .into(),
                );
            }
        }
        Ok(())
    }
    pub(crate) fn id(&self) -> String {
        let bytes = serde_json::to_vec(self).expect("validated scene is serializable");
        let hash = bytes.iter().fold(0xcbf29ce484222325u64, |h, b| {
            (h ^ u64::from(*b)).wrapping_mul(0x100000001b3)
        });
        format!("{hash:016x}")
    }
}

/// Intersects a renderer-matched camera ray with the laboratory floor (Y=0).
pub(crate) fn pick_floor(
    camera: Camera,
    view: &Transform3,
    ndc_x: f64,
    ndc_y: f64,
) -> Option<Point> {
    if !ndc_x.is_finite() || !ndc_y.is_finite() || ndc_x.abs() > 1.0 || ndc_y.abs() > 1.0 {
        return None;
    }
    let tangent = (camera.fov_y_rad * 0.5).tan();
    let local = Vec3::new(
        ndc_x * f64::from(camera.width) / f64::from(camera.height.max(1)) * tangent,
        ndc_y * tangent,
        -1.0,
    );
    let ray = view.rotation * local;
    if ray.y >= -1e-8 {
        return None;
    }
    let t = -view.translation.y / ray.y;
    if t <= 0.0 {
        return None;
    }
    let p = view.translation + ray * t;
    p.is_finite().then_some(Point { x_m: p.x, z_m: p.z })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn scene_roundtrips_and_rejects_invalid_geometry_atomically() {
        let scene = SceneSpec::default();
        let json = serde_json::to_string(&scene).unwrap();
        let restored = SceneSpec::parse(&json).unwrap();
        assert_eq!(scene, restored);
        assert_eq!(scene.id(), restored.id());
        assert!(SceneSpec::parse(r#"{"goal":{"x_m":0,"z_m":0},"obstacles":[]}"#).is_err());
        assert!(
            SceneSpec::parse(r#"{"goal":{"x_m":2,"z_m":0},"obstacles":[],"surprise":true}"#)
                .is_err()
        );
        let mut invalid = scene.clone();
        invalid.goal.x_m = f64::NAN;
        assert!(invalid.validate().is_err());
        invalid = scene.clone();
        invalid.obstacles = vec![BoxSpec {
            x_m: 0.0,
            z_m: 0.0,
            width_m: 1.0,
            depth_m: 1.0,
            appear_step: 0,
        }];
        assert!(invalid.validate().is_err());
        invalid.obstacles[0].appear_step = 60;
        assert!(invalid.validate().is_ok());
        invalid.obstacles[0].width_m = 3.1;
        assert!(invalid.validate().is_err());
        invalid.obstacles[0].width_m = 1.0;
        invalid.obstacles[0].appear_step = 901;
        assert!(invalid.validate().is_err());
        invalid.obstacles[0].appear_step = 60;
        invalid.obstacles[0].x_m = 3.4;
        assert!(invalid.validate().is_err());
        invalid.obstacles[0].x_m = 0.0;
        invalid.obstacles = vec![invalid.obstacles[0].clone(); 25];
        assert!(invalid.validate().is_err());
        assert!(SceneSpec::parse(&" ".repeat(65_537)).is_err());
    }
    #[test]
    fn floor_picking_matches_projection_after_orbit_and_resize() {
        for (width, height) in [(1280, 800), (390, 422)] {
            let camera = Camera::new(width, height, std::f64::consts::FRAC_PI_4);
            for yaw in [0.0, 0.75, 1.4] {
                let orbit = rne_render_wgpu::CameraOrbit {
                    yaw_rad: yaw,
                    pitch_rad: 1.0,
                    distance_m: 4.5,
                    ..Default::default()
                };
                let view = orbit.camera_transform();
                let target = Vec3::new(0.4, 0.0, -0.3);
                let ndc = camera.view_projection(&view).project_point3(target);
                let hit = pick_floor(camera, &view, ndc.x, ndc.y).unwrap();
                assert!((hit.x_m - target.x).abs() < 1e-9);
                assert!((hit.z_m - target.z).abs() < 1e-9);
            }
        }
    }
}
