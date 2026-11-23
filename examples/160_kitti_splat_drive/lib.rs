//! Shared input parsing for the KITTI splat drive examples (160, 161).
//!
//! A long drive is reconstructed as several splats, one per stretch of the route, because one
//! splat of the whole route does not fit a CPU renderer's buffers. The examples switch splat
//! (and localization map) by the KITTI frame of the car's closest path point.

use rne_math::{DMat4, DVec3, Quat, Transform3, Vec3};
use std::path::{Path, PathBuf};

/// The path the car follows, thinned to points at least 1 m apart.
#[derive(Debug, Clone, PartialEq)]
pub struct DrivePath {
    /// Positions in the RNE frame (+Y up).
    pub points: Vec<Vec3>,
    /// KITTI frame of each point (0 when the CSV has no frame column).
    pub frames: Vec<u32>,
    /// Yaw about +Y at the first point.
    pub start_yaw: f64,
}

/// Parses a path CSV: `t_s, x, y, z, yaw[, kitti_frame]`, `#` comments.
pub fn parse_path(text: &str) -> Result<DrivePath, String> {
    let mut path = DrivePath {
        points: Vec::new(),
        frames: Vec::new(),
        start_yaw: 0.0,
    };
    for (number, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let v = line
            .split(',')
            .map(|s| s.trim().parse::<f64>())
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| format!("path line {}: {e}", number + 1))?;
        if v.len() < 5 {
            return Err(format!(
                "path line {}: expected at least 5 columns",
                number + 1
            ));
        }
        let p = Vec3::new(v[1], v[2], v[3]);
        if path
            .points
            .last()
            .is_none_or(|last| (p - *last).length() >= 1.0)
        {
            if path.points.is_empty() {
                path.start_yaw = v[4];
            }
            path.points.push(p);
            path.frames.push(v.get(5).map_or(0, |f| *f as u32));
        }
    }
    if path.points.is_empty() {
        return Err("path has no points".into());
    }
    Ok(path)
}

/// One stretch of the route: its splat manifest and, optionally, its localization map.
#[derive(Debug, Clone, PartialEq)]
pub struct Segment {
    /// First KITTI frame this segment is used for.
    pub start_frame: u32,
    pub manifest: PathBuf,
    pub map: Option<PathBuf>,
}

/// Parses a segment list: `start_frame, manifest[, map_dir]` per line, `#` comments.
/// Relative paths are taken relative to `base` (the list's directory).
pub fn parse_segments(text: &str, base: &Path) -> Result<Vec<Segment>, String> {
    let mut segments: Vec<Segment> = Vec::new();
    for (number, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = line.split(',').map(str::trim).collect();
        if !(2..=3).contains(&fields.len()) {
            return Err(format!(
                "segments line {}: expected 2 or 3 fields",
                number + 1
            ));
        }
        let start_frame = fields[0]
            .parse()
            .map_err(|e| format!("segments line {}: {e}", number + 1))?;
        if segments
            .last()
            .is_some_and(|s| s.start_frame >= start_frame)
        {
            return Err(format!(
                "segments line {}: start frames must increase",
                number + 1
            ));
        }
        segments.push(Segment {
            start_frame,
            manifest: base.join(fields[1]),
            map: fields.get(2).map(|m| base.join(m)),
        });
    }
    if segments.is_empty() {
        return Err("no segments".into());
    }
    Ok(segments)
}

/// Index of the segment used at `frame`: the last one starting at or before it.
pub fn segment_for(segments: &[Segment], frame: u32) -> usize {
    segments
        .iter()
        .rposition(|s| s.start_frame <= frame)
        .unwrap_or(0)
}

/// RNE camera pose of a SLAM camera: `r_wc` is the world <- optical camera rotation (x right,
/// y down, z forward; world z up) and `c` its centre. SLAM (x, y, z) maps to RNE (x, z, -y); an
/// RNE camera looks along its local -Z with +Y up, so its +X, +Y, -Z are the optical x, -y, z.
pub fn rne_view_from_slam(r_wc: &[[f64; 3]; 3], c: &[f64; 3]) -> Transform3 {
    let to_rne = |v: [f64; 3]| Vec3::new(v[0], v[2], -v[1]);
    let column = |j: usize, sign: f64| {
        to_rne([sign * r_wc[0][j], sign * r_wc[1][j], sign * r_wc[2][j]]).extend(0.0)
    };
    let basis = DMat4::from_cols(
        column(0, 1.0),
        column(1, -1.0),
        column(2, -1.0),
        DVec3::ZERO.extend(1.0),
    );
    Transform3::from_translation_rotation(to_rne(*c), Quat::from_mat4(&basis).normalize())
}

/// Cumulative horizontal arc length (x-z plane) at each path point.
pub fn arc_lengths(points: &[Vec3]) -> Vec<f64> {
    let mut s = Vec::with_capacity(points.len());
    let mut total = 0.0;
    for (i, p) in points.iter().enumerate() {
        if i > 0 {
            let d = *p - points[i - 1];
            total += (d.x * d.x + d.z * d.z).sqrt();
        }
        s.push(total);
    }
    s
}

/// Position and yaw (about +Y) at arc length `s` along the path, clamped to its ends.
pub fn pose_at(points: &[Vec3], lengths: &[f64], s: f64) -> (Vec3, f64) {
    let last = points.len() - 1;
    let i = lengths.partition_point(|&l| l <= s).clamp(1, last.max(1)) - 1;
    let j = (i + 1).min(last);
    let span = (lengths[j] - lengths[i]).max(1e-9);
    let u = ((s - lengths[i]) / span).clamp(0.0, 1.0);
    let p = points[i] + (points[j] - points[i]) * u;
    let d = points[j] - points[i];
    // Yaw about +Y with +X forward: +Z is to the right, so yaw = atan2(-dz, dx).
    (p, (-d.z).atan2(d.x))
}

/// Arc length of the point on the path closest to `p`, searching from point `hint` onward.
pub fn arc_length_near(points: &[Vec3], lengths: &[f64], hint: usize, p: Vec3) -> (f64, usize) {
    let mut i = hint.min(points.len() - 1);
    while i + 1 < points.len() && (points[i + 1] - p).length() <= (points[i] - p).length() {
        i += 1;
    }
    let j = (i + 1).min(points.len() - 1);
    let ab = points[j] - points[i];
    let len2 = ab.x * ab.x + ab.z * ab.z;
    let u = if len2 < 1e-12 {
        0.0
    } else {
        (((p.x - points[i].x) * ab.x + (p.z - points[i].z) * ab.z) / len2).clamp(0.0, 1.0)
    };
    (lengths[i] + u * (lengths[j] - lengths[i]), i)
}

/// Intelligent Driver Model parameters (car following).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Idm {
    /// Desired speed (m/s).
    pub desired_speed: f64,
    /// Desired time headway (s).
    pub headway_s: f64,
    /// Standstill gap (m).
    pub min_gap_m: f64,
    /// Maximum acceleration (m/s^2).
    pub max_accel: f64,
    /// Comfortable deceleration (m/s^2).
    pub comfort_decel: f64,
}

impl Idm {
    /// Acceleration for speed `v`, bumper gap `gap` and leader speed `v_lead`
    /// (`gap = None`: free road).
    pub fn accel(&self, v: f64, gap: Option<f64>, v_lead: f64) -> f64 {
        let free = 1.0 - (v / self.desired_speed).powi(4);
        let Some(gap) = gap else {
            return self.max_accel * free;
        };
        let dv = v - v_lead;
        let wanted = self.min_gap_m
            + (v * self.headway_s + v * dv / (2.0 * (self.max_accel * self.comfort_decel).sqrt()))
                .max(0.0);
        self.max_accel * (free - (wanted / gap.max(0.1)).powi(2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn path_thins_points_and_keeps_frames() {
        let text = "# t,x,y,z,yaw,frame\n0,0,0,0,0.5,1\n0.1,0.4,0,0,0.5,2\n0.2,1.2,0,0,0.5,3\n";
        let path = parse_path(text).unwrap();
        assert_eq!(path.points.len(), 2);
        assert_eq!(path.frames, vec![1, 3]);
        assert_eq!(path.start_yaw, 0.5);
    }

    #[test]
    fn path_without_frame_column_uses_zero() {
        let path = parse_path("0,0,0,0,0\n1,2,0,0,0\n").unwrap();
        assert_eq!(path.frames, vec![0, 0]);
    }

    #[test]
    fn path_rejects_short_rows_and_empty_input() {
        assert!(parse_path("0,1,2\n").is_err());
        assert!(parse_path("# only a comment\n").is_err());
    }

    #[test]
    fn segments_resolve_paths_and_pick_by_frame() {
        let text = "0, a.toml, map_a\n390, b.toml\n770, /abs/c.toml, /abs/map_c\n";
        let segments = parse_segments(text, Path::new("/base")).unwrap();
        assert_eq!(segments[0].manifest, PathBuf::from("/base/a.toml"));
        assert_eq!(segments[0].map, Some(PathBuf::from("/base/map_a")));
        assert_eq!(segments[1].map, None);
        assert_eq!(segments[2].manifest, PathBuf::from("/abs/c.toml"));
        assert_eq!(segment_for(&segments, 0), 0);
        assert_eq!(segment_for(&segments, 389), 0);
        assert_eq!(segment_for(&segments, 390), 1);
        assert_eq!(segment_for(&segments, 2000), 2);
    }

    #[test]
    fn segments_reject_unordered_starts() {
        assert!(parse_segments("10, a.toml\n10, b.toml\n", Path::new(".")).is_err());
        assert!(parse_segments("", Path::new(".")).is_err());
    }

    #[test]
    fn slam_camera_maps_to_rne_view() {
        // Optical camera looking along SLAM +x (x right = -y, y down = -z), centre (1, 2, 3).
        let r_wc = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]];
        let view = rne_view_from_slam(&r_wc, &[1.0, 2.0, 3.0]);
        assert!((view.translation - Vec3::new(1.0, 3.0, -2.0)).length() < 1e-12);
        // Looks along RNE +x (local -Z), +Y stays up, +X is SLAM -y = RNE +z.
        let forward = view.rotation * Vec3::new(0.0, 0.0, -1.0);
        let up = view.rotation * Vec3::new(0.0, 1.0, 0.0);
        let right = view.rotation * Vec3::new(1.0, 0.0, 0.0);
        assert!((forward - Vec3::new(1.0, 0.0, 0.0)).length() < 1e-12);
        assert!((up - Vec3::new(0.0, 1.0, 0.0)).length() < 1e-12);
        assert!((right - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-12);
    }

    #[test]
    fn pose_and_arc_length_follow_the_path() {
        let points = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 1.0, 0.0),
            Vec3::new(10.0, 1.0, -10.0),
        ];
        let lengths = arc_lengths(&points);
        assert_eq!(lengths, vec![0.0, 10.0, 20.0]);
        let (p, yaw) = pose_at(&points, &lengths, 5.0);
        assert!((p - Vec3::new(5.0, 0.5, 0.0)).length() < 1e-12);
        assert!(yaw.abs() < 1e-12);
        let (p, yaw) = pose_at(&points, &lengths, 15.0);
        assert!((p - Vec3::new(10.0, 1.0, -5.0)).length() < 1e-12);
        assert!((yaw - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert_eq!(pose_at(&points, &lengths, 99.0).0, points[2]);
        let (s, i) = arc_length_near(&points, &lengths, 0, Vec3::new(10.5, 0.0, -3.0));
        assert!((s - 13.0).abs() < 1e-9);
        assert_eq!(i, 1);
    }

    #[test]
    fn idm_brakes_for_a_close_leader_and_accelerates_on_free_road() {
        let idm = Idm {
            desired_speed: 8.0,
            headway_s: 1.2,
            min_gap_m: 4.0,
            max_accel: 1.5,
            comfort_decel: 3.0,
        };
        assert!(idm.accel(5.0, None, 0.0) > 0.0);
        assert!(idm.accel(8.0, None, 0.0).abs() < 1e-12);
        assert!(idm.accel(6.0, Some(8.0), 0.0) < -3.0);
        assert!(idm.accel(6.0, Some(60.0), 6.0) > 0.0);
    }
}
