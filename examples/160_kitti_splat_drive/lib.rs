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
}
