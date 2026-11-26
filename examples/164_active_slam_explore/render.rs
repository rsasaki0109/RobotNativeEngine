//! Draws both exploration runs side by side, top-down, on the CPU: the SLAM
//! map as it grows (unknown dark, free light, occupied slate), the frontiers
//! the explorer chooses from (orange), the chosen goal and the planned path,
//! the LiDAR returns, and the robot's true trail.
//!
//! Everything map-related is drawn from the SLAM map in its own frame, placed
//! in the world by the start pose, so map error shows as an offset from the
//! true footprints (thin outlines).

use crate::warehouse::FOOTPRINTS;
use crate::Run;
use rne_math::Vec3;
use rne_nav::Pose2d;
use std::fs;
use std::path::{Path, PathBuf};

/// Pixels per meter.
const SCALE: f64 = 30.0;
/// World window drawn, `(min_x, max_x, min_z, max_z)`.
pub(crate) const VIEW: (f64, f64, f64, f64) = (-7.5, 7.5, -5.0, 5.0);
const HEADER: u32 = 46;
const MARGIN: u32 = 6;
const GAP: u32 = 8;
/// Frames the last picture is held for.
const HOLD_FRAMES: usize = 24;
/// Height of the legend strip under the panels.
const LEGEND: u32 = 20;

const UNKNOWN: [u8; 3] = [0x2a, 0x30, 0x3a];
const FREE: [u8; 3] = [0xdf, 0xe3, 0xea];
const OCCUPIED: [u8; 3] = [0x04, 0x05, 0x07];
const OUTLINE: [u8; 3] = [0x8a, 0x96, 0xa8];
const FRONTIER: [u8; 3] = [0xff, 0x9f, 0x1a];
const PATH: [u8; 3] = [0x00, 0xa6, 0xf0];
const GOAL: [u8; 3] = [0xff, 0x2e, 0x7e];
const TRAIL: [u8; 3] = [0x1f, 0x8a, 0x70];
const SCAN: [u8; 3] = [0xe6, 0x39, 0x46];
const ROBOT: [u8; 3] = [0xe6, 0x39, 0x46];
const ESTIMATE: [u8; 3] = [0x14, 0x21, 0x3d];
const BACKGROUND: [u8; 3] = [0x0e, 0x11, 0x16];
const TEXT: [u8; 3] = [0xf1, 0xf3, 0xf6];
const DIM_TEXT: [u8; 3] = [0x9a, 0xa4, 0xb2];

/// A crop of the SLAM map: cell values `-1` / `0` / `100`, row-major.
pub(crate) struct MapCells {
    pub(crate) width: usize,
    pub(crate) height: usize,
    pub(crate) resolution_m: f64,
    /// Map-frame pose of the crop's cell `(0, 0)` corner.
    pub(crate) origin: Pose2d,
    pub(crate) values: Vec<i8>,
}

impl MapCells {
    fn value_at(&self, map_point: Vec3) -> i8 {
        let local = self.origin.inverse_transform_point(map_point);
        let x = (local.x / self.resolution_m).floor();
        let y = (local.y / self.resolution_m).floor();
        if x < 0.0 || y < 0.0 || x >= self.width as f64 || y >= self.height as f64 {
            return -1;
        }
        self.values[y as usize * self.width + x as usize]
    }
}

/// One picture of a run.
pub(crate) struct Snapshot {
    /// The SLAM map (map frame).
    pub(crate) cells: MapCells,
    /// Frontier cell centres (map frame).
    pub(crate) frontier_m: Vec<(f64, f64)>,
    /// LiDAR returns (world).
    pub(crate) scan_m: Vec<(f64, f64)>,
    /// True pose (world).
    pub(crate) truth: Pose2d,
    /// SLAM estimate (map frame).
    pub(crate) estimate: Pose2d,
    /// True poses so far (world).
    pub(crate) trail: Vec<Pose2d>,
    /// Planned path (map frame).
    pub(crate) path_m: Vec<(f64, f64)>,
    /// Chosen goal (map frame).
    pub(crate) goal: Option<Pose2d>,
    pub(crate) time_s: f64,
    pub(crate) distance_m: f64,
    pub(crate) coverage: f64,
}

struct Canvas {
    width: u32,
    height: u32,
    rgb: Vec<u8>,
    /// Drawing is limited to this rectangle, `(x0, y0, x1, y1)` exclusive.
    clip: (i64, i64, i64, i64),
}

impl Canvas {
    fn new(width: u32, height: u32) -> Self {
        let mut rgb = vec![0; (width * height * 3) as usize];
        for pixel in rgb.chunks_exact_mut(3) {
            pixel.copy_from_slice(&BACKGROUND);
        }
        Self {
            width,
            height,
            rgb,
            clip: (0, 0, i64::from(width), i64::from(height)),
        }
    }

    fn inside(&self, x: i64, y: i64) -> bool {
        x >= self.clip.0 && y >= self.clip.1 && x < self.clip.2 && y < self.clip.3
    }

    fn put(&mut self, x: i64, y: i64, color: [u8; 3]) {
        if !self.inside(x, y) {
            return;
        }
        let offset = ((y as u32 * self.width + x as u32) * 3) as usize;
        self.rgb[offset..offset + 3].copy_from_slice(&color);
    }

    fn blend(&mut self, x: i64, y: i64, color: [u8; 3], alpha: f64) {
        if !self.inside(x, y) {
            return;
        }
        let offset = ((y as u32 * self.width + x as u32) * 3) as usize;
        for (pixel, new) in self.rgb[offset..offset + 3].iter_mut().zip(color) {
            let old = f64::from(*pixel);
            *pixel = (old + (f64::from(new) - old) * alpha).round() as u8;
        }
    }

    fn disc(&mut self, cx: f64, cy: f64, radius: f64, color: [u8; 3]) {
        let r = radius.ceil() as i64;
        for dy in -r..=r {
            for dx in -r..=r {
                let (px, py) = (cx.floor() as i64 + dx, cy.floor() as i64 + dy);
                let d = ((px as f64 + 0.5 - cx).powi(2) + (py as f64 + 0.5 - cy).powi(2)).sqrt();
                if d <= radius {
                    self.put(px, py, color);
                }
            }
        }
    }

    fn ring(&mut self, cx: f64, cy: f64, radius: f64, width: f64, color: [u8; 3]) {
        let r = (radius + width).ceil() as i64;
        for dy in -r..=r {
            for dx in -r..=r {
                let (px, py) = (cx.floor() as i64 + dx, cy.floor() as i64 + dy);
                let d = ((px as f64 + 0.5 - cx).powi(2) + (py as f64 + 0.5 - cy).powi(2)).sqrt();
                if (d - radius).abs() <= 0.5 * width {
                    self.put(px, py, color);
                }
            }
        }
    }

    fn line(&mut self, a: (f64, f64), b: (f64, f64), width: f64, color: [u8; 3]) {
        let length = (b.0 - a.0).hypot(b.1 - a.1);
        let steps = (length * 2.0).ceil().max(1.0) as usize;
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let (x, y) = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
            if width <= 1.0 {
                self.put(x.floor() as i64, y.floor() as i64, color);
            } else {
                self.disc(x, y, 0.5 * width, color);
            }
        }
    }

    fn text(&mut self, x: u32, y: u32, scale: u32, color: [u8; 3], text: &str) {
        let mut pen = x;
        for character in text.chars() {
            for (row, bits) in glyph(character).iter().enumerate() {
                for column in 0..5u32 {
                    if bits & (1 << (4 - column)) == 0 {
                        continue;
                    }
                    for dy in 0..scale {
                        for dx in 0..scale {
                            self.put(
                                i64::from(pen + column * scale + dx),
                                i64::from(y + row as u32 * scale + dy),
                                color,
                            );
                        }
                    }
                }
            }
            pen += 6 * scale;
        }
    }

    fn rgba(&self) -> Vec<u8> {
        self.rgb
            .chunks_exact(3)
            .flat_map(|p| [p[0], p[1], p[2], 255])
            .collect()
    }
}

fn panel_size() -> (u32, u32) {
    let (min_x, max_x, min_z, max_z) = VIEW;
    (
        ((max_x - min_x) * SCALE).round() as u32 + 2 * MARGIN,
        ((max_z - min_z) * SCALE).round() as u32 + HEADER + MARGIN,
    )
}

/// Pixel of a world point inside a panel whose top-left corner is `left`.
fn to_pixel(left: u32, x: f64, z: f64) -> (f64, f64) {
    let (min_x, _, _, max_z) = VIEW;
    (
        f64::from(left + MARGIN) + (x - min_x) * SCALE,
        f64::from(HEADER) + (max_z - z) * SCALE,
    )
}

fn draw_panel(canvas: &mut Canvas, left: u32, name: &str, snap: &Snapshot, world_from_map: Pose2d) {
    let (width, height) = panel_size();
    let (min_x, _, _, max_z) = VIEW;
    canvas.clip = (
        i64::from(left + MARGIN),
        i64::from(HEADER),
        i64::from(left + width - MARGIN),
        i64::from(height - MARGIN),
    );
    let to_world = |p: (f64, f64)| {
        let w = world_from_map.transform_point(Vec3::new(p.0, p.1, 0.0));
        (w.x, w.y)
    };

    // The map.
    for py in HEADER..height - MARGIN {
        for px in left + MARGIN..left + width - MARGIN {
            let x = min_x + (f64::from(px - left - MARGIN) + 0.5) / SCALE;
            let z = max_z - (f64::from(py - HEADER) + 0.5) / SCALE;
            let map_point = world_from_map.inverse_transform_point(Vec3::new(x, z, 0.0));
            let color = match snap.cells.value_at(map_point) {
                0 => FREE,
                100 => OCCUPIED,
                _ => UNKNOWN,
            };
            canvas.put(i64::from(px), i64::from(py), color);
        }
    }
    // The true footprints, as thin outlines.
    for f in &FOOTPRINTS {
        let corners = [
            (f.x - f.half_x, f.z - f.half_z),
            (f.x + f.half_x, f.z - f.half_z),
            (f.x + f.half_x, f.z + f.half_z),
            (f.x - f.half_x, f.z + f.half_z),
        ];
        for i in 0..4 {
            let a = corners[i];
            let b = corners[(i + 1) % 4];
            let (pa, pb) = (to_pixel(left, a.0, a.1), to_pixel(left, b.0, b.1));
            let length = (pb.0 - pa.0).hypot(pb.1 - pa.1);
            let steps = (length * 2.0).ceil().max(1.0) as usize;
            for s in 0..=steps {
                let t = s as f64 / steps as f64;
                canvas.blend(
                    (pa.0 + (pb.0 - pa.0) * t).floor() as i64,
                    (pa.1 + (pb.1 - pa.1) * t).floor() as i64,
                    OUTLINE,
                    0.6,
                );
            }
        }
    }
    // Frontiers.
    for &cell in &snap.frontier_m {
        let (x, z) = to_world(cell);
        let (px, py) = to_pixel(left, x, z);
        canvas.disc(px, py, 1.6, FRONTIER);
    }
    // The true trail.
    for pair in snap.trail.windows(2) {
        let a = to_pixel(left, pair[0].x_m, pair[0].y_m);
        let b = to_pixel(left, pair[1].x_m, pair[1].y_m);
        canvas.line(a, b, 2.0, TRAIL);
    }
    // The planned path and the goal.
    for pair in snap.path_m.windows(2) {
        let (a, b) = (to_world(pair[0]), to_world(pair[1]));
        canvas.line(
            to_pixel(left, a.0, a.1),
            to_pixel(left, b.0, b.1),
            2.0,
            PATH,
        );
    }
    if let Some(goal) = snap.goal {
        let (x, z) = to_world((goal.x_m, goal.y_m));
        let (px, py) = to_pixel(left, x, z);
        canvas.ring(px, py, 7.0, 3.0, GOAL);
        canvas.disc(px, py, 2.0, GOAL);
    }
    // LiDAR returns.
    for &(x, z) in &snap.scan_m {
        let (px, py) = to_pixel(left, x, z);
        canvas.disc(px, py, 1.2, SCAN);
    }
    // The SLAM estimate (ring) and the true robot (disc with heading).
    let estimate = world_from_map.compose(snap.estimate);
    let (ex, ey) = to_pixel(left, estimate.x_m, estimate.y_m);
    canvas.ring(ex, ey, crate::ROBOT_RADIUS_M * SCALE + 2.0, 2.0, ESTIMATE);
    let (rx, ry) = to_pixel(left, snap.truth.x_m, snap.truth.y_m);
    canvas.disc(rx, ry, crate::ROBOT_RADIUS_M * SCALE, ROBOT);
    let nose = to_pixel(
        left,
        snap.truth.x_m + 0.3 * snap.truth.yaw_rad.cos(),
        snap.truth.y_m + 0.3 * snap.truth.yaw_rad.sin(),
    );
    canvas.line((rx, ry), nose, 2.0, TEXT);

    // Header.
    canvas.clip = (0, 0, i64::from(canvas.width), i64::from(canvas.height));
    canvas.text(left + MARGIN, 8, 2, TEXT, name);
    let stats = format!(
        "T {:>3.0}S   DRIVEN {:>5.1}M   MAPPED {:>3.0}%",
        snap.time_s,
        snap.distance_m,
        100.0 * snap.coverage
    );
    canvas.text(left + MARGIN, 28, 1, DIM_TEXT, &stats);
}

/// One line of coloured keys under the panels.
fn draw_legend(canvas: &mut Canvas, y: u32) {
    let keys: [(&str, [u8; 3]); 7] = [
        ("FRONTIER", FRONTIER),
        ("GOAL", GOAL),
        ("PATH", PATH),
        ("TRAIL", TRAIL),
        ("LIDAR", SCAN),
        ("UNKNOWN", UNKNOWN),
        ("WALL OUTLINE: TRUTH", OUTLINE),
    ];
    let mut x = MARGIN;
    for (label, color) in keys {
        for dy in 0..9 {
            for dx in 0..9 {
                canvas.put(i64::from(x + dx), i64::from(y + dy), color);
            }
        }
        canvas.text(x + 14, y + 1, 1, DIM_TEXT, label);
        x += 14 + 6 * label.len() as u32 + 18;
    }
}

/// Renders both runs into `frames_dir` and encodes `gif_path`.
pub(crate) fn render_gif(
    runs: &[&Run],
    world_from_map: Pose2d,
    frames_dir: &Path,
    gif_path: &Path,
) -> std::io::Result<()> {
    let _ = fs::remove_dir_all(frames_dir);
    fs::create_dir_all(frames_dir)?;
    if let Some(parent) = gif_path.parent() {
        fs::create_dir_all(parent)?;
    }
    let (panel_w, panel_h) = panel_size();
    let width = panel_w * runs.len() as u32 + GAP * (runs.len() as u32 - 1);
    let count = runs.iter().map(|r| r.frames.len()).max().unwrap_or(0) + HOLD_FRAMES;
    for index in 0..count {
        let mut canvas = Canvas::new(width, panel_h + LEGEND);
        for (column, run) in runs.iter().enumerate() {
            let snap = &run.frames[index.min(run.frames.len() - 1)];
            let left = column as u32 * (panel_w + GAP);
            let finished = index >= run.frames.len() - 1;
            let name = if finished {
                format!("{}  DONE", run.name)
            } else {
                run.name.to_string()
            };
            draw_panel(&mut canvas, left, &name, snap, world_from_map);
        }
        draw_legend(&mut canvas, panel_h + 4);
        write_png(
            &frames_dir.join(format!("frame-{index:04}.png")),
            width,
            panel_h + LEGEND,
            &canvas.rgba(),
        )?;
    }
    let script = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../tools/encode_gif.py");
    let status = std::process::Command::new("python3")
        .arg(script)
        .arg(frames_dir)
        .arg(gif_path)
        .args(["--fps", "12", "--colors", "64"])
        .status()?;
    if !status.success() {
        return Err(std::io::Error::other("encode_gif.py failed"));
    }
    Ok(())
}

fn write_png(path: &Path, width: u32, height: u32, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = png::Encoder::new(file, width, height);
    encoder.set_color(png::ColorType::Rgba);
    encoder.set_depth(png::BitDepth::Eight);
    encoder
        .write_header()?
        .write_image_data(rgba)
        .map_err(std::io::Error::other)
}

/// 5x7 glyphs, rows as 5-bit masks with the leftmost pixel in bit 4.
/// Characters without a glyph are blank.
fn glyph(character: char) -> [u8; 7] {
    match character {
        '0' => [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
        '1' => [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
        '2' => [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
        '3' => [0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E],
        '4' => [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
        '5' => [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
        '6' => [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
        '7' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
        '9' => [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
        'A' => [0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'B' => [0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E],
        'C' => [0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E],
        'D' => [0x1C, 0x12, 0x11, 0x11, 0x11, 0x12, 0x1C],
        'E' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F],
        'F' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10],
        'G' => [0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F],
        'H' => [0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'I' => [0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E],
        'J' => [0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C],
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
        'M' => [0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11],
        'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'P' => [0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10],
        'Q' => [0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D],
        'R' => [0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11],
        'S' => [0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E],
        'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A],
        'X' => [0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11],
        'Y' => [0x11, 0x11, 0x11, 0x0A, 0x04, 0x04, 0x04],
        'Z' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F],
        '%' => [0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03],
        '.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C],
        ':' => [0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00],
        '-' => [0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00],
        '/' => [0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10],
        _ => [0x00; 7],
    }
}

#[cfg(test)]
mod tests {
    use super::glyph;

    #[test]
    fn font_covers_the_rendered_charset() {
        for character in "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ%.:-/".chars() {
            assert!(
                glyph(character).iter().any(|row| *row != 0),
                "missing glyph for {character:?}"
            );
        }
    }
}
