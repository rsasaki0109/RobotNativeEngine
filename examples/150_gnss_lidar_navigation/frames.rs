//! Animation frames: one SVG per recorded instant, showing the robot, its
//! satellites' ray-traced paths, the LiDAR scan, trails, the localization
//! source and the sky at the robot. `scripts/svg_frames_to_gif.py` turns the
//! frames into a GIF.

use crate::city::{Patch, Signals, Vehicle, BASE_ENU_M, OFFICE};
use crate::gnss::SolutionLabel;
use crate::{
    draw_buildings, draw_ground, source_index, to_px, RunLog, MAP_VIEW, PX_PER_M, SOURCE_COLORS,
};
use rne_gnss::{GnssEpoch, GnssScene, SignalPath};
use rne_math::Vec3;
use rne_nav::Pose2d;
use rne_slam::LocalizationSource;
use std::f64::consts::{FRAC_PI_2, TAU};
use std::fmt::Write as _;
use std::path::Path;

/// Seconds of simulation between recorded instants.
pub(crate) const RECORD_PERIOD_S: f64 = 0.5;
/// Recorded instants per SVG frame (one frame per 1.5 s).
const SVG_STRIDE: usize = 3;
/// Vertical shift of the map below the header bar, in pixels.
const MAP_SHIFT_PX: f64 = 20.0;

/// One recorded instant.
#[derive(Clone, Debug)]
pub(crate) struct FrameRecord {
    pub(crate) t: f64,
    pub(crate) sample_count: usize,
    pub(crate) truth: Pose2d,
    pub(crate) estimate: Pose2d,
    pub(crate) source: LocalizationSource,
    pub(crate) label: Option<SolutionLabel>,
    pub(crate) heading_fixed: bool,
    pub(crate) epoch: Option<GnssEpoch>,
    pub(crate) scan_points: Vec<(f64, f64)>,
    pub(crate) vehicles: Vec<Vehicle>,
    pub(crate) signals: Signals,
}

fn px(x: f64, y: f64) -> (f64, f64) {
    let (u, v) = to_px(x, y);
    (u, v + MAP_SHIFT_PX)
}

/// Writes every frame as `frame_NNNN.svg` into `dir`.
pub(crate) fn write_frames(
    dir: &Path,
    scene: &GnssScene,
    ground: &[Patch],
    log: &RunLog,
    frames: &[FrameRecord],
) -> std::io::Result<usize> {
    std::fs::create_dir_all(dir)?;
    for (index, frame) in frames.iter().step_by(SVG_STRIDE).enumerate() {
        std::fs::write(
            dir.join(format!("frame_{index:04}.svg")),
            render_frame(scene, ground, log, frame),
        )?;
    }
    Ok(frames.len().div_ceil(SVG_STRIDE))
}

fn render_frame(scene: &GnssScene, ground: &[Patch], log: &RunLog, frame: &FrameRecord) -> String {
    let [x0, y0, x1, y1] = MAP_VIEW;
    let width = (x1 - x0) * PX_PER_M + 20.0;
    let height = (y1 - y0) * PX_PER_M + 50.0 + MAP_SHIFT_PX + 10.0;
    let mut svg = String::new();
    let _ = writeln!(
        svg,
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width:.0}" height="{height:.0}" viewBox="0 0 {width:.0} {height:.0}" font-family="Helvetica, Arial, sans-serif" font-size="13"><rect width="100%" height="100%" fill="#ffffff"/>"##
    );
    // The map, shifted below the header.
    let _ = writeln!(svg, r##"<g transform="translate(0,{MAP_SHIFT_PX})">"##);
    draw_ground(&mut svg, ground);
    draw_buildings(&mut svg, scene);
    // Vehicles.
    for v in &frame.vehicles {
        let b = v.scene_box();
        let (u0, v1) = to_px(b.min_m.x, b.min_m.y);
        let (u1, v0) = to_px(b.max_m.x, b.max_m.y);
        let fill = if v.kind == "bus" {
            "#1f7a5a"
        } else {
            [
                "#c81e24", "#eeeeee", "#22262c", "#2a5aaa", "#a0a6ac", "#dcaa28",
            ][v.color % 6]
        };
        let _ = write!(
            svg,
            r##"<rect x="{u0:.1}" y="{v0:.1}" width="{:.1}" height="{:.1}" fill="{fill}" stroke="#252525" stroke-width="0.5"/>"##,
            u1 - u0,
            v1 - v0
        );
    }
    svg.push_str("</g>\n");
    let (ox, oy) = px(OFFICE[0], OFFICE[3]);
    let _ = writeln!(
        svg,
        r##"<text x="{ox:.1}" y="{:.1}" fill="#252525">office (roofed)</text>"##,
        oy - 6.0
    );
    let (bx, by) = px(BASE_ENU_M.x, BASE_ENU_M.y);
    let (lx, ly) = px(BASE_ENU_M.x, 45.5);
    let _ = writeln!(
        svg,
        r##"<path d="M{bx:.1},{:.1} l4,8 l-8,0 z" fill="#6a51a3"/><text x="{lx:.1}" y="{ly:.1}" text-anchor="middle" fill="#3f007d" font-size="11">RTK base on the rooftop ▼</text>"##,
        by - 14.0
    );

    // Trails up to this instant.
    let samples = &log.samples[..frame.sample_count.min(log.samples.len())];
    let mut truth_trail = String::new();
    for s in samples.iter().step_by(4) {
        let (u, v) = px(s.truth.x_m, s.truth.y_m);
        let _ = write!(truth_trail, "{u:.1},{v:.1} ");
    }
    let _ = writeln!(
        svg,
        r##"<polyline points="{truth_trail}" fill="none" stroke="#000000" stroke-width="2.2" stroke-opacity="0.5"/>"##
    );
    let mut run = String::new();
    let mut run_source: Option<LocalizationSource> = None;
    let flush = |svg: &mut String, run: &str, source: LocalizationSource| {
        let _ = writeln!(
            svg,
            r##"<polyline points="{run}" fill="none" stroke="{}" stroke-width="1.6"/>"##,
            SOURCE_COLORS[source_index(source)]
        );
    };
    for s in samples.iter().step_by(4) {
        let (u, v) = px(s.hybrid.x_m, s.hybrid.y_m);
        if run_source.is_some_and(|src| src != s.source) {
            flush(&mut svg, &run, run_source.unwrap_or(s.source));
            run = run
                .rsplit(' ')
                .nth(1)
                .map(|p| format!("{p} "))
                .unwrap_or_default();
        }
        run_source = Some(s.source);
        let _ = write!(run, "{u:.1},{v:.1} ");
    }
    if let Some(source) = run_source {
        flush(&mut svg, &run, source);
    }

    // LiDAR returns.
    for &(x, y) in frame.scan_points.iter().step_by(2) {
        let (u, v) = px(x, y);
        let _ = write!(
            svg,
            r##"<circle cx="{u:.1}" cy="{v:.1}" r="1.1" fill="#00a6d6"/>"##
        );
    }

    // Satellite paths from the rear antenna.
    if let Some(epoch) = &frame.epoch {
        let antenna = epoch.antenna_enu_m;
        let (au, av) = px(antenna.x, antenna.y);
        for o in &epoch.observations {
            let Some(primary) = o.primary() else {
                continue;
            };
            let reach_m = 70.0 * o.elevation_rad.cos();
            let (dx, dy) = (o.azimuth_rad.sin(), o.azimuth_rad.cos());
            match primary.path {
                SignalPath::LineOfSight | SignalPath::LineOfSightMultipath => {
                    let (u, v) = px(antenna.x + dx * reach_m, antenna.y + dy * reach_m);
                    let _ = write!(
                        svg,
                        r##"<line x1="{au:.1}" y1="{av:.1}" x2="{u:.1}" y2="{v:.1}" stroke="#1a9850" stroke-width="1.2" stroke-opacity="0.75"/>"##
                    );
                }
                SignalPath::NonLineOfSight | SignalPath::Diffracted => {
                    let Some(point) = primary.interaction_point_m else {
                        continue;
                    };
                    let color = if primary.path == SignalPath::Diffracted {
                        "#807dba"
                    } else {
                        "#d7301f"
                    };
                    let (pu, pv) = px(point.x, point.y);
                    let (u, v) = px(point.x + dx * 0.5 * reach_m, point.y + dy * 0.5 * reach_m);
                    let _ = write!(
                        svg,
                        r##"<line x1="{au:.1}" y1="{av:.1}" x2="{pu:.1}" y2="{pv:.1}" stroke="{color}" stroke-width="1.4" stroke-dasharray="4,2"/><line x1="{pu:.1}" y1="{pv:.1}" x2="{u:.1}" y2="{v:.1}" stroke="{color}" stroke-width="1.0" stroke-opacity="0.6"/><circle cx="{pu:.1}" cy="{pv:.1}" r="2.2" fill="{color}"/>"##
                    );
                }
                _ => {}
            }
        }
    }

    // The robot: true pose (body and antennas), estimate as a ring.
    draw_robot(&mut svg, frame.truth);
    let (eu, ev) = px(frame.estimate.x_m, frame.estimate.y_m);
    let _ = writeln!(
        svg,
        r##"<circle cx="{eu:.1}" cy="{ev:.1}" r="9" fill="none" stroke="{}" stroke-width="2.4"/>"##,
        SOURCE_COLORS[source_index(frame.source)]
    );

    if let Some(epoch) = &frame.epoch {
        draw_sky_inset(&mut svg, scene, epoch);
    }
    draw_header(&mut svg, width, frame);
    draw_legend(&mut svg, height);
    svg.push_str("</svg>\n");
    svg
}

fn draw_robot(svg: &mut String, pose: Pose2d) {
    let (sin, cos) = pose.yaw_rad.sin_cos();
    let corner = |fx: f64, fy: f64| {
        px(
            pose.x_m + cos * fx - sin * fy,
            pose.y_m + sin * fx + cos * fy,
        )
    };
    // Drawn 2.5x oversize so it reads at map scale.
    let (a, b, c) = (corner(2.0, 0.0), corner(-1.5, 1.3), corner(-1.5, -1.3));
    let _ = writeln!(
        svg,
        r##"<polygon points="{:.1},{:.1} {:.1},{:.1} {:.1},{:.1}" fill="#252525" stroke="#ffffff" stroke-width="1"/>"##,
        a.0, a.1, b.0, b.1, c.0, c.1
    );
}

fn draw_header(svg: &mut String, width: f64, frame: &FrameRecord) {
    let (source_text, source_color) = match frame.source {
        LocalizationSource::Gnss => ("GNSS", SOURCE_COLORS[0]),
        LocalizationSource::Lidar => ("LiDAR (AMCL)", SOURCE_COLORS[1]),
        LocalizationSource::DeadReckoning => ("dead reckoning", SOURCE_COLORS[2]),
    };
    let (rtk_text, rtk_color) = match frame.label {
        Some(SolutionLabel::Fixed) => ("RTK FIX", "#1a9850"),
        Some(SolutionLabel::Float) => ("RTK FLOAT", "#fdae61"),
        Some(SolutionLabel::Single) => ("SINGLE", "#d7301f"),
        _ => ("NO FIX", "#737373"),
    };
    let error = (frame.estimate.x_m - frame.truth.x_m).hypot(frame.estimate.y_m - frame.truth.y_m);
    let counts = frame.epoch.as_ref().map_or([0; 3], |e| {
        [
            e.count(SignalPath::LineOfSight) + e.count(SignalPath::LineOfSightMultipath),
            e.count(SignalPath::NonLineOfSight),
            e.count(SignalPath::Diffracted),
        ]
    });
    let _ = writeln!(
        svg,
        r##"<rect x="0" y="0" width="{width:.0}" height="38" fill="#f0f0f0"/><text x="12" y="25" font-size="16" font-weight="bold">t = {:5.1} s</text><rect x="100" y="8" width="190" height="22" rx="4" fill="{source_color}"/><text x="195" y="24" fill="#ffffff" text-anchor="middle" font-weight="bold">localizing: {source_text}</text><rect x="300" y="8" width="96" height="22" rx="4" fill="{rtk_color}"/><text x="348" y="24" fill="#ffffff" text-anchor="middle" font-weight="bold">{rtk_text}</text><text x="408" y="24">{}  error {:.2} m   satellites: <tspan fill="#1a9850" font-weight="bold">{} LOS</tspan>  <tspan fill="#d7301f" font-weight="bold">{} NLOS</tspan>  <tspan fill="#807dba" font-weight="bold">{} diffracted</tspan></text>"##,
        frame.t,
        if frame.heading_fixed {
            "2-antenna heading ✓"
        } else {
            "2-antenna heading –"
        },
        error,
        counts[0],
        counts[1],
        counts[2]
    );
}

fn draw_legend(svg: &mut String, height: f64) {
    let items = [
        ("#1a9850", "line of sight"),
        ("#d7301f", "NLOS reflection"),
        ("#807dba", "diffraction"),
        ("#00a6d6", "LiDAR returns"),
        ("#000000", "truth"),
        (SOURCE_COLORS[0], "estimate: GNSS"),
        (SOURCE_COLORS[1], "estimate: LiDAR"),
        (SOURCE_COLORS[2], "estimate: dead reckoning"),
    ];
    let y = height - 12.0;
    let mut x = 12.0;
    for (color, label) in items {
        let _ = write!(
            svg,
            r##"<rect x="{x:.0}" y="{:.0}" width="14" height="4" fill="{color}"/><text x="{:.0}" y="{:.0}" font-size="12">{label}</text>"##,
            y - 4.0,
            x + 18.0,
            y + 1.0
        );
        x += 18.0 + 6.3 * label.len() as f64 + 14.0;
    }
}

/// A small sky plot of the rear antenna's satellites over the skyline.
fn draw_sky_inset(svg: &mut String, scene: &GnssScene, epoch: &GnssEpoch) {
    let (cx, cy, radius) = (92.0, 150.0, 74.0);
    let polar = |az: f64, el: f64| {
        let r = radius * (1.0 - el / FRAC_PI_2);
        (cx + r * az.sin(), cy - r * az.cos())
    };
    let antenna = epoch.antenna_enu_m;
    let mut skyline = String::new();
    for step in 0..=90 {
        let az = TAU * step as f64 / 90.0;
        let dir = |el: f64| Vec3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin());
        let (mut lo, mut hi) = (0.0, FRAC_PI_2);
        if scene.occluded(antenna, dir(0.001), f64::INFINITY) {
            for _ in 0..16 {
                let mid = 0.5 * (lo + hi);
                if scene.occluded(antenna, dir(mid), f64::INFINITY) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
        }
        let (u, v) = polar(az, lo);
        let _ = write!(skyline, "{u:.1},{v:.1} ");
    }
    let _ = writeln!(
        svg,
        r##"<circle cx="{cx}" cy="{cy}" r="{:.0}" fill="#ffffff" fill-opacity="0.9" stroke="#969696"/><circle cx="{cx}" cy="{cy}" r="{radius}" fill="#bdbdbd"/><polygon points="{skyline}" fill="#f7fbff" stroke="#636363" stroke-width="0.6"/><circle cx="{cx}" cy="{cy}" r="{:.1}" fill="none" stroke="#969696" stroke-dasharray="2,3"/><text x="{cx}" y="{:.0}" text-anchor="middle" font-size="11" font-weight="bold">N</text><text x="{cx}" y="{:.0}" text-anchor="middle" font-size="11">sky at the robot</text>"##,
        radius + 14.0,
        radius * (1.0 - 60.0 / 90.0),
        cy - radius - 3.0,
        cy + radius + 13.0
    );
    for o in &epoch.observations {
        if o.elevation_rad < 0.0 {
            continue;
        }
        let color = match o.path() {
            SignalPath::LineOfSight | SignalPath::LineOfSightMultipath => "#1a9850",
            SignalPath::NonLineOfSight => "#d7301f",
            SignalPath::Diffracted => "#807dba",
            _ => "#737373",
        };
        let (u, v) = polar(o.azimuth_rad, o.elevation_rad);
        let _ = write!(
            svg,
            r##"<circle cx="{u:.1}" cy="{v:.1}" r="4.5" fill="{color}" stroke="#252525" stroke-width="0.6"/>"##
        );
    }
}
