//! A JSON trace of the run for the 3D renderer (`151_gnss_lidar_navigation_render`):
//! the scene boxes, and per recorded instant the robot, its antennas, every
//! tracked satellite signal's ray-traced path as a 3D polyline, the LiDAR
//! returns, the localization source and the GNSS solution.

use crate::city::{City, Layer, BASE_ENU_M, OFFICE};
use crate::frames::FrameRecord;
use crate::gnss::{SolutionLabel, BODY_HEIGHT_M, LEVER_ARMS_M};
use crate::RunLog;
use rne_gnss::{trace_satellite, GnssScene, PathKind, SignalPath, TraceConfig};
use rne_math::Vec3;
use rne_slam::LocalizationSource;
use serde::Serialize;
use std::path::Path;

/// How far rays are drawn toward the sky, in meters.
const SKY_REACH_M: f64 = 90.0;

#[derive(Serialize)]
struct TraceBox {
    min: [f64; 3],
    max: [f64; 3],
    kind: &'static str,
    mapped: bool,
}

#[derive(Serialize)]
struct TraceCanopy {
    min: [f64; 3],
    max: [f64; 3],
}

#[derive(Serialize)]
struct TraceVehicle {
    kind: &'static str,
    min: [f64; 3],
    max: [f64; 3],
    color: usize,
    heading: &'static str,
    braking: bool,
}

#[derive(Serialize)]
struct TracePatch {
    rect: [f64; 4],
    kind: &'static str,
}

#[derive(Serialize)]
struct TraceRay {
    kind: &'static str,
    points: Vec<[f64; 3]>,
}

#[derive(Serialize)]
struct TraceSatellite {
    id: String,
    azimuth_rad: f64,
    elevation_rad: f64,
    path: &'static str,
}

#[derive(Serialize)]
struct TraceFrame {
    t: f64,
    truth: [f64; 3],
    estimate: [f64; 3],
    source: &'static str,
    solution: &'static str,
    heading_fixed: bool,
    error_m: f64,
    antennas: Vec<[f64; 3]>,
    rays: Vec<TraceRay>,
    vehicles: Vec<TraceVehicle>,
    main_green: bool,
    cross_green: bool,
    satellites: Vec<TraceSatellite>,
    lidar: Vec<[f64; 2]>,
    trail_samples: usize,
}

#[derive(Serialize)]
struct TraceSample {
    truth: [f64; 2],
    estimate: [f64; 2],
    source: &'static str,
}

#[derive(Serialize)]
struct TraceFile {
    format: &'static str,
    body_height_m: f64,
    lever_arms_m: Vec<[f64; 3]>,
    base_m: [f64; 3],
    office: [f64; 4],
    boxes: Vec<TraceBox>,
    canopies: Vec<TraceCanopy>,
    ground: Vec<TracePatch>,
    samples: Vec<TraceSample>,
    frames: Vec<TraceFrame>,
}

fn source_name(source: LocalizationSource) -> &'static str {
    match source {
        LocalizationSource::Gnss => "gnss",
        LocalizationSource::Lidar => "lidar",
        LocalizationSource::DeadReckoning => "dead_reckoning",
    }
}

fn path_name(path: SignalPath) -> &'static str {
    match path {
        SignalPath::LineOfSight | SignalPath::LineOfSightMultipath => "los",
        SignalPath::NonLineOfSight => "nlos",
        SignalPath::Diffracted => "diffracted",
        SignalPath::Blocked | SignalPath::TooWeak => "blocked",
        SignalPath::BelowMask => "below_mask",
    }
}

/// Every tracked primary signal as a polyline from the antenna to the sky.
fn rays(scene: &GnssScene, trace: &TraceConfig, frame: &FrameRecord) -> Vec<TraceRay> {
    let Some(epoch) = &frame.epoch else {
        return Vec::new();
    };
    let antenna = epoch.antenna_enu_m;
    let mut out = Vec::new();
    for o in &epoch.observations {
        let Some(primary) = o.primary() else {
            continue;
        };
        if !primary.path.is_tracked() {
            continue;
        }
        let (sin_az, cos_az) = o.azimuth_rad.sin_cos();
        let (sin_el, cos_el) = o.elevation_rad.sin_cos();
        let u = Vec3::new(sin_az * cos_el, cos_az * cos_el, sin_el);
        let mut points = vec![antenna];
        let kind = path_name(primary.path);
        if kind != "los" {
            // Re-trace to recover every interaction point of the tracked path.
            let propagation = trace_satellite(scene, antenna, u, trace);
            let Some(path) = propagation
                .paths
                .iter()
                .filter(|p| p.kind != PathKind::Direct)
                .min_by(|a, b| {
                    (a.excess_delay_m - primary.excess_path_m)
                        .abs()
                        .total_cmp(&(b.excess_delay_m - primary.excess_path_m).abs())
                })
            else {
                continue;
            };
            points.extend(path.points_m.iter().copied());
        }
        let last = *points.last().unwrap_or(&antenna);
        points.push(last + u * SKY_REACH_M);
        out.push(TraceRay {
            kind,
            points: points.iter().map(|p| p.to_array()).collect(),
        });
    }
    out
}

/// Writes the trace as JSON.
pub(crate) fn write_trace(
    path: &Path,
    city: &City,
    trace: &TraceConfig,
    log: &RunLog,
    frames: &[FrameRecord],
) -> std::io::Result<()> {
    let file = TraceFile {
        format: "rne-151-gnss-lidar-trace/1",
        body_height_m: BODY_HEIGHT_M,
        lever_arms_m: LEVER_ARMS_M.iter().map(|v| v.to_array()).collect(),
        base_m: BASE_ENU_M.to_array(),
        office: OFFICE,
        boxes: city
            .items
            .iter()
            .map(|item| TraceBox {
                min: item.scene_box.min_m.to_array(),
                max: item.scene_box.max_m.to_array(),
                kind: item.kind,
                mapped: item.layer == Layer::Building,
            })
            .collect(),
        canopies: city
            .canopies
            .iter()
            .map(|c| TraceCanopy {
                min: c.min_m.to_array(),
                max: c.max_m.to_array(),
            })
            .collect(),
        ground: city
            .ground
            .iter()
            .map(|g| TracePatch {
                rect: g.rect,
                kind: g.kind,
            })
            .collect(),
        samples: log
            .samples
            .iter()
            .map(|s| TraceSample {
                truth: [s.truth.x_m, s.truth.y_m],
                estimate: [s.hybrid.x_m, s.hybrid.y_m],
                source: source_name(s.source),
            })
            .collect(),
        frames: frames
            .iter()
            .map(|f| TraceFrame {
                t: f.t,
                truth: [f.truth.x_m, f.truth.y_m, f.truth.yaw_rad],
                estimate: [f.estimate.x_m, f.estimate.y_m, f.estimate.yaw_rad],
                source: source_name(f.source),
                solution: match f.label {
                    Some(SolutionLabel::Fixed) => "fixed",
                    Some(SolutionLabel::Float) => "float",
                    Some(SolutionLabel::Single) => "single",
                    _ => "none",
                },
                heading_fixed: f.heading_fixed,
                error_m: (f.estimate.x_m - f.truth.x_m).hypot(f.estimate.y_m - f.truth.y_m),
                antennas: {
                    let (sin, cos) = f.truth.yaw_rad.sin_cos();
                    LEVER_ARMS_M
                        .iter()
                        .map(|l| {
                            [
                                f.truth.x_m + cos * l.x - sin * l.y,
                                f.truth.y_m + sin * l.x + cos * l.y,
                                BODY_HEIGHT_M + l.z,
                            ]
                        })
                        .collect()
                },
                rays: rays(&city.world_scene_with(&f.vehicles), trace, f),
                vehicles: f
                    .vehicles
                    .iter()
                    .map(|v| {
                        let b = v.scene_box();
                        TraceVehicle {
                            kind: v.kind,
                            min: b.min_m.to_array(),
                            max: b.max_m.to_array(),
                            color: v.color,
                            heading: v.heading.name(),
                            braking: !v.parked && v.speed_m_s < 0.5 * v.max_speed_m_s,
                        }
                    })
                    .collect(),
                main_green: f.signals.main_green,
                cross_green: f.signals.cross_green,
                satellites: f
                    .epoch
                    .iter()
                    .flat_map(|e| &e.observations)
                    .filter(|o| o.elevation_rad > 0.0)
                    .map(|o| TraceSatellite {
                        id: o.id.to_string(),
                        azimuth_rad: o.azimuth_rad,
                        elevation_rad: o.elevation_rad,
                        path: path_name(o.path()),
                    })
                    .collect(),
                lidar: f.scan_points.iter().map(|&(x, y)| [x, y]).collect(),
                trail_samples: f.sample_count,
            })
            .collect(),
    };
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let json = serde_json::to_vec(&file).map_err(std::io::Error::other)?;
    std::fs::write(path, json)
}
