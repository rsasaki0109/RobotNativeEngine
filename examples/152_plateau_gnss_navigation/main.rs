//! Ray-traced GNSS in real Tokyo streets: PLATEAU LOD2 buildings around
//! Shibuya and Shinjuku stations.
//!
//! Fetch the CityGML tiles once (about 80 MB, read straight out of the
//! official archive):
//!
//! ```bash
//! python3 scripts/fetch_plateau_tokyo.py
//! cargo run --release -p plateau_gnss_navigation --example 152_plateau_gnss_navigation
//! ```

// Shared with example 151, which uses all of it.
mod crowd;
#[allow(dead_code)]
#[path = "../151_gnss_lidar_navigation/gnss.rs"]
mod gnss;
mod lidar3d;
mod nav;
mod plateau;
#[cfg(feature = "capture")]
mod render;

#[cfg(feature = "capture")]
fn render_area(
    dir: &Path,
    city: &PlateauCity,
    route: &[(f64, f64)],
    base: Vec3,
    run: &RunReport,
) -> Result<(), String> {
    render::render(dir, city, route, base, run).map_err(|e| e.to_string())
}

#[cfg(not(feature = "capture"))]
fn render_area(
    _dir: &Path,
    _city: &PlateauCity,
    _route: &[(f64, f64)],
    _base: Vec3,
    _run: &RunReport,
) -> Result<(), String> {
    Err("--render needs `--features capture`".into())
}

use crowd::Weather;
use nav::{RunOptions, RunReport};
use plateau::PlateauCity;
use rne_gnss::Constellation;
use rne_math::Vec3;
use std::path::{Path, PathBuf};

/// A station area: the PLATEAU tile and the ENU origin.
struct Area {
    name: &'static str,
    slug: &'static str,
    tiles: &'static [&'static str],
    center: (f64, f64),
    ground_tp_m: f64,
    /// Walking route waypoints in ENU meters; the route between them is
    /// planned over the streets.
    waypoints: &'static [(f64, f64)],
    /// RTK reference station: a pole in a nearby park (latitude, longitude
    /// degrees), beyond the loaded buildings so its sky is open.
    base: (f64, f64),
}

const AREAS: [Area; 2] = [
    Area {
        name: "Shibuya",
        slug: "shibuya",
        tiles: &["53393585", "53393586", "53393595", "53393596"],
        center: (35.6590, 139.7010),
        ground_tp_m: 15.0,
        // The scramble crossing, west up the Dogenzaka side, north up Koen-dori,
        // and back to the crossing.
        waypoints: &[(-40.0, 47.0), (-180.0, 70.0), (25.0, 250.0), (-40.0, 47.0)],
        // Yoyogi Park, about 1.3 km north-west.
        base: (35.6690, 139.6950),
    },
    Area {
        name: "Shinjuku",
        slug: "shinjuku",
        tiles: &["53394525", "53394526", "53394535", "53394536"],
        // The west exit and the Nishi-Shinjuku skyscrapers.
        center: (35.6905, 139.6955),
        ground_tp_m: 38.0,
        // From the west exit side west among the towers, north, and back.
        waypoints: &[
            (140.0, -140.0),
            (-100.0, 33.0),
            (33.0, 180.0),
            (140.0, -140.0),
        ],
        // Shinjuku Gyoen, about 1.4 km east-south-east.
        base: (35.6850, 139.7090),
    },
];

/// Radius of buildings loaded around each station, meters.
const LOAD_RADIUS_M: f64 = 450.0;
/// Radius of the street survey, meters.
const SURVEY_RADIUS_M: f64 = 300.0;
const SURVEY_STEP_M: f64 = 3.0;
const ANTENNA_HEIGHT_M: f64 = 1.5;
const ELEVATION_MASK_DEG: f64 = 10.0;
const SKY_EPOCH_S: f64 = 20_000.0;

fn main() -> Result<(), String> {
    let data = PathBuf::from(
        std::env::args()
            .skip_while(|a| a != "--data")
            .nth(1)
            .unwrap_or_else(|| "target/plateau/tokyo23".to_string()),
    );
    let survey_only = std::env::args().any(|a| a == "--survey");
    let quick = std::env::args().any(|a| a == "--quick");
    let use_lidar3d = std::env::args().any(|a| a == "--lidar3d");
    let conditions = std::env::args().any(|a| a == "--conditions");
    let seeds: u64 = std::env::args()
        .skip_while(|a| a != "--seeds")
        .nth(1)
        .map_or(Ok(1), |n| n.parse())
        .map_err(|e| format!("--seeds: {e}"))?;
    let only_config = std::env::args().skip_while(|a| a != "--config").nth(1);
    let render_dir = std::env::args()
        .skip_while(|a| a != "--render")
        .nth(1)
        .map(PathBuf::from);
    let constellations = [
        ("GPS+Galileo", Constellation::gps_galileo_nominal()),
        (
            "GPS+Galileo+QZSS",
            Constellation::gps_galileo_qzss_nominal(),
        ),
    ];
    let only = std::env::args().skip_while(|a| a != "--area").nth(1);
    for area in AREAS
        .iter()
        .filter(|a| only.as_ref().is_none_or(|o| o.eq_ignore_ascii_case(a.slug)))
    {
        let paths: Vec<PathBuf> = area
            .tiles
            .iter()
            .map(|mesh| data.join(format!("{mesh}_bldg_6697_2_op.gml")))
            .collect();
        if let Some(missing) = paths.iter().find(|p| !p.exists()) {
            return Err(format!(
                "{} is missing: run `python3 scripts/fetch_plateau_tokyo.py` first",
                missing.display()
            ));
        }
        let started = std::time::Instant::now();
        let mut city = PlateauCity::load(
            area.name,
            &paths,
            area.center,
            area.ground_tp_m,
            LOAD_RADIUS_M,
        )?;
        println!(
            "{}: {} buildings, {} triangles, {} diffraction edges within {LOAD_RADIUS_M} m ({:.1} s)",
            city.name,
            city.buildings.len(),
            city.mesh.triangles().len(),
            city.mesh.diffraction_edges().len(),
            started.elapsed().as_secs_f64()
        );
        let mut qzss_survey = None;
        for (label, constellation) in &constellations {
            let survey = survey(&city, constellation);
            survey.print(label);
            qzss_survey = Some(survey);
        }
        let survey = qzss_survey.expect("survey");
        if survey_only {
            let png = PathBuf::from(format!("target/152_{}_sky.png", area.slug));
            write_map(&png, &city, &survey, &[], None)?;
            println!("  wrote {}", png.display());
            continue;
        }
        let route = nav::plan_route(&city, area.waypoints);
        let furniture = city.add_street_furniture(&route);
        let length: f64 = route
            .windows(2)
            .map(|s| (s[1].0 - s[0].0).hypot(s[1].1 - s[0].1))
            .sum();
        let base = base_station(&city, area.base);
        println!(
            "  route: {length:.0} m over {} waypoints, {furniture} poles and vending machines along it; RTK base in a park at ({:.0}, {:.0}, {:.0}) m",
            route.len(),
            base.x,
            base.y,
            base.z
        );
        let mut main_run = None;
        let runs = if conditions {
            condition_runs()
        } else {
            standard_runs(
                render_dir.is_some(),
                std::env::args().any(|a| a == "--render-weather"),
            )
        };
        // The 3D LiDAR's prior map: the mesh and terrain along the route.
        let cloud = use_lidar3d.then(|| {
            let started = std::time::Instant::now();
            let map = lidar3d::build_map(&city, &route, lidar3d::RANGE_M + 4.0);
            println!(
                "  3D LiDAR map: {} points ({:.1} s)",
                map.len(),
                started.elapsed().as_secs_f64()
            );
            map
        });
        for (label, options, glass_from_m) in runs {
            if quick && main_run.is_some() {
                break;
            }
            if let Some(only) = &only_config {
                if !label.contains(only.as_str()) {
                    continue;
                }
            }
            let styled = (options.weather.rain || glass_from_m != plateau::GLASS_TOWER_HEIGHT_M)
                .then(|| city.restyled(glass_from_m, options.weather.rain));
            let mut summary = Vec::new();
            for seed in 0..seeds {
                let started = std::time::Instant::now();
                let options = RunOptions {
                    seed,
                    lidar3d: use_lidar3d,
                    ..options
                };
                let report = nav::run(
                    styled.as_ref().unwrap_or(&city),
                    &route,
                    base,
                    options,
                    cloud.as_ref(),
                );
                print_report(label, &report, started.elapsed().as_secs_f64());
                summary.push(report.summary());
                if seed == 0 {
                    main_run.get_or_insert(report);
                }
            }
            if seeds > 1 {
                print_summary(label, &summary);
            }
        }
        let png = PathBuf::from(format!("target/152_{}_sky.png", area.slug));
        write_map(&png, &city, &survey, &route, main_run.as_ref())?;
        println!("  wrote {}", png.display());
        if let (Some(dir), Some(run)) = (&render_dir, &main_run) {
            render_area(&dir.join(area.slug), &city, &route, base, run)?;
        }
    }
    Ok(())
}

/// The configurations compared by default, as `(label, options, glass
/// curtain walls from this height)`; `record` keeps frames of the first,
/// which runs in a crowd and rain with `weather`.
fn standard_runs(record: bool, weather: bool) -> Vec<(&'static str, RunOptions, f64)> {
    vec![
        (
            if weather {
                "map-aided GNSS + QZSS, crowd + rain"
            } else {
                "map-aided GNSS + QZSS"
            },
            RunOptions {
                map_aided_gnss: true,
                qzss: true,
                record,
                factor_graph: false,
                weather: if weather {
                    Weather {
                        pedestrians: PEDESTRIANS,
                        rain: true,
                    }
                } else {
                    Weather::default()
                },
                seed: 0,
                lidar3d: false,
            },
        ),
        (
            "GNSS + QZSS, no map aid",
            RunOptions {
                map_aided_gnss: false,
                qzss: true,
                record: false,
                factor_graph: false,
                weather: Weather::default(),
                seed: 0,
                lidar3d: false,
            },
        ),
        (
            "factor graph",
            RunOptions {
                map_aided_gnss: true,
                qzss: true,
                record: false,
                factor_graph: true,
                weather: Weather::default(),
                seed: 0,
                lidar3d: false,
            },
        ),
        (
            "factor graph, no map aid",
            RunOptions {
                map_aided_gnss: false,
                qzss: true,
                record: false,
                factor_graph: true,
                weather: Weather::default(),
                seed: 0,
                lidar3d: false,
            },
        ),
        (
            "map-aided GNSS, no QZSS",
            RunOptions {
                map_aided_gnss: true,
                qzss: false,
                record: false,
                factor_graph: false,
                weather: Weather::default(),
                seed: 0,
                lidar3d: false,
            },
        ),
    ]
    .into_iter()
    .map(|(label, options)| (label, options, plateau::GLASS_TOWER_HEIGHT_M))
    .collect()
}

/// Pedestrians along the route in the crowd conditions.
const PEDESTRIANS: usize = 60;

/// `--conditions`: the main configuration, hybrid and factor graph, in clear
/// weather, in a crowd, in rain, and with more glass, as `(label, options, glass curtain walls from this height)`.
fn condition_runs() -> Vec<(&'static str, RunOptions, f64)> {
    let options = |weather, factor_graph| RunOptions {
        map_aided_gnss: true,
        qzss: true,
        record: false,
        factor_graph,
        weather,
        seed: 0,
        lidar3d: false,
    };
    let crowd = Weather {
        pedestrians: PEDESTRIANS,
        rain: false,
    };
    let rain = Weather {
        pedestrians: PEDESTRIANS,
        rain: true,
    };
    let glass = plateau::GLASS_TOWER_HEIGHT_M;
    let clear = Weather::default();
    vec![
        ("clear, hybrid", options(clear, false), glass),
        ("clear, factor graph", options(clear, true), glass),
        ("crowd, hybrid", options(crowd, false), glass),
        ("crowd, factor graph", options(crowd, true), glass),
        ("crowd + rain, hybrid", options(rain, false), glass),
        ("crowd + rain, factor graph", options(rain, true), glass),
        ("glass from 20 m, hybrid", options(clear, false), 20.0),
        ("glass from 20 m, factor graph", options(clear, true), 20.0),
    ]
}

/// The RTK reference station: an antenna on a 2 m pole in a park, at street
/// level (a base hundreds of metres above the rover would leave a
/// tropospheric difference that the double differences do not cancel).
fn base_station(city: &PlateauCity, (lat, lon): (f64, f64)) -> Vec3 {
    let at = city
        .frame
        .ecef_to_enu(rne_gnss::Geodetic::from_degrees(lat, lon, 0.0).to_ecef());
    Vec3::new(at.x, at.y, city.ground_m(at.x, at.y) + 2.0)
}

fn print_report(label: &str, report: &RunReport, wall_s: f64) {
    let (rmse, max) = report.rmse_max();
    let [fixed, float, single, none] = report.fix_shares();
    let epochs = report.epochs.max(1) as f64;
    println!(
        "  {label:<24} goal {} in {:.0} s ({:.0} m); error RMSE {rmse:.2} m, max {max:.2} m; \
         RTK fixed {fixed:.0} % float {float:.0} % single {single:.0} % none {none:.0} %",
        if report.reached {
            "reached"
        } else {
            "NOT reached"
        },
        report.duration_s,
        report.distance_m,
    );
    println!(
        "  {:<24} sky: {:.1} tracked ({:.1} LOS, {:.1} NLOS/diffracted); GNSS {:.0} s, LiDAR {:.0} s, \
         dead reckoning {:.0} s (longest {:.1} s, {} safety stops); waited for obstacles {:.0} s; \
         min LiDAR clearance {:.2} m [{wall_s:.0} s]",
        "",
        report.tracked / epochs,
        report.los / epochs,
        report.nlos / epochs,
        report.time_in[0],
        report.time_in[1],
        report.time_in[2],
        report.longest_dead_reckoning_s,
        report.safety_stops,
        report.obstacle_wait_s,
        report.clearance_m,
    );
}

/// One line over several seeds: goals reached, median and worst RMSE, the
/// worst error and the longest dead reckoning.
fn print_summary(label: &str, runs: &[nav::RunSummary]) {
    let mut rmse: Vec<f64> = runs.iter().map(|r| r.rmse_m).collect();
    rmse.sort_by(f64::total_cmp);
    let fold = |f: fn(&nav::RunSummary) -> f64| runs.iter().map(f).fold(0.0, f64::max);
    println!(
        "  {label:<24} SUMMARY over {} seeds: goal reached {}/{}; RMSE median {:.2} m, worst {:.2} m; max error {:.2} m; longest dead reckoning {:.1} s",
        runs.len(),
        runs.iter().filter(|r| r.reached).count(),
        runs.len(),
        rmse[rmse.len() / 2],
        rmse.last().copied().unwrap_or(0.0),
        fold(|r| r.max_m),
        fold(|r| r.longest_dead_reckoning_s),
    );
}

/// Satellites in view at each street point.
struct Survey {
    /// `(x, y, line-of-sight satellites, satellites above the mask)`.
    points: Vec<(f64, f64, usize, usize)>,
}

impl Survey {
    fn print(&self, name: &str) {
        let n = self.points.len().max(1) as f64;
        let share = |f: &dyn Fn(usize) -> bool| {
            100.0 * self.points.iter().filter(|p| f(p.2)).count() as f64 / n
        };
        let mean_los = self.points.iter().map(|p| p.2 as f64).sum::<f64>() / n;
        let mean_up = self.points.iter().map(|p| p.3 as f64).sum::<f64>() / n;
        println!(
            "  {name:<17} {} street points: {mean_los:.1} of {mean_up:.1} satellites in line of sight; \
             >= 8: {:.0} % of points, < 4: {:.0} %",
            self.points.len(),
            share(&|c| c >= 8),
            share(&|c| c < 4)
        );
    }
}

fn satellite_directions(city: &PlateauCity, constellation: &Constellation, at: Vec3) -> Vec<Vec3> {
    let rx = city.frame.enu_to_ecef(at);
    constellation
        .satellites()
        .iter()
        .filter_map(|s| {
            let d = city
                .frame
                .ecef_vector_to_enu(s.orbit.position_ecef(SKY_EPOCH_S) - rx)
                .normalize();
            (d.z.asin().to_degrees() >= ELEVATION_MASK_DEG).then_some(d)
        })
        .collect()
}

fn survey(city: &PlateauCity, constellation: &Constellation) -> Survey {
    let directions = satellite_directions(city, constellation, Vec3::ZERO);
    let mut points = Vec::new();
    let steps = (SURVEY_RADIUS_M / SURVEY_STEP_M) as i32;
    for iy in -steps..=steps {
        for ix in -steps..=steps {
            let (x, y) = (f64::from(ix) * SURVEY_STEP_M, f64::from(iy) * SURVEY_STEP_M);
            if x.hypot(y) > SURVEY_RADIUS_M {
                continue;
            }
            let ground = city.ground_m(x, y);
            let antenna = Vec3::new(x, y, ground + ANTENNA_HEIGHT_M);
            // Skip points inside or under buildings (and decks).
            if city.roof_m(x, y).is_some_and(|roof| roof > antenna.z) {
                continue;
            }
            let los = directions
                .iter()
                .filter(|d| !city.mesh.occluded(antenna, **d, f64::INFINITY))
                .count();
            points.push((x, y, los, directions.len()));
        }
    }
    Survey { points }
}

fn write_map(
    path: &Path,
    city: &PlateauCity,
    survey: &Survey,
    route: &[(f64, f64)],
    run: Option<&RunReport>,
) -> Result<(), String> {
    const PX_PER_M: f64 = 1.5;
    let half = SURVEY_RADIUS_M;
    let size = (2.0 * half * PX_PER_M) as usize;
    let mut rgb = vec![0_u8; size * size * 3];
    let mut los_at = std::collections::HashMap::new();
    for &(x, y, los, _) in &survey.points {
        los_at.insert(
            (
                (x / SURVEY_STEP_M).round() as i64,
                (y / SURVEY_STEP_M).round() as i64,
            ),
            los,
        );
    }
    for row in 0..size {
        for col in 0..size {
            let x = col as f64 / PX_PER_M - half;
            let y = half - row as f64 / PX_PER_M;
            let colour = if let Some(roof) = city.roof_m(x, y) {
                let shade = (90.0 + (roof - city.ground_m(x, y)).clamp(0.0, 200.0) * 0.6) as u8;
                [shade, shade, shade.saturating_add(8)]
            } else if let Some(&los) = los_at.get(&(
                (x / SURVEY_STEP_M).round() as i64,
                (y / SURVEY_STEP_M).round() as i64,
            )) {
                // Red (few) → yellow → green (many satellites).
                let f = (los as f64 / 12.0).clamp(0.0, 1.0);
                [
                    (255.0 * (2.0 - 2.0 * f).min(1.0)) as u8,
                    (220.0 * (2.0 * f).min(1.0)) as u8,
                    40,
                ]
            } else {
                [30, 30, 34]
            };
            let i = (row * size + col) * 3;
            rgb[i..i + 3].copy_from_slice(&colour);
        }
    }
    let mut plot = |x: f64, y: f64, colour: [u8; 3], radius: i64| {
        let col = ((x + half) * PX_PER_M) as i64;
        let row = ((half - y) * PX_PER_M) as i64;
        for dr in -radius..=radius {
            for dc in -radius..=radius {
                let (r, c) = (row + dr, col + dc);
                if r >= 0 && c >= 0 && (r as usize) < size && (c as usize) < size {
                    let i = (r as usize * size + c as usize) * 3;
                    rgb[i..i + 3].copy_from_slice(&colour);
                }
            }
        }
    };
    // The planned route in white, then the walked track coloured by the
    // localization source (blue GNSS, magenta LiDAR, orange dead reckoning).
    for pair in route.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let steps = ((b.0 - a.0).hypot(b.1 - a.1) * PX_PER_M * 2.0)
            .ceil()
            .max(1.0) as usize;
        for k in 0..=steps {
            let f = k as f64 / steps as f64;
            plot(
                a.0 + f * (b.0 - a.0),
                a.1 + f * (b.1 - a.1),
                [255, 255, 255],
                1,
            );
        }
    }
    if let Some(run) = run {
        for (truth, _, source) in &run.track {
            let colour = match source {
                rne_slam::LocalizationSource::Gnss => [40, 110, 255],
                rne_slam::LocalizationSource::Lidar => [230, 40, 230],
                rne_slam::LocalizationSource::DeadReckoning => [255, 140, 0],
            };
            plot(truth.x_m, truth.y_m, colour, 1);
        }
    }
    let file = std::fs::File::create(path).map_err(|e| e.to_string())?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), size as u32, size as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(|e| e.to_string())?;
    writer.write_image_data(&rgb).map_err(|e| e.to_string())
}
