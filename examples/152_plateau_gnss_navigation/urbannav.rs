//! `--urbannav CSV`: the ray tracer against a real drive through Shinjuku.
//!
//! The CSV (from `urbannav.py`) lists, once a second along the UrbanNav
//! Tokyo "Shinjuku" run, every GPS, Galileo and QZSS satellite above 5° at
//! the true antenna position, with the C/N0 the u-blox receiver measured
//! (empty when it did not track the satellite). Each direction is traced
//! through the PLATEAU model of the same streets: line of sight, NLOS (only
//! a reflected or diffracted path reaches the antenna) or blocked. The
//! measured C/N0 of each class shows whether the model's predictions are
//! the ones the street produced.

use crate::plateau::PlateauCity;
use rne_gnss::{trace_satellite, Geodetic, GnssScene};
use rne_math::Vec3;
use std::path::{Path, PathBuf};

/// The tiles the drive crosses (PLATEAU third-level meshes).
const TILES: [&str; 6] = [
    "53394514", "53394515", "53394524", "53394525", "53394534", "53394535",
];
/// The drive's centre and the radius of buildings loaded around it.
const CENTER: (f64, f64) = (35.6890, 139.6900);
const RADIUS_M: f64 = 1_300.0;
/// Tokyo Peil height of the ground around the centre, meters.
const GROUND_TP_M: f64 = 38.0;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Class {
    LineOfSight,
    Nlos,
    Blocked,
}

/// One satellite at one epoch.
struct Row {
    tow_s: f64,
    lat_deg: f64,
    lon_deg: f64,
    height_m: f64,
    az_deg: f64,
    el_deg: f64,
    cn0_dbhz: Option<f64>,
}

fn read(path: &Path) -> Result<Vec<Row>, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut rows = Vec::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 8 {
            continue;
        }
        let num = |s: &str| s.trim().parse::<f64>().map_err(|e| format!("{line}: {e}"));
        rows.push(Row {
            tow_s: num(f[0])?,
            lat_deg: num(f[1])?,
            lon_deg: num(f[2])?,
            height_m: num(f[3])?,
            az_deg: num(f[5])?,
            el_deg: num(f[6])?,
            cn0_dbhz: (!f[7].trim().is_empty()).then(|| num(f[7])).transpose()?,
        });
    }
    Ok(rows)
}

fn median(mut v: Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// Loads the PLATEAU tiles the drive crosses from `data` and validates
/// against the CSV.
pub(crate) fn run(data: &Path, csv: &Path) -> Result<(), String> {
    let paths: Vec<PathBuf> = TILES
        .iter()
        .map(|mesh| data.join(format!("{mesh}_bldg_6697_2_op.gml")))
        .collect();
    if let Some(missing) = paths.iter().find(|p| !p.exists()) {
        return Err(format!(
            "{} is missing: run `python3 scripts/fetch_plateau_tokyo.py {}` first",
            missing.display(),
            TILES.join(" ")
        ));
    }
    let started = std::time::Instant::now();
    let city = PlateauCity::load(
        "Shinjuku (UrbanNav drive)",
        &paths,
        CENTER,
        GROUND_TP_M,
        RADIUS_M,
    )?;
    println!(
        "{}: {} buildings, {} triangles ({:.1} s)",
        city.name,
        city.buildings.len(),
        city.mesh.triangles().len(),
        started.elapsed().as_secs_f64()
    );
    validate(&city, csv)
}

/// Traces every satellite-epoch of the CSV and prints how the measured
/// signals compare with the predictions.
fn validate(city: &PlateauCity, csv: &Path) -> Result<(), String> {
    let rows = read(csv)?;
    let scene = GnssScene {
        mesh: Some(city.mesh.clone()),
        ..GnssScene::open_sky()
    };
    let trace = crate::nav::receiver_config().trace;
    let mut classes = Vec::with_capacity(rows.len());
    let mut heights = Vec::new();
    let mut last_tow = f64::NAN;
    for row in &rows {
        let antenna = city
            .frame
            .ecef_to_enu(Geodetic::from_degrees(row.lat_deg, row.lon_deg, row.height_m).to_ecef());
        if row.tow_s != last_tow {
            heights.push(antenna.z - city.ground_m(antenna.x, antenna.y));
            last_tow = row.tow_s;
        }
        let (az, el) = (row.az_deg.to_radians(), row.el_deg.to_radians());
        let u = Vec3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin());
        let class = if !city.mesh.occluded(antenna, u, 3_000.0) {
            Class::LineOfSight
        } else if trace_satellite(&scene, antenna, u, &trace)
            .indirect_paths()
            .next()
            .is_some()
        {
            Class::Nlos
        } else {
            Class::Blocked
        };
        classes.push(class);
    }
    let epochs = heights.len();
    println!(
        "UrbanNav Shinjuku: {} satellite-epochs (GPS, Galileo, QZSS above 5°) over {epochs} s; \
         the reference point is {:.1} m above the PLATEAU terrain (median)",
        rows.len(),
        median(heights)
    );
    println!("  predicted          share   tracked   C/N0 median   C/N0 >= 40 dB-Hz");
    for (class, name) in [
        (Class::LineOfSight, "line of sight"),
        (Class::Nlos, "NLOS (reflected)"),
        (Class::Blocked, "blocked"),
    ] {
        let members: Vec<&Row> = rows
            .iter()
            .zip(&classes)
            .filter(|(_, c)| **c == class)
            .map(|(r, _)| r)
            .collect();
        let tracked: Vec<f64> = members.iter().filter_map(|r| r.cn0_dbhz).collect();
        let strong = tracked.iter().filter(|c| **c >= 40.0).count();
        println!(
            "  {name:<17} {:5.1} %   {:5.1} %   {:6.1} dB-Hz   {:5.1} %",
            100.0 * members.len() as f64 / rows.len() as f64,
            100.0 * tracked.len() as f64 / members.len().max(1) as f64,
            median(tracked),
            100.0 * strong as f64 / members.len().max(1) as f64,
        );
    }
    // How well the model's line-of-sight call predicts a strong signal.
    let (mut tp, mut fp, mut fn_, mut tn) = (0, 0, 0, 0);
    for (row, class) in rows.iter().zip(&classes) {
        let strong = row.cn0_dbhz.is_some_and(|c| c >= 40.0);
        match (*class == Class::LineOfSight, strong) {
            (true, true) => tp += 1,
            (true, false) => fp += 1,
            (false, true) => fn_ += 1,
            (false, false) => tn += 1,
        }
    }
    let accuracy = f64::from(tp + tn) / rows.len() as f64;
    let open_sky = f64::from(tp + fn_) / rows.len() as f64;
    println!(
        "  line of sight vs a strong (>= 40 dB-Hz) signal: agree {:.0} % of satellite-epochs \
         (an open-sky guess: {:.0} %); of predicted line of sight {:.0} % strong, of strong \
         signals {:.0} % predicted line of sight",
        100.0 * accuracy,
        100.0 * open_sky,
        100.0 * f64::from(tp) / f64::from((tp + fp).max(1)),
        100.0 * f64::from(tp) / f64::from((tp + fn_).max(1)),
    );
    // Per epoch: satellites the model predicts in line of sight vs tracked
    // strongly.
    let mut per_epoch: Vec<(f64, usize, usize)> = Vec::new();
    for (row, class) in rows.iter().zip(&classes) {
        if per_epoch.last().is_none_or(|e| e.0 != row.tow_s) {
            per_epoch.push((row.tow_s, 0, 0));
        }
        let e = per_epoch.last_mut().expect("epoch");
        e.1 += usize::from(*class == Class::LineOfSight);
        e.2 += usize::from(row.cn0_dbhz.is_some_and(|c| c >= 40.0));
    }
    let n = per_epoch.len() as f64;
    let mean = |f: &dyn Fn(&(f64, usize, usize)) -> f64| per_epoch.iter().map(f).sum::<f64>() / n;
    let (mp, ms) = (mean(&|e| e.1 as f64), mean(&|e| e.2 as f64));
    let cov = mean(&|e| (e.1 as f64 - mp) * (e.2 as f64 - ms));
    let (sp, ss) = (
        mean(&|e| (e.1 as f64 - mp).powi(2)).sqrt(),
        mean(&|e| (e.2 as f64 - ms).powi(2)).sqrt(),
    );
    println!(
        "  per second: {mp:.1} satellites predicted in line of sight, {ms:.1} tracked at >= 40 dB-Hz; \
         correlation {:.2}",
        cov / (sp * ss)
    );
    Ok(())
}
