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

mod plateau;

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
}

const AREAS: [Area; 2] = [
    Area {
        name: "Shibuya",
        slug: "shibuya",
        tiles: &["53393585", "53393586", "53393595", "53393596"],
        center: (35.6590, 139.7010),
        ground_tp_m: 15.0,
    },
    Area {
        name: "Shinjuku",
        slug: "shinjuku",
        tiles: &["53394525", "53394526", "53394535", "53394536"],
        // The west exit and the Nishi-Shinjuku skyscrapers.
        center: (35.6905, 139.6955),
        ground_tp_m: 38.0,
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
    let constellations = [
        ("GPS+Galileo", Constellation::gps_galileo_nominal()),
        (
            "GPS+Galileo+QZSS",
            Constellation::gps_galileo_qzss_nominal(),
        ),
    ];
    for area in &AREAS {
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
        let city = PlateauCity::load(
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
        for (label, constellation) in &constellations {
            let survey = survey(&city, constellation);
            survey.print(label);
            if label.ends_with("QZSS") {
                let png = PathBuf::from(format!("target/152_{}_sky.png", area.slug));
                write_map(&png, &city, &survey)?;
                println!("  wrote {}", png.display());
            }
        }
    }
    Ok(())
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

fn write_map(path: &Path, city: &PlateauCity, survey: &Survey) -> Result<(), String> {
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
    let file = std::fs::File::create(path).map_err(|e| e.to_string())?;
    let mut encoder = png::Encoder::new(std::io::BufWriter::new(file), size as u32, size as u32);
    encoder.set_color(png::ColorType::Rgb);
    encoder.set_depth(png::BitDepth::Eight);
    let mut writer = encoder.write_header().map_err(|e| e.to_string())?;
    writer.write_image_data(&rgb).map_err(|e| e.to_string())
}
