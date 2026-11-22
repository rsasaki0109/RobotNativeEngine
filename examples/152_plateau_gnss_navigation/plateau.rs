//! PLATEAU LOD2 buildings as a GNSS ray-tracing mesh in a local ENU frame.

use rne_gnss::{EnuFrame, Geodetic, MeshTriangle, SurfaceMaterial, TriangleMesh};
use rne_math::Vec3;
use rne_plateau::BuildingSurface;
use std::path::PathBuf;
use std::sync::Arc;

/// Geoid height above the GRS80 ellipsoid around central Tokyo (GSIGEO2011),
/// meters: PLATEAU heights are orthometric (Tokyo Peil), GNSS needs
/// ellipsoidal ones.
pub(crate) const GEOID_TOKYO_M: f64 = 36.7;
/// Buildings at least this tall are treated as glass curtain-wall towers;
/// lower ones as concrete (PLATEAU's 2020 Tokyo LOD2 has no facade material).
pub(crate) const GLASS_TOWER_HEIGHT_M: f64 = 45.0;

/// One building of the loaded area.
#[derive(Clone, Debug)]
pub(crate) struct Building {
    /// Footprint bounds `[x0, y0, x1, y1]` in ENU meters.
    pub(crate) bounds: [f64; 4],
    /// Lowest point (ground) and highest point in ENU meters.
    pub(crate) z_range: (f64, f64),
}

/// The loaded city: an ENU frame at a station and its buildings as a mesh.
#[derive(Clone, Debug)]
pub(crate) struct PlateauCity {
    pub(crate) name: String,
    pub(crate) frame: EnuFrame,
    pub(crate) mesh: Arc<TriangleMesh>,
    pub(crate) buildings: Vec<Building>,
}

impl PlateauCity {
    /// Loads the buildings of `CityGML` tiles within `radius_m` of `center`
    /// (latitude, longitude degrees). The ENU origin is at `center` on the
    /// ground (`ground_tp_m`, Tokyo Peil height).
    pub(crate) fn load(
        name: &str,
        paths: &[PathBuf],
        center: (f64, f64),
        ground_tp_m: f64,
        radius_m: f64,
    ) -> Result<Self, String> {
        let mut parsed = Vec::new();
        for path in paths {
            let xml =
                std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
            parsed.extend(
                rne_plateau::parse_building_triangles(&xml)
                    .map_err(|e| format!("{}: {e}", path.display()))?,
            );
        }
        let frame = EnuFrame::new(Geodetic::from_degrees(
            center.0,
            center.1,
            ground_tp_m + GEOID_TOKYO_M,
        ));
        let to_enu = |v: [f64; 3]| {
            frame.ecef_to_enu(Geodetic::from_degrees(v[0], v[1], v[2] + GEOID_TOKYO_M).to_ecef())
        };
        let mut triangles = Vec::new();
        let mut buildings = Vec::new();
        for building in parsed {
            let corners: Vec<[Vec3; 3]> = building
                .triangles
                .iter()
                .map(|t| t.vertices.map(to_enu))
                .collect();
            let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
            for v in corners.iter().flatten() {
                lo = lo.min(*v);
                hi = hi.max(*v);
            }
            let centre = 0.5 * (lo + hi);
            if corners.is_empty() || centre.x.hypot(centre.y) > radius_m {
                continue;
            }
            // CityGML winding is defined in the CRS's (latitude, longitude)
            // axis order; make each building's surface face outward in ENU
            // (positive enclosed volume).
            let volume: f64 = corners
                .iter()
                .map(|[a, b, c]| (*a - centre).dot((*b - centre).cross(*c - centre)))
                .sum();
            let flip = volume < 0.0;
            let height = hi.z - lo.z;
            let wall = if height >= GLASS_TOWER_HEIGHT_M {
                SurfaceMaterial::GLASS
            } else {
                SurfaceMaterial::CONCRETE
            };
            let object = buildings.len() as u32;
            for (t, [a, b, c]) in building.triangles.iter().zip(&corners) {
                let material = match t.surface {
                    BuildingSurface::Wall | BuildingSurface::Unknown => wall,
                    BuildingSurface::Roof | BuildingSurface::OuterCeiling => {
                        SurfaceMaterial::CONCRETE
                    }
                    // Ground and closure surfaces are never seen from outside.
                    _ => SurfaceMaterial::ABSORBER,
                };
                let vertices = if flip { [*a, *c, *b] } else { [*a, *b, *c] };
                if let Some(tri) = MeshTriangle::new(vertices, material, object) {
                    triangles.push(tri);
                }
            }
            buildings.push(Building {
                bounds: [lo.x, lo.y, hi.x, hi.y],
                z_range: (lo.z, hi.z),
            });
        }
        if buildings.is_empty() {
            return Err(format!("no buildings within {radius_m} m of {center:?}"));
        }
        Ok(Self {
            name: name.to_string(),
            frame,
            mesh: Arc::new(TriangleMesh::new(triangles)),
            buildings,
        })
    }

    /// Ground height at a point: the lowest building base nearby (PLATEAU
    /// buildings stand on the terrain), inverse-distance weighted.
    pub(crate) fn ground_m(&self, x: f64, y: f64) -> f64 {
        let (mut sum, mut weight) = (0.0, 0.0);
        for b in &self.buildings {
            let dx = (b.bounds[0] - x).max(x - b.bounds[2]).max(0.0);
            let dy = (b.bounds[1] - y).max(y - b.bounds[3]).max(0.0);
            let d = dx.hypot(dy);
            if d < 60.0 {
                let w = 1.0 / (d + 5.0).powi(2);
                sum += w * b.z_range.0;
                weight += w;
            }
        }
        if weight > 0.0 {
            sum / weight
        } else {
            0.0
        }
    }

    /// Roof height under a point (for drawing), or `None` on open ground.
    pub(crate) fn roof_m(&self, x: f64, y: f64) -> Option<f64> {
        let top = 1000.0;
        self.mesh
            .raycast(Vec3::new(x, y, top), Vec3::NEG_Z, 2.0 * top)
            .map(|(t, _)| top - t)
    }
}
