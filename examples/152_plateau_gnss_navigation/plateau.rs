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
    /// Terrain heights on a grid (see [`TERRAIN_CELL_M`]).
    terrain: Vec<f64>,
    terrain_half_cells: usize,
}

/// Terrain grid spacing, meters.
const TERRAIN_CELL_M: f64 = 2.0;

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
        let mut city = Self {
            name: name.to_string(),
            frame,
            mesh: Arc::new(TriangleMesh::new(triangles)),
            buildings,
            terrain: Vec::new(),
            terrain_half_cells: (radius_m / TERRAIN_CELL_M).ceil() as usize + 1,
        };
        let n = 2 * city.terrain_half_cells + 1;
        let half = city.terrain_half_cells as f64;
        city.terrain = (0..n * n)
            .map(|k| {
                let x = ((k % n) as f64 - half) * TERRAIN_CELL_M;
                let y = ((k / n) as f64 - half) * TERRAIN_CELL_M;
                city.ground_idw(x, y)
            })
            .collect();
        Ok(city)
    }

    /// Ground height at a point, interpolated from the terrain grid.
    pub(crate) fn ground_m(&self, x: f64, y: f64) -> f64 {
        let n = 2 * self.terrain_half_cells + 1;
        let half = self.terrain_half_cells as f64;
        let gx = (x / TERRAIN_CELL_M + half).clamp(0.0, (n - 2) as f64);
        let gy = (y / TERRAIN_CELL_M + half).clamp(0.0, (n - 2) as f64);
        let (i, j) = (gx.floor() as usize, gy.floor() as usize);
        let (fx, fy) = (gx - i as f64, gy - j as f64);
        let h = |i: usize, j: usize| self.terrain[j * n + i];
        (1.0 - fy) * ((1.0 - fx) * h(i, j) + fx * h(i + 1, j))
            + fy * ((1.0 - fx) * h(i, j + 1) + fx * h(i + 1, j + 1))
    }

    /// Ground height at a point from the bases of nearby buildings (PLATEAU
    /// buildings stand on the terrain), inverse-distance weighted.
    fn ground_idw(&self, x: f64, y: f64) -> f64 {
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

    /// Adds the street furniture LOD2 models leave out along a route: utility
    /// poles and signposts every 6-14 m (an irregular, fixed sequence) about
    /// 0.8 m in front of the facades on both sides, and a vending machine
    /// against the facade at every fourth spot. They are part of the city
    /// for the sensors and the prior map (GNSS treats them as small
    /// absorbers). Returns how many were placed.
    pub(crate) fn add_street_furniture(&mut self, route: &[(f64, f64)]) -> usize {
        const SPACING_M: [f64; 8] = [6.0, 9.0, 13.0, 7.0, 11.0, 8.0, 14.0, 10.0];
        let mut triangles = self.mesh.triangles().to_vec();
        let mut placed: Vec<(f64, f64)> = Vec::new();
        let mut object = self.buildings.len() as u32 + 1_000_000;
        let (mut k, mut carry) = (0_usize, 0.0);
        for leg in route.windows(2) {
            let (a, b) = (leg[0], leg[1]);
            let length = (b.0 - a.0).hypot(b.1 - a.1);
            if length < 1.0e-6 {
                continue;
            }
            let (dx, dy) = ((b.0 - a.0) / length, (b.1 - a.1) / length);
            let mut s = SPACING_M[k % SPACING_M.len()] - carry;
            while s < length {
                let (px, py) = (a.0 + dx * s, a.1 + dy * s);
                let ground = self.ground_m(px, py);
                for side in [-1.0, 1.0] {
                    let (nx, ny) = (-dy * side, dx * side);
                    let Some((h, _)) = self.mesh.raycast(
                        Vec3::new(px, py, ground + 1.0),
                        Vec3::new(nx, ny, 0.0),
                        25.0,
                    ) else {
                        continue;
                    };
                    let vending = k % 4 == 3;
                    let offset = if vending { h - 0.4 } else { h - 0.8 };
                    if offset < 1.8 {
                        continue;
                    }
                    let (cx, cy) = (px + nx * offset, py + ny * offset);
                    if placed.iter().any(|q| (q.0 - cx).hypot(q.1 - cy) < 3.0) {
                        continue;
                    }
                    let z = self.ground_m(cx, cy);
                    let (half_along, half_across, height) = if vending {
                        (0.5, 0.35, 1.8)
                    } else {
                        (0.15, 0.15, 8.0)
                    };
                    // Axis-aligned along the street direction's dominant axis.
                    let (hx, hy) = if dx.abs() > dy.abs() {
                        (half_along, half_across)
                    } else {
                        (half_across, half_along)
                    };
                    triangles.extend(rne_gnss::box_triangles(
                        Vec3::new(cx - hx, cy - hy, z - 0.5),
                        Vec3::new(cx + hx, cy + hy, z + height),
                        SurfaceMaterial::ABSORBER,
                        object,
                    ));
                    object += 1;
                    placed.push((cx, cy));
                }
                k += 1;
                s += SPACING_M[k % SPACING_M.len()];
            }
            carry = length - (s - SPACING_M[k % SPACING_M.len()]);
        }
        self.mesh = Arc::new(TriangleMesh::new(triangles));
        placed.len()
    }

    /// Roof height under a point (for drawing), or `None` on open ground.
    pub(crate) fn roof_m(&self, x: f64, y: f64) -> Option<f64> {
        let top = 1000.0;
        self.mesh
            .raycast(Vec3::new(x, y, top), Vec3::NEG_Z, 2.0 * top)
            .map(|(t, _)| top - t)
    }
}
