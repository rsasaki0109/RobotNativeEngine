//! Dense 3D occupancy.

use glam::DVec3;
use serde::{Deserialize, Serialize};

use robot_foresight::ForesightError;

/// A dense, axis-aligned 3D occupancy grid. Space outside the grid counts
/// as occupied, so a plan never leaves the mapped volume.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VoxelGrid {
    origin_m: DVec3,
    resolution_m: f64,
    dims: [usize; 3],
    occupied: Vec<bool>,
}

impl VoxelGrid {
    /// An all-free grid with its minimum corner at `origin_m`.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] for a non-positive
    /// resolution, a zero dimension or a non-finite origin.
    pub fn new(
        origin_m: DVec3,
        resolution_m: f64,
        dims: [usize; 3],
    ) -> Result<Self, ForesightError> {
        if !(origin_m.is_finite() && resolution_m.is_finite() && resolution_m > 0.0)
            || dims.contains(&0)
        {
            return Err(ForesightError::InvalidConfig("voxel grid geometry".into()));
        }
        Ok(Self {
            origin_m,
            resolution_m,
            dims,
            occupied: vec![false; dims[0] * dims[1] * dims[2]],
        })
    }

    /// Cell edge length (m).
    #[must_use]
    pub fn resolution_m(&self) -> f64 {
        self.resolution_m
    }

    /// Cells per axis.
    #[must_use]
    pub fn dims(&self) -> [usize; 3] {
        self.dims
    }

    /// Minimum corner (m).
    #[must_use]
    pub fn origin_m(&self) -> DVec3 {
        self.origin_m
    }

    fn index(&self, c: [usize; 3]) -> usize {
        (c[2] * self.dims[1] + c[1]) * self.dims[0] + c[0]
    }

    /// Cell containing `p`, if inside the grid.
    #[must_use]
    pub fn cell_of(&self, p: DVec3) -> Option<[usize; 3]> {
        let rel = (p - self.origin_m) / self.resolution_m;
        let mut out = [0usize; 3];
        for (axis, slot) in out.iter_mut().enumerate() {
            let v = rel[axis].floor();
            if !(v >= 0.0 && (v as usize) < self.dims[axis]) {
                return None;
            }
            *slot = v as usize;
        }
        Some(out)
    }

    /// Centre of cell `c` (m).
    #[must_use]
    pub fn cell_center(&self, c: [usize; 3]) -> DVec3 {
        self.origin_m
            + (DVec3::new(c[0] as f64, c[1] as f64, c[2] as f64) + DVec3::splat(0.5))
                * self.resolution_m
    }

    /// Whether cell `c` is occupied (out-of-range cells are occupied).
    #[must_use]
    pub fn is_occupied_cell(&self, c: [usize; 3]) -> bool {
        if c.iter().zip(self.dims).any(|(v, d)| *v >= d) {
            return true;
        }
        self.occupied[self.index(c)]
    }

    /// Marks cell `c` (ignored when out of range).
    pub fn set_occupied_cell(&mut self, c: [usize; 3], occupied: bool) {
        if c.iter().zip(self.dims).all(|(v, d)| *v < d) {
            let i = self.index(c);
            self.occupied[i] = occupied;
        }
    }

    /// Marks every cell whose centre lies in the box `[min_m, max_m]`.
    pub fn fill_box(&mut self, min_m: DVec3, max_m: DVec3) {
        for z in 0..self.dims[2] {
            for y in 0..self.dims[1] {
                for x in 0..self.dims[0] {
                    let c = self.cell_center([x, y, z]);
                    if c.cmpge(min_m).all() && c.cmple(max_m).all() {
                        let i = self.index([x, y, z]);
                        self.occupied[i] = true;
                    }
                }
            }
        }
    }

    /// Number of occupied cells.
    #[must_use]
    pub fn occupied_count(&self) -> usize {
        self.occupied.iter().filter(|o| **o).count()
    }

    /// Whether a sphere touches an occupied cell or leaves the grid.
    #[must_use]
    pub fn sphere_collides(&self, center_m: DVec3, radius_m: f64) -> bool {
        self.sphere_clearance(center_m, radius_m, self.resolution_m) <= 0.0
    }

    /// Distance (m) from the sphere's surface to the nearest occupied cell
    /// or grid face, searched up to `search_m` beyond the sphere; returns
    /// `search_m` when nothing is that close and a non-positive value on
    /// contact.
    #[must_use]
    pub fn sphere_clearance(&self, center_m: DVec3, radius_m: f64, search_m: f64) -> f64 {
        let lo = self.origin_m;
        let hi = self.origin_m
            + DVec3::new(
                self.dims[0] as f64,
                self.dims[1] as f64,
                self.dims[2] as f64,
            ) * self.resolution_m;
        let to_face = (center_m - lo).min(hi - center_m).min_element();
        let mut best = (to_face - radius_m).min(search_m);
        if best <= 0.0 {
            return best;
        }
        let reach = radius_m + best;
        let cmin = ((center_m - DVec3::splat(reach) - lo) / self.resolution_m)
            .floor()
            .max(DVec3::ZERO);
        let cmax = ((center_m + DVec3::splat(reach) - lo) / self.resolution_m).floor();
        let half = DVec3::splat(self.resolution_m * 0.5);
        for z in cmin.z as usize..=(cmax.z as usize).min(self.dims[2] - 1) {
            for y in cmin.y as usize..=(cmax.y as usize).min(self.dims[1] - 1) {
                for x in cmin.x as usize..=(cmax.x as usize).min(self.dims[0] - 1) {
                    if !self.occupied[self.index([x, y, z])] {
                        continue;
                    }
                    let c = self.cell_center([x, y, z]);
                    let d = ((center_m - c).abs() - half).max(DVec3::ZERO).length();
                    best = best.min(d - radius_m);
                }
            }
        }
        best
    }

    /// A copy with every cell within `radius_m` of an occupied cell's centre
    /// marked occupied (configuration-space inflation for a spherical robot).
    #[must_use]
    pub fn inflated(&self, radius_m: f64) -> Self {
        let mut out = self.clone();
        let r = (radius_m / self.resolution_m).ceil() as i64;
        let r2 = (radius_m / self.resolution_m).powi(2);
        let d = self.dims.map(|v| v as i64);
        for z in 0..d[2] {
            for y in 0..d[1] {
                for x in 0..d[0] {
                    if !self.occupied[self.index([x as usize, y as usize, z as usize])] {
                        continue;
                    }
                    for dz in -r..=r {
                        for dy in -r..=r {
                            for dx in -r..=r {
                                if ((dx * dx + dy * dy + dz * dz) as f64) > r2 {
                                    continue;
                                }
                                let (nx, ny, nz) = (x + dx, y + dy, z + dz);
                                if nx >= 0
                                    && ny >= 0
                                    && nz >= 0
                                    && nx < d[0]
                                    && ny < d[1]
                                    && nz < d[2]
                                {
                                    let i = out.index([nx as usize, ny as usize, nz as usize]);
                                    out.occupied[i] = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> VoxelGrid {
        VoxelGrid::new(DVec3::ZERO, 0.5, [10, 10, 4]).unwrap()
    }

    #[test]
    fn cell_lookup_roundtrips() {
        let g = grid();
        let c = g.cell_of(DVec3::new(1.2, 3.9, 0.6)).unwrap();
        assert_eq!(c, [2, 7, 1]);
        assert_eq!(g.cell_of(g.cell_center(c)), Some(c));
        assert_eq!(g.cell_of(DVec3::new(-0.1, 0.0, 0.0)), None);
        assert_eq!(g.cell_of(DVec3::new(0.0, 0.0, 2.0)), None);
    }

    #[test]
    fn sphere_checks_boxes_and_bounds() {
        let mut g = grid();
        g.fill_box(DVec3::new(2.0, 2.0, 0.0), DVec3::new(3.0, 3.0, 2.0));
        assert!(g.occupied_count() > 0);
        assert!(g.sphere_collides(DVec3::new(2.5, 2.5, 1.0), 0.1));
        assert!(!g.sphere_collides(DVec3::new(1.0, 1.0, 1.0), 0.2));
        assert!(
            g.sphere_collides(DVec3::new(0.1, 1.0, 1.0), 0.2),
            "grid face"
        );
        let clear = g.sphere_clearance(DVec3::new(1.0, 2.75, 1.0), 0.2, 5.0);
        assert!((clear - 0.8).abs() < 1e-9, "{clear}");
    }

    #[test]
    fn inflation_grows_obstacles() {
        let mut g = grid();
        g.set_occupied_cell([5, 5, 2], true);
        let inf = g.inflated(0.5);
        assert!(inf.is_occupied_cell([6, 5, 2]));
        assert!(!inf.is_occupied_cell([6, 6, 3]));
        assert!(!inf.is_occupied_cell([7, 5, 2]));
    }
}
