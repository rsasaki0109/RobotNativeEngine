//! Information gain: the unknown area a range sensor would see.

use rne_math::Vec3;
use rne_nav::{GridCoord, OccupancyGrid, Pose2d};

/// A planar range sensor for gain estimation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorModel {
    /// Maximum range in metres.
    pub range_m: f64,
    /// Horizontal field of view in radians, centred on the pose's yaw.
    /// `2π` or more is a 360° sensor.
    pub fov_rad: f64,
    /// Number of rays cast across the field of view.
    pub rays: usize,
}

impl SensorModel {
    /// A 360° 2D lidar.
    pub fn lidar(range_m: f64) -> Self {
        Self {
            range_m,
            fov_rad: std::f64::consts::TAU,
            rays: 360,
        }
    }

    /// A forward camera with the given horizontal field of view.
    pub fn camera(range_m: f64, fov_rad: f64) -> Self {
        Self {
            range_m,
            fov_rad,
            rays: 90,
        }
    }

    fn is_omnidirectional(&self) -> bool {
        self.fov_rad >= std::f64::consts::TAU
    }
}

impl Default for SensorModel {
    fn default() -> Self {
        Self::lidar(4.0)
    }
}

/// Unknown area in square metres that `sensor` would observe from `pose`.
///
/// Rays pass through unknown cells (optimistically, as free space) and stop at
/// the first occupied cell. Each unknown cell counts once.
pub fn information_gain(grid: &OccupancyGrid, pose: Pose2d, sensor: &SensorModel) -> f64 {
    let rays = sensor.rays.max(1);
    let width = grid.width();
    let mut seen = vec![false; width * grid.height()];
    let mut unknown = 0usize;
    let origin = Vec3::new(pose.x_m, pose.y_m, 0.0);
    for i in 0..rays {
        let angle = if sensor.is_omnidirectional() {
            pose.yaw_rad + std::f64::consts::TAU * i as f64 / rays as f64
        } else if rays == 1 {
            pose.yaw_rad
        } else {
            pose.yaw_rad - 0.5 * sensor.fov_rad + sensor.fov_rad * i as f64 / (rays - 1) as f64
        };
        let end = origin + Vec3::new(angle.cos(), angle.sin(), 0.0) * sensor.range_m;
        for cell in grid.raycast(origin, end) {
            if grid.is_occupied(cell) {
                break;
            }
            let index = cell_index(width, cell);
            if !seen[index] {
                seen[index] = true;
                if !grid.is_known(cell) {
                    unknown += 1;
                }
            }
        }
    }
    unknown as f64 * grid.resolution_m() * grid.resolution_m()
}

fn cell_index(width: usize, coord: GridCoord) -> usize {
    coord.y as usize * width + coord.x as usize
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reach::tests::room_grid;

    #[test]
    fn known_room_has_no_gain() {
        let grid = room_grid(&[
            "#####", //
            "#...#", //
            "#####",
        ]);
        let gain = information_gain(&grid, Pose2d::new(2.5, 1.5, 0.0), &SensorModel::lidar(5.0));
        assert_eq!(gain, 0.0);
    }

    #[test]
    fn walls_block_the_view() {
        // Unknown space on both sides; the right side is behind a wall.
        let grid = room_grid(&[
            "?????#?????", //
            "?????#?????", //
            "??...#.....", //
            "?????#?????", //
            "?????#?????",
        ]);
        let pose = Pose2d::new(3.5, 2.5, 0.0);
        let sensor = SensorModel::lidar(10.0);
        let gain = information_gain(&grid, pose, &sensor);
        assert!(gain > 0.0);
        // Nothing right of the wall (x >= 6) can be seen from the left.
        let mut walled = grid.clone();
        for y in 0..5 {
            for x in 6..11 {
                let c = GridCoord { x, y };
                for _ in 0..3 {
                    walled.mark_free(c);
                }
            }
        }
        assert_eq!(information_gain(&walled, pose, &sensor), gain);
    }

    #[test]
    fn camera_sees_only_ahead() {
        let grid = room_grid(&[
            "???????????", //
            "???????????", //
            "?????.?????", //
            "???????????", //
            "???????????",
        ]);
        let camera = SensorModel::camera(3.0, 60f64.to_radians());
        let facing_right = information_gain(&grid, Pose2d::new(5.5, 2.5, 0.0), &camera);
        let lidar = information_gain(&grid, Pose2d::new(5.5, 2.5, 0.0), &SensorModel::lidar(3.0));
        assert!(facing_right > 0.0);
        assert!(facing_right < lidar);
    }
}
