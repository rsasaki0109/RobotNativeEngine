//! Path length from the robot to every reachable cell.

use rne_math::Vec3;
use rne_nav::{Costmap, GridCoord, COST_INSCRIBED, COST_NO_INFORMATION};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f64::consts::SQRT_2;

/// Path lengths in metres from one start cell to every cell reachable from
/// it, by Dijkstra over the 8-connected grid.
///
/// A cell is traversable when it is known and its cost is below
/// [`COST_INSCRIBED`]: the robot's footprint fits there. Unknown cells are not
/// traversable, so a goal is only reachable through space already mapped. The
/// start cell is always expanded, so a robot standing in an inscribed cell
/// (close to a wall) can still drive away.
#[derive(Clone, Debug, PartialEq)]
pub struct ReachMap {
    width: usize,
    height: usize,
    distance_m: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct Node {
    distance_m: f64,
    coord: GridCoord,
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap on distance, ties by coordinate.
        other
            .distance_m
            .total_cmp(&self.distance_m)
            .then_with(|| other.coord.cmp(&self.coord))
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub(crate) fn traversable(costmap: &Costmap, coord: GridCoord) -> bool {
    match costmap.cost_at(coord) {
        Some(COST_NO_INFORMATION) | None => false,
        Some(cost) => cost < COST_INSCRIBED,
    }
}

impl ReachMap {
    /// Computes path lengths from the cell under `start_m`. Returns `None`
    /// when `start_m` is outside the costmap.
    pub fn compute(costmap: &Costmap, start_m: Vec3) -> Option<Self> {
        let start = costmap.world_to_grid(start_m)?;
        let width = costmap.width();
        let height = costmap.height();
        let resolution = costmap.resolution_m();
        let index = |c: GridCoord| c.y as usize * width + c.x as usize;

        let mut distance_m = vec![f64::INFINITY; width * height];
        let mut heap = BinaryHeap::new();
        distance_m[index(start)] = 0.0;
        heap.push(Node {
            distance_m: 0.0,
            coord: start,
        });
        while let Some(Node {
            distance_m: d,
            coord,
        }) = heap.pop()
        {
            if d > distance_m[index(coord)] {
                continue;
            }
            for dy in -1..=1isize {
                for dx in -1..=1isize {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let n = GridCoord {
                        x: coord.x + dx,
                        y: coord.y + dy,
                    };
                    if !traversable(costmap, n) {
                        continue;
                    }
                    if dx != 0 && dy != 0 {
                        // No corner cutting between two blocked cells.
                        let side_a = GridCoord {
                            x: coord.x + dx,
                            y: coord.y,
                        };
                        let side_b = GridCoord {
                            x: coord.x,
                            y: coord.y + dy,
                        };
                        if !traversable(costmap, side_a) && !traversable(costmap, side_b) {
                            continue;
                        }
                    }
                    let step = if dx != 0 && dy != 0 { SQRT_2 } else { 1.0 } * resolution;
                    let candidate = d + step;
                    if candidate < distance_m[index(n)] {
                        distance_m[index(n)] = candidate;
                        heap.push(Node {
                            distance_m: candidate,
                            coord: n,
                        });
                    }
                }
            }
        }
        Some(Self {
            width,
            height,
            distance_m,
        })
    }

    /// Path length to `coord` in metres, or `None` when it is unreachable or
    /// outside the grid.
    pub fn distance_m(&self, coord: GridCoord) -> Option<f64> {
        if coord.x < 0
            || coord.y < 0
            || coord.x as usize >= self.width
            || coord.y as usize >= self.height
        {
            return None;
        }
        let d = self.distance_m[coord.y as usize * self.width + coord.x as usize];
        d.is_finite().then_some(d)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use rne_nav::{CostmapConfig, OccupancyGrid, Pose2d};

    /// Builds a 1 m grid from rows drawn top (high y) to bottom: `#`
    /// occupied, `.` free, `?` unknown. Row 0 of the drawing is the top.
    pub(crate) fn room_grid(rows: &[&str]) -> OccupancyGrid {
        let height = rows.len();
        let width = rows[0].len();
        let mut grid = OccupancyGrid::new(width, height, 1.0, Pose2d::IDENTITY).unwrap();
        for (row, line) in rows.iter().enumerate() {
            let y = (height - 1 - row) as isize;
            for (x, ch) in line.chars().enumerate() {
                let coord = GridCoord { x: x as isize, y };
                match ch {
                    '#' => {
                        for _ in 0..3 {
                            grid.mark_occupied(coord);
                        }
                    }
                    '.' => {
                        for _ in 0..3 {
                            grid.mark_free(coord);
                        }
                    }
                    _ => {}
                }
            }
        }
        grid
    }

    /// A costmap without inflation, so the drawn walls are the only
    /// obstacles.
    pub(crate) fn plain_costmap(grid: &OccupancyGrid) -> Costmap {
        Costmap::from_occupancy(
            grid,
            &CostmapConfig {
                inscribed_radius_m: 0.0,
                inflation_radius_m: 0.0,
                ..CostmapConfig::default()
            },
        )
        .unwrap()
    }

    #[test]
    fn distances_follow_the_corridor_not_the_straight_line() {
        // A U-shaped corridor: the goal is 2 m away in a straight line but 8
        // m around the wall.
        let grid = room_grid(&[
            "#######", //
            "#.....#", //
            "####..#", //
            "#.....#", //
            "#######",
        ]);
        let costmap = plain_costmap(&grid);
        let reach = ReachMap::compute(&costmap, Vec3::new(1.5, 1.5, 0.0)).unwrap();
        assert_eq!(reach.distance_m(GridCoord { x: 1, y: 1 }), Some(0.0));
        let around = reach.distance_m(GridCoord { x: 1, y: 3 }).unwrap();
        assert!(around > 5.0, "went through the wall: {around}");
        assert_eq!(reach.distance_m(GridCoord { x: 0, y: 0 }), None);
    }

    #[test]
    fn unknown_space_is_not_traversed() {
        let grid = room_grid(&[
            "#####", //
            "#.?.#", //
            "#####",
        ]);
        let costmap = plain_costmap(&grid);
        let reach = ReachMap::compute(&costmap, Vec3::new(1.5, 1.5, 0.0)).unwrap();
        assert_eq!(reach.distance_m(GridCoord { x: 3, y: 1 }), None);
    }
}
