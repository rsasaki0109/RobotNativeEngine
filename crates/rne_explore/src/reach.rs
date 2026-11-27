//! Path length from the robot to every reachable cell.

use rne_math::Vec3;
use rne_nav::{Costmap, GridCoord, COST_INSCRIBED, COST_LETHAL, COST_NO_INFORMATION};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::f64::consts::SQRT_2;

/// Path lengths in metres from one start cell to every cell reachable from
/// it, by Dijkstra over the 8-connected grid.
///
/// A cell is traversable when it is known and its cost is below
/// [`COST_INSCRIBED`]: the robot's footprint fits there. Unknown cells are not
/// traversable, so a goal is only reachable through space already mapped.
///
/// Near the start the rule is relaxed: within `escape_radius_m` of path
/// length, inscribed (but not lethal) cells are traversable too. A robot whose
/// estimate has ended up close to a wall can then still drive away from it,
/// instead of finding nothing reachable.
#[derive(Clone, Debug, PartialEq)]
pub struct ReachMap {
    width: usize,
    height: usize,
    distance_m: Vec<f64>,
    /// Index of the cell each cell was reached from (`usize::MAX`: none).
    came_from: Vec<usize>,
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

fn traversable(costmap: &Costmap, coord: GridCoord, escaping: bool) -> bool {
    match costmap.cost_at(coord) {
        Some(COST_NO_INFORMATION) | None => false,
        Some(cost) if escaping => cost < COST_LETHAL,
        Some(cost) => cost < COST_INSCRIBED,
    }
}

impl ReachMap {
    /// Computes path lengths from the cell under `start_m`, relaxing the
    /// inscribed rule within `escape_radius_m` of it. Returns `None` when
    /// `start_m` is outside the costmap.
    pub fn compute(costmap: &Costmap, start_m: Vec3, escape_radius_m: f64) -> Option<Self> {
        let start = costmap.world_to_grid(start_m)?;
        let width = costmap.width();
        let height = costmap.height();
        let resolution = costmap.resolution_m();
        let index = |c: GridCoord| c.y as usize * width + c.x as usize;

        let mut distance_m = vec![f64::INFINITY; width * height];
        let mut came_from = vec![usize::MAX; width * height];
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
                    let step = if dx != 0 && dy != 0 { SQRT_2 } else { 1.0 } * resolution;
                    let candidate = d + step;
                    let escaping = candidate <= escape_radius_m;
                    if !traversable(costmap, n, escaping) {
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
                        if !traversable(costmap, side_a, escaping)
                            && !traversable(costmap, side_b, escaping)
                        {
                            continue;
                        }
                    }
                    if candidate < distance_m[index(n)] {
                        distance_m[index(n)] = candidate;
                        came_from[index(n)] = index(coord);
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
            came_from,
        })
    }

    /// The shortest path from the start to `coord`, start first, or `None`
    /// when `coord` is unreachable.
    pub fn path_to(&self, coord: GridCoord) -> Option<Vec<GridCoord>> {
        self.distance_m(coord)?;
        let mut cells = vec![coord];
        let mut index = coord.y as usize * self.width + coord.x as usize;
        while self.came_from[index] != usize::MAX {
            index = self.came_from[index];
            cells.push(GridCoord {
                x: (index % self.width) as isize,
                y: (index / self.width) as isize,
            });
        }
        cells.reverse();
        Some(cells)
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
        let reach = ReachMap::compute(&costmap, Vec3::new(1.5, 1.5, 0.0), 0.0).unwrap();
        assert_eq!(reach.distance_m(GridCoord { x: 1, y: 1 }), Some(0.0));
        let around = reach.distance_m(GridCoord { x: 1, y: 3 }).unwrap();
        assert!(around > 5.0, "went through the wall: {around}");
        let path = reach.path_to(GridCoord { x: 1, y: 3 }).unwrap();
        assert_eq!(path.first(), Some(&GridCoord { x: 1, y: 1 }));
        assert_eq!(path.last(), Some(&GridCoord { x: 1, y: 3 }));
        assert!(path.iter().all(|c| grid.is_free(*c)));
        assert_eq!(reach.distance_m(GridCoord { x: 0, y: 0 }), None);
    }

    #[test]
    fn a_robot_next_to_a_wall_can_escape() {
        let grid = room_grid(&[
            "########", //
            "#......#", //
            "#......#", //
            "#......#", //
            "########",
        ]);
        // 1 m inscribed radius: every cell next to a wall is inscribed.
        let costmap = Costmap::from_occupancy(
            &grid,
            &CostmapConfig {
                inscribed_radius_m: 1.0,
                inflation_radius_m: 1.0,
                ..CostmapConfig::default()
            },
        )
        .unwrap();
        let start = Vec3::new(1.5, 1.5, 0.0);
        let centre = GridCoord { x: 3, y: 2 };
        let stuck = ReachMap::compute(&costmap, start, 0.0).unwrap();
        assert_eq!(stuck.distance_m(centre), None);
        let escaped = ReachMap::compute(&costmap, start, 1.5).unwrap();
        assert!(escaped.distance_m(centre).is_some());
    }

    #[test]
    fn unknown_space_is_not_traversed() {
        let grid = room_grid(&[
            "#####", //
            "#.?.#", //
            "#####",
        ]);
        let costmap = plain_costmap(&grid);
        let reach = ReachMap::compute(&costmap, Vec3::new(1.5, 1.5, 0.0), 0.0).unwrap();
        assert_eq!(reach.distance_m(GridCoord { x: 3, y: 1 }), None);
    }
}
