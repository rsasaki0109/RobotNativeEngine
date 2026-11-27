//! Frontier extraction: free cells that border unknown space.

use rne_math::Vec3;
use rne_nav::{GridCoord, OccupancyGrid};

const NEIGHBOURS_4: [(isize, isize); 4] = [(0, -1), (-1, 0), (1, 0), (0, 1)];
const NEIGHBOURS_8: [(isize, isize); 8] = [
    (-1, -1),
    (0, -1),
    (1, -1),
    (-1, 0),
    (1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
];

/// Settings of [`extract_frontiers`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrontierConfig {
    /// Clusters with fewer cells are dropped as noise.
    pub min_cluster_cells: usize,
}

impl Default for FrontierConfig {
    fn default() -> Self {
        Self {
            min_cluster_cells: 5,
        }
    }
}

/// An 8-connected cluster of frontier cells.
#[derive(Clone, Debug, PartialEq)]
pub struct Frontier {
    /// The cluster's cells in row-major order.
    pub cells: Vec<GridCoord>,
    /// Mean world position of the cells' centres (z = 0).
    pub centroid_m: Vec3,
}

impl Frontier {
    /// Number of cells in the cluster.
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// Whether the cluster has no cells.
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }
}

/// Whether `coord` is a frontier cell: free, with an in-grid unknown
/// 4-neighbour. Cells outside the grid do not count as unknown, so the grid
/// border is not a frontier.
pub(crate) fn is_frontier_cell(grid: &OccupancyGrid, coord: GridCoord) -> bool {
    grid.is_free(coord)
        && NEIGHBOURS_4.iter().any(|(dx, dy)| {
            let n = GridCoord {
                x: coord.x + dx,
                y: coord.y + dy,
            };
            grid.contains(n) && !grid.is_known(n)
        })
}

/// Finds the frontier clusters of `grid`.
///
/// Clusters are found by flood fill from the first unvisited frontier cell in
/// row-major order, so their order and contents are deterministic. Clusters
/// smaller than [`FrontierConfig::min_cluster_cells`] are dropped.
pub fn extract_frontiers(grid: &OccupancyGrid, config: &FrontierConfig) -> Vec<Frontier> {
    let width = grid.width();
    let height = grid.height();
    let index = |c: GridCoord| c.y as usize * width + c.x as usize;

    let mut frontier = vec![false; width * height];
    for y in 0..height {
        for x in 0..width {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            frontier[index(coord)] = is_frontier_cell(grid, coord);
        }
    }

    let mut visited = vec![false; width * height];
    let mut clusters = Vec::new();
    for y in 0..height {
        for x in 0..width {
            let seed = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            if !frontier[index(seed)] || visited[index(seed)] {
                continue;
            }
            visited[index(seed)] = true;
            let mut stack = vec![seed];
            let mut cells = Vec::new();
            while let Some(cell) = stack.pop() {
                cells.push(cell);
                for (dx, dy) in NEIGHBOURS_8 {
                    let n = GridCoord {
                        x: cell.x + dx,
                        y: cell.y + dy,
                    };
                    if grid.contains(n) && frontier[index(n)] && !visited[index(n)] {
                        visited[index(n)] = true;
                        stack.push(n);
                    }
                }
            }
            if cells.len() < config.min_cluster_cells.max(1) {
                continue;
            }
            cells.sort_by_key(|c| (c.y, c.x));
            let sum = cells
                .iter()
                .fold(Vec3::ZERO, |acc, &c| acc + grid.grid_to_world(c));
            clusters.push(Frontier {
                centroid_m: sum / cells.len() as f64,
                cells,
            });
        }
    }
    clusters
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reach::tests::room_grid;

    #[test]
    fn closed_room_has_no_frontier() {
        // Every cell known: walls around free space.
        let grid = room_grid(&[
            "#######", //
            "#.....#", //
            "#.....#", //
            "#######",
        ]);
        assert!(extract_frontiers(&grid, &FrontierConfig::default()).is_empty());
    }

    #[test]
    fn open_doorway_is_one_frontier() {
        let grid = room_grid(&[
            "#########", //
            "#.......?", //
            "#.......?", //
            "#.......?", //
            "#########",
        ]);
        let config = FrontierConfig {
            min_cluster_cells: 1,
        };
        let frontiers = extract_frontiers(&grid, &config);
        assert_eq!(frontiers.len(), 1);
        let cells: Vec<_> = frontiers[0].cells.iter().map(|c| (c.x, c.y)).collect();
        assert_eq!(cells, vec![(7, 1), (7, 2), (7, 3)]);
    }

    #[test]
    fn small_clusters_are_dropped() {
        let grid = room_grid(&[
            "#####?###", //
            "#.......#", //
            "#.......?", //
            "#.......?", //
            "#.......?", //
            "#########",
        ]);
        let frontiers = extract_frontiers(
            &grid,
            &FrontierConfig {
                min_cluster_cells: 2,
            },
        );
        assert_eq!(frontiers.len(), 1);
        assert_eq!(frontiers[0].len(), 3);
        let all = extract_frontiers(
            &grid,
            &FrontierConfig {
                min_cluster_cells: 1,
            },
        );
        assert_eq!(all.len(), 2);
        // Row-major from y = 0: the right-hand cluster (y = 1..=3) comes
        // before the one under the top gap (y = 4).
        assert_eq!(all[0].cells[0], GridCoord { x: 7, y: 1 });
        assert_eq!(all[1].cells, vec![GridCoord { x: 5, y: 4 }]);
    }
}
