//! Scoring frontiers and choosing the next goal.

use crate::frontier::{extract_frontiers, FrontierConfig};
use crate::gain::{information_gain, SensorModel};
use crate::reach::ReachMap;
use rne_math::Vec3;
use rne_nav::{Costmap, CostmapConfig, GridCoord, GridError, OccupancyGrid, Pose2d};
use std::cmp::Ordering;

/// How a frontier's utility is computed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScorePolicy {
    /// The closest reachable frontier by path length (Yamauchi 1997). The
    /// baseline: `utility = -path_m`.
    NearestFrontier,
    /// Unknown area seen from the goal against the drive there:
    /// `utility = gain_m2 - distance_weight * path_m`.
    InformationGain {
        /// Square metres of new area one metre of driving must buy.
        distance_weight: f64,
    },
}

impl Default for ScorePolicy {
    fn default() -> Self {
        Self::InformationGain {
            distance_weight: 1.0,
        }
    }
}

/// Settings of [`Explorer`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExplorerConfig {
    /// Frontier clustering.
    pub frontier: FrontierConfig,
    /// The sensor whose view gives the information gain.
    pub sensor: SensorModel,
    /// Inflation of the costmap that decides which cells the robot can reach.
    pub costmap: CostmapConfig,
    /// Scoring.
    pub policy: ScorePolicy,
    /// Frontiers whose gain is below this (square metres) are ignored; when
    /// none is left, exploration is done.
    pub min_gain_m2: f64,
    /// A new candidate replaces the current goal only if its utility is
    /// higher by more than this.
    pub switch_margin: f64,
    /// A candidate within this distance of the current goal counts as the
    /// current goal (frontiers move as they are explored).
    pub hysteresis_radius_m: f64,
}

impl Default for ExplorerConfig {
    fn default() -> Self {
        Self {
            frontier: FrontierConfig::default(),
            sensor: SensorModel::default(),
            costmap: CostmapConfig::default(),
            policy: ScorePolicy::default(),
            min_gain_m2: 0.25,
            switch_margin: 0.5,
            hysteresis_radius_m: 1.0,
        }
    }
}

/// A reachable frontier with its goal and score.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Candidate {
    /// Goal pose: on the frontier, facing away from the robot.
    pub goal: Pose2d,
    /// Grid cell of the goal.
    pub goal_cell: GridCoord,
    /// Path length from the robot in metres.
    pub path_m: f64,
    /// Unknown area visible from the goal, square metres.
    pub gain_m2: f64,
    /// Number of cells of the frontier cluster.
    pub frontier_cells: usize,
    /// Score under the [`ScorePolicy`]; higher is better.
    pub utility: f64,
}

/// What the robot should do next.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Decision {
    /// Drive to this candidate's goal.
    Goal(Candidate),
    /// No reachable frontier with enough gain is left.
    Done,
}

/// Chooses exploration goals, keeping the current one unless another is
/// clearly better.
#[derive(Clone, Debug, PartialEq)]
pub struct Explorer {
    config: ExplorerConfig,
    current: Option<Candidate>,
}

impl Explorer {
    /// Creates an explorer with no current goal.
    pub fn new(config: ExplorerConfig) -> Self {
        Self {
            config,
            current: None,
        }
    }

    /// The settings.
    pub fn config(&self) -> &ExplorerConfig {
        &self.config
    }

    /// The goal chosen by the last [`Explorer::decide`], if any.
    pub fn current(&self) -> Option<&Candidate> {
        self.current.as_ref()
    }

    /// Forgets the current goal (for example once it is reached or its path
    /// is blocked), so the next decision starts afresh.
    pub fn clear_goal(&mut self) {
        self.current = None;
    }

    /// Every reachable frontier with enough gain, best first. Ties are broken
    /// by shorter path, then by goal cell.
    pub fn candidates(
        &self,
        grid: &OccupancyGrid,
        robot: Pose2d,
    ) -> Result<Vec<Candidate>, GridError> {
        let costmap = Costmap::from_occupancy(grid, &self.config.costmap)?;
        let robot_m = Vec3::new(robot.x_m, robot.y_m, 0.0);
        let Some(reach) = ReachMap::compute(&costmap, robot_m) else {
            return Ok(Vec::new());
        };
        let mut candidates = Vec::new();
        for frontier in extract_frontiers(grid, &self.config.frontier) {
            // The reachable cluster cell nearest the centroid.
            let goal = frontier
                .cells
                .iter()
                .filter_map(|&cell| reach.distance_m(cell).map(|d| (cell, d)))
                .min_by(|a, b| {
                    let da = grid
                        .grid_to_world(a.0)
                        .distance_squared(frontier.centroid_m);
                    let db = grid
                        .grid_to_world(b.0)
                        .distance_squared(frontier.centroid_m);
                    da.total_cmp(&db).then_with(|| a.0.cmp(&b.0))
                });
            let Some((cell, path_m)) = goal else {
                continue;
            };
            let position = grid.grid_to_world(cell);
            let (dx, dy) = (position.x - robot.x_m, position.y - robot.y_m);
            let yaw = if dx == 0.0 && dy == 0.0 {
                robot.yaw_rad
            } else {
                dy.atan2(dx)
            };
            let pose = Pose2d::new(position.x, position.y, yaw);
            let gain_m2 = information_gain(grid, pose, &self.config.sensor);
            if gain_m2 < self.config.min_gain_m2 {
                continue;
            }
            let utility = match self.config.policy {
                ScorePolicy::NearestFrontier => -path_m,
                ScorePolicy::InformationGain { distance_weight } => {
                    gain_m2 - distance_weight * path_m
                }
            };
            candidates.push(Candidate {
                goal: pose,
                goal_cell: cell,
                path_m,
                gain_m2,
                frontier_cells: frontier.len(),
                utility,
            });
        }
        candidates.sort_by(compare_candidates);
        Ok(candidates)
    }

    /// Chooses the next goal for a robot at `robot` on `grid`.
    ///
    /// The best candidate replaces the current goal only when its utility is
    /// higher by more than [`ExplorerConfig::switch_margin`]; otherwise the
    /// candidate nearest the current goal (within
    /// [`ExplorerConfig::hysteresis_radius_m`]) is kept.
    pub fn decide(&mut self, grid: &OccupancyGrid, robot: Pose2d) -> Result<Decision, GridError> {
        let candidates = self.candidates(grid, robot)?;
        let Some(best) = candidates.first().copied() else {
            self.current = None;
            return Ok(Decision::Done);
        };
        let kept = self.current.and_then(|current| {
            let at = Vec3::new(current.goal.x_m, current.goal.y_m, 0.0);
            candidates
                .iter()
                .map(|c| (c, Vec3::new(c.goal.x_m, c.goal.y_m, 0.0).distance(at)))
                .filter(|(_, d)| *d <= self.config.hysteresis_radius_m)
                .min_by(|a, b| {
                    a.1.total_cmp(&b.1)
                        .then_with(|| a.0.goal_cell.cmp(&b.0.goal_cell))
                })
                .map(|(c, _)| *c)
        });
        let chosen = match kept {
            Some(kept) if best.utility <= kept.utility + self.config.switch_margin => kept,
            _ => best,
        };
        self.current = Some(chosen);
        Ok(Decision::Goal(chosen))
    }
}

fn compare_candidates(a: &Candidate, b: &Candidate) -> Ordering {
    b.utility
        .total_cmp(&a.utility)
        .then_with(|| a.path_m.total_cmp(&b.path_m))
        .then_with(|| a.goal_cell.cmp(&b.goal_cell))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reach::tests::room_grid;

    fn config(policy: ScorePolicy) -> ExplorerConfig {
        ExplorerConfig {
            frontier: FrontierConfig {
                min_cluster_cells: 1,
            },
            sensor: SensorModel::lidar(6.0),
            costmap: CostmapConfig {
                inscribed_radius_m: 0.0,
                inflation_radius_m: 0.0,
                ..CostmapConfig::default()
            },
            policy,
            min_gain_m2: 0.5,
            switch_margin: 0.5,
            hysteresis_radius_m: 1.0,
        }
    }

    /// A room with two openings: near the robot, a gap in the bottom wall
    /// onto a dead end of two unknown cells; far from it, a doorway in the
    /// right wall onto a large unknown hall.
    fn two_doors() -> OccupancyGrid {
        room_grid(&[
            "#########################", // y = 11
            "#.........#??????????????", //
            "#.........#??????????????", //
            "#.........#??????????????", //
            "#.........???????????????", // y = 7
            "#.........???????????????", // y = 6
            "#.........#??????????????", //
            "#.........#??????????????", //
            "#.........#??????????????", // y = 3
            "#?#########??????????????", // y = 2
            "#?#######################", // y = 1
            "#########################", // y = 0
        ])
    }

    #[test]
    fn closed_room_is_done() {
        let grid = room_grid(&[
            "#####", //
            "#...#", //
            "#####",
        ]);
        let mut explorer = Explorer::new(config(ScorePolicy::default()));
        assert_eq!(
            explorer.decide(&grid, Pose2d::new(2.5, 1.5, 0.0)).unwrap(),
            Decision::Done
        );
    }

    #[test]
    fn nearest_frontier_takes_the_dead_end_gain_takes_the_hall() {
        let grid = two_doors();
        let robot = Pose2d::new(2.5, 4.5, 0.0);

        let nearest = Explorer::new(config(ScorePolicy::NearestFrontier))
            .candidates(&grid, robot)
            .unwrap();
        assert_eq!(nearest.len(), 2);
        // The dead end below the room is closer.
        assert_eq!(nearest[0].goal_cell, GridCoord { x: 1, y: 3 });

        let gain = Explorer::new(config(ScorePolicy::InformationGain {
            distance_weight: 0.1,
        }))
        .candidates(&grid, robot)
        .unwrap();
        // The hall on the right opens far more unknown space.
        assert_eq!(gain[0].goal_cell.x, 9);
        assert!(gain[0].gain_m2 > gain[1].gain_m2);
    }

    #[test]
    fn hysteresis_keeps_the_current_goal() {
        let grid = two_doors();
        let mut explorer = Explorer::new(ExplorerConfig {
            switch_margin: 1.0e6,
            ..config(ScorePolicy::NearestFrontier)
        });
        let Decision::Goal(first) = explorer.decide(&grid, Pose2d::new(2.5, 4.5, 0.0)).unwrap()
        else {
            panic!("expected a goal");
        };
        // From the far side of the room the hall is nearer, but the large
        // margin keeps the first goal.
        let Decision::Goal(second) = explorer.decide(&grid, Pose2d::new(8.5, 9.5, 0.0)).unwrap()
        else {
            panic!("expected a goal");
        };
        assert_eq!(first.goal_cell, second.goal_cell);

        explorer.clear_goal();
        let Decision::Goal(fresh) = explorer.decide(&grid, Pose2d::new(8.5, 9.5, 0.0)).unwrap()
        else {
            panic!("expected a goal");
        };
        assert_ne!(fresh.goal_cell, first.goal_cell);
    }

    #[test]
    fn decisions_are_repeatable() {
        let grid = two_doors();
        let robot = Pose2d::new(4.5, 4.5, 0.3);
        let a = Explorer::new(config(ScorePolicy::default()))
            .candidates(&grid, robot)
            .unwrap();
        let b = Explorer::new(config(ScorePolicy::default()))
            .candidates(&grid, robot)
            .unwrap();
        assert_eq!(a, b);
    }
}
