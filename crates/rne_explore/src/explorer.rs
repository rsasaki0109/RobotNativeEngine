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
    /// Within this path length of the robot, inscribed cells are traversable
    /// (see [`ReachMap::compute`]), so a robot close to a wall can leave.
    pub escape_radius_m: f64,
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
            escape_radius_m: 0.5,
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
    /// Drive back to a place visited long ago, to close a loop (see
    /// [`Explorer::decide_with_history`]). The candidate's `gain_m2` is zero.
    Revisit(Candidate),
    /// No reachable frontier with enough gain is left.
    Done,
}

/// Chooses exploration goals, keeping the current one unless another is
/// clearly better.
#[derive(Clone, Debug, PartialEq)]
pub struct Explorer {
    config: ExplorerConfig,
    current: Option<Candidate>,
    /// Reachability from the robot at the last decision.
    reach: Option<ReachMap>,
    revisit: Option<RevisitConfig>,
    /// The revisit goal being driven to.
    current_revisit: Option<Candidate>,
    /// Revisit goals reached without a loop closure since the open-loop
    /// distance last fell; not tried again until it does.
    tried: Vec<(f64, f64)>,
    last_open_loop_m: f64,
}

/// When and where [`Explorer::decide_with_history`] sends the robot back to
/// close a loop.
///
/// Odometry and scan-matching error grow with the distance driven since the
/// last loop closure, and a closure is only accepted while the error is still
/// small (a match far from the prediction is rejected as an outlier). So the
/// robot goes back to a place it saw long ago once it has driven
/// `open_loop_m` without a closure, instead of waiting for exploration to
/// bring it back by chance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RevisitConfig {
    /// Distance driven since the last loop closure that triggers a revisit.
    pub open_loop_m: f64,
    /// A visited place qualifies only if it lies at least this far back along
    /// the trajectory (closer places cannot close a loop).
    pub min_loop_m: f64,
    /// Places further than this by path from the robot are not considered.
    pub max_detour_m: f64,
    /// A place within this distance of a revisit tried without a closure is
    /// skipped.
    pub tried_radius_m: f64,
}

impl Default for RevisitConfig {
    fn default() -> Self {
        Self {
            open_loop_m: 15.0,
            min_loop_m: 6.0,
            max_detour_m: 8.0,
            tried_radius_m: 1.5,
        }
    }
}

impl Explorer {
    /// Creates an explorer with no current goal.
    pub fn new(config: ExplorerConfig) -> Self {
        Self {
            config,
            current: None,
            reach: None,
            revisit: None,
            current_revisit: None,
            tried: Vec::new(),
            last_open_loop_m: 0.0,
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
    /// is blocked), so the next decision starts afresh. A revisit goal being
    /// cleared counts as tried: it is not chosen again until a loop closes.
    pub fn clear_goal(&mut self) {
        self.current = None;
        if let Some(revisit) = self.current_revisit.take() {
            self.tried.push((revisit.goal.x_m, revisit.goal.y_m));
        }
    }

    /// Enables revisits for [`Explorer::decide_with_history`].
    pub fn with_revisits(mut self, config: RevisitConfig) -> Self {
        self.revisit = Some(config);
        self
    }

    /// Like [`Explorer::decide`], but goes back to close a loop once the
    /// robot has driven [`RevisitConfig::open_loop_m`] since the last
    /// closure.
    ///
    /// `history` is the trajectory so far, oldest first, in the map frame
    /// (for `rne_slam::Slam2d`, the pose graph's nodes); `open_loop_m` is the
    /// distance driven since the last loop closure, which the caller resets
    /// when one is accepted. Without [`Explorer::with_revisits`] this is
    /// [`Explorer::decide`].
    pub fn decide_with_history(
        &mut self,
        grid: &OccupancyGrid,
        robot: Pose2d,
        history: &[Pose2d],
        open_loop_m: f64,
    ) -> Result<Decision, GridError> {
        let (candidates, reach) = self.score(grid, robot)?;
        self.reach = reach;
        if let Some(config) = self.revisit {
            if open_loop_m < self.last_open_loop_m {
                // A loop closed since the last decision.
                self.tried.clear();
                self.current_revisit = None;
            }
            self.last_open_loop_m = open_loop_m;
            if open_loop_m >= config.open_loop_m {
                if let Some(target) = self.revisit_target(grid, history, &config) {
                    self.current_revisit = Some(target);
                    return Ok(Decision::Revisit(target));
                }
            } else {
                self.current_revisit = None;
            }
        }
        Ok(self.choose(candidates))
    }

    /// The visited place to go back to: old enough along the trajectory,
    /// reachable within the detour, not tried; the nearest by path.
    fn revisit_target(
        &self,
        grid: &OccupancyGrid,
        history: &[Pose2d],
        config: &RevisitConfig,
    ) -> Option<Candidate> {
        if let Some(current) = self.current_revisit {
            let cell = current.goal_cell;
            if let Some(path_m) = self.reach.as_ref().and_then(|r| r.distance_m(cell)) {
                return Some(Candidate { path_m, ..current });
            }
        }
        let reach = self.reach.as_ref()?;
        // Distance along the trajectory from each pose to its end.
        let mut behind = vec![0.0; history.len()];
        for i in (0..history.len().saturating_sub(1)).rev() {
            let (a, b) = (history[i], history[i + 1]);
            behind[i] = behind[i + 1] + (b.x_m - a.x_m).hypot(b.y_m - a.y_m);
        }
        history
            .iter()
            .zip(&behind)
            .filter(|(_, along)| **along >= config.min_loop_m)
            .filter(|(pose, _)| {
                !self
                    .tried
                    .iter()
                    .any(|(x, y)| (pose.x_m - x).hypot(pose.y_m - y) < config.tried_radius_m)
            })
            .filter_map(|(pose, _)| {
                let cell = grid.world_to_grid(Vec3::new(pose.x_m, pose.y_m, 0.0))?;
                let path_m = reach.distance_m(cell)?;
                (path_m <= config.max_detour_m && path_m > 0.0).then_some((cell, path_m, pose))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)))
            .map(|(cell, path_m, pose)| {
                let centre = grid.grid_to_world(cell);
                Candidate {
                    goal: Pose2d::new(centre.x, centre.y, pose.yaw_rad),
                    goal_cell: cell,
                    path_m,
                    gain_m2: 0.0,
                    frontier_cells: 0,
                    utility: -path_m,
                }
            })
    }

    /// The path the last [`Explorer::decide`] found to `goal_cell`, as world
    /// cell centres from the robot's cell to the goal. It runs through the
    /// same cells the explorer counted as reachable, so it exists for every
    /// candidate that decision returned.
    pub fn path_to(&self, grid: &OccupancyGrid, goal_cell: GridCoord) -> Option<Vec<Vec3>> {
        let cells = self.reach.as_ref()?.path_to(goal_cell)?;
        Some(cells.into_iter().map(|c| grid.grid_to_world(c)).collect())
    }

    /// Every reachable frontier with enough gain, best first. Ties are broken
    /// by shorter path, then by goal cell.
    pub fn candidates(
        &self,
        grid: &OccupancyGrid,
        robot: Pose2d,
    ) -> Result<Vec<Candidate>, GridError> {
        Ok(self.score(grid, robot)?.0)
    }

    fn score(
        &self,
        grid: &OccupancyGrid,
        robot: Pose2d,
    ) -> Result<(Vec<Candidate>, Option<ReachMap>), GridError> {
        let costmap = Costmap::from_occupancy(grid, &self.config.costmap)?;
        let robot_m = Vec3::new(robot.x_m, robot.y_m, 0.0);
        let Some(reach) = ReachMap::compute(&costmap, robot_m, self.config.escape_radius_m) else {
            return Ok((Vec::new(), None));
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
        Ok((candidates, Some(reach)))
    }

    /// Chooses the next goal for a robot at `robot` on `grid`.
    ///
    /// The best candidate replaces the current goal only when its utility is
    /// higher by more than [`ExplorerConfig::switch_margin`]; otherwise the
    /// candidate nearest the current goal (within
    /// [`ExplorerConfig::hysteresis_radius_m`]) is kept.
    pub fn decide(&mut self, grid: &OccupancyGrid, robot: Pose2d) -> Result<Decision, GridError> {
        let (candidates, reach) = self.score(grid, robot)?;
        self.reach = reach;
        Ok(self.choose(candidates))
    }

    /// The frontier decision over scored candidates, with hysteresis.
    fn choose(&mut self, candidates: Vec<Candidate>) -> Decision {
        let Some(best) = candidates.first().copied() else {
            self.current = None;
            return Decision::Done;
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
        Decision::Goal(chosen)
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
            escape_radius_m: 0.0,
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

    /// A trajectory down the room and back up the other side, as the pose
    /// graph would hold it.
    fn history() -> Vec<Pose2d> {
        let mut poses = Vec::new();
        for i in 0..8 {
            poses.push(Pose2d::new(1.5 + i as f64, 8.5, 0.0));
        }
        for i in 0..8 {
            poses.push(Pose2d::new(8.5 - i as f64, 4.5, std::f64::consts::PI));
        }
        poses
    }

    fn revisiting() -> Explorer {
        Explorer::new(config(ScorePolicy::NearestFrontier)).with_revisits(RevisitConfig {
            open_loop_m: 10.0,
            min_loop_m: 6.0,
            max_detour_m: 8.0,
            tried_radius_m: 1.5,
        })
    }

    #[test]
    fn revisits_only_after_driving_open_loop() {
        let grid = two_doors();
        let robot = Pose2d::new(1.5, 4.5, std::f64::consts::PI);
        let mut explorer = revisiting();
        // Below the open-loop distance: an ordinary frontier goal.
        assert!(matches!(
            explorer
                .decide_with_history(&grid, robot, &history(), 5.0)
                .unwrap(),
            Decision::Goal(_)
        ));
        // Above it: back to a place seen long ago, the nearest by path.
        let Decision::Revisit(target) = explorer
            .decide_with_history(&grid, robot, &history(), 12.0)
            .unwrap()
        else {
            panic!("expected a revisit");
        };
        assert_eq!(target.gain_m2, 0.0);
        assert!(target.path_m <= 8.0);
        // The recent half of the trajectory (y = 4.5) cannot close a loop.
        assert_eq!(target.goal_cell.y, 8);
    }

    #[test]
    fn a_revisit_without_closure_is_not_repeated_until_a_loop_closes() {
        let grid = two_doors();
        let robot = Pose2d::new(1.5, 4.5, std::f64::consts::PI);
        let mut explorer = revisiting();
        let Decision::Revisit(first) = explorer
            .decide_with_history(&grid, robot, &history(), 12.0)
            .unwrap()
        else {
            panic!("expected a revisit");
        };
        // Reached, but no loop closed: the open-loop distance kept growing.
        explorer.clear_goal();
        match explorer
            .decide_with_history(&grid, robot, &history(), 13.0)
            .unwrap()
        {
            Decision::Revisit(next) => {
                let d = (next.goal.x_m - first.goal.x_m).hypot(next.goal.y_m - first.goal.y_m);
                assert!(d >= 1.5, "went back to the same place");
            }
            Decision::Goal(_) => {}
            Decision::Done => panic!("frontiers are left"),
        }
        // A loop closes (the distance falls): the place may be chosen again.
        explorer.clear_goal();
        explorer
            .decide_with_history(&grid, robot, &history(), 0.0)
            .unwrap();
        let Decision::Revisit(again) = explorer
            .decide_with_history(&grid, robot, &history(), 12.0)
            .unwrap()
        else {
            panic!("expected a revisit");
        };
        assert_eq!(again.goal_cell, first.goal_cell);
    }

    #[test]
    fn without_revisits_history_is_ignored() {
        let grid = two_doors();
        let robot = Pose2d::new(1.5, 4.5, std::f64::consts::PI);
        let mut plain = Explorer::new(config(ScorePolicy::NearestFrontier));
        let mut reference = Explorer::new(config(ScorePolicy::NearestFrontier));
        assert_eq!(
            plain
                .decide_with_history(&grid, robot, &history(), 100.0)
                .unwrap(),
            reference.decide(&grid, robot).unwrap()
        );
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
