//! Space-time A*: search over `(voxel, time)` against the forecast.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashSet};

use glam::DVec3;
use robot_foresight::{ForesightError, ProposalContext, Proposer, Provenance, WorldForecast};
use serde::{Deserialize, Serialize};

use crate::{NavProblem, Trajectory3, Waypoint3};

/// Classical proposer: A* over 26-connected voxels plus a wait action, in
/// continuous time.
///
/// Each move takes the shortest time the speed limits allow; a state is
/// rejected when its cell is occupied in the map inflated by the robot
/// radius plus half a cell diagonal or when the forecast collision probability at the move's midpoint
/// or end exceeds the problem threshold. States are deduplicated per cell
/// and time bucket. The cost is arrival time plus `risk_weight` times the
/// summed collision probability; the heuristic is the straight-line travel
/// time times `heuristic_weight`. With `heuristic_weight = 1` and
/// `risk_weight = 0` the plan is time-optimal on the lattice; larger weights
/// bound the arrival time by that factor and expand far fewer of the
/// `(cell, time)` states. Ties go to the deeper state.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpaceTimeAstar {
    /// Duration (s) of the wait action.
    pub wait_s: f64,
    /// Width (s) of the time buckets used for duplicate detection.
    pub time_bucket_s: f64,
    /// Longest plan considered (s after the start).
    pub max_duration_s: f64,
    /// Weight of collision probability in the cost (s per unit probability).
    pub risk_weight: f64,
    /// Heuristic inflation (`>= 1`); the plan cost is at most this factor
    /// above the lattice optimum.
    pub heuristic_weight: f64,
    /// Expansion budget.
    pub max_expansions: usize,
}

impl Default for SpaceTimeAstar {
    fn default() -> Self {
        Self {
            wait_s: 0.25,
            time_bucket_s: 0.25,
            max_duration_s: 60.0,
            risk_weight: 2.0,
            heuristic_weight: 1.5,
            max_expansions: 400_000,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Node {
    cell: [usize; 3],
    pos: DVec3,
    t_s: f64,
    g: f64,
    parent: Option<usize>,
}

/// Total order on non-negative finite costs.
fn key(f: f64) -> u64 {
    f.max(0.0).to_bits()
}

impl SpaceTimeAstar {
    /// Runs the search.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::NoSolution`] when the start or goal is
    /// blocked or the budget runs out.
    pub fn plan(&self, problem: &NavProblem) -> Result<Trajectory3, ForesightError> {
        problem.validate()?;
        // Inflate by the robot radius plus half a cell diagonal: a free cell
        // centre is then at least the robot radius from every occupied
        // cell's box, not only from its centre.
        let res = problem.map.resolution_m();
        let map = problem
            .map
            .inflated(problem.robot_radius_m + res * 3f64.sqrt() * 0.5);
        let start_cell = map
            .cell_of(problem.start_m)
            .filter(|c| !map.is_occupied_cell(*c))
            .ok_or_else(|| ForesightError::NoSolution("start is blocked or off the map".into()))?;
        let goal_cell = map
            .cell_of(problem.goal_m)
            .filter(|c| !map.is_occupied_cell(*c))
            .ok_or_else(|| ForesightError::NoSolution("goal is blocked or off the map".into()))?;
        let risk_at = |p: DVec3, t: f64| {
            problem
                .forecast
                .collision_probability(p, problem.robot_radius_m, t)
        };
        let w = self.heuristic_weight.max(1.0);
        let h = |p: DVec3| w * problem.min_travel_time_s(problem.goal_m - p);
        let t_max = problem.start_t_s + self.max_duration_s;
        let bucket = |t: f64| ((t - problem.start_t_s) / self.time_bucket_s).round() as i64;

        let mut nodes = vec![Node {
            cell: start_cell,
            pos: problem.start_m,
            t_s: problem.start_t_s,
            g: 0.0,
            parent: None,
        }];
        let mut open = BinaryHeap::new();
        open.push(Reverse((key(h(problem.start_m)), Reverse(0u64), 0usize)));
        let mut closed: HashSet<([usize; 3], i64)> = HashSet::new();
        let mut expansions = 0usize;

        while let Some(Reverse((_, _, idx))) = open.pop() {
            let node = nodes[idx];
            if !closed.insert((node.cell, bucket(node.t_s))) {
                continue;
            }
            if node.cell == goal_cell || problem.at_goal(node.pos) {
                return Ok(self.finish(problem, &nodes, idx));
            }
            expansions += 1;
            if expansions > self.max_expansions {
                return Err(ForesightError::NoSolution(
                    "expansion budget exhausted".into(),
                ));
            }
            for (cell, pos, dt) in self.successors(problem, &map, &node) {
                let t = node.t_s + dt;
                if t > t_max || closed.contains(&(cell, bucket(t))) {
                    continue;
                }
                let mid = risk_at(node.pos.lerp(pos, 0.5), node.t_s + dt * 0.5);
                let end = risk_at(pos, t);
                if mid > problem.risk_threshold || end > problem.risk_threshold {
                    continue;
                }
                let g = node.g + dt + self.risk_weight * (mid + end) * 0.5 * dt;
                nodes.push(Node {
                    cell,
                    pos,
                    t_s: t,
                    g,
                    parent: Some(idx),
                });
                // Ties go to the larger g (deeper), then to insertion order.
                open.push(Reverse((key(g + h(pos)), Reverse(key(g)), nodes.len() - 1)));
            }
        }
        Err(ForesightError::NoSolution("search space exhausted".into()))
    }

    fn successors(
        &self,
        problem: &NavProblem,
        map: &crate::VoxelGrid,
        node: &Node,
    ) -> Vec<([usize; 3], DVec3, f64)> {
        let mut out = Vec::with_capacity(27);
        out.push((node.cell, node.pos, self.wait_s));
        let dims = map.dims();
        for dz in -1i64..=1 {
            for dy in -1i64..=1 {
                for dx in -1i64..=1 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let c = [
                        node.cell[0] as i64 + dx,
                        node.cell[1] as i64 + dy,
                        node.cell[2] as i64 + dz,
                    ];
                    if c.iter().zip(dims).any(|(v, d)| *v < 0 || *v >= d as i64) {
                        continue;
                    }
                    let cell = [c[0] as usize, c[1] as usize, c[2] as usize];
                    if map.is_occupied_cell(cell) {
                        continue;
                    }
                    let pos = map.cell_center(cell);
                    let dt = problem.min_travel_time_s(pos - node.pos);
                    if dt.is_finite() {
                        out.push((cell, pos, dt));
                    }
                }
            }
        }
        out
    }

    fn finish(&self, problem: &NavProblem, nodes: &[Node], goal_idx: usize) -> Trajectory3 {
        let mut rev = Vec::new();
        let mut cur = Some(goal_idx);
        while let Some(i) = cur {
            rev.push(Waypoint3 {
                t_s: nodes[i].t_s,
                position_m: nodes[i].pos,
            });
            cur = nodes[i].parent;
        }
        rev.reverse();
        let last = rev[rev.len() - 1];
        let dt = problem.min_travel_time_s(problem.goal_m - last.position_m);
        if last.position_m.distance(problem.goal_m) > 1e-9 && dt.is_finite() {
            rev.push(Waypoint3 {
                t_s: last.t_s + dt,
                position_m: problem.goal_m,
            });
        }
        Trajectory3::new(rev)
    }
}

impl Proposer<NavProblem, Trajectory3> for SpaceTimeAstar {
    fn name(&self) -> &str {
        "spacetime_astar"
    }

    fn provenance(&self) -> Provenance {
        Provenance::classical("spacetime_astar")
    }

    fn propose(
        &self,
        problem: &NavProblem,
        _ctx: &mut ProposalContext,
    ) -> Result<Vec<Trajectory3>, ForesightError> {
        Ok(vec![self.plan(problem)?])
    }
}
