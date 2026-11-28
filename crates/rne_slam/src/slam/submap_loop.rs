//! Loop closure against submaps; see [`Slam2d::close_loops_against_submaps`].

use super::{KeyframeLog, Slam2d};
use crate::likelihood::LikelihoodField;
use crate::pose_graph::PoseGraphEdge;
use crate::scan_match::{
    match_position_covariance, ScanMatchConfig, ScanMatcher, DEGENERACY_RATIO,
};
use rne_math::Vec3;
use rne_nav::{integrate_scan, GridError, OccupancyGrid, Pose2d};
use serde::{Deserialize, Serialize};

/// Settings for [`Slam2d::close_loops_against_submaps`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SubmapLoopConfig {
    /// An earlier keyframe is a candidate when its pose is within this
    /// distance of the current estimate.
    pub search_radius_m: f64,
    /// ... and at least this far back along the trajectory.
    pub min_path_m: f64,
    /// The submap holds the scans of the candidate keyframe and this many on
    /// each side (only those at least `min_path_m` back).
    pub submap_half_keyframes: usize,
    /// Side of the square submap grid, centred on the candidate.
    pub submap_size_m: f64,
    /// Matcher for the closure; its window bounds the drift a closure can
    /// correct.
    pub matcher: ScanMatchConfig,
    /// Minimum mean likelihood of the match against the submap.
    pub min_score: f64,
    /// Largest correction (matched pose against the current estimate)
    /// accepted with no path between candidate and current keyframe ...
    pub max_correction_m: f64,
    /// ... plus this much per metre of trajectory between them: the drift
    /// odometry and tracking can have built up.
    pub max_correction_per_m: f64,
    /// Rejects matches that are degenerate in one direction (a corridor, a
    /// single wall), where the scan fits the submap about as well anywhere
    /// along it: the match score's weaker curvature must be at least this
    /// fraction of the stronger one. Zero accepts every match; values above
    /// 0.2 act as 0.2 (the resolution of [`match_position_covariance`]).
    pub min_curvature_ratio: f64,
    /// Consecutive keyframes whose closures must agree before any is added.
    pub confirmations: usize,
    /// Two closures agree when their corrections differ by at most this ...
    pub confirm_translation_m: f64,
    /// ... and this much yaw.
    pub confirm_rotation_rad: f64,
    /// Keyframes allowed between agreeing closures.
    pub confirm_gap_keyframes: usize,
    /// Confirmed closures whose corrections are all smaller than this are
    /// added without re-optimizing the graph (or rebuilding the map): the
    /// graph already agrees with them, and the dense optimization grows with
    /// the cube of the keyframe count.
    pub optimize_min_correction_m: f64,
    /// After closures are added, no new candidate is searched for until the
    /// robot has driven this much further: one closure per stretch of
    /// trajectory is enough, and each costs a submap match and possibly an
    /// optimization.
    pub closure_spacing_m: f64,
}

impl Default for SubmapLoopConfig {
    fn default() -> Self {
        Self {
            search_radius_m: 2.5,
            min_path_m: 8.0,
            submap_half_keyframes: 10,
            submap_size_m: 12.0,
            matcher: ScanMatchConfig {
                linear_window_m: 1.0,
                angular_window_rad: 0.25,
                levels: 4,
                linear_samples: 11,
                angular_samples: 9,
                min_valid_beams: 30,
                coarse_downsample_factor: 4,
            },
            min_score: 0.6,
            max_correction_m: 0.3,
            max_correction_per_m: 0.05,
            min_curvature_ratio: 0.1,
            confirmations: 2,
            confirm_translation_m: 0.15,
            confirm_rotation_rad: 0.05,
            confirm_gap_keyframes: 3,
            optimize_min_correction_m: 0.05,
            closure_spacing_m: 2.0,
        }
    }
}

/// Gauss-Newton iterations per closure. A closure around a long loop moves
/// the graph far from the previous optimum: after the 8 iterations used
/// before, 100 more moved the nodes of a 370-keyframe loop another 0.5 m.
const OPTIMIZE_ITERATIONS: usize = 100;

/// State of submap loop closure inside [`Slam2d`].
#[derive(Clone, Debug)]
pub(super) struct SubmapLoops {
    config: SubmapLoopConfig,
    matcher: ScanMatcher,
    /// Closures found but not yet confirmed, oldest first.
    pending: Vec<Pending>,
    /// `[low score, degenerate, too large, unconfirmed]`.
    rejections: [usize; 4],
    /// Trajectory length before which no candidate is searched for.
    quiet_until_m: f64,
}

#[derive(Clone, Copy, Debug)]
struct Pending {
    candidate: usize,
    node: usize,
    measurement: Pose2d,
    /// Map-frame transform taking the current estimate to the matched pose.
    correction: Pose2d,
}

impl SubmapLoops {
    pub(super) fn new(config: SubmapLoopConfig) -> Self {
        Self {
            config,
            matcher: ScanMatcher::new(config.matcher),
            pending: Vec::new(),
            rejections: [0; 4],
            quiet_until_m: f64::NEG_INFINITY,
        }
    }

    pub(super) fn rejections(&self) -> [usize; 4] {
        self.rejections
    }

    /// Records a closure found at `node` and returns the closures to add when
    /// it completes a confirmed run.
    fn confirm(&mut self, found: Pending) -> Vec<Pending> {
        let config = self.config;
        let agrees = self.pending.last().is_some_and(|last| {
            let difference = last.correction.inverse().compose(found.correction);
            found.node - last.node <= config.confirm_gap_keyframes + 1
                && difference.x_m.hypot(difference.y_m) <= config.confirm_translation_m
                && wrap_angle(difference.yaw_rad).abs() <= config.confirm_rotation_rad
        });
        if !agrees {
            self.rejections[3] += self.pending.len();
            self.pending.clear();
        }
        self.pending.push(found);
        if self.pending.len() >= config.confirmations.max(1) {
            std::mem::take(&mut self.pending)
        } else {
            Vec::new()
        }
    }
}

impl Slam2d {
    /// Looks for a loop closure of the newest keyframe against a submap and
    /// adds the closures it confirms. Returns how many were added, and
    /// whether the graph was re-optimized.
    pub(super) fn try_submap_loop_closure(
        &mut self,
        points: &[Vec3],
        sensor_from_base: Pose2d,
        current_pose: Pose2d,
    ) -> Result<(usize, bool), GridError> {
        let (Some(loops), Some(log)) = (self.submap_loops.as_ref(), self.keyframes.as_ref()) else {
            return Ok((0, false));
        };
        let config = loops.config;
        let current = self.graph.node_count() - 1;
        let path_m = log.path_m[current];
        if path_m < loops.quiet_until_m {
            return Ok((0, false));
        }
        let Some(candidate) = self.submap_candidate(log, current, current_pose, &config) else {
            return Ok((0, false));
        };
        let submap = self.submap(log, candidate, current, &config)?;
        let field = LikelihoodField::from_occupancy(&submap, &self.config.likelihood)?;
        let Some(result) = loops
            .matcher
            .match_scan(&field, points, sensor_from_base, current_pose)
        else {
            return Ok((0, false));
        };
        let between_m = log.path_m[current] - log.path_m[candidate];
        let rejected = if result.score < config.min_score {
            Some(0)
        } else if degenerate(
            &field,
            points,
            sensor_from_base,
            result.pose,
            config.min_curvature_ratio,
        ) {
            Some(1)
        } else if (result.pose.x_m - current_pose.x_m).hypot(result.pose.y_m - current_pose.y_m)
            > config.max_correction_m + config.max_correction_per_m * between_m
        {
            Some(2)
        } else {
            None
        };
        let loops = self.submap_loops.as_mut().expect("checked above");
        if let Some(reason) = rejected {
            loops.rejections[reason] += 1;
            return Ok((0, false));
        }
        let candidate_pose = self.graph.node(candidate).expect("candidate node");
        let confirmed = loops.confirm(Pending {
            candidate,
            node: current,
            measurement: candidate_pose.inverse().compose(result.pose),
            correction: result.pose.compose(current_pose.inverse()),
        });
        let added = self.add_confirmed(&confirmed, config.optimize_min_correction_m)?;
        if added.0 > 0 {
            if let Some(loops) = self.submap_loops.as_mut() {
                loops.quiet_until_m = path_m + config.closure_spacing_m;
            }
        }
        Ok(added)
    }

    /// The nearest earlier keyframe within the search radius and at least
    /// `min_path_m` back along the trajectory (ties to the older one).
    fn submap_candidate(
        &self,
        log: &KeyframeLog,
        current: usize,
        current_pose: Pose2d,
        config: &SubmapLoopConfig,
    ) -> Option<usize> {
        let cutoff = log.path_m[current] - config.min_path_m;
        (0..current)
            .take_while(|&node| log.path_m[node] <= cutoff)
            .filter_map(|node| {
                let pose = self.graph.node(node)?;
                let distance = (pose.x_m - current_pose.x_m).hypot(pose.y_m - current_pose.y_m);
                (distance <= config.search_radius_m).then_some((node, distance))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1).then_with(|| a.0.cmp(&b.0)))
            .map(|(node, _)| node)
    }

    /// An occupancy grid around `candidate` with only the scans of the
    /// keyframes next to it, at their graph poses.
    fn submap(
        &self,
        log: &KeyframeLog,
        candidate: usize,
        current: usize,
        config: &SubmapLoopConfig,
    ) -> Result<OccupancyGrid, GridError> {
        let resolution = self.grid.resolution_m();
        let cells = (config.submap_size_m / resolution).ceil().max(1.0) as usize;
        let centre = self.graph.node(candidate).expect("candidate node");
        let half = 0.5 * cells as f64 * resolution;
        let mut grid = OccupancyGrid::new(
            cells,
            cells,
            resolution,
            Pose2d::new(centre.x_m - half, centre.y_m - half, 0.0),
        )?;
        let cutoff = log.path_m[current] - config.min_path_m;
        let first = candidate.saturating_sub(config.submap_half_keyframes);
        let last = (candidate + config.submap_half_keyframes).min(current);
        for node in first..=last {
            if log.path_m[node] > cutoff {
                break;
            }
            let (Some(pose), Some((scan, mounting))) = (self.graph.node(node), log.scans.get(node))
            else {
                continue;
            };
            integrate_scan(&mut grid, scan, pose.compose(*mounting), &self.config.scan)?;
        }
        Ok(grid)
    }

    /// Adds confirmed closures, optimizes when any correction reaches
    /// `optimize_min_m`, and rolls them all back when any is still
    /// inconsistent with the graph afterwards.
    fn add_confirmed(
        &mut self,
        confirmed: &[Pending],
        optimize_min_m: f64,
    ) -> Result<(usize, bool), GridError> {
        if confirmed.is_empty() {
            return Ok((0, false));
        }
        let optimize = confirmed
            .iter()
            .any(|c| c.correction.x_m.hypot(c.correction.y_m) >= optimize_min_m);
        let first_edge = self.graph.edge_count();
        for closure in confirmed {
            self.graph.add_edge(PoseGraphEdge::loop_closure(
                closure.candidate,
                closure.node,
                closure.measurement,
                (1.0, 1.0, 1.0),
            ));
        }
        if optimize {
            self.graph
                .optimize(OPTIMIZE_ITERATIONS, 1.0e-3, 0)
                .map_err(|_| GridError::NonFinite)?;
        }
        let edges = first_edge..self.graph.edge_count();
        let inconsistent = edges.clone().any(|edge| {
            self.graph.edge_residual(edge).is_some_and(|r| {
                r.0.hypot(r.1) > self.config.loop_max_translation_residual_m
                    || r.2.abs() > self.config.loop_max_rotation_residual_rad
            })
        });
        if inconsistent {
            for edge in edges.rev() {
                self.graph.remove_edge(edge);
            }
            if optimize {
                let _ = self.graph.optimize(OPTIMIZE_ITERATIONS, 1.0e-3, 0);
            }
            if let Some(loops) = self.submap_loops.as_mut() {
                loops.rejections[3] += confirmed.len();
            }
            return Ok((0, false));
        }
        self.loop_edges.extend(edges);
        Ok((confirmed.len(), optimize))
    }
}

/// Whether the match is degenerate: the score's weaker curvature is under
/// `min_ratio` of the stronger one. [`match_position_covariance`] with unit
/// base variance reports `DEGENERACY_RATIO * strong / weak` once that exceeds
/// one, so the test is on that variance.
fn degenerate(
    field: &LikelihoodField,
    points: &[Vec3],
    sensor_from_base: Pose2d,
    pose: Pose2d,
    min_ratio: f64,
) -> bool {
    if min_ratio <= 0.0 {
        return false;
    }
    let c = match_position_covariance(field, points, sensor_from_base, pose, 1.0, 1.0e6);
    let mean = 0.5 * (c[0][0] + c[1][1]);
    let spread = (0.25 * (c[0][0] - c[1][1]).powi(2) + c[0][1] * c[0][1]).sqrt();
    let threshold = (DEGENERACY_RATIO / min_ratio.min(DEGENERACY_RATIO)).max(1.0);
    mean + spread > threshold + 1.0e-9
}

fn wrap_angle(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}
