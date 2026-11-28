//! Grasp planning: classical antipodal sampling, classical and learned
//! scoring, and kinematic reachability certification.

use std::sync::Arc;

use glam::DVec3;
use robot_foresight::{
    Certificate, Certifier, Critic, ForesightError, InferenceModel, ProposalContext, Proposer,
    Provenance,
};
use serde::{Deserialize, Serialize};

use crate::SerialChain;

/// A point cloud with outward surface normals.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PointCloud {
    /// Points (m).
    pub points: Vec<DVec3>,
    /// Unit outward normals, one per point.
    pub normals: Vec<DVec3>,
}

impl PointCloud {
    /// Samples the surface of an axis-aligned box on a grid of `spacing_m`.
    #[must_use]
    pub fn sample_box(center_m: DVec3, size_m: DVec3, spacing_m: f64) -> Self {
        let mut cloud = Self::default();
        let half = size_m * 0.5;
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                let nu = (size_m[u] / spacing_m).round().max(1.0) as usize;
                let nv = (size_m[v] / spacing_m).round().max(1.0) as usize;
                for i in 0..=nu {
                    for j in 0..=nv {
                        let mut p = DVec3::ZERO;
                        p[axis] = sign * half[axis];
                        p[u] = -half[u] + size_m[u] * i as f64 / nu as f64;
                        p[v] = -half[v] + size_m[v] * j as f64 / nv as f64;
                        let mut n = DVec3::ZERO;
                        n[axis] = sign;
                        cloud.points.push(center_m + p);
                        cloud.normals.push(n);
                    }
                }
            }
        }
        cloud
    }

    /// Centroid of the points.
    #[must_use]
    pub fn centroid(&self) -> DVec3 {
        if self.points.is_empty() {
            return DVec3::ZERO;
        }
        self.points.iter().copied().sum::<DVec3>() / self.points.len() as f64
    }

    fn min_z(&self) -> f64 {
        self.points
            .iter()
            .map(|p| p.z)
            .fold(f64::INFINITY, f64::min)
    }
}

/// A parallel-jaw grasp.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct GraspCandidate {
    /// Midpoint between the contacts (m).
    pub center_m: DVec3,
    /// Unit axis from the first contact to the second.
    pub closing_axis: DVec3,
    /// Unit direction the gripper moves along to reach the object.
    pub approach_axis: DVec3,
    /// Jaw opening at contact (m).
    pub width_m: f64,
    /// Angle (rad) between the closing line and the first contact's inward normal.
    pub friction_angle_a_rad: f64,
    /// Angle (rad) between the closing line and the second contact's inward normal.
    pub friction_angle_b_rad: f64,
}

/// Plan a grasp on a point cloud for an arm.
#[derive(Clone, Debug)]
pub struct GraspProblem {
    /// Object surface.
    pub cloud: Arc<PointCloud>,
    /// The arm.
    pub chain: Arc<SerialChain>,
    /// IK seed configuration (rad).
    pub seed_q: Vec<f64>,
    /// Largest jaw opening (m).
    pub max_width_m: f64,
    /// Coulomb friction coefficient at the contacts.
    pub friction_coefficient: f64,
    /// Distance (m) of the pre-grasp point back along the approach axis.
    pub pregrasp_offset_m: f64,
}

/// Classical proposer: antipodal contact pairs inside the friction cone.
///
/// For `attempts` seeded random first contacts, the partner is the point
/// whose inward normal best opposes the first contact's along the closing
/// line; pairs outside the friction cone or wider than the gripper are
/// dropped, as are near-duplicates (centres within `dedupe_m` with parallel
/// closing axes). The approach axis is the direction perpendicular to the
/// closing axis that is closest to straight down.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AntipodalGraspSampler {
    /// First contacts tried.
    pub attempts: usize,
    /// Largest number of candidates returned.
    pub max_candidates: usize,
    /// Duplicate radius (m).
    pub dedupe_m: f64,
}

impl Default for AntipodalGraspSampler {
    fn default() -> Self {
        Self {
            attempts: 200,
            max_candidates: 16,
            dedupe_m: 0.01,
        }
    }
}

fn approach_for(closing: DVec3) -> DVec3 {
    let down = -DVec3::Z;
    let a = down - closing * down.dot(closing);
    if a.length() > 1e-6 {
        a.normalize()
    } else {
        closing.any_orthonormal_vector()
    }
}

impl Proposer<GraspProblem, GraspCandidate> for AntipodalGraspSampler {
    fn name(&self) -> &str {
        "antipodal_sampler"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("antipodal_sampler")
    }
    fn propose(
        &self,
        p: &GraspProblem,
        ctx: &mut ProposalContext,
    ) -> Result<Vec<GraspCandidate>, ForesightError> {
        let cloud = &p.cloud;
        if cloud.points.len() != cloud.normals.len() || cloud.points.len() < 2 {
            return Err(ForesightError::InvalidInput(
                "point cloud needs >= 2 points with normals".into(),
            ));
        }
        let cone = p.friction_coefficient.max(0.0).atan();
        let mut out: Vec<GraspCandidate> = Vec::new();
        for _ in 0..self.attempts {
            if out.len() >= self.max_candidates {
                break;
            }
            let i = ctx.rng.index(cloud.points.len());
            let (pa, na) = (cloud.points[i], cloud.normals[i]);
            let best = (0..cloud.points.len())
                .filter(|&j| j != i)
                .filter_map(|j| {
                    let d = cloud.points[j] - pa;
                    let w = d.length();
                    if w < 1e-6 || w > p.max_width_m {
                        return None;
                    }
                    let c = d / w;
                    let ang_a = c.angle_between(-na);
                    let ang_b = (-c).angle_between(-cloud.normals[j]);
                    (ang_a <= cone && ang_b <= cone).then_some((j, ang_a + ang_b, ang_a, ang_b))
                })
                .min_by(|x, y| x.1.total_cmp(&y.1).then(x.0.cmp(&y.0)));
            let Some((j, _, ang_a, ang_b)) = best else {
                continue;
            };
            let pb = cloud.points[j];
            let closing = (pb - pa).normalize();
            let cand = GraspCandidate {
                center_m: (pa + pb) * 0.5,
                closing_axis: closing,
                approach_axis: approach_for(closing),
                width_m: pa.distance(pb),
                friction_angle_a_rad: ang_a,
                friction_angle_b_rad: ang_b,
            };
            let dup = out.iter().any(|o| {
                o.center_m.distance(cand.center_m) < self.dedupe_m
                    && o.closing_axis.dot(cand.closing_axis).abs() > 0.99
            });
            if !dup {
                out.push(cand);
            }
        }
        if out.is_empty() {
            return Err(ForesightError::NoSolution(
                "no antipodal pair inside the friction cone".into(),
            ));
        }
        Ok(out)
    }
}

/// Classical critic: mean friction-cone usage in `[0, 1]` (0 = both contact
/// normals aligned with the closing line).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AntipodalQualityCritic;

impl Critic<GraspProblem, GraspCandidate> for AntipodalQualityCritic {
    fn name(&self) -> &str {
        "friction_cone_usage"
    }
    fn provenance(&self) -> Provenance {
        Provenance::classical("antipodal_quality")
    }
    fn cost(&self, p: &GraspProblem, g: &GraspCandidate) -> Result<f64, ForesightError> {
        let cone = p.friction_coefficient.max(1e-6).atan();
        Ok((g.friction_angle_a_rad + g.friction_angle_b_rad) / (2.0 * cone))
    }
}

/// Number of [`grasp_features`].
pub const GRASP_FEATURES: usize = 6;

/// Features of a grasp for learned scorers: width (m), both friction angles
/// (rad), centre height above the object's lowest point (m), the approach
/// axis's z component, and the centre's distance to the cloud centroid (m).
#[must_use]
pub fn grasp_features(p: &GraspProblem, g: &GraspCandidate) -> [f32; GRASP_FEATURES] {
    [
        g.width_m as f32,
        g.friction_angle_a_rad as f32,
        g.friction_angle_b_rad as f32,
        (g.center_m.z - p.cloud.min_z()) as f32,
        g.approach_axis.z as f32,
        g.center_m.distance(p.cloud.centroid()) as f32,
    ]
}

/// Learned critic: a model maps [`grasp_features`] to a success logit; the
/// cost is the predicted failure probability `1 - sigmoid(logit)`.
#[derive(Clone, Debug)]
pub struct LearnedGraspScorer<M> {
    /// The model (6 inputs, 1 output).
    pub model: M,
}

impl<M: InferenceModel> LearnedGraspScorer<M> {
    /// Wraps a model with 6 inputs and 1 output.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] for any other shape.
    pub fn new(model: M) -> Result<Self, ForesightError> {
        let s = model.spec();
        if s.input_dim != GRASP_FEATURES || s.output_dim != 1 {
            return Err(ForesightError::InvalidConfig(format!(
                "grasp scorer `{}` must map 6 -> 1",
                s.model_id
            )));
        }
        Ok(Self { model })
    }
}

impl<M: InferenceModel> Critic<GraspProblem, GraspCandidate> for LearnedGraspScorer<M> {
    fn name(&self) -> &str {
        &self.model.spec().model_id
    }
    fn provenance(&self) -> Provenance {
        Provenance::learned(&self.model.spec().model_id)
    }
    fn cost(&self, p: &GraspProblem, g: &GraspCandidate) -> Result<f64, ForesightError> {
        let logit = f64::from(self.model.infer(&grasp_features(p, g))?[0]);
        Ok(1.0 - 1.0 / (1.0 + (-logit).exp()))
    }
}

/// Hard check: the grasp and pre-grasp points are reachable by position IK
/// within `tolerance_m`, and the width fits the gripper. Margin is the
/// tolerance minus the worse residual (m).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ReachabilityCertifier {
    /// Accepted IK residual (m).
    pub tolerance_m: f64,
    /// IK iterations.
    pub iterations: usize,
}

impl Default for ReachabilityCertifier {
    fn default() -> Self {
        Self {
            tolerance_m: 0.005,
            iterations: 150,
        }
    }
}

impl Certifier<GraspProblem, GraspCandidate> for ReachabilityCertifier {
    fn name(&self) -> &str {
        "reachability"
    }
    fn certify(&self, p: &GraspProblem, g: &GraspCandidate) -> Certificate {
        let reject = |r: String| Certificate::Rejected {
            reason: r,
            at_time_s: None,
        };
        if g.width_m > p.max_width_m {
            return reject("wider than the gripper".into());
        }
        let pre = g.center_m - g.approach_axis * p.pregrasp_offset_m;
        let mut worst: f64 = 0.0;
        let mut seed = p.seed_q.clone();
        for target in [pre, g.center_m] {
            match p.chain.solve_position_ik(target, &seed, self.iterations) {
                Ok((q, residual)) => {
                    worst = worst.max(residual);
                    seed = q;
                }
                Err(e) => return reject(e.to_string()),
            }
        }
        if worst > self.tolerance_m {
            return reject(format!("unreachable: IK residual {worst:.3} m"));
        }
        Certificate::Certified {
            margin: self.tolerance_m - worst,
        }
    }
}
