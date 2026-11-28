//! Learned and classical algorithms as interchangeable pipeline stages.
//!
//! A [`HybridPipeline`] runs four kinds of stage over a problem `P` and a
//! solution type `S`:
//!
//! | stage | role | typical classical | typical learned |
//! |---|---|---|---|
//! | [`Proposer`] | produce candidates | A*, RRT-Connect, antipodal sampling | policy rollout, diffusion sampler, motion prior |
//! | [`Refiner`] | improve one candidate | shortcutting, CEM, time scaling | learned warm-start optimiser |
//! | [`Critic`] | price one candidate | duration, length, risk | learned comfort or grasp-success score |
//! | [`Certifier`] | hard accept/reject | collision over the forecast, limits, reachability | — (certifiers should be verifiable) |
//!
//! No stage knows the provenance of another. A learned proposal and an A*
//! path are refined, priced and certified by the same code, and the
//! [`Decision`] records which one won and why every other one lost.

use std::fmt;

use serde::{Deserialize, Serialize};

use crate::{ForesightError, SplitMix64};

/// Where an algorithm or a candidate comes from.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Provenance {
    /// A hand-designed algorithm.
    Classical {
        /// Algorithm name, e.g. `"spacetime_astar"`.
        algorithm: String,
    },
    /// A learned model.
    Learned {
        /// Model identifier from its [`crate::ModelSpec`].
        model_id: String,
    },
    /// A composition of several sources.
    Hybrid {
        /// The parts, in order of application.
        parts: Vec<Provenance>,
    },
}

impl Provenance {
    /// Classical provenance.
    #[must_use]
    pub fn classical(algorithm: &str) -> Self {
        Self::Classical {
            algorithm: algorithm.to_owned(),
        }
    }

    /// Learned provenance.
    #[must_use]
    pub fn learned(model_id: &str) -> Self {
        Self::Learned {
            model_id: model_id.to_owned(),
        }
    }

    /// Whether any part is learned.
    #[must_use]
    pub fn involves_learning(&self) -> bool {
        match self {
            Self::Classical { .. } => false,
            Self::Learned { .. } => true,
            Self::Hybrid { parts } => parts.iter().any(Self::involves_learning),
        }
    }

    /// Short human-readable label, e.g. `learned:policy_v3+classical:shortcut`.
    #[must_use]
    pub fn label(&self) -> String {
        match self {
            Self::Classical { algorithm } => format!("classical:{algorithm}"),
            Self::Learned { model_id } => format!("learned:{model_id}"),
            Self::Hybrid { parts } => parts.iter().map(Self::label).collect::<Vec<_>>().join("+"),
        }
    }

    /// This provenance followed by `next` (flattened).
    #[must_use]
    pub fn then(self, next: Provenance) -> Self {
        let mut parts = match self {
            Self::Hybrid { parts } => parts,
            other => vec![other],
        };
        match next {
            Self::Hybrid { parts: more } => parts.extend(more),
            other => parts.push(other),
        }
        Self::Hybrid { parts }
    }
}

/// Per-proposer state handed to stochastic stages.
#[derive(Clone, Debug)]
pub struct ProposalContext {
    /// Seeded generator; derived from the pipeline seed and the stage index,
    /// so adding a stage does not change the randomness of the others.
    pub rng: SplitMix64,
}

/// Produces candidate solutions.
pub trait Proposer<P, S> {
    /// Stable stage name, recorded in the trace.
    fn name(&self) -> &str;
    /// Where the candidates come from.
    fn provenance(&self) -> Provenance;
    /// Proposes zero or more candidates.
    ///
    /// # Errors
    ///
    /// An error is recorded as a [`ProposerFailure`]; the pipeline continues
    /// with the other proposers.
    fn propose(&self, problem: &P, ctx: &mut ProposalContext) -> Result<Vec<S>, ForesightError>;
}

/// Improves one candidate.
pub trait Refiner<P, S> {
    /// Stable stage name.
    fn name(&self) -> &str;
    /// Where the refinement comes from.
    fn provenance(&self) -> Provenance;
    /// Returns an improved solution.
    ///
    /// # Errors
    ///
    /// On error the candidate keeps its previous solution and the failure is
    /// recorded in [`Evaluated::refine_failures`].
    fn refine(
        &self,
        problem: &P,
        solution: &S,
        ctx: &mut ProposalContext,
    ) -> Result<S, ForesightError>;
}

/// Prices one candidate (lower is better).
pub trait Critic<P, S> {
    /// Stable stage name.
    fn name(&self) -> &str;
    /// Where the price comes from.
    fn provenance(&self) -> Provenance;
    /// Raw cost; must be finite to be selectable.
    ///
    /// # Errors
    ///
    /// An error prices the candidate at infinity.
    fn cost(&self, problem: &P, solution: &S) -> Result<f64, ForesightError>;
}

/// Hard acceptance test.
pub trait Certifier<P, S> {
    /// Stable stage name.
    fn name(&self) -> &str;
    /// Accepts or rejects one candidate.
    fn certify(&self, problem: &P, solution: &S) -> Certificate;
}

/// Outcome of one [`Certifier`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Certificate {
    /// Accepted, with the smallest margin found (units defined by the certifier).
    Certified {
        /// Smallest margin over the solution.
        margin: f64,
    },
    /// Rejected.
    Rejected {
        /// Why.
        reason: String,
        /// Time (s) of the first violation, if the solution is timed.
        at_time_s: Option<f64>,
    },
}

impl Certificate {
    /// Whether the certificate accepts.
    #[must_use]
    pub fn is_certified(&self) -> bool {
        matches!(self, Self::Certified { .. })
    }
}

/// A candidate solution and where it came from.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate<S> {
    /// The solution after refinement.
    pub solution: S,
    /// Name of the proposer that produced it.
    pub proposer: String,
    /// Provenance including refiners.
    pub provenance: Provenance,
}

/// One weighted critic term.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CostTerm {
    /// Critic name.
    pub critic: String,
    /// Weight in the total.
    pub weight: f64,
    /// Raw cost (infinite when the critic failed).
    pub raw: f64,
}

/// A fully evaluated candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct Evaluated<S> {
    /// The candidate.
    pub candidate: Candidate<S>,
    /// Refiners that changed it, in order.
    pub refined_by: Vec<String>,
    /// Refiners that failed: `(name, error)`.
    pub refine_failures: Vec<(String, String)>,
    /// Critic terms.
    pub cost_terms: Vec<CostTerm>,
    /// Weighted sum of the cost terms.
    pub total_cost: f64,
    /// Every certifier's verdict: `(name, certificate)`.
    pub certificates: Vec<(String, Certificate)>,
}

impl<S> Evaluated<S> {
    /// Whether every certifier accepted and the cost is finite.
    #[must_use]
    pub fn is_selectable(&self) -> bool {
        self.total_cost.is_finite() && self.certificates.iter().all(|(_, c)| c.is_certified())
    }

    /// First rejection reason, if any.
    #[must_use]
    pub fn rejection(&self) -> Option<&str> {
        self.certificates.iter().find_map(|(_, c)| match c {
            Certificate::Rejected { reason, .. } => Some(reason.as_str()),
            Certificate::Certified { .. } => None,
        })
    }
}

/// A proposer that returned an error.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProposerFailure {
    /// Proposer name.
    pub proposer: String,
    /// Error text.
    pub error: String,
}

/// The result of [`HybridPipeline::decide`], with its full trace.
#[derive(Clone, Debug, PartialEq)]
pub struct Decision<S> {
    /// Index into `evaluated` of the selected candidate.
    pub selected: Option<usize>,
    /// Every candidate, in proposer order.
    pub evaluated: Vec<Evaluated<S>>,
    /// Proposers that failed.
    pub proposer_failures: Vec<ProposerFailure>,
}

impl<S> Decision<S> {
    /// The selected candidate.
    #[must_use]
    pub fn selected(&self) -> Option<&Evaluated<S>> {
        self.selected.map(|i| &self.evaluated[i])
    }

    /// Consumes the decision, returning the selected solution.
    #[must_use]
    pub fn into_solution(mut self) -> Option<S> {
        let i = self.selected?;
        Some(self.evaluated.swap_remove(i).candidate.solution)
    }

    /// One line per candidate: proposer, provenance, cost and verdict.
    #[must_use]
    pub fn summary(&self) -> String {
        let mut out = String::new();
        for (i, e) in self.evaluated.iter().enumerate() {
            let mark = if Some(i) == self.selected { "*" } else { " " };
            let verdict = e
                .rejection()
                .map_or_else(|| "certified".to_owned(), |r| format!("rejected: {r}"));
            out.push_str(&format!(
                "{mark} [{i:>2}] {:<24} {:<44} cost {:>9.3}  {verdict}\n",
                e.candidate.proposer,
                e.candidate.provenance.label(),
                e.total_cost
            ));
        }
        for f in &self.proposer_failures {
            out.push_str(&format!("  [ -] {:<24} failed: {}\n", f.proposer, f.error));
        }
        out
    }
}

/// How the pipeline picks among selectable candidates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SelectionPolicy {
    /// Lowest total cost; ties go to the earlier candidate.
    #[default]
    LowestCost,
    /// The first selectable candidate in proposer order: a priority list,
    /// e.g. "the learned policy if it certifies, else the classical planner".
    FirstCertified,
}

/// Proposers, refiners, critics and certifiers over one problem type.
pub struct HybridPipeline<P, S> {
    seed: u64,
    policy: SelectionPolicy,
    proposers: Vec<Box<dyn Proposer<P, S>>>,
    refiners: Vec<Box<dyn Refiner<P, S>>>,
    critics: Vec<(f64, Box<dyn Critic<P, S>>)>,
    certifiers: Vec<Box<dyn Certifier<P, S>>>,
}

impl<P, S> fmt::Debug for HybridPipeline<P, S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("HybridPipeline")
            .field("seed", &self.seed)
            .field("policy", &self.policy)
            .field(
                "proposers",
                &self.proposers.iter().map(|p| p.name()).collect::<Vec<_>>(),
            )
            .field(
                "refiners",
                &self.refiners.iter().map(|r| r.name()).collect::<Vec<_>>(),
            )
            .field(
                "critics",
                &self
                    .critics
                    .iter()
                    .map(|(w, c)| (c.name(), *w))
                    .collect::<Vec<_>>(),
            )
            .field(
                "certifiers",
                &self.certifiers.iter().map(|c| c.name()).collect::<Vec<_>>(),
            )
            .finish()
    }
}

/// Mixes the pipeline seed with a stage index (SplitMix64 finaliser).
fn stage_seed(seed: u64, stage: u64) -> u64 {
    SplitMix64::new(seed ^ stage.wrapping_mul(0xD1B5_4A32_D192_ED03)).next_u64()
}

impl<P, S: Clone> HybridPipeline<P, S> {
    /// An empty pipeline with an explicit seed.
    #[must_use]
    pub fn new(seed: u64) -> Self {
        Self {
            seed,
            policy: SelectionPolicy::default(),
            proposers: Vec::new(),
            refiners: Vec::new(),
            critics: Vec::new(),
            certifiers: Vec::new(),
        }
    }

    /// Sets the selection policy.
    #[must_use]
    pub fn with_policy(mut self, policy: SelectionPolicy) -> Self {
        self.policy = policy;
        self
    }

    /// Appends a proposer (proposer order is priority order for
    /// [`SelectionPolicy::FirstCertified`]).
    #[must_use]
    pub fn with_proposer(mut self, proposer: impl Proposer<P, S> + 'static) -> Self {
        self.proposers.push(Box::new(proposer));
        self
    }

    /// Appends a refiner, applied to every candidate in order.
    #[must_use]
    pub fn with_refiner(mut self, refiner: impl Refiner<P, S> + 'static) -> Self {
        self.refiners.push(Box::new(refiner));
        self
    }

    /// Appends a critic with a weight.
    #[must_use]
    pub fn with_critic(mut self, weight: f64, critic: impl Critic<P, S> + 'static) -> Self {
        self.critics.push((weight, Box::new(critic)));
        self
    }

    /// Appends a certifier.
    #[must_use]
    pub fn with_certifier(mut self, certifier: impl Certifier<P, S> + 'static) -> Self {
        self.certifiers.push(Box::new(certifier));
        self
    }

    /// Names of the proposers, in order.
    #[must_use]
    pub fn proposer_names(&self) -> Vec<&str> {
        self.proposers.iter().map(|p| p.name()).collect()
    }

    /// Proposes, refines, prices, certifies and selects.
    pub fn decide(&self, problem: &P) -> Decision<S> {
        let mut evaluated = Vec::new();
        let mut proposer_failures = Vec::new();
        for (pi, proposer) in self.proposers.iter().enumerate() {
            let mut ctx = ProposalContext {
                rng: SplitMix64::new(stage_seed(self.seed, pi as u64)),
            };
            match proposer.propose(problem, &mut ctx) {
                Ok(solutions) => {
                    for solution in solutions {
                        evaluated.push(self.evaluate(
                            problem,
                            proposer.as_ref(),
                            solution,
                            &mut ctx,
                        ));
                    }
                }
                Err(e) => proposer_failures.push(ProposerFailure {
                    proposer: proposer.name().to_owned(),
                    error: e.to_string(),
                }),
            }
        }
        let selectable = evaluated
            .iter()
            .enumerate()
            .filter(|(_, e)| e.is_selectable());
        let selected = match self.policy {
            SelectionPolicy::FirstCertified => selectable.map(|(i, _)| i).next(),
            SelectionPolicy::LowestCost => selectable
                .fold(None::<(usize, f64)>, |best, (i, e)| match best {
                    Some((_, c)) if c <= e.total_cost => best,
                    _ => Some((i, e.total_cost)),
                })
                .map(|(i, _)| i),
        };
        Decision {
            selected,
            evaluated,
            proposer_failures,
        }
    }

    fn evaluate(
        &self,
        problem: &P,
        proposer: &dyn Proposer<P, S>,
        mut solution: S,
        ctx: &mut ProposalContext,
    ) -> Evaluated<S> {
        let mut provenance = proposer.provenance();
        let mut refined_by = Vec::new();
        let mut refine_failures = Vec::new();
        for refiner in &self.refiners {
            match refiner.refine(problem, &solution, ctx) {
                Ok(next) => {
                    solution = next;
                    refined_by.push(refiner.name().to_owned());
                    provenance = provenance.then(refiner.provenance());
                }
                Err(e) => refine_failures.push((refiner.name().to_owned(), e.to_string())),
            }
        }
        let cost_terms: Vec<CostTerm> = self
            .critics
            .iter()
            .map(|(weight, critic)| CostTerm {
                critic: critic.name().to_owned(),
                weight: *weight,
                raw: critic
                    .cost(problem, &solution)
                    .ok()
                    .filter(|c| c.is_finite())
                    .unwrap_or(f64::INFINITY),
            })
            .collect();
        let total_cost = cost_terms
            .iter()
            .map(|t| {
                if t.weight == 0.0 {
                    0.0
                } else {
                    t.weight * t.raw
                }
            })
            .sum();
        let certificates = self
            .certifiers
            .iter()
            .map(|c| (c.name().to_owned(), c.certify(problem, &solution)))
            .collect();
        Evaluated {
            candidate: Candidate {
                solution,
                proposer: proposer.name().to_owned(),
                provenance,
            },
            refined_by,
            refine_failures,
            cost_terms,
            total_cost,
            certificates,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Problem: reach a number; solutions are numbers.
    struct Target(f64);

    struct Fixed(&'static str, Provenance, Vec<f64>);
    impl Proposer<Target, f64> for Fixed {
        fn name(&self) -> &str {
            self.0
        }
        fn provenance(&self) -> Provenance {
            self.1.clone()
        }
        fn propose(&self, _: &Target, _: &mut ProposalContext) -> Result<Vec<f64>, ForesightError> {
            Ok(self.2.clone())
        }
    }

    struct Broken;
    impl Proposer<Target, f64> for Broken {
        fn name(&self) -> &str {
            "broken_model"
        }
        fn provenance(&self) -> Provenance {
            Provenance::learned("broken")
        }
        fn propose(&self, _: &Target, _: &mut ProposalContext) -> Result<Vec<f64>, ForesightError> {
            Err(ForesightError::NonFiniteOutput("broken".into()))
        }
    }

    struct Noisy;
    impl Proposer<Target, f64> for Noisy {
        fn name(&self) -> &str {
            "noisy"
        }
        fn provenance(&self) -> Provenance {
            Provenance::learned("sampler")
        }
        fn propose(
            &self,
            t: &Target,
            ctx: &mut ProposalContext,
        ) -> Result<Vec<f64>, ForesightError> {
            Ok((0..4).map(|_| t.0 + ctx.rng.uniform(-1.0, 1.0)).collect())
        }
    }

    struct Halve;
    impl Refiner<Target, f64> for Halve {
        fn name(&self) -> &str {
            "halve_error"
        }
        fn provenance(&self) -> Provenance {
            Provenance::classical("bisection")
        }
        fn refine(
            &self,
            t: &Target,
            s: &f64,
            _: &mut ProposalContext,
        ) -> Result<f64, ForesightError> {
            Ok(t.0 + (s - t.0) * 0.5)
        }
    }

    struct Error;
    impl Critic<Target, f64> for Error {
        fn name(&self) -> &str {
            "abs_error"
        }
        fn provenance(&self) -> Provenance {
            Provenance::classical("abs")
        }
        fn cost(&self, t: &Target, s: &f64) -> Result<f64, ForesightError> {
            Ok((s - t.0).abs())
        }
    }

    struct NonNegative;
    impl Certifier<Target, f64> for NonNegative {
        fn name(&self) -> &str {
            "non_negative"
        }
        fn certify(&self, _: &Target, s: &f64) -> Certificate {
            if *s >= 0.0 {
                Certificate::Certified { margin: *s }
            } else {
                Certificate::Rejected {
                    reason: "negative".into(),
                    at_time_s: None,
                }
            }
        }
    }

    fn pipeline() -> HybridPipeline<Target, f64> {
        HybridPipeline::new(42)
            .with_proposer(Fixed(
                "learned_policy",
                Provenance::learned("policy"),
                vec![-1.0, 4.0],
            ))
            .with_proposer(Broken)
            .with_proposer(Fixed(
                "classical_search",
                Provenance::classical("search"),
                vec![2.0],
            ))
            .with_critic(1.0, Error)
            .with_certifier(NonNegative)
    }

    #[test]
    fn lowest_cost_among_certified_wins() {
        let d = pipeline().decide(&Target(3.0));
        assert_eq!(d.evaluated.len(), 3);
        // -1 is closest after none, but rejected; 4 and 2 tie at cost 1 -> earlier (4) wins.
        let s = d.selected().unwrap();
        assert_eq!(s.candidate.solution, 4.0);
        assert!(s.candidate.provenance.involves_learning());
        assert_eq!(d.evaluated[0].rejection(), Some("negative"));
        assert_eq!(d.proposer_failures.len(), 1);
        assert_eq!(d.proposer_failures[0].proposer, "broken_model");
    }

    #[test]
    fn first_certified_is_a_priority_fallback() {
        let p = HybridPipeline::new(1)
            .with_policy(SelectionPolicy::FirstCertified)
            .with_proposer(Fixed(
                "learned_policy",
                Provenance::learned("policy"),
                vec![-5.0],
            ))
            .with_proposer(Fixed(
                "classical_search",
                Provenance::classical("search"),
                vec![9.0],
            ))
            .with_critic(1.0, Error)
            .with_certifier(NonNegative);
        let d = p.decide(&Target(0.0));
        assert_eq!(d.selected().unwrap().candidate.proposer, "classical_search");
    }

    #[test]
    fn refiners_compose_provenance() {
        let p = pipeline().with_refiner(Halve);
        let d = p.decide(&Target(3.0));
        let s = d.selected().unwrap();
        assert_eq!(s.refined_by, vec!["halve_error".to_owned()]);
        assert_eq!(s.candidate.solution, 3.5);
        assert_eq!(
            s.candidate.provenance.label(),
            "learned:policy+classical:bisection"
        );
    }

    #[test]
    fn decisions_replay_from_the_seed() {
        let p = HybridPipeline::new(7)
            .with_proposer(Noisy)
            .with_critic(1.0, Error);
        let a = p.decide(&Target(0.0));
        let b = p.decide(&Target(0.0));
        assert_eq!(a, b);
        let other = HybridPipeline::new(8)
            .with_proposer(Noisy)
            .with_critic(1.0, Error);
        assert_ne!(
            a.evaluated[0].candidate.solution,
            other.decide(&Target(0.0)).evaluated[0].candidate.solution
        );
    }

    #[test]
    fn nothing_selectable_selects_nothing() {
        let d = HybridPipeline::new(0)
            .with_proposer(Fixed("p", Provenance::classical("p"), vec![-1.0]))
            .with_certifier(NonNegative)
            .decide(&Target(0.0));
        assert!(d.selected().is_none());
        assert!(d.into_solution().is_none());
    }
}
