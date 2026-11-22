//! Ray-traced GNSS signal propagation: line-of-sight occlusion, multi-bounce
//! specular reflections, and single knife-edge diffraction.
//!
//! At 20 000 km a satellite's signal arrives as a plane wave, so each
//! satellite is a direction `u` (antenna toward satellite, ENU).
//!
//! * **Direct** — a ray from the antenna along `u`; line-of-sight (LOS) when
//!   no box blocks it.
//! * **Reflections** — for a sequence of facets `g1 … gk` (`g1` nearest the
//!   antenna, `k ≤ max_reflections`) the image method gives the look
//!   direction at the antenna, `a = M(g1) … M(gk) u`, where `M(g)` mirrors a
//!   direction in facet `g`. The ray is traced forward: it must hit each
//!   facet in turn within its bounds and from its front, with nothing in
//!   between, and leave the last facet toward the satellite unobstructed. For
//!   a plane wave the path is longer than the direct one by
//!   `Σ|Pᵢ − Pᵢ₋₁| − (Pk − A)·u`.
//! * **Diffraction** — for every box edge, the point `E` on the edge that
//!   minimises the path `|E − A| − (E − A)·u` (Keller's law for a plane wave)
//!   gives a diffracted path when `A → E` and `E → satellite` are clear. Its
//!   loss is the ITU-R P.526 knife-edge loss `J(ν)` with
//!   `ν = 2 √(Δ / λ)` for the excess path `Δ`, so it depends on frequency.
//!   Diffraction is traced only when the direct path is blocked.
//! * **Foliage** — every path is attenuated by the length it runs through
//!   the scene's foliage volumes (tree canopies).
//!
//! Facets and boxes are visited in index order, so results are deterministic.

use crate::scene::{Facet, FacetSource, GnssScene};
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Small offset that lifts secondary rays off the surface they start on.
const SURFACE_EPSILON_M: f64 = 1.0e-6;

/// Ray-tracing options.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TraceConfig {
    /// Maximum number of specular bounces per path (0 disables reflections).
    pub max_reflections: usize,
    /// Whether to trace knife-edge diffraction around box edges.
    pub diffraction: bool,
    /// Diffracted paths longer than the direct one by more than this are
    /// dropped (their knife-edge loss is far beyond tracking), in meters.
    pub max_diffraction_excess_m: f64,
    /// Surfaces and edges farther than this from the antenna are not traced,
    /// in meters (distant reflections are long-delayed and weak; this keeps
    /// large scenes fast).
    pub max_interaction_distance_m: f64,
    /// Mesh triangles smaller than this are not traced as specular
    /// reflectors, m² (a specular reflection needs a surface comparable to
    /// the first Fresnel zone, a few m² at L1 at street distances; smaller
    /// details scatter rather than mirror).
    #[serde(default = "default_min_reflector_area")]
    pub min_reflector_area_m2: f64,
    /// Only this many facets nearest the antenna are combined into
    /// multi-bounce paths (single bounces use every facet in range), which
    /// bounds the `O(n²)` search on detailed meshes. Unlimited by default;
    /// a few dozen keeps a city model fast.
    #[serde(default = "default_max_multi_bounce_facets")]
    pub max_multi_bounce_facets: usize,
}

fn default_min_reflector_area() -> f64 {
    4.0
}

fn default_max_multi_bounce_facets() -> usize {
    usize::MAX
}

impl Default for TraceConfig {
    fn default() -> Self {
        Self {
            max_reflections: 2,
            diffraction: true,
            max_diffraction_excess_m: 2.0,
            max_interaction_distance_m: 150.0,
            min_reflector_area_m2: default_min_reflector_area(),
            max_multi_bounce_facets: default_max_multi_bounce_facets(),
        }
    }
}

/// How a path reaches the antenna.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PathKind {
    /// The unobstructed direct path.
    Direct,
    /// Specular reflections off this many surfaces.
    Reflected {
        /// Number of bounces.
        bounces: u8,
    },
    /// Single diffraction over a box edge.
    Diffracted,
}

/// One propagation path from a satellite to the antenna.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PropagationPath {
    /// Path type.
    pub kind: PathKind,
    /// Extra length relative to the direct path in meters.
    pub excess_delay_m: f64,
    /// Direction from the antenna toward the first interaction (or the
    /// satellite for the direct path), unit ENU.
    pub arrival_direction: Vec3,
    /// Sum of material reflection losses in dB (frequency independent).
    pub reflection_loss_db: f64,
    /// Attenuation through foliage along the whole path, in dB.
    pub foliage_loss_db: f64,
    /// Interaction points from the antenna outward, in ENU meters.
    pub points_m: Vec<Vec3>,
    /// The surfaces (or, for diffraction, the box) involved, in order.
    pub sources: Vec<FacetSource>,
}

impl PropagationPath {
    /// Total path loss in dB at a carrier wavelength (reflection loss plus,
    /// for a diffracted path, the knife-edge loss).
    pub fn loss_db(&self, wavelength_m: f64) -> f64 {
        let interaction = match self.kind {
            PathKind::Diffracted => {
                knife_edge_loss_db(2.0 * (self.excess_delay_m.max(0.0) / wavelength_m).sqrt())
            }
            _ => self.reflection_loss_db,
        };
        interaction + self.foliage_loss_db
    }
}

/// ITU-R P.526 single knife-edge diffraction loss `J(ν)` in dB.
pub fn knife_edge_loss_db(nu: f64) -> f64 {
    if nu <= -0.78 {
        0.0
    } else {
        6.9 + 20.0 * (((nu - 0.1).powi(2) + 1.0).sqrt() + nu - 0.1).log10()
    }
}

/// How a satellite's signal reaches the antenna.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Propagation {
    /// Whether the direct path is unobstructed.
    pub line_of_sight: bool,
    /// The box or mesh triangle blocking the direct path, if any.
    pub blocked_by: Option<FacetSource>,
    /// All paths: the direct path first when LOS, then reflections by bounce
    /// count, then diffraction.
    pub paths: Vec<PropagationPath>,
}

impl Propagation {
    /// Indirect (reflected or diffracted) paths.
    pub fn indirect_paths(&self) -> impl Iterator<Item = &PropagationPath> {
        self.paths.iter().filter(|p| p.kind != PathKind::Direct)
    }
}

/// Traces every path from an antenna toward a satellite.
///
/// `satellite_direction` is the ENU direction from the antenna to the
/// satellite. Satellites at or below the geometric horizon, and antennas
/// inside a solid box, get no paths.
pub fn trace_satellite(
    scene: &GnssScene,
    antenna_m: Vec3,
    satellite_direction: Vec3,
    config: &TraceConfig,
) -> Propagation {
    let u = satellite_direction.normalize();
    let blocked_by = scene
        .raycast(antenna_m, u, f64::INFINITY)
        .map(|hit| hit.source)
        .filter(|source| *source != FacetSource::Ground);
    let mut propagation = Propagation {
        line_of_sight: blocked_by.is_none(),
        blocked_by,
        paths: Vec::new(),
    };
    if u.z <= 0.0 || scene.inside_solid(antenna_m) {
        propagation.line_of_sight = false;
        return propagation;
    }
    if propagation.line_of_sight {
        propagation.paths.push(PropagationPath {
            kind: PathKind::Direct,
            excess_delay_m: 0.0,
            arrival_direction: u,
            reflection_loss_db: 0.0,
            foliage_loss_db: scene.foliage_loss_db(antenna_m, u, f64::INFINITY),
            points_m: Vec::new(),
            sources: Vec::new(),
        });
    }

    let reach = config.max_interaction_distance_m;
    let facets: Vec<(f64, Facet)> = scene
        .facets(config.min_reflector_area_m2)
        .into_iter()
        .map(|f| (box_distance(f.min_m, f.max_m, antenna_m), f))
        .filter(|(d, _)| *d <= reach)
        .collect();
    let all: Vec<Facet> = facets.iter().map(|(_, f)| *f).collect();
    // Facets the satellite illuminates (candidates for the last bounce).
    let lit = |set: &[Facet]| -> Vec<usize> {
        (0..set.len())
            .filter(|&i| u.dot(set[i].normal) > 0.0)
            .collect()
    };
    let mut sequence = Vec::with_capacity(config.max_reflections);
    if config.max_reflections >= 1 {
        enumerate_sequences(
            scene,
            &all,
            &lit(&all),
            antenna_m,
            u,
            1,
            &mut sequence,
            &mut propagation.paths,
        );
    }
    if config.max_reflections >= 2 {
        // The nearest facets (ties by scene order), kept in scene order.
        let mut ranked: Vec<usize> = (0..facets.len()).collect();
        ranked.sort_by(|&a, &b| facets[a].0.total_cmp(&facets[b].0).then(a.cmp(&b)));
        ranked.truncate(config.max_multi_bounce_facets);
        ranked.sort_unstable();
        let near: Vec<Facet> = ranked.iter().map(|&i| facets[i].1).collect();
        let near_lit = lit(&near);
        for depth in 2..=config.max_reflections {
            enumerate_sequences(
                scene,
                &near,
                &near_lit,
                antenna_m,
                u,
                depth,
                &mut sequence,
                &mut propagation.paths,
            );
        }
    }
    if config.diffraction && !propagation.line_of_sight {
        trace_diffraction(scene, antenna_m, u, config, &mut propagation.paths);
    }
    propagation
}

#[allow(clippy::too_many_arguments)]
fn enumerate_sequences(
    scene: &GnssScene,
    facets: &[Facet],
    lit: &[usize],
    antenna_m: Vec3,
    u: Vec3,
    depth: usize,
    sequence: &mut Vec<usize>,
    out: &mut Vec<PropagationPath>,
) {
    if sequence.len() == depth {
        if let Some(path) = trace_sequence(scene, facets, sequence, antenna_m, u) {
            out.push(path);
        }
        return;
    }
    let last = sequence.len() + 1 == depth;
    let candidates: Box<dyn Iterator<Item = usize>> = if last {
        Box::new(lit.iter().copied())
    } else {
        Box::new(0..facets.len())
    };
    for index in candidates {
        if sequence.last() == Some(&index) {
            continue;
        }
        // The antenna must be in front of the first facet.
        if sequence.is_empty()
            && (antenna_m - facets[index].point_m).dot(facets[index].normal) <= 0.0
        {
            continue;
        }
        sequence.push(index);
        enumerate_sequences(scene, facets, lit, antenna_m, u, depth, sequence, out);
        sequence.pop();
    }
}

/// Distance from a point to an axis-aligned box (zero inside).
fn box_distance(min_m: Vec3, max_m: Vec3, point_m: Vec3) -> f64 {
    let clamped = point_m.clamp(min_m, max_m);
    let d = point_m - clamped;
    let d = Vec3::new(
        if d.x.is_finite() { d.x } else { 0.0 },
        if d.y.is_finite() { d.y } else { 0.0 },
        if d.z.is_finite() { d.z } else { 0.0 },
    );
    d.length()
}

fn mirror(direction: Vec3, normal: Vec3) -> Vec3 {
    direction - normal * (2.0 * direction.dot(normal))
}

/// Forward-traces a facet sequence; `None` when the geometry does not close.
fn trace_sequence(
    scene: &GnssScene,
    facets: &[Facet],
    sequence: &[usize],
    antenna_m: Vec3,
    u: Vec3,
) -> Option<PropagationPath> {
    // Look direction at the antenna: mirror u back through the sequence.
    let mut look = u;
    for &index in sequence.iter().rev() {
        look = mirror(look, facets[index].normal);
    }
    // Cheap geometric checks first: every hit must approach a front face.
    let mut direction = look;
    for &index in sequence {
        if direction.dot(facets[index].normal) >= 0.0 {
            return None;
        }
        direction = mirror(direction, facets[index].normal);
    }

    let mut origin = antenna_m;
    let mut direction = look;
    let mut points = Vec::with_capacity(sequence.len());
    let mut length = 0.0;
    let mut loss = 0.0;
    let mut foliage = 0.0;
    for &index in sequence {
        let facet = &facets[index];
        let n = facet.normal;
        let t = (facet.point_m - origin).dot(n) / direction.dot(n);
        if !t.is_finite() || t <= 1.0e-9 {
            return None;
        }
        let hit = origin + direction * t;
        if !facet.contains_in_plane(hit) {
            return None;
        }
        if scene.occluded(origin, direction, t - 1.0e-6) {
            return None;
        }
        length += t;
        loss += facet.material.reflection_loss_db;
        foliage += scene.foliage_loss_db(origin, direction, t);
        points.push(hit);
        origin = hit + n * SURFACE_EPSILON_M;
        direction = mirror(direction, n);
    }
    if scene.occluded(origin, u, f64::INFINITY) {
        return None;
    }
    foliage += scene.foliage_loss_db(origin, u, f64::INFINITY);
    let last = *points.last()?;
    Some(PropagationPath {
        kind: PathKind::Reflected {
            bounces: sequence.len() as u8,
        },
        excess_delay_m: length - (last - antenna_m).dot(u),
        arrival_direction: look,
        reflection_loss_db: loss,
        foliage_loss_db: foliage,
        points_m: points,
        sources: sequence.iter().map(|&i| facets[i].source).collect(),
    })
}

/// Adds the best diffracted path over each box's edges.
fn trace_diffraction(
    scene: &GnssScene,
    antenna_m: Vec3,
    u: Vec3,
    config: &TraceConfig,
    out: &mut Vec<PropagationPath>,
) {
    let excess = |point: Vec3| {
        let d = point - antenna_m;
        d.length() - d.dot(u)
    };
    for (box_index, scene_box) in scene.boxes.iter().enumerate() {
        if !scene_box.material.is_reflective()
            || box_distance(scene_box.min_m, scene_box.max_m, antenna_m)
                > config.max_interaction_distance_m
        {
            continue;
        }
        let mut best: Option<(f64, Vec3, Vec3, f64)> = None;
        for (p0, p1) in scene_box.edges() {
            // Convex 1-D minimisation by golden-section search.
            let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
            let ratio = 0.618_033_988_749_895;
            for _ in 0..48 {
                let a = hi - ratio * (hi - lo);
                let b = lo + ratio * (hi - lo);
                if excess(p0.lerp(p1, a)) < excess(p0.lerp(p1, b)) {
                    hi = b;
                } else {
                    lo = a;
                }
            }
            let point = p0.lerp(p1, 0.5 * (lo + hi));
            let delta = excess(point);
            if delta > config.max_diffraction_excess_m
                || best.is_some_and(|(d, _, _, _)| delta >= d)
            {
                continue;
            }
            // Outward offset away from the box at the edge.
            let mut outward = Vec3::ZERO;
            for axis in 0..3 {
                if (point[axis] - scene_box.min_m[axis]).abs() < 1.0e-9 {
                    outward[axis] = -1.0;
                } else if (point[axis] - scene_box.max_m[axis]).abs() < 1.0e-9 {
                    outward[axis] = 1.0;
                }
            }
            let lifted = point + outward.normalize_or_zero() * 1.0e-4;
            let to_edge = lifted - antenna_m;
            let distance = to_edge.length();
            if distance < 1.0e-6 {
                continue;
            }
            let toward = to_edge / distance;
            if scene.occluded(antenna_m, toward, distance - 1.0e-6)
                || scene.occluded(lifted, u, f64::INFINITY)
            {
                continue;
            }
            let foliage = scene.foliage_loss_db(antenna_m, toward, distance)
                + scene.foliage_loss_db(lifted, u, f64::INFINITY);
            best = Some((delta, point, toward, foliage));
        }
        if let Some((delta, point, toward, foliage)) = best {
            out.push(PropagationPath {
                kind: PathKind::Diffracted,
                excess_delay_m: delta,
                arrival_direction: toward,
                reflection_loss_db: 0.0,
                foliage_loss_db: foliage,
                points_m: vec![point],
                sources: vec![FacetSource::Box(box_index)],
            });
        }
    }
    if let Some(mesh) = &scene.mesh {
        trace_mesh_diffraction(scene, mesh, antenna_m, u, config, out);
    }
}

/// The point on segment `p0..p1` minimising `excess` (convex along a line),
/// by golden-section search.
fn minimise_on_segment(p0: Vec3, p1: Vec3, excess: impl Fn(Vec3) -> f64) -> Vec3 {
    let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
    let ratio = 0.618_033_988_749_895;
    for _ in 0..48 {
        let a = hi - ratio * (hi - lo);
        let b = lo + ratio * (hi - lo);
        if excess(p0.lerp(p1, a)) < excess(p0.lerp(p1, b)) {
            hi = b;
        } else {
            lo = a;
        }
    }
    p0.lerp(p1, 0.5 * (lo + hi))
}

/// Adds the best diffracted path over each mesh object's edges.
fn trace_mesh_diffraction(
    scene: &GnssScene,
    mesh: &crate::mesh::TriangleMesh,
    antenna_m: Vec3,
    u: Vec3,
    config: &TraceConfig,
    out: &mut Vec<PropagationPath>,
) {
    let excess = |point: Vec3| {
        let d = point - antenna_m;
        d.length() - d.dot(u)
    };
    // Best candidate per object: (excess, point, lifted point, edge index).
    let mut best: std::collections::BTreeMap<u32, (f64, Vec3, Vec3, usize)> =
        std::collections::BTreeMap::new();
    for (index, edge) in mesh.diffraction_edges().iter().enumerate() {
        if box_distance(
            edge.start_m.min(edge.end_m),
            edge.start_m.max(edge.end_m),
            antenna_m,
        ) > config.max_interaction_distance_m
        {
            continue;
        }
        let point = minimise_on_segment(edge.start_m, edge.end_m, excess);
        let delta = excess(point);
        if delta > config.max_diffraction_excess_m
            || best.get(&edge.object).is_some_and(|b| delta >= b.0)
        {
            continue;
        }
        let lifted = point + edge.outward * 1.0e-4;
        let to_edge = lifted - antenna_m;
        let distance = to_edge.length();
        if distance < 1.0e-6 {
            continue;
        }
        let toward = to_edge / distance;
        if scene.occluded(antenna_m, toward, distance - 1.0e-6)
            || scene.occluded(lifted, u, f64::INFINITY)
        {
            continue;
        }
        best.insert(edge.object, (delta, point, lifted, index));
    }
    for (delta, point, lifted, index) in best.into_values() {
        let to_edge = lifted - antenna_m;
        let distance = to_edge.length();
        let toward = to_edge / distance;
        let foliage = scene.foliage_loss_db(antenna_m, toward, distance)
            + scene.foliage_loss_db(lifted, u, f64::INFINITY);
        let edge = &mesh.diffraction_edges()[index];
        // Attribute the path to the nearest triangle of the edge's object
        // for path identity (the first triangle hit just below the edge).
        let source = mesh
            .raycast(lifted, -edge.outward, 1.0)
            .map_or(FacetSource::Triangle(usize::MAX), |(_, t)| {
                FacetSource::Triangle(t)
            });
        out.push(PropagationPath {
            kind: PathKind::Diffracted,
            excess_delay_m: delta,
            arrival_direction: toward,
            reflection_loss_db: 0.0,
            foliage_loss_db: foliage,
            points_m: vec![point],
            sources: vec![source],
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::{SceneBox, SurfaceMaterial};
    use approx::assert_relative_eq;

    fn direction(azimuth_deg: f64, elevation_deg: f64) -> Vec3 {
        let (az, el) = (azimuth_deg.to_radians(), elevation_deg.to_radians());
        Vec3::new(az.sin() * el.cos(), az.cos() * el.cos(), el.sin())
    }

    fn single_bounce() -> TraceConfig {
        TraceConfig {
            max_reflections: 1,
            diffraction: false,
            ..TraceConfig::default()
        }
    }

    #[test]
    fn open_sky_is_los_with_ground_multipath_of_two_h_sin_el() {
        let scene = GnssScene::open_sky();
        let height = 1.5;
        let elevation = 30.0_f64;
        let p = trace_satellite(
            &scene,
            Vec3::new(0.0, 0.0, height),
            direction(45.0, elevation),
            &TraceConfig::default(),
        );
        assert!(p.line_of_sight);
        assert_eq!(p.paths.len(), 2, "direct + ground");
        let ground = &p.paths[1];
        assert_eq!(ground.sources, vec![FacetSource::Ground]);
        assert_relative_eq!(
            ground.excess_delay_m,
            2.0 * height * elevation.to_radians().sin(),
            epsilon = 1e-9
        );
    }

    #[test]
    fn wall_blocks_a_low_satellite_behind_it() {
        let mut scene = GnssScene::default();
        scene.add_box(SceneBox::building(
            10.0,
            -50.0,
            12.0,
            50.0,
            30.0,
            SurfaceMaterial::CONCRETE,
        ));
        let antenna = Vec3::new(0.0, 0.0, 1.0);
        let low_east = trace_satellite(&scene, antenna, direction(90.0, 40.0), &single_bounce());
        assert!(!low_east.line_of_sight);
        assert_eq!(low_east.blocked_by, Some(FacetSource::Box(0)));
        let high_east = trace_satellite(&scene, antenna, direction(90.0, 80.0), &single_bounce());
        assert!(high_east.line_of_sight);
    }

    fn street() -> GnssScene {
        // A 30 m concrete wall 10 m east and a 20 m glass wall 10 m west.
        let mut scene = GnssScene::default();
        scene.add_box(SceneBox::building(
            10.0,
            -50.0,
            12.0,
            50.0,
            30.0,
            SurfaceMaterial::CONCRETE,
        ));
        scene.add_box(SceneBox::building(
            -12.0,
            -50.0,
            -10.0,
            50.0,
            20.0,
            SurfaceMaterial::GLASS,
        ));
        scene
    }

    #[test]
    fn opposite_wall_creates_an_nlos_path_with_image_method_delay() {
        let antenna = Vec3::new(0.0, 0.0, 1.0);
        let elevation = 40.0_f64.to_radians();
        // The western satellite is masked by the glass wall, but its signal
        // bounces off the east wall and clears the glass wall on the way in.
        let p = trace_satellite(&street(), antenna, direction(270.0, 40.0), &single_bounce());
        assert!(!p.line_of_sight);
        let east_wall = p
            .paths
            .iter()
            .find(|r| r.sources == vec![FacetSource::Box(0)])
            .expect("reflection off the east wall");
        assert_relative_eq!(
            east_wall.excess_delay_m,
            2.0 * 10.0 * elevation.cos(),
            epsilon = 1e-9
        );
        assert_relative_eq!(east_wall.points_m[0].x, 10.0, epsilon = 1e-9);
    }

    #[test]
    fn two_bounces_between_parallel_walls_follow_the_image_method() {
        // Every two-bounce path found between the street walls must have the
        // excess delay of its unfolded image: the antenna mirrored in both
        // walls.
        let antenna = Vec3::new(0.0, 0.0, 1.0);
        let config = TraceConfig {
            max_reflections: 2,
            diffraction: false,
            ..TraceConfig::default()
        };
        let mut found = 0;
        for az in [80.0, 100.0, 260.0, 280.0] {
            let u = direction(az, 35.0);
            let p = trace_satellite(&street(), antenna, u, &config);
            for path in p
                .paths
                .iter()
                .filter(|p| p.kind == (PathKind::Reflected { bounces: 2 }))
            {
                found += 1;
                // Unfolded: the antenna's double image across both walls.
                let (n1, n2) = (
                    facet_normal(&path.sources[0]),
                    facet_normal(&path.sources[1]),
                );
                let p1 = path.points_m[0];
                let p2 = path.points_m[1];
                let image1 = antenna - n1 * (2.0 * (antenna - p1).dot(n1));
                let image2 = image1 - n2 * (2.0 * (image1 - p2).dot(n2));
                let expected = (antenna - image2).dot(u);
                assert_relative_eq!(path.excess_delay_m, expected, epsilon = 1e-6);
            }
        }
        assert!(found > 0, "no two-bounce paths in a street");
    }

    fn facet_normal(source: &FacetSource) -> Vec3 {
        match source {
            FacetSource::Box(0) => Vec3::NEG_X,
            FacetSource::Box(1) => Vec3::X,
            _ => Vec3::Z,
        }
    }

    #[test]
    fn roof_edge_diffraction_is_weak_deep_in_shadow_and_strong_near_the_edge() {
        let mut scene = GnssScene::default();
        // A 10 m high wall 20 m east; satellite due east just below the edge.
        scene.add_box(SceneBox::building(
            20.0,
            -100.0,
            21.0,
            100.0,
            10.0,
            SurfaceMaterial::CONCRETE,
        ));
        let antenna = Vec3::new(0.0, 0.0, 1.0);
        let edge_elevation = (9.0_f64 / 20.0).atan().to_degrees();
        let config = TraceConfig {
            max_reflections: 0,
            ..TraceConfig::default()
        };
        let near = trace_satellite(
            &scene,
            antenna,
            direction(90.0, edge_elevation - 0.3),
            &config,
        );
        assert!(!near.line_of_sight);
        let diffracted = near
            .paths
            .iter()
            .find(|p| p.kind == PathKind::Diffracted)
            .expect("diffracted path");
        // The edge point is on the near top edge.
        assert_relative_eq!(diffracted.points_m[0].z, 10.0, epsilon = 1e-6);
        let near_loss = diffracted.loss_db(0.19);
        let deep = trace_satellite(
            &scene,
            antenna,
            direction(90.0, edge_elevation - 10.0),
            &config,
        );
        let deep_loss = deep
            .paths
            .iter()
            .find(|p| p.kind == PathKind::Diffracted)
            .map_or(f64::INFINITY, |p| p.loss_db(0.19));
        assert!(near_loss > 6.0 && near_loss < 20.0, "near {near_loss}");
        assert!(
            deep_loss > near_loss + 10.0,
            "near {near_loss} deep {deep_loss}"
        );
        // Longer wavelengths diffract more strongly.
        assert!(diffracted.loss_db(0.255) < near_loss);
    }

    #[test]
    fn canopy_attenuates_the_direct_path_and_distant_walls_are_culled() {
        use crate::scene::FoliageVolume;
        let mut scene = GnssScene::open_sky();
        scene.foliage.push(FoliageVolume {
            min_m: Vec3::new(-3.0, -3.0, 3.0),
            max_m: Vec3::new(3.0, 3.0, 7.0),
            loss_db_per_m: 1.0,
        });
        let p = trace_satellite(
            &scene,
            Vec3::new(0.0, 0.0, 1.0),
            direction(0.0, 90.0),
            &TraceConfig::default(),
        );
        assert!(p.line_of_sight);
        // 4 m of canopy straight up.
        assert_relative_eq!(p.paths[0].loss_db(0.19), 4.0, epsilon = 1e-9);

        let mut far = street();
        far.boxes.iter_mut().for_each(|b| {
            b.min_m.x += 500.0;
            b.max_m.x += 500.0;
        });
        let culled = trace_satellite(
            &far,
            Vec3::new(0.0, 0.0, 1.0),
            direction(90.0, 30.0),
            &TraceConfig::default(),
        );
        assert!(culled
            .paths
            .iter()
            .all(|path| path.kind == PathKind::Direct));
    }

    #[test]
    fn a_street_traced_as_a_mesh_matches_the_same_street_as_boxes() {
        let boxes = street();
        let mut triangles = Vec::new();
        for (object, b) in boxes.boxes.iter().enumerate() {
            triangles.extend(crate::mesh::box_triangles(
                b.min_m,
                b.max_m,
                b.material,
                object as u32,
            ));
        }
        let mesh = GnssScene {
            mesh: Some(std::sync::Arc::new(crate::mesh::TriangleMesh::new(
                triangles,
            ))),
            ..GnssScene::default()
        };
        let config = TraceConfig {
            min_reflector_area_m2: 0.0,
            ..TraceConfig::default()
        };
        let antenna = Vec3::new(1.0, 3.0, 1.5);
        let summary = |p: &Propagation| {
            let mut reflected: Vec<(u8, f64)> = p
                .paths
                .iter()
                .filter_map(|path| match path.kind {
                    PathKind::Reflected { bounces } => Some((bounces, path.excess_delay_m)),
                    _ => None,
                })
                .collect();
            reflected.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
            // A reflection point on a wall's triangulation diagonal is found
            // in both triangles.
            reflected.dedup_by(|a, b| a.0 == b.0 && (a.1 - b.1).abs() < 1.0e-6);
            let diffracted = p
                .paths
                .iter()
                .filter(|path| path.kind == PathKind::Diffracted)
                .map(|path| path.excess_delay_m)
                .fold(f64::INFINITY, f64::min);
            (p.line_of_sight, reflected, diffracted)
        };
        let mut compared = 0;
        for az in (7..360).step_by(23) {
            for el in [12.0, 25.0, 40.0, 63.0] {
                let u = direction(f64::from(az), el);
                let (los_b, refl_b, diff_b) =
                    summary(&trace_satellite(&boxes, antenna, u, &config));
                let (los_m, refl_m, diff_m) = summary(&trace_satellite(&mesh, antenna, u, &config));
                assert_eq!(los_b, los_m, "az {az} el {el}");
                assert_eq!(
                    refl_b.len(),
                    refl_m.len(),
                    "az {az} el {el}: {refl_b:?} vs {refl_m:?}"
                );
                for (b, m) in refl_b.iter().zip(&refl_m) {
                    assert_eq!(b.0, m.0);
                    assert_relative_eq!(b.1, m.1, epsilon = 1.0e-6);
                }
                assert_eq!(diff_b.is_finite(), diff_m.is_finite(), "az {az} el {el}");
                if diff_b.is_finite() {
                    assert_relative_eq!(diff_b, diff_m, epsilon = 1.0e-6);
                }
                compared += refl_b.len();
            }
        }
        assert!(compared > 20, "{compared}");
    }

    #[test]
    fn knife_edge_loss_matches_itu_values() {
        assert_relative_eq!(knife_edge_loss_db(0.0), 6.0, epsilon = 0.1);
        assert!(knife_edge_loss_db(1.0) > 13.0 && knife_edge_loss_db(1.0) < 14.5);
        assert_eq!(knife_edge_loss_db(-1.0), 0.0);
    }

    #[test]
    fn wide_roof_blocks_everything() {
        let mut scene = GnssScene::open_sky();
        scene.add_box(SceneBox::new(
            Vec3::new(-20.0, -20.0, 3.0),
            Vec3::new(20.0, 20.0, 3.3),
            SurfaceMaterial::CONCRETE,
        ));
        for el in [20.0, 50.0, 89.0] {
            let p = trace_satellite(
                &scene,
                Vec3::new(0.0, 0.0, 1.0),
                direction(10.0, el),
                &TraceConfig::default(),
            );
            assert!(!p.line_of_sight);
            // Only heavily attenuated paths can remain.
            assert!(
                p.paths.iter().all(|path| path.loss_db(0.19) > 15.0),
                "elevation {el}"
            );
        }
    }
}
