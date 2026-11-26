//! The worlds the robot explores: example 127's warehouse, and three kinds
//! generated from a seed (`WorldRandom`, so a seed always gives the same
//! world).
//!
//! - **office**: a corridor with rooms on both sides, each with a door onto
//!   it, some rooms also joined to their neighbour, furniture against the
//!   outer walls. Many dead ends and small rooms.
//! - **loop**: one long corridor around a solid block, with dead-end notches
//!   cut into the block. Odometry drift builds up before the robot comes back.
//! - **maze**: a grid of cells carved by a randomized depth-first search,
//!   with a few extra walls removed so it has loops.
//!
//! World coordinates are the physics world's `(x, z)`; the SLAM plane maps
//! `x -> x` and `z -> y`, so a rectangle here is a rectangle on the map.

use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Vec3};
use rne_nav::Pose2d;
use rne_physics::{Collider, ColliderShape, RigidBody, RigidBodyType};
use rne_world::{Transform3, WorldRandom};

const WALL_HALF_M: f64 = 0.1;
const WALL_HEIGHT_M: f64 = 2.4;
/// Wide enough for the robot (0.25 m radius) and its 0.3 m clearance on
/// both sides, with room for grid rounding.
const DOOR_M: f64 = 1.4;

/// An axis-aligned footprint: centre `(x, z)` and half extents, in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Footprint {
    pub(crate) x: f64,
    pub(crate) z: f64,
    pub(crate) half_x: f64,
    pub(crate) half_z: f64,
    pub(crate) height: f64,
}

impl Footprint {
    const fn new(x: f64, z: f64, half_x: f64, half_z: f64, height: f64) -> Self {
        Self {
            x,
            z,
            half_x,
            half_z,
            height,
        }
    }

    /// A box spanning `[x0, x1] x [z0, z1]`.
    fn span(x0: f64, x1: f64, z0: f64, z1: f64, height: f64) -> Self {
        Self::new(
            0.5 * (x0 + x1),
            0.5 * (z0 + z1),
            0.5 * (x1 - x0).abs(),
            0.5 * (z1 - z0).abs(),
            height,
        )
    }

    /// Whether a disc of radius `margin` at `(x, z)` overlaps the box.
    pub(crate) fn contains(&self, x: f64, z: f64, margin: f64) -> bool {
        let dx = ((x - self.x).abs() - self.half_x).max(0.0);
        let dz = ((z - self.z).abs() - self.half_z).max(0.0);
        dx.hypot(dz) <= margin
    }
}

/// A world to explore.
#[derive(Clone, Debug)]
pub(crate) struct Scene {
    /// For titles and file names, e.g. `office-3`.
    pub(crate) name: String,
    pub(crate) footprints: Vec<Footprint>,
    /// Inside of the outer walls, `(min_x, max_x, min_z, max_z)`.
    pub(crate) interior: (f64, f64, f64, f64),
    /// Start poses; the first is the default.
    pub(crate) starts: Vec<Pose2d>,
    /// Places every exploration should be able to reach (room and cell
    /// centres, the loop's corners); checked by the tests.
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) checkpoints: Vec<(f64, f64)>,
}

impl Scene {
    /// Whether a disc of `radius` at `(x, z)` touches anything.
    pub(crate) fn collides(&self, x: f64, z: f64, radius: f64) -> bool {
        self.footprints.iter().any(|f| f.contains(x, z, radius))
    }

    /// Spawns one fixed box per footprint.
    pub(crate) fn spawn(&self, world: &mut World) {
        for footprint in &self.footprints {
            let entity = spawn_named(world, "structure");
            world.entity_mut(entity).insert((
                RigidBody {
                    body_type: RigidBodyType::Fixed,
                    ..RigidBody::default()
                },
                Collider {
                    shape: ColliderShape::Cuboid {
                        half_extents_m: Vec3::new(
                            footprint.half_x,
                            0.5 * footprint.height,
                            footprint.half_z,
                        ),
                    },
                    ..Collider::default()
                },
                Transform3::from_translation_rotation(
                    Vec3::new(footprint.x, 0.5 * footprint.height, footprint.z),
                    Quat::IDENTITY,
                ),
            ));
        }
    }
}

/// The kinds of world.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Kind {
    Warehouse,
    Office,
    Loop,
    Maze,
}

impl Kind {
    pub(crate) const GENERATED: [Kind; 3] = [Kind::Office, Kind::Loop, Kind::Maze];

    pub(crate) fn parse(text: &str) -> Option<Self> {
        match text {
            "warehouse" => Some(Self::Warehouse),
            "office" => Some(Self::Office),
            "loop" => Some(Self::Loop),
            "maze" => Some(Self::Maze),
            _ => None,
        }
    }

    pub(crate) fn label(self) -> &'static str {
        match self {
            Self::Warehouse => "warehouse",
            Self::Office => "office",
            Self::Loop => "loop",
            Self::Maze => "maze",
        }
    }
}

/// The world of `kind` for `seed` (the warehouse ignores the seed).
pub(crate) fn generate(kind: Kind, seed: u64) -> Scene {
    match kind {
        Kind::Warehouse => warehouse(),
        Kind::Office => office(seed),
        Kind::Loop => loop_world(seed),
        Kind::Maze => maze(seed),
    }
}

/// Adds small bumps (pillars, shelves, door frames) to every wall face that
/// faces the inside, 1.2 to 2.5 m apart. A long bare wall is the same
/// everywhere along it to a 2D scan, so the scan matcher cannot tell how far
/// the robot moved along a corridor; real walls are rarely that bare. Uses
/// its own random stream, so the layout of a seed does not change.
fn texture_walls(footprints: &mut Vec<Footprint>, interior: (f64, f64, f64, f64), seed: u64) {
    let mut rng = WorldRandom::new(seed ^ 0x5eed_7e47_u64);
    let (min_x, max_x, min_z, max_z) = interior;
    let inside = |x: f64, z: f64| x > min_x && x < max_x && z > min_z && z < max_z;
    let mut bumps = Vec::new();
    for f in footprints.iter().filter(|f| f.height >= 2.0) {
        // Faces: (along x?, fixed coordinate, from, to, outward sign).
        let faces = [
            (true, f.z + f.half_z, f.x - f.half_x, f.x + f.half_x, 1.0),
            (true, f.z - f.half_z, f.x - f.half_x, f.x + f.half_x, -1.0),
            (false, f.x + f.half_x, f.z - f.half_z, f.z + f.half_z, 1.0),
            (false, f.x - f.half_x, f.z - f.half_z, f.z + f.half_z, -1.0),
        ];
        for (along_x, fixed, from, to, sign) in faces {
            let (from, to) = (
                from.max(if along_x { min_x } else { min_z }),
                to.min(if along_x { max_x } else { max_z }),
            );
            if to - from < 1.2 {
                continue;
            }
            let mid = 0.5 * (from + to);
            let probe = fixed + sign * 0.05;
            let faces_inside = if along_x {
                inside(mid, probe)
            } else {
                inside(probe, mid)
            };
            if !faces_inside {
                continue;
            }
            let mut at = from + rng.uniform_f64(0.3, 1.2);
            loop {
                let half = rng.uniform_f64(0.15, 0.35);
                let depth = rng.uniform_f64(0.12, 0.3);
                if at + 2.0 * half > to - 0.25 {
                    break;
                }
                let c = at + half;
                let n = fixed + sign * 0.5 * depth;
                bumps.push(if along_x {
                    Footprint::new(c, n, half, 0.5 * depth, f.height)
                } else {
                    Footprint::new(n, c, 0.5 * depth, half, f.height)
                });
                at += 2.0 * half + rng.uniform_f64(1.2, 2.5);
            }
        }
    }
    footprints.extend(bumps);
}

/// The four walls around `[min_x, max_x] x [min_z, max_z]`.
fn outer_walls(min_x: f64, max_x: f64, min_z: f64, max_z: f64) -> Vec<Footprint> {
    let t = 2.0 * WALL_HALF_M;
    vec![
        Footprint::span(min_x - t, max_x + t, max_z, max_z + t, WALL_HEIGHT_M),
        Footprint::span(min_x - t, max_x + t, min_z - t, min_z, WALL_HEIGHT_M),
        Footprint::span(max_x, max_x + t, min_z - t, max_z + t, WALL_HEIGHT_M),
        Footprint::span(min_x - t, min_x, min_z - t, max_z + t, WALL_HEIGHT_M),
    ]
}

/// A wall along x at `z` from `x0` to `x1`, with openings `(centre, width)`.
fn wall_x(x0: f64, x1: f64, z: f64, openings: &[(f64, f64)]) -> Vec<Footprint> {
    let mut cuts: Vec<(f64, f64)> = openings
        .iter()
        .map(|&(c, w)| (c - 0.5 * w, c + 0.5 * w))
        .collect();
    cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut walls = Vec::new();
    let mut from = x0;
    for (a, b) in cuts.into_iter().chain(std::iter::once((x1, x1))) {
        if a - from > 1e-6 {
            walls.push(Footprint::span(
                from,
                a,
                z - WALL_HALF_M,
                z + WALL_HALF_M,
                WALL_HEIGHT_M,
            ));
        }
        from = from.max(b);
    }
    walls
}

/// A wall along z at `x` from `z0` to `z1`, with openings `(centre, width)`.
fn wall_z(z0: f64, z1: f64, x: f64, openings: &[(f64, f64)]) -> Vec<Footprint> {
    wall_x(z0, z1, x, openings)
        .into_iter()
        .map(|f| Footprint::new(f.z, f.x, f.half_z, f.half_x, f.height))
        .collect()
}

/// Example 127's warehouse on its first day.
fn warehouse() -> Scene {
    const PALLET: f64 = 0.45;
    let mut footprints = vec![
        Footprint::new(0.0, 4.6, 7.2, 0.1, 2.4),    // north wall
        Footprint::new(0.0, -4.6, 7.2, 0.1, 2.4),   // south wall
        Footprint::new(7.1, 0.0, 0.1, 4.6, 2.4),    // east wall
        Footprint::new(-7.1, 0.0, 0.1, 4.6, 2.4),   // west wall
        Footprint::new(-3.0, -3.7, 2.2, 0.45, 2.0), // long rack
        Footprint::new(3.4, -3.7, 1.4, 0.45, 2.0),  // short rack
        Footprint::new(0.0, 1.05, 0.2, 0.2, 2.4),   // column
        Footprint::new(5.9, 3.5, 1.2, 1.0, 2.4),    // office block
    ];
    for (x, z) in [
        (-3.0, -0.2),
        (-1.6, -0.2),
        (1.4, -0.2),
        (-5.0, 3.3),
        (-1.8, 3.3),
        (0.2, 3.3),
    ] {
        footprints.push(Footprint::new(x, z, PALLET, PALLET, 1.2));
    }
    Scene {
        name: "warehouse".to_string(),
        footprints,
        interior: (-7.0, 7.0, -4.5, 4.5),
        starts: vec![
            Pose2d::new(-5.0, -2.0, 0.0),
            Pose2d::new(5.0, -2.0, std::f64::consts::PI),
            Pose2d::new(-5.0, 1.9, 0.0),
            Pose2d::new(3.0, 1.5, std::f64::consts::FRAC_PI_2),
        ],
        checkpoints: vec![(-5.0, -2.0), (5.0, -2.0), (-5.0, 1.9), (3.0, 1.5)],
    }
}

/// A corridor along x with rooms north and south of it.
fn office(seed: u64) -> Scene {
    let mut rng = WorldRandom::new(seed);
    let cols = 3 + rng.uniform_usize(3);
    let widths: Vec<f64> = (0..cols).map(|_| rng.uniform_f64(3.0, 4.5)).collect();
    let width: f64 = widths.iter().sum();
    let corridor = 2.2;
    let depth_north = rng.uniform_f64(3.0, 4.0);
    let depth_south = rng.uniform_f64(3.0, 4.0);
    let (min_x, max_x) = (-0.5 * width, 0.5 * width);
    let (cz_n, cz_s) = (0.5 * corridor, -0.5 * corridor);
    let (min_z, max_z) = (cz_s - depth_south, cz_n + depth_north);

    let mut footprints = outer_walls(min_x, max_x, min_z, max_z);
    let mut checkpoints = Vec::new();
    // Room edges along x.
    let mut edges = vec![min_x];
    for w in &widths {
        edges.push(edges[edges.len() - 1] + w);
    }
    for (side, cz, far_z) in [(1.0, cz_n, max_z), (-1.0, cz_s, min_z)] {
        // Corridor wall with one door per room.
        let doors: Vec<(f64, f64)> = edges
            .windows(2)
            .map(|e| {
                let margin = 0.5 * DOOR_M + 0.4;
                (rng.uniform_f64(e[0] + margin, e[1] - margin), DOOR_M)
            })
            .collect();
        footprints.extend(wall_x(min_x, max_x, cz, &doors));
        // Partitions between rooms, sometimes with a door near the corridor.
        for &x in &edges[1..edges.len() - 1] {
            let (z0, z1) = if side > 0.0 { (cz, far_z) } else { (far_z, cz) };
            let openings = if rng.uniform_f64(0.0, 1.0) < 0.35 {
                let centre = cz + side * rng.uniform_f64(0.2 + 0.5 * DOOR_M, 1.8);
                vec![(centre, DOOR_M)]
            } else {
                Vec::new()
            };
            footprints.extend(wall_z(z0, z1, x, &openings));
        }
        // Furniture against the outer wall, leaving 2 m by the corridor.
        for e in edges.windows(2) {
            checkpoints.push((0.5 * (e[0] + e[1]), cz + side * 1.2));
            for _ in 0..rng.uniform_usize(3) {
                let half_x = rng.uniform_f64(0.3, 0.6);
                let half_z = rng.uniform_f64(0.25, 0.5);
                let x = rng.uniform_f64(e[0] + 0.1 + half_x, e[1] - 0.1 - half_x);
                let z = far_z - side * (0.1 + half_z);
                footprints.push(Footprint::new(x, z, half_x, half_z, 0.9));
            }
        }
    }
    checkpoints.push((min_x + 1.0, 0.0));
    checkpoints.push((max_x - 1.0, 0.0));
    texture_walls(&mut footprints, (min_x, max_x, min_z, max_z), seed);
    Scene {
        name: format!("office-{seed}"),
        footprints,
        interior: (min_x, max_x, min_z, max_z),
        starts: vec![
            Pose2d::new(min_x + 1.0, 0.0, 0.0),
            Pose2d::new(max_x - 1.0, 0.0, std::f64::consts::PI),
            Pose2d::new(0.0, 0.0, 0.0),
        ],
        checkpoints,
    }
}

/// One corridor around a solid block, with dead-end notches cut into it.
fn loop_world(seed: u64) -> Scene {
    let mut rng = WorldRandom::new(seed);
    let width = rng.uniform_f64(20.0, 26.0);
    let height = rng.uniform_f64(13.0, 17.0);
    let corridor = rng.uniform_f64(2.2, 2.8);
    let (min_x, max_x) = (-0.5 * width, 0.5 * width);
    let (min_z, max_z) = (-0.5 * height, 0.5 * height);
    let mut footprints = outer_walls(min_x, max_x, min_z, max_z);

    // The block, cut into slabs along x; a notch lowers or raises one slab.
    let (bx0, bx1) = (min_x + corridor, max_x - corridor);
    let (bz0, bz1) = (min_z + corridor, max_z - corridor);
    let notch_w = 1.8;
    let slots = ((bx1 - bx0 - 2.0) / (notch_w + 1.2)).floor() as usize;
    let mut notches: Vec<(f64, bool, f64)> = Vec::new(); // (x0, from north, depth)
    for slot in 0..slots {
        if rng.uniform_f64(0.0, 1.0) < 0.45 {
            let x0 = bx0 + 1.0 + slot as f64 * (notch_w + 1.2);
            let depth = rng.uniform_f64(1.5, 0.5 * (bz1 - bz0) - 0.3);
            notches.push((x0, rng.uniform_usize(2) == 0, depth));
        }
    }
    let mut checkpoints = vec![
        (min_x + 0.5 * corridor, min_z + 0.5 * corridor),
        (max_x - 0.5 * corridor, min_z + 0.5 * corridor),
        (max_x - 0.5 * corridor, max_z - 0.5 * corridor),
        (min_x + 0.5 * corridor, max_z - 0.5 * corridor),
    ];
    let mut x = bx0;
    for &(x0, north, depth) in &notches {
        footprints.push(Footprint::span(x, x0, bz0, bz1, WALL_HEIGHT_M));
        let (z0, z1) = if north {
            (bz0, bz1 - depth)
        } else {
            (bz0 + depth, bz1)
        };
        footprints.push(Footprint::span(x0, x0 + notch_w, z0, z1, WALL_HEIGHT_M));
        let mouth = if north { bz1 } else { bz0 };
        let inside = if north { -1.0 } else { 1.0 };
        checkpoints.push((x0 + 0.5 * notch_w, mouth + inside * 0.5 * depth));
        x = x0 + notch_w;
    }
    footprints.push(Footprint::span(x, bx1, bz0, bz1, WALL_HEIGHT_M));
    texture_walls(&mut footprints, (min_x, max_x, min_z, max_z), seed);
    Scene {
        name: format!("loop-{seed}"),
        footprints,
        interior: (min_x, max_x, min_z, max_z),
        starts: vec![
            Pose2d::new(min_x + 0.5 * corridor, min_z + 0.5 * corridor, 0.0),
            Pose2d::new(
                max_x - 0.5 * corridor,
                max_z - 0.5 * corridor,
                std::f64::consts::PI,
            ),
        ],
        checkpoints,
    }
}

/// A grid of cells carved by a randomized depth-first search.
fn maze(seed: u64) -> Scene {
    let mut rng = WorldRandom::new(seed);
    let cols = 6 + rng.uniform_usize(3);
    let rows = 4 + rng.uniform_usize(2);
    let cell = 2.6;
    let (min_x, min_z) = (-0.5 * cols as f64 * cell, -0.5 * rows as f64 * cell);
    let (max_x, max_z) = (-min_x, -min_z);
    // open_east[r][c]: no wall between (c, r) and (c + 1, r); open_north
    // likewise towards (c, r + 1).
    let mut open_east = vec![vec![false; cols]; rows];
    let mut open_north = vec![vec![false; cols]; rows];
    let mut visited = vec![vec![false; cols]; rows];
    let mut stack = vec![(0usize, 0usize)];
    visited[0][0] = true;
    while let Some(&(c, r)) = stack.last() {
        let mut next = Vec::new();
        if c + 1 < cols && !visited[r][c + 1] {
            next.push((c + 1, r));
        }
        if c > 0 && !visited[r][c - 1] {
            next.push((c - 1, r));
        }
        if r + 1 < rows && !visited[r + 1][c] {
            next.push((c, r + 1));
        }
        if r > 0 && !visited[r - 1][c] {
            next.push((c, r - 1));
        }
        if next.is_empty() {
            stack.pop();
            continue;
        }
        let (nc, nr) = next[rng.uniform_usize(next.len())];
        match (nc as isize - c as isize, nr as isize - r as isize) {
            (1, 0) => open_east[r][c] = true,
            (-1, 0) => open_east[r][nc] = true,
            (0, 1) => open_north[r][c] = true,
            _ => open_north[nr][c] = true,
        }
        visited[nr][nc] = true;
        stack.push((nc, nr));
    }
    // A few extra openings, so the maze has loops.
    for r in 0..rows {
        for c in 0..cols {
            if c + 1 < cols && !open_east[r][c] && rng.uniform_f64(0.0, 1.0) < 0.12 {
                open_east[r][c] = true;
            }
            if r + 1 < rows && !open_north[r][c] && rng.uniform_f64(0.0, 1.0) < 0.12 {
                open_north[r][c] = true;
            }
        }
    }
    let mut footprints = outer_walls(min_x, max_x, min_z, max_z);
    let mut checkpoints = Vec::new();
    for r in 0..rows {
        for c in 0..cols {
            let x0 = min_x + c as f64 * cell;
            let z0 = min_z + r as f64 * cell;
            checkpoints.push((x0 + 0.5 * cell, z0 + 0.5 * cell));
            if c + 1 < cols && !open_east[r][c] {
                footprints.extend(wall_z(z0, z0 + cell, x0 + cell, &[]));
            }
            if r + 1 < rows && !open_north[r][c] {
                footprints.extend(wall_x(x0, x0 + cell, z0 + cell, &[]));
            }
        }
    }
    let first = checkpoints[0];
    texture_walls(&mut footprints, (min_x, max_x, min_z, max_z), seed);
    Scene {
        name: format!("maze-{seed}"),
        footprints,
        interior: (min_x, max_x, min_z, max_z),
        starts: vec![Pose2d::new(first.0, first.1, 0.0)],
        checkpoints,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;

    /// The robot's radius plus its clearance, and a little for the grid.
    const CLEARANCE_M: f64 = 0.6;
    const STEP_M: f64 = 0.05;

    /// Cells of the interior whose centre a robot with `CLEARANCE_M` can
    /// reach from `start`, by 4-connected flood fill.
    fn reachable(scene: &Scene, start: (f64, f64)) -> (Vec<bool>, usize, (f64, f64)) {
        let (min_x, max_x, min_z, max_z) = scene.interior;
        let w = ((max_x - min_x) / STEP_M).ceil() as usize;
        let h = ((max_z - min_z) / STEP_M).ceil() as usize;
        let centre = |i: usize, j: usize| {
            (
                min_x + (i as f64 + 0.5) * STEP_M,
                min_z + (j as f64 + 0.5) * STEP_M,
            )
        };
        let free: Vec<bool> = (0..w * h)
            .map(|k| {
                let (x, z) = centre(k % w, k / w);
                !scene.collides(x, z, CLEARANCE_M)
            })
            .collect();
        let cell = |(x, z): (f64, f64)| {
            let i = ((x - min_x) / STEP_M).floor() as usize;
            let j = ((z - min_z) / STEP_M).floor() as usize;
            j * w + i
        };
        let mut seen = vec![false; w * h];
        let s = cell(start);
        assert!(free[s], "{}: start {start:?} is not free", scene.name);
        seen[s] = true;
        let mut queue = VecDeque::from([s]);
        while let Some(k) = queue.pop_front() {
            let (i, j) = (k % w, k / w);
            let mut push = |n: usize| {
                if free[n] && !seen[n] {
                    seen[n] = true;
                    queue.push_back(n);
                }
            };
            if i > 0 {
                push(k - 1);
            }
            if i + 1 < w {
                push(k + 1);
            }
            if j > 0 {
                push(k - w);
            }
            if j + 1 < h {
                push(k + w);
            }
        }
        (seen, w, (min_x, min_z))
    }

    fn all_scenes() -> Vec<Scene> {
        let mut scenes = vec![generate(Kind::Warehouse, 0)];
        for kind in Kind::GENERATED {
            for seed in 0..20 {
                scenes.push(generate(kind, seed));
            }
        }
        scenes
    }

    #[test]
    fn every_checkpoint_and_start_is_reachable() {
        for scene in all_scenes() {
            let start = scene.starts[0];
            let (seen, w, (min_x, min_z)) = reachable(&scene, (start.x_m, start.y_m));
            for &(x, z) in scene.checkpoints.iter().chain(
                scene
                    .starts
                    .iter()
                    .map(|s| (s.x_m, s.y_m))
                    .collect::<Vec<_>>()
                    .iter(),
            ) {
                let i = ((x - min_x) / STEP_M).floor() as usize;
                let j = ((z - min_z) / STEP_M).floor() as usize;
                assert!(
                    seen[j * w + i],
                    "{}: ({x:.2}, {z:.2}) cannot be reached from the start",
                    scene.name
                );
            }
        }
    }

    #[test]
    fn a_seed_always_gives_the_same_world() {
        for kind in Kind::GENERATED {
            for seed in 0..5 {
                let (a, b) = (generate(kind, seed), generate(kind, seed));
                assert_eq!(a.footprints, b.footprints);
                assert_eq!(a.starts, b.starts);
            }
            assert_ne!(generate(kind, 0).footprints, generate(kind, 1).footprints);
        }
    }
}
