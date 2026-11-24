//! The street surfaces of the render: asphalt carriageways, sidewalks raised
//! behind curbs and paved in tiles, white edge lines, tactile paving and
//! zebra crossings where the roads enter their intersections.
//!
//! PLATEAU's LOD1 road areas (`tran`) run from building line to building
//! line, sidewalks included, without saying where the carriageway is. They
//! are rasterized at `CELL_M`; the distance of every raster node from the
//! nearest point outside the roads then places the sidewalks: the outer
//! `SIDEWALK_M` of every road at least `WIDE_ROAD_M` wide. Marching squares
//! turn the raster back into smooth curbs and lines.

use crate::plateau::{Cover, PlateauCity, Scenery};
use crate::render::{linear, p, SolidMesh};
use rne_math::Vec3;
use rne_render::{ImageFrame, RenderSceneItem};
use std::collections::VecDeque;
use std::sync::Arc;

/// A point or direction in the ENU plane.
type Point = (f64, f64);

/// Raster spacing, meters.
const CELL_M: f64 = 0.5;
/// Sidewalk width, meters.
const SIDEWALK_M: f64 = 3.0;
/// Roads narrower than this (building line to building line) have no
/// raised sidewalks, only painted edge lines, meters.
const WIDE_ROAD_M: f64 = 10.0;
/// Curb height (the sidewalk above the carriageway), meters.
const CURB_M: f64 = 0.15;
/// The carriageway's surface relative to the terrain, meters.
pub(crate) const ROAD_LIFT_M: f64 = -0.04;
/// Sizes of the repeating textures on the ground, meters.
pub(crate) const ASPHALT_TILE_M: f64 = 16.0;
const PAVING_TILE_M: f64 = 4.8;

/// The rasterized road areas.
pub(crate) struct Streets {
    origin: (f64, f64),
    nx: usize,
    ny: usize,
    road: Vec<bool>,
    /// Distance from the nearest point outside the roads, meters.
    distance: Vec<f32>,
    wide: Vec<bool>,
    /// Distance from the nearest intersection, meters.
    from_junction: Vec<f32>,
}

impl Streets {
    /// Rasterizes the city's roads over a rectangle (ENU x0, y0, x1, y1).
    pub(crate) fn new(city: &PlateauCity, bounds: [f64; 4]) -> Self {
        let [x0, y0, x1, y1] = bounds;
        let nx = ((x1 - x0) / CELL_M) as usize + 1;
        let ny = ((y1 - y0) / CELL_M) as usize + 1;
        let road = rasterize(city.roads.iter(), (x0, y0), nx, ny);
        let junction = rasterize(city.junctions.iter().flatten(), (x0, y0), nx, ny);
        let outside_junctions: Vec<bool> = junction.iter().map(|&j| !j).collect();
        let from_junction = distance_outside(&outside_junctions, nx, ny);
        // Close the slivers between neighbouring road polygons.
        let road = erode(&dilate(&road, nx, ny), nx, ny);
        let distance = distance_outside(&road, nx, ny);
        let half_width = (WIDE_ROAD_M / 2.0) as f32;
        let reach = (WIDE_ROAD_M / CELL_M) as usize;
        let wide = max_filter(&distance, nx, ny, reach)
            .iter()
            .map(|&d| d >= half_width)
            .collect();
        Self {
            origin: (x0, y0),
            nx,
            ny,
            road,
            distance,
            wide,
            from_junction,
        }
    }

    /// Whether a node is on a road, its distance from outside the roads,
    /// whether the road is wide and its distance from an intersection.
    fn at(&self, i: usize, j: usize) -> (bool, f64, bool, f64) {
        let k = j * self.nx + i;
        (
            self.road[k],
            f64::from(self.distance[k]),
            self.wide[k],
            f64::from(self.from_junction[k]),
        )
    }

    fn node(&self, i: usize, j: usize) -> (f64, f64) {
        (
            self.origin.0 + i as f64 * CELL_M,
            self.origin.1 + j as f64 * CELL_M,
        )
    }

    fn nearest(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let i = ((x - self.origin.0) / CELL_M).round();
        let j = ((y - self.origin.1) / CELL_M).round();
        (i >= 0.0 && j >= 0.0 && (i as usize) < self.nx && (j as usize) < self.ny)
            .then_some((i as usize, j as usize))
    }

    /// Whether a point is on a raised sidewalk.
    fn on_sidewalk(&self, x: f64, y: f64) -> bool {
        self.nearest(x, y).is_some_and(|(i, j)| {
            let (road, d, wide, _) = self.at(i, j);
            road && wide && d < SIDEWALK_M
        })
    }

    /// How far above the terrain the street surface is at a point, meters:
    /// people and the robot stand on it.
    pub(crate) fn surface_m(&self, x: f64, y: f64) -> f64 {
        if self.on_sidewalk(x, y) {
            ROAD_LIFT_M + CURB_M
        } else if self
            .nearest(x, y)
            .is_some_and(|(i, j)| self.road[j * self.nx + i])
        {
            ROAD_LIFT_M
        } else {
            ROAD_LIFT_M + CURB_M
        }
    }

    /// Marching squares in the cell with lower-left node (i, j): the part
    /// where `field` is positive and the zero contour's segments, or `None`
    /// where `field` declines a corner.
    fn cell(
        &self,
        i: usize,
        j: usize,
        field: &impl Fn(bool, f64, bool, f64) -> Option<f64>,
    ) -> Option<Cell> {
        let corners = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)];
        let mut g = [0.0; 4];
        for (k, &(a, b)) in corners.iter().enumerate() {
            let (r, d, w, f) = self.at(a, b);
            g[k] = field(r, d, w, f)?;
        }
        let xy = corners.map(|(a, b)| self.node(a, b));
        let mut cell = Cell {
            polygon: Vec::new(),
            crossings: Vec::new(),
            full: g.iter().all(|&v| v > 0.0),
        };
        if cell.full || g.iter().all(|&v| v <= 0.0) {
            return Some(cell);
        }
        for k in 0..4 {
            let (a, b) = (k, (k + 1) % 4);
            if g[a] > 0.0 {
                cell.polygon.push(xy[a]);
            }
            if (g[a] > 0.0) != (g[b] > 0.0) {
                let t = g[a] / (g[a] - g[b]);
                let c = (
                    xy[a].0 + t * (xy[b].0 - xy[a].0),
                    xy[a].1 + t * (xy[b].1 - xy[a].1),
                );
                cell.polygon.push(c);
                cell.crossings.push(c);
            }
        }
        Some(cell)
    }

    /// The zero contour of `field` over every cell whose corners are all on
    /// roads.
    fn contour(
        &self,
        field: impl Fn(bool, f64, bool, f64) -> Option<f64>,
        mut line: impl FnMut((f64, f64), (f64, f64)),
    ) {
        for j in 0..self.ny - 1 {
            for i in 0..self.nx - 1 {
                let all_road = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
                    .iter()
                    .all(|&(a, b)| self.road[b * self.nx + a]);
                if !all_road {
                    continue;
                }
                if let Some(cell) = self.cell(i, j, &field) {
                    for pair in cell.crossings.chunks_exact(2) {
                        line(pair[0], pair[1]);
                    }
                }
            }
        }
    }
}

/// Marks the raster nodes inside the triangles.
fn rasterize<'a>(
    triangles: impl Iterator<Item = &'a [Vec3; 3]>,
    (x0, y0): (f64, f64),
    nx: usize,
    ny: usize,
) -> Vec<bool> {
    let mut inside = vec![false; nx * ny];
    for &[a, b, c] in triangles {
        let lo = |v: f64, o: f64| ((v - o) / CELL_M).floor().max(0.0) as usize;
        let (i0, i1) = (
            lo(a.x.min(b.x).min(c.x), x0),
            (lo(a.x.max(b.x).max(c.x), x0) + 2).min(nx),
        );
        let (j0, j1) = (
            lo(a.y.min(b.y).min(c.y), y0),
            (lo(a.y.max(b.y).max(c.y), y0) + 2).min(ny),
        );
        let edge =
            |p: Vec3, q: Vec3, x: f64, y: f64| (q.x - p.x) * (y - p.y) - (q.y - p.y) * (x - p.x);
        for j in j0..j1 {
            for i in i0..i1 {
                let (x, y) = (x0 + i as f64 * CELL_M, y0 + j as f64 * CELL_M);
                let (u, v, w) = (edge(a, b, x, y), edge(b, c, x, y), edge(c, a, x, y));
                if (u >= 0.0 && v >= 0.0 && w >= 0.0) || (u <= 0.0 && v <= 0.0 && w <= 0.0) {
                    inside[j * nx + i] = true;
                }
            }
        }
    }
    inside
}

/// The outline edges of a polygon's triangles, each with the outward
/// normal (ENU, unit).
fn outline(triangles: &[[Vec3; 3]]) -> Vec<(Point, Point, Point)> {
    let key = |v: Vec3| ((v.x * 1000.0).round() as i64, (v.y * 1000.0).round() as i64);
    let mut count = std::collections::HashMap::new();
    for t in triangles {
        for k in 0..3 {
            let (a, b) = (key(t[k]), key(t[(k + 1) % 3]));
            *count
                .entry(if a < b { (a, b) } else { (b, a) })
                .or_insert(0) += 1;
        }
    }
    let mut edges = Vec::new();
    for t in triangles {
        for k in 0..3 {
            let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            let (ka, kb) = (key(a), key(b));
            if count[&if ka < kb { (ka, kb) } else { (kb, ka) }] != 1 {
                continue;
            }
            let (dx, dy) = (b.x - a.x, b.y - a.y);
            let length = dx.hypot(dy);
            if length < 1.0e-3 {
                continue;
            }
            let mut n = (-dy / length, dx / length);
            if n.0 * (c.x - a.x) + n.1 * (c.y - a.y) > 0.0 {
                n = (-n.0, -n.1);
            }
            edges.push(((a.x, a.y), (b.x, b.y), n));
        }
    }
    edges
}

/// One marching-squares cell.
struct Cell {
    /// The positive part, counterclockwise (empty when all or none is).
    polygon: Vec<(f64, f64)>,
    /// The contour's crossings of the cell edges, in pairs.
    crossings: Vec<(f64, f64)>,
    /// Whether the whole cell is positive.
    full: bool,
}

/// Cells per side of the coarse ground blocks without roads.
const BLOCK: usize = 8;

/// The render items of the streets: the carriageways (from `roads`, the
/// road triangles draped over the terrain), the sidewalks and curbs, the
/// paved ground outside the roads, edge lines, tactile paving and the
/// crossings at the intersections.
pub(crate) fn street_items(
    city: &PlateauCity,
    streets: &Streets,
    roads: SolidMesh,
) -> Vec<RenderSceneItem> {
    let z = |x: f64, y: f64, lift: f64| p(x, y, city.ground_m(x, y) + lift);
    let side_lift = ROAD_LIFT_M + CURB_M;

    // Sidewalks: positive in the outer band of wide roads and outside the
    // roads, negative on carriageways.
    let mut sidewalk = SolidMesh::default();
    let mut curb_face = SolidMesh::default();
    let mut curb_top = SolidMesh::default();
    let sidewalk_field = |road: bool, d: f64, wide: bool, _: f64| {
        Some(match (road, wide) {
            (false, _) => 0.5,
            (true, true) => SIDEWALK_M - d,
            (true, false) => -1.0,
        })
    };
    let mut curb = |a: (f64, f64), b: (f64, f64)| {
        // The curb faces the carriageway: find the sidewalk's side.
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let length = dx.hypot(dy);
        if length < 1.0e-6 {
            return;
        }
        let mut n = (-dy / length, dx / length);
        let mid = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
        let probe = |s: f64| {
            streets
                .nearest(mid.0 + n.0 * s, mid.1 + n.1 * s)
                .map_or(0.0, |(i, j)| {
                    let (r, d, w, f) = streets.at(i, j);
                    sidewalk_field(r, d, w, f).unwrap_or(0.0)
                })
        };
        if probe(0.4) < probe(-0.4) {
            n = (-n.0, -n.1);
        }
        let top = |q: (f64, f64)| z(q.0, q.1, side_lift + 0.01);
        let bottom = |q: (f64, f64)| z(q.0, q.1, ROAD_LIFT_M - 0.02);
        let normal = p(-n.0, -n.1, 0.0);
        curb_face.face([bottom(a), bottom(b), top(b), top(a)], normal);
        curb_face.face([bottom(a), top(a), top(b), bottom(b)], -normal);
        let inner = |q: (f64, f64)| (q.0 + n.0 * 0.18, q.1 + n.1 * 0.18);
        flat_quad(&mut curb_top, [a, b, inner(b), inner(a)], &top);
    };
    let paved = |corners: [(f64, f64); 3], mesh: &mut SolidMesh| {
        flat(
            mesh,
            corners.map(|(x, y)| z(x, y, side_lift)),
            PAVING_TILE_M,
        );
    };
    let mut ground = SolidMesh::default();
    let (bx, by) = ((streets.nx - 1) / BLOCK, (streets.ny - 1) / BLOCK);
    for bj in 0..by {
        for bi in 0..bx {
            let (i0, j0) = (bi * BLOCK, bj * BLOCK);
            let any_road = (j0..=j0 + BLOCK)
                .any(|j| (i0..=i0 + BLOCK).any(|i| streets.road[j * streets.nx + i]));
            if !any_road {
                // Paved ground outside the roads, in one piece.
                let c = [
                    (i0, j0),
                    (i0 + BLOCK, j0),
                    (i0 + BLOCK, j0 + BLOCK),
                    (i0, j0 + BLOCK),
                ]
                .map(|(i, j)| streets.node(i, j));
                flat_ground(&mut ground, c, z, side_lift);
                continue;
            }
            for j in j0..j0 + BLOCK {
                // Runs of whole cells become one quad.
                let mut run: Option<usize> = None;
                for i in i0..=i0 + BLOCK {
                    let cell = (i < i0 + BLOCK)
                        .then(|| streets.cell(i, j, &sidewalk_field))
                        .flatten();
                    let full = cell.as_ref().is_some_and(|c| c.full);
                    if full && run.is_none() {
                        run = Some(i);
                    }
                    if !full {
                        if let Some(start) = run.take() {
                            let c = [(start, j), (i, j), (i, j + 1), (start, j + 1)]
                                .map(|(i, j)| streets.node(i, j));
                            flat_ground(&mut sidewalk, c, z, side_lift);
                        }
                    }
                    let Some(cell) = cell else { continue };
                    for k in 1..cell.polygon.len().saturating_sub(1) {
                        paved(
                            [cell.polygon[0], cell.polygon[k], cell.polygon[k + 1]],
                            &mut sidewalk,
                        );
                    }
                    for pair in cell.crossings.chunks_exact(2) {
                        curb(pair[0], pair[1]);
                    }
                }
            }
        }
    }

    let [lines, tactile, zebra] = markings(city, streets);

    let mut items = Vec::new();
    let mut textured = |mesh: SolidMesh, texture: Arc<ImageFrame>, roughness: f32| {
        if !mesh.positions.is_empty() {
            let mut item = mesh.item([1.0; 4], roughness, 0.0, [0.0; 3]);
            item.base_color_texture = Some(texture);
            items.push(item);
        }
    };
    // A synthetic area's ground outside the road is grass, not paving.
    let tiles = Arc::new(if city.scenery.is_some() {
        cover_texture(Cover::Grass)
    } else {
        paving(512, 16, [168, 163, 155], 3)
    });
    textured(ground, tiles.clone(), 0.9);
    textured(roads, Arc::new(asphalt(512)), 0.85);
    textured(sidewalk, tiles, 0.9);
    for (mesh, color, roughness) in [
        (curb_face, linear([196, 194, 188]), 0.85),
        (curb_top, linear([212, 210, 204]), 0.8),
        (lines, linear([232, 232, 226]), 0.6),
        (zebra, linear([236, 236, 230]), 0.6),
        (tactile, linear([228, 182, 40]), 0.7),
    ] {
        if !mesh.positions.is_empty() {
            items.push(mesh.item(color, roughness, 0.0, [0.0; 3]));
        }
    }
    items
}

/// The painted lines, the tactile paving and the zebra crossings.
fn markings(city: &PlateauCity, streets: &Streets) -> [SolidMesh; 3] {
    let z = |x: f64, y: f64, lift: f64| p(x, y, city.ground_m(x, y) + lift);
    let side_lift = ROAD_LIFT_M + CURB_M;
    // Edge lines 0.4 m beside the curbs of wide roads and 0.5 m inside the
    // building line of narrow ones, and a line of tactile paving 1.6 m in
    // from the building line along the sidewalks.
    let mut lines = SolidMesh::default();
    streets.contour(
        |_, d, wide, junction_m| {
            (junction_m > 5.5).then_some(if wide {
                d - (SIDEWALK_M + 0.4)
            } else {
                d - 0.5
            })
        },
        |a, b| strip(&mut lines, a, b, 0.15, |q| z(q.0, q.1, ROAD_LIFT_M + 0.04)),
    );
    let mut tactile = SolidMesh::default();
    streets.contour(
        |_, d, wide, _| (wide && d < SIDEWALK_M + 0.5).then_some(d - 1.6),
        |a, b| strip(&mut tactile, a, b, 0.3, |q| z(q.0, q.1, side_lift + 0.02)),
    );

    // Zebra crossings across every wide road where it enters an
    // intersection: 0.45 m bars along the traffic, 0.45 m apart, from 0.5 m
    // to 4.5 m outside the intersection, over the carriageway only.
    let mut zebra = SolidMesh::default();
    let carriageway = |q: (f64, f64)| {
        streets.nearest(q.0, q.1).is_some_and(|(i, j)| {
            let (road, d, wide, junction_m) = streets.at(i, j);
            road && wide && d > SIDEWALK_M + 0.5 && junction_m > 0.0
        })
    };
    for junction in city.junctions.iter() {
        for (a, b, n) in outline(junction) {
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let length = dx.hypot(dy);
            let t = (dx / length, dy / length);
            let mut s = 0.225;
            while s < length {
                let c = (a.0 + t.0 * s, a.1 + t.1 * s);
                let at = |k: f64| (c.0 + n.0 * k, c.1 + n.1 * k);
                if carriageway(at(0.5)) && carriageway(at(2.5)) && carriageway(at(4.5)) {
                    strip(&mut zebra, at(0.5), at(4.5), 0.45, |q| {
                        z(q.0, q.1, ROAD_LIFT_M + 0.05)
                    });
                }
                s += 0.9;
            }
        }
    }

    [lines, tactile, zebra]
}

/// A flat quad of paving at `lift` above the terrain.
fn flat_ground(
    mesh: &mut SolidMesh,
    c: [(f64, f64); 4],
    z: impl Fn(f64, f64, f64) -> Vec3,
    lift: f64,
) {
    let v = c.map(|(x, y)| z(x, y, lift));
    flat(mesh, [v[0], v[1], v[2]], PAVING_TILE_M);
    flat(mesh, [v[0], v[2], v[3]], PAVING_TILE_M);
}

/// World texture coordinates for a ground point.
fn ground_uv(v: Vec3, tile_m: f64) -> [f32; 2] {
    // Renderer frame: x east, z south.
    [(v.x / tile_m) as f32, (-v.z / tile_m) as f32]
}

/// An upward-facing triangle on the ground with world texture coordinates.
pub(crate) fn flat(mesh: &mut SolidMesh, corners: [Vec3; 3], tile_m: f64) {
    let normal = (corners[1] - corners[0]).cross(corners[2] - corners[0]);
    let (corners, normal) = if normal.y < 0.0 {
        ([corners[0], corners[2], corners[1]], -normal)
    } else {
        (corners, normal)
    };
    mesh.textured(
        corners,
        normal.normalize_or_zero(),
        corners.map(|v| ground_uv(v, tile_m)),
    );
}

fn flat_quad(mesh: &mut SolidMesh, q: [(f64, f64); 4], lift: &impl Fn((f64, f64)) -> Vec3) {
    let v = q.map(lift);
    flat(mesh, [v[0], v[1], v[2]], 1.0);
    flat(mesh, [v[0], v[2], v[3]], 1.0);
}

/// A painted strip of `width` along a segment.
fn strip(
    mesh: &mut SolidMesh,
    a: (f64, f64),
    b: (f64, f64),
    width: f64,
    lift: impl Fn((f64, f64)) -> Vec3,
) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let length = dx.hypot(dy);
    if length < 1.0e-6 {
        return;
    }
    // Overlap the neighbouring segments a little so corners stay closed,
    // and follow the terrain in pieces of at most 1 m.
    let (tx, ty) = (dx / length * 0.03, dy / length * 0.03);
    let (nx, ny) = (-dy / length * width * 0.5, dx / length * width * 0.5);
    let (a, b) = ((a.0 - tx, a.1 - ty), (b.0 + tx, b.1 + ty));
    let pieces = length.ceil().max(1.0) as usize;
    let at = |k: usize| {
        let t = k as f64 / pieces as f64;
        (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1))
    };
    for k in 0..pieces {
        let (p, q) = (at(k), at(k + 1));
        flat_quad(
            mesh,
            [
                (p.0 - nx, p.1 - ny),
                (q.0 - nx, q.1 - ny),
                (q.0 + nx, q.1 + ny),
                (p.0 + nx, p.1 + ny),
            ],
            &lift,
        );
    }
}

fn dilate(mask: &[bool], nx: usize, ny: usize) -> Vec<bool> {
    let mut out = mask.to_vec();
    for j in 0..ny {
        for i in 0..nx {
            if mask[j * nx + i] {
                continue;
            }
            out[j * nx + i] = [(-1, 0), (1, 0), (0, -1), (0, 1)].iter().any(|&(di, dj)| {
                let (a, b) = (i as i64 + di, j as i64 + dj);
                a >= 0
                    && b >= 0
                    && (a as usize) < nx
                    && (b as usize) < ny
                    && mask[b as usize * nx + a as usize]
            });
        }
    }
    out
}

fn erode(mask: &[bool], nx: usize, ny: usize) -> Vec<bool> {
    let inverted: Vec<bool> = mask.iter().map(|&m| !m).collect();
    dilate(&inverted, nx, ny).iter().map(|&m| !m).collect()
}

/// Euclidean distance of every road node from the nearest point outside
/// the roads (half a cell nearer than the nearest node outside), meters;
/// zero outside.
fn distance_outside(road: &[bool], nx: usize, ny: usize) -> Vec<f32> {
    const FAR: f32 = 1.0e9;
    let mut grid: Vec<f32> = road.iter().map(|&r| if r { FAR } else { 0.0 }).collect();
    let mut line = vec![0.0_f32; nx.max(ny)];
    let mut out = vec![0.0_f32; nx.max(ny)];
    for j in 0..ny {
        line[..nx].copy_from_slice(&grid[j * nx..(j + 1) * nx]);
        squared_distance_1d(&line[..nx], &mut out[..nx]);
        grid[j * nx..(j + 1) * nx].copy_from_slice(&out[..nx]);
    }
    for i in 0..nx {
        for j in 0..ny {
            line[j] = grid[j * nx + i];
        }
        squared_distance_1d(&line[..ny], &mut out[..ny]);
        for j in 0..ny {
            grid[j * nx + i] = out[j];
        }
    }
    grid.iter()
        .map(|&s| ((s.sqrt() - 0.5) * CELL_M as f32).max(0.0))
        .collect()
}

/// Felzenszwalb and Huttenlocher's lower envelope of parabolas: `out[q]`
/// is the minimum over p of `(q - p)^2 + f[p]`.
fn squared_distance_1d(f: &[f32], out: &mut [f32]) {
    let n = f.len();
    let f: Vec<f64> = f.iter().map(|&v| f64::from(v)).collect();
    let mut v = vec![0_usize; n];
    let mut z = vec![0.0_f64; n + 1];
    let mut k = 0;
    z[0] = f64::NEG_INFINITY;
    z[1] = f64::INFINITY;
    let meet = |q: usize, p: usize| {
        ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64)) / (2.0 * (q as f64 - p as f64))
    };
    for q in 1..n {
        let mut s = meet(q, v[k]);
        while s <= z[k] {
            k -= 1;
            s = meet(q, v[k]);
        }
        k += 1;
        v[k] = q;
        z[k] = s;
        z[k + 1] = f64::INFINITY;
    }
    k = 0;
    for (q, o) in out.iter_mut().enumerate() {
        while z[k + 1] < q as f64 {
            k += 1;
        }
        let d = q as f64 - v[k] as f64;
        *o = (d * d + f[v[k]]) as f32;
    }
}

/// The maximum over a square of `reach` nodes either side.
fn max_filter(values: &[f32], nx: usize, ny: usize, reach: usize) -> Vec<f32> {
    let run = |get: &dyn Fn(usize) -> f32, n: usize, put: &mut dyn FnMut(usize, f32)| {
        let mut window: VecDeque<usize> = VecDeque::new();
        let mut next = 0;
        for q in 0..n {
            while next < n && next <= q + reach {
                while window.back().is_some_and(|&b| get(b) <= get(next)) {
                    window.pop_back();
                }
                window.push_back(next);
                next += 1;
            }
            while window.front().is_some_and(|&f| f + reach < q) {
                window.pop_front();
            }
            put(q, window.front().map_or(0.0, |&f| get(f)));
        }
    };
    let mut rows = vec![0.0_f32; nx * ny];
    for j in 0..ny {
        run(&|i| values[j * nx + i], nx, &mut |i, v| {
            rows[j * nx + i] = v
        });
    }
    let mut out = vec![0.0_f32; nx * ny];
    for i in 0..nx {
        run(&|j| rows[j * nx + i], ny, &mut |j, v| out[j * nx + i] = v);
    }
    out
}

/// A deterministic hash of a lattice point to [0, 1).
fn hash(x: i64, y: i64, seed: u64) -> f64 {
    let mut h = (x as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
        ^ (y as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F)
        ^ seed.wrapping_mul(0x1656_67B1_9E37_79F9);
    h ^= h >> 31;
    h = h.wrapping_mul(0xBF58_476D_1CE4_E5B9);
    h ^= h >> 29;
    (h >> 11) as f64 / (1_u64 << 53) as f64
}

/// Value noise with `period` lattice cells across a tile that repeats.
fn noise(u: f64, v: f64, period: i64, seed: u64) -> f64 {
    let (x, y) = (u * period as f64, v * period as f64);
    let (i, j) = (x.floor() as i64, y.floor() as i64);
    let (fx, fy) = (x - i as f64, y - j as f64);
    let s = |t: f64| t * t * (3.0 - 2.0 * t);
    let h = |a: i64, b: i64| hash(a.rem_euclid(period), b.rem_euclid(period), seed);
    let (sx, sy) = (s(fx), s(fy));
    let top = h(i, j) * (1.0 - sx) + h(i + 1, j) * sx;
    let bottom = h(i, j + 1) * (1.0 - sx) + h(i + 1, j + 1) * sx;
    top * (1.0 - sy) + bottom * sy
}

fn to_srgb(rgb: [f64; 3]) -> [u8; 4] {
    let c = rgb.map(|v| v.round().clamp(0.0, 255.0) as u8);
    [c[0], c[1], c[2], 255]
}

/// Metres one tile of a ground cover's texture spans.
fn cover_tile_m(cover: Cover) -> f64 {
    match cover {
        Cover::Grass | Cover::Dirt => 6.0,
        Cover::Rows => 7.2,
        Cover::Paddy | Cover::Soil | Cover::Concrete => 4.8,
    }
}

/// The texture of a ground cover, 512 px over [`cover_tile_m`]; rows run
/// east-west.
fn cover_texture(cover: Cover) -> ImageFrame {
    let size = 512_u32;
    if cover == Cover::Concrete {
        return paving(size, 4, [178, 175, 168], 7);
    }
    let mut rgba = Vec::with_capacity((size * size * 4) as usize);
    for y in 0..size {
        for x in 0..size {
            let (u, v) = (
                f64::from(x) / f64::from(size),
                f64::from(y) / f64::from(size),
            );
            let broad = noise(u, v, 3, 11) * 0.6 + noise(u, v, 11, 12) * 0.4;
            let grain = hash(i64::from(x), i64::from(y), 13);
            let rgb = match cover {
                Cover::Grass => {
                    let t = 0.85 + (broad - 0.5) * 0.3 + (grain - 0.5) * 0.12;
                    [84.0 * t, 116.0 * t, 56.0 * t]
                }
                Cover::Paddy => {
                    // Young rice in rows 0.3 m apart over muddy water.
                    let row = (v * 16.0).fract();
                    let hill = (u * 24.0).fract();
                    let plant = (row - 0.5).abs() < 0.16 && (hill - 0.5).abs() < 0.3;
                    let t = 0.9 + (broad - 0.5) * 0.2 + (grain - 0.5) * 0.08;
                    if plant {
                        [96.0 * t, 138.0 * t, 58.0 * t]
                    } else {
                        [92.0 * t, 104.0 * t, 96.0 * t]
                    }
                }
                Cover::Rows => {
                    // Ridges of leafy vegetables 0.9 m apart on dark soil.
                    let row = (v * 8.0).fract();
                    let leaf = (row - 0.5).abs() < 0.22 && grain > 0.25;
                    let t = 0.9 + (broad - 0.5) * 0.25 + (grain - 0.5) * 0.1;
                    if leaf {
                        [70.0 * t, 118.0 * t, 52.0 * t]
                    } else {
                        [96.0 * t, 76.0 * t, 58.0 * t]
                    }
                }
                Cover::Soil => {
                    let furrow = ((v * 32.0).fract() - 0.5).abs() * 0.12;
                    let t = 0.88 + furrow + (broad - 0.5) * 0.2 + (grain - 0.5) * 0.1;
                    [118.0 * t, 92.0 * t, 68.0 * t]
                }
                Cover::Dirt => {
                    let t = 0.9 + (broad - 0.5) * 0.2 + (grain - 0.5) * 0.12;
                    [150.0 * t, 130.0 * t, 100.0 * t]
                }
                Cover::Concrete => unreachable!(),
            };
            rgba.extend(to_srgb(rgb));
        }
    }
    ImageFrame::from_rgba8(size, size, rgba)
}

/// A synthetic area's ground covers (fields, drives) just above the grass.
pub(crate) fn cover_items(city: &PlateauCity, scenery: &Scenery) -> Vec<RenderSceneItem> {
    let lift = ROAD_LIFT_M + CURB_M + 0.02;
    let mut items = Vec::new();
    for cover in [
        Cover::Grass,
        Cover::Paddy,
        Cover::Rows,
        Cover::Soil,
        Cover::Concrete,
        Cover::Dirt,
    ] {
        let mut mesh = SolidMesh::default();
        let tile = cover_tile_m(cover);
        // Paths sit a little above the fields they cross.
        let lift = if cover == Cover::Dirt {
            lift + 0.01
        } else {
            lift
        };
        for (_, quad) in scenery.ground.iter().filter(|(c, _)| *c == cover) {
            let v = quad.map(|(x, y)| p(x, y, city.ground_m(x, y) + lift));
            flat(&mut mesh, [v[0], v[1], v[2]], tile);
            flat(&mut mesh, [v[0], v[2], v[3]], tile);
        }
        if !mesh.positions.is_empty() {
            let roughness = if cover == Cover::Paddy { 0.35 } else { 0.9 };
            let mut item = mesh.item([1.0; 4], roughness, 0.0, [0.0; 3]);
            item.base_color_texture = Some(Arc::new(cover_texture(cover)));
            items.push(item);
        }
    }
    items
}

/// Worn asphalt: aggregate grain, darker and lighter patches and stains.
fn asphalt(size: u32) -> ImageFrame {
    let mut rgba = Vec::with_capacity((size * size * 4) as usize);
    for y in 0..size {
        for x in 0..size {
            let (u, v) = (
                f64::from(x) / f64::from(size),
                f64::from(y) / f64::from(size),
            );
            let broad =
                noise(u, v, 4, 1) * 0.5 + noise(u, v, 9, 2) * 0.3 + noise(u, v, 23, 3) * 0.2;
            let grain = hash(i64::from(x), i64::from(y), 4);
            let stone = if hash(i64::from(x), i64::from(y), 5) > 0.98 {
                8.0
            } else {
                0.0
            };
            let level = 70.0 + (broad - 0.5) * 26.0 + (grain - 0.5) * 6.0 + stone;
            rgba.extend(to_srgb([level, level, level + 3.0]));
        }
    }
    ImageFrame::from_rgba8(size, size, rgba)
}

/// Square paving blocks, `blocks` across the tile, each a slightly
/// different tone, with darker joints.
fn paving(size: u32, blocks: u32, base: [u8; 3], seed: u64) -> ImageFrame {
    let cell = size / blocks;
    let mut rgba = Vec::with_capacity((size * size * 4) as usize);
    for y in 0..size {
        for x in 0..size {
            let (bx, by) = (i64::from(x / cell), i64::from(y / cell));
            let (u, v) = (
                f64::from(x) / f64::from(size),
                f64::from(y) / f64::from(size),
            );
            let joint = x % cell < 2 || y % cell < 2;
            // Low contrast within the tile: without mipmaps, fine detail
            // shimmers in the distance.
            let tone = 1.0
                + (hash(bx, by, seed) - 0.5) * 0.08
                + (noise(u, v, 5, seed + 1) - 0.5) * 0.12
                + (hash(i64::from(x), i64::from(y), seed + 2) - 0.5) * 0.02;
            let tone = if joint { tone * 0.9 } else { tone };
            rgba.extend(to_srgb(base.map(|c| f64::from(c) * tone)));
        }
    }
    ImageFrame::from_rgba8(size, size, rgba)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn distance_transform_is_euclidean() {
        // A 9 x 9 road with one node outside it at its centre.
        let (nx, ny) = (9, 9);
        let mut road = vec![true; nx * ny];
        road[4 * nx + 4] = false;
        let d = distance_outside(&road, nx, ny);
        let expect = |i: usize, j: usize| {
            let r = ((i as f64 - 4.0).powi(2) + (j as f64 - 4.0).powi(2)).sqrt();
            ((r - 0.5) * CELL_M).max(0.0)
        };
        for j in 0..ny {
            for i in 0..nx {
                assert!((f64::from(d[j * nx + i]) - expect(i, j)).abs() < 1.0e-5);
            }
        }
    }

    #[test]
    fn outline_skips_shared_edges_and_points_outward() {
        // A unit square from two triangles: four outline edges.
        let v = |x: f64, y: f64| Vec3::new(x, y, 0.0);
        let square = [
            [v(0.0, 0.0), v(1.0, 0.0), v(1.0, 1.0)],
            [v(0.0, 0.0), v(1.0, 1.0), v(0.0, 1.0)],
        ];
        let edges = outline(&square);
        assert_eq!(edges.len(), 4);
        for (a, b, n) in edges {
            let mid = ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5);
            // Outward: away from the centre.
            assert!(n.0 * (mid.0 - 0.5) + n.1 * (mid.1 - 0.5) > 0.0);
        }
    }

    #[test]
    fn max_filter_spreads_a_peak() {
        let (nx, ny) = (7, 5);
        let mut values = vec![0.0; nx * ny];
        values[2 * nx + 3] = 1.0;
        let out = max_filter(&values, nx, ny, 1);
        for j in 0..ny {
            for i in 0..nx {
                let inside = (2..=4).contains(&i) && (1..=3).contains(&j);
                assert_eq!(out[j * nx + i], if inside { 1.0 } else { 0.0 });
            }
        }
    }
}
