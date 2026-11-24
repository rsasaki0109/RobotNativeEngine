//! A synthetic suburb (`--area suburb`): one straight road east-west through
//! the origin, a few hundred metres of houses at each end and rice paddies
//! and vegetable fields between them.
//!
//! The houses are two-storey, gabled, with eaves, balconies, window frames
//! and air-conditioner units, behind block walls or hedges, a car in the
//! drive and a tree in the garden; side lanes cross the rows, with curve
//! mirrors at their corners; concrete utility poles carry wires, a
//! transformer or a street light; vending machines stand by some walls.
//! Woods, farmhouses and greenhouses stand only beyond the 3D LiDAR's reach
//! from the open stretch, and hills close the horizon. Every object is in
//! the sensor mesh; windows, doors, wires, the hills and the ground covers
//! are drawn only.

use crate::plateau::{Building, Cover, Finish, PlateauCity, Scenery};
use rne_gnss::{MeshTriangle, SurfaceMaterial};
use rne_math::Vec3;

const ROAD_HALF_WIDTH_M: f64 = 4.0;
/// Side lanes' half width, meters.
const LANE_HALF_WIDTH_M: f64 = 2.3;
/// The lots' front line, from the road's centre line, meters.
const LOT_FRONT_M: f64 = 5.0;
/// Nearest the open stretch's woods, farmhouses and greenhouses come to
/// the road, meters: beyond the 3D LiDAR's 60 m from the route.
const FAR_M: f64 = 72.0;
/// Utility poles' line, from the road's centre line, meters.
const POLE_LINE_M: f64 = 4.6;

const WALLS: [[u8; 3]; 7] = [
    [238, 234, 224],
    [228, 216, 194],
    [200, 182, 154],
    [150, 120, 94],
    [206, 206, 200],
    [176, 188, 194],
    [118, 116, 112],
];
const ROOFS: [[u8; 3]; 5] = [
    [64, 68, 76],
    [56, 70, 100],
    [128, 76, 56],
    [44, 44, 46],
    [98, 104, 94],
];
const CARS: [[u8; 3]; 6] = [
    [240, 240, 238],
    [190, 192, 196],
    [30, 30, 34],
    [160, 30, 34],
    [40, 70, 130],
    [200, 190, 160],
];
const GREENS: [[u8; 3]; 4] = [[58, 96, 48], [72, 110, 52], [46, 82, 44], [86, 116, 60]];
const VENDING: [[u8; 3]; 3] = [[200, 40, 40], [236, 236, 236], [40, 90, 170]];
const GLASS: [u8; 3] = [58, 70, 84];
const FRAME: [u8; 3] = [236, 236, 232];
const DOOR: [u8; 3] = [110, 84, 60];
const BLOCK_WALL: [u8; 3] = [176, 174, 166];
const CONCRETE: [u8; 3] = [168, 166, 160];
const TRUNK: [u8; 3] = [92, 72, 54];
const WIRE: [u8; 3] = [30, 30, 30];
const GREENHOUSE: [u8; 3] = [226, 232, 230];
const HILLS: [u8; 3] = [58, 84, 60];
const MIRROR_POST: [u8; 3] = [236, 120, 30];

/// A fixed irregular sequence.
struct Draws(u64);

impl Draws {
    fn uniform(&mut self, lo: f64, hi: f64) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        let unit = (self.0 >> 11) as f64 / (1_u64 << 53) as f64;
        lo + (hi - lo) * unit
    }

    fn chance(&mut self, p: f64) -> bool {
        self.uniform(0.0, 1.0) < p
    }

    fn pick<T: Copy>(&mut self, items: &[T]) -> T {
        items[(self.uniform(0.0, items.len() as f64) as usize).min(items.len() - 1)]
    }
}

/// Faces of a convex solid as polygons, triangulated as fans and wound
/// outward from the centroid of their corners.
fn convex(faces: &[Vec<Vec3>]) -> Vec<[Vec3; 3]> {
    let corners: Vec<Vec3> = faces.iter().flatten().copied().collect();
    let centre = corners.iter().fold(Vec3::ZERO, |a, &v| a + v) / corners.len() as f64;
    faces
        .iter()
        .flat_map(|face| {
            (1..face.len() - 1).map(move |k| {
                let [a, b, c] = [face[0], face[k], face[k + 1]];
                if (b - a).cross(c - a).dot(a - centre) < 0.0 {
                    [a, c, b]
                } else {
                    [a, b, c]
                }
            })
        })
        .collect()
}

/// The faces of a hexahedron from its bottom and top quads (each in order
/// around).
fn hexahedron(bottom: [Vec3; 4], top: [Vec3; 4]) -> Vec<Vec<Vec3>> {
    let mut faces = vec![bottom.to_vec(), top.to_vec()];
    for k in 0..4 {
        let l = (k + 1) % 4;
        faces.push(vec![bottom[k], bottom[l], top[l], top[k]]);
    }
    faces
}

/// An axis-aligned box.
fn cuboid(lo: Vec3, hi: Vec3) -> Vec<[Vec3; 3]> {
    let q = |z: f64| {
        [
            Vec3::new(lo.x, lo.y, z),
            Vec3::new(hi.x, lo.y, z),
            Vec3::new(hi.x, hi.y, z),
            Vec3::new(lo.x, hi.y, z),
        ]
    };
    convex(&hexahedron(q(lo.z), q(hi.z)))
}

/// A thin square beam from `a` to `b`.
fn beam(a: Vec3, b: Vec3, width: f64) -> Vec<[Vec3; 3]> {
    let along = (b - a).normalize();
    let side = if along.z.abs() > 0.9 {
        Vec3::X
    } else {
        Vec3::Z
    };
    let u = along.cross(side).normalize() * (width * 0.5);
    let v = along.cross(u).normalize() * (width * 0.5);
    let ring = |c: Vec3| [c + u + v, c - u + v, c - u - v, c + u - v];
    convex(&hexahedron(ring(a), ring(b)))
}

/// An upright `n`-sided prism (a cylinder) on `(x, y)`.
fn prism(x: f64, y: f64, r: f64, z0: f64, z1: f64, n: usize) -> Vec<[Vec3; 3]> {
    let ring = |z: f64| -> Vec<Vec3> {
        (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                Vec3::new(x + r * a.cos(), y + r * a.sin(), z)
            })
            .collect()
    };
    let (bottom, top) = (ring(z0), ring(z1));
    let mut faces = vec![bottom.clone(), top.clone()];
    for k in 0..n {
        let l = (k + 1) % n;
        faces.push(vec![bottom[k], bottom[l], top[l], top[k]]);
    }
    convex(&faces)
}

/// A quad drawn on a surface, wound to face `out`.
fn panel(corners: [Vec3; 4], out: Vec3) -> Vec<[Vec3; 3]> {
    let [a, b, c, d] = corners;
    if (b - a).cross(c - a).dot(out) < 0.0 {
        vec![[a, c, b], [a, d, c]]
    } else {
        vec![[a, b, c], [a, c, d]]
    }
}

/// One part of an object: triangles, colour, roughness, finish, material.
type Part = (Vec<[Vec3; 3]>, [u8; 3], f32, Finish, SurfaceMaterial);

#[derive(Default)]
struct Builder {
    triangles: Vec<MeshTriangle>,
    buildings: Vec<Building>,
    scenery: Scenery,
}

impl Builder {
    /// Adds an object to the sensor mesh and the scenery.
    fn object(&mut self, parts: Vec<Part>) {
        let object = self.buildings.len() as u32;
        let (mut lo, mut hi) = (Vec3::splat(f64::INFINITY), Vec3::splat(f64::NEG_INFINITY));
        for (triangles, colour, roughness, finish, material) in parts {
            for t in &triangles {
                for v in t {
                    lo = lo.min(*v);
                    hi = hi.max(*v);
                }
                if let Some(tri) = MeshTriangle::new(*t, material, object) {
                    self.triangles.push(tri);
                }
            }
            self.paint(colour, roughness, finish, triangles);
        }
        self.buildings.push(Building {
            bounds: [lo.x, lo.y, hi.x, hi.y],
            z_range: (lo.z, hi.z),
            textured: true,
        });
    }

    /// Adds triangles to the scenery only.
    fn paint(
        &mut self,
        colour: [u8; 3],
        roughness: f32,
        finish: Finish,
        triangles: Vec<[Vec3; 3]>,
    ) {
        match self
            .scenery
            .solids
            .iter_mut()
            .find(|(c, r, f, _)| *c == colour && (*r - roughness).abs() < 1.0e-6 && *f == finish)
        {
            Some((_, _, _, list)) => list.extend(triangles),
            None => self
                .scenery
                .solids
                .push((colour, roughness, finish, triangles)),
        }
    }

    fn solid(
        &mut self,
        t: Vec<[Vec3; 3]>,
        colour: [u8; 3],
        roughness: f32,
        finish: Finish,
        material: SurfaceMaterial,
    ) {
        self.object(vec![(t, colour, roughness, finish, material)]);
    }

    fn cover(&mut self, cover: Cover, x: (f64, f64), y: (f64, f64)) {
        self.scenery
            .ground
            .push((cover, [(x.0, y.0), (x.1, y.0), (x.1, y.1), (x.0, y.1)]));
    }

    /// A house on the footprint, gabled along its longer side, with eaves,
    /// windows on every wall, a door, and (two storeys) a balcony on the
    /// side facing `front_y`.
    #[allow(clippy::too_many_arguments)]
    fn house(
        &mut self,
        x: (f64, f64),
        y: (f64, f64),
        eave: f64,
        ridge: f64,
        d: &mut Draws,
        front_y: f64,
    ) {
        let along_x = x.1 - x.0 >= y.1 - y.0;
        let (xm, ym) = ((x.0 + x.1) * 0.5, (y.0 + y.1) * 0.5);
        let p = Vec3::new;
        let base = [
            p(x.0, y.0, 0.0),
            p(x.1, y.0, 0.0),
            p(x.1, y.1, 0.0),
            p(x.0, y.1, 0.0),
        ];
        let [b00, b10, b11, b01] = base;
        let [e00, e10, e11, e01] = base.map(|v| p(v.x, v.y, eave));
        let (r0, r1) = if along_x {
            (p(x.0, ym, ridge), p(x.1, ym, ridge))
        } else {
            (p(xm, y.0, ridge), p(xm, y.1, ridge))
        };
        let body = if along_x {
            vec![
                vec![b00, b10, e10, e00],
                vec![b01, b11, e11, e01],
                vec![b00, b01, e01, r0, e00],
                vec![b10, b11, e11, r1, e10],
                vec![e00, e10, r1, r0],
                vec![e01, e11, r1, r0],
                vec![b00, b10, b11, b01],
            ]
        } else {
            vec![
                vec![b00, b01, e01, e00],
                vec![b10, b11, e11, e10],
                vec![b00, b10, e10, r0, e00],
                vec![b01, b11, e11, r1, e01],
                vec![e00, e01, r1, r0],
                vec![e10, e11, r1, r0],
                vec![b00, b10, b11, b01],
            ]
        };
        let wall_finish = if d.chance(0.6) {
            Finish::Siding
        } else {
            Finish::Stucco
        };
        let wall_colour = d.pick(&WALLS);
        // The sensor mesh gets the closed body and the roof slabs; the
        // render draws the walls, the gables and the slabs.
        let object = self.buildings.len() as u32;
        let walls = convex(&[&body[..4], &body[6..]].concat());
        for t in convex(&body) {
            if let Some(tri) = MeshTriangle::new(t, SurfaceMaterial::CONCRETE, object) {
                self.triangles.push(tri);
            }
        }
        // Roof slabs over the eaves (0.45 m) and the gables (0.3 m).
        let roof_colour = d.pick(&ROOFS);
        let (ridge_dir, eave_lines) = if along_x {
            (Vec3::X, [(e00, e10, -Vec3::Y), (e01, e11, Vec3::Y)])
        } else {
            (Vec3::Y, [(e00, e01, -Vec3::X), (e10, e11, Vec3::X)])
        };
        let mut slabs = Vec::new();
        for (a, b, out) in eave_lines {
            let drop = (ridge - eave) / if along_x { y.1 - y.0 } else { x.1 - x.0 } * 2.0 * 0.45;
            let lo = |v: Vec3, s: f64| v + out * 0.45 - Vec3::Z * drop + ridge_dir * s;
            let hi = |v: Vec3, s: f64| v + ridge_dir * s;
            let (ra, rb) = (
                p(
                    if along_x { a.x } else { xm },
                    if along_x { ym } else { a.y },
                    ridge,
                ),
                p(
                    if along_x { b.x } else { xm },
                    if along_x { ym } else { b.y },
                    ridge,
                ),
            );
            let top = [lo(a, -0.3), lo(b, 0.3), hi(rb, 0.3), hi(ra, -0.3)];
            let normal = (top[1] - top[0]).cross(top[3] - top[0]).normalize();
            let normal = if normal.z < 0.0 { -normal } else { normal };
            let under = top.map(|v| v - normal * 0.14);
            slabs.extend(convex(&hexahedron(under, top)));
        }
        for t in &slabs {
            if let Some(tri) = MeshTriangle::new(*t, SurfaceMaterial::CONCRETE, object) {
                self.triangles.push(tri);
            }
        }
        self.paint(wall_colour, 0.85, wall_finish, walls);
        self.paint(roof_colour, 0.7, Finish::Tiles, slabs);
        self.buildings.push(Building {
            bounds: [x.0 - 0.45, y.0 - 0.45, x.1 + 0.45, y.1 + 0.45],
            z_range: (0.0, ridge),
            textured: true,
        });

        self.openings(base, eave, d, front_y);
    }

    /// Windows with frames on the four walls of a house on `base` (a storey
    /// 2.8 m), a door and a balcony facing `front_y`, an air conditioner on
    /// a side wall.
    fn openings(&mut self, base: [Vec3; 4], eave: f64, d: &mut Draws, front_y: f64) {
        let p = Vec3::new;
        let [b00, b10, b11, b01] = base;
        let (xm, ym) = ((b00.x + b11.x) * 0.5, (b00.y + b11.y) * 0.5);
        let centre = p(xm, ym, 0.0);
        let (mut glass, mut frames, mut door) = (Vec::new(), Vec::new(), Vec::new());
        let mut balcony_done = false;
        let mut ac_done = false;
        for (a, b) in [(b00, b10), (b10, b11), (b11, b01), (b01, b00)] {
            let length = (b - a).length();
            let along = (b - a) / length;
            let mut out = p(along.y, -along.x, 0.0);
            if out.dot(a - centre) < 0.0 {
                out = -out;
            }
            let at = |u: f64, z: f64, off: f64| a + along * u + out * off + Vec3::Z * z;
            let rect = |u0: f64, u1: f64, z0: f64, z1: f64, off: f64| {
                panel(
                    [
                        at(u0, z0, off),
                        at(u1, z0, off),
                        at(u1, z1, off),
                        at(u0, z1, off),
                    ],
                    out,
                )
            };
            let facing = (a.y - b.y).abs() < 1.0e-6
                && (a.y - centre.y).signum() == (front_y - centre.y).signum();
            let mut u = d.uniform(1.0, 1.6);
            let mut first = true;
            while u + 1.4 < length - 0.8 {
                if facing && first {
                    frames.extend(rect(u - 0.08, u + 1.08, 0.0, 2.18, 0.02));
                    door.extend(rect(u, u + 1.0, 0.0, 2.1, 0.035));
                    first = false;
                } else {
                    frames.extend(rect(u - 0.08, u + 1.48, 0.82, 2.08, 0.02));
                    glass.extend(rect(u, u + 0.68, 0.9, 2.0, 0.035));
                    glass.extend(rect(u + 0.72, u + 1.4, 0.9, 2.0, 0.035));
                }
                if eave > 5.0 {
                    frames.extend(rect(u - 0.08, u + 1.48, 3.62, 4.88, 0.02));
                    glass.extend(rect(u, u + 0.68, 3.7, 4.8, 0.035));
                    glass.extend(rect(u + 0.72, u + 1.4, 3.7, 4.8, 0.035));
                }
                u += d.uniform(2.6, 3.6);
            }
            if facing && eave > 5.0 && !balcony_done && length > 6.0 {
                balcony_done = true;
                let (u0, u1) = (length * 0.35, length * 0.35 + 3.6);
                let floor = cuboid_on(at(u0, 2.85, 0.0), at(u1, 2.85, 1.0), 0.15);
                let rail = cuboid_on(at(u0, 3.0, 0.95), at(u1, 3.0, 1.0), 1.0);
                self.solid(
                    floor,
                    CONCRETE,
                    0.85,
                    Finish::Stucco,
                    SurfaceMaterial::CONCRETE,
                );
                self.solid(rail, FRAME, 0.5, Finish::Plain, SurfaceMaterial::CONCRETE);
            }
            if !facing && !ac_done && (a.x - b.x).abs() < 1.0e-6 && length > 5.0 {
                ac_done = true;
                let unit = cuboid_on(at(1.0, 0.0, 0.05), at(1.8, 0.0, 0.35), 0.6);
                self.solid(
                    unit,
                    [222, 222, 218],
                    0.6,
                    Finish::Plain,
                    SurfaceMaterial::CONCRETE,
                );
            }
        }
        self.paint(GLASS, 0.1, Finish::Plain, glass);
        self.paint(FRAME, 0.5, Finish::Plain, frames);
        self.paint(DOOR, 0.6, Finish::Plain, door);
    }

    /// A tree of a few lobes of radius about `r` on a trunk.
    fn tree(&mut self, x: f64, y: f64, r: f64, colour: [u8; 3], d: &mut Draws) {
        let trunk_top = 1.4 + r * 0.5;
        let trunk = prism(x, y, 0.14 + r * 0.03, 0.0, trunk_top + r * 0.5, 6);
        let mut parts: Vec<Part> =
            vec![(trunk, TRUNK, 0.9, Finish::Bark, SurfaceMaterial::ABSORBER)];
        let lobes = if r > 2.0 { 3 } else { 2 };
        for k in 0..lobes {
            let lr = r * d.uniform(0.6, 0.85);
            let (ox, oy) = if k == 0 {
                (0.0, 0.0)
            } else {
                let a = d.uniform(0.0, std::f64::consts::TAU);
                (a.cos() * r * 0.45, a.sin() * r * 0.45)
            };
            let mid = trunk_top + lr * 0.7 + if k == 0 { r * 0.3 } else { 0.0 };
            parts.push((
                lobe(x + ox, y + oy, mid, lr, d),
                colour,
                0.95,
                Finish::Leaves,
                SurfaceMaterial::ABSORBER,
            ));
        }
        self.object(parts);
    }

    /// A parked car, nose to the road, centred at `(x, y)`.
    fn car(&mut self, x: f64, y: f64, colour: [u8; 3]) {
        let (hw, hl) = (0.85, 2.2);
        let body = cuboid(
            Vec3::new(x - hw, y - hl, 0.25),
            Vec3::new(x + hw, y + hl, 0.95),
        );
        let cabin = cuboid(
            Vec3::new(x - hw + 0.1, y - hl * 0.45, 0.95),
            Vec3::new(x + hw - 0.1, y + hl * 0.5, 1.48),
        );
        let mut wheels = Vec::new();
        for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            wheels.extend(cuboid(
                Vec3::new(x + sx * hw - 0.12, y + sy * (hl - 0.7) - 0.32, 0.0),
                Vec3::new(x + sx * hw + 0.12, y + sy * (hl - 0.7) + 0.32, 0.62),
            ));
        }
        self.object(vec![
            (body, colour, 0.3, Finish::Plain, SurfaceMaterial::CONCRETE),
            (cabin, GLASS, 0.1, Finish::Plain, SurfaceMaterial::CONCRETE),
            (
                wheels,
                [26, 26, 28],
                0.8,
                Finish::Plain,
                SurfaceMaterial::CONCRETE,
            ),
        ]);
    }

    /// One lot on side `s` (+1 north, -1 south) from `x0` to `x1`, its front
    /// `front` metres from the centre line; `street` lots have a drive, a
    /// car and a wall or hedge along the front.
    fn lot(&mut self, x0: f64, x1: f64, s: f64, front: f64, street: bool, d: &mut Draws) {
        let y = |u: f64| s * u;
        let span = |u0: f64, u1: f64| (y(u0).min(y(u1)), y(u0).max(y(u1)));
        let drive = 3.2;
        let depth = d.uniform(7.5, 10.0);
        let back = front + 2.8;
        let house_x = (x0 + drive + 0.8, x1 - 1.2);
        if street {
            self.cover(Cover::Concrete, (x0, x0 + drive), span(front, front + 6.5));
            if d.chance(0.75) {
                let c = d.pick(&CARS);
                self.car(x0 + drive * 0.5, y(front + 3.4), c);
            }
            if d.chance(0.6) {
                let (y0, y1) = span(front + 0.1, front + 0.25);
                let t = cuboid(
                    Vec3::new(x0 + drive + 0.2, y0, 0.0),
                    Vec3::new(x1 - 0.3, y1, 1.2),
                );
                self.solid(
                    t,
                    BLOCK_WALL,
                    0.9,
                    Finish::Blocks,
                    SurfaceMaterial::CONCRETE,
                );
                if d.chance(0.12) {
                    // A vending machine against the wall, facing the road.
                    let (v0, v1) = span(front - 0.7, front);
                    let c = d.pick(&VENDING);
                    let t = cuboid(Vec3::new(x1 - 1.6, v0, 0.0), Vec3::new(x1 - 0.6, v1, 1.85));
                    self.solid(t, c, 0.4, Finish::Plain, SurfaceMaterial::CONCRETE);
                }
            } else {
                let (y0, y1) = span(front + 0.1, front + 0.8);
                let c = d.pick(&GREENS);
                let t = cuboid(
                    Vec3::new(x0 + drive + 0.2, y0, 0.0),
                    Vec3::new(x1 - 0.3, y1, 1.3),
                );
                self.solid(t, c, 0.95, Finish::Leaves, SurfaceMaterial::ABSORBER);
            }
        }
        let eave = if d.chance(0.85) {
            d.uniform(5.3, 6.1)
        } else {
            d.uniform(2.9, 3.3)
        };
        let ridge = eave + d.uniform(1.5, 2.5);
        self.house(house_x, span(back, back + depth), eave, ridge, d, y(front));
        if d.chance(0.5) {
            let r = d.uniform(1.0, 1.8);
            let c = d.pick(&GREENS);
            self.tree(x1 - 1.6, y(back + depth + 2.2), r, c, d);
        }
    }

    /// Two rows of lots on both sides from `x0` to `x1`, broken by the side
    /// lanes at `lanes`.
    fn town(&mut self, x0: f64, x1: f64, lanes: &[f64], d: &mut Draws) {
        for s in [-1.0, 1.0] {
            for (front, street) in [(LOT_FRONT_M, true), (LOT_FRONT_M + 18.0, false)] {
                let mut x = x0;
                loop {
                    let width = d.uniform(11.5, 15.0);
                    if x + width > x1 {
                        break;
                    }
                    let clear = LANE_HALF_WIDTH_M + 1.0;
                    if let Some(&lane) = lanes
                        .iter()
                        .find(|&&l| x < l + clear && x + width > l - clear)
                    {
                        x = lane + clear;
                        continue;
                    }
                    self.lot(x, x + width - 0.6, s, front, street, d);
                    x += width;
                }
            }
        }
        for &lane in lanes {
            // A curve mirror at the corner.
            for s in [-1.0, 1.0] {
                let (mx, my) = (
                    lane + LANE_HALF_WIDTH_M + 0.6,
                    s * (ROAD_HALF_WIDTH_M + 0.6),
                );
                let post = prism(mx, my, 0.05, 0.0, 2.6, 6);
                let face = prism(mx, my, 0.4, 2.5, 2.56, 10)
                    .into_iter()
                    .map(|t| t.map(|v| rotate_up(v, Vec3::new(mx, my, 2.53), -s)))
                    .collect();
                self.object(vec![
                    (
                        post,
                        MIRROR_POST,
                        0.6,
                        Finish::Plain,
                        SurfaceMaterial::CONCRETE,
                    ),
                    (
                        face,
                        MIRROR_POST,
                        0.3,
                        Finish::Plain,
                        SurfaceMaterial::CONCRETE,
                    ),
                ]);
            }
        }
    }

    /// Concrete utility poles along the north side from `x0` to `x1`, every
    /// 30–36 m, with crossarms, some with a transformer or a street light,
    /// and three wires between neighbours.
    fn poles(&mut self, x0: f64, x1: f64, lanes: &[f64], d: &mut Draws) {
        let mut tops = Vec::new();
        let mut x = x0 + d.uniform(2.0, 8.0);
        while x < x1 {
            if lanes
                .iter()
                .any(|l| (x - l).abs() < LANE_HALF_WIDTH_M + 1.0)
            {
                x += 4.0;
                continue;
            }
            let y = POLE_LINE_M;
            let pole = prism(x, y, 0.17, 0.0, 10.5, 8);
            let arm = cuboid(
                Vec3::new(x - 0.06, y - 0.8, 9.6),
                Vec3::new(x + 0.06, y + 0.8, 9.72),
            );
            let mut parts: Vec<Part> = vec![
                (
                    pole,
                    CONCRETE,
                    0.85,
                    Finish::Stucco,
                    SurfaceMaterial::CONCRETE,
                ),
                (
                    arm,
                    [90, 92, 96],
                    0.5,
                    Finish::Plain,
                    SurfaceMaterial::CONCRETE,
                ),
            ];
            if d.chance(0.3) {
                parts.push((
                    prism(x, y + 0.45, 0.28, 7.2, 8.4, 8),
                    [150, 154, 156],
                    0.5,
                    Finish::Plain,
                    SurfaceMaterial::CONCRETE,
                ));
            } else if d.chance(0.5) {
                let mut light = beam(Vec3::new(x, y, 6.2), Vec3::new(x, y - 1.6, 6.5), 0.08);
                light.extend(cuboid(
                    Vec3::new(x - 0.15, y - 1.9, 6.35),
                    Vec3::new(x + 0.15, y - 1.35, 6.5),
                ));
                parts.push((
                    light,
                    [200, 200, 196],
                    0.4,
                    Finish::Plain,
                    SurfaceMaterial::CONCRETE,
                ));
            }
            self.object(parts);
            tops.push(x);
            x += d.uniform(30.0, 36.0);
        }
        let mut wires = Vec::new();
        for pair in tops.windows(2) {
            for (dy, z) in [(-0.7, 9.72), (0.0, 9.72), (0.7, 9.72), (0.0, 8.0)] {
                let (a, b) = (pair[0], pair[1]);
                let n = 8;
                let at = |k: usize| {
                    let t = k as f64 / n as f64;
                    let sag = 4.0 * t * (1.0 - t) * 0.6;
                    Vec3::new(a + (b - a) * t, POLE_LINE_M + dy, z - sag)
                };
                for k in 0..n {
                    wires.extend(beam(at(k), at(k + 1), 0.025));
                }
            }
        }
        self.paint(WIRE, 0.6, Finish::Plain, wires);
    }

    /// Paddies and fields from `x0` to `x1` on both sides, from `near`
    /// metres off the centre line to `far`, with dirt paths between them.
    fn fields(&mut self, x0: f64, x1: f64, near: f64, far: f64, d: &mut Draws) {
        for s in [-1.0_f64, 1.0] {
            let mut x = x0;
            while x < x1 - 10.0 {
                let length = d.uniform(30.0, 55.0).min(x1 - x);
                let mut u = near;
                while u < far - 10.0 {
                    let depth = d.uniform(28.0, 50.0).min(far - u);
                    let cover = d.pick(&[
                        Cover::Paddy,
                        Cover::Paddy,
                        Cover::Rows,
                        Cover::Soil,
                        Cover::Grass,
                    ]);
                    let (a, b) = (s * u, s * (u + depth));
                    self.cover(cover, (x, x + length - 2.5), (a.min(b), a.max(b)));
                    let (a, b) = (s * (u + depth - 2.5), s * (u + depth));
                    self.cover(Cover::Dirt, (x, x + length), (a.min(b), a.max(b)));
                    u += depth;
                }
                let (a, b) = (s * near, s * far);
                self.cover(
                    Cover::Dirt,
                    (x + length - 2.5, x + length),
                    (a.min(b), a.max(b)),
                );
                x += length;
            }
        }
    }

    /// Woods, farmhouses and greenhouses on both sides beyond `FAR_M`.
    fn countryside(&mut self, x0: f64, x1: f64, d: &mut Draws) {
        for s in [-1.0_f64, 1.0] {
            let mut x = x0;
            while x < x1 {
                let stretch = d.uniform(40.0, 90.0);
                let roll = d.uniform(0.0, 1.0);
                if roll < 0.45 {
                    // A wood.
                    let mut tx = x;
                    while tx < (x + stretch).min(x1) {
                        for row in 0..3 {
                            let r = d.uniform(2.5, 4.0);
                            let u = FAR_M + r + f64::from(row) * 7.0 + d.uniform(0.0, 3.0);
                            let c = d.pick(&GREENS);
                            self.tree(tx, s * u, r, c, d);
                        }
                        tx += d.uniform(5.0, 8.0);
                    }
                } else if roll < 0.75 {
                    // Greenhouses, tunnels 6 m wide and 30 m long.
                    let u0 = FAR_M + d.uniform(3.0, 15.0);
                    for k in 0..(stretch / 8.0) as usize {
                        let gx = x + 8.0 * k as f64;
                        let mut faces = Vec::new();
                        let arc: Vec<(f64, f64)> = (0..=6)
                            .map(|j| {
                                let a = std::f64::consts::PI * f64::from(j) / 6.0;
                                (gx + 3.0 - 3.0 * a.cos(), 3.0 * a.sin())
                            })
                            .collect();
                        let (ya, yb) = (s * u0, s * (u0 + 30.0));
                        faces.push(arc.iter().map(|&(ax, z)| Vec3::new(ax, ya, z)).collect());
                        faces.push(arc.iter().map(|&(ax, z)| Vec3::new(ax, yb, z)).collect());
                        for w in arc.windows(2) {
                            faces.push(vec![
                                Vec3::new(w[0].0, ya, w[0].1),
                                Vec3::new(w[1].0, ya, w[1].1),
                                Vec3::new(w[1].0, yb, w[1].1),
                                Vec3::new(w[0].0, yb, w[0].1),
                            ]);
                        }
                        faces.push(vec![
                            Vec3::new(gx, ya, 0.0),
                            Vec3::new(gx + 6.0, ya, 0.0),
                            Vec3::new(gx + 6.0, yb, 0.0),
                            Vec3::new(gx, yb, 0.0),
                        ]);
                        self.solid(
                            convex(&faces),
                            GREENHOUSE,
                            0.25,
                            Finish::Plain,
                            SurfaceMaterial::CONCRETE,
                        );
                    }
                } else {
                    // A farmhouse among trees.
                    let u = FAR_M + d.uniform(15.0, 40.0);
                    let (a, b) = (s * u, s * (u + 10.0));
                    self.house((x + 10.0, x + 24.0), (a.min(b), a.max(b)), 3.2, 6.0, d, 0.0);
                    for k in 0..4 {
                        let c = d.pick(&GREENS);
                        let r = d.uniform(2.5, 3.5);
                        self.tree(x + 4.0 + 8.0 * f64::from(k), s * (u + 15.0), r, c, d);
                    }
                }
                x += stretch + d.uniform(10.0, 40.0);
            }
        }
    }

    /// Wooded hills 650–1000 m from the road on both sides, drawn only.
    fn hills(&mut self, x0: f64, x1: f64, d: &mut Draws) {
        let step = 80.0;
        let n = ((x1 - x0) / step) as usize;
        for s in [-1.0_f64, 1.0] {
            let rows = [650.0, 760.0, 870.0, 1000.0];
            let heights: Vec<Vec<f64>> = rows
                .iter()
                .enumerate()
                .map(|(r, _)| {
                    (0..=n)
                        .map(|_| {
                            if r == 0 || r == 3 {
                                0.0
                            } else {
                                d.uniform(25.0, 90.0)
                            }
                        })
                        .collect()
                })
                .collect();
            let mut triangles = Vec::new();
            for r in 0..rows.len() - 1 {
                for k in 0..n {
                    let v = |rr: usize, kk: usize| {
                        Vec3::new(x0 + step * kk as f64, s * rows[rr], heights[rr][kk])
                    };
                    let up = Vec3::Z;
                    triangles.extend(panel(
                        [v(r, k), v(r, k + 1), v(r + 1, k + 1), v(r + 1, k)],
                        up,
                    ));
                }
            }
            self.paint(HILLS, 1.0, Finish::Leaves, triangles);
        }
    }
}

/// A box from the wall point `a` to `b` (both at the box's base, `b`
/// outward along the wall's normal), `height` tall.
fn cuboid_on(a: Vec3, b: Vec3, height: f64) -> Vec<[Vec3; 3]> {
    let lo = a.min(b);
    let hi = a.max(b);
    let pad = |v: f64| if v.abs() < 0.05 { 0.05 } else { 0.0 };
    let (dx, dy) = (pad(hi.x - lo.x), pad(hi.y - lo.y));
    cuboid(
        Vec3::new(lo.x - dx, lo.y - dy, lo.z),
        Vec3::new(hi.x + dx, hi.y + dy, lo.z + height),
    )
}

/// A crown lobe: an eight-sided double cone of radius `r` around height
/// `mid`, its corners jittered.
fn lobe(x: f64, y: f64, mid: f64, r: f64, d: &mut Draws) -> Vec<[Vec3; 3]> {
    let ring: Vec<Vec3> = (0..8)
        .map(|k| {
            let a = std::f64::consts::TAU * f64::from(k) / 8.0;
            let rr = r * d.uniform(0.85, 1.1);
            Vec3::new(
                x + rr * a.cos(),
                y + rr * a.sin(),
                mid + d.uniform(-0.2, 0.2) * r,
            )
        })
        .collect();
    let (top, bottom) = (
        Vec3::new(x, y, mid + r * 1.05),
        Vec3::new(x, y, mid - r * 0.8),
    );
    let mut faces = Vec::new();
    for k in 0..8 {
        let (a, b) = (ring[k], ring[(k + 1) % 8]);
        faces.push(vec![a, b, top]);
        faces.push(vec![a, b, bottom]);
    }
    convex(&faces)
}

/// Tilts `v` about the horizontal x axis through `centre` so a horizontal
/// disc there faces the road (`toward` -1 south, +1 north).
fn rotate_up(v: Vec3, centre: Vec3, toward: f64) -> Vec3 {
    let r = v - centre;
    // A quarter turn about x: z up becomes y toward the road.
    centre + Vec3::new(r.x, -toward * r.z, toward * r.y)
}

/// The suburb: a road `length_m` long with `open_m` of fields in the middle.
pub(crate) fn build(name: &str, center: (f64, f64), length_m: f64, open_m: f64) -> PlateauCity {
    let mut d = Draws(0x5EED_CAFE);
    let mut b = Builder::default();
    let half = length_m * 0.5;
    let edge = half + 40.0;
    let west_lanes = [
        -0.5 * (edge + open_m * 0.5) - 60.0,
        -0.5 * (edge + open_m * 0.5) + 70.0,
    ];
    let east_lanes = west_lanes.map(|l| -l);
    b.town(-edge, -open_m * 0.5, &west_lanes, &mut d);
    b.town(open_m * 0.5, edge, &east_lanes, &mut d);
    b.poles(-edge, -open_m * 0.5, &west_lanes, &mut d);
    b.poles(open_m * 0.5, edge, &east_lanes, &mut d);
    let verge = ROAD_HALF_WIDTH_M + 2.0;
    b.fields(-open_m * 0.5, open_m * 0.5, verge, FAR_M + 60.0, &mut d);
    b.countryside(-open_m * 0.5 + 10.0, open_m * 0.5 - 10.0, &mut d);
    // Fields behind the houses.
    b.fields(-edge, -open_m * 0.5, 45.0, 160.0, &mut d);
    b.fields(open_m * 0.5, edge, 45.0, 160.0, &mut d);
    b.hills(-half - 600.0, half + 600.0, &mut d);
    let (a, e) = (-half - 150.0, half + 150.0);
    let (w, n) = (-ROAD_HALF_WIDTH_M, ROAD_HALF_WIDTH_M);
    let mut roads = vec![
        [
            Vec3::new(a, w, 0.0),
            Vec3::new(e, w, 0.0),
            Vec3::new(e, n, 0.0),
        ],
        [
            Vec3::new(a, w, 0.0),
            Vec3::new(e, n, 0.0),
            Vec3::new(a, n, 0.0),
        ],
    ];
    for lane in west_lanes.into_iter().chain(east_lanes) {
        let (l0, l1) = (lane - LANE_HALF_WIDTH_M, lane + LANE_HALF_WIDTH_M);
        let (y0, y1) = (-60.0, 60.0);
        roads.push([
            Vec3::new(l0, y0, 0.0),
            Vec3::new(l1, y0, 0.0),
            Vec3::new(l1, y1, 0.0),
        ]);
        roads.push([
            Vec3::new(l0, y0, 0.0),
            Vec3::new(l1, y1, 0.0),
            Vec3::new(l0, y1, 0.0),
        ]);
    }
    PlateauCity::synthetic(
        name,
        center,
        b.triangles,
        b.buildings,
        roads,
        half + 300.0,
        b.scenery,
    )
}
