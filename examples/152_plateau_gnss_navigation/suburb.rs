//! A synthetic suburb (`--area suburb`): one straight road east-west through
//! the origin, a few hundred metres of houses at each end (gabled roofs,
//! block walls or hedges, a car in the drive, garden trees) and rice
//! paddies and vegetable fields between them, with woods and farmhouses only
//! beyond the LiDAR's reach. Every object is in the sensor mesh; windows,
//! doors and the ground covers are drawn only.

use crate::plateau::{Building, Cover, PlateauCity, Scenery};
use rne_gnss::{MeshTriangle, SurfaceMaterial};
use rne_math::Vec3;

const ROAD_HALF_WIDTH_M: f64 = 4.0;
/// The lots' front line, from the road's centre line, meters.
const LOT_FRONT_M: f64 = 5.0;
/// Nearest the open stretch's woods and farmhouses come to the road,
/// meters: beyond the 3D LiDAR's 60 m from the route.
const FAR_M: f64 = 72.0;

const WALLS: [[u8; 3]; 6] = [
    [236, 232, 222],
    [226, 214, 192],
    [196, 178, 150],
    [158, 128, 100],
    [204, 204, 198],
    [180, 190, 196],
];
const ROOFS: [[u8; 3]; 5] = [
    [62, 66, 74],
    [54, 68, 98],
    [124, 74, 54],
    [42, 42, 44],
    [96, 102, 92],
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
const GLASS: [u8; 3] = [64, 76, 90];
const DOOR: [u8; 3] = [110, 84, 60];
const BLOCK_WALL: [u8; 3] = [172, 170, 162];
const TRUNK: [u8; 3] = [92, 72, 54];

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
fn convex(faces: &[Vec<Vec3>]) -> Vec<Vec<[Vec3; 3]>> {
    let corners: Vec<Vec3> = faces.iter().flatten().copied().collect();
    let centre = corners.iter().fold(Vec3::ZERO, |a, &v| a + v) / corners.len() as f64;
    faces
        .iter()
        .map(|face| {
            (1..face.len() - 1)
                .map(|k| {
                    let [a, b, c] = [face[0], face[k], face[k + 1]];
                    if (b - a).cross(c - a).dot(a - centre) < 0.0 {
                        [a, c, b]
                    } else {
                        [a, b, c]
                    }
                })
                .collect()
        })
        .collect()
}

/// The faces of an axis-aligned box.
fn cuboid(lo: Vec3, hi: Vec3) -> Vec<Vec<Vec3>> {
    let v = |x: bool, y: bool, z: bool| {
        Vec3::new(
            if x { hi.x } else { lo.x },
            if y { hi.y } else { lo.y },
            if z { hi.z } else { lo.z },
        )
    };
    let (f, t) = (false, true);
    vec![
        vec![v(f, f, f), v(t, f, f), v(t, t, f), v(f, t, f)],
        vec![v(f, f, t), v(t, f, t), v(t, t, t), v(f, t, t)],
        vec![v(f, f, f), v(t, f, f), v(t, f, t), v(f, f, t)],
        vec![v(f, t, f), v(t, t, f), v(t, t, t), v(f, t, t)],
        vec![v(f, f, f), v(f, t, f), v(f, t, t), v(f, f, t)],
        vec![v(t, f, f), v(t, t, f), v(t, t, t), v(t, f, t)],
    ]
}

/// One part of an object: its triangles, colour, roughness and material.
type Part = (Vec<[Vec3; 3]>, [u8; 3], f32, SurfaceMaterial);

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
        for (triangles, colour, roughness, material) in parts {
            for t in &triangles {
                for v in t {
                    lo = lo.min(*v);
                    hi = hi.max(*v);
                }
                if let Some(tri) = MeshTriangle::new(*t, material, object) {
                    self.triangles.push(tri);
                }
            }
            self.paint(colour, roughness, triangles);
        }
        self.buildings.push(Building {
            bounds: [lo.x, lo.y, hi.x, hi.y],
            z_range: (lo.z, hi.z),
            textured: true,
        });
    }

    /// Adds triangles to the scenery only.
    fn paint(&mut self, colour: [u8; 3], roughness: f32, triangles: Vec<[Vec3; 3]>) {
        match self
            .scenery
            .solids
            .iter_mut()
            .find(|(c, r, _)| *c == colour && (*r - roughness).abs() < 1.0e-6)
        {
            Some((_, _, list)) => list.extend(triangles),
            None => self.scenery.solids.push((colour, roughness, triangles)),
        }
    }

    fn cover(&mut self, cover: Cover, x: (f64, f64), y: (f64, f64)) {
        self.scenery
            .ground
            .push((cover, [(x.0, y.0), (x.1, y.0), (x.1, y.1), (x.0, y.1)]));
    }

    /// A two-storey house on the footprint, gabled along its longer side,
    /// with windows on every wall and a door on the side facing `door_y`.
    fn house(
        &mut self,
        x: (f64, f64),
        y: (f64, f64),
        eave: f64,
        ridge: f64,
        d: &mut Draws,
        door_y: f64,
    ) {
        let along_x = x.1 - x.0 >= y.1 - y.0;
        let (xm, ym) = ((x.0 + x.1) * 0.5, (y.0 + y.1) * 0.5);
        let p = |px: f64, py: f64, z: f64| Vec3::new(px, py, z);
        let [b00, b10, b11, b01] = [
            p(x.0, y.0, 0.0),
            p(x.1, y.0, 0.0),
            p(x.1, y.1, 0.0),
            p(x.0, y.1, 0.0),
        ];
        let [e00, e10, e11, e01] = [b00, b10, b11, b01].map(|v| Vec3::new(v.x, v.y, eave));
        let (r0, r1) = if along_x {
            (p(x.0, ym, ridge), p(x.1, ym, ridge))
        } else {
            (p(xm, y.0, ridge), p(xm, y.1, ridge))
        };
        let (walls, roof) = if along_x {
            (
                vec![
                    vec![b00, b10, e10, e00],
                    vec![b01, b11, e11, e01],
                    vec![b00, b01, e01, r0, e00],
                    vec![b10, b11, e11, r1, e10],
                ],
                vec![vec![e00, e10, r1, r0], vec![e01, e11, r1, r0]],
            )
        } else {
            (
                vec![
                    vec![b00, b01, e01, e00],
                    vec![b10, b11, e11, e10],
                    vec![b00, b10, e10, r0, e00],
                    vec![b01, b11, e11, r1, e01],
                ],
                vec![vec![e00, e01, r1, r0], vec![e10, e11, r1, r0]],
            )
        };
        let floor = vec![vec![b00, b10, b11, b01]];
        let mut faces = convex(&[walls.clone(), roof.clone(), floor].concat()).into_iter();
        let walls_t: Vec<[Vec3; 3]> = faces.by_ref().take(walls.len()).flatten().collect();
        let roof_t: Vec<[Vec3; 3]> = faces.by_ref().take(roof.len()).flatten().collect();
        let floor_t: Vec<[Vec3; 3]> = faces.flatten().collect();
        self.object(vec![
            (walls_t, d.pick(&WALLS), 0.85, SurfaceMaterial::CONCRETE),
            (roof_t, d.pick(&ROOFS), 0.7, SurfaceMaterial::CONCRETE),
            (floor_t, [80, 80, 80], 0.9, SurfaceMaterial::ABSORBER),
        ]);
        // Windows on the four walls (a storey 2.8 m), a door facing the road.
        let centre = Vec3::new(xm, ym, 0.0);
        let mut glass = Vec::new();
        let mut door = Vec::new();
        for (a, b) in [(b00, b10), (b10, b11), (b11, b01), (b01, b00)] {
            let length = (b - a).length();
            let along = (b - a) / length;
            let mut out = Vec3::new(along.y, -along.x, 0.0);
            if out.dot(a - centre) < 0.0 {
                out = -out;
            }
            let at = |u: f64, z: f64| a + along * u + out * 0.03 + Vec3::new(0.0, 0.0, z);
            let quad = |u0: f64, u1: f64, z0: f64, z1: f64| {
                let [q0, q1, q2, q3] = [at(u0, z0), at(u1, z0), at(u1, z1), at(u0, z1)];
                let wound = if (q1 - q0).cross(q2 - q0).dot(out) < 0.0 {
                    [[q0, q2, q1], [q0, q3, q2]]
                } else {
                    [[q0, q1, q2], [q0, q2, q3]]
                };
                wound.to_vec()
            };
            let faces_road = (a.y - centre.y).signum() == (door_y - centre.y).signum()
                && (a.y - b.y).abs() < 1.0e-6;
            let mut u = 1.2;
            let mut first = true;
            while u + 1.3 < length - 0.8 {
                if faces_road && first {
                    door.extend(quad(u, u + 1.0, 0.0, 2.1));
                    first = false;
                } else {
                    glass.extend(quad(u, u + 1.3, 0.9, 2.0));
                }
                if eave > 5.0 {
                    glass.extend(quad(u, u + 1.3, 3.7, 4.8));
                }
                u += d.uniform(2.6, 3.6);
            }
        }
        self.paint(GLASS, 0.15, glass);
        self.paint(DOOR, 0.6, door);
    }

    fn cuboid(
        &mut self,
        lo: Vec3,
        hi: Vec3,
        colour: [u8; 3],
        roughness: f32,
        material: SurfaceMaterial,
    ) {
        let t = convex(&cuboid(lo, hi)).into_iter().flatten().collect();
        self.object(vec![(t, colour, roughness, material)]);
    }

    /// A tree: a trunk and an eight-sided double-cone crown of radius `r`.
    fn tree(&mut self, x: f64, y: f64, r: f64, colour: [u8; 3]) {
        let trunk_top = 1.6 + r * 0.4;
        let trunk = convex(&cuboid(
            Vec3::new(x - 0.15, y - 0.15, 0.0),
            Vec3::new(x + 0.15, y + 0.15, trunk_top + 0.3),
        ))
        .into_iter()
        .flatten()
        .collect();
        let mid = trunk_top + r * 0.6;
        let ring: Vec<Vec3> = (0..8)
            .map(|k| {
                let a = std::f64::consts::TAU * f64::from(k) / 8.0;
                Vec3::new(x + r * a.cos(), y + r * a.sin(), mid)
            })
            .collect();
        let (top, bottom) = (Vec3::new(x, y, mid + r * 1.2), Vec3::new(x, y, trunk_top));
        let mut faces = Vec::new();
        for k in 0..8 {
            let (a, b) = (ring[k], ring[(k + 1) % 8]);
            faces.push(vec![a, b, top]);
            faces.push(vec![a, b, bottom]);
        }
        let crown = convex(&faces).into_iter().flatten().collect();
        self.object(vec![
            (trunk, TRUNK, 0.9, SurfaceMaterial::ABSORBER),
            (crown, colour, 0.95, SurfaceMaterial::ABSORBER),
        ]);
    }

    /// A parked car, nose to the road, centred at `(x, y)`.
    fn car(&mut self, x: f64, y: f64, colour: [u8; 3]) {
        let (hw, hl) = (0.87, 2.2);
        let body = convex(&cuboid(
            Vec3::new(x - hw, y - hl, 0.2),
            Vec3::new(x + hw, y + hl, 0.95),
        ))
        .into_iter()
        .flatten()
        .collect();
        let cabin = convex(&cuboid(
            Vec3::new(x - hw + 0.08, y - hl * 0.5, 0.95),
            Vec3::new(x + hw - 0.08, y + hl * 0.55, 1.5),
        ))
        .into_iter()
        .flatten()
        .collect();
        self.object(vec![
            (body, colour, 0.35, SurfaceMaterial::CONCRETE),
            (cabin, GLASS, 0.15, SurfaceMaterial::CONCRETE),
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
        let back = front + 2.5;
        let house_x = (x0 + drive + 0.8, x1 - 1.2);
        if street {
            self.cover(Cover::Concrete, (x0, x0 + drive), span(front, front + 6.5));
            if d.chance(0.75) {
                let c = d.pick(&CARS);
                self.car(x0 + drive * 0.5, y(front + 3.4), c);
            }
            if d.chance(0.6) {
                let (y0, y1) = span(front + 0.1, front + 0.25);
                self.cuboid(
                    Vec3::new(x0 + drive + 0.2, y0, 0.0),
                    Vec3::new(x1 - 0.3, y1, 1.2),
                    BLOCK_WALL,
                    0.9,
                    SurfaceMaterial::CONCRETE,
                );
            } else {
                let (y0, y1) = span(front + 0.1, front + 0.8);
                let c = d.pick(&GREENS);
                self.cuboid(
                    Vec3::new(x0 + drive + 0.2, y0, 0.0),
                    Vec3::new(x1 - 0.3, y1, 1.3),
                    c,
                    0.95,
                    SurfaceMaterial::ABSORBER,
                );
            }
        }
        let eave = d.uniform(5.3, 6.1);
        let ridge = eave + d.uniform(1.5, 2.5);
        let road_side = y(front);
        self.house(house_x, span(back, back + depth), eave, ridge, d, road_side);
        if d.chance(0.45) {
            let r = d.uniform(1.0, 1.7);
            let c = d.pick(&GREENS);
            self.tree(x1 - 1.6, y(back + depth + 2.0), r, c);
        }
    }

    /// Two rows of lots on both sides from `x0` to `x1`.
    fn town(&mut self, x0: f64, x1: f64, d: &mut Draws) {
        for s in [-1.0, 1.0] {
            for (front, street) in [(LOT_FRONT_M, true), (LOT_FRONT_M + 18.0, false)] {
                let mut x = x0;
                loop {
                    let width = d.uniform(11.5, 15.0);
                    if x + width > x1 {
                        break;
                    }
                    self.lot(x, x + width - 0.6, s, front, street, d);
                    x += width;
                }
            }
        }
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
                self.cover(Cover::Dirt, (x + length - 2.5, x + length), {
                    let (a, b) = (s * near, s * far);
                    (a.min(b), a.max(b))
                });
                x += length;
            }
        }
    }

    /// Woods and a few farmhouses on both sides beyond `FAR_M`.
    fn countryside(&mut self, x0: f64, x1: f64, d: &mut Draws) {
        for s in [-1.0_f64, 1.0] {
            let mut x = x0;
            while x < x1 {
                let stretch = d.uniform(40.0, 90.0);
                if d.chance(0.6) {
                    // A wood.
                    let mut tx = x;
                    while tx < (x + stretch).min(x1) {
                        for row in 0..3 {
                            let r = d.uniform(2.5, 4.0);
                            let u = FAR_M + r + f64::from(row) * 7.0 + d.uniform(0.0, 3.0);
                            let c = d.pick(&GREENS);
                            self.tree(tx, s * u, r, c);
                        }
                        tx += d.uniform(5.0, 8.0);
                    }
                } else if d.chance(0.5) {
                    // A farmhouse among trees.
                    let u = FAR_M + d.uniform(15.0, 40.0);
                    let (a, b) = (s * u, s * (u + 10.0));
                    self.house((x + 10.0, x + 24.0), (a.min(b), a.max(b)), 3.2, 6.0, d, 0.0);
                    for k in 0..4 {
                        let c = d.pick(&GREENS);
                        self.tree(
                            x + 4.0 + 8.0 * f64::from(k),
                            s * (u + 15.0),
                            d.uniform(2.5, 3.5),
                            c,
                        );
                    }
                }
                x += stretch + d.uniform(10.0, 40.0);
            }
        }
    }
}

/// The suburb: a road `length_m` long with `open_m` of fields in the middle.
pub(crate) fn build(name: &str, center: (f64, f64), length_m: f64, open_m: f64) -> PlateauCity {
    let mut d = Draws(0x5EED_CAFE);
    let mut b = Builder::default();
    let half = length_m * 0.5;
    let edge = half + 40.0;
    b.town(-edge, -open_m * 0.5, &mut d);
    b.town(open_m * 0.5, edge, &mut d);
    let verge = ROAD_HALF_WIDTH_M + 2.0;
    b.fields(-open_m * 0.5, open_m * 0.5, verge, FAR_M + 60.0, &mut d);
    b.countryside(-open_m * 0.5 + 10.0, open_m * 0.5 - 10.0, &mut d);
    // Fields behind the houses.
    b.fields(-edge, -open_m * 0.5, 45.0, 160.0, &mut d);
    b.fields(open_m * 0.5, edge, 45.0, 160.0, &mut d);
    let (a, e) = (-half - 150.0, half + 150.0);
    let (w, n) = (-ROAD_HALF_WIDTH_M, ROAD_HALF_WIDTH_M);
    let roads = vec![
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
