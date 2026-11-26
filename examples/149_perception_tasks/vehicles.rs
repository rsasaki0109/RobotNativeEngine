//! Procedural car and bicycle meshes, and a riding pose for the person mesh.
//!
//! The meshes are generated here (original geometry, no external asset), in
//! the object's frame: engine y-up, +x forward, +z to the object's right,
//! origin on the ground under the object's centre.

use std::f64::consts::{PI, TAU};

use rne_math::{DQuat, Vec3};
use rne_render::TriangleMesh;

/// A mesh part with its linear RGB color.
pub(crate) struct Part {
    pub(crate) mesh: TriangleMesh,
    pub(crate) color: [f32; 3],
}

/// Accumulates triangles; normals are averaged per vertex.
#[derive(Default)]
struct Builder {
    positions: Vec<Vec3>,
    indices: Vec<u32>,
}

impl Builder {
    fn vertex(&mut self, p: Vec3) -> u32 {
        self.positions.push(p);
        self.positions.len() as u32 - 1
    }

    fn tri(&mut self, a: u32, b: u32, c: u32) {
        self.indices.extend([a, b, c]);
    }

    /// Joins consecutive closed rings of equal length with quads.
    fn rings(&mut self, rings: &[Vec<Vec3>], cap_ends: bool) {
        let n = rings[0].len() as u32;
        let first = self.positions.len() as u32;
        for ring in rings {
            for p in ring {
                self.vertex(*p);
            }
        }
        for r in 0..rings.len() as u32 - 1 {
            for k in 0..n {
                let a = first + r * n + k;
                let b = first + r * n + (k + 1) % n;
                let c = first + (r + 1) * n + k;
                let d = first + (r + 1) * n + (k + 1) % n;
                self.tri(a, c, b);
                self.tri(b, c, d);
            }
        }
        if cap_ends {
            for (ring, flip) in [(0, true), (rings.len() - 1, false)] {
                let centre = rings[ring].iter().fold(Vec3::ZERO, |s, p| s + *p) / f64::from(n);
                let c = self.vertex(centre);
                let base = first + ring as u32 * n;
                for k in 0..n {
                    let (a, b) = (base + k, base + (k + 1) % n);
                    if flip {
                        self.tri(c, a, b);
                    } else {
                        self.tri(c, b, a);
                    }
                }
            }
        }
    }

    fn finish(self, color: [f32; 3]) -> Part {
        let mut normals = vec![Vec3::ZERO; self.positions.len()];
        for t in self.indices.chunks_exact(3) {
            let [a, b, c] = [t[0] as usize, t[1] as usize, t[2] as usize];
            let n = (self.positions[b] - self.positions[a])
                .cross(self.positions[c] - self.positions[a]);
            for i in [a, b, c] {
                normals[i] += n;
            }
        }
        let f = |v: Vec3| [v.x as f32, v.y as f32, v.z as f32];
        Part {
            mesh: TriangleMesh {
                positions: self.positions.iter().map(|p| f(*p)).collect(),
                normals: normals.iter().map(|n| f(n.normalize_or(Vec3::Y))).collect(),
                texcoords: vec![[0.0, 0.0]; self.positions.len()],
                indices: self.indices,
                skinning: None,
            },
            color,
        }
    }
}

/// Two unit vectors perpendicular to `axis` and each other.
fn basis(axis: Vec3) -> (Vec3, Vec3) {
    let helper = if axis.y.abs() < 0.9 { Vec3::Y } else { Vec3::X };
    let u = axis.cross(helper).normalize();
    (u, axis.cross(u).normalize())
}

/// A tube of radius `r` from `a` to `b`, closed at both ends.
fn tube(b: &mut Builder, from: Vec3, to: Vec3, r: f64) {
    let axis = (to - from).normalize_or(Vec3::Y);
    let (u, v) = basis(axis);
    let ring = |c: Vec3| {
        (0..10)
            .map(|k| {
                let t = TAU * f64::from(k) / 10.0;
                c + (u * t.cos() + v * t.sin()) * r
            })
            .collect::<Vec<_>>()
    };
    b.rings(&[ring(from), ring(to)], true);
}

/// A torus around `axis` through `centre`.
fn torus(b: &mut Builder, centre: Vec3, axis: Vec3, major: f64, minor: f64) {
    let (u, v) = basis(axis.normalize());
    let rings: Vec<Vec<Vec3>> = (0..=36)
        .map(|i| {
            let a = TAU * f64::from(i) / 36.0;
            let radial = u * a.cos() + v * a.sin();
            (0..10)
                .map(|k| {
                    let t = TAU * f64::from(k) / 10.0;
                    centre
                        + radial * (major + minor * t.cos())
                        + axis.normalize() * (minor * t.sin())
                })
                .collect()
        })
        .collect();
    b.rings(&rings, false);
}

/// A flat disc (thin cylinder) around `axis`.
fn disc(b: &mut Builder, centre: Vec3, axis: Vec3, radius: f64, thickness: f64) {
    let half = axis.normalize() * (thickness / 2.0);
    let (u, v) = basis(axis.normalize());
    let ring = |c: Vec3| {
        (0..24)
            .map(|k| {
                let t = TAU * f64::from(k) / 24.0;
                c + (u * t.cos() + v * t.sin()) * radius
            })
            .collect::<Vec<_>>()
    };
    b.rings(&[ring(centre - half), ring(centre + half)], true);
}

/// A rounded cross-section: superellipse of half-width `w` and half-height `h`.
fn section(x: f64, w_bottom: f64, w_top: f64, z_bottom: f64, z_top: f64) -> Vec<Vec3> {
    let zc = (z_bottom + z_top) / 2.0;
    let h = (z_top - z_bottom) / 2.0;
    (0..24)
        .map(|k| {
            let t = TAU * f64::from(k) / 24.0;
            let (s, c) = t.sin_cos();
            let p = |v: f64| v.signum() * v.abs().powf(0.4);
            let y = zc + h * p(s);
            let blend = (y - z_bottom) / (z_top - z_bottom).max(1e-6);
            let w = w_bottom + (w_top - w_bottom) * blend;
            Vec3::new(x, y, w * p(c))
        })
        .collect()
}

/// Piecewise-linear interpolation through `(x, value)` knots.
fn knots(x: f64, points: &[(f64, f64)]) -> f64 {
    for pair in points.windows(2) {
        let ((x0, y0), (x1, y1)) = (pair[0], pair[1]);
        if x <= x1 {
            let s = ((x - x0) / (x1 - x0)).clamp(0.0, 1.0);
            return y0 + s * (y1 - y0);
        }
    }
    points[points.len() - 1].1
}

/// A 4.4 m sedan: painted body, glass cabin, roof, wheels, lights.
pub(crate) fn car(paint: [f32; 3]) -> Vec<Part> {
    let mut body = Builder::default();
    // Body: sill to belt line, the hood and boot falling toward the ends.
    let stations: Vec<f64> = (0..=22).map(|i| -2.2 + 4.4 * f64::from(i) / 22.0).collect();
    let body_rings: Vec<Vec<Vec3>> = stations
        .iter()
        .map(|&x| {
            let top = knots(x, &[(-2.2, 0.78), (-1.9, 0.98), (1.4, 0.95), (2.2, 0.72)]);
            let bottom = knots(x, &[(-2.2, 0.32), (-2.0, 0.24), (1.9, 0.24), (2.2, 0.34)]);
            let w = knots(x, &[(-2.2, 0.78), (-1.9, 0.9), (1.8, 0.9), (2.2, 0.8)]);
            section(x, w, w - 0.06, bottom, top)
        })
        .collect();
    body.rings(&body_rings, true);

    // Cabin (glass): windshield from x = 1.0 up to the roof at 0.0, rear window down to -1.5.
    let mut glass = Builder::default();
    let cabin: Vec<f64> = (0..=16)
        .map(|i| -1.55 + 2.6 * f64::from(i) / 16.0)
        .collect();
    let glass_rings: Vec<Vec<Vec3>> = cabin
        .iter()
        .map(|&x| {
            let top = knots(x, &[(-1.55, 0.97), (-0.9, 1.4), (0.15, 1.44), (1.05, 0.97)]);
            section(x, 0.84, 0.66, 0.9, top.max(0.93))
        })
        .collect();
    glass.rings(&glass_rings, true);

    // Roof skin over the flat part of the cabin.
    let mut roof = Builder::default();
    let roof_rings: Vec<Vec<Vec3>> = (0..=8)
        .map(|i| {
            let x = -0.85 + 1.0 * f64::from(i) / 8.0;
            section(x, 0.69, 0.66, 1.36, 1.46)
        })
        .collect();
    roof.rings(&roof_rings, true);

    let mut tyres = Builder::default();
    let mut rims = Builder::default();
    for (x, z) in [(1.35, 0.8), (1.35, -0.8), (-1.35, 0.8), (-1.35, -0.8)] {
        let c = Vec3::new(x, 0.33, z);
        torus(&mut tyres, c, Vec3::Z, 0.24, 0.09);
        disc(
            &mut rims,
            c + Vec3::new(0.0, 0.0, 0.07 * z.signum()),
            Vec3::Z,
            0.19,
            0.04,
        );
    }
    let mut head = Builder::default();
    let mut tail = Builder::default();
    for z in [0.6, -0.6] {
        disc(&mut head, Vec3::new(2.17, 0.72, z), Vec3::X, 0.1, 0.06);
        disc(&mut tail, Vec3::new(-2.17, 0.82, z), Vec3::X, 0.09, 0.06);
    }
    vec![
        body.finish(paint),
        glass.finish([0.03, 0.04, 0.05]),
        roof.finish(paint),
        tyres.finish([0.02, 0.02, 0.02]),
        rims.finish([0.55, 0.55, 0.58]),
        head.finish([1.0, 0.95, 0.8]),
        tail.finish([0.6, 0.02, 0.02]),
    ]
}

/// Where the rider's hips, hands and feet go on the bicycle (object frame).
pub(crate) const SADDLE: [f64; 3] = [-0.2, 0.9, 0.0];

/// A city bicycle: wheels, diamond frame, fork, handlebar, saddle and cranks.
pub(crate) fn bicycle(frame_color: [f32; 3]) -> Vec<Part> {
    let p = |x: f64, y: f64, z: f64| Vec3::new(x, y, z);
    let (rear, front) = (p(-0.52, 0.34, 0.0), p(0.52, 0.34, 0.0));
    let bracket = p(-0.02, 0.3, 0.0);
    let seat_top = p(-0.17, 0.82, 0.0);
    let head_top = p(0.38, 0.8, 0.0);
    let head_bottom = p(0.43, 0.62, 0.0);

    let mut tyres = Builder::default();
    let mut rims = Builder::default();
    for c in [rear, front] {
        torus(&mut tyres, c, Vec3::Z, 0.31, 0.028);
        torus(&mut rims, c, Vec3::Z, 0.28, 0.012);
        for k in 0..12 {
            let a = TAU * f64::from(k) / 12.0;
            let spoke = c + p(a.cos() * 0.28, a.sin() * 0.28, 0.0);
            tube(&mut rims, c, spoke, 0.004);
        }
    }
    let mut frame = Builder::default();
    let r = 0.02;
    tube(&mut frame, bracket, seat_top, r);
    tube(&mut frame, seat_top, head_top, r);
    tube(&mut frame, bracket, head_bottom, r * 1.2);
    tube(&mut frame, head_bottom, head_top, r * 1.3);
    for z in [0.05, -0.05] {
        tube(&mut frame, bracket, rear + p(0.0, 0.0, z), r * 0.7);
        tube(
            &mut frame,
            seat_top + p(0.02, -0.04, 0.0),
            rear + p(0.0, 0.0, z),
            r * 0.7,
        );
        tube(&mut frame, head_bottom, front + p(0.0, 0.0, z), r * 0.8);
    }
    let mut parts = Builder::default();
    let bar = p(0.33, 0.98, 0.0);
    tube(&mut parts, head_top, bar, 0.015);
    tube(
        &mut parts,
        bar + p(0.0, 0.0, -0.28),
        bar + p(0.0, 0.0, 0.28),
        0.014,
    );
    tube(
        &mut parts,
        seat_top,
        p(SADDLE[0], SADDLE[1] - 0.04, 0.0),
        0.013,
    );
    // Saddle and cranks with pedals.
    let saddle: Vec<Vec<Vec3>> = (0..=6)
        .map(|i| {
            let x = SADDLE[0] - 0.13 + 0.26 * f64::from(i) / 6.0;
            let w = knots(x - SADDLE[0], &[(-0.13, 0.09), (0.0, 0.07), (0.13, 0.025)]);
            section(x, w, w, SADDLE[1] - 0.04, SADDLE[1] + 0.01)
        })
        .collect();
    parts.rings(&saddle, true);
    for (side, phase) in [(0.09, 0.3), (-0.09, 0.3 + PI)] {
        let pedal = bracket + p(phase.cos() * 0.17, phase.sin() * 0.17, side);
        tube(&mut parts, bracket + p(0.0, 0.0, side), pedal, 0.012);
        tube(
            &mut parts,
            pedal + p(0.0, 0.0, -0.05),
            pedal + p(0.0, 0.0, 0.05),
            0.015,
        );
    }
    vec![
        tyres.finish([0.02, 0.02, 0.02]),
        rims.finish([0.6, 0.6, 0.62]),
        frame.finish(frame_color),
        parts.finish([0.05, 0.05, 0.05]),
    ]
}

/// Rotations that turn the standing pose into a riding one, by joint name,
/// about the person mesh's own axes (x to the person's left, y up, z
/// forward), applied to the bone after its parents have moved.
pub(crate) fn riding_pose() -> Vec<(&'static str, DQuat)> {
    let x = |deg: f64| DQuat::from_rotation_x(deg.to_radians());
    let z = |deg: f64| DQuat::from_rotation_z(deg.to_radians());
    vec![
        // Lean the torso forward.
        ("spine_01", x(28.0)),
        // Hips flexed, knees bent: one pedal up, one down.
        ("thigh_l", x(-75.0)),
        ("thigh_r", x(-55.0)),
        ("calf_l", x(95.0)),
        ("calf_r", x(85.0)),
        // Arms forward and down to the handlebar, elbows slightly bent.
        ("upperarm_l", x(-50.0) * z(-12.0)),
        ("upperarm_r", x(-50.0) * z(12.0)),
        ("lowerarm_l", x(-15.0)),
        ("lowerarm_r", x(-15.0)),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Axis-aligned bounds `(min, max)` of all parts.
    fn bounds(parts: &[Part]) -> ([f32; 3], [f32; 3]) {
        let mut lo = [f32::MAX; 3];
        let mut hi = [f32::MIN; 3];
        for part in parts {
            let mesh = &part.mesh;
            assert_eq!(mesh.indices.len() % 3, 0);
            assert!(mesh
                .indices
                .iter()
                .all(|i| (*i as usize) < mesh.positions.len()));
            assert_eq!(mesh.normals.len(), mesh.positions.len());
            for n in &mesh.normals {
                let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                assert!((length - 1.0).abs() < 1e-4);
            }
            for p in &mesh.positions {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
        (lo, hi)
    }

    #[test]
    fn car_has_a_sedan_size_and_stands_on_the_ground() {
        let (lo, hi) = bounds(&car([0.5, 0.1, 0.1]));
        assert!(
            (hi[0] - lo[0] - 4.4).abs() < 0.1,
            "length {}",
            hi[0] - lo[0]
        );
        assert!((hi[2] - lo[2] - 1.8).abs() < 0.1, "width {}", hi[2] - lo[2]);
        assert!(
            (hi[1] - 1.46).abs() < 0.05 && lo[1].abs() < 0.01,
            "height {} to {}",
            lo[1],
            hi[1]
        );
    }

    #[test]
    fn bicycle_has_a_bicycle_size_and_stands_on_the_ground() {
        let (lo, hi) = bounds(&bicycle([0.1, 0.2, 0.6]));
        assert!(
            (hi[0] - lo[0] - 1.73).abs() < 0.05,
            "length {}",
            hi[0] - lo[0]
        );
        assert!(hi[2] - lo[2] < 0.7, "width {}", hi[2] - lo[2]);
        assert!(
            lo[1].abs() < 0.01 && hi[1] < 1.05,
            "height {} to {}",
            lo[1],
            hi[1]
        );
    }
}
