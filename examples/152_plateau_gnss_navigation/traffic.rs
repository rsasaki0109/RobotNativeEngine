//! Vehicles on the street beside the route: buses, trucks and cars in both
//! directions. They are metal boxes to the GNSS ray tracer (strong
//! reflectors and occluders the city model used for NLOS prediction does not
//! have) and obstacles to the LiDARs.

use crate::crowd::{pedestrian_boxes, Crowd, Weather};
use crate::plateau::PlateauCity;
use rne_core::DeterministicRng;
use rne_gnss::{SceneBox, SurfaceMaterial};
use rne_math::Vec3;

/// Kinds of vehicles: (length, width, height) in meters.
const BUS: (f64, f64, f64) = (11.0, 2.5, 3.2);
const TRUCK: (f64, f64, f64) = (7.5, 2.3, 3.0);
const CAR: (f64, f64, f64) = (4.5, 1.8, 1.5);
/// Lane centres, meters to the side of the route (left positive).
const LANES_M: [f64; 2] = [5.0, -5.0];
/// Vehicles keep out of this radius around the route's start (the
/// pedestrian crossing there), meters.
const CROSSING_CLEAR_M: f64 = 18.0;
/// Vehicles keep their footprint at least this far from the route the
/// robot walks (a lane beside one leg of the route can cross the next leg
/// at a corner), meters.
const ROUTE_CLEAR_M: f64 = 1.5;

/// A vehicle at one instant: footprint centre, heading and half sizes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct VehicleBox {
    pub center: (f64, f64),
    pub ground_m: f64,
    pub yaw_rad: f64,
    pub half: (f64, f64, f64),
}

#[derive(Clone, Copy, Debug)]
struct Vehicle {
    lane: usize,
    start_m: f64,
    speed_m_s: f64,
    size: (f64, f64, f64),
}

/// Vehicles driving along the route's street at 6-12 m/s, deterministic
/// from the seed. They are left out wherever their footprint would be
/// inside a building (the route's street is not always wide enough for two
/// lanes), within `ROUTE_CLEAR_M` of the route (they do not drive through
/// the robot) and near the crossing at the start.
#[derive(Clone, Debug)]
pub(crate) struct Traffic {
    route: Vec<(f64, f64)>,
    cumulative_m: Vec<f64>,
    vehicles: Vec<Vehicle>,
}

impl Traffic {
    pub(crate) fn new(route: &[(f64, f64)], count: usize, seed: u64) -> Self {
        let mut cumulative_m = vec![0.0];
        for pair in route.windows(2) {
            let length = (pair[1].0 - pair[0].0).hypot(pair[1].1 - pair[0].1);
            cumulative_m.push(cumulative_m.last().expect("start") + length);
        }
        let total = *cumulative_m.last().expect("length");
        let mut rng = DeterministicRng::new(seed);
        let vehicles = (0..count)
            .map(|k| {
                let pick = rng.uniform_f64(0.0, 1.0);
                Vehicle {
                    lane: k % 2,
                    start_m: rng.uniform_f64(0.0, total),
                    speed_m_s: rng.uniform_f64(6.0, 12.0),
                    size: if pick < 0.15 {
                        BUS
                    } else if pick < 0.35 {
                        TRUCK
                    } else {
                        CAR
                    },
                }
            })
            .collect();
        Self {
            route: route.to_vec(),
            cumulative_m,
            vehicles,
        }
    }

    /// The vehicles on the street at `time_s`.
    pub(crate) fn boxes(&self, time_s: f64, city: &PlateauCity) -> Vec<VehicleBox> {
        let total = *self.cumulative_m.last().expect("length");
        if total <= 0.0 {
            return Vec::new();
        }
        let start = self.route[0];
        self.vehicles
            .iter()
            .filter_map(|v| {
                // Lane 0 drives along the route, lane 1 against it.
                let travelled = (v.start_m + v.speed_m_s * time_s).rem_euclid(total);
                let s = if v.lane == 0 {
                    travelled
                } else {
                    total - travelled
                };
                let (x, y, heading) = self.along(s, LANES_M[v.lane]);
                let yaw = if v.lane == 0 {
                    heading
                } else {
                    heading + std::f64::consts::PI
                };
                if (x - start.0).hypot(y - start.1) < CROSSING_CLEAR_M {
                    return None;
                }
                let ground = city.ground_m(x, y);
                let (sin, cos) = yaw.sin_cos();
                let (hl, hw) = (0.5 * v.size.0, 0.5 * v.size.1);
                // The footprint's outline (every metre along its sides) and
                // centre must all be outside buildings and clear of the route.
                let steps = (2.0 * hl).ceil() as usize;
                let mut points = vec![(0.0, 0.0), (hl, 0.0), (-hl, 0.0)];
                for k in 0..=steps {
                    let a = -hl + 2.0 * hl * k as f64 / steps.max(1) as f64;
                    points.extend([(a, hw), (a, -hw)]);
                }
                let clear = points.iter().all(|(a, b)| {
                    let (px, py) = (x + a * cos - b * sin, y + a * sin + b * cos);
                    !city.mesh.contains(Vec3::new(px, py, ground + 1.0))
                        && crate::nav::distance_to_route(&self.route, px, py) >= ROUTE_CLEAR_M
                });
                clear.then_some(VehicleBox {
                    center: (x, y),
                    ground_m: ground,
                    yaw_rad: yaw,
                    half: (hl, hw, 0.5 * v.size.2),
                })
            })
            .collect()
    }

    /// Point `lateral_m` to the left of the route at arc length `s_m`, and the
    /// route's heading there.
    fn along(&self, s_m: f64, lateral_m: f64) -> (f64, f64, f64) {
        let leg = self
            .cumulative_m
            .partition_point(|&c| c <= s_m)
            .clamp(1, self.route.len() - 1);
        let (a, b) = (self.route[leg - 1], self.route[leg]);
        let length = (b.0 - a.0).hypot(b.1 - a.1).max(1.0e-9);
        let f = ((s_m - self.cumulative_m[leg - 1]) / length).clamp(0.0, 1.0);
        let (dx, dy) = ((b.0 - a.0) / length, (b.1 - a.1) / length);
        (
            a.0 + f * (b.0 - a.0) - dy * lateral_m,
            a.1 + f * (b.1 - a.1) + dx * lateral_m,
            dy.atan2(dx),
        )
    }
}

/// Everything that moves along the route: the crowd and the traffic.
pub(crate) struct Street {
    crowd: Crowd,
    traffic: Traffic,
}

/// The people and the vehicles at one instant.
#[derive(Clone, Debug, Default)]
pub(crate) struct Movers {
    pub people: Vec<(f64, f64)>,
    pub vehicles: Vec<VehicleBox>,
}

impl Street {
    pub(crate) fn new(route: &[(f64, f64)], weather: Weather, seed: u64) -> Self {
        Self {
            crowd: Crowd::new(route, weather.pedestrians, 0xC0_0D ^ seed)
                .with_scramble(weather.scramble, 0x5C_4A ^ seed),
            traffic: Traffic::new(route, weather.vehicles, 0x7A_F1C ^ seed),
        }
    }

    /// Where everyone is at `time_s`, people stepping around the robot.
    pub(crate) fn at(&self, time_s: f64, robot: (f64, f64), city: &PlateauCity) -> Movers {
        Movers {
            people: self.crowd.positions(time_s, robot),
            vehicles: self.traffic.boxes(time_s, city),
        }
    }
}

impl Movers {
    /// Distance from `(x, y)` to the nearest person's body or vehicle,
    /// meters.
    pub(crate) fn nearest_m(&self, (x, y): (f64, f64)) -> f64 {
        let people = self
            .people
            .iter()
            .map(|p| (p.0 - x).hypot(p.1 - y) - crate::crowd::PEDESTRIAN_RADIUS_M);
        let vehicles = self.vehicles.iter().map(|v| {
            let (sin, cos) = v.yaw_rad.sin_cos();
            let (dx, dy) = (x - v.center.0, y - v.center.1);
            let (u, w) = (dx * cos + dy * sin, -dx * sin + dy * cos);
            (u.abs() - v.half.0)
                .max(0.0)
                .hypot((w.abs() - v.half.1).max(0.0))
        });
        people.chain(vehicles).fold(f64::INFINITY, f64::min)
    }

    /// The people (absorbers) and vehicles (metal) for the GNSS ray tracer.
    pub(crate) fn gnss_boxes(&self, city: &PlateauCity) -> Vec<SceneBox> {
        let mut boxes = pedestrian_boxes(&self.people, |x, y| city.ground_m(x, y));
        boxes.extend(vehicle_boxes(&self.vehicles));
        boxes
    }
}

/// Vehicles as GNSS scene boxes (axis-aligned bounds of the rotated body,
/// metal).
pub(crate) fn vehicle_boxes(vehicles: &[VehicleBox]) -> Vec<SceneBox> {
    vehicles
        .iter()
        .map(|v| {
            let (sin, cos) = v.yaw_rad.sin_cos();
            let ex = v.half.0 * cos.abs() + v.half.1 * sin.abs();
            let ey = v.half.0 * sin.abs() + v.half.1 * cos.abs();
            SceneBox::new(
                Vec3::new(v.center.0 - ex, v.center.1 - ey, v.ground_m + 0.3),
                Vec3::new(
                    v.center.0 + ex,
                    v.center.1 + ey,
                    v.ground_m + 2.0 * v.half.2,
                ),
                SurfaceMaterial::METAL,
            )
        })
        .collect()
}

/// Range along a horizontal ray `height_m` above the ground (a planar
/// LiDAR slice) to the nearest vehicle body, if any.
pub(crate) fn vehicle_range_planar(
    vehicles: &[VehicleBox],
    (x, y): (f64, f64),
    (dx, dy): (f64, f64),
    height_m: f64,
) -> Option<f64> {
    vehicles
        .iter()
        .filter_map(|v| {
            vehicle_range(
                std::slice::from_ref(v),
                Vec3::new(x, y, v.ground_m + height_m),
                Vec3::new(dx, dy, 0.0),
            )
        })
        .fold(None, |best: Option<f64>, t| {
            Some(best.map_or(t, |b| b.min(t)))
        })
}

/// Range along a ray to the nearest vehicle body (oriented boxes from 0.2 m
/// above the ground to the roof), if any.
pub(crate) fn vehicle_range(vehicles: &[VehicleBox], origin: Vec3, dir: Vec3) -> Option<f64> {
    vehicles
        .iter()
        .filter_map(|v| {
            let (sin, cos) = v.yaw_rad.sin_cos();
            // Into the vehicle's frame.
            let (ox, oy) = (origin.x - v.center.0, origin.y - v.center.1);
            let o = [
                ox * cos + oy * sin,
                -ox * sin + oy * cos,
                origin.z - v.ground_m,
            ];
            let d = [dir.x * cos + dir.y * sin, -dir.x * sin + dir.y * cos, dir.z];
            let lo = [-v.half.0, -v.half.1, 0.2];
            let hi = [v.half.0, v.half.1, 2.0 * v.half.2];
            let (mut near, mut far) = (0.0_f64, f64::INFINITY);
            for axis in 0..3 {
                if d[axis].abs() < 1.0e-12 {
                    if o[axis] < lo[axis] || o[axis] > hi[axis] {
                        return None;
                    }
                    continue;
                }
                let (t1, t2) = (
                    (lo[axis] - o[axis]) / d[axis],
                    (hi[axis] - o[axis]) / d[axis],
                );
                near = near.max(t1.min(t2));
                far = far.min(t1.max(t2));
            }
            (near <= far && near > 0.0).then_some(near)
        })
        .fold(None, |best: Option<f64>, t| {
            Some(best.map_or(t, |b| b.min(t)))
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ray_hits_a_turned_vehicle_on_its_side() {
        let bus = VehicleBox {
            center: (10.0, 0.0),
            ground_m: 0.0,
            yaw_rad: std::f64::consts::FRAC_PI_2,
            half: (5.5, 1.25, 1.6),
        };
        // Along x at 1 m height: the bus stands across the ray, 2.5 m wide.
        let t = vehicle_range(&[bus], Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
        assert!((t - 8.75).abs() < 1e-9, "{t}");
        // Under the body (0.1 m) or over the roof it misses.
        assert!(
            vehicle_range(&[bus], Vec3::new(0.0, 0.0, 0.1), Vec3::new(1.0, 0.0, 0.0)).is_none()
        );
        assert!(
            vehicle_range(&[bus], Vec3::new(0.0, 0.0, 3.5), Vec3::new(1.0, 0.0, 0.0)).is_none()
        );
        // The GNSS box covers the rotated footprint.
        let b = vehicle_boxes(&[bus])[0];
        assert!((b.max_m.x - b.min_m.x - 2.5).abs() < 1e-9);
        assert!((b.max_m.y - b.min_m.y - 11.0).abs() < 1e-9);
    }
}
