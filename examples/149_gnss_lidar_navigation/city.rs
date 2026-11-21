//! The site: an open plaza, a street of towers with sidewalks, trees, street
//! furniture, parked and moving traffic, and a single-storey office.
//!
//! The same geometry serves three purposes, as on a real deployment:
//!
//! * the **world** the sensors see (`world_scene`): buildings, street
//!   furniture, tree trunks and canopies, parked and moving vehicles;
//! * the **3D city model** the GNSS pipeline predicts NLOS satellites from
//!   (`model_scene`): buildings only, like a PLATEAU LOD1/LOD2 model;
//! * the **LiDAR prior map** (`map_scene`): buildings and permanent street
//!   furniture, surveyed once; vehicles are never in it.

use rne_gnss::{FoliageVolume, GnssScene, SceneBox, SurfaceMaterial};
use rne_math::Vec3;

/// Office building footprint `[x0, y0, x1, y1]`.
pub(crate) const OFFICE: [f64; 4] = [130.0, -12.0, 160.0, 12.0];
/// Height of the office walls.
const OFFICE_HEIGHT_M: f64 = 3.7;

/// Which layers an item belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Layer {
    /// In the city model, the LiDAR map and the world.
    Building,
    /// In the LiDAR map and the world (poles, trunks, benches, shelters).
    Furniture,
}

/// One static box with its render kind.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Item {
    pub(crate) scene_box: SceneBox,
    pub(crate) kind: &'static str,
    pub(crate) layer: Layer,
}

/// A vehicle driving along the street (or parked when `speed_m_s` is zero).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Vehicle {
    pub(crate) kind: &'static str,
    pub(crate) length_m: f64,
    pub(crate) width_m: f64,
    pub(crate) height_m: f64,
    pub(crate) lane_y_m: f64,
    pub(crate) x0_m: f64,
    pub(crate) speed_m_s: f64,
    pub(crate) color: usize,
}

/// Street stretch vehicles drive along before wrapping round.
const TRAFFIC_X: (f64, f64) = (41.0, 119.0);

impl Vehicle {
    /// Centre x at time `t`.
    pub(crate) fn x_at(&self, t: f64) -> f64 {
        if self.speed_m_s == 0.0 {
            return self.x0_m;
        }
        let span = TRAFFIC_X.1 - TRAFFIC_X.0 - self.length_m;
        let start = TRAFFIC_X.0 + 0.5 * self.length_m;
        start + (self.x0_m - start + self.speed_m_s * t).rem_euclid(span)
    }

    /// The vehicle's box at time `t`.
    pub(crate) fn scene_box(&self, t: f64) -> SceneBox {
        let x = self.x_at(t);
        SceneBox::new(
            Vec3::new(
                x - 0.5 * self.length_m,
                self.lane_y_m - 0.5 * self.width_m,
                0.0,
            ),
            Vec3::new(
                x + 0.5 * self.length_m,
                self.lane_y_m + 0.5 * self.width_m,
                self.height_m,
            ),
            SurfaceMaterial::METAL,
        )
    }
}

/// The whole site.
#[derive(Clone, Debug)]
pub(crate) struct City {
    pub(crate) items: Vec<Item>,
    pub(crate) canopies: Vec<FoliageVolume>,
    pub(crate) vehicles: Vec<Vehicle>,
}

fn building(
    items: &mut Vec<Item>,
    kind: &'static str,
    [x0, y0, x1, y1]: [f64; 4],
    (z0, z1): (f64, f64),
) {
    let material = match kind {
        "glass" => SurfaceMaterial::GLASS,
        "metal" => SurfaceMaterial::METAL,
        _ => SurfaceMaterial::CONCRETE,
    };
    items.push(Item {
        scene_box: SceneBox::new(Vec3::new(x0, y0, z0), Vec3::new(x1, y1, z1), material),
        kind,
        layer: Layer::Building,
    });
}

fn furniture(
    items: &mut Vec<Item>,
    kind: &'static str,
    x: f64,
    y: f64,
    half_x: f64,
    half_y: f64,
    height: f64,
) {
    let material = if kind == "shelter" {
        SurfaceMaterial::GLASS
    } else {
        SurfaceMaterial::ABSORBER
    };
    items.push(Item {
        scene_box: SceneBox::new(
            Vec3::new(x - half_x, y - half_y, 0.0),
            Vec3::new(x + half_x, y + half_y, height),
            material,
        ),
        kind,
        layer: Layer::Furniture,
    });
}

impl City {
    /// Builds the site.
    pub(crate) fn build() -> Self {
        let mut items = Vec::new();
        let mut canopies = Vec::new();
        Self::street_buildings(&mut items);
        Self::office(&mut items);
        Self::street_furniture(&mut items, &mut canopies);
        Self::plaza(&mut items, &mut canopies);
        Self {
            items,
            canopies,
            vehicles: Self::traffic(),
        }
    }

    fn street_buildings(items: &mut Vec<Item>) {
        // North side of the 30 m street: a podium with a glass tower, a
        // concrete block with a penthouse, and the 40 m glass tower that
        // carries the RTK base station.
        building(items, "concrete", [40.0, 15.0, 65.0, 40.0], (0.0, 10.0));
        building(items, "glass", [43.0, 18.0, 62.0, 38.0], (10.0, 48.0));
        building(items, "concrete", [70.0, 15.0, 95.0, 35.0], (0.0, 25.0));
        building(items, "concrete", [76.0, 20.0, 89.0, 31.0], (25.0, 29.0));
        building(items, "glass", [100.0, 15.0, 120.0, 40.0], (0.0, 40.0));
        // South side: concrete, a podium with a glass tower, metal cladding.
        building(items, "concrete", [40.0, -40.0, 60.0, -15.0], (0.0, 28.0));
        building(items, "concrete", [63.0, -38.0, 90.0, -15.0], (0.0, 9.0));
        building(items, "glass", [66.0, -35.0, 87.0, -18.0], (9.0, 50.0));
        building(items, "metal", [94.0, -40.0, 120.0, -15.0], (0.0, 30.0));
        // The blocks behind, and low buildings around the plaza.
        for (kind, x0, y0, x1, y1, h) in [
            ("concrete", 40.0, 46.0, 60.0, 72.0, 30.0),
            ("glass", 63.0, 44.0, 90.0, 75.0, 22.0),
            ("glass", 93.0, 46.0, 120.0, 70.0, 55.0),
            ("concrete", 40.0, -80.0, 62.0, -46.0, 20.0),
            ("metal", 65.0, -78.0, 92.0, -44.0, 35.0),
            ("concrete", 95.0, -80.0, 120.0, -46.0, 26.0),
            ("concrete", -14.0, 20.0, 34.0, 40.0, 12.0),
            ("glass", -14.0, -40.0, 34.0, -22.0, 15.0),
            ("concrete", -45.0, -30.0, -24.0, 30.0, 18.0),
            ("concrete", 128.0, 18.0, 160.0, 40.0, 14.0),
            ("glass", 128.0, -40.0, 160.0, -20.0, 22.0),
        ] {
            building(items, kind, [x0, y0, x1, y1], (0.0, h));
        }
    }

    fn office(items: &mut Vec<Item>) {
        let [x0, y0, x1, y1] = OFFICE;
        let (wall, h) = (0.3, OFFICE_HEIGHT_M);
        for (a, b, c, d) in [
            (x0, y0, x0 + wall, -1.5),
            (x0, 1.5, x0 + wall, y1),
            (x1 - wall, y0, x1, y1),
            (x0, y0, x1, y0 + wall),
            (x0, y1 - wall, x1, y1),
            (x0 + wall, 3.9, 148.0, 4.1),
            (152.0, 3.9, x1 - wall, 4.1),
            (136.0, -4.1, x1 - wall, -3.9),
        ] {
            building(items, "office", [a, b, c, d], (0.0, h));
        }
        for (a, b) in [(139.7, -2.8), (147.7, -2.8)] {
            building(items, "pillar", [a, b, a + 0.6, b + 0.6], (0.0, h));
        }
        building(items, "roof", [x0, y0, x1, y1], (h, h + 0.3));
        for (a, b, c, d) in [
            (136.0, 8.5, 139.0, 10.0),
            (143.0, 9.0, 146.0, 11.0),
            (155.0, 5.0, 158.0, 6.5),
        ] {
            building(items, "desk", [a, b, c, d], (0.0, 0.75));
        }
    }

    fn street_furniture(items: &mut Vec<Item>, canopies: &mut Vec<FoliageVolume>) {
        for side in [-1.0, 1.0] {
            // Lamp posts at the kerb and street trees between them.
            let mut x = 42.0;
            while x < 120.0 {
                furniture(items, "lamp", x, side * 9.4, 0.1, 0.1, 7.5);
                x += 14.0;
            }
            let mut x = 49.0;
            while x < 118.0 {
                // No trees in the cross streets.
                if !(60.0..71.0).contains(&x) && !(89.0..101.0).contains(&x) {
                    tree(items, canopies, x, side * 9.9, 2.2);
                }
                x += 14.0;
            }
            // Traffic signals at the cross streets.
            for x in [62.0, 71.0, 92.0, 101.0] {
                furniture(items, "signal", x, side * 9.3, 0.12, 0.12, 4.6);
            }
        }
        furniture(items, "shelter", 80.0, 11.6, 2.0, 0.75, 2.6);
    }

    fn plaza(items: &mut Vec<Item>, canopies: &mut Vec<FoliageVolume>) {
        for x in [-6.0, 4.0, 14.0, 24.0] {
            tree(items, canopies, x, 11.5, 2.6);
            tree(items, canopies, x, -12.5, 2.6);
        }
        for (x, y) in [(9.0, 9.5), (19.0, 9.5), (-1.0, -10.0)] {
            furniture(items, "bench", x, y, 0.9, 0.3, 0.45);
        }
        for (x, y) in [(-2.0, 13.5), (30.0, 13.0), (30.0, -14.0), (-10.0, -6.0)] {
            furniture(items, "lamp", x, y, 0.1, 0.1, 6.0);
        }
        furniture(items, "kiosk", -9.0, 4.0, 1.5, 1.5, 3.0);
    }

    fn traffic() -> Vec<Vehicle> {
        let car = |lane_y_m, x0_m, speed_m_s, color| Vehicle {
            kind: "car",
            length_m: 4.5,
            width_m: 1.8,
            height_m: 1.5,
            lane_y_m,
            x0_m,
            speed_m_s,
            color,
        };
        let bus = |lane_y_m, x0_m, speed_m_s| Vehicle {
            kind: "bus",
            length_m: 12.0,
            width_m: 2.5,
            height_m: 3.2,
            lane_y_m,
            x0_m,
            speed_m_s,
            color: 0,
        };
        vec![
            // Parked along both kerbs, and a bus at the stop.
            car(-7.5, 46.0, 0.0, 1),
            car(-7.5, 52.0, 0.0, 2),
            car(-7.5, 76.0, 0.0, 3),
            car(-7.5, 105.0, 0.0, 4),
            car(-7.5, 111.0, 0.0, 0),
            car(7.5, 55.0, 0.0, 5),
            car(7.5, 107.0, 0.0, 2),
            bus(7.5, 81.0, 0.0),
            // Moving traffic, eastbound south of the centre line.
            car(-3.0, 45.0, 9.0, 0),
            car(-3.0, 72.0, 8.0, 4),
            car(-3.0, 98.0, 9.5, 5),
            bus(-3.0, 60.0, 7.0),
            // Westbound.
            car(3.0, 60.0, -8.5, 3),
            car(3.0, 92.0, -9.0, 1),
            car(3.0, 115.0, -8.0, 2),
        ]
    }

    fn scene_of(&self, layers: &[Layer]) -> GnssScene {
        let mut scene = GnssScene::open_sky();
        for item in &self.items {
            if layers.contains(&item.layer) {
                scene.add_box(item.scene_box);
            }
        }
        scene
    }

    /// The 3D city model: buildings only.
    pub(crate) fn model_scene(&self) -> GnssScene {
        self.scene_of(&[Layer::Building])
    }

    /// The LiDAR prior map's geometry: buildings and street furniture.
    pub(crate) fn map_scene(&self) -> GnssScene {
        self.scene_of(&[Layer::Building, Layer::Furniture])
    }

    /// Everything the sensors see at time `t`.
    pub(crate) fn world_scene(&self, t: f64) -> GnssScene {
        let mut scene = self.map_scene();
        for vehicle in &self.vehicles {
            scene.add_box(vehicle.scene_box(t));
        }
        scene.foliage.clone_from(&self.canopies);
        scene
    }
}

/// A street tree: a trunk (an obstacle for LiDAR and GNSS) and a canopy
/// (foliage that attenuates GNSS by about 1 dB per metre).
fn tree(items: &mut Vec<Item>, canopies: &mut Vec<FoliageVolume>, x: f64, y: f64, radius: f64) {
    furniture(items, "trunk", x, y, 0.18, 0.18, 3.2);
    canopies.push(FoliageVolume {
        min_m: Vec3::new(x - radius, y - radius, 3.0),
        max_m: Vec3::new(x + radius, y + radius, 3.0 + 2.0 * radius),
        loss_db_per_m: 1.0,
    });
}
