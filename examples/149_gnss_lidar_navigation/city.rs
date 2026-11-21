//! The site: a city district laid out as a street grid.
//!
//! An east–west main street (two lanes each way, kerbside parking,
//! sidewalks) runs through two signalised crossroads with north–south cross
//! streets. Buildings stand on the block property lines; a public plaza fills
//! part of the south-west block and a single-storey office sits back from the
//! street behind a forecourt in the south-east block. Traffic follows the
//! signals: cars stop at the stop lines on red and keep their distance.
//!
//! The same geometry serves three purposes, as on a real deployment:
//!
//! * the **world** the sensors see (`world_scene`): buildings, street
//!   furniture, tree trunks and canopies, parked and moving vehicles;
//! * the **3D city model** the GNSS pipeline predicts NLOS satellites from
//!   (`model_scene`): buildings only, like a PLATEAU LOD1/LOD2 model;
//! * the **LiDAR prior map** (`map_scene`): buildings and permanent street
//!   furniture, surveyed once; vehicles are never in it.
//!
//! `ground` lists the road surface patches (carriageways, sidewalks,
//! markings) that the 2D figures and the 3D renderer draw.

use rne_gnss::{FoliageVolume, GnssScene, SceneBox, SurfaceMaterial};
use rne_math::Vec3;

/// Office building footprint `[x0, y0, x1, y1]` (south-east block).
pub(crate) const OFFICE: [f64; 4] = [140.0, -45.0, 170.0, -21.0];
/// The office door: a gap in the north wall.
const OFFICE_DOOR_X: (f64, f64) = (153.5, 156.5);
/// Height of the office walls.
const OFFICE_HEIGHT_M: f64 = 3.7;
/// RTK base-station antenna: on the roof of the tower north of the street.
pub(crate) const BASE_ENU_M: Vec3 = Vec3::new(93.0, 31.0, 46.5);

/// Main street: carriageway `|y| < 9`, sidewalks to `|y| = 15`.
pub(crate) const MAIN_HALF_ROAD_M: f64 = 9.0;
/// Main street right-of-way half width (property line).
pub(crate) const MAIN_HALF_ROW_M: f64 = 15.0;
/// Extent of the main street along x.
const MAIN_X: (f64, f64) = (-80.0, 240.0);
/// Cross-street centre lines (x).
pub(crate) const CROSS_STREETS_X: [f64; 2] = [68.0, 118.0];
/// Cross street: carriageway `|x - c| < 5`, sidewalks to `|x - c| = 9`.
pub(crate) const CROSS_HALF_ROAD_M: f64 = 5.0;
const CROSS_HALF_ROW_M: f64 = 9.0;
/// Extent of the cross streets along y.
const CROSS_Y: (f64, f64) = (-120.0, 120.0);

/// Signal cycle in seconds.
const SIGNAL_CYCLE_S: f64 = 50.0;
/// Main-street green ends (then yellow, all-red).
const MAIN_GREEN_END_S: f64 = 32.0;
const CROSS_GREEN: (f64, f64) = (37.0, 47.0);
/// Pedestrians crossing a cross street (along the main street) may start
/// only this early in the main-street green, to finish before it ends.
const WALK_START_WINDOW_S: f64 = 14.0;

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

/// A flat ground patch `[x0, y0, x1, y1]` of a given kind.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Patch {
    pub(crate) rect: [f64; 4],
    pub(crate) kind: &'static str,
}

/// Direction of travel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Heading {
    East,
    West,
    North,
    South,
}

impl Heading {
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::East => "e",
            Self::West => "w",
            Self::North => "n",
            Self::South => "s",
        }
    }

    fn along_x(self) -> bool {
        matches!(self, Self::East | Self::West)
    }

    fn sign(self) -> f64 {
        match self {
            Self::East | Self::North => 1.0,
            Self::West | Self::South => -1.0,
        }
    }
}

/// A vehicle: moving in a lane, or parked.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Vehicle {
    pub(crate) kind: &'static str,
    pub(crate) length_m: f64,
    pub(crate) width_m: f64,
    pub(crate) height_m: f64,
    pub(crate) heading: Heading,
    /// Lane centre (y for east/west, x for north/south).
    pub(crate) lane_m: f64,
    /// Centre position along the direction of travel's axis.
    pub(crate) position_m: f64,
    pub(crate) speed_m_s: f64,
    pub(crate) max_speed_m_s: f64,
    pub(crate) parked: bool,
    pub(crate) color: usize,
}

impl Vehicle {
    /// The vehicle's box.
    pub(crate) fn scene_box(&self) -> SceneBox {
        let (half_l, half_w) = (0.5 * self.length_m, 0.5 * self.width_m);
        let (lo, hi) = if self.heading.along_x() {
            (
                Vec3::new(self.position_m - half_l, self.lane_m - half_w, 0.0),
                Vec3::new(
                    self.position_m + half_l,
                    self.lane_m + half_w,
                    self.height_m,
                ),
            )
        } else {
            (
                Vec3::new(self.lane_m - half_w, self.position_m - half_l, 0.0),
                Vec3::new(
                    self.lane_m + half_w,
                    self.position_m + half_l,
                    self.height_m,
                ),
            )
        };
        SceneBox::new(lo, hi, SurfaceMaterial::METAL)
    }

    /// Stop lines ahead of this vehicle's direction, as positions along its
    /// axis (front bumper stops there).
    fn stop_lines(&self) -> Vec<f64> {
        match self.heading {
            Heading::East => CROSS_STREETS_X
                .iter()
                .map(|c| c - CROSS_HALF_ROW_M - 0.5)
                .collect(),
            Heading::West => CROSS_STREETS_X
                .iter()
                .map(|c| c + CROSS_HALF_ROW_M + 0.5)
                .collect(),
            Heading::South => vec![MAIN_HALF_ROW_M + 0.5],
            Heading::North => vec![-MAIN_HALF_ROW_M - 0.5],
        }
    }

    fn range(&self) -> (f64, f64) {
        if self.heading.along_x() {
            MAIN_X
        } else {
            CROSS_Y
        }
    }
}

/// Signal state at time `t`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Signals {
    /// Main-street traffic may go.
    pub(crate) main_green: bool,
    /// Cross-street traffic may go.
    pub(crate) cross_green: bool,
    /// Pedestrians may start crossing a cross street along the main street.
    pub(crate) walk_along_main: bool,
}

/// The signal plan shared by both crossroads.
pub(crate) fn signals_at(t: f64) -> Signals {
    let phase = t.rem_euclid(SIGNAL_CYCLE_S);
    Signals {
        main_green: phase < MAIN_GREEN_END_S,
        cross_green: (CROSS_GREEN.0..CROSS_GREEN.1).contains(&phase),
        walk_along_main: phase < WALK_START_WINDOW_S,
    }
}

/// The whole site.
#[derive(Clone, Debug)]
pub(crate) struct City {
    pub(crate) items: Vec<Item>,
    pub(crate) canopies: Vec<FoliageVolume>,
    pub(crate) vehicles: Vec<Vehicle>,
    pub(crate) ground: Vec<Patch>,
}

fn building(items: &mut Vec<Item>, kind: &'static str, rect: [f64; 4], (z0, z1): (f64, f64)) {
    let material = match kind {
        "glass" => SurfaceMaterial::GLASS,
        "metal" => SurfaceMaterial::METAL,
        _ => SurfaceMaterial::CONCRETE,
    };
    items.push(Item {
        scene_box: SceneBox::new(
            Vec3::new(rect[0], rect[1], z0),
            Vec3::new(rect[2], rect[3], z1),
            material,
        ),
        kind,
        layer: Layer::Building,
    });
}

fn furniture(
    items: &mut Vec<Item>,
    kind: &'static str,
    (x, y): (f64, f64),
    (half_x, half_y): (f64, f64),
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

/// Whether `x` lies within a crossroads (cross-street right-of-way).
fn at_crossroads(x: f64, margin: f64) -> bool {
    CROSS_STREETS_X
        .iter()
        .any(|c| (x - c).abs() < CROSS_HALF_ROW_M + margin)
}

impl City {
    /// Builds the district.
    pub(crate) fn build() -> Self {
        let mut items = Vec::new();
        let mut canopies = Vec::new();
        Self::blocks(&mut items);
        Self::office(&mut items);
        Self::street_furniture(&mut items, &mut canopies);
        Self::plaza(&mut items, &mut canopies);
        Self {
            items,
            canopies,
            vehicles: Self::traffic(),
            ground: Self::ground(),
        }
    }

    /// Buildings on the block property lines.
    fn blocks(items: &mut Vec<Item>) {
        let towers: [(&str, [f64; 4], (f64, f64)); 25] = [
            // North-west block.
            ("concrete", [-80.0, 15.0, -2.0, 60.0], (0.0, 16.0)),
            ("glass", [0.0, 15.0, 30.0, 48.0], (0.0, 30.0)),
            ("concrete", [32.0, 15.0, 59.0, 50.0], (0.0, 10.0)),
            ("glass", [35.0, 18.0, 56.0, 47.0], (10.0, 42.0)),
            ("concrete", [-80.0, 62.0, 59.0, 120.0], (0.0, 20.0)),
            // North-centre block: podium and the tower with the base station.
            ("concrete", [77.0, 15.0, 109.0, 50.0], (0.0, 12.0)),
            ("glass", [80.0, 18.0, 106.0, 44.0], (12.0, 45.0)),
            ("concrete", [77.0, 54.0, 109.0, 120.0], (0.0, 24.0)),
            // North-east block.
            ("metal", [127.0, 15.0, 165.0, 48.0], (0.0, 26.0)),
            ("concrete", [167.0, 15.0, 240.0, 55.0], (0.0, 18.0)),
            ("glass", [127.0, 57.0, 240.0, 120.0], (0.0, 30.0)),
            // South-west block around the plaza.
            ("concrete", [-80.0, -60.0, -2.0, -15.0], (0.0, 14.0)),
            ("glass", [0.0, -110.0, 59.0, -64.0], (0.0, 22.0)),
            ("concrete", [-80.0, -120.0, -2.0, -62.0], (0.0, 18.0)),
            // South-centre block: podium and glass tower, metal behind.
            ("concrete", [77.0, -50.0, 109.0, -15.0], (0.0, 9.0)),
            ("glass", [80.0, -47.0, 106.0, -18.0], (9.0, 50.0)),
            ("metal", [77.0, -120.0, 109.0, -54.0], (0.0, 20.0)),
            // South-east block: the office behind its forecourt, between
            // neighbours on the property line.
            ("concrete", [127.0, -60.0, 138.0, -15.0], (0.0, 22.0)),
            ("metal", [172.0, -60.0, 240.0, -15.0], (0.0, 30.0)),
            ("concrete", [127.0, -120.0, 240.0, -62.0], (0.0, 16.0)),
            ("concrete", [140.0, -60.0, 170.0, -48.0], (0.0, 12.0)),
            // Far blocks across the cross streets (skyline only).
            ("concrete", [-80.0, -200.0, 59.0, -124.0], (0.0, 25.0)),
            ("glass", [77.0, -200.0, 109.0, -124.0], (0.0, 38.0)),
            ("concrete", [-80.0, 124.0, 59.0, 200.0], (0.0, 28.0)),
            ("metal", [127.0, 124.0, 240.0, 200.0], (0.0, 34.0)),
        ];
        for (kind, rect, heights) in towers {
            building(items, kind, rect, heights);
        }
    }

    fn office(items: &mut Vec<Item>) {
        let [x0, y0, x1, y1] = OFFICE;
        let (wall, h) = (0.3, OFFICE_HEIGHT_M);
        let (d0, d1) = OFFICE_DOOR_X;
        for rect in [
            [x0, y1 - wall, d0, y1],
            [d1, y1 - wall, x1, y1],
            [x0, y0, x0 + wall, y1],
            [x1 - wall, y0, x1, y1],
            [x0, y0, x1, y0 + wall],
            // A partition with a 4 m gap between the north and south rooms.
            [x0 + wall, -33.1, 152.0, -32.9],
            [156.0, -33.1, x1 - wall, -32.9],
        ] {
            building(items, "office", rect, (0.0, h));
        }
        for (a, b) in [(146.0, -27.3), (164.0, -27.3)] {
            building(items, "pillar", [a, b, a + 0.6, b + 0.6], (0.0, h));
        }
        building(items, "roof", OFFICE, (h, h + 0.3));
        for rect in [
            [143.0, -42.5, 146.0, -41.0],
            [160.0, -43.0, 163.0, -41.5],
            [165.0, -25.0, 168.0, -23.5],
            [142.0, -25.0, 145.0, -23.5],
        ] {
            building(items, "desk", rect, (0.0, 0.75));
        }
    }

    fn street_furniture(items: &mut Vec<Item>, canopies: &mut Vec<FoliageVolume>) {
        for side in [-1.0, 1.0] {
            // Main street: lamp posts at the kerb, street trees between,
            // none inside the crossroads.
            let mut x = -60.0;
            while x < 220.0 {
                if !at_crossroads(x, 2.0) {
                    furniture(items, "lamp", (x, side * 9.4), (0.1, 0.1), 7.5);
                }
                if !at_crossroads(x + 8.0, 3.5) {
                    tree(items, canopies, x + 8.0, side * 9.9, 2.2);
                }
                x += 16.0;
            }
            // Cross streets: lamp posts along both kerbs.
            for c in CROSS_STREETS_X {
                for y in [-100.0, -76.0, -52.0, -28.0, 28.0, 52.0, 76.0, 100.0] {
                    furniture(items, "lamp", (c + side * 5.4, y), (0.1, 0.1), 7.5);
                }
                // Signals on the four corners.
                for ys in [-1.0, 1.0] {
                    furniture(
                        items,
                        "signal",
                        (c + side * 5.6, ys * 9.6),
                        (0.12, 0.12),
                        4.6,
                    );
                }
            }
        }
        // A bus stop on the north sidewalk.
        furniture(items, "shelter", (93.0, 11.9), (2.2, 0.75), 2.6);
        // Planters in front of the shops, irregularly spaced as real
        // frontages are (they also keep the LiDAR scan from looking the same
        // every lamp spacing along a flat facade).
        let fronts = [
            (-1.0, -78.0, -4.0),
            (-1.0, 79.0, 107.0),
            (-1.0, 128.0, 136.0),
            (-1.0, 174.0, 238.0),
            (1.0, -78.0, -4.0),
            (1.0, 2.0, 57.0),
            (1.0, 79.0, 107.0),
            (1.0, 129.0, 238.0),
        ];
        let gaps = [7.0, 11.5, 5.5, 9.0, 13.0, 6.5, 10.0];
        let mut k = 0;
        for (side, x0, x1) in fronts {
            let mut x = x0 + 2.0;
            while x < x1 {
                furniture(items, "planter", (x, side * 14.6), (0.8, 0.3), 0.6);
                x += gaps[k % gaps.len()];
                k += 1;
            }
        }
    }

    fn plaza(items: &mut Vec<Item>, canopies: &mut Vec<FoliageVolume>) {
        for (x, y) in [
            (6.0, -24.0),
            (6.0, -38.0),
            (6.0, -52.0),
            (44.0, -24.0),
            (44.0, -38.0),
            (52.0, -52.0),
            (16.0, -56.0),
            (34.0, -56.0),
        ] {
            tree(items, canopies, x, y, 2.6);
        }
        for (x, y) in [(12.0, -30.0), (12.0, -46.0), (40.0, -31.0), (40.0, -45.0)] {
            furniture(items, "bench", (x, y), (0.9, 0.3), 0.45);
        }
        for (x, y) in [(4.0, -18.0), (54.0, -18.0), (26.0, -58.0)] {
            furniture(items, "lamp", (x, y), (0.1, 0.1), 6.0);
        }
        furniture(items, "kiosk", (50.0, -42.0), (1.5, 1.5), 3.0);
    }

    fn traffic() -> Vec<Vehicle> {
        let vehicle =
            |kind: &'static str, heading, lane_m, position_m, max_speed_m_s, parked, color| {
                let (length_m, width_m, height_m) = if kind == "bus" {
                    (12.0, 2.5, 3.2)
                } else {
                    (4.5, 1.8, 1.5)
                };
                Vehicle {
                    kind,
                    length_m,
                    width_m,
                    height_m,
                    heading,
                    lane_m,
                    position_m,
                    speed_m_s: if parked { 0.0 } else { max_speed_m_s },
                    max_speed_m_s,
                    parked,
                    color,
                }
            };
        let mut vehicles = Vec::new();
        // Parked along both kerbs, clear of the crossroads, and a bus at the
        // stop (westbound side, by the shelter).
        for (x, color) in [
            (20.0, 1),
            (26.0, 3),
            (42.0, 2),
            (86.0, 4),
            (100.0, 0),
            (140.0, 5),
            (146.0, 2),
            (170.0, 1),
        ] {
            vehicles.push(vehicle("car", Heading::East, -7.75, x, 0.0, true, color));
        }
        for (x, color) in [(14.0, 4), (36.0, 0), (48.0, 5), (132.0, 3), (152.0, 1)] {
            vehicles.push(vehicle("car", Heading::West, 7.75, x, 0.0, true, color));
        }
        vehicles.push(vehicle("bus", Heading::West, 7.75, 93.0, 0.0, true, 0));
        // Moving traffic on the main street (two lanes each way).
        for (lane, x, v, kind, color) in [
            (-1.6, 10.0, 12.0, "car", 0),
            (-1.6, 60.0, 12.5, "car", 4),
            (-1.6, 150.0, 11.0, "car", 5),
            (-4.9, 30.0, 9.0, "bus", 0),
            (-4.9, 100.0, 10.5, "car", 2),
            (-4.9, 190.0, 10.0, "car", 3),
        ] {
            vehicles.push(vehicle(kind, Heading::East, lane, x, v, false, color));
        }
        for (lane, x, v, kind, color) in [
            (1.6, 200.0, 12.0, "car", 1),
            (1.6, 120.0, 11.5, "car", 3),
            (1.6, 20.0, 12.0, "car", 2),
            (4.9, 160.0, 9.0, "bus", 0),
            (4.9, 80.0, 10.0, "car", 5),
        ] {
            vehicles.push(vehicle(kind, Heading::West, lane, x, v, false, color));
        }
        // Cross-street traffic (right-hand driving).
        for c in CROSS_STREETS_X {
            vehicles.push(vehicle(
                "car",
                Heading::South,
                c - 2.5,
                60.0,
                10.0,
                false,
                1,
            ));
            vehicles.push(vehicle(
                "car",
                Heading::South,
                c - 2.5,
                -40.0,
                10.0,
                false,
                4,
            ));
            vehicles.push(vehicle(
                "car",
                Heading::North,
                c + 2.5,
                -70.0,
                10.0,
                false,
                2,
            ));
            vehicles.push(vehicle(
                "car",
                Heading::North,
                c + 2.5,
                30.0,
                10.0,
                false,
                0,
            ));
        }
        vehicles
    }

    /// Carriageways, sidewalks, plaza, forecourt and road markings.
    fn ground() -> Vec<Patch> {
        let mut g = Vec::new();
        let mut patch = |rect: [f64; 4], kind: &'static str| g.push(Patch { rect, kind });
        let (r, w) = (MAIN_HALF_ROAD_M, MAIN_HALF_ROW_M);
        patch([MAIN_X.0, -r, MAIN_X.1, r], "road");
        for c in CROSS_STREETS_X {
            patch(
                [
                    c - CROSS_HALF_ROAD_M,
                    CROSS_Y.0,
                    c + CROSS_HALF_ROAD_M,
                    CROSS_Y.1,
                ],
                "road",
            );
        }
        // Main-street sidewalks between the crossroads, and the cross-street
        // sidewalks outside the main street.
        let mut edges = vec![MAIN_X.0];
        for c in CROSS_STREETS_X {
            edges.extend([c - CROSS_HALF_ROAD_M, c + CROSS_HALF_ROAD_M]);
        }
        edges.push(MAIN_X.1);
        for pair in edges.chunks(2) {
            patch([pair[0], r, pair[1], w], "sidewalk");
            patch([pair[0], -w, pair[1], -r], "sidewalk");
        }
        for c in CROSS_STREETS_X {
            for (x0, x1) in [
                (c - CROSS_HALF_ROW_M, c - CROSS_HALF_ROAD_M),
                (c + CROSS_HALF_ROAD_M, c + CROSS_HALF_ROW_M),
            ] {
                patch([x0, CROSS_Y.0, x1, -r], "sidewalk");
                patch([x0, r, x1, CROSS_Y.1], "sidewalk");
            }
        }
        // Block interiors (paved courtyards and alleys between buildings).
        for (x0, x1) in [(MAIN_X.0, 59.0), (77.0, 109.0), (127.0, MAIN_X.1)] {
            patch([x0, w, x1, CROSS_Y.1], "block");
            patch([x0, CROSS_Y.0, x1, -w], "block");
        }
        patch([0.0, -62.0, 59.0, -15.0], "plaza");
        patch([138.0, -21.0, 172.0, -15.0], "forecourt");
        patch([OFFICE[0], OFFICE[1], OFFICE[2], OFFICE[3]], "floor");

        // Markings: centre line, lane dashes and parking lines, broken at the
        // crossroads; crossings on every leg; stop lines.
        let mut x = MAIN_X.0;
        while x < MAIN_X.1 {
            let x1 = x + 3.0;
            if !at_crossroads(x, 4.0) && !at_crossroads(x1, 4.0) {
                patch([x, -0.15, x1, -0.05], "paint");
                patch([x, 0.05, x1, 0.15], "paint");
                patch([x, -3.3, x1 - 1.0, -3.2], "paint");
                patch([x, 3.2, x1 - 1.0, 3.3], "paint");
                patch([x, -6.55, x1, -6.45], "paint");
                patch([x, 6.45, x1, 6.55], "paint");
            }
            x = x1;
        }
        for c in CROSS_STREETS_X {
            let mut y = CROSS_Y.0;
            while y < CROSS_Y.1 {
                if y.abs() > w + 4.0 {
                    patch([c - 0.08, y, c + 0.08, y + 2.0], "paint");
                }
                y += 4.0;
            }
            // Zebra crossings across the main street (west and east legs).
            for x0 in [c - CROSS_HALF_ROW_M + 0.5, c + CROSS_HALF_ROAD_M + 0.5] {
                let mut y = -r + 0.3;
                while y < r - 0.3 {
                    patch([x0, y, x0 + 3.0, y + 0.5], "paint");
                    y += 1.0;
                }
            }
            // Across the cross street (north and south legs), on the line of
            // the main-street sidewalks.
            for y0 in [-w + 1.5, r + 1.5] {
                let mut x = c - CROSS_HALF_ROAD_M + 0.3;
                while x < c + CROSS_HALF_ROAD_M - 0.3 {
                    patch([x, y0, x + 0.5, y0 + 3.0], "paint");
                    x += 1.0;
                }
            }
            patch(
                [
                    c - CROSS_HALF_ROW_M - 0.6,
                    -6.5,
                    c - CROSS_HALF_ROW_M - 0.3,
                    0.0,
                ],
                "paint",
            );
            patch(
                [
                    c + CROSS_HALF_ROW_M + 0.3,
                    0.0,
                    c + CROSS_HALF_ROW_M + 0.6,
                    6.5,
                ],
                "paint",
            );
            patch([c - CROSS_HALF_ROAD_M, w + 0.3, c, w + 0.6], "paint");
            patch([c, -w - 0.6, c + CROSS_HALF_ROAD_M, -w - 0.3], "paint");
        }
        g
    }

    /// Advances the traffic by `dt` seconds at time `t`: cars keep a safe
    /// gap, and stop at the stop line when their signal is not green.
    pub(crate) fn step_traffic(&mut self, t: f64, dt: f64) {
        let signals = signals_at(t);
        let snapshot = self.vehicles.clone();
        for (index, vehicle) in self.vehicles.iter_mut().enumerate() {
            if vehicle.parked {
                continue;
            }
            let sign = vehicle.heading.sign();
            let front = vehicle.position_m + sign * 0.5 * vehicle.length_m;
            let mut limit = vehicle.max_speed_m_s;
            // Car following: the nearest vehicle ahead in the same lane.
            for (other_index, other) in snapshot.iter().enumerate() {
                if other_index == index
                    || other.heading != vehicle.heading
                    || (other.lane_m - vehicle.lane_m).abs() > 0.5
                {
                    continue;
                }
                let rear = other.position_m - sign * 0.5 * other.length_m;
                let gap = sign * (rear - front);
                if gap > 0.0 && gap < 60.0 {
                    limit = limit.min(((gap - 2.5) / 1.2).max(0.0));
                }
            }
            // Signals: stop at a stop line ahead when not green.
            let green = if vehicle.heading.along_x() {
                signals.main_green
            } else {
                signals.cross_green
            };
            if !green {
                for line in vehicle.stop_lines() {
                    let distance = sign * (line - front);
                    if distance > -0.5 && distance < 45.0 {
                        let braking = (2.0 * 4.0 * (distance - 0.3).max(0.0)).sqrt();
                        // Too close to stop comfortably: carry on through.
                        if vehicle.speed_m_s * vehicle.speed_m_s / (2.0 * 6.0) < distance {
                            limit = limit.min(braking);
                        }
                    }
                }
            }
            let accel = (limit - vehicle.speed_m_s).clamp(-6.0 * dt, 2.0 * dt);
            vehicle.speed_m_s = (vehicle.speed_m_s + accel).max(0.0);
            vehicle.position_m += sign * vehicle.speed_m_s * dt;
            let (lo, hi) = vehicle.range();
            if vehicle.position_m > hi {
                vehicle.position_m = lo;
            } else if vehicle.position_m < lo {
                vehicle.position_m = hi;
            }
        }
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

    /// Everything the sensors see with the given vehicles.
    pub(crate) fn world_scene_with(&self, vehicles: &[Vehicle]) -> GnssScene {
        let mut scene = self.map_scene();
        for vehicle in vehicles {
            scene.add_box(vehicle.scene_box());
        }
        scene.foliage.clone_from(&self.canopies);
        scene
    }

    /// Everything the sensors see now.
    pub(crate) fn world_scene(&self) -> GnssScene {
        self.world_scene_with(&self.vehicles)
    }
}

/// A street tree: a trunk (an obstacle for LiDAR and GNSS) and a canopy
/// (foliage that attenuates GNSS by about 1 dB per metre).
fn tree(items: &mut Vec<Item>, canopies: &mut Vec<FoliageVolume>, x: f64, y: f64, radius: f64) {
    furniture(items, "trunk", (x, y), (0.18, 0.18), 3.2);
    canopies.push(FoliageVolume {
        min_m: Vec3::new(x - radius, y - radius, 3.0),
        max_m: Vec3::new(x + radius, y + radius, 3.0 + 2.0 * radius),
        loss_db_per_m: 1.0,
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cars_stop_on_red_and_never_overlap() {
        let mut city = City::build();
        let dt = 0.05;
        for step in 0..4000 {
            let t = step as f64 * dt;
            city.step_traffic(t, dt);
            let moving: Vec<_> = city.vehicles.iter().filter(|v| !v.parked).collect();
            for (i, a) in moving.iter().enumerate() {
                for b in &moving[i + 1..] {
                    if a.heading == b.heading && (a.lane_m - b.lane_m).abs() < 0.5 {
                        let gap =
                            (a.position_m - b.position_m).abs() - 0.5 * (a.length_m + b.length_m);
                        assert!(
                            gap > -0.2 || (a.position_m - b.position_m).abs() > 200.0,
                            "overlap at {t}"
                        );
                    }
                }
            }
            // No main-street car inside a crossroads while the cross street
            // has green (after the all-red clearance).
            if signals_at(t).cross_green && t.rem_euclid(SIGNAL_CYCLE_S) > CROSS_GREEN.0 + 3.0 {
                for v in &moving {
                    if v.heading.along_x() {
                        for c in CROSS_STREETS_X {
                            assert!(
                                (v.position_m - c).abs()
                                    > CROSS_HALF_ROAD_M + 0.5 * v.length_m - 0.5,
                                "car in the crossroads on cross green at {t}"
                            );
                        }
                    }
                }
            }
        }
    }
}
