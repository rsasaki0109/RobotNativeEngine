//! Generated two-room layouts for `--explore --layout-seed N`: the shell of
//! `unitree_go2_two_rooms` (outer walls, the partition at x = 2.4) with the
//! doorway moved and the furniture replaced by seeded boxes, so exploration is
//! tested beyond the one hand-dressed scene (`docs/PLAN_ACTIVE_SLAM.md` P3).
//!
//! Every layout keeps the Go2 able to reach all of its floor: furniture stands
//! against a wall or at least [`PASSAGE_M`] from walls and other furniture,
//! and nothing is placed around the start or in front of the doorway. The
//! tests check that with a flood fill over the floor.

use rne_world::WorldRandom;

/// Room extents in world `[x, z]` (inner faces of the walls).
pub(crate) const ROOM_A: [[f64; 2]; 2] = [[-2.4, -2.4], [2.35, 2.4]];
pub(crate) const ROOM_B: [[f64; 2]; 2] = [[2.45, -2.4], [7.0, 2.4]];
const PARTITION_X: f64 = 2.4;
const DOORWAY_M: f64 = 1.0;
/// Free width kept between furniture, and between furniture and walls,
/// unless it stands against the wall: the Go2 needs 0.84 m (2 × 0.42).
pub(crate) const PASSAGE_M: f64 = 1.0;
/// Nothing within this of the start (the origin) or of the doorway's centre.
const START_CLEAR_M: f64 = 1.2;
const DOORWAY_CLEAR_M: f64 = 1.2;
/// Furniture heights: crates, a sofa seat, a table, a shelf.
const HEIGHTS_M: [f64; 5] = [0.4, 0.5, 0.45, 0.75, 1.2];

/// A box of furniture in world `[x, z]`, standing on the floor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Furniture {
    pub(crate) min: [f64; 2],
    pub(crate) max: [f64; 2],
    pub(crate) height_m: f64,
}

impl Furniture {
    fn gap(&self, other: &Furniture) -> f64 {
        let dx = (other.min[0] - self.max[0]).max(self.min[0] - other.max[0]);
        let dz = (other.min[1] - self.max[1]).max(self.min[1] - other.max[1]);
        if dx > 0.0 && dz > 0.0 {
            dx.hypot(dz)
        } else {
            dx.max(dz)
        }
    }

    fn distance_to(&self, point: [f64; 2]) -> f64 {
        let dx = (self.min[0] - point[0])
            .max(point[0] - self.max[0])
            .max(0.0);
        let dz = (self.min[1] - point[1])
            .max(point[1] - self.max[1])
            .max(0.0);
        dx.hypot(dz)
    }
}

/// A generated layout.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Layout {
    /// Centre of the 1.0 m doorway along z.
    pub(crate) doorway_z: f64,
    pub(crate) furniture: Vec<Furniture>,
}

/// The layout for `seed`: always the same for the same seed.
pub(crate) fn generate(seed: u64) -> Layout {
    let mut rng = WorldRandom::new(seed ^ 0x6020_1a70_u64);
    let doorway_z = rng.uniform_f64(-1.6, 1.6);
    let mut furniture = Vec::new();
    for room in [ROOM_A, ROOM_B] {
        let wanted = 3 + rng.uniform_usize(2);
        let mut placed = 0;
        for _ in 0..400 {
            if placed == wanted {
                break;
            }
            let candidate = propose(&mut rng, room);
            if fits(&candidate, room, doorway_z, &furniture) {
                furniture.push(candidate);
                placed += 1;
            }
        }
    }
    Layout {
        doorway_z,
        furniture,
    }
}

/// A random box in `room`: against one of its walls half the time.
fn propose(rng: &mut WorldRandom, room: [[f64; 2]; 2]) -> Furniture {
    let mut size = [rng.uniform_f64(0.4, 1.6), rng.uniform_f64(0.4, 0.9)];
    if rng.uniform_usize(2) == 0 {
        size.swap(0, 1);
    }
    let height_m = HEIGHTS_M[rng.uniform_usize(HEIGHTS_M.len())];
    let mut min = [
        rng.uniform_f64(room[0][0], room[1][0] - size[0]),
        rng.uniform_f64(room[0][1], room[1][1] - size[1]),
    ];
    if rng.uniform_usize(2) == 0 {
        match rng.uniform_usize(4) {
            0 => min[0] = room[0][0],
            1 => min[0] = room[1][0] - size[0],
            2 => min[1] = room[0][1],
            _ => min[1] = room[1][1] - size[1],
        }
    }
    Furniture {
        min,
        max: [min[0] + size[0], min[1] + size[1]],
        height_m,
    }
}

/// Inside the room, clear of the start, the doorway and the other furniture,
/// and either against each wall or a passage away from it.
fn fits(candidate: &Furniture, room: [[f64; 2]; 2], doorway_z: f64, placed: &[Furniture]) -> bool {
    let inside = candidate.min[0] >= room[0][0] - 1e-9
        && candidate.min[1] >= room[0][1] - 1e-9
        && candidate.max[0] <= room[1][0] + 1e-9
        && candidate.max[1] <= room[1][1] + 1e-9;
    let wall_gaps = [
        candidate.min[0] - room[0][0],
        room[1][0] - candidate.max[0],
        candidate.min[1] - room[0][1],
        room[1][1] - candidate.max[1],
    ];
    let walls_ok = wall_gaps.iter().all(|g| *g < 1e-9 || *g >= PASSAGE_M);
    inside
        && walls_ok
        && candidate.distance_to([0.0, 0.0]) >= START_CLEAR_M
        && candidate.distance_to([PARTITION_X, doorway_z]) >= DOORWAY_CLEAR_M
        && placed.iter().all(|other| candidate.gap(other) >= PASSAGE_M)
}

/// The scene file for `layout`, with the robot at `robot_path`.
pub(crate) fn scene_toml(layout: &Layout, robot_path: &str) -> String {
    let mut out = format!(
        "[world]\ngravity_m_s2 = [0.0, -9.81, 0.0]\nseed = 361\n\n[ground]\nenabled = true\n\n\
         [[robots]]\npath = \"{robot_path}\"\n"
    );
    let mut push = |name: &str, centre: [f64; 3], size: [f64; 3], color: [f64; 3]| {
        out.push_str(&format!(
            "\n[[objects]]\nname = \"{name}\"\ntranslation_m = [{:.4}, {:.4}, {:.4}]\n\
             visual = {{ shape = \"box\", size_m = [{:.4}, {:.4}, {:.4}], color_rgba = [{:.2}, {:.2}, {:.2}, 1.0] }}\n\
             collision = {{ shape = \"box\", size_m = [{:.4}, {:.4}, {:.4}] }}\nbody_type = \"fixed\"\n",
            centre[0], centre[1], centre[2], size[0], size[1], size[2], color[0], color[1], color[2],
            size[0], size[1], size[2],
        ));
    };
    let wall = [0.68, 0.70, 0.75];
    push("wall_west", [-2.45, 0.5, 0.0], [0.1, 1.0, 5.0], wall);
    push("wall_east", [7.05, 0.5, 0.0], [0.1, 1.0, 5.0], wall);
    push("wall_north", [2.3, 0.5, -2.45], [9.6, 1.0, 0.1], wall);
    push("wall_south", [2.3, 0.5, 2.45], [9.6, 1.0, 0.1], wall);
    let (door_lo, door_hi) = (
        layout.doorway_z - 0.5 * DOORWAY_M,
        layout.doorway_z + 0.5 * DOORWAY_M,
    );
    let north = door_lo + 2.4;
    let south = 2.4 - door_hi;
    push(
        "partition_north",
        [PARTITION_X, 0.5, -2.4 + 0.5 * north],
        [0.1, 1.0, north],
        wall,
    );
    push(
        "partition_south",
        [PARTITION_X, 0.5, door_hi + 0.5 * south],
        [0.1, 1.0, south],
        wall,
    );
    for (index, item) in layout.furniture.iter().enumerate() {
        let size = [
            item.max[0] - item.min[0],
            item.height_m,
            item.max[1] - item.min[1],
        ];
        let centre = [
            0.5 * (item.min[0] + item.max[0]),
            0.5 * item.height_m,
            0.5 * (item.min[1] + item.max[1]),
        ];
        push(
            &format!("furniture_{index}"),
            centre,
            size,
            [0.55, 0.42 + 0.04 * index as f64, 0.30],
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Floor cells (5 cm) where a disc of the Go2's inscribed radius fits,
    /// flood-filled from the start: every such cell must be reached.
    fn unreachable_floor(layout: &Layout) -> usize {
        const CELL: f64 = 0.05;
        const RADIUS: f64 = 0.42;
        let (x0, z0) = (-2.4, -2.4);
        let (nx, nz) = ((9.4 / CELL) as usize, (4.8 / CELL) as usize);
        let centre =
            |i: usize, j: usize| [x0 + (i as f64 + 0.5) * CELL, z0 + (j as f64 + 0.5) * CELL];
        let door = (
            layout.doorway_z - 0.5 * DOORWAY_M,
            layout.doorway_z + 0.5 * DOORWAY_M,
        );
        let free = |p: [f64; 2]| {
            let in_room = |r: [[f64; 2]; 2]| {
                p[0] - RADIUS >= r[0][0]
                    && p[0] + RADIUS <= r[1][0]
                    && p[1] - RADIUS >= r[0][1]
                    && p[1] + RADIUS <= r[1][1]
            };
            let in_doorway = (p[0] - PARTITION_X).abs() <= 0.5
                && p[1] - RADIUS >= door.0
                && p[1] + RADIUS <= door.1;
            (in_room(ROOM_A) || in_room(ROOM_B) || in_doorway)
                && layout.furniture.iter().all(|f| f.distance_to(p) >= RADIUS)
        };
        let mut seen = vec![false; nx * nz];
        let start = ((2.4 / CELL) as usize, (2.4 / CELL) as usize);
        let mut stack = vec![start];
        seen[start.1 * nx + start.0] = true;
        while let Some((i, j)) = stack.pop() {
            for (di, dj) in [(1_isize, 0_isize), (-1, 0), (0, 1), (0, -1)] {
                let (ni, nj) = (i as isize + di, j as isize + dj);
                if ni < 0 || nj < 0 || ni >= nx as isize || nj >= nz as isize {
                    continue;
                }
                let (ni, nj) = (ni as usize, nj as usize);
                if !seen[nj * nx + ni] && free(centre(ni, nj)) {
                    seen[nj * nx + ni] = true;
                    stack.push((ni, nj));
                }
            }
        }
        (0..nz)
            .flat_map(|j| (0..nx).map(move |i| (i, j)))
            .filter(|&(i, j)| free(centre(i, j)) && !seen[j * nx + i])
            .count()
    }

    #[test]
    fn layouts_are_reachable_furnished_and_repeatable() {
        for seed in 0..50 {
            let layout = generate(seed);
            assert_eq!(layout, generate(seed));
            assert!(layout.furniture.len() >= 6, "seed {seed}: {layout:?}");
            assert_eq!(unreachable_floor(&layout), 0, "seed {seed}: {layout:?}");
        }
    }

    #[test]
    fn scene_has_walls_partition_and_furniture() {
        let layout = generate(3);
        let toml = scene_toml(&layout, "/robot.toml");
        assert_eq!(
            toml.matches("[[objects]]").count(),
            6 + layout.furniture.len()
        );
        assert!(toml.contains("path = \"/robot.toml\""));
    }
}
