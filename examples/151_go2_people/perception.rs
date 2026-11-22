//! What the robot knows about the people around it.
//!
//! Three sources, all reporting tracks in the robot's odometry frame:
//!
//! * [`Oracle`]: the simulation's truth, as if perception were perfect;
//! * [`NearestTracker`]: Mid-360 person detections tracked by greedy
//!   nearest-neighbour association with a smoothed velocity;
//! * [`ExternalTracker`]: the same detections tracked by an external process
//!   that speaks the robot-perception JSONL stream protocol (one
//!   `lidar_batch` line in, one `lidar_frame` line out).

use std::collections::{BTreeMap, HashMap, VecDeque};
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

use serde_json::{json, Value};

/// A tracked person in the odometry frame (x, y; metres, m/s).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Track {
    pub(crate) id: u64,
    pub(crate) position: [f64; 2],
    pub(crate) velocity: [f64; 2],
}

/// A person-sized cluster in the odometry frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Detection {
    pub(crate) position: [f64; 2],
    pub(crate) points: usize,
}

/// Clustering grid cell; touching cells join.
const CELL_M: f64 = 0.15;
/// Returns this high above the floor can belong to a standing person.
pub(crate) const PERSON_BAND_M: (f64, f64) = (0.15, 1.9);
/// Clusters wider than this are walls or furniture.
const MAX_PERSON_WIDTH_M: f64 = 1.0;
/// Clusters with fewer returns are noise.
const MIN_POINTS: usize = 4;
/// Clusters spanning less height are low clutter. The Go2's Mid-360 hangs
/// upside down and looks mostly down, so a person 2 m away shows only from
/// the shins to the knees.
const MIN_HEIGHT_SPAN_M: f64 = 0.2;

/// Person detections from points `[forward, left, height above floor]`
/// in the robot's base frame, placed in the odometry frame at `odom`.
pub(crate) fn detect_people(points: &[[f64; 3]], odom: [f64; 3]) -> Vec<Detection> {
    detect_moving_people(points, odom, None)
}

/// Grid cell of the static map.
const STATIC_CELL_M: f64 = 0.2;
/// A cell is static structure once it was first seen at least this many
/// frames ago (6 s at 10 Hz) and has been hit in at least
/// [`STATIC_MIN_HITS`] frames since. Walls are seen from afar long before the
/// robot passes them; a person standing still for less than 6 s is not
/// absorbed. The Mid-360's pattern hits a given cell only now and then, so
/// the hit count is well below the frame count.
const STATIC_AGE_FRAMES: u64 = 60;
const STATIC_MIN_HITS: u32 = 15;
/// A cluster mostly on static cells is structure, not a person.
const STATIC_SHARE: f64 = 0.5;

/// Where the robot has seen something in the person band for a long time, in
/// the odometry frame: walls, pillars, furniture. Built online from the
/// robot's own scans; no map is given.
#[derive(Default)]
pub(crate) struct StaticMap {
    /// Per cell: the frame it was first hit in, and the frames that hit it.
    cells: HashMap<(i64, i64), (u64, u32)>,
    frame: u64,
}

impl StaticMap {
    fn cell(p: [f64; 2]) -> (i64, i64) {
        (
            (p[0] / STATIC_CELL_M).floor() as i64,
            (p[1] / STATIC_CELL_M).floor() as i64,
        )
    }

    /// Counts the cells this frame's person-band returns fall in (once each).
    pub(crate) fn update(&mut self, points: &[[f64; 3]], odom: [f64; 3]) {
        let mut cells: Vec<(i64, i64)> = points
            .iter()
            .filter(|p| p[2] >= PERSON_BAND_M.0 && p[2] <= PERSON_BAND_M.1)
            .map(|p| Self::cell(to_odom(odom, [p[0], p[1]])))
            .collect();
        cells.sort_unstable();
        cells.dedup();
        for cell in cells {
            self.cells.entry(cell).or_insert((self.frame, 0)).1 += 1;
        }
        self.frame += 1;
    }

    pub(crate) fn is_static(&self, p: [f64; 2]) -> bool {
        self.cells.get(&Self::cell(p)).is_some_and(|(first, hits)| {
            self.frame - first >= STATIC_AGE_FRAMES && *hits >= STATIC_MIN_HITS
        })
    }
}

/// [`detect_people`], leaving out clusters that lie mostly on cells `map`
/// has seen occupied for a long time.
pub(crate) fn detect_moving_people(
    points: &[[f64; 3]],
    odom: [f64; 3],
    map: Option<&StaticMap>,
) -> Vec<Detection> {
    let mut cells: BTreeMap<(i64, i64), Vec<usize>> = BTreeMap::new();
    for (index, p) in points.iter().enumerate() {
        if p[2] < PERSON_BAND_M.0 || p[2] > PERSON_BAND_M.1 {
            continue;
        }
        let key = (
            (p[0] / CELL_M).floor() as i64,
            (p[1] / CELL_M).floor() as i64,
        );
        cells.entry(key).or_default().push(index);
    }
    let mut seen: BTreeMap<(i64, i64), bool> = cells.keys().map(|k| (*k, false)).collect();
    let mut detections = Vec::new();
    for start in cells.keys() {
        if seen[start] {
            continue;
        }
        seen.insert(*start, true);
        let mut queue = VecDeque::from([*start]);
        let mut members = Vec::new();
        while let Some(cell) = queue.pop_front() {
            members.extend_from_slice(&cells[&cell]);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let next = (cell.0 + dx, cell.1 + dy);
                    if seen.get(&next) == Some(&false) {
                        seen.insert(next, true);
                        queue.push_back(next);
                    }
                }
            }
        }
        let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
        for &i in &members {
            for k in 0..3 {
                lo[k] = lo[k].min(points[i][k]);
                hi[k] = hi[k].max(points[i][k]);
            }
        }
        if let Some(map) = map {
            let on_static = members
                .iter()
                .filter(|&&i| map.is_static(to_odom(odom, [points[i][0], points[i][1]])))
                .count();
            if on_static as f64 > STATIC_SHARE * members.len() as f64 {
                continue;
            }
        }
        let width = (hi[0] - lo[0]).hypot(hi[1] - lo[1]);
        if members.len() < MIN_POINTS
            || width > MAX_PERSON_WIDTH_M
            || hi[2] - lo[2] < MIN_HEIGHT_SPAN_M
        {
            continue;
        }
        let local = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
        detections.push(Detection {
            position: to_odom(odom, local),
            points: members.len(),
        });
    }
    detections
}

/// A base-frame point `[forward, left]` in the odometry frame at pose `odom` (x, y, yaw).
pub(crate) fn to_odom(odom: [f64; 3], local: [f64; 2]) -> [f64; 2] {
    let (sin, cos) = odom[2].sin_cos();
    [
        odom[0] + cos * local[0] - sin * local[1],
        odom[1] + sin * local[0] + cos * local[1],
    ]
}

/// Perfect perception: every person within `range_m`, with its true velocity
/// and identity, given relative to the robot's true pose and placed at its
/// odometry pose (the frame the controller works in).
pub(crate) struct Oracle {
    pub(crate) range_m: f64,
}

impl Oracle {
    /// `people` are `(id, position, velocity)` in the world plane; `truth`
    /// and `odom` are the robot's true and odometry poses (x, y, yaw).
    pub(crate) fn tracks(
        &self,
        people: &[(u64, [f64; 2], [f64; 2])],
        truth: [f64; 3],
        odom: [f64; 3],
    ) -> Vec<Track> {
        let (sin, cos) = truth[2].sin_cos();
        let turn = odom[2] - truth[2];
        let (s2, c2) = turn.sin_cos();
        people
            .iter()
            .filter_map(|(id, p, v)| {
                let (dx, dy) = (p[0] - truth[0], p[1] - truth[1]);
                if dx.hypot(dy) > self.range_m {
                    return None;
                }
                let local = [cos * dx + sin * dy, -sin * dx + cos * dy];
                Some(Track {
                    id: *id,
                    position: to_odom(odom, local),
                    velocity: [c2 * v[0] - s2 * v[1], s2 * v[0] + c2 * v[1]],
                })
            })
            .collect()
    }
}

/// Greedy nearest-neighbour tracking with an exponentially smoothed velocity.
#[derive(Default)]
pub(crate) struct NearestTracker {
    tracks: Vec<NnTrack>,
    next_id: u64,
}

struct NnTrack {
    id: u64,
    position: [f64; 2],
    velocity: [f64; 2],
    hits: u32,
    last_seen_s: f64,
}

/// Detections farther than this from a track's prediction start a new track.
const NN_GATE_M: f64 = 0.8;
/// Tracks unseen this long are dropped.
const NN_MAX_AGE_S: f64 = 0.5;
/// Weight of the newest velocity sample.
const NN_VELOCITY_ALPHA: f64 = 0.4;

impl NearestTracker {
    pub(crate) fn update(
        &mut self,
        detections: &[Detection],
        time_s: f64,
        dt_s: f64,
    ) -> Vec<Track> {
        let predicted: Vec<[f64; 2]> = self
            .tracks
            .iter()
            .map(|t| {
                let age = time_s - t.last_seen_s;
                [
                    t.position[0] + t.velocity[0] * age,
                    t.position[1] + t.velocity[1] * age,
                ]
            })
            .collect();
        let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
        for (i, p) in predicted.iter().enumerate() {
            for (j, d) in detections.iter().enumerate() {
                let dist = (p[0] - d.position[0]).hypot(p[1] - d.position[1]);
                if dist <= NN_GATE_M {
                    pairs.push((dist, i, j));
                }
            }
        }
        pairs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let (mut used_t, mut used_d) = (
            vec![false; self.tracks.len()],
            vec![false; detections.len()],
        );
        for (_, i, j) in pairs {
            if used_t[i] || used_d[j] {
                continue;
            }
            used_t[i] = true;
            used_d[j] = true;
            let track = &mut self.tracks[i];
            let elapsed = (time_s - track.last_seen_s).max(dt_s);
            let sample = [
                (detections[j].position[0] - track.position[0]) / elapsed,
                (detections[j].position[1] - track.position[1]) / elapsed,
            ];
            let alpha = if track.hits == 1 {
                1.0
            } else {
                NN_VELOCITY_ALPHA
            };
            for (velocity, sample) in track.velocity.iter_mut().zip(sample) {
                *velocity += alpha * (sample - *velocity);
            }
            track.position = detections[j].position;
            track.hits += 1;
            track.last_seen_s = time_s;
        }
        for (j, d) in detections.iter().enumerate() {
            if !used_d[j] {
                self.next_id += 1;
                self.tracks.push(NnTrack {
                    id: self.next_id,
                    position: d.position,
                    velocity: [0.0, 0.0],
                    hits: 1,
                    last_seen_s: time_s,
                });
            }
        }
        self.tracks
            .retain(|t| time_s - t.last_seen_s <= NN_MAX_AGE_S);
        self.tracks
            .iter()
            .filter(|t| t.hits >= 2)
            .map(|t| {
                let age = time_s - t.last_seen_s;
                Track {
                    id: t.id,
                    position: [
                        t.position[0] + t.velocity[0] * age,
                        t.position[1] + t.velocity[1] * age,
                    ],
                    velocity: t.velocity,
                }
            })
            .collect()
    }
}

/// A tracker in another process, fed one `lidar_batch` per scan.
pub(crate) struct ExternalTracker {
    child: Child,
    input: ChildStdin,
    output: BufReader<ChildStdout>,
    sequence: u64,
}

impl ExternalTracker {
    /// Starts `command` (program and arguments). It must read JSONL records
    /// on stdin and answer each `lidar_batch` with one `lidar_frame` line.
    pub(crate) fn spawn(command: &[String]) -> std::io::Result<Self> {
        let (program, args) = command
            .split_first()
            .ok_or_else(|| std::io::Error::other("empty tracker command"))?;
        let mut child = Command::new(program)
            .args(args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()?;
        let input = child
            .stdin
            .take()
            .ok_or_else(|| std::io::Error::other("no tracker stdin"))?;
        let output = BufReader::new(
            child
                .stdout
                .take()
                .ok_or_else(|| std::io::Error::other("no tracker stdout"))?,
        );
        Ok(Self {
            child,
            input,
            output,
            sequence: 0,
        })
    }

    /// Sends one scan's detections and returns the confirmed tracks.
    pub(crate) fn update(
        &mut self,
        detections: &[Detection],
        time_s: f64,
    ) -> std::io::Result<Vec<Track>> {
        let t_ns = (time_s * 1e9).round() as i64;
        let records: Vec<Value> = detections
            .iter()
            .map(|d| {
                json!({
                    "position": [d.position[0], d.position[1], 0.9],
                    "covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.04],
                    "class": 1,
                    "score": (1.0 - (-(d.points as f64) / 12.0).exp()),
                })
            })
            .collect();
        let batch = json!({
            "type": "lidar_batch", "schema_version": 1, "sensor_id": 1, "sequence": self.sequence,
            "event_time_ns": t_ns, "arrival_time_ns": t_ns, "frame_id": "odom", "detections": records,
        });
        self.sequence += 1;
        writeln!(self.input, "{batch}")?;
        self.input.flush()?;
        let mut line = String::new();
        loop {
            line.clear();
            if self.output.read_line(&mut line)? == 0 {
                return Err(std::io::Error::other("tracker closed its output"));
            }
            let frame: Value = serde_json::from_str(&line).map_err(std::io::Error::other)?;
            if frame["type"] != "lidar_frame" {
                continue;
            }
            let tracks = frame["tracks"].as_array().cloned().unwrap_or_default();
            return Ok(tracks
                .iter()
                .filter(|t| t["lifecycle"] == "confirmed")
                .filter_map(|t| {
                    let p = t["position"].as_array()?;
                    let v = t["velocity"].as_array()?;
                    Some(Track {
                        id: t["track_id"].as_u64()?,
                        position: [p[0].as_f64()?, p[1].as_f64()?],
                        velocity: [v[0].as_f64()?, v[1].as_f64()?],
                    })
                })
                .collect());
        }
    }
}

impl Drop for ExternalTracker {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn blob(center: [f64; 2]) -> Vec<[f64; 3]> {
        let mut points = Vec::new();
        for i in 0..6 {
            for z in [0.3, 0.6] {
                points.push([
                    center[0] + 0.05 * f64::from(i),
                    center[1] + 0.03 * f64::from(i),
                    z,
                ]);
            }
        }
        points
    }

    #[test]
    fn people_are_detected_and_walls_are_not() {
        let mut points = blob([3.0, 1.0]);
        // A 4 m wall: tall and long.
        for i in 0..40 {
            for z in [0.5, 1.0, 1.5] {
                points.push([5.0, -2.0 + 0.1 * f64::from(i), z]);
            }
        }
        let detections = detect_people(&points, [10.0, 0.0, std::f64::consts::FRAC_PI_2]);
        assert_eq!(detections.len(), 1);
        // Robot at (10, 0) facing +y: 3 m ahead and 1 m left is (9, 3).
        let p = detections[0].position;
        assert!(
            (p[0] - 8.93).abs() < 0.1 && (p[1] - 3.13).abs() < 0.1,
            "{p:?}"
        );
    }

    #[test]
    fn static_map_removes_walls_but_keeps_people_walking_past() {
        // A wall seen for 7 s; a person walking past it, 0.5 m away.
        let mut map = StaticMap::default();
        let wall: Vec<[f64; 3]> = (0..10)
            .flat_map(|i| {
                [
                    [4.0, 0.1 * f64::from(i), 0.5],
                    [4.0, 0.1 * f64::from(i), 1.0],
                ]
            })
            .collect();
        for _ in 0..70 {
            map.update(&wall, [0.0, 0.0, 0.0]);
        }
        assert!(detect_moving_people(&wall, [0.0, 0.0, 0.0], Some(&map)).is_empty());
        assert_eq!(detect_people(&wall, [0.0, 0.0, 0.0]).len(), 1);
        for k in 0..20 {
            let person = blob([3.0, -2.0 + 0.1 * f64::from(k)]);
            assert_eq!(
                detect_moving_people(&person, [0.0, 0.0, 0.0], Some(&map)).len(),
                1,
                "step {k}"
            );
            map.update(&person, [0.0, 0.0, 0.0]);
        }
    }

    #[test]
    fn nearest_tracker_keeps_identity_and_estimates_velocity() {
        let mut tracker = NearestTracker::default();
        let mut id = None;
        for k in 0..20 {
            let t = f64::from(k) * 0.1;
            let d = Detection {
                position: [2.0 + 0.5 * t, 0.0],
                points: 20,
            };
            let tracks = tracker.update(&[d], t, 0.1);
            if k >= 1 {
                assert_eq!(tracks.len(), 1);
                assert_eq!(*id.get_or_insert(tracks[0].id), tracks[0].id);
            }
            if k == 19 {
                assert!(
                    (tracks[0].velocity[0] - 0.5).abs() < 0.05,
                    "{:?}",
                    tracks[0]
                );
            }
        }
    }
}
