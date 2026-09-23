use glam::DVec3;

use crate::arm::Arm;

/// How close the EE needs to get before a waypoint counts as "reached".
const ARRIVAL_DIST_M: f64 = 0.025;

/// A sequenced list of Cartesian waypoints with per-waypoint dwell times.
pub struct WaypointSeq {
    waypoints: Vec<(DVec3, f64)>, // (target, dwell_seconds)
    current_idx: usize,
    dwell_timer: f64,
    dwelling: bool,
    active: bool,
    speed_m_s: f64,
    pub label: &'static str,
}

impl WaypointSeq {
    pub fn new(waypoints: Vec<(DVec3, f64)>, speed_m_s: f64, label: &'static str) -> Self {
        Self {
            waypoints,
            current_idx: 0,
            dwell_timer: 0.0,
            dwelling: false,
            active: true,
            speed_m_s,
            label,
        }
    }

    /// Kick off the first waypoint move. Call once after creation.
    pub fn start(&mut self, arm: &mut Arm) {
        if let Some((target, _)) = self.waypoints.first() {
            arm.start_cartesian_move(*target, self.speed_m_s);
        } else {
            self.active = false;
        }
    }

    /// Drive the sequence forward. Call every frame with the current
    /// EE-to-target distance and `dt`.
    pub fn advance(&mut self, arm: &mut Arm, dist_to_target: f64, dt: f64) {
        if !self.active || self.current_idx >= self.waypoints.len() {
            return;
        }

        let cartesian_done = !arm.is_cartesian_active();

        if self.dwelling {
            self.dwell_timer -= dt;
            if self.dwell_timer <= 0.0 {
                self.dwelling = false;
                self.current_idx += 1;
                if self.current_idx < self.waypoints.len() {
                    let (next, _) = self.waypoints[self.current_idx];
                    arm.start_cartesian_move(next, self.speed_m_s);
                } else {
                    self.active = false;
                }
            }
        } else if cartesian_done && dist_to_target < ARRIVAL_DIST_M {
            let (_, dwell) = self.waypoints[self.current_idx];
            if dwell > 0.0 {
                self.dwelling = true;
                self.dwell_timer = dwell;
            } else {
                self.current_idx += 1;
                if self.current_idx < self.waypoints.len() {
                    let (next, _) = self.waypoints[self.current_idx];
                    arm.start_cartesian_move(next, self.speed_m_s);
                } else {
                    self.active = false;
                }
            }
        }
    }

    pub fn is_done(&self) -> bool {
        !self.active
    }

    /// The current target position — the caller uses this to compute
    /// `dist_to_target` for [`WaypointSeq::advance`].
    pub fn current_target(&self) -> Option<DVec3> {
        self.waypoints.get(self.current_idx).map(|(p, _)| *p)
    }
}

/// Sample a figure-8 (Lissajous 1:2): `x = cx + ax*sin(t)`, `y = cy + ay*sin(2t)`.
fn lissajous_figure_eight(n: usize, cx: f64, cy: f64, z: f64, ax: f64, ay: f64) -> Vec<DVec3> {
    (0..n)
        .map(|i| {
            let t = std::f64::consts::TAU * i as f64 / n as f64;
            DVec3::new(cx + ax * t.sin(), cy + ay * (2.0 * t).sin(), z)
        })
        .collect()
}

/// Trace a figure-8 (Lissajous 1:2) in the XY plane, centred at (1.0, 0.8, 0.0).
pub fn figure_eight(speed: f64) -> WaypointSeq {
    let pts = lissajous_figure_eight(16, 1.0, 0.8, 0.0, 0.38, 0.28);
    let waypoints = pts.into_iter().map(|p| (p, 0.0)).collect();
    WaypointSeq::new(waypoints, speed, "Figure-Eight")
}

/// Horizontal circle in the XZ plane at y=0.8, radius 0.5, centred at (0.7, 0.8, 0.0).
pub fn circle_sweep(speed: f64) -> WaypointSeq {
    let n = 12;
    let (cx, cz, y, r) = (0.7_f64, 0.0_f64, 0.8_f64, 0.5_f64);
    let waypoints = (0..n)
        .map(|i| {
            let t = std::f64::consts::TAU * i as f64 / n as f64;
            (DVec3::new(cx + r * t.cos(), y, cz + r * t.sin()), 0.0)
        })
        .collect();
    WaypointSeq::new(waypoints, speed, "Circle Sweep")
}

/// Helix: circle of radius 0.4 around (0.8, y, 0.0), spiralling y=0.4 to y=1.2, two turns.
pub fn helix(speed: f64) -> WaypointSeq {
    let n: usize = 16;
    let (cx, r, y_start, y_end) = (0.8_f64, 0.4_f64, 0.4_f64, 1.2_f64);
    let waypoints = (0..n)
        .map(|i| {
            let frac = i as f64 / (n - 1) as f64;
            let t = std::f64::consts::TAU * 2.0 * i as f64 / n as f64;
            let y = y_start + (y_end - y_start) * frac;
            (DVec3::new(cx + r * t.cos(), y, r * t.sin()), 0.0)
        })
        .collect();
    WaypointSeq::new(waypoints, speed, "Helix")
}

/// Rise to shoulder height then oscillate in Z, like a friendly hand wave.
pub fn wave_hello(speed: f64) -> WaypointSeq {
    let centre = DVec3::new(0.8, 1.3, 0.0);
    let left = DVec3::new(0.8, 1.3, -0.22);
    let right = DVec3::new(0.8, 1.3, 0.35);

    let mut waypoints = vec![(DVec3::new(0.8, 0.9, 0.0), 0.2), (centre, 0.15)];
    for i in 0..4 {
        waypoints.push(if i % 2 == 0 {
            (right, 0.0)
        } else {
            (left, 0.0)
        });
    }
    waypoints.push((right, 0.0));
    waypoints.push((centre, 0.3));

    WaypointSeq::new(waypoints, speed, "Wave Hello")
}

/// Pick an object from one spot and place it at another: pick-floor -> lift
/// -> swing -> lower -> place.
pub fn pick_and_place(speed: f64) -> WaypointSeq {
    let pick_floor = DVec3::new(1.0, 0.3, 0.3);
    let pick_lift = DVec3::new(1.0, 0.8, 0.3);
    let place_lift = DVec3::new(1.0, 0.8, -0.3);
    let place_floor = DVec3::new(1.0, 0.3, -0.3);

    let waypoints = vec![
        (pick_floor, 0.5),
        (pick_lift, 0.1),
        (place_lift, 0.1),
        (place_floor, 0.5),
    ];
    WaypointSeq::new(waypoints, speed, "Pick & Place")
}

/// Trace a triangle in 3D space, pausing at each vertex — good for verifying
/// straight-line Cartesian moves between very different poses.
pub fn triangle(speed: f64) -> WaypointSeq {
    let waypoints = vec![
        (DVec3::new(1.2, 0.4, 0.0), 0.3),
        (DVec3::new(0.6, 1.1, 0.5), 0.3),
        (DVec3::new(0.6, 1.1, -0.5), 0.3),
    ];
    WaypointSeq::new(waypoints, speed, "Triangle")
}

/// Knock on the table: drop to near-floor, tap three times, then retreat.
pub fn knock_knock(speed: f64) -> WaypointSeq {
    let up = DVec3::new(0.9, 0.7, 0.0);
    let down = DVec3::new(0.9, 0.18, 0.0);
    let waypoints = vec![
        (up, 0.1),
        (down, 0.05),
        (up, 0.1),
        (down, 0.05),
        (up, 0.1),
        (down, 0.05),
        (up, 0.4),
    ];
    WaypointSeq::new(waypoints, speed * 1.5, "Knock Knock")
}

/// Write a rough "Z" in the vertical XZ plane — three strokes.
pub fn zorro(speed: f64) -> WaypointSeq {
    let waypoints = vec![
        (DVec3::new(0.65, 1.1, -0.35), 0.1),
        (DVec3::new(0.65, 1.1, 0.35), 0.1),
        (DVec3::new(0.65, 0.5, -0.35), 0.1),
        (DVec3::new(0.65, 0.5, 0.35), 0.3),
    ];
    WaypointSeq::new(waypoints, speed, "Zorro Z")
}

/// A slow, dreamy figure-eight at reduced speed with micro-dwells.
pub fn slow_drift(speed: f64) -> WaypointSeq {
    let pts = lissajous_figure_eight(24, 0.95, 0.75, 0.0, 0.32, 0.22);
    let waypoints = pts.into_iter().map(|p| (p, 0.04)).collect();
    WaypointSeq::new(waypoints, speed * 0.45, "Slow Drift")
}
