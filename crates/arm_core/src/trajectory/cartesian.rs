use glam::DVec3;

/// A constant-speed straight-line path for the end-effector between two
/// points. Sampled every simulation step and re-solved through IK (see
/// `Arm::step`), so the EE traces a line instead of the arc a per-joint
/// trapezoidal retarget alone would produce.
pub struct CartesianTraj {
    start: DVec3,
    end: DVec3,
    total_time: f64,
    elapsed: f64,
    done: bool,
}

impl CartesianTraj {
    pub fn new(start: DVec3, end: DVec3, speed_m_s: f64) -> Self {
        let dist = (end - start).length();
        let (total_time, done) = if dist > 1e-4 {
            (dist / speed_m_s, false)
        } else {
            (0.0, true)
        };
        Self {
            start,
            end,
            total_time,
            elapsed: 0.0,
            done,
        }
    }

    /// Advance by `dt` seconds, returning the current waypoint on the line.
    pub fn advance(&mut self, dt: f64) -> DVec3 {
        if self.done {
            return self.end;
        }
        self.elapsed = (self.elapsed + dt).min(self.total_time);
        if self.elapsed >= self.total_time {
            self.done = true;
        }
        let t = self.elapsed / self.total_time;
        self.start.lerp(self.end, t)
    }

    pub fn is_done(&self) -> bool {
        self.done
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_and_completes() {
        let mut traj = CartesianTraj::new(DVec3::ZERO, DVec3::new(1.0, 0.0, 0.0), 0.5);
        let wp = traj.advance(1.0);
        assert!((wp.x - 0.5).abs() < 1e-9, "halfway: {}", wp.x);
        assert!(!traj.is_done());
        let wp = traj.advance(1.0);
        assert!((wp.x - 1.0).abs() < 1e-9, "end: {}", wp.x);
        assert!(traj.is_done());
    }

    #[test]
    fn zero_length_immediately_done() {
        let p = DVec3::new(1.0, 2.0, 3.0);
        let traj = CartesianTraj::new(p, p, 0.5);
        assert!(traj.is_done());
    }
}
