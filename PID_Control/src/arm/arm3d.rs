use bevy::math::{DQuat, DVec3};
use crate::arm::{ik, joint3d::Joint3d, cartesian_traj::CartesianTraj};

/// 3-DOF serial arm: pan (Y-axis) → shoulder (local Z) → elbow (local Z).
///
/// Convention:
///   - pan = 0 → arm extends along +X
///   - positive pan rotates arm toward +Z
///   - positive shoulder tilts arm upward
///   - elbow bends relative to shoulder (same axis)
pub struct ArmSim3d {
    pub pan: Joint3d,
    pub shoulder: Joint3d,
    pub elbow: Joint3d,
    cartesian_traj: Option<CartesianTraj>,
}

impl ArmSim3d {
    pub fn new(pan: Joint3d, shoulder: Joint3d, elbow: Joint3d) -> Self {
        Self { pan, shoulder, elbow, cartesian_traj: None }
    }

    pub fn step(&mut self, dt: f64) {
        // Advance Cartesian trajectory and update joint goals via IK each frame.
        // update_goal() keeps profile velocity so the arm doesn't brake at every tiny waypoint.
        let cartesian_done = if let Some(traj) = &mut self.cartesian_traj {
            let wp = traj.advance(dt);
            let done = traj.is_done();
            let l1 = self.shoulder.length;
            let l2 = self.elbow.length;
            let t0 = wp.z.atan2(wp.x);
            let r  = (wp.x * wp.x + wp.z * wp.z).sqrt();
            let h  = wp.y;
            if let Some(solutions) = ik::solve(l1, l2, r, h) {
                let sol = &solutions.elbow_neg;
                self.pan.update_goal(t0);
                self.shoulder.update_goal(sol.theta1);
                self.elbow.update_goal(sol.theta2);
            }
            done
        } else {
            false
        };
        if cartesian_done { self.cartesian_traj = None; }

        // Gravity torques — computed before stepping so each joint fights the correct load.
        //
        // Convention: load_torque > 0 opposes the motor. Gravity resists raising the arm above
        // horizontal (θ > 0) and assists it when below. The formula τ = g·m·r·cos(θ) has the
        // right sign for the motor model's `net = motor_torque - b·ω - load_torque`.
        //
        // Pan rotates about vertical (Y), so gravity never creates a torque on it.
        // Shoulder and elbow rotate about horizontal axes in the plane of the arm.
        let t1 = self.shoulder.angle_rad();
        let t2 = self.elbow.angle_rad();
        let m1 = self.shoulder.link_mass;
        let m2 = self.elbow.link_mass;
        let l1 = self.shoulder.length;
        let l2 = self.elbow.length;
        const G: f64 = 9.81;

        // Shoulder must support its own link COM (l1/2) plus the elbow assembly (l1 out).
        let tau_shoulder = G * ((m1 * l1 / 2.0 + m2 * l1) * t1.cos()
                                + m2 * l2 / 2.0 * (t1 + t2).cos());
        // Elbow supports only its own link COM (l2/2, at absolute angle t1+t2).
        let tau_elbow = G * m2 * l2 / 2.0 * (t1 + t2).cos();

        self.shoulder.set_load_torque(tau_shoulder);
        self.elbow.set_load_torque(tau_elbow);

        self.pan.step(dt);
        self.shoulder.step(dt);
        self.elbow.step(dt);
    }

    pub fn reset(&mut self) {
        self.pan.reset();
        self.shoulder.reset();
        self.elbow.reset();
        self.cartesian_traj = None;
    }

    /// Returns [base, shoulder, elbow, end_effector] in world space (metres).
    pub fn forward_kinematics(&self) -> [DVec3; 4] {
        let t0 = self.pan.angle_rad();
        let t1 = self.shoulder.angle_rad();
        let t2 = self.elbow.angle_rad();

        // Cumulative world rotation.
        // Negate t0 so positive angle rotates arm toward +Z (right-hand about Y = toward -Z,
        // so -t0 gives the intuitive "positive angle sweeps toward +Z" behaviour).
        let r_pan = DQuat::from_rotation_y(-t0);
        let r_sho = r_pan * DQuat::from_rotation_z(t1);
        let r_elb = r_sho * DQuat::from_rotation_z(t2);

        let base     = DVec3::ZERO;
        let shoulder = DVec3::ZERO; // shoulder joint coincides with base pivot
        let elbow    = shoulder + r_sho * DVec3::new(self.shoulder.length, 0.0, 0.0);
        let ee       = elbow    + r_elb * DVec3::new(self.elbow.length,    0.0, 0.0);

        [base, shoulder, elbow, ee]
    }

    pub fn ee_pos(&self) -> DVec3 {
        self.forward_kinematics()[3]
    }

    /// Solve IK and set joint setpoints. Returns false if target is unreachable.
    ///
    /// Decomposition: pan faces target horizontally, then 2-link planar IK in
    /// the vertical plane defined by the pan direction.
    pub fn set_target(&mut self, target: DVec3) -> bool {
        let l1 = self.shoulder.length;
        let l2 = self.elbow.length;

        let t0 = target.z.atan2(target.x);
        let r  = (target.x * target.x + target.z * target.z).sqrt();
        let h  = target.y;

        match ik::solve(l1, l2, r, h) {
            Some(solutions) => {
                let sol = &solutions.elbow_neg;
                self.pan.set_setpoint(t0);
                self.shoulder.set_setpoint(sol.theta1);
                self.elbow.set_setpoint(sol.theta2);
                true
            }
            None => false,
        }
    }

    /// Begin a straight-line EE path from current EE position to `target` at `speed_m_s` m/s.
    /// IK is re-solved at every simulation step, so the EE traces a line instead of an arc.
    pub fn start_cartesian_move(&mut self, target: DVec3, speed_m_s: f64) {
        let start = self.ee_pos();
        self.cartesian_traj = Some(CartesianTraj::new(start, target, speed_m_s));
    }

    pub fn is_cartesian_active(&self) -> bool {
        self.cartesian_traj.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::{joint3d::Joint3d, trajectory::TrapezoidProfile};
    use crate::motor::motor_sim::{MotorParams, MotorSim};
    use crate::pid::pid_controller::PidController;

    // Mirror of config.toml values so tests reflect real sim behavior.
    fn make_arm() -> ArmSim3d {
        let pan = Joint3d::new(0.0, 0.0,
            MotorSim::with_params(MotorParams::default()),
            PidController::new(8.0, 0.5, 0.5),
            TrapezoidProfile::new(1.5, 3.0))
            .with_limits((-180f64).to_radians(), (180f64).to_radians())
            .with_velocity_ff(0.3);
        let shoulder = Joint3d::new(1.0, 0.3,
            MotorSim::with_params(MotorParams::default()),
            PidController::new(10.0, 0.8, 1.0),
            TrapezoidProfile::new(1.0, 2.0))
            .with_limits((-30f64).to_radians(), (135f64).to_radians())
            .with_velocity_ff(0.5);
        let elbow = Joint3d::new(0.8, 0.15,
            MotorSim::with_params(MotorParams::default()),
            PidController::new(10.0, 0.8, 1.0),
            TrapezoidProfile::new(1.2, 2.5))
            .with_limits((-150f64).to_radians(), (10f64).to_radians())
            .with_velocity_ff(0.4);
        ArmSim3d::new(pan, shoulder, elbow)
    }

    // Step for `n` iterations at `dt`, returning final EE position.
    fn run(arm: &mut ArmSim3d, n: usize, dt: f64) -> DVec3 {
        for _ in 0..n { arm.step(dt); }
        arm.ee_pos()
    }

    // Settle the arm at a position via set_target, then measure SS error.
    fn settle_dist(arm: &mut ArmSim3d, target: DVec3) -> f64 {
        arm.set_target(target);
        let ee = run(arm, 6000, 0.005);  // 30 simulated seconds
        (ee - target).length()
    }

    #[test]
    fn arm_converges_forward_reach() {
        // Easy case — arm mostly forward, moderate gravity. Should settle tight.
        let mut arm = make_arm();
        let dist = settle_dist(&mut arm, DVec3::new(1.2, 0.8, 0.4));
        assert!(dist < 0.012, "expected <12mm SS error, got {:.1}mm", dist * 1000.0);
    }

    #[test]
    fn arm_converges_lateral_reach() {
        // Harder — significant pan + gravity loading. Integral needs more time.
        let mut arm = make_arm();
        let dist = settle_dist(&mut arm, DVec3::new(0.7, 0.6, 0.5));
        assert!(dist < 0.015, "expected <15mm SS error, got {:.1}mm", dist * 1000.0);
    }

    #[test]
    fn arm_converges_high_target() {
        // Hardest — high shoulder elevation, maximum gravity torque on shoulder.
        let mut arm = make_arm();
        let dist = settle_dist(&mut arm, DVec3::new(0.8, 1.2, 0.0));
        assert!(dist < 0.015, "expected <15mm SS error, got {:.1}mm", dist * 1000.0);
    }

    #[test]
    fn cartesian_move_ends_within_arrival_threshold() {
        const ARRIVAL_DIST_M: f64 = 0.025;
        let target = DVec3::new(1.0, 0.7, 0.3);
        let mut arm = make_arm();
        arm.start_cartesian_move(target, 0.3);
        let ee = run(&mut arm, 8000, 0.005);  // 40 simulated seconds
        let dist = (ee - target).length();
        assert!(dist < ARRIVAL_DIST_M,
            "EE should be within arrival threshold ({:.0}mm) after Cartesian move, got {:.1}mm",
            ARRIVAL_DIST_M * 1000.0, dist * 1000.0);
    }

    #[test]
    fn cartesian_move_tracks_straight_line() {
        // Verify the EE traces approximately a straight line during a Cartesian move
        // by sampling intermediate positions and checking lateral deviation.
        let start = DVec3::new(0.8, 0.5, 0.0);
        let end   = DVec3::new(1.2, 1.0, 0.3);
        let mut arm = make_arm();
        arm.set_target(start);
        run(&mut arm, 4000, 0.005);  // settle at start

        arm.start_cartesian_move(end, 0.25);

        // Sample every 50 steps while the Cartesian move is active
        let mut max_lateral_err = 0.0f64;
        let line_dir = (end - start).normalize();
        for _ in 0..200 {
            arm.step(0.005);
            if !arm.is_cartesian_active() { break; }
            let ee = arm.ee_pos();
            // Project onto line, compute lateral offset
            let to_ee = ee - start;
            let along = to_ee.dot(line_dir).clamp(0.0, (end - start).length());
            let proj  = start + line_dir * along;
            let lateral = (ee - proj).length();
            max_lateral_err = max_lateral_err.max(lateral);
        }
        assert!(max_lateral_err < 0.10,
            "Cartesian path should stay within 10cm of straight line, max deviation: {:.1}mm",
            max_lateral_err * 1000.0);
    }

    #[test]
    fn choreography_figure_eight_advances_waypoints() {
        use crate::arm::choreography;
        let mut arm = make_arm();
        let mut choreo = choreography::figure_eight(0.25);
        choreo.start(&mut arm);

        // Run for up to 60 simulated seconds — figure-eight should complete
        let dt = 0.005;
        let max_steps = 12000;
        for _ in 0..max_steps {
            arm.step(dt);
            if choreo.is_done() { break; }
            let dist = choreo.current_target()
                .map(|t| (arm.ee_pos() - t).length())
                .unwrap_or(0.0);
            choreo.advance(&mut arm, dist, dt);
        }
        assert!(choreo.is_done(), "figure-eight should complete within 60 simulated seconds");
    }
}
