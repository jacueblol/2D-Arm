use bevy::math::{DQuat, DVec3};
use crate::arm::{ik, joint3d::Joint3d};

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
}

impl ArmSim3d {
    pub fn new(pan: Joint3d, shoulder: Joint3d, elbow: Joint3d) -> Self {
        Self { pan, shoulder, elbow }
    }

    pub fn step(&mut self, dt: f64) {
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
}
