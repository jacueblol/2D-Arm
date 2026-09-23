use glam::DVec3;

use super::joint::Joint;
use crate::kinematics::{self, ElbowConfig};
use crate::trajectory::CartesianTraj;

/// The 3-DOF serial arm: pan (Y-axis) → shoulder (local Z) → elbow (local
/// Z). Convention: pan=0 → arm extends along +X; positive pan rotates the
/// arm toward +Z; positive shoulder tilts upward; elbow bends relative to
/// the shoulder (same axis).
pub struct Arm {
    pub pan: Joint,
    pub shoulder: Joint,
    pub elbow: Joint,
    elbow_config: ElbowConfig,
    cartesian_traj: Option<CartesianTraj>,
}

const G: f64 = 9.81;

impl Arm {
    pub fn new(pan: Joint, shoulder: Joint, elbow: Joint) -> Self {
        Self {
            pan,
            shoulder,
            elbow,
            elbow_config: ElbowConfig::default(),
            cartesian_traj: None,
        }
    }

    /// Which IK solution `set_target` picks when a target admits both an
    /// elbow-up and an elbow-down configuration.
    pub fn with_elbow_config(mut self, config: ElbowConfig) -> Self {
        self.elbow_config = config;
        self
    }

    pub fn set_elbow_config(&mut self, config: ElbowConfig) {
        self.elbow_config = config;
    }

    pub fn elbow_config(&self) -> ElbowConfig {
        self.elbow_config
    }

    pub fn step(&mut self, dt: f64) {
        // Advance the active Cartesian move (if any) and re-solve IK at the
        // interpolated waypoint every step, using update_goal() (not
        // set_setpoint()) so each joint's trapezoidal profile keeps its
        // velocity instead of braking at every tiny waypoint — that's what
        // makes the EE trace a straight line instead of a jerky arc.
        let cartesian_done = if let Some(traj) = &mut self.cartesian_traj {
            let waypoint = traj.advance(dt);
            let done = traj.is_done();
            let l1 = self.shoulder.length;
            let l2 = self.elbow.length;
            if let Some((pan, theta1, theta2)) =
                kinematics::solve_3d(l1, l2, waypoint, self.elbow_config)
            {
                self.pan.update_goal(pan);
                self.shoulder.update_goal(theta1);
                self.elbow.update_goal(theta2);
            }
            done
        } else {
            false
        };
        if cartesian_done {
            self.cartesian_traj = None;
        }

        // Gravity torques, computed before stepping so each joint fights
        // the load it's actually under this step.
        //
        // Convention: load_torque > 0 opposes the motor. Gravity resists
        // raising the arm above horizontal (θ > 0) and assists it below.
        // τ = g·m·r·cos(θ) has the right sign for the motor model's
        // `net = motor_torque − b·ω − load_torque`.
        //
        // Pan rotates about the vertical (Y) axis, so gravity never creates
        // a torque on it. Shoulder and elbow rotate about horizontal axes
        // in the plane of the arm.
        let t1 = self.shoulder.angle_rad();
        let t2 = self.elbow.angle_rad();
        let m1 = self.shoulder.link_mass;
        let m2 = self.elbow.link_mass;
        let l1 = self.shoulder.length;
        let l2 = self.elbow.length;

        // Shoulder supports its own link's center of mass (l1/2) plus the
        // whole elbow assembly (out at l1).
        let tau_shoulder =
            G * ((m1 * l1 / 2.0 + m2 * l1) * t1.cos() + m2 * l2 / 2.0 * (t1 + t2).cos());
        // Elbow supports only its own link's center of mass (l2/2, at
        // absolute angle t1+t2).
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

    /// `[base, shoulder, elbow, end_effector]` in world space (metres).
    pub fn forward_kinematics(&self) -> [DVec3; 4] {
        kinematics::forward_kinematics(
            self.pan.angle_rad(),
            self.shoulder.angle_rad(),
            self.elbow.angle_rad(),
            self.shoulder.length,
            self.elbow.length,
        )
    }

    pub fn ee_pos(&self) -> DVec3 {
        self.forward_kinematics()[3]
    }

    /// Solve IK (using [`Arm::elbow_config`]) and set joint setpoints.
    /// Returns `false` if the target is unreachable.
    pub fn set_target(&mut self, target: DVec3) -> bool {
        let l1 = self.shoulder.length;
        let l2 = self.elbow.length;

        match kinematics::solve_3d(l1, l2, target, self.elbow_config) {
            Some((pan, theta1, theta2)) => {
                self.pan.set_setpoint(pan);
                self.shoulder.set_setpoint(theta1);
                self.elbow.set_setpoint(theta2);
                true
            }
            None => false,
        }
    }

    /// Begin a straight-line EE path from the current EE position to
    /// `target` at `speed_m_s` m/s. IK is re-solved at every simulation
    /// step, so the EE traces a line instead of an arc.
    pub fn start_cartesian_move(&mut self, target: DVec3, speed_m_s: f64) {
        let start = self.ee_pos();
        self.cartesian_traj = Some(CartesianTraj::new(start, target, speed_m_s));
    }

    pub fn is_cartesian_active(&self) -> bool {
        self.cartesian_traj.is_some()
    }
}
