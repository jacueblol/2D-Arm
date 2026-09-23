use crate::motor::MotorSim;
use crate::pid::PidController;
use crate::trajectory::{TrapezoidProfile, TrapezoidState};

/// Re-express `target` as the angle nearest `reference` that's congruent to
/// it modulo a full turn — the shortest signed path from `reference` to some
/// angle equivalent to `target`. A motor's accumulated position isn't
/// wrapped to a canonical range (it just keeps counting revolutions), so a
/// raw `setpoint - position` PID error could demand going the long way
/// around; re-expressing the setpoint this way before differencing makes
/// the error always take the short path.
fn nearest_congruent_angle(reference: f64, target: f64) -> f64 {
    let raw = target - reference;
    let wrapped = raw - (raw / std::f64::consts::TAU).round() * std::f64::consts::TAU;
    reference + wrapped
}

/// A single joint: motor + PID + trapezoidal motion profile + limits,
/// wired together. `arm_core::arm::Arm` owns three of these (pan, shoulder,
/// elbow).
pub struct Joint {
    pub length: f64,
    pub link_mass: f64,
    motor: MotorSim,
    pid: PidController,
    profile: TrapezoidProfile,
    profile_state: TrapezoidState,
    goal_rad: f64,
    min_rad: f64,
    max_rad: f64,
    /// Velocity feedforward gain, V per rad/s of profile velocity.
    kf: f64,
    prev_error: f64,
    integ_total: f64,
}

impl Joint {
    pub fn new(
        length: f64,
        link_mass: f64,
        motor: MotorSim,
        pid: PidController,
        profile: TrapezoidProfile,
    ) -> Self {
        Self {
            length,
            link_mass,
            motor,
            pid,
            profile,
            profile_state: TrapezoidState {
                position: 0.0,
                velocity: 0.0,
            },
            goal_rad: 0.0,
            min_rad: f64::NEG_INFINITY,
            max_rad: f64::INFINITY,
            kf: 0.0,
            prev_error: 0.0,
            integ_total: 0.0,
        }
    }

    /// Clamp the joint's commanded range. Defaults to unlimited.
    pub fn with_limits(mut self, min_rad: f64, max_rad: f64) -> Self {
        self.min_rad = min_rad;
        self.max_rad = max_rad;
        self
    }

    /// Velocity feedforward gain: adds `kf * profile_velocity` to motor
    /// voltage, reducing tracking lag when the profile is moving fast.
    pub fn with_velocity_ff(mut self, kf: f64) -> Self {
        self.kf = kf;
        self
    }

    /// Set the commanded goal angle (hard retarget): the trajectory profile
    /// resets its velocity and ramps toward the new goal from rest. Goal is
    /// clamped to the joint's limits.
    pub fn set_setpoint(&mut self, rad: f64) {
        let pos = self.motor.get_position_rad();
        self.goal_rad = nearest_congruent_angle(pos, rad).clamp(self.min_rad, self.max_rad);
        self.profile_state = TrapezoidState {
            position: pos,
            velocity: self.motor.get_velocity_rad_s(),
        };
    }

    /// Update the goal without resetting trajectory profile velocity. Use
    /// for continuous Cartesian tracking, where the goal moves a tiny step
    /// every frame — resetting velocity here would brake the arm at every
    /// waypoint instead of tracking smoothly.
    pub fn update_goal(&mut self, rad: f64) {
        let pos = self.motor.get_position_rad();
        self.goal_rad = nearest_congruent_angle(pos, rad).clamp(self.min_rad, self.max_rad);
        self.profile_state.position = pos;
        // profile_state.velocity is intentionally kept — continuous motion stays smooth.
    }

    /// Instantaneous profile setpoint (ramping toward `goal_rad`).
    pub fn setpoint_rad(&self) -> f64 {
        self.profile_state.position
    }

    pub fn goal_rad(&self) -> f64 {
        self.goal_rad
    }

    pub fn angle_rad(&self) -> f64 {
        self.motor.get_position_rad()
    }

    pub fn error_rad(&self) -> f64 {
        self.prev_error
    }

    pub fn velocity_rad_s(&self) -> f64 {
        self.motor.get_velocity_rad_s()
    }

    pub fn set_load_torque(&mut self, torque: f64) {
        self.motor.set_load_torque(torque);
    }

    pub fn step(&mut self, dt: f64) {
        self.profile_state = self
            .profile
            .calculate(dt, self.profile_state, self.goal_rad);

        let pos = self.motor.get_position_rad();
        let effective_sp = nearest_congruent_angle(pos, self.profile_state.position);

        let out = self
            .pid
            .step(dt, effective_sp, pos, self.prev_error, self.integ_total);
        self.prev_error = out.error;
        self.integ_total = out.integ_total;

        let gravity_ff = self.motor.gravity_feedforward_volts();
        let vel_ff = self.kf * self.profile_state.velocity;
        self.motor.set_voltage(out.output + gravity_ff + vel_ff);
        self.motor.step(dt);
    }

    pub fn reset(&mut self) {
        self.motor.reset();
        self.prev_error = 0.0;
        self.integ_total = 0.0;
        self.profile_state = TrapezoidState {
            position: 0.0,
            velocity: 0.0,
        };
        self.goal_rad = 0.0;
    }

    pub fn gains(&self) -> (f64, f64, f64, f64) {
        (self.pid.k_p, self.pid.k_i, self.pid.k_d, self.kf)
    }

    pub fn set_gains(&mut self, kp: f64, ki: f64, kd: f64, kf: f64) {
        self.pid.k_p = kp;
        self.pid.k_i = ki;
        self.pid.k_d = kd;
        self.kf = kf;
    }

    pub fn reset_integrator(&mut self) {
        self.integ_total = 0.0;
        self.prev_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::MotorParams;
    use std::f64::consts::PI;

    fn make_joint() -> Joint {
        Joint::new(
            1.0,
            0.0,
            MotorSim::with_params(MotorParams::default()),
            PidController::new(10.0, 0.0, 0.0),
            TrapezoidProfile::new(1.5, 3.0),
        )
    }

    fn drive(start_rad: f64, setpoint_rad: f64, steps: usize) -> f64 {
        let mut j = make_joint();
        j.set_setpoint(start_rad);
        for _ in 0..4000 {
            j.step(0.005);
        }
        j.set_setpoint(setpoint_rad);
        for _ in 0..steps {
            j.step(0.005);
        }
        j.angle_rad()
    }

    #[test]
    fn nearest_congruent_angle_no_wrap_needed() {
        assert!((nearest_congruent_angle(0.0, 1.0) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn nearest_congruent_angle_takes_short_path_across_pi() {
        // From just past +π, the nearest angle congruent to -π+0.1 should be
        // slightly ahead, not a whole lap behind.
        let reference = PI + 0.05;
        let target = -PI + 0.1;
        let result = nearest_congruent_angle(reference, target);
        assert!(
            (result - reference).abs() < 0.2,
            "expected a short hop, got delta={}",
            result - reference
        );
    }

    #[test]
    fn no_wrap_needed_stays_short() {
        let final_angle = drive(0.0, 1.0, 4000);
        assert!(
            (final_angle - 1.0).abs() < 0.05,
            "Expected ~1.0 rad, got {final_angle}"
        );
    }

    #[test]
    fn wrap_across_pi_takes_short_path() {
        let start = 170.0_f64.to_radians();
        let target = -170.0_f64.to_radians();
        let final_angle = drive(start, target, 4000);
        let err_deg = (final_angle.to_degrees() - target.to_degrees()).abs();
        let err_deg = if err_deg > 180.0 {
            360.0 - err_deg
        } else {
            err_deg
        };
        assert!(
            err_deg < 5.0,
            "Expected to reach -170°, err={err_deg:.1}°, final={:.1}°",
            final_angle.to_degrees()
        );
    }

    #[test]
    fn wrap_across_negative_pi() {
        let start = -170.0_f64.to_radians();
        let target = 170.0_f64.to_radians();
        let final_angle = drive(start, target, 4000);
        let err_deg = (final_angle.to_degrees() - target.to_degrees()).abs();
        let err_deg = if err_deg > 180.0 {
            360.0 - err_deg
        } else {
            err_deg
        };
        assert!(
            err_deg < 5.0,
            "Expected to reach +170°, err={err_deg:.1}°, final={:.1}°",
            final_angle.to_degrees()
        );
    }

    #[test]
    fn full_circle_wrap() {
        let start = 10.0_f64.to_radians();
        let target = 350.0_f64.to_radians();
        let final_angle = drive(start, target, 4000);
        let diff = (final_angle - target).abs();
        let diff = diff - (diff / (2.0 * PI)).round() * 2.0 * PI;
        assert!(
            diff.abs() < 0.1,
            "Expected ~350°, diff={:.2} rad, final={:.1}°",
            diff,
            final_angle.to_degrees()
        );
    }

    #[test]
    fn joint_limits_clamp_goal() {
        let mut j = Joint::new(
            1.0,
            0.0,
            MotorSim::with_params(MotorParams::default()),
            PidController::new(10.0, 0.0, 0.0),
            TrapezoidProfile::new(1.5, 3.0),
        )
        .with_limits(-PI / 4.0, PI / 4.0); // ±45°

        // Unambiguous: +90° is clearly above max, wrap keeps it positive, clamps to +45°.
        j.set_setpoint(PI / 2.0);
        assert!(
            (j.goal_rad() - PI / 4.0).abs() < 1e-9,
            "goal should clamp to max +45°, got {:.4}",
            j.goal_rad()
        );

        // Unambiguous: -90° is clearly below min, clamps to -45°.
        j.set_setpoint(-PI / 2.0);
        assert!(
            (j.goal_rad() - (-PI / 4.0)).abs() < 1e-9,
            "goal should clamp to min -45°, got {:.4}",
            j.goal_rad()
        );

        // Within limits: 20° stays untouched.
        j.set_setpoint(20.0_f64.to_radians());
        assert!(
            (j.goal_rad() - 20.0_f64.to_radians()).abs() < 1e-6,
            "goal within limits should not be clamped"
        );
    }

    #[test]
    fn set_gains_and_reset_integrator_round_trip() {
        let mut j = make_joint();
        j.set_gains(5.0, 0.5, 0.1, 0.2);
        assert_eq!(j.gains(), (5.0, 0.5, 0.1, 0.2));

        j.set_setpoint(1.0);
        for _ in 0..100 {
            j.step(0.005);
        }
        j.reset_integrator();
        assert_eq!(j.error_rad(), 0.0);
    }
}
