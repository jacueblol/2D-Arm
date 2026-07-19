use crate::{
    arm::trajectory::{TrapezoidProfile, TrapezoidState},
    motor::{motor_io::MotorFn, motor_sim::MotorSim},
    pid::pid_controller::PidController,
};

pub struct Joint3d {
    pub length: f64,
    pub link_mass: f64,
    motor: MotorSim,
    pid: PidController,
    profile: TrapezoidProfile,
    profile_state: TrapezoidState,
    goal_rad: f64,
    min_rad: f64,
    max_rad: f64,
    kf: f64,   // velocity feedforward gain (V per rad/s of profile velocity)
    prev_error: f64,
    integ_total: f64,
}

impl Joint3d {
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
            profile_state: TrapezoidState { position: 0.0, velocity: 0.0 },
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

    /// Set velocity feedforward gain. Adds `kf * profile_velocity` to motor voltage,
    /// reducing tracking lag when the profile is moving fast.
    pub fn with_velocity_ff(mut self, kf: f64) -> Self {
        self.kf = kf;
        self
    }

    /// Set the commanded goal angle. Trajectory profile ramps toward it; goal is clamped to limits.
    pub fn set_setpoint(&mut self, rad: f64) {
        let pos = self.motor.get_position_rad();
        let raw = rad - pos;
        let wrapped = raw - (raw / std::f64::consts::TAU).round() * std::f64::consts::TAU;
        self.goal_rad = (pos + wrapped).clamp(self.min_rad, self.max_rad);
        self.profile_state = TrapezoidState {
            position: pos,
            velocity: self.motor.get_velocity_rad_s(),
        };
    }

    /// Instantaneous profile setpoint (ramping toward goal_rad).
    pub fn get_setpoint_rad(&self) -> f64 { self.profile_state.position }
    #[allow(dead_code)]
    pub fn get_goal_rad(&self)    -> f64  { self.goal_rad }

    /// Updates the goal without resetting trajectory profile velocity.
    /// Use for continuous Cartesian tracking: the goal moves a tiny step each frame
    /// and resetting velocity would cause the arm to brake at every waypoint.
    pub fn update_goal(&mut self, rad: f64) {
        let pos = self.motor.get_position_rad();
        let raw = rad - pos;
        let wrapped = raw - (raw / std::f64::consts::TAU).round() * std::f64::consts::TAU;
        self.goal_rad = (pos + wrapped).clamp(self.min_rad, self.max_rad);
        self.profile_state.position = pos;
        // profile_state.velocity is intentionally kept — continuous motion stays smooth.
    }
    pub fn angle_rad(&self)       -> f64  { self.motor.get_position_rad() }
    pub fn error_rad(&self)       -> f64  { self.prev_error }
    pub fn velocity_rad_s(&self)  -> f64  { self.motor.get_velocity_rad_s() }

    pub fn set_load_torque(&mut self, torque: f64) {
        self.motor.set_load_torque(torque);
    }

    pub fn step(&mut self, dt: f64) {
        let current = TrapezoidState {
            position: self.profile_state.position,
            velocity: self.profile_state.velocity,
        };
        self.profile_state = self.profile.calculate(dt, current, self.goal_rad);

        let pos = self.motor.get_position_rad();
        let raw = self.profile_state.position - pos;
        let wrapped = raw - (raw / std::f64::consts::TAU).round() * std::f64::consts::TAU;
        let effective_sp = pos + wrapped;

        let out = self.pid.step(dt, effective_sp, pos, self.prev_error, self.integ_total);
        self.prev_error  = out.error;
        self.integ_total = out.integ_total;

        let gravity_ff = self.motor.gravity_feedforward_volts();
        let vel_ff     = self.kf * self.profile_state.velocity;
        self.motor.set_voltage(out.output + gravity_ff + vel_ff);
        self.motor.step(dt);
    }

    pub fn reset(&mut self) {
        self.motor.reset();
        self.prev_error  = 0.0;
        self.integ_total = 0.0;
        self.profile_state = TrapezoidState { position: 0.0, velocity: 0.0 };
        self.goal_rad = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::motor_sim::MotorParams;
    use std::f64::consts::PI;

    fn make_joint() -> Joint3d {
        Joint3d::new(
            1.0, 0.0,
            MotorSim::new(),
            PidController::new(10.0, 0.0, 0.0),
            TrapezoidProfile::new(1.5, 3.0),
        )
    }

    fn drive(start_rad: f64, setpoint_rad: f64, steps: usize) -> f64 {
        let mut j = make_joint();
        j.set_setpoint(start_rad);
        for _ in 0..4000 { j.step(0.005); }
        j.set_setpoint(setpoint_rad);
        for _ in 0..steps { j.step(0.005); }
        j.angle_rad()
    }

    #[test]
    fn no_wrap_needed_stays_short() {
        let final_angle = drive(0.0, 1.0, 4000);
        assert!((final_angle - 1.0).abs() < 0.05, "Expected ~1.0 rad, got {final_angle}");
    }

    #[test]
    fn wrap_across_pi_takes_short_path() {
        let start  =  170.0_f64.to_radians();
        let target = -170.0_f64.to_radians();
        let final_angle = drive(start, target, 4000);
        let err_deg = (final_angle.to_degrees() - target.to_degrees()).abs();
        let err_deg = if err_deg > 180.0 { 360.0 - err_deg } else { err_deg };
        assert!(err_deg < 5.0, "Expected to reach -170°, err={err_deg:.1}°, final={:.1}°", final_angle.to_degrees());
    }

    #[test]
    fn wrap_across_negative_pi() {
        let start  = -170.0_f64.to_radians();
        let target =  170.0_f64.to_radians();
        let final_angle = drive(start, target, 4000);
        let err_deg = (final_angle.to_degrees() - target.to_degrees()).abs();
        let err_deg = if err_deg > 180.0 { 360.0 - err_deg } else { err_deg };
        assert!(err_deg < 5.0, "Expected to reach +170°, err={err_deg:.1}°, final={:.1}°", final_angle.to_degrees());
    }

    #[test]
    fn full_circle_wrap() {
        let start  = 10.0_f64.to_radians();
        let target = 350.0_f64.to_radians();
        let final_angle = drive(start, target, 4000);
        let diff = (final_angle - target).abs();
        let diff = diff - (diff / (2.0 * PI)).round() * 2.0 * PI;
        assert!(diff.abs() < 0.1, "Expected ~350°, diff={:.2} rad, final={:.1}°", diff, final_angle.to_degrees());
    }

    #[test]
    fn joint_limits_clamp_goal() {
        let mut j = Joint3d::new(
            1.0, 0.0,
            MotorSim::with_params(MotorParams::default()),
            PidController::new(10.0, 0.0, 0.0),
            TrapezoidProfile::new(1.5, 3.0),
        ).with_limits(-PI / 4.0, PI / 4.0);  // ±45°

        // Unambiguous: +90° is clearly above max, wrap keeps it positive, clamps to +45°.
        j.set_setpoint(PI / 2.0);
        assert!((j.get_goal_rad() - PI / 4.0).abs() < 1e-9,
            "goal should clamp to max +45°, got {:.4}", j.get_goal_rad());

        // Unambiguous: -90° is clearly below min, clamps to -45°.
        j.set_setpoint(-PI / 2.0);
        assert!((j.get_goal_rad() - (-PI / 4.0)).abs() < 1e-9,
            "goal should clamp to min -45°, got {:.4}", j.get_goal_rad());

        // Within limits: 20° stays untouched.
        j.set_setpoint(20.0_f64.to_radians());
        assert!((j.get_goal_rad() - 20.0_f64.to_radians()).abs() < 1e-6,
            "goal within limits should not be clamped");
    }
}
