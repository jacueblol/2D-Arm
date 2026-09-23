//! Integration tests driven by the real `config.toml` at the repo root —
//! not a hand-duplicated copy of its values. If the config changes, these
//! tests feel it immediately instead of silently drifting out of sync.

use arm_core::arm::Arm;
use arm_core::config::Config;
use glam::DVec3;

fn make_arm_from_config() -> Arm {
    let cfg = Config::embedded_default();
    Arm::new(
        cfg.pan.build(cfg.motor.to_motor_params()),
        cfg.shoulder.build(cfg.motor.to_motor_params()),
        cfg.elbow.build(cfg.motor.to_motor_params()),
    )
}

#[test]
fn shoulder_joint_from_real_config_settles_near_its_own_setpoint() {
    let cfg = Config::embedded_default();
    let motor_params = cfg.motor.to_motor_params();
    let mut shoulder = cfg.shoulder.build(motor_params);

    let target_deg = 45.0_f64;
    shoulder.set_setpoint(target_deg.to_radians());

    // These gains (kp=10, ki=0.8, kd=1.0) overshoot to ~47° by t=4s and then
    // settle with a long decaying tail — checked empirically before picking
    // this bound: 25 simulated seconds at a 5ms step lands within ~0.5°.
    for _ in 0..5000 {
        shoulder.step(0.005);
    }

    let final_deg = shoulder.angle_rad().to_degrees();
    assert!(
        (final_deg - target_deg).abs() < 1.0,
        "shoulder should settle near {target_deg}°, got {final_deg:.3}°"
    );
}

#[test]
fn elbow_joint_from_real_config_respects_its_configured_limits() {
    let cfg = Config::embedded_default();
    let motor_params = cfg.motor.to_motor_params();
    let mut elbow = cfg.elbow.build(motor_params);

    // Command well past the configured max (elbow max is 10 degrees).
    elbow.set_setpoint(90.0_f64.to_radians());
    let max_rad = cfg.elbow.max_angle_deg.to_radians();
    assert!(
        elbow.goal_rad() <= max_rad + 1e-9,
        "goal should be clamped to the configured max ({} deg), got {} deg",
        cfg.elbow.max_angle_deg,
        elbow.goal_rad().to_degrees()
    );
}

#[test]
fn pan_joint_from_real_config_has_no_gravity_load() {
    // Pan's link_length_m/link_mass_kg are 0 in config.toml (it's a
    // vertical-axis rotation) — a joint built straight from config with no
    // externally-applied load torque should have zero gravity feedforward.
    let cfg = Config::embedded_default();
    let pan = cfg.pan.build(cfg.motor.to_motor_params());
    assert_eq!(pan.link_mass, 0.0);
}

fn settle_dist(arm: &mut Arm, target: DVec3, steps: usize) -> f64 {
    arm.set_target(target);
    for _ in 0..steps {
        arm.step(0.005);
    }
    (arm.ee_pos() - target).length()
}

#[test]
fn arm_converges_forward_reach() {
    // Easy case: mostly forward, moderate gravity load.
    let mut arm = make_arm_from_config();
    let dist = settle_dist(&mut arm, DVec3::new(1.2, 0.8, 0.4), 6000); // 30 simulated seconds
    assert!(
        dist < 0.015,
        "expected <15mm SS error, got {:.1}mm",
        dist * 1000.0
    );
}

#[test]
fn arm_converges_lateral_reach() {
    // Harder: significant pan + gravity loading, integral needs more time.
    let mut arm = make_arm_from_config();
    let dist = settle_dist(&mut arm, DVec3::new(0.7, 0.6, 0.5), 6000);
    assert!(
        dist < 0.02,
        "expected <20mm SS error, got {:.1}mm",
        dist * 1000.0
    );
}

#[test]
fn arm_converges_high_target() {
    // Hardest: high shoulder elevation, maximum gravity torque on shoulder.
    let mut arm = make_arm_from_config();
    let dist = settle_dist(&mut arm, DVec3::new(0.8, 1.2, 0.0), 6000);
    assert!(
        dist < 0.02,
        "expected <20mm SS error, got {:.1}mm",
        dist * 1000.0
    );
}

#[test]
fn arm_set_target_unreachable_returns_false() {
    let mut arm = make_arm_from_config();
    // Shoulder+elbow max reach is 1.8m; this is far outside it.
    assert!(!arm.set_target(DVec3::new(10.0, 10.0, 10.0)));
}
