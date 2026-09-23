use glam::{DQuat, DVec3};

/// Forward kinematics for the 3-DOF pan/shoulder/elbow arm.
///
/// Convention: pan=0 → arm extends along +X; positive pan rotates the arm
/// toward +Z; positive shoulder tilts upward; elbow bends relative to the
/// shoulder (same rotation axis).
///
/// Returns `[base, shoulder, elbow, end_effector]` in world space (metres).
/// The base and shoulder joint coincide at the origin — pan rotates about
/// the vertical axis without translating anything.
pub fn forward_kinematics(
    pan_rad: f64,
    shoulder_rad: f64,
    elbow_rad: f64,
    shoulder_len: f64,
    elbow_len: f64,
) -> [DVec3; 4] {
    // Negate pan so a positive angle sweeps the arm toward +Z (a right-hand
    // rotation about +Y sweeps toward -Z, which reads backwards).
    let r_pan = DQuat::from_rotation_y(-pan_rad);
    let r_sho = r_pan * DQuat::from_rotation_z(shoulder_rad);
    let r_elb = r_sho * DQuat::from_rotation_z(elbow_rad);

    let base = DVec3::ZERO;
    let shoulder = DVec3::ZERO;
    let elbow = shoulder + r_sho * DVec3::new(shoulder_len, 0.0, 0.0);
    let ee = elbow + r_elb * DVec3::new(elbow_len, 0.0, 0.0);

    [base, shoulder, elbow, ee]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_angles_extend_straight_along_x() {
        let [base, shoulder, elbow, ee] = forward_kinematics(0.0, 0.0, 0.0, 1.0, 0.8);
        assert_eq!(base, DVec3::ZERO);
        assert_eq!(shoulder, DVec3::ZERO);
        assert!((elbow - DVec3::new(1.0, 0.0, 0.0)).length() < 1e-12);
        assert!((ee - DVec3::new(1.8, 0.0, 0.0)).length() < 1e-12);
    }

    #[test]
    fn positive_pan_sweeps_toward_positive_z() {
        let [.., ee] = forward_kinematics(std::f64::consts::FRAC_PI_2, 0.0, 0.0, 1.0, 0.8);
        assert!(ee.x.abs() < 1e-9, "expected x~0 at 90° pan, got {}", ee.x);
        assert!(ee.z > 0.0, "expected +Z at positive pan, got z={}", ee.z);
    }

    #[test]
    fn positive_shoulder_tilts_upward() {
        let [.., ee] = forward_kinematics(0.0, std::f64::consts::FRAC_PI_2, 0.0, 1.0, 0.8);
        assert!(ee.y > 0.0, "expected +Y at 90° shoulder, got y={}", ee.y);
    }

    #[test]
    fn elbow_bends_relative_to_shoulder() {
        // Shoulder straight up (+Y): elbow joint sits at (0, l1, 0). Bending
        // the elbow by exactly -shoulder cancels the shoulder's rotation
        // for the second link, so it points back out horizontally (+X) from
        // that elevated elbow joint, landing at (l2, l1, 0).
        let sho = std::f64::consts::FRAC_PI_2;
        let [_, _, elbow, ee] = forward_kinematics(0.0, sho, -sho, 1.0, 0.8);
        assert!(
            (elbow - DVec3::new(0.0, 1.0, 0.0)).length() < 1e-9,
            "elbow={elbow:?}"
        );
        assert!(
            (ee - DVec3::new(0.8, 1.0, 0.0)).length() < 1e-9,
            "ee={ee:?}"
        );
    }
}
