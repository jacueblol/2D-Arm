use bevy::prelude::*;

use super::resources::SimState;

pub fn draw_gizmos(state: Res<SimState>, mut gizmos: Gizmos) {
    // EE trail — fades in from the oldest sample.
    let trail: Vec<Vec3> = state.ee_trail.iter().copied().collect();
    let len = trail.len();
    for (i, pair) in trail.windows(2).enumerate() {
        let alpha = (i + 1) as f32 / len as f32;
        gizmos.line(pair[0], pair[1], Color::srgba(1.0, 0.75, 0.15, alpha * 0.9));
    }

    // Workspace outer/inner reachability boundary.
    let l1 = state.arm.shoulder.length as f32;
    let l2 = state.arm.elbow.length as f32;
    gizmos.sphere(
        Isometry3d::IDENTITY,
        l1 + l2,
        Color::srgba(0.3, 0.3, 0.6, 0.15),
    );
    if (l1 - l2).abs() > 0.01 {
        gizmos.sphere(
            Isometry3d::IDENTITY,
            (l1 - l2).abs(),
            Color::srgba(0.3, 0.3, 0.6, 0.15),
        );
    }

    // Target crosshair — red while reachable, gray once jogged past the
    // workspace boundary (state.target then holds the last valid point).
    let tp = state.target.as_vec3();
    let tc = if state.ik_valid {
        Color::srgb(0.95, 0.2, 0.2)
    } else {
        Color::srgb(0.5, 0.5, 0.5)
    };
    gizmos.circle(
        Isometry3d::new(tp, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        0.12,
        tc,
    );
    gizmos.line(
        tp - Vec3::new(0.15, 0.0, 0.0),
        tp + Vec3::new(0.15, 0.0, 0.0),
        tc,
    );
    gizmos.line(
        tp - Vec3::new(0.0, 0.15, 0.0),
        tp + Vec3::new(0.0, 0.15, 0.0),
        tc,
    );
    gizmos.line(
        tp - Vec3::new(0.0, 0.0, 0.15),
        tp + Vec3::new(0.0, 0.0, 0.15),
        tc,
    );

    // Floor grid.
    let grid_col = Color::srgba(0.3, 0.3, 0.3, 0.3);
    for i in -5i32..=5 {
        let f = i as f32 * 0.5;
        gizmos.line(Vec3::new(f, 0.0, -2.5), Vec3::new(f, 0.0, 2.5), grid_col);
        gizmos.line(Vec3::new(-2.5, 0.0, f), Vec3::new(2.5, 0.0, f), grid_col);
    }
}
