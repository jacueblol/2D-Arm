use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;

use super::resources::OrbitCam;

#[derive(Component)]
pub struct MainCamera;

pub fn handle_camera(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut motion: MessageReader<MouseMotion>,
    mut wheel: MessageReader<MouseWheel>,
    mut orbit: ResMut<OrbitCam>,
    mut cam_query: Query<&mut Transform, With<MainCamera>>,
) {
    let Ok(mut cam) = cam_query.single_mut() else {
        return;
    };

    for ev in wheel.read() {
        orbit.distance = (orbit.distance - ev.y * orbit.distance * 0.1).clamp(0.4, 20.0);
    }

    if mouse_buttons.pressed(MouseButton::Left) {
        for ev in motion.read() {
            orbit.yaw -= ev.delta.x * 0.007;
            orbit.pitch = (orbit.pitch - ev.delta.y * 0.007).clamp(
                -std::f32::consts::FRAC_PI_2 + 0.05,
                std::f32::consts::FRAC_PI_2 - 0.05,
            );
        }
    } else {
        for _ in motion.read() {}
    }

    cam.translation = orbit.camera_pos();
    cam.look_at(orbit.focus, Vec3::Y);
}
