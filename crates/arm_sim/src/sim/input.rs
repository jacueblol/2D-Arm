use bevy::prelude::*;
use glam::DVec3;

use super::resources::SimState;

/// Target jog speed, m/s, while a key is held.
const TARGET_SPEED: f32 = 0.6;

pub fn handle_keyboard(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<SimState>,
    time: Res<Time>,
) {
    if keys.just_pressed(KeyCode::Space) {
        state.paused = !state.paused;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        state.arm.reset();
        state.ee_trail.clear();
        let t = state.target;
        state.arm.set_target(t);
    }

    // Move target with WASD (XZ plane) + Q/E (vertical).
    let dt = time.delta_secs();
    let spd = (TARGET_SPEED * dt) as f64;
    let mut delta = DVec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        delta.x += spd;
    }
    if keys.pressed(KeyCode::KeyS) {
        delta.x -= spd;
    }
    if keys.pressed(KeyCode::KeyA) {
        delta.z -= spd;
    }
    if keys.pressed(KeyCode::KeyD) {
        delta.z += spd;
    }
    if keys.pressed(KeyCode::KeyE) {
        delta.y += spd;
    }
    if keys.pressed(KeyCode::KeyQ) {
        delta.y -= spd;
    }
    if delta != DVec3::ZERO {
        let new_target = state.target + delta;
        state.set_target(new_target);
    }
}
