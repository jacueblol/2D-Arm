use arm_core::trajectory::choreography;
use bevy::log::info;
use bevy::prelude::*;
use glam::DVec3;

use super::resources::{CARTESIAN_SPEED, SimState};

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
        state.active_choreo = None;
        let t = state.target;
        state.arm.set_target(t);
        state.arm.start_cartesian_move(t, CARTESIAN_SPEED);
    }

    // Choreography shortcuts — 1-9 launch sequences, C cancels.
    let launch: Option<choreography::WaypointSeq> = if keys.just_pressed(KeyCode::Digit1) {
        Some(choreography::figure_eight(0.25))
    } else if keys.just_pressed(KeyCode::Digit2) {
        Some(choreography::circle_sweep(0.25))
    } else if keys.just_pressed(KeyCode::Digit3) {
        Some(choreography::helix(0.25))
    } else if keys.just_pressed(KeyCode::Digit4) {
        Some(choreography::wave_hello(0.30))
    } else if keys.just_pressed(KeyCode::Digit5) {
        Some(choreography::pick_and_place(0.25))
    } else if keys.just_pressed(KeyCode::Digit6) {
        Some(choreography::triangle(0.25))
    } else if keys.just_pressed(KeyCode::Digit7) {
        Some(choreography::knock_knock(0.25))
    } else if keys.just_pressed(KeyCode::Digit8) {
        Some(choreography::slow_drift(0.25))
    } else if keys.just_pressed(KeyCode::Digit9) {
        Some(choreography::zorro(0.25))
    } else {
        None
    };
    if let Some(mut seq) = launch {
        info!("Starting choreography: {}", seq.label);
        seq.start(&mut state.arm);
        state.active_choreo = Some(seq);
    }
    if keys.just_pressed(KeyCode::KeyC) {
        if state.active_choreo.is_some() {
            info!("Cancelled choreography");
        }
        state.active_choreo = None;
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
        // Manual move cancels any active choreography.
        state.active_choreo = None;
    }
}
