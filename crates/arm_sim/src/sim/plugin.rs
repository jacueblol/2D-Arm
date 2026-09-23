use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use super::camera::handle_camera;
use super::gizmos::draw_gizmos;
use super::input::handle_keyboard;
use super::resources::{OrbitCam, SimState, TRAIL_LEN};
use super::scene::{setup, update_status, update_visuals};
use super::ui::{PlotHistory, UiState, draw_panel, setup_egui_theme, update_plot_history};

pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<OrbitCam>()
            .init_resource::<PlotHistory>()
            .init_resource::<UiState>()
            .add_systems(Startup, setup)
            .add_systems(
                Update,
                (
                    handle_keyboard,
                    handle_camera,
                    step_sim,
                    run_choreography,
                    update_plot_history,
                    update_visuals,
                    draw_gizmos,
                    update_status,
                )
                    .chain(),
            )
            .add_systems(
                EguiPrimaryContextPass,
                (setup_egui_theme, draw_panel).chain(),
            );
    }
}

fn step_sim(mut state: ResMut<SimState>, time: Res<Time>) {
    if state.paused {
        return;
    }
    // Clamp dt so a debugger pause or a slow frame doesn't blow up the
    // integrator with a huge single step.
    let dt = time.delta_secs_f64().min(0.05);
    if dt > 0.0 {
        state.arm.step(dt);
    }

    let ee = state.arm.ee_pos().as_vec3();
    state.ee_trail.push_back(ee);
    if state.ee_trail.len() > TRAIL_LEN {
        state.ee_trail.pop_front();
    }
}

fn run_choreography(mut state: ResMut<SimState>, time: Res<Time>) {
    if state.paused || state.active_choreo.is_none() {
        return;
    }
    let dt = time.delta_secs_f64().min(0.05);
    let dist = {
        let state = &*state;
        state
            .active_choreo
            .as_ref()
            .and_then(|c| c.current_target())
            .map(|t| (state.arm.ee_pos() - t).length())
            .unwrap_or(0.0)
    };

    let SimState {
        arm, active_choreo, ..
    } = &mut *state;
    if let Some(choreo) = active_choreo {
        choreo.advance(arm, dist, dt);
        if choreo.is_done() {
            *active_choreo = None;
        }
    }
}
