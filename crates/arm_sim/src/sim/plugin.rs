use bevy::prelude::*;

use super::resources::SimState;
use super::scene::{setup, update_status, update_visuals};

pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, setup)
            .add_systems(Update, (step_sim, update_visuals, update_status).chain());
    }
}

fn step_sim(mut state: ResMut<SimState>, time: Res<Time>) {
    // Clamp dt so a debugger pause or a slow frame doesn't blow up the
    // integrator with a huge single step.
    let dt = time.delta_secs_f64().min(0.05);
    if dt > 0.0 {
        state.arm.step(dt);
    }
}
