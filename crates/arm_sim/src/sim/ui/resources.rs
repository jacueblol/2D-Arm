use std::collections::VecDeque;

use bevy::prelude::*;

use super::super::resources::SimState;

pub const PLOT_HISTORY_LEN: usize = 300;

/// Ring-buffer of time-series telemetry for all three joints, shared X axis.
/// Index: 0=Pan, 1=Shoulder, 2=Elbow.
#[derive(Resource)]
pub struct PlotHistory {
    pub angle: [VecDeque<f32>; 3],
    pub sp: [VecDeque<f32>; 3],
    pub err: [VecDeque<f32>; 3],
    pub vel: [VecDeque<f32>; 3],
    pub times: VecDeque<f32>,
    max_len: usize,
    elapsed_s: f32,
}

impl PlotHistory {
    pub fn new(max_len: usize) -> Self {
        Self {
            angle: std::array::from_fn(|_| VecDeque::new()),
            sp: std::array::from_fn(|_| VecDeque::new()),
            err: std::array::from_fn(|_| VecDeque::new()),
            vel: std::array::from_fn(|_| VecDeque::new()),
            times: VecDeque::new(),
            max_len,
            elapsed_s: 0.0,
        }
    }

    fn push_clamped(buf: &mut VecDeque<f32>, val: f32, max_len: usize) {
        buf.push_back(val);
        if buf.len() > max_len {
            buf.pop_front();
        }
    }

    pub fn push(&mut self, state: &SimState, dt: f32) {
        self.elapsed_s += dt;
        let ml = self.max_len;
        let joints = [&state.arm.pan, &state.arm.shoulder, &state.arm.elbow];
        for (i, j) in joints.iter().enumerate() {
            Self::push_clamped(&mut self.angle[i], j.angle_rad() as f32, ml);
            Self::push_clamped(&mut self.sp[i], j.setpoint_rad() as f32, ml);
            Self::push_clamped(&mut self.err[i], j.error_rad() as f32, ml);
            Self::push_clamped(&mut self.vel[i], j.velocity_rad_s() as f32, ml);
        }
        Self::push_clamped(&mut self.times, self.elapsed_s, ml);
    }
}

impl Default for PlotHistory {
    fn default() -> Self {
        Self::new(PLOT_HISTORY_LEN)
    }
}

/// Which joint tab and top-level tab (telemetry vs tuning) the panel shows.
#[derive(Resource, Default)]
pub struct UiState {
    pub selected_joint: usize, // 0=Pan, 1=Shoulder, 2=Elbow
    pub tuning_tab: bool,      // false=Telemetry, true=Tuning
}

pub fn update_plot_history(
    state: Res<SimState>,
    mut history: ResMut<PlotHistory>,
    time: Res<Time>,
) {
    if state.paused {
        return;
    }
    history.push(&state, time.delta_secs());
}
