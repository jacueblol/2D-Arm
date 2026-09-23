use std::collections::VecDeque;

use bevy_egui::EguiContexts;
use bevy_egui::egui::{self, Color32};
use glam::DVec3;

use super::super::resources::SimState;
use super::resources::{PlotHistory, UiState};
use bevy::prelude::*;

const PANEL_WIDTH: f32 = 380.0;
const JOINT_LABELS: [&str; 3] = ["Pan", "Shoulder", "Elbow"];

pub fn setup_egui_theme(mut contexts: EguiContexts) -> Result {
    let ctx = contexts.ctx_mut()?;
    let mut visuals = egui::Visuals::dark();
    visuals.selection.bg_fill = Color32::from_rgb(50, 100, 200);
    visuals.widgets.active.bg_fill = Color32::from_rgb(40, 80, 160);
    ctx.set_visuals(visuals);
    Ok(())
}

fn time_plot_points<'a>(times: &VecDeque<f32>, vals: &VecDeque<f32>) -> egui_plot::PlotPoints<'a> {
    times
        .iter()
        .zip(vals.iter())
        .map(|(&t, &v)| [t as f64, v as f64])
        .collect()
}

pub fn draw_panel(
    mut contexts: EguiContexts,
    history: Res<PlotHistory>,
    mut state: ResMut<SimState>,
    mut ui_state: ResMut<UiState>,
) -> Result {
    let ctx = contexts.ctx_mut()?;

    // egui's Panel type (unifying the old SidePanel/TopBottomPanel) draws
    // into an existing Ui rather than straight onto the Context, so we need
    // a full-screen background Ui to host it in.
    let mut viewport_ui = egui::Ui::new(
        ctx.clone(),
        "viewport".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );

    egui::Panel::right("charts_panel")
        .exact_size(PANEL_WIDTH)
        .resizable(false)
        .show(&mut viewport_ui, |ui| {
            ui.add_space(6.0);
            ui.heading("Joint Telemetry");
            ui.separator();

            ui.horizontal(|ui| {
                if ui
                    .selectable_label(!ui_state.tuning_tab, "Telemetry")
                    .clicked()
                {
                    ui_state.tuning_tab = false;
                }
                if ui.selectable_label(ui_state.tuning_tab, "Tuning").clicked() {
                    ui_state.tuning_tab = true;
                }
            });
            ui.separator();

            ui.horizontal(|ui| {
                for (i, label) in JOINT_LABELS.iter().enumerate() {
                    if ui
                        .selectable_label(ui_state.selected_joint == i, *label)
                        .clicked()
                    {
                        ui_state.selected_joint = i;
                    }
                }
            });
            ui.separator();

            let ji = ui_state.selected_joint;

            egui::ScrollArea::vertical().show(ui, |ui| {
                if ui_state.tuning_tab {
                    draw_tuning_tab(ui, &mut state, ji);
                } else {
                    draw_telemetry_tab(ui, &history, ji);
                }

                ui.separator();
                draw_target_controls(ui, &mut state);

                ui.separator();
                ui.label(
                    egui::RichText::new(
                        "WASD/QE move · 1-9 choreography · C cancel · Space pause · R reset",
                    )
                    .weak()
                    .small(),
                );
            });
        });

    Ok(())
}

fn draw_telemetry_tab(ui: &mut egui::Ui, history: &PlotHistory, ji: usize) {
    ui.label("Angle vs Setpoint (rad)");
    egui_plot::Plot::new(format!("angle_{ji}"))
        .height(110.0)
        .allow_zoom(false)
        .allow_scroll(false)
        .x_axis_label("t (s)")
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new(
                    "angle",
                    time_plot_points(&history.times, &history.angle[ji]),
                )
                .color(Color32::from_rgb(100, 180, 255)),
            );
            plot_ui.line(
                egui_plot::Line::new(
                    "setpoint",
                    time_plot_points(&history.times, &history.sp[ji]),
                )
                .color(Color32::from_rgb(255, 200, 80)),
            );
        });

    ui.label("Error (rad)");
    egui_plot::Plot::new(format!("err_{ji}"))
        .height(80.0)
        .allow_zoom(false)
        .allow_scroll(false)
        .x_axis_label("t (s)")
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new("error", time_plot_points(&history.times, &history.err[ji]))
                    .color(Color32::from_rgb(255, 100, 100)),
            );
        });

    ui.label("Velocity (rad/s)");
    egui_plot::Plot::new(format!("vel_{ji}"))
        .height(80.0)
        .allow_zoom(false)
        .allow_scroll(false)
        .x_axis_label("t (s)")
        .show(ui, |plot_ui| {
            plot_ui.line(
                egui_plot::Line::new(
                    "velocity",
                    time_plot_points(&history.times, &history.vel[ji]),
                )
                .color(Color32::from_rgb(100, 220, 130)),
            );
        });
}

fn draw_tuning_tab(ui: &mut egui::Ui, state: &mut SimState, ji: usize) {
    let joint = match ji {
        0 => &mut state.arm.pan,
        1 => &mut state.arm.shoulder,
        _ => &mut state.arm.elbow,
    };
    let (mut kp, mut ki, mut kd, mut kf) = joint.gains();

    ui.group(|ui| {
        ui.label("PID Gains");
        let mut changed = false;

        ui.horizontal(|ui| {
            ui.label("kp:");
            changed |= ui
                .add(egui::Slider::new(&mut kp, 0.0..=50.0).step_by(0.1))
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("ki:");
            changed |= ui
                .add(egui::Slider::new(&mut ki, 0.0..=10.0).step_by(0.01))
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("kd:");
            changed |= ui
                .add(egui::Slider::new(&mut kd, 0.0..=10.0).step_by(0.01))
                .changed();
        });
        ui.horizontal(|ui| {
            ui.label("kf:");
            changed |= ui
                .add(egui::Slider::new(&mut kf, 0.0..=5.0).step_by(0.01))
                .changed();
        });

        if changed {
            joint.set_gains(kp, ki, kd, kf);
        }

        ui.add_space(4.0);
        if ui.button("Reset Integrator").clicked() {
            joint.reset_integrator();
        }
    });

    ui.add_space(4.0);
    ui.label(
        egui::RichText::new("Changes apply immediately")
            .weak()
            .small(),
    );
}

fn draw_target_controls(ui: &mut egui::Ui, state: &mut SimState) {
    ui.label("Target position (m)");
    let mut tgt = [
        state.target.x as f32,
        state.target.y as f32,
        state.target.z as f32,
    ];
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label("X:");
        changed |= ui
            .add(
                egui::DragValue::new(&mut tgt[0])
                    .speed(0.01)
                    .range(0.0..=2.0),
            )
            .changed();
        ui.label("Y:");
        changed |= ui
            .add(
                egui::DragValue::new(&mut tgt[1])
                    .speed(0.01)
                    .range(0.0..=2.0),
            )
            .changed();
        ui.label("Z:");
        changed |= ui
            .add(
                egui::DragValue::new(&mut tgt[2])
                    .speed(0.01)
                    .range(-1.0..=1.0),
            )
            .changed();
    });
    if changed {
        let new_target = DVec3::new(tgt[0] as f64, tgt[1] as f64, tgt[2] as f64);
        state.set_target(new_target);
        state.active_choreo = None;
    }
}
