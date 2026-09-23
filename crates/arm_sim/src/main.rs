mod config;
mod sim;

use arm_core::arm::Arm;
use bevy::log::info;
use bevy::prelude::*;
use bevy::window::WindowResolution;
use bevy_egui::EguiPlugin;
use glam::DVec3;

use sim::{SimPlugin, SimState};

fn main() {
    let cfg = config::load_config();

    let mut arm = Arm::new(
        cfg.pan.build(cfg.motor.to_motor_params()),
        cfg.shoulder.build(cfg.motor.to_motor_params()),
        cfg.elbow.build(cfg.motor.to_motor_params()),
    );

    let initial_target = DVec3::from_array(cfg.sim.initial_target);
    let reached = arm.set_target(initial_target);
    info!(
        "Initial target ({:.2},{:.2},{:.2}) — pan={:.1} sho={:.1} elb={:.1} deg (reachable: {reached})",
        initial_target.x,
        initial_target.y,
        initial_target.z,
        arm.pan.goal_rad().to_degrees(),
        arm.shoulder.goal_rad().to_degrees(),
        arm.elbow.goal_rad().to_degrees(),
    );

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Articulated Arm Simulator".into(),
                resolution: WindowResolution::new(1280, 960),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .insert_resource(SimState::new(arm, initial_target, reached))
        .add_plugins(SimPlugin)
        .run();
}
