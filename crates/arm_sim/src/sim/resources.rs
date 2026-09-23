use arm_core::arm::Arm;
use bevy::prelude::*;
use glam::DVec3;

#[derive(Resource)]
pub struct SimState {
    pub arm: Arm,
    pub target: DVec3,
}

/// Handles to the entities `scene::update_visuals` repositions each frame.
#[derive(Resource)]
pub struct VisualEntities {
    pub link1: Entity,
    pub link2: Entity,
    pub elbow: Entity,
    pub ee: Entity,
    pub target_sphere: Entity,
}
