use std::collections::VecDeque;

use arm_core::arm::Arm;
use arm_core::trajectory::WaypointSeq;
use bevy::prelude::*;
use glam::DVec3;

pub const TRAIL_LEN: usize = 400;

/// EE linear speed for Cartesian moves triggered by jogging the target or
/// launching a choreography sequence.
pub const CARTESIAN_SPEED: f64 = 0.3;

#[derive(Resource)]
pub struct SimState {
    pub arm: Arm,
    pub target: DVec3,
    pub ik_valid: bool,
    pub paused: bool,
    pub ee_trail: VecDeque<Vec3>,
    pub active_choreo: Option<WaypointSeq>,
}

impl SimState {
    pub fn new(arm: Arm, target: DVec3, ik_valid: bool) -> Self {
        Self {
            arm,
            target,
            ik_valid,
            paused: false,
            ee_trail: VecDeque::new(),
            active_choreo: None,
        }
    }

    /// Attempt to retarget the arm via a straight-line Cartesian move. On
    /// success, updates `target` and clears the trail (a fresh move
    /// shouldn't show the old approach path). On failure (target outside
    /// the reachable workspace), leaves `target` where it was — jogging
    /// past the workspace boundary just stops.
    pub fn set_target(&mut self, new_target: DVec3) {
        if self.arm.set_target(new_target) {
            self.arm.start_cartesian_move(new_target, CARTESIAN_SPEED);
            self.target = new_target;
            self.ik_valid = true;
            self.ee_trail.clear();
        } else {
            self.ik_valid = false;
        }
    }
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

#[derive(Resource)]
pub struct OrbitCam {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub focus: Vec3,
}

impl OrbitCam {
    pub fn camera_pos(&self) -> Vec3 {
        Vec3::new(
            self.focus.x + self.distance * self.pitch.cos() * self.yaw.sin(),
            self.focus.y + self.distance * self.pitch.sin(),
            self.focus.z + self.distance * self.pitch.cos() * self.yaw.cos(),
        )
    }
}

impl Default for OrbitCam {
    fn default() -> Self {
        Self {
            yaw: std::f32::consts::FRAC_PI_4,
            pitch: 0.45,
            distance: 4.5,
            focus: Vec3::new(0.6, 0.4, 0.2),
        }
    }
}
