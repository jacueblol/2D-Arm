pub struct TrapezoidState {
    pub position: f64,
    pub velocity: f64,
}

pub struct TrapezoidProfile {
    pub max_vel: f64,
    pub max_accel: f64,
}

impl TrapezoidProfile {
    pub fn new(max_vel: f64, max_accel: f64) -> Self {
        Self { max_vel, max_accel }
    }

    pub fn calculate(&self, dt: f64, current: TrapezoidState, goal: f64) -> TrapezoidState {
        let error = goal - current.position;

        if error.abs() < 1e-9 && current.velocity.abs() < 1e-6 {
            return TrapezoidState { position: goal, velocity: 0.0 };
        }

        let sign = if error >= 0.0 { 1.0 } else { -1.0 };

        // Distance needed to brake current velocity to zero.
        let stopping_dist = current.velocity.abs().powi(2) / (2.0 * self.max_accel);
        let moving_toward_goal = current.velocity * sign > 0.0;

        let target_accel = if moving_toward_goal && stopping_dist >= error.abs() {
            -sign * self.max_accel  // decelerate to stop at goal
        } else {
            sign * self.max_accel   // accelerate toward goal
        };

        let mut new_vel = (current.velocity + target_accel * dt).clamp(-self.max_vel, self.max_vel);
        let new_pos = current.position + new_vel * dt;

        // Clamp if we overshot.
        if (new_pos - goal) * sign > 0.0 {
            return TrapezoidState { position: goal, velocity: 0.0 };
        }

        // Damp velocity if we're at the goal but still drifting.
        if (goal - new_pos).abs() < 1e-6 {
            new_vel = 0.0;
        }

        TrapezoidState { position: new_pos, velocity: new_vel }
    }
}
