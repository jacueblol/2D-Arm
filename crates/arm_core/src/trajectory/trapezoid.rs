/// Position + velocity of a trajectory profile at a point in time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrapezoidState {
    pub position: f64,
    pub velocity: f64,
}

/// Bounded-velocity, bounded-acceleration motion profile: accelerates
/// toward the goal, then decelerates to arrive at exactly zero velocity —
/// the setpoint a controller tracks ramps smoothly instead of jumping.
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
            return TrapezoidState {
                position: goal,
                velocity: 0.0,
            };
        }

        let sign = if error >= 0.0 { 1.0 } else { -1.0 };

        // Distance needed to brake current velocity to zero.
        let stopping_dist = current.velocity.abs().powi(2) / (2.0 * self.max_accel);
        let moving_toward_goal = current.velocity * sign > 0.0;

        let target_accel = if moving_toward_goal && stopping_dist >= error.abs() {
            -sign * self.max_accel // decelerate to stop at goal
        } else {
            sign * self.max_accel // accelerate toward goal
        };

        let mut new_vel = (current.velocity + target_accel * dt).clamp(-self.max_vel, self.max_vel);
        let new_pos = current.position + new_vel * dt;

        // Clamp if we overshot.
        if (new_pos - goal) * sign > 0.0 {
            return TrapezoidState {
                position: goal,
                velocity: 0.0,
            };
        }

        // Damp velocity if we're at the goal but still drifting.
        if (goal - new_pos).abs() < 1e-6 {
            new_vel = 0.0;
        }

        TrapezoidState {
            position: new_pos,
            velocity: new_vel,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accelerates_from_rest_toward_goal() {
        let profile = TrapezoidProfile::new(1.0, 2.0);
        let state = profile.calculate(
            0.1,
            TrapezoidState {
                position: 0.0,
                velocity: 0.0,
            },
            10.0,
        );
        assert!(
            state.velocity > 0.0,
            "should start accelerating toward a positive goal"
        );
        assert!(state.position > 0.0);
    }

    #[test]
    fn respects_max_velocity() {
        let profile = TrapezoidProfile::new(1.0, 100.0);
        let mut state = TrapezoidState {
            position: 0.0,
            velocity: 0.0,
        };
        for _ in 0..50 {
            state = profile.calculate(0.05, state, 100.0);
        }
        assert!(
            state.velocity <= 1.0 + 1e-9,
            "velocity should not exceed max_vel, got {}",
            state.velocity
        );
    }

    #[test]
    fn decelerates_to_arrive_exactly_at_goal_with_zero_velocity() {
        let profile = TrapezoidProfile::new(2.0, 4.0);
        let mut state = TrapezoidState {
            position: 0.0,
            velocity: 0.0,
        };
        for _ in 0..2000 {
            state = profile.calculate(0.005, state, 5.0);
        }
        assert!(
            (state.position - 5.0).abs() < 1e-6,
            "should arrive exactly at goal, got {}",
            state.position
        );
        assert_eq!(state.velocity, 0.0, "should be at rest on arrival");
    }

    #[test]
    fn negative_goal_moves_backward() {
        let profile = TrapezoidProfile::new(1.0, 2.0);
        let mut state = TrapezoidState {
            position: 0.0,
            velocity: 0.0,
        };
        for _ in 0..500 {
            state = profile.calculate(0.01, state, -3.0);
        }
        assert!((state.position - (-3.0)).abs() < 1e-6);
    }
}
