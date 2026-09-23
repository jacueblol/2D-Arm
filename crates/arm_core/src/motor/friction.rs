//! Coulomb (static + kinetic) friction / stiction model.
//!
//! # Why torque-projection instead of naive velocity checking
//!
//! The naive approach:
//! ```text
//! if |ω| < ε:
//!     if |τ_net| < τ_static → keep ω = 0
//!     else → apply kinetic friction
//! ```
//! chatters badly in discrete time. Near the stiction threshold, on one step
//! the motor "sticks" (ω set to 0), on the next the torque pushes it past ε,
//! it "releases" and Euler integrates a velocity, which then gets clamped
//! back to 0 the step after — oscillating at the simulation frequency. The
//! result is high-frequency noise in the velocity signal that feeds back
//! into the PID and can excite the plant.
//!
//! Our approach (torque-projection / state-machine): we carry a `stuck`
//! flag.
//! - **While stuck**: compute τ_net (electromagnetic − viscous − load, no
//!   friction term yet). If `|τ_net| < τ_static`, friction exactly cancels
//!   τ_net → the motor stays stuck at ω = 0. Otherwise, break away: clear
//!   `stuck`, apply kinetic friction in the direction of τ_net.
//! - **While moving**: apply kinetic friction opposing ω. If the resulting
//!   velocity would cross zero (friction decelerating past zero), the caller
//!   clamps to zero and re-[`latch`](Stiction::latch)es `stuck`, preventing
//!   the motor from being "dragged backward" by kinetic friction alone.
//!
//! This gives clean stick/slip transitions with no chatter.

/// Below this speed we treat the motor as "stopped" for stiction purposes.
/// Small enough not to affect normal operation, large enough to prevent
/// chattering.
pub const STICTION_VELOCITY_EPSILON: f64 = 1e-6; // rad/s

#[derive(Clone, Copy, Debug)]
pub struct Stiction {
    /// Break-away (static) friction torque, Nm. The motor will not begin to
    /// move until net electromagnetic torque exceeds this.
    pub static_nm: f64,
    /// Sliding (kinetic) friction torque, Nm, applied once moving.
    pub kinetic_nm: f64,
    stuck: bool,
}

impl Stiction {
    pub fn new(static_nm: f64, kinetic_nm: f64) -> Self {
        Self {
            static_nm,
            kinetic_nm,
            stuck: false,
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.static_nm > 0.0 || self.kinetic_nm > 0.0
    }

    pub fn is_stuck(&self) -> bool {
        self.stuck
    }

    pub fn reset(&mut self) {
        self.stuck = false;
    }

    /// Force the stuck state on — used when kinetic friction decelerates the
    /// motor through zero velocity, so stiction can re-engage cleanly next
    /// step.
    pub fn latch(&mut self) {
        self.stuck = true;
    }

    /// Kinetic friction torque opposing a given velocity. Used when the
    /// motor is confidently moving, including across RK4 sub-stages where
    /// the friction magnitude is deliberately held constant at the
    /// step-start velocity (see the `MotorSim::step` RK4 branch).
    pub fn kinetic_opposing(&self, vel: f64) -> f64 {
        self.kinetic_nm * vel.signum()
    }

    /// Resolve the friction-adjusted net torque at `vel`, given the
    /// friction-free net torque `tau_net` (electromagnetic − viscous damping
    /// − load). Updates internal stuck state as a side effect. Returns
    /// `None` if the motor is held by static friction (net torque
    /// insufficient to break away) — the caller should treat acceleration
    /// as zero and force velocity to exactly zero.
    pub fn resolve(&mut self, vel: f64, tau_net: f64) -> Option<f64> {
        if self.stuck || vel.abs() < STICTION_VELOCITY_EPSILON {
            if tau_net.abs() < self.static_nm {
                self.stuck = true;
                return None;
            }
            self.stuck = false;
            let friction = self.kinetic_nm * tau_net.signum();
            return Some(tau_net - friction);
        }

        let friction = self.kinetic_opposing(vel);
        Some(tau_net - friction)
    }
}
