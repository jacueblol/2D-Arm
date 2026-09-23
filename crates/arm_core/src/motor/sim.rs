use super::encoder::EncoderConfig;
use super::friction::Stiction;

/// Motor supply voltage limit, V. `MotorSim::set_voltage` clamps to this, and
/// it's the natural output bound for a `Joint`'s PID (see
/// `PidController::with_output_limits`), since anything past it can never
/// actually reach the motor anyway.
pub const MAX_VOLTAGE: f64 = 12.0;

/// ODE integrator selection for the motor dynamics.
///
/// The motor dynamics are a first-order linear ODE:
/// `dω/dt = (Kt·I − b·ω − τ_load) / J`.
///
/// Forward Euler discretises this as `ω[n+1] = ω[n] + dt · f(ω[n])`. It's
/// O(dt) accurate and can *add energy* to an underdamped system when dt is
/// too large. For the default params (J=5, b=1) the time constant τ = J/b =
/// 5 s, so at a typical game-loop dt ≈ 16 ms Euler is fine — but dial up Kp
/// or drop J and instability appears quickly.
///
/// RK4 is O(dt⁴) accurate with a much larger stability region — roughly 4×
/// larger dt than Euler for the same system — at the cost of 4 function
/// evaluations per step instead of 1.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum IntegrationMethod {
    /// First-order forward Euler. Default.
    #[default]
    Euler,
    /// 4th-order Runge-Kutta.
    RK4,
}

pub struct MotorParams {
    pub r: f64,  // Ω winding resistance
    pub kt: f64, // Nm/A torque constant
    pub kv: f64, // V·s/rad back-EMF constant
    pub j: f64,  // kg·m² effective inertia
    pub b: f64,  // Nm·s/rad viscous damping

    /// Static (break-away) friction torque, Nm. Geared actuators can have
    /// surprisingly high stiction due to worm gears / harmonic drives —
    /// values of 1-5 Nm are realistic for a robotic arm joint. Must satisfy
    /// `coulomb_static_nm >= coulomb_kinetic_nm`. Default 0.0 (disabled).
    pub coulomb_static_nm: f64,
    /// Kinetic (sliding) friction torque, Nm, typically 60-80% of
    /// `coulomb_static_nm` for geared mechanisms. Default 0.0 (disabled).
    pub coulomb_kinetic_nm: f64,

    pub encoder: EncoderConfig,
    pub integration: IntegrationMethod,
}

impl Default for MotorParams {
    fn default() -> Self {
        // Geared servo: max ~1.45 rad/s, ~48 Nm stall, overdamped at Kp=10.
        Self {
            r: 2.0,
            kt: 8.0,
            kv: 8.0,
            j: 5.0,
            b: 1.0,
            coulomb_static_nm: 0.0,
            coulomb_kinetic_nm: 0.0,
            encoder: EncoderConfig::default(),
            integration: IntegrationMethod::Euler,
        }
    }
}

pub struct MotorSim {
    position: f64,
    velocity: f64,
    input_voltage: f64,

    r: f64,
    kt: f64,
    kv: f64,
    j: f64,
    b: f64,
    load_torque: f64,

    friction: Stiction,
    encoder: EncoderConfig,
    integration: IntegrationMethod,
}

impl MotorSim {
    pub fn with_params(p: MotorParams) -> Self {
        Self {
            position: 0.0,
            velocity: 0.0,
            input_voltage: 0.0,
            r: p.r,
            kt: p.kt,
            kv: p.kv,
            j: p.j,
            b: p.b,
            load_torque: 0.0,
            friction: Stiction::new(p.coulomb_static_nm, p.coulomb_kinetic_nm),
            encoder: p.encoder,
            integration: p.integration,
        }
    }

    // -------------------------------------------------------------------
    // Actuation / sensing
    // -------------------------------------------------------------------

    pub fn set_voltage(&mut self, volts: f64) {
        self.input_voltage = volts.clamp(-MAX_VOLTAGE, MAX_VOLTAGE);
    }

    pub fn reset(&mut self) {
        self.position = 0.0;
        self.velocity = 0.0;
        self.input_voltage = 0.0;
        self.load_torque = 0.0;
        self.friction.reset();
    }

    pub fn get_position_rad(&self) -> f64 {
        self.encoder.read(self.position)
    }

    pub fn get_velocity_rad_s(&self) -> f64 {
        self.velocity
    }

    pub fn is_stuck(&self) -> bool {
        self.friction.is_stuck()
    }

    pub fn set_load_torque(&mut self, load: f64) {
        self.load_torque = load;
    }

    /// Feedforward voltage to pre-load against the current gravity torque.
    /// `V_ff = τ_load × R / Kt`
    pub fn gravity_feedforward_volts(&self) -> f64 {
        self.load_torque * self.r / self.kt
    }

    // -------------------------------------------------------------------
    // Current / torque observers
    // -------------------------------------------------------------------

    /// Estimated motor winding current at the current operating point:
    /// `I = (V_applied − Kv·ω) / R`. At stall (ω=0), `I = V/R` (max current,
    /// highest torque); at no-load speed, back-EMF ≈ V_applied so `I → 0`.
    pub fn get_current_amps(&self) -> f64 {
        self.current(self.velocity)
    }

    /// Estimated electromagnetic torque at the rotor, before friction,
    /// damping, or load: `τ = Kt · I`.
    pub fn get_torque_nm(&self) -> f64 {
        self.kt * self.get_current_amps()
    }

    // -------------------------------------------------------------------
    // Internal ODE helpers
    // -------------------------------------------------------------------

    fn current(&self, vel: f64) -> f64 {
        (self.input_voltage - self.kv * vel) / self.r
    }

    /// Net driving torque before friction: electromagnetic − viscous
    /// damping − load.
    fn tau_net(&self, vel: f64) -> f64 {
        self.kt * self.current(vel) - self.b * vel - self.load_torque
    }

    /// Angular acceleration at `vel` given a pre-computed `friction_torque`
    /// (already signed to oppose motion; 0.0 when friction is disabled).
    fn acceleration(&self, vel: f64, friction_torque: f64) -> f64 {
        (self.tau_net(vel) - friction_torque) / self.j
    }

    /// Angular acceleration incorporating the Coulomb/stiction model.
    /// Updates `self.friction`'s stuck state as a side effect.
    fn coulomb_acceleration(&mut self, vel: f64) -> f64 {
        let tau_net = self.tau_net(vel);
        match self.friction.resolve(vel, tau_net) {
            Some(tau_effective) => tau_effective / self.j,
            None => 0.0,
        }
    }

    // -------------------------------------------------------------------
    // Main step
    // -------------------------------------------------------------------

    pub fn step(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        // Use true (noiseless) velocity — physics must not see encoder noise.
        let vel0 = self.velocity;
        let pos0 = self.position;
        let has_coulomb = self.friction.is_enabled();

        let (new_vel, new_pos) = match self.integration {
            IntegrationMethod::Euler => {
                let acc = if has_coulomb {
                    self.coulomb_acceleration(vel0)
                } else {
                    self.acceleration(vel0, 0.0)
                };

                let mut vel = vel0 + acc * dt;
                // If the motor was stuck, force velocity to exactly 0.
                if self.friction.is_stuck() {
                    vel = 0.0;
                }
                // If kinetic friction decelerated past zero, latch to zero so
                // stiction can re-engage cleanly next step.
                if has_coulomb && vel0 != 0.0 && vel * vel0 < 0.0 {
                    vel = 0.0;
                    self.friction.latch();
                }
                let pos = pos0 + vel * dt;
                (vel, pos)
            }

            // For the stuck regime we fall back to Euler (or zero): the
            // stiction model has a discontinuity at break-away that makes
            // the intermediate RK4 stages meaningless. Once moving we use
            // full RK4, with kinetic friction evaluated at vel0 and held
            // constant across the four stages (a first-order approximation —
            // friction direction doesn't change sign mid-step for
            // reasonable dt — that avoids re-evaluating signum per stage).
            IntegrationMethod::RK4 => {
                if has_coulomb
                    && (vel0.abs() < super::friction::STICTION_VELOCITY_EPSILON
                        || self.friction.is_stuck())
                {
                    let acc = self.coulomb_acceleration(vel0);
                    if self.friction.is_stuck() {
                        (0.0_f64, pos0)
                    } else {
                        // Just broke free — one Euler step to exit the
                        // discontinuity cleanly; RK4 takes over next step.
                        let vel = vel0 + acc * dt;
                        let pos = pos0 + vel * dt;
                        (vel, pos)
                    }
                } else {
                    let friction = if has_coulomb {
                        self.friction.kinetic_opposing(vel0)
                    } else {
                        0.0
                    };

                    // Derivative: d(pos)/dt = vel, d(vel)/dt = acc(vel).
                    let k1v = self.acceleration(vel0, friction);
                    let k1p = vel0;

                    let v2 = vel0 + 0.5 * k1v * dt;
                    let k2v = self.acceleration(v2, friction);
                    let k2p = v2;

                    let v3 = vel0 + 0.5 * k2v * dt;
                    let k3v = self.acceleration(v3, friction);
                    let k3p = v3;

                    let v4 = vel0 + k3v * dt;
                    let k4v = self.acceleration(v4, friction);
                    let k4p = v4;

                    let vel = vel0 + dt / 6.0 * (k1v + 2.0 * k2v + 2.0 * k3v + k4v);
                    let pos = pos0 + dt / 6.0 * (k1p + 2.0 * k2p + 2.0 * k3p + k4p);

                    if has_coulomb && vel0 != 0.0 && vel * vel0 < 0.0 {
                        self.friction.latch();
                        (0.0_f64, pos0)
                    } else {
                        (vel, pos)
                    }
                }
            }
        };

        self.velocity = new_vel;
        self.position = new_pos;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn perfect_encoder_returns_true_position() {
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(12.0);
        for _ in 0..100 {
            m.step(0.01);
        }
        let pos = m.get_position_rad();
        assert!(pos > 0.0, "motor should have moved under 12V");
        assert!(pos.is_finite(), "position should be finite");
    }

    #[test]
    fn quantization_rounds_to_steps() {
        let params = MotorParams {
            encoder: EncoderConfig {
                noise_std_rad: 0.0,
                quantization_rad: 0.1,
            },
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        m.set_voltage(12.0);
        for _ in 0..100 {
            m.step(0.01);
        }
        let pos = m.get_position_rad();
        let remainder = (pos / 0.1).round() * 0.1 - pos;
        assert!(
            remainder.abs() < 1e-9,
            "position {pos:.4} not quantized to 0.1 rad steps"
        );
    }

    /// A tiny voltage produces τ_motor < τ_static — motor must not move.
    /// τ_static = 5 Nm. At V=0.1, ω=0: I=0.05A → τ=0.4 Nm < 5 Nm → stuck.
    #[test]
    fn stiction_prevents_small_disturbance() {
        let params = MotorParams {
            coulomb_static_nm: 5.0,
            coulomb_kinetic_nm: 3.0,
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        m.set_voltage(0.1);
        for _ in 0..200 {
            m.step(0.01);
        }
        assert_eq!(
            m.position, 0.0,
            "motor should be held by stiction under small voltage"
        );
        assert!(m.is_stuck(), "stuck flag should be set");
    }

    /// A large voltage exceeds τ_static — motor must start moving.
    /// τ_static = 2 Nm. At V=12: I_stall=6A, τ=48 Nm >> 2 Nm → releases.
    #[test]
    fn stiction_releases_above_threshold() {
        let params = MotorParams {
            coulomb_static_nm: 2.0,
            coulomb_kinetic_nm: 1.0,
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        m.set_voltage(12.0);
        for _ in 0..100 {
            m.step(0.01);
        }
        assert!(
            m.position > 0.0,
            "motor should have moved when torque exceeds τ_static"
        );
        assert!(!m.is_stuck(), "motor should not be stuck after break-away");
    }

    /// Zero Coulomb params must produce identical results to the plain default.
    #[test]
    fn no_friction_matches_default_euler() {
        let mut a = MotorSim::with_params(MotorParams::default());
        let mut b = MotorSim::with_params(MotorParams {
            coulomb_static_nm: 0.0,
            coulomb_kinetic_nm: 0.0,
            ..MotorParams::default()
        });
        a.set_voltage(6.0);
        b.set_voltage(6.0);
        for _ in 0..50 {
            a.step(0.02);
            b.step(0.02);
        }
        let diff = (a.position - b.position).abs();
        assert!(
            diff < 1e-12,
            "zero-friction should be identical to default (diff={diff:.2e})"
        );
    }

    /// Kinetic friction noticeably reduces steady-state speed.
    #[test]
    fn kinetic_friction_reduces_velocity() {
        let mut no_fric = MotorSim::with_params(MotorParams::default());
        let mut fric = MotorSim::with_params(MotorParams {
            coulomb_static_nm: 1.0,
            coulomb_kinetic_nm: 1.0,
            ..MotorParams::default()
        });
        no_fric.set_voltage(12.0);
        fric.set_voltage(12.0);
        for _ in 0..500 {
            no_fric.step(0.01);
            fric.step(0.01);
        }
        assert!(
            fric.velocity < no_fric.velocity,
            "friction motor should reach lower steady-state speed (no_fric={:.4}, fric={:.4})",
            no_fric.velocity,
            fric.velocity,
        );
    }

    /// A motor that was moving and has voltage removed should re-latch to stuck.
    #[test]
    fn motor_re_sticks_when_torque_drops() {
        let params = MotorParams {
            coulomb_static_nm: 2.0,
            coulomb_kinetic_nm: 1.5,
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        m.set_voltage(12.0);
        for _ in 0..200 {
            m.step(0.01);
        }
        assert!(m.velocity > 0.0, "should be moving after spin-up");

        m.set_voltage(0.0);
        for _ in 0..2000 {
            m.step(0.01);
        }
        assert!(
            m.velocity.abs() < 1e-3,
            "motor should stop when voltage removed and friction re-latches (vel={:.6})",
            m.velocity,
        );
    }

    /// At stall (ω=0), I = V / R.
    #[test]
    fn current_at_stall() {
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(6.0);
        let expected = 6.0 / 2.0; // 3.0 A
        let actual = m.get_current_amps();
        assert!(
            (actual - expected).abs() < 1e-10,
            "stall current: expected {expected} A, got {actual} A"
        );
    }

    /// Stall torque = Kt × (V / R).
    #[test]
    fn torque_at_stall() {
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(6.0);
        let expected = 8.0 * (6.0 / 2.0); // 24.0 Nm
        let actual = m.get_torque_nm();
        assert!(
            (actual - expected).abs() < 1e-10,
            "stall torque: expected {expected} Nm, got {actual} Nm"
        );
    }

    /// As back-EMF builds with speed, current should drop.
    #[test]
    fn current_drops_with_speed() {
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(12.0);
        let i_initial = m.get_current_amps();
        for _ in 0..200 {
            m.step(0.01);
        }
        let i_running = m.get_current_amps();
        assert!(
            i_running < i_initial,
            "back-EMF should reduce current as speed increases (initial={i_initial:.3} A, running={i_running:.3} A)"
        );
    }

    /// Negative voltage produces negative current.
    #[test]
    fn current_sign_matches_voltage() {
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(-12.0);
        assert!(
            m.get_current_amps() < 0.0,
            "negative voltage should give negative current at stall"
        );
    }

    /// RK4 and Euler converge to the same steady-state velocity.
    #[test]
    fn rk4_converges_to_same_steady_state() {
        let mut euler = MotorSim::with_params(MotorParams {
            integration: IntegrationMethod::Euler,
            ..MotorParams::default()
        });
        let mut rk4 = MotorSim::with_params(MotorParams {
            integration: IntegrationMethod::RK4,
            ..MotorParams::default()
        });
        euler.set_voltage(8.0);
        rk4.set_voltage(8.0);
        // 10 simulated seconds at 10ms — well past the τ=J/b=5s settling time.
        for _ in 0..1000 {
            euler.step(0.01);
            rk4.step(0.01);
        }
        let diff = (euler.velocity - rk4.velocity).abs();
        assert!(
            diff < 1e-4,
            "RK4 and Euler must converge to the same steady state (diff={diff:.6} rad/s)"
        );
    }

    /// RK4 is more accurate than Euler at large dt, compared to a fine reference.
    #[test]
    fn rk4_more_accurate_at_large_dt() {
        let mut reference = MotorSim::with_params(MotorParams::default());
        reference.set_voltage(12.0);
        for _ in 0..10_000 {
            reference.step(0.0001);
        }

        let mut coarse_euler = MotorSim::with_params(MotorParams {
            integration: IntegrationMethod::Euler,
            ..MotorParams::default()
        });
        coarse_euler.set_voltage(12.0);
        for _ in 0..100 {
            coarse_euler.step(0.01);
        }

        let mut coarse_rk4 = MotorSim::with_params(MotorParams {
            integration: IntegrationMethod::RK4,
            ..MotorParams::default()
        });
        coarse_rk4.set_voltage(12.0);
        for _ in 0..100 {
            coarse_rk4.step(0.01);
        }

        let err_euler = (coarse_euler.velocity - reference.velocity).abs();
        let err_rk4 = (coarse_rk4.velocity - reference.velocity).abs();
        assert!(
            err_rk4 < err_euler,
            "RK4 should be more accurate than Euler at 10ms steps (euler_err={err_euler:.6}, rk4_err={err_rk4:.6})"
        );
    }

    /// RK4 with friction should also converge to a stable steady state.
    #[test]
    fn rk4_with_friction_stable() {
        let mut m = MotorSim::with_params(MotorParams {
            coulomb_static_nm: 1.0,
            coulomb_kinetic_nm: 0.8,
            integration: IntegrationMethod::RK4,
            ..MotorParams::default()
        });
        m.set_voltage(12.0);
        for _ in 0..1000 {
            m.step(0.01);
        }
        assert!(m.velocity.is_finite(), "velocity should be finite");
        assert!(m.position.is_finite(), "position should be finite");
        assert!(
            m.velocity >= 0.0,
            "should not reverse direction under positive voltage"
        );
    }
}
