use crate::motor::motor_io::{MotorFn, MotorIO, MotorLogSample};

const MAX_VOLTAGE: f64 = 12.0;

/// Threshold below which we consider the motor "stopped" for stiction purposes.
/// Small enough not to affect normal operation, large enough to prevent chattering.
const STICTION_VELOCITY_EPSILON: f64 = 1e-6; // rad/s

// ---------------------------------------------------------------------------
// Encoder simulation
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct EncoderConfig {
    pub noise_std_rad:    f64,  // Gaussian noise std dev (0 = perfect)
    pub quantization_rad: f64,  // Resolution step (0 = perfect; 2π/4096 for 12-bit encoder)
}

impl Default for EncoderConfig {
    fn default() -> Self { Self { noise_std_rad: 0.0, quantization_rad: 0.0 } }
}

// ---------------------------------------------------------------------------
// Integration method
// ---------------------------------------------------------------------------

/// ODE integrator selection for the motor dynamics.
///
/// # Why this matters
///
/// The motor dynamics are a first-order linear ODE:
///   dω/dt = (Kt·I - b·ω - τ_load) / J
///
/// Forward Euler discretises this as:
///   ω[n+1] = ω[n] + dt · f(ω[n])
///
/// The Euler method is O(dt) accurate and can *add energy* to an underdamped
/// system when dt is too large. For the default params (J=5, b=1) the time
/// constant τ = J/b = 5 s, so at typical game-loop dt ≈ 16 ms Euler is fine.
/// But if you dial up Kp or drop J, instability appears quickly.
///
/// RK4 is O(dt⁴) accurate and has a much larger stability region — it stays
/// stable at roughly 4× larger dt than Euler for the same system, at the cost
/// of 4 function evaluations per step instead of 1.
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum IntegrationMethod {
    /// First-order forward Euler. Identical to previous behavior. Default.
    #[default]
    Euler,
    /// 4th-order Runge-Kutta. More accurate and stable at larger dt.
    RK4,
}

// ---------------------------------------------------------------------------
// Motor parameters
// ---------------------------------------------------------------------------

pub struct MotorParams {
    pub r:  f64,   // Ω        winding resistance
    pub kt: f64,   // Nm/A     torque constant
    pub kv: f64,   // V·s/rad  back-EMF constant
    pub j:  f64,   // kg·m²    effective inertia
    pub b:  f64,   // Nm·s/rad viscous damping

    /// Static (break-away) friction torque, Nm.
    ///
    /// The motor will not begin to move until the net electromagnetic torque
    /// exceeds this value. Geared actuators can have surprisingly high stiction
    /// due to worm gears / harmonic drives — values of 1–5 Nm are realistic for
    /// a robotic arm joint.
    ///
    /// Must satisfy: coulomb_static_nm >= coulomb_kinetic_nm.
    /// Default: 0.0 (disabled — existing behavior unchanged).
    pub coulomb_static_nm:  f64,

    /// Kinetic (sliding) friction torque, Nm.
    ///
    /// Applied as a constant torque opposing the direction of motion once the
    /// motor is moving. Typically 60–80 % of τ_static for geared mechanisms.
    ///
    /// Default: 0.0 (disabled — existing behavior unchanged).
    pub coulomb_kinetic_nm: f64,

    pub encoder: EncoderConfig,

    /// Which ODE integrator to use. Default: Euler (identical to old behavior).
    pub integration: IntegrationMethod,
}

impl Default for MotorParams {
    fn default() -> Self {
        // Geared servo: max ~1.45 rad/s, ~48 Nm stall, overdamped at Kp=10.
        Self {
            r: 2.0, kt: 8.0, kv: 8.0, j: 5.0, b: 1.0,
            coulomb_static_nm:  0.0,
            coulomb_kinetic_nm: 0.0,
            encoder: EncoderConfig::default(),
            integration: IntegrationMethod::Euler,
        }
    }
}

// ---------------------------------------------------------------------------
// Motor simulator
// ---------------------------------------------------------------------------

pub struct MotorSim {
    motor_io:   MotorIO,
    r:          f64,
    kt:         f64,
    kv:         f64,
    j:          f64,
    b:          f64,
    load_torque: f64,

    coulomb_static_nm:  f64,
    coulomb_kinetic_nm: f64,

    /// True when the motor is currently held stationary by stiction.
    /// Carried across steps to avoid re-evaluating the static threshold while
    /// in steady motion (which would cause chattering near ω = 0).
    stuck: bool,

    encoder:     EncoderConfig,
    integration: IntegrationMethod,
}

impl MotorSim {
    #[allow(dead_code)]
    pub fn new() -> Self { Self::with_params(MotorParams::default()) }

    pub fn with_params(p: MotorParams) -> Self {
        Self {
            motor_io: MotorIO::new(),
            r: p.r, kt: p.kt, kv: p.kv, j: p.j, b: p.b,
            load_torque: 0.0,
            coulomb_static_nm:  p.coulomb_static_nm,
            coulomb_kinetic_nm: p.coulomb_kinetic_nm,
            stuck: false,
            encoder: p.encoder,
            integration: p.integration,
        }
    }
}

// ---------------------------------------------------------------------------
// MotorFn trait impl (the public sensing/actuation interface)
// ---------------------------------------------------------------------------

impl MotorFn for MotorSim {
    fn set_voltage(&mut self, volts: f64) {
        self.motor_io.input_voltage = volts.clamp(-MAX_VOLTAGE, MAX_VOLTAGE);
    }

    fn reset(&mut self) {
        self.motor_io.position     = 0.0;
        self.motor_io.velocity     = 0.0;
        self.motor_io.input_voltage = 0.0;
        self.load_torque           = 0.0;
        self.stuck                 = false;
    }

    fn get_position_rad(&self) -> f64 {
        let true_pos = self.motor_io.position;

        // Quantize first (simulates ADC discretization).
        let quantized = if self.encoder.quantization_rad > 1e-12 {
            (true_pos / self.encoder.quantization_rad).round() * self.encoder.quantization_rad
        } else {
            true_pos
        };

        // Add Gaussian noise (simulates electrical interference / thermal noise).
        if self.encoder.noise_std_rad > 1e-12 {
            use rand_distr::{Distribution, Normal};
            let normal = Normal::new(0.0_f64, self.encoder.noise_std_rad)
                .expect("valid normal params");
            quantized + normal.sample(&mut rand::thread_rng())
        } else {
            quantized
        }
    }

    fn get_velocity_rad_s(&self) -> f64 {
        self.motor_io.velocity
    }
}

// ---------------------------------------------------------------------------
// Physics impl
// ---------------------------------------------------------------------------

#[allow(dead_code)]
impl MotorSim {
    pub fn set_load_torque(&mut self, load: f64) {
        self.load_torque = load;
    }

    /// Feedforward voltage to pre-load against the current gravity torque.
    /// V_ff = τ_load × R / Kt
    pub fn gravity_feedforward_volts(&self) -> f64 {
        self.load_torque * self.r / self.kt
    }

    // -----------------------------------------------------------------------
    // Current / torque observers (Priority 2)
    // -----------------------------------------------------------------------

    /// Estimated motor winding current at the current operating point.
    ///
    /// ```text
    /// I = (V_applied − Kv · ω) / R
    /// ```
    ///
    /// At stall (ω = 0): I_stall = V / R  (maximum current, highest torque).
    /// At no-load speed: back-EMF ≈ V_applied, so I → 0.
    ///
    /// Useful for detecting stall conditions (high current + low velocity) and
    /// for future current-limiting logic.
    pub fn get_current_amps(&self) -> f64 {
        (self.motor_io.input_voltage - self.kv * self.motor_io.velocity) / self.r
    }

    /// Estimated electromagnetic torque produced by the motor.
    ///
    /// ```text
    /// τ = Kt · I
    /// ```
    ///
    /// This is the torque at the rotor before friction, damping, or load.
    pub fn get_torque_nm(&self) -> f64 {
        self.kt * self.get_current_amps()
    }

    // -----------------------------------------------------------------------
    // Internal ODE helpers
    // -----------------------------------------------------------------------

    /// Angular acceleration at velocity `vel` with a given pre-computed
    /// `friction_torque` (already signed to oppose motion).
    ///
    /// `friction_torque = 0.0` when friction is disabled or the stuck path
    /// handles it separately.
    #[inline]
    fn acceleration(&self, vel: f64, friction_torque: f64) -> f64 {
        let i   = (self.motor_io.input_voltage - self.kv * vel) / self.r;
        let tau = self.kt * i - self.b * vel - self.load_torque - friction_torque;
        tau / self.j
    }

    // -----------------------------------------------------------------------
    // Coulomb / stiction friction model (Priority 1)
    // -----------------------------------------------------------------------
    //
    // Design notes — why torque-projection instead of naive velocity checking
    // -----------------------------------------------------------------------
    //
    // The naive approach:
    //   if |ω| < ε:
    //       if |τ_net| < τ_static → keep ω = 0
    //       else → apply kinetic friction
    //
    // This chatters badly in discrete time. Near the stiction threshold, on
    // one step the motor "sticks" (ω set to 0), on the next the torque pushes
    // it past ε, it "releases" and Euler integrates a velocity, which then
    // gets clamped back to 0 the step after — oscillating at the simulation
    // frequency. The result is high-frequency noise in the velocity signal that
    // feeds back into the PID and can excite the plant.
    //
    // Our approach (torque-projection / state-machine):
    //
    // We carry a `stuck` boolean. When stuck:
    //   - Compute τ_net (em + viscous + load, no friction term yet).
    //   - If |τ_net| < τ_static: friction exactly cancels τ_net → acc = 0.
    //     Motor stays stuck. Velocity is also clamped to exactly 0.
    //   - If |τ_net| ≥ τ_static: break-away. Clear `stuck`. Apply kinetic
    //     friction in the direction of net torque. Compute real acceleration.
    //
    // When not stuck:
    //   - Apply kinetic friction opposing ω.
    //   - If the resulting velocity would cross zero (friction decelerates past
    //     zero), clamp to zero and set `stuck = true`. This prevents the motor
    //     from being "dragged backward" by kinetic friction alone.
    //
    // This gives clean stick/slip transitions with no chatter.

    /// Compute angular acceleration incorporating the Coulomb/stiction model.
    /// Updates `self.stuck` as a side effect.
    fn coulomb_acceleration(&mut self, vel: f64) -> f64 {
        let tau_static  = self.coulomb_static_nm;
        let tau_kinetic = self.coulomb_kinetic_nm;

        if self.stuck || vel.abs() < STICTION_VELOCITY_EPSILON {
            // Compute net torque without friction.
            let i       = (self.motor_io.input_voltage - self.kv * vel) / self.r;
            let tau_net = self.kt * i - self.b * vel - self.load_torque;

            if tau_net.abs() < tau_static {
                // Static friction can absorb the load — motor stays stuck.
                self.stuck = true;
                return 0.0;
            }
            // Break-away: apply kinetic friction opposing the direction of
            // net torque (the direction the motor is about to move).
            self.stuck = false;
            let friction  = tau_kinetic * tau_net.signum();
            let tau_total = tau_net - friction;
            return tau_total / self.j;
        }

        // Motor is already moving — kinetic friction opposes velocity.
        let friction = tau_kinetic * vel.signum();
        self.acceleration(vel, friction)
    }

    // -----------------------------------------------------------------------
    // Main step
    // -----------------------------------------------------------------------

    pub fn step(&mut self, dt: f64) {
        if dt <= 0.0 { return; }

        // Use true (noiseless) velocity — physics must not see encoder noise.
        let vel0 = self.motor_io.velocity;
        let pos0 = self.motor_io.position;

        let has_coulomb = self.coulomb_static_nm > 0.0 || self.coulomb_kinetic_nm > 0.0;

        let (new_vel, new_pos) = match self.integration {
            // ------------------------------------------------------------------
            IntegrationMethod::Euler => {
                let acc = if has_coulomb {
                    self.coulomb_acceleration(vel0)
                } else {
                    self.acceleration(vel0, 0.0)
                };

                let mut vel = vel0 + acc * dt;
                // If the motor was stuck, force velocity to exactly 0.
                if self.stuck { vel = 0.0; }
                // If kinetic friction decelerated past zero, latch to zero
                // so stiction can re-engage cleanly next step.
                if has_coulomb && vel0 != 0.0 && vel * vel0 < 0.0 {
                    vel = 0.0;
                    self.stuck = true;
                }
                let pos = pos0 + vel * dt;
                (vel, pos)
            }

            // ------------------------------------------------------------------
            // RK4 integration (Priority 3)
            //
            // For the stuck regime we fall back to Euler (or zero): the stiction
            // model has a discontinuity at break-away that makes the intermediate
            // RK4 stages meaningless. Once moving we can use full RK4.
            //
            // Kinetic friction is evaluated at vel0 and held constant across the
            // four stages. This is a first-order approximation (friction direction
            // doesn't change sign mid-step for reasonable dt), and avoids
            // discontinuities from re-evaluating signum at each stage.
            IntegrationMethod::RK4 => {
                if has_coulomb && (vel0.abs() < STICTION_VELOCITY_EPSILON || self.stuck) {
                    // Evaluate stiction decision at current state.
                    let acc = self.coulomb_acceleration(vel0);
                    if self.stuck {
                        // Still stuck — no movement.
                        (0.0_f64, pos0)
                    } else {
                        // Just broke free — use one Euler step to exit the
                        // discontinuity cleanly; RK4 takes over next step.
                        let vel = vel0 + acc * dt;
                        let pos = pos0 + vel * dt;
                        (vel, pos)
                    }
                } else {
                    // Fully moving (or no Coulomb friction) — standard RK4.
                    let friction = if has_coulomb {
                        self.coulomb_kinetic_nm * vel0.signum()
                    } else {
                        0.0
                    };

                    // Derivative: d(pos)/dt = vel,  d(vel)/dt = acc(vel)
                    let k1v = self.acceleration(vel0, friction);
                    let k1p = vel0;

                    let v2  = vel0 + 0.5 * k1v * dt;
                    let k2v = self.acceleration(v2, friction);
                    let k2p = v2;

                    let v3  = vel0 + 0.5 * k2v * dt;
                    let k3v = self.acceleration(v3, friction);
                    let k3p = v3;

                    let v4  = vel0 + k3v * dt;
                    let k4v = self.acceleration(v4, friction);
                    let k4p = v4;

                    let vel = vel0 + dt / 6.0 * (k1v + 2.0*k2v + 2.0*k3v + k4v);
                    let pos = pos0 + dt / 6.0 * (k1p + 2.0*k2p + 2.0*k3p + k4p);

                    // Clamp through-zero deceleration.
                    if has_coulomb && vel0 != 0.0 && vel * vel0 < 0.0 {
                        self.stuck = true;
                        (0.0_f64, pos0)
                    } else {
                        (vel, pos)
                    }
                }
            }
        };

        self.motor_io.velocity = new_vel;
        self.motor_io.position = new_pos;

        if let Some(logger) = &mut self.motor_io.logger {
            let t = logger.start_time.elapsed().as_secs_f64();
            logger.log(MotorLogSample {
                time_s:     t,
                position:   self.motor_io.position,
                velocity:   self.motor_io.velocity,
                voltage:    self.motor_io.input_voltage,
                load_torque: self.load_torque,
            });
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -------------------------------------------------------------------------
    // Existing encoder tests — must remain passing; behavior unchanged
    // -------------------------------------------------------------------------

    #[test]
    fn perfect_encoder_returns_true_position() {
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(12.0);
        for _ in 0..100 { m.step(0.01); }
        let pos = m.get_position_rad();
        assert!(pos > 0.0, "motor should have moved under 12V");
        assert!(pos.is_finite(), "position should be finite");
    }

    #[test]
    fn quantization_rounds_to_steps() {
        let params = MotorParams {
            encoder: EncoderConfig { noise_std_rad: 0.0, quantization_rad: 0.1 },
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        m.set_voltage(12.0);
        for _ in 0..100 { m.step(0.01); }
        let pos = m.get_position_rad();
        let remainder = (pos / 0.1).round() * 0.1 - pos;
        assert!(remainder.abs() < 1e-9, "position {pos:.4} not quantized to 0.1 rad steps");
    }

    // -------------------------------------------------------------------------
    // Stiction / Coulomb friction tests
    // -------------------------------------------------------------------------

    /// A tiny voltage produces τ_motor < τ_static — motor must not move.
    ///
    /// τ_static = 5 Nm.
    /// At V=0.1 V, ω=0: I = 0.1/2 = 0.05 A → τ = 8 × 0.05 = 0.4 Nm < 5 Nm → stuck.
    #[test]
    fn stiction_prevents_small_disturbance() {
        let params = MotorParams {
            coulomb_static_nm:  5.0,
            coulomb_kinetic_nm: 3.0,
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        m.set_voltage(0.1);
        for _ in 0..200 { m.step(0.01); }
        assert_eq!(
            m.motor_io.position, 0.0,
            "motor should be held by stiction under small voltage"
        );
        assert!(m.stuck, "stuck flag should be set");
    }

    /// A large voltage exceeds τ_static — motor must start moving.
    ///
    /// τ_static = 2 Nm. At V=12: I_stall = 6 A, τ = 48 Nm >> 2 Nm → releases.
    #[test]
    fn stiction_releases_above_threshold() {
        let params = MotorParams {
            coulomb_static_nm:  2.0,
            coulomb_kinetic_nm: 1.0,
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        m.set_voltage(12.0);
        for _ in 0..100 { m.step(0.01); }
        assert!(
            m.motor_io.position > 0.0,
            "motor should have moved when torque exceeds τ_static"
        );
        assert!(!m.stuck, "motor should not be stuck after break-away");
    }

    /// Zero Coulomb params must produce identical results to the plain default.
    #[test]
    fn no_friction_matches_default_euler() {
        let mut a = MotorSim::with_params(MotorParams::default());
        let mut b = MotorSim::with_params(MotorParams {
            coulomb_static_nm:  0.0,
            coulomb_kinetic_nm: 0.0,
            ..MotorParams::default()
        });
        a.set_voltage(6.0);
        b.set_voltage(6.0);
        for _ in 0..50 { a.step(0.02); b.step(0.02); }
        let diff = (a.motor_io.position - b.motor_io.position).abs();
        assert!(diff < 1e-12, "zero-friction should be identical to default (diff={diff:.2e})");
    }

    /// Kinetic friction noticeably reduces steady-state speed.
    #[test]
    fn kinetic_friction_reduces_velocity() {
        let mut no_fric = MotorSim::with_params(MotorParams::default());
        let mut fric    = MotorSim::with_params(MotorParams {
            coulomb_static_nm:  1.0,
            coulomb_kinetic_nm: 1.0,
            ..MotorParams::default()
        });
        no_fric.set_voltage(12.0);
        fric.set_voltage(12.0);
        for _ in 0..500 { no_fric.step(0.01); fric.step(0.01); }
        assert!(
            fric.motor_io.velocity < no_fric.motor_io.velocity,
            "friction motor should reach lower steady-state speed (no_fric={:.4}, fric={:.4})",
            no_fric.motor_io.velocity, fric.motor_io.velocity,
        );
    }

    /// A motor that was moving and has voltage removed should re-latch to stuck.
    #[test]
    fn motor_re_sticks_when_torque_drops() {
        let params = MotorParams {
            coulomb_static_nm:  2.0,
            coulomb_kinetic_nm: 1.5,
            ..MotorParams::default()
        };
        let mut m = MotorSim::with_params(params);
        // Spin it up.
        m.set_voltage(12.0);
        for _ in 0..200 { m.step(0.01); }
        assert!(m.motor_io.velocity > 0.0, "should be moving after spin-up");

        // Kill voltage — motor decelerates and stiction should re-engage.
        m.set_voltage(0.0);
        for _ in 0..2000 { m.step(0.01); }
        // Must have stopped (either stuck or velocity negligible).
        assert!(
            m.motor_io.velocity.abs() < 1e-3,
            "motor should stop when voltage removed and friction re-latches (vel={:.6})",
            m.motor_io.velocity,
        );
    }

    // -------------------------------------------------------------------------
    // Current / torque estimation tests (Priority 2)
    // -------------------------------------------------------------------------

    /// At stall (ω = 0), I = V / R.
    #[test]
    fn current_at_stall() {
        // R = 2.0, Kv = 8.0 (defaults). ω = 0 initially.
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(6.0);
        // Don't step — keep ω = 0.
        let expected = 6.0 / 2.0; // 3.0 A
        let actual   = m.get_current_amps();
        assert!(
            (actual - expected).abs() < 1e-10,
            "stall current: expected {expected} A, got {actual} A"
        );
    }

    /// Stall torque = Kt × (V / R).
    #[test]
    fn torque_at_stall() {
        // Kt = 8.0, R = 2.0 (defaults). ω = 0.
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(6.0);
        let expected = 8.0 * (6.0 / 2.0); // 24.0 Nm
        let actual   = m.get_torque_nm();
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
        let i_initial = m.get_current_amps(); // ω = 0
        for _ in 0..200 { m.step(0.01); }
        let i_running = m.get_current_amps();
        assert!(
            i_running < i_initial,
            "back-EMF should reduce current as speed increases \
             (initial={i_initial:.3} A, running={i_running:.3} A)"
        );
    }

    /// Negative voltage produces negative current.
    #[test]
    fn current_sign_matches_voltage() {
        let mut m = MotorSim::with_params(MotorParams::default());
        m.set_voltage(-12.0);
        assert!(m.get_current_amps() < 0.0, "negative voltage → negative current at stall");
    }

    // -------------------------------------------------------------------------
    // RK4 integration tests (Priority 3)
    // -------------------------------------------------------------------------

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
        // 10 simulated seconds at 10 ms — well past the τ = J/b = 5 s settling time.
        for _ in 0..1000 { euler.step(0.01); rk4.step(0.01); }
        let diff = (euler.motor_io.velocity - rk4.motor_io.velocity).abs();
        assert!(
            diff < 1e-4,
            "RK4 and Euler must converge to the same steady state (diff={diff:.6} rad/s)"
        );
    }

    /// RK4 is more accurate than Euler at large dt, compared to a fine reference.
    ///
    /// We run a fine-grained Euler simulation (dt = 0.1 ms) as ground truth,
    /// then compare coarse Euler (dt = 10 ms) and coarse RK4 (dt = 10 ms).
    /// RK4's O(dt⁴) error should be substantially smaller than Euler's O(dt).
    #[test]
    fn rk4_more_accurate_at_large_dt() {
        // Fine-grained reference (0.1 ms steps).
        let mut reference = MotorSim::with_params(MotorParams::default());
        reference.set_voltage(12.0);
        for _ in 0..10_000 { reference.step(0.0001); }

        // Coarse Euler (10 ms steps, 100 steps = 1 s).
        let mut coarse_euler = MotorSim::with_params(MotorParams {
            integration: IntegrationMethod::Euler,
            ..MotorParams::default()
        });
        coarse_euler.set_voltage(12.0);
        for _ in 0..100 { coarse_euler.step(0.01); }

        // Coarse RK4 (10 ms steps, 100 steps = 1 s).
        let mut coarse_rk4 = MotorSim::with_params(MotorParams {
            integration: IntegrationMethod::RK4,
            ..MotorParams::default()
        });
        coarse_rk4.set_voltage(12.0);
        for _ in 0..100 { coarse_rk4.step(0.01); }

        let err_euler = (coarse_euler.motor_io.velocity - reference.motor_io.velocity).abs();
        let err_rk4   = (coarse_rk4.motor_io.velocity  - reference.motor_io.velocity).abs();
        assert!(
            err_rk4 < err_euler,
            "RK4 should be more accurate than Euler at 10 ms steps \
             (euler_err={err_euler:.6}, rk4_err={err_rk4:.6})"
        );
    }

    /// RK4 with friction should also converge to a stable steady state.
    #[test]
    fn rk4_with_friction_stable() {
        let mut m = MotorSim::with_params(MotorParams {
            coulomb_static_nm:  1.0,
            coulomb_kinetic_nm: 0.8,
            integration: IntegrationMethod::RK4,
            ..MotorParams::default()
        });
        m.set_voltage(12.0);
        for _ in 0..1000 { m.step(0.01); }
        assert!(m.motor_io.velocity.is_finite(), "velocity should be finite");
        assert!(m.motor_io.position.is_finite(), "position should be finite");
        assert!(m.motor_io.velocity >= 0.0, "should not reverse direction under positive voltage");
    }
}
