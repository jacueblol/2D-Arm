use crate::motor::motor_io::{MotorFn, MotorIO, MotorLogSample};

const MAX_VOLTAGE: f64 = 12.0;



pub struct MotorParams {
    pub r:  f64,   // Ω       winding resistance
    pub kt: f64,   // Nm/A    torque constant
    pub kv: f64,   // V·s/rad back-EMF constant
    pub j:  f64,   // kg·m²   effective inertia
    pub b:  f64,   // Nm·s/rad viscous damping
}

impl Default for MotorParams {
    fn default() -> Self {
        // Geared servo: max ~1.45 rad/s, ~48 Nm stall, overdamped at Kp=10.
        Self { r: 2.0, kt: 8.0, kv: 8.0, j: 5.0, b: 1.0 }
    }
}

pub struct MotorSim {
    motor_io: MotorIO,
    r: f64,
    kt: f64,
    kv: f64,
    j: f64,
    b: f64,
    load_torque: f64,
}

impl MotorSim {
    #[allow(dead_code)]
    pub fn new() -> Self { Self::with_params(MotorParams::default()) }

    pub fn with_params(p: MotorParams) -> Self {
        Self { motor_io: MotorIO::new(), r: p.r, kt: p.kt, kv: p.kv, j: p.j, b: p.b, load_torque: 0.0 }
    }
}

// General motor implimentation 
impl MotorFn for MotorSim {
    fn set_voltage(&mut self, volts:f64) {
        self.motor_io.input_voltage = volts.clamp(-MAX_VOLTAGE, MAX_VOLTAGE);
    }

    fn reset(&mut self) {
        self.motor_io.position = 0.0;
        self.motor_io.velocity = 0.0;
        self.motor_io.input_voltage = 0.0;
        self.load_torque = 0.0;
    }

    fn get_position_rad(&self) -> f64 {
        self.motor_io.position
    }

    fn get_velocity_rad_s(&self) -> f64 {
        self.motor_io.velocity
    }
}

#[allow(dead_code)]
impl MotorSim {
    pub fn set_load_torque(&mut self, load: f64) {
        self.load_torque = load;
    }

    /// Feedforward voltage to pre-load against the current load torque (gravity).
    /// V_ff = τ_load × R / Kt  — produces exactly the current needed to hold the load.
    pub fn gravity_feedforward_volts(&self) -> f64 {
        self.load_torque * self.r / self.kt
    }

    pub fn step(&mut self, dt: f64) {
        if dt <= 0.0 {
            return;
        }

        // Current from electrical model
        let i = (self.motor_io.input_voltage - self.kv * self.get_velocity_rad_s()) / self.r;

        // Torque from current
        let motor_torque = self.kt * i;

        // Net torque
        let net = motor_torque - self.b * self.get_velocity_rad_s() - self.load_torque;

        // Angular acceleration
        let acc = net / self.j;

        // Integrate
        self.motor_io.velocity += acc * dt;
        self.motor_io.position += self.get_velocity_rad_s() * dt;

        // logger
        if let Some(logger) = &mut self.motor_io.logger {
            let t = logger.start_time.elapsed().as_secs_f64();
            logger.log(MotorLogSample {
                time_s: t,
                position: self.motor_io.position,
                velocity: self.motor_io.velocity,
                voltage: self.motor_io.input_voltage,
                load_torque: self.load_torque,
            });
        }
    }

}

