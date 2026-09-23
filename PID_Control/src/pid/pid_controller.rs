pub struct PidController {
    pub k_p: f64,
    pub k_i: f64,
    pub k_d: f64,
}

pub struct PidOut {
    pub output: f64,
    pub error: f64,
    pub integ_total: f64,
}

impl PidController {
    pub fn new(k_p: f64, k_i: f64, k_d: f64) -> Self {
        Self { k_p, k_i, k_d }
    }

    /// Classic PID:
    ///   error = setpoint - measurement
    ///   P = k_p * error
    ///   I += k_i * error * dt
    ///   D = k_d * (error - prev_error) / dt
    ///
    /// You keep track of prev_error and integ_total outside,
    /// pass them in, and we return the updated values.
    pub fn step(
        &self,
        dt: f64,
        setpoint: f64,
        measurement: f64,
        prev_err: f64,
        mut integ_total: f64,
    ) -> PidOut {
        let error = setpoint - measurement;

        // Proportional
        let p = self.k_p * error;

        // Integral
        integ_total += self.k_i * error * dt;

        // Derivative
        let derivative = if dt > 0.0 {
            (error - prev_err) / dt
        } else {
            0.0
        };
        let d = self.k_d * derivative;

        let output = p + integ_total + d;

        PidOut {
            output,
            error,
            integ_total,
        }
    }
}
