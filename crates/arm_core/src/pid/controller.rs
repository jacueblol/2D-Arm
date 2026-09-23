/// Classic PID controller, deliberately stateless: the caller owns
/// `prev_error` and `integ_total` and threads them through each call. This
/// keeps `PidController` itself trivially `Copy`-able config, and makes it
/// easy for a caller (e.g. a joint) to reset just the integrator without
/// touching gains, or to run several independent control loops off one set
/// of gains.
#[derive(Clone, Copy, Debug)]
pub struct PidController {
    pub k_p: f64,
    pub k_i: f64,
    pub k_d: f64,

    /// Output is clamped to `[output_min, output_max]`, and the integrator
    /// uses conditional integration (a standard clamping anti-windup
    /// technique) against those same bounds: once the output has saturated
    /// in a direction, further error in that same direction is not
    /// integrated, so the integral term never winds up past what's needed
    /// to hit the limit. Error that would pull the output back out of
    /// saturation is still integrated normally, so recovery isn't delayed.
    ///
    /// Defaults to unbounded (`±∞`) — anti-windup only activates once
    /// [`PidController::with_output_limits`] sets real bounds.
    pub output_min: f64,
    pub output_max: f64,
}

pub struct PidOut {
    pub output: f64,
    pub error: f64,
    pub integ_total: f64,
}

impl PidController {
    pub fn new(k_p: f64, k_i: f64, k_d: f64) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            output_min: f64::NEG_INFINITY,
            output_max: f64::INFINITY,
        }
    }

    pub fn with_output_limits(mut self, min: f64, max: f64) -> Self {
        self.output_min = min;
        self.output_max = max;
        self
    }

    /// `error = setpoint - measurement`, `P = kp*error`, `I += ki*error*dt`
    /// (subject to anti-windup, see [`PidController::output_min`]),
    /// `D = kd*(error - prev_error)/dt`, `output = clamp(P + I + D)`.
    ///
    /// Returns the updated `integ_total` for the caller to pass back in on
    /// the next call.
    pub fn step(
        &self,
        dt: f64,
        setpoint: f64,
        measurement: f64,
        prev_err: f64,
        integ_total: f64,
    ) -> PidOut {
        let error = setpoint - measurement;

        let p = self.k_p * error;

        let derivative = if dt > 0.0 {
            (error - prev_err) / dt
        } else {
            0.0
        };
        let d = self.k_d * derivative;

        // Trial integral accumulation, then decide whether to keep it.
        let integ_trial = integ_total + self.k_i * error * dt;
        let output_trial = p + integ_trial + d;

        let saturated_high = output_trial > self.output_max;
        let saturated_low = output_trial < self.output_min;
        // Only freeze the integrator when the error is pushing further into
        // the saturation it's already in — error pulling back out is still
        // integrated, so recovery from windup isn't delayed either.
        let winding_up = (saturated_high && error > 0.0) || (saturated_low && error < 0.0);

        let integ_total = if winding_up { integ_total } else { integ_trial };
        let output = (p + integ_total + d).clamp(self.output_min, self.output_max);

        PidOut {
            output,
            error,
            integ_total,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proportional_only_scales_error() {
        let pid = PidController::new(2.0, 0.0, 0.0);
        let out = pid.step(0.01, 10.0, 4.0, 0.0, 0.0);
        assert_eq!(out.error, 6.0);
        assert!((out.output - 12.0).abs() < 1e-12);
        assert_eq!(out.integ_total, 0.0, "ki=0 should never accumulate");
    }

    #[test]
    fn integral_accumulates_over_time() {
        let pid = PidController::new(0.0, 1.0, 0.0);
        let mut integ = 0.0;
        let mut prev_err = 0.0;
        for _ in 0..10 {
            let out = pid.step(0.1, 5.0, 0.0, prev_err, integ);
            integ = out.integ_total;
            prev_err = out.error;
        }
        // error=5 constant, ki=1, dt=0.1 -> +0.5 per step, 10 steps = 5.0.
        assert!(
            (integ - 5.0).abs() < 1e-9,
            "expected integ_total=5.0, got {integ}"
        );
    }

    #[test]
    fn derivative_responds_to_error_change() {
        let pid = PidController::new(0.0, 0.0, 3.0);
        // error goes from 2.0 to 5.0 across this step.
        let out = pid.step(0.5, 10.0, 5.0, 2.0, 0.0);
        assert_eq!(out.error, 5.0);
        // d = kd * (error - prev_err) / dt = 3 * (5-2) / 0.5 = 18
        assert!((out.output - 18.0).abs() < 1e-9);
    }

    #[test]
    fn unbounded_never_clamps() {
        let pid = PidController::new(1000.0, 0.0, 0.0);
        let out = pid.step(0.01, 1.0, 0.0, 0.0, 0.0);
        assert!(
            (out.output - 1000.0).abs() < 1e-9,
            "default limits should not clamp"
        );
    }

    /// The regression case the anti-windup milestone exists for: under
    /// sustained saturation, an unbounded integrator winds up without limit,
    /// while a bounded (anti-windup) one stays near what's actually needed
    /// to hold the output at its limit.
    #[test]
    fn anti_windup_bounds_integrator_under_sustained_saturation() {
        let unbounded = PidController::new(0.05, 1.0, 0.0);
        let bounded = unbounded.with_output_limits(-12.0, 12.0);

        let (mut u_prev_err, mut u_integ) = (0.0, 0.0);
        let (mut b_prev_err, mut b_integ) = (0.0, 0.0);

        // Sustained large error: measurement stuck at 0 while setpoint is
        // far out of reach, for far longer than it takes the output to
        // saturate.
        for _ in 0..1000 {
            let out_u = unbounded.step(0.01, 100.0, 0.0, u_prev_err, u_integ);
            u_prev_err = out_u.error;
            u_integ = out_u.integ_total;

            let out_b = bounded.step(0.01, 100.0, 0.0, b_prev_err, b_integ);
            b_prev_err = out_b.error;
            b_integ = out_b.integ_total;
        }

        assert!(
            u_integ > 500.0,
            "unbounded integrator should wind up substantially, got {u_integ}"
        );
        assert!(
            b_integ < 20.0,
            "anti-windup integrator should stay near the saturation boundary, got {b_integ}"
        );
    }

    /// Once saturated, error pulling back the other way should still be
    /// integrated immediately (no artificial recovery delay).
    #[test]
    fn anti_windup_still_integrates_error_pulling_out_of_saturation() {
        let pid = PidController::new(0.05, 1.0, 0.0).with_output_limits(-12.0, 12.0);

        // Saturate high first.
        let (mut prev_err, mut integ) = (0.0, 0.0);
        for _ in 0..50 {
            let out = pid.step(0.01, 100.0, 0.0, prev_err, integ);
            prev_err = out.error;
            integ = out.integ_total;
        }
        let integ_at_saturation = integ;

        // Now the error flips sign (measurement overshoots past setpoint) —
        // this should immediately start pulling the integral back down.
        let out = pid.step(0.01, 100.0, 200.0, prev_err, integ);
        assert!(
            out.integ_total < integ_at_saturation,
            "error reversing should immediately reduce the integrator, not stay frozen"
        );
    }
}
