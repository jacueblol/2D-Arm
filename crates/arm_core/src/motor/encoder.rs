//! Simulated rotary encoder: quantization + Gaussian noise on position reads.
//!
//! Both effects are applied only on the read path — the physics state
//! (`MotorSim`'s true position/velocity) is never touched, so noise never
//! feeds back into the dynamics.

use rand_distr::{Distribution, Normal};

#[derive(Clone, Debug)]
pub struct EncoderConfig {
    /// Gaussian noise standard deviation, rad. 0 = perfect.
    pub noise_std_rad: f64,
    /// Quantization step, rad. 0 = perfect (e.g. 2π/4096 for a 12-bit encoder).
    pub quantization_rad: f64,
}

impl Default for EncoderConfig {
    fn default() -> Self {
        Self {
            noise_std_rad: 0.0,
            quantization_rad: 0.0,
        }
    }
}

impl EncoderConfig {
    /// Apply quantization first (simulates ADC discretization), then additive
    /// Gaussian noise (simulates electrical interference / thermal noise) to a
    /// true position reading.
    pub fn read(&self, true_pos: f64) -> f64 {
        let quantized = if self.quantization_rad > 1e-12 {
            (true_pos / self.quantization_rad).round() * self.quantization_rad
        } else {
            true_pos
        };

        if self.noise_std_rad > 1e-12 {
            let normal = Normal::new(0.0_f64, self.noise_std_rad).expect("valid normal params");
            quantized + normal.sample(&mut rand::rng())
        } else {
            quantized
        }
    }
}
