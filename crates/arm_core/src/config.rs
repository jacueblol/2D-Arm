//! TOML-driven configuration, shared between `arm_core`'s own tests and the
//! `arm_sim` Bevy app — both parse the same [`Config`] schema against the
//! same `config.toml`, so there's exactly one place these values can drift.

use crate::arm::Joint;
use crate::motor::{EncoderConfig, IntegrationMethod, MAX_VOLTAGE, MotorParams, MotorSim};
use crate::pid::PidController;
use crate::trajectory::TrapezoidProfile;

/// The repo's `config.toml`, embedded at compile time as a fallback for
/// when no file is present on disk at runtime — the app always has a valid
/// configuration to fall back to.
const EMBEDDED_DEFAULT: &str = include_str!("../../../config.toml");

#[derive(serde::Deserialize)]
pub struct Config {
    pub sim: SimCfg,
    pub motor: MotorCfg,
    pub pan: JointCfg,
    pub shoulder: JointCfg,
    pub elbow: JointCfg,
}

#[derive(serde::Deserialize)]
pub struct SimCfg {
    pub initial_target: [f64; 3],
}

#[derive(serde::Deserialize)]
pub struct MotorCfg {
    pub resistance_ohm: f64,
    pub kt_nm_per_amp: f64,
    pub kv_v_per_rads: f64,
    pub inertia_kg_m2: f64,
    pub damping_nm_s_rad: f64,
    #[serde(default)]
    pub encoder_noise_std_rad: f64,
    #[serde(default)]
    pub encoder_quantization_rad: f64,
    #[serde(default)]
    pub coulomb_static_nm: f64,
    #[serde(default)]
    pub coulomb_kinetic_nm: f64,
    #[serde(default)]
    pub use_rk4: bool,
}

impl MotorCfg {
    pub fn to_motor_params(&self) -> MotorParams {
        MotorParams {
            r: self.resistance_ohm,
            kt: self.kt_nm_per_amp,
            kv: self.kv_v_per_rads,
            j: self.inertia_kg_m2,
            b: self.damping_nm_s_rad,
            coulomb_static_nm: self.coulomb_static_nm,
            coulomb_kinetic_nm: self.coulomb_kinetic_nm,
            encoder: EncoderConfig {
                noise_std_rad: self.encoder_noise_std_rad,
                quantization_rad: self.encoder_quantization_rad,
            },
            integration: if self.use_rk4 {
                IntegrationMethod::RK4
            } else {
                IntegrationMethod::Euler
            },
        }
    }
}

#[derive(serde::Deserialize)]
pub struct JointCfg {
    pub link_length_m: f64,
    pub link_mass_kg: f64,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub max_vel_rads: f64,
    pub max_accel_rads2: f64,
    pub min_angle_deg: f64,
    pub max_angle_deg: f64,
    #[serde(default)]
    pub kf: f64,
}

impl JointCfg {
    pub fn build(&self, motor_params: MotorParams) -> Joint {
        // Bound the PID's own output to the motor's actual voltage limit —
        // past that, extra command can never reach the motor anyway, so
        // conditional-integration anti-windup (see PidController) should
        // kick in there rather than let the integrator wind up chasing an
        // output the actuator can't deliver.
        let pid = PidController::new(self.kp, self.ki, self.kd)
            .with_output_limits(-MAX_VOLTAGE, MAX_VOLTAGE);

        Joint::new(
            self.link_length_m,
            self.link_mass_kg,
            MotorSim::with_params(motor_params),
            pid,
            TrapezoidProfile::new(self.max_vel_rads, self.max_accel_rads2),
        )
        .with_limits(
            self.min_angle_deg.to_radians(),
            self.max_angle_deg.to_radians(),
        )
        .with_velocity_ff(self.kf)
    }
}

impl Config {
    pub fn from_toml_str(s: &str) -> Result<Config, toml::de::Error> {
        toml::from_str(s)
    }

    /// The embedded default configuration (the repo's `config.toml` as it
    /// was at compile time). Panics if it fails to parse — that would mean
    /// the embedded file itself is broken, a build-time bug, not a runtime
    /// condition.
    pub fn embedded_default() -> Config {
        Self::from_toml_str(EMBEDDED_DEFAULT).expect("embedded default config.toml is invalid")
    }

    /// Read and parse `path`, falling back to [`Config::embedded_default`]
    /// on any I/O or parse error. Pure — callers that want to report the
    /// distinction (missing file vs. malformed) should call
    /// `std::fs::read_to_string`/`from_toml_str` directly instead.
    pub fn load_or_default(path: &str) -> Config {
        std::fs::read_to_string(path)
            .ok()
            .and_then(|text| Self::from_toml_str(&text).ok())
            .unwrap_or_else(Self::embedded_default)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_default_parses() {
        let cfg = Config::embedded_default();
        assert!(cfg.shoulder.link_length_m > 0.0);
    }

    #[test]
    fn load_or_default_falls_back_on_missing_file() {
        let cfg = Config::load_or_default("/nonexistent/path/does/not/exist.toml");
        assert!(cfg.elbow.link_length_m > 0.0);
    }

    #[test]
    fn load_or_default_falls_back_on_malformed_toml() {
        let dir = std::env::temp_dir();
        let path = dir.join(format!(
            "arm_core_test_bad_config_{}.toml",
            std::process::id()
        ));
        std::fs::write(&path, "not valid toml {{{").unwrap();
        let cfg = Config::load_or_default(path.to_str().unwrap());
        assert!(cfg.pan.max_vel_rads > 0.0);
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn joint_cfg_build_applies_limits_and_ff() {
        let cfg = Config::embedded_default();
        let joint = cfg.shoulder.build(cfg.motor.to_motor_params());
        assert_eq!(joint.length, cfg.shoulder.link_length_m);
        let (_, _, _, kf) = joint.gains();
        assert_eq!(kf, cfg.shoulder.kf);
    }
}
