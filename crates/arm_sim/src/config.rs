use arm_core::config::Config;
use bevy::log::{info, warn};

const CONFIG_PATH: &str = "config.toml";

/// Load `config.toml` from the working directory, logging the outcome.
/// Falls back to the embedded default on any missing-file or parse error —
/// see `arm_core::config::Config::load_or_default` for the pure logic this
/// wraps with app-level logging.
pub fn load_config() -> Config {
    match std::fs::read_to_string(CONFIG_PATH) {
        Ok(text) => match Config::from_toml_str(&text) {
            Ok(cfg) => {
                info!("Loaded {CONFIG_PATH}");
                cfg
            }
            Err(e) => {
                warn!("Parse error in {CONFIG_PATH}: {e} — using embedded defaults");
                Config::embedded_default()
            }
        },
        Err(_) => {
            warn!("{CONFIG_PATH} not found — using embedded defaults");
            Config::embedded_default()
        }
    }
}
