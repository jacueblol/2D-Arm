//! Physics, control, and kinematics core for the articulated-arm simulator.
//!
//! This crate has no dependency on Bevy or any rendering/ECS framework — every
//! type here must be testable and usable headlessly. `arm_sim` is the only
//! crate that wires this into a Bevy app.

#[cfg(test)]
mod tests {
    #[test]
    fn workspace_compiles() {
        assert_eq!(2 + 2, 4);
    }
}
