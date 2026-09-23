# Articulated Arm Simulator

A 3-DOF (pan/shoulder/elbow) robotic arm simulator in Rust: closed-form inverse
kinematics, PID control with gravity and velocity feedforward, realistic motor
physics (Coulomb stiction, RK4 integration, encoder noise), trapezoidal
trajectory planning, and an interactive 3D Bevy visualization with a live
telemetry/tuning UI.

This is a from-scratch rewrite of an earlier prototype, done for the author's
portfolio. The rewrite is in progress — see [`DEVLOG.md`](DEVLOG.md) for the
running development narrative and [`docs/architecture.md`](docs/architecture.md)
for the technical design once it lands.

> **Status**: early scaffolding (M0). Not yet runnable end-to-end — check
> `DEVLOG.md` for current progress.

## Quickstart

```bash
cargo run -p arm_sim
```

## Workspace layout

- `crates/arm_core` — physics, control, and kinematics. No Bevy dependency;
  fully testable headlessly (`cargo test -p arm_core`).
- `crates/arm_sim` — the interactive Bevy application: rendering, input,
  egui telemetry/tuning panels.

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) at your
option.
