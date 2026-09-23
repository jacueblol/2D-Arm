# Development Log

Reverse-chronological. One entry per milestone. This is the narrative
successor to the old project's `PROGRESS.md`, which stays on `main` as the
historical record of the pre-rewrite prototype rather than being carried
forward as live state.

## M0 — Rewrite kickoff and scaffolding (2026-09-22)

**What and why.** Starting a full rewrite of the arm simulator for my
portfolio. The old code (still on `main`, under `PID_Control/`) actually
works — 27 passing tests, a real 3-DOF arm with PID control, closed-form IK,
gravity/velocity feedforward, trapezoidal trajectory planning, Coulomb
stiction, and a Bevy 3D visualization with live egui tuning — but it doesn't
read that way. The README described a years-old abandoned C++ prototype,
`main.rs` had grown into a 1165-line file mixing config parsing, a TCP
server, all Bevy ECS code, and the UI, and there was no CI, no license, and
some dead code (a telemetry logger that collected data but never wrote it
anywhere; an elbow-up IK solution that was computed but never used).

The physics and control math were already validated, so this rewrite ports
that math deliberately rather than re-deriving it, while fixing the
structural problems: a real crate boundary between simulation logic and the
Bevy app, anti-windup added to the PID controller, the elbow-up IK solution
surfaced as a real feature instead of dead code, and tests that run against
the actual `config.toml` instead of a hand-duplicated copy of its values.

**What changed.**
- New Cargo workspace: `crates/arm_core` (physics/control/kinematics, no
  Bevy dependency — enforced by the crate boundary, not just convention) and
  `crates/arm_sim` (the Bevy binary). The old code already had `arm3d.rs`
  importing `bevy::math::DVec3` despite having no ECS code in it; splitting
  into two crates makes that class of leak a compile error instead of a
  discipline problem.
- Dependency versions locked and verified compatible: Bevy 0.19.1,
  `bevy_egui` 0.42 (egui 0.36), `egui_plot` 0.37 (egui 0.36), `glam` 0.32
  (the same version Bevy 0.19 uses internally) — checked directly against
  crates.io before pinning, rather than inheriting whatever the old project
  happened to land on (Bevy 0.15, several versions behind).
- CI (`cargo fmt --check`, `cargo check --workspace --all-targets`,
  `cargo clippy -- -D warnings`, `cargo test --workspace`) — there was none
  before.
- Dual MIT/Apache-2.0 license — there was none before.
- Housekeeping on `main` ahead of branching: committed the WIP that was
  sitting in the working tree (Cartesian EE tracking, choreography, velocity
  feedforward — real, tested work that just hadn't been committed), removed
  9 stale leftover git worktrees and their branches from old agent sessions,
  and archived the original project-kickoff prompt into `docs/`.

**Cut, deliberately.** The TCP socket console and the CSV/plot telemetry
export are not carried into the rewrite — both were ad hoc and effectively
untested in the old code (the telemetry logger in particular: it allocated
and appended forever but nothing ever called its CSV writer). The live egui
charts cover the same need.

**What's next.** M1: port the motor physics core (`MotorSim`, Euler/RK4
integration, the Coulomb stiction state machine, encoder quantization/noise)
into `arm_core::motor`, headless, with its existing test coverage.
