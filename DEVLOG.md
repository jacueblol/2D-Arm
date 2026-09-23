# Development Log

Reverse-chronological. One entry per milestone. This is the narrative
successor to the old project's `PROGRESS.md`, which stays on `main` as the
historical record of the pre-rewrite prototype rather than being carried
forward as live state.

## M2 — PID with anti-windup (2026-09-22)

**What and why.** The old `PidController` was the simplest possible PID: no
output clamp, no anti-windup. That was fine as long as the loop never
saturated for long — but a real actuator has limits (the motor's voltage
clamps at ±12V), and any PID whose integral term keeps accumulating while
the output is already pinned at that limit will "wind up": the integral
grows far past what's actually needed, and once the error finally reverses,
the controller keeps driving the output at the limit for a long stretch
purely to unwind the excess integral — a classic, well-documented control
bug, and the excuse for making this its own milestone rather than folding it
quietly into the joint work in M3.

`arm_core::pid::PidController` now takes optional `output_min`/`output_max`
bounds (`with_output_limits`; unbounded by default, so old-style unbounded
behavior is still available for anywhere it's wanted) and implements
**clamping anti-windup via conditional integration**: once the output has
saturated in a direction, further error pushing the same way is not
integrated, so the integral term never grows past what's needed to hold the
output at its limit. Error that reverses and would pull the output back out
of saturation is still integrated immediately — recovery isn't delayed.

**The regression case.** `anti_windup_bounds_integrator_under_sustained_saturation`
drives a controller with `kp=0.05, ki=1.0` under a sustained error of 100 for
10 seconds simulated (1000 steps at 10ms). An unbounded controller's
integral term grows to >500 over that run — genuinely unbounded, it would
keep climbing forever. The same controller with `with_output_limits(-12, 12)`
stays under 20: it climbs only until the output hits the limit (around
integ≈7, matching the arithmetic: `kp*error + integ = 12` → `integ = 12 -
0.05*100 = 7`), then freezes. A second test confirms that once saturated,
an error reversal still reduces the integrator immediately rather than
staying artificially frozen.

**What's next.** M3: `Joint` — wires a motor + PID + trapezoidal profile +
limits together, config-driven, tested against the real `config.toml`
instead of hand-duplicated values.

## M1 — Motor physics core (2026-09-22)

**What and why.** Ported the motor physics from the old prototype into
`arm_core::motor`, headless — no Bevy in sight, `cargo test -p arm_core`
finishes in well under a second even though the DC-motor ODE, RK4
integration, and the Coulomb stiction state machine are all exercised by the
same 14 tests the old code had. Getting this right first matters: everything
downstream (PID, joints, the whole arm) sits on top of this.

Rather than one 700-line file, split by concern:
- `motor/sim.rs` — the ODE itself (`dω/dt = (Kt·I − b·ω − τ_load − τ_friction)/J`),
  Euler and RK4 integration, current/torque observers, gravity feedforward.
- `motor/friction.rs` — `Stiction`, the Coulomb static/kinetic friction
  state machine, as a self-contained type that only knows about torques and
  velocity (not R/Kt/Kv) — `sim.rs` computes the friction-free net torque and
  hands it to `Stiction::resolve`. This is a real improvement over the old
  code, where the stiction logic and the electrical model were interleaved
  in one method: the state machine can now be reasoned about (and tested)
  independently of the motor's electrical parameters.
- `motor/encoder.rs` — quantization-then-noise, read-side only, unchanged
  in behavior from the old code.

The `MotorFn` trait and `MotorIO` wrapper from the old code are gone: `MotorFn`
was never actually used polymorphically anywhere in the old codebase (checked
before dropping it), and `MotorIO` only existed to bundle position/velocity/
voltage with a logger — now that the logger is cut (see M0), those three
fields just live directly on `MotorSim`.

**Verified, not assumed.** All 14 ported tests pass with the same tolerances
as before — encoder quantization/noise, stiction breakaway/re-latch/
zero-friction-equivalence, current/torque estimation at stall and under
back-EMF, and both RK4 convergence properties (matches Euler at steady
state; more accurate than Euler at large dt against a fine-grained
reference). `cargo clippy -p arm_core --all-targets -- -D warnings` is clean.

**What's next.** M2: `PidController` with anti-windup and output clamping
added deliberately (the old controller had neither).

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
