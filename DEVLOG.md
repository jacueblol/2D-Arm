# Development Log

Reverse-chronological. One entry per milestone. This is the narrative
successor to the old project's `PROGRESS.md`, which stays on `main` as the
historical record of the pre-rewrite prototype rather than being carried
forward as live state.

## M7 — Interactive control (2026-09-22)

**What and why.** The arm is now something you drive, not just watch.
Ported from the old prototype: an orbit camera (`OrbitCam` resource,
left-drag to orbit, scroll to zoom), WASD/QE keyboard target jogging,
Space to pause, R to reset, an EE trail that fades in from its oldest
point, a translucent workspace-reachability sphere, and a target crosshair
that turns gray when jogged past the reachable boundary. Split across new
`sim/camera.rs`, `sim/input.rs`, and `sim/gizmos.rs` modules rather than
piling into `scene.rs`.

No Cartesian interpolation yet (that's M8) — jogging the target just calls
`Arm::set_target` every frame it moves, which hard-retargets each joint's
trapezoidal profile from its current position and velocity each time. At
60 fps with small per-frame deltas this reads as smooth continuous
tracking in practice; the visibly bounded, non-instant motion **is** the
trapezoidal profile at work, which is the whole point of this milestone's
"visible ramp" goal — no special-casing needed, it falls out of M3's
`Joint` doing its job.

**A second Bevy 0.19 API break**, again caught by the compiler rather than
assumed: mouse motion/wheel events are read via `MessageReader`, not
`EventReader` — Bevy renamed its event system to "messages" between 0.15
and 0.19 (`MouseMotion`/`MouseWheel` now derive `Message`, not `Event`).

**Verification, and where it fell short.** Built and ran the binary
against the real Wayland session again; confirmed clean startup, correct
rendering of the new gizmos (EE trail, target crosshair, floor grid all
visible in a screenshot), and — importantly — got at least one clear
positive signal that input really reaches the app: a `Space` keystroke
sent via `wtype` did toggle `SimState.paused` (visible in the status
overlay), proving the `ButtonInput` pipeline is wired correctly end to
end. What I could *not* cleanly verify automated: individual WASD taps
in isolation. `wtype`'s virtual-keyboard events appear to queue at the
Wayland seat level somewhat independently of which window nominally has
focus, and this sandbox has no pointer-click tool (`ydotool`/`wlrctl`
aren't installed) to force genuine focus the way a real click would — so
repeated single-key tests kept landing with unpredictable delay or not
at all, rather than reproducing cleanly. This is a testing-environment
gap, not a code-confidence gap: the logic is a faithful port of
previously-verified behavior, it type-checks against the real APIs, and
the one clean signal I did get (pause) confirms the wiring works. Noting
this explicitly rather than overclaiming a full interactive pass — worth
a manual `cargo run -p arm_sim` check with a real keyboard/mouse.

**What's next.** M8: Cartesian moves (`CartesianTraj`, straight-line EE
interpolation) and the 9 choreography sequences — the strongest demo
material in the whole project.

## M6 — First pixel: Bevy visualization MVP (2026-09-22)

**What and why.** `arm_sim` finally does something: a real window, a real
arm. `SimPlugin` wraps `arm_core::arm::Arm` in a `SimState` resource;
`scene::setup` spawns a fixed camera, lighting, ground, and meshes for the
two links, elbow joint, end-effector, and target; `scene::update_visuals`
repositions everything from `forward_kinematics()` every frame;
`scene::update_status` shows a minimal text overlay (joint angles, EE and
target position). No camera control, no keyboard/mouse input, no egui yet —
those are M7 and M9. This is deliberately the smallest possible "it's
alive" milestone.

**Porting across four Bevy major versions surfaced real API breaks**,
checked against the actual downloaded Bevy 0.19.1 source in the local
cargo registry cache rather than assumed from the old (Bevy 0.15) code:
- `AmbientLight` is no longer a `Resource` — it's now a `Component` you
  attach to a camera (to override the scene default), with a new
  `GlobalAmbientLight` resource for the scene-wide default. Missing this
  distinction would've been a silent behavior change, not a compile error —
  `insert_resource(AmbientLight {...})` no longer exists to fail loudly, so
  it had to be caught by reading the type definitions, not by the compiler.
- `Cylinder::new(radius, height)` now takes **full** height and halves it
  internally — the old code passed a pre-halved length (`link_length / 2.0`),
  which would have silently rendered every link at half its real length.
  Caught by reading `Cylinder::new`'s source, not by a compiler error.
- `DirectionalLight.shadows_enabled` was renamed `shadow_maps_enabled`.
- `TextFont.font_size` changed from `f32` to a `FontSize` enum
  (`FontSize::Px(13.0)` for the old pixel-size behavior).
- `WindowResolution` now has an unambiguous `::new(u32, u32)` — the old
  `(1280.0, 960.0).into()` triggered an f32-vs-other-numeric-type
  ambiguity warning that's absent here from the start.

**Verified by actually running it**, not just `cargo check`: built the
binary, launched it against the real Wayland session, confirmed the window
opens (`Articulated Arm Simulator`, via `hyprctl clients`), and captured a
screenshot with `grim`. It shows the arm converging toward the configured
initial target (`end-effector (1.152, 0.818, 0.404)` approaching
`target (1.200, 0.800, 0.400)`), correctly colored links/joints, and the
status overlay rendering live angle/position text. Startup log is clean —
no warnings, no panics; Vulkan (`radv`) backend, GPU clustering and
preprocessing both report supported.

**What's next.** M7: orbit camera, WASD/QE target jog, EE trail, workspace
gizmo — the first genuinely interactive milestone.

## M5 — Full 3-DOF arm, still headless (2026-09-22)

**What and why.** `arm_core::arm::Arm` (`arm/assembly.rs` — clippy's
`module_inception` lint rejects a module literally named `arm::arm`)
orchestrates the three `Joint`s and the `kinematics` module into the
complete arm: gravity coupling (ported verbatim, same τ_shoulder/τ_elbow
formulas and sign convention as the old prototype), `set_target` (IK →
per-joint setpoints), `forward_kinematics`/`ee_pos`. This is the milestone
the whole headless-first approach was building toward: the control loop
is fully proven correct — closed-loop IK-to-PID-to-motor convergence,
against real `config.toml` gains — before a single pixel gets drawn in M6.

`Arm` also makes M4's elbow-up work land somewhere real: it carries an
`elbow_config: ElbowConfig` (default `Down`, matching old behavior) that
`set_target` passes into `kinematics::solve_3d`, with
`with_elbow_config`/`set_elbow_config` to change it — once M6+ exists this
is a straightforward UI toggle away from being a user-visible feature
instead of a config-file curiosity.

**Verified, not assumed.** `crates/arm_core/tests/config_driven.rs` grew
convergence tests for three target poses (forward reach, lateral reach
against pan + gravity, and the hardest case — high shoulder elevation under
maximum gravity torque), each settling within 15-20mm of the commanded
target after 30 simulated seconds, all built from
`Config::embedded_default()` rather than a hand-duplicated copy of gains —
closing the drift risk the old `arm3d.rs` test helper had. All passed on
the first run at the same tolerances the old prototype's tests used,
confirming the ported gravity coupling and M2's anti-windup bound
(`±MAX_VOLTAGE` on each joint's PID) didn't change steady-state behavior.

**What's next.** M6: the first pixel. A minimal Bevy app in `arm_sim` — a
`SimPlugin` wrapping this `Arm` as a resource, meshes positioned from
`forward_kinematics` each frame, targeting Bevy 0.19 from the start.

## M4 — Kinematics: forward + inverse (2026-09-22)

**What and why.** `arm_core::kinematics` ports the quaternion-chain forward
kinematics and the closed-form 2-link inverse kinematics from the old
prototype — both pure math, no `Joint`/`Arm` orchestration, which is why
they're a separate module from `arm::joint` rather than living inside it.

The headline change from the old code: the elbow-up solution is no longer
dead. The IK math has always computed both `elbow_pos` (bends upward) and
`elbow_neg` (bends downward) — the old code just always threw `elbow_pos`
away (`#[allow(dead_code)]` on the whole struct). `ElbowConfig` is now a
real, small enum (`Down` default — unchanged behavior — or `Up`), and
`IkSolutions::pick(config)` selects between them. `solve_3d(l1, l2, target,
elbow)` is the same pan-plus-planar decomposition as before, now taking that
choice as a parameter instead of hardcoding `elbow_neg`.

**Proven, not just ported.** The milestone plan called for "FK∘IK
round-trip property tests across sampled reachable workspace" — that's
`planar_fk_ik_round_trip_both_elbow_configs` (12 points around the annulus,
both elbow configs) and `solve_3d_fk_ik_round_trip` (4 targets through the
full pan+planar dispatch, both configs): solve IK for a target, feed the
result through forward kinematics, and check it reproduces the original
target to within 1e-9 — including for the elbow-up branch, which had never
been exercised by anything in the old codebase since nothing ever selected
it.

**What's next.** M5: `Arm` — pan+shoulder+elbow orchestration on top of
`Joint` and `kinematics`, gravity coupling, still fully headless. This is
where "control is proven correct before a single pixel is drawn" actually
lands — full closed-loop convergence tests against real config values,
matching the old `arm3d.rs` convergence tests but against `Config` instead
of hand-duplicated gains.

## M3 — Joint, config-driven (2026-09-22)

**What and why.** `arm_core::arm::Joint` wires a motor, a PID controller, a
trapezoidal motion profile, and joint limits together — this is what a
config file's `[shoulder]`/`[elbow]`/`[pan]` section turns into. Ported
straight from the old `Joint3d`, renamed (the "3d" suffix was doing no work
— nothing here is 2D vs 3D specific, it's just "one joint's control loop"),
plus:

- **`arm_core::config`** is new: the same `Config`/`JointCfg`/`MotorCfg`
  schema the old `main.rs` had inlined, now shared between `arm_core`'s own
  tests and (once M6 wires up the Bevy app) `arm_sim` — one schema, one
  `config.toml` (now at the repo root), so they can't drift apart the way a
  test helper hand-copying config values could. `JointCfg::build()` now also
  applies M2's anti-windup: the joint's PID output is bounded to
  `motor::MAX_VOLTAGE` (±12V), since command past that can never reach the
  motor anyway — this is where the M2 work actually gets used for the first
  time.
- The angle-wrap math (re-express a target as the nearest angle congruent
  to it, mod a full turn, so a PID error always takes the short way around
  the circle) is now a standalone free function,
  `nearest_congruent_angle`, with its own fast unit tests — it was inline
  logic duplicated at three call sites in the old `Joint3d`.
- `crates/arm_core/tests/config_driven.rs`: integration tests that call
  `Config::embedded_default()` and build real joints from it, rather than
  hand-duplicating gains/limits like the old `arm3d.rs` test helper did.
  One of these caught a real thing worth knowing: the shoulder's configured
  gains (kp=10, ki=0.8, kd=1.0) overshoot to ~47° on a 45° command before
  settling with a long decaying tail — checked empirically (logged the
  trajectory out to 38 simulated seconds) before picking a settle-time bound
  for the test, rather than guessing a tolerance and loosening it until
  green.

**What's next.** M4: closed-form forward/inverse kinematics — the
quaternion-chain FK and the 2-link IK — with the elbow-up solution finally
surfaced as a real, selectable option instead of dead code.

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
