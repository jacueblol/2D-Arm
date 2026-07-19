# 2D-Arm Project Progress

## Current State

### Build
- `cargo check` / `cargo build`: clean (0 errors, 0 warnings)
- `cargo run`: launches 3D Bevy window with orbit camera, 3-DOF arm, egui telemetry panel

### What's running
- 3D arm sim with PID control on 3 joints (pan/shoulder/elbow)
- IK: 3D target → pan angle + planar 2-link IK → joint setpoints
- Bevy 3D scene: cylinders, spheres, EE trail, workspace gizmos, velocity arrows
- egui side panel: tabbed joint telemetry charts (angle/sp/error/velocity vs time), drag controls for target X/Y/Z
- Angle-wrap fix on all joints (shortest-path PID across ±π boundary)

---

## Architecture Notes

### FRC/WPILib-style lifecycle (intentional)
- `RobotBase` trait: `robot_init`, `robot_periodic`, `auto_init/periodic`, `teleop_init/periodic`
- Mirrors WPILib's `TimedRobot` exactly — this is by design given FRC mentoring background
- `RobotMode`: AUTONOMOUS / TELEOP / None

### Motor module (`src/motor/`)
- `MotorSim` owns a `MotorIO` (state) + physics constants
- `MotorFn` trait: `set_voltage`, `reset`, `get_position_rad`, `get_velocity_rad_s`
- `MotorLogger` inside `MotorIO.logger: Option<MotorLogger>`

### PID (`src/pid/`)
- Stateless design — caller tracks `prev_error` and `integ_total` across ticks
- Returns `PidOut { output, error, integ_total }`

---

## Open Questions (answered)

| Question | Answer |
|---|---|
| Scope of "arm" | Multi-link 2D kinematics — build toward full 2-link arm |
| Command system | Scrap it — keep simpler direct RobotBase periodic loop |
| Hardware target | Sim-only for the foreseeable future |
| Telemetry | Keep CSV + PNG now; flag interactive/real-time (ratatui TUI) for M4 |

---

## Plan / Milestones

### Pre-work: Cleanup
- [x] Delete `src/motor_position_command.rs`
- [ ] Rewrite `README.md` (after M1 is stable)
- [x] Remove `LOOP_PERIOD_MS` from `constants.rs`
- [x] Remove commented-out dead code from `motor_io.rs`

### Milestone 1: Single-joint PID end-to-end ✅
- [x] Wire `PidController::step()` → `MotorSim::set_voltage()` → `MotorSim::step(dt)` in main loop
- [x] Write telemetry (CSV + PNG) on loop exit
- [x] Log when motor reaches setpoint

**Result**: Motor reached 44.72° (target 45°, tolerance 0.01 rad) within ~141ms. `motor_log.csv` and `motor_plot.png` written after 5s run.

### Milestone 2: Two-link arm simulation ✅
- [x] `src/arm/link.rs` — `Link { length, motor: MotorSim, pid: PidController, prev_error, integ_total, setpoint_rad }`
- [x] `src/arm/arm_sim.rs` — `ArmSim { link1, link2 }` with `step(dt)` and `forward_kinematics() -> (f64, f64)`
- [x] Log both joints; write link1_log.csv / link1_plot.png, link2_log.csv / link2_plot.png

**Result**: Link1 settles at 45.01°, Link2 at -30.00° (relative). End-effector converges to (1.4797 m, 0.9143 m). FK printed once per second during run.

### Milestone 3: Inverse kinematics ✅
- [x] `src/arm/ik.rs` — closed-form 2-link IK (law of cosines), returns `Option<IkSolutions>` with both elbow configurations
- [x] Wire: target (x, y) → IK → per-joint setpoints → ArmSim

**Result**: Target (1.2, 1.0 m) solved to θ1=66.14°, θ2=-60.00°. Both joints settled in ~180ms. End-effector converged to 0.31 mm from target.

### Milestone 4: Visualization ✅
- [x] Added Bevy 0.15 as dependency
- [x] Replaced standalone loop with Bevy App (sim state as Resource, step as Update system)
- [x] Sprites for link1 (blue), link2 (green), elbow joint (yellow), end-effector (orange), target (red)
- [x] `update_visuals` system recomputes transforms from FK each frame
- [x] Status text overlay: live joint angles, end-effector pos, reached status
- [x] `step_sim` system uses `time.delta_secs_f64()` — runs at display frame rate, dt-correct

**Result**: Builds cleanly. Run `cargo run` from `PID_Control/` to see window.

### Milestone 5: 3-DOF 3D Arm ✅
- [x] `src/arm/joint3d.rs` — `Joint3d` wrapping `MotorSim` + `PidController`, mirrors `Link` for 3D context
- [x] `src/arm/arm3d.rs` — `ArmSim3d { pan, shoulder, elbow }` with FK (quaternion chain) and IK
- [x] 3D IK: decompose target into pan angle (atan2) + planar (r, h) → reuse `ik::solve`
- [x] FK convention: `r_pan = Rot_Y(-t0)` so positive pan → +Z; cumulative quaternion chain for shoulder/elbow
- [x] Full 3D Bevy scene: orbit camera (left-drag, scroll zoom), 3D cylinders for links, spheres for joints/EE/target
- [x] Interactive target movement (WASD/QE at 0.6 m/s), space=pause, R=reset+re-IK
- [x] EE trail (VecDeque), workspace sphere gizmo, velocity arrows, floor grid, 3-axis crosshair at target
- [x] Status overlay: 3 joints × (angle, setpoint, error, velocity), EE pos, target pos, distance

**Config**: pan L=0m (PID 8/0.05/0.4), shoulder L=1.0m (PID 10/0.05/0.5), elbow L=0.8m (PID 10/0.05/0.5)
**Result**: Arm reaches 3D targets interactively. IK + FK verified: target (1.2, 0.8, 0.4) → pan=18.4°, sho=62°, elb=-68°, EE matches.

### Milestone 6: UI Modernization & Code Cleanup ✅
- [x] Fixed egui crash: `ctx_mut()` before context init → `try_ctx_mut()` with early return
- [x] Dark egui theme: `setup_egui` startup system, blue accent on selection/active widgets
- [x] Tabbed joint panel: Pan/Shoulder/Elbow tabs replace stacked collapsing headers — one joint shown at a time
- [x] Time-indexed X axis: charts now show `t (s)` instead of sample count; `PlotHistory.times` ring-buffer stores elapsed_s
- [x] `PlotHistory` refactored: 12 separate `VecDeque` fields → `[VecDeque<f32>; 3]` arrays, shared `times` axis
- [x] Target drag controls in egui panel: X/Y/Z `DragValue` widgets (complements keyboard WASD/QE)
- [x] Deleted `arm_sim.rs`, `link.rs` — legacy 2D modules superseded by `joint3d`/`arm3d`
- [x] Deleted `robot.rs`, `robot_base.rs`, `ticker.rs`, `constants.rs` — FRC boilerplate never compiled in Bevy main
- [x] Removed `plotters` dep + dead `plot()` method from `motor_io.rs`
- [x] `#[allow(dead_code)]` on intentional public API (`MotorLogger`, logger helpers, `IkSolutions.elbow_pos`)
- [x] 0 warnings in build

**Architecture note**: parallel subagents (isolated worktrees) used — Agent A owned `main.rs`, Agent B owned everything else. No merge conflicts.

### Milestone 7: Realistic Physics + Trajectory Planning ✅
- [x] `TrapezoidProfile` — per-joint trapezoidal velocity profile; setpoint ramps at bounded vel/accel
- [x] Realistic motor params: `j=5 kg·m²`, `kt=kv=8`, `r=2Ω`, `b=1` — max ~1.45 rad/s, overdamped settle ~2.5s
- [x] Gravity loading: `arm3d::step()` computes `τ_gravity` from FK + link masses, sets on each joint motor
- [x] Socket console: TCP port 7878, `get`/`set_target`/`pause`/`resume`/`reset` over `socat`/`nc`
- [x] `serde_json` state snapshot pushed every frame; full JSON on `get`

### Milestone 8: Gravity Feedforward ✅
- [x] `MotorSim::gravity_feedforward_volts()` — `V_ff = τ_load × R / Kt`
- [x] Applied in `Joint3d::step()` before `set_voltage` — PID only corrects residual error, not gravity sag
- [x] Eliminates integral wind-up from fighting gravity; arm holds horizontal without steady-state error

### Milestone 9: Joint Limits ✅
- [x] `Joint3d::with_limits(min_rad, max_rad)` builder — defaults to unlimited
- [x] `set_setpoint()` clamps goal to `[min, max]` after shortest-path wrap
- [x] Limits per-joint in `config.toml`: shoulder -30°–135°, elbow -150°–10°, pan ±180°
- [x] Test: `joint_limits_clamp_goal` verifies clamping and pass-through within range

### Milestone 10: Config File ✅
- [x] `config.toml` at `PID_Control/config.toml` — motor params, PID gains, trajectory limits, joint limits, initial target
- [x] `load_config()` reads TOML at startup; falls back to embedded defaults on missing/parse error
- [x] `MotorSim::with_params(MotorParams)` replaces hardcoded defaults — all motor params driven by config
- [x] `JointCfg::build()` constructs fully-configured joint from config section
- [x] No recompile needed to retune gains or change joint limits

---

### Long-term vision
- Full 3D articulated arm simulation → hardware target
- Real motor controllers, encoders, embedded comms layer (when ready)

### Next candidates (M11+)
- **Cartesian straight-line paths**: re-run IK at each trajectory step so EE traces a line, not an arc
- **Velocity feedforward**: add `kF × v_profile` to PID output for smoother trajectory tracking
- **Encoder simulation**: add noise/quantization to `get_position_rad()` for hardware-realistic testing
- **Serial/CAN protocol**: define a message format to talk to real motor controllers
