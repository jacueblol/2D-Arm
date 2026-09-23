# Prompt: 2D-Arm Project Onboarding & Game Plan

Paste this into Claude Code at the root of the `2D-Arm` repo (the actual local clone,
not a fresh zip export — I want git history if it exists).

---

## Context

This is my personal side project, `2D-Arm`. The long-term goal is a 2D articulated
robotic arm — simulation first, real hardware eventually. The current code
(`PID_Control/`) is a Rust project structured like FRC/WPILib robot code: a
`RobotBase` trait with `robot_init`/`robot_periodic` and separate
`autonomous`/`teleop` init+periodic hooks, a simulated DC motor
(`motor::motor_sim::MotorSim`), a stateless `PidController`, a colored
console `logger`, and a `Ticker` for loop timing. I have FRC mentoring
experience (WPILib command-based robot code), which is why the architecture
looks the way it does — that's intentional, not accidental complexity.

I haven't touched this in a while and want to get back into it. **Do not start
writing or refactoring code yet.** This session is discovery + planning only.
I want you to get fully oriented, tell me honestly where things actually
stand, and then help me figure out what to build next — asking me questions
where the direction is genuinely my call, not assuming.

## Working conventions for this session

**Keep a persistent `PROGRESS.md` file at the repo root.** Create it now if
it doesn't exist. This is the source of truth for project state across
sessions — I don't want to re-derive "where did I leave off" from scratch
every time I open Claude Code. Structure it with sections like: *Current
State* (what actually works today, verified not assumed), *Architecture
Notes* (the FRC/WPILib-style patterns in use and why), *Open Questions*
(decisions still pending), and *Plan / Milestones* (the phased roadmap,
with checkboxes). Update it as you go — after the build verification, after
I answer the open questions in Step 4, and after the plan is drafted in
Step 5. Treat it the way you'd treat a running lab notebook, not a one-time
report.

**Use subagents for anything exploratory, experimental, or open-ended.**
Any time a task means going down a rabbit hole — chasing what
`motor_position_command.rs` was probably trying to become, digging through
`git log` for buried context, trying a couple of different approaches to
see what compiles, researching how command-based schedulers or 2-link
inverse kinematics are typically structured in Rust — dispatch that to a
subagent rather than doing it inline in this conversation. Let the subagent
go deep and mess around freely; you just want the distilled conclusion back
here. This keeps the main thread focused on decisions with me instead of
getting cluttered with exploration transcripts. Use your judgment on when
something's simple enough to just do directly (e.g. reading one file)
versus worth spinning off.

## Step 1 — Read everything

Read every file in the repo, not just the `.rs` files: `README.md`,
`Cargo.toml`, both `.gitignore` files, and all of `PID_Control/src/`
including submodules (`motor/`, `pid/`). If there's a `.git` history, dispatch
a subagent to dig through it (`git log --oneline --all`, and full diffs on
anything touching `motor_position_command.rs`) and report back a summary of
what it finds — that kind of history archaeology is exactly the sort of
rabbit hole to hand off rather than do inline.

## Step 2 — Verify the actual state, don't trust the README

The README describes a C++/CMake project (`Vector2D.h`, `Link2D.h`,
`Simulation.h`, `main.cpp`) that does not exist in the current tree — it's
stale, left over from an earlier iteration before the Rust rewrite. Confirm
this yourself rather than taking my word for it.

Then actually try to build it:

```
cd PID_Control
cargo check
cargo build
```

Report the real compiler output. In particular I want to know:

- Does `motor_position_command.rs` currently compile at all, or is it dead
  code that isn't declared as a module anywhere (I don't see a `mod
  motor_position_command;` in `main.rs`)? If it's excluded from the build,
  say so explicitly — I don't want you fixing imports in a file that isn't
  even part of the binary without me knowing that's what's happening.
- Does the current `main.rs` loop actually move the simulated motor, or does
  it just spin an empty `robot_periodic()`/`teleop_periodic()` loop? Trace
  the actual data flow from `PidController` → `MotorSim` and tell me if
  they're connected.

If figuring out what `motor_position_command.rs` was reaching for (a real
`CommandBase`/scheduler setup?) takes actual investigation rather than a
quick read, send a subagent to reconstruct the likely intended design from
the broken imports and report back a plausible picture — don't burn the
main thread's context on trial and error.

## Step 3 — Give me a plain-language state-of-the-project summary

Before any planning, write up what's actually here today: what runs, what's
scaffolded but inert, what's broken/orphaned, and what architectural
pattern you think I was going for with the command-style file
(`motor_position_command.rs`) given the FRC-inspired structure elsewhere.
Call out anything else inconsistent or half-finished that I haven't
mentioned. Give me this in chat, and also write the *Current State* and
*Architecture Notes* sections of `PROGRESS.md` with it.

## Step 4 — Ask me before assuming scope

Don't guess at direction on things that are genuinely open design decisions.
At minimum, ask me about:

1. **Scope of "arm"**: right now this only simulates a single motor/joint.
   Do I want you to help build out real multi-link 2D kinematics (forward
   kinematics, and eventually inverse kinematics) on top of the existing
   motor/PID groundwork, or is the near-term goal just to get single-joint
   PID control fully working end-to-end first?
2. **The orphaned command file**: finish it out as a real WPILib-style
   Command/Subsystem scheduler (multiple simultaneous commands, a proper
   `CommandBase` trait + scheduler loop), or scrap it and keep the simpler
   direct `RobotBase` periodic structure for now?
3. **Target**: sim-only for the foreseeable future, or is real hardware
   (motor controllers, encoders) on the roadmap soon enough that it should
   shape the `MotorFn`/`MotorIO` abstraction now (e.g. a future
   `motor_hw.rs` alongside `motor_sim.rs`)?
4. **Telemetry**: keep the CSV + PNG plot via `plotters`, or is there
   interest in something more interactive/real-time later?

Ask these as a short list, wait for my answers, don't proceed on
assumptions for any of them. Log the questions and my answers in the *Open
Questions* section of `PROGRESS.md` as we go, so the reasoning behind future
decisions is traceable later.

If it'd help you frame any of these questions well — e.g. showing me what a
minimal command-scheduler would actually look like in Rust, or what 2-link
IK typically involves — have a subagent throw together a quick
throwaway sketch or spike to ground the question, rather than speculating
in the abstract. Small experiment, not production code.

## Step 5 — Once I've answered, draft a phased game plan

Turn my answers into a concrete, milestone-based plan (not just a flat todo
list) — something like: Milestone 1 gets a single joint under closed-loop
PID control actually moving and logging correctly end-to-end; Milestone 2
addresses whatever I said about the command architecture; Milestone 3 is
kinematics, etc. For each milestone, call out roughly what files change and
what "done" looks like (e.g. "motor reaches setpoint within X rad, plot
shows settling"). Keep it something we can execute over multiple sessions,
not a one-shot rewrite.

Write this plan into the *Plan / Milestones* section of `PROGRESS.md` as
checkboxes so future sessions (including subagents) can see at a glance
what's done and what's next.

Also flag the stale README as something to fix once the plan is set — it
should describe the actual Rust/PID architecture, not the old C++ version.

**Stop after presenting the plan and wait for my go-ahead before writing
any code.**
