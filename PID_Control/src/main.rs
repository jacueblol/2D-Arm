mod arm;
mod logger;
mod motor;
mod pid;

use std::collections::VecDeque;
use std::io::{BufRead, BufReader, Write};
use std::net::TcpListener;
use std::sync::{Arc, Mutex};
use bevy::{
    diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin},
    input::mouse::{MouseMotion, MouseWheel},
    math::DVec3,
    prelude::*,
};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use egui_plot::{Line, Plot, PlotPoints};
use crate::{
    arm::{arm3d::ArmSim3d, joint3d::Joint3d, trajectory::TrapezoidProfile},
    motor::motor_sim::{EncoderConfig, MotorParams, MotorSim},
    pid::pid_controller::PidController,
};

// ── config ────────────────────────────────────────────────────────────────────

#[derive(serde::Deserialize)]
struct Config {
    sim:      SimCfg,
    motor:    MotorCfg,
    pan:      JointCfg,
    shoulder: JointCfg,
    elbow:    JointCfg,
}

#[derive(serde::Deserialize)]
struct SimCfg {
    initial_target: [f64; 3],
}

#[derive(serde::Deserialize)]
struct MotorCfg {
    resistance_ohm:   f64,
    kt_nm_per_amp:    f64,
    kv_v_per_rads:    f64,
    inertia_kg_m2:    f64,
    damping_nm_s_rad: f64,
    #[serde(default)]
    encoder_noise_std_rad:    f64,
    #[serde(default)]
    encoder_quantization_rad: f64,
    #[serde(default)]
    coulomb_static_nm:  f64,
    #[serde(default)]
    coulomb_kinetic_nm: f64,
    #[serde(default)]
    use_rk4: bool,
}

#[derive(serde::Deserialize)]
struct JointCfg {
    link_length_m:   f64,
    link_mass_kg:    f64,
    kp:              f64,
    ki:              f64,
    kd:              f64,
    max_vel_rads:    f64,
    max_accel_rads2: f64,
    min_angle_deg:   f64,
    max_angle_deg:   f64,
    #[serde(default)]
    kf:              f64,
}

impl JointCfg {
    fn build(&self, motor_params: MotorParams) -> Joint3d {
        Joint3d::new(
            self.link_length_m,
            self.link_mass_kg,
            MotorSim::with_params(motor_params),
            PidController::new(self.kp, self.ki, self.kd),
            TrapezoidProfile::new(self.max_vel_rads, self.max_accel_rads2),
        ).with_limits(
            self.min_angle_deg.to_radians(),
            self.max_angle_deg.to_radians(),
        ).with_velocity_ff(self.kf)
    }
}

fn load_config() -> Config {
    let path = "config.toml";
    match std::fs::read_to_string(path) {
        Ok(text) => match toml::from_str(&text) {
            Ok(cfg)  => { logger::info("[Config]", format!("Loaded {path}")); cfg }
            Err(e)   => { logger::warn("[Config]", format!("Parse error in {path}: {e} — using defaults")); default_config() }
        },
        Err(_) => { logger::warn("[Config]", format!("{path} not found — using defaults")); default_config() }
    }
}

fn default_config() -> Config {
    toml::from_str(include_str!("../config.toml")).expect("embedded default config is invalid")
}

// ── socket console ────────────────────────────────────────────────────────────

const SOCKET_PORT: u16 = 7878;

/// Read-only snapshot of sim state, written by Bevy each frame, read by socket thread on `get`.
#[derive(Clone, Default, serde::Serialize)]
struct SimSnapshot {
    elapsed_s:      f64,
    paused:         bool,
    pan_angle_deg:  f64,
    pan_sp_deg:     f64,
    pan_vel_rads:   f64,
    sho_angle_deg:  f64,
    sho_sp_deg:     f64,
    sho_vel_rads:   f64,
    elb_angle_deg:  f64,
    elb_sp_deg:     f64,
    elb_vel_rads:   f64,
    ee:             [f64; 3],
    target:         [f64; 3],
    dist_mm:        f64,
    target_reached: bool,
    ik_valid:       bool,
}

/// Commands sent from the socket thread into Bevy.
enum SocketCmd {
    SetTarget(f64, f64, f64),
    Pause,
    Resume,
    Reset,
}

#[derive(Resource)]
struct SocketState {
    cmd_rx:   Mutex<std::sync::mpsc::Receiver<SocketCmd>>,
    snapshot: Arc<Mutex<SimSnapshot>>,
}

fn spawn_socket_server(
    cmd_tx:   std::sync::mpsc::Sender<SocketCmd>,
    snapshot: Arc<Mutex<SimSnapshot>>,
) {
    std::thread::spawn(move || {
        let listener = match TcpListener::bind(("127.0.0.1", SOCKET_PORT)) {
            Ok(l)  => { logger::info("[Socket]", format!("Console on 127.0.0.1:{SOCKET_PORT}")); l }
            Err(e) => { logger::warn("[Socket]", format!("Failed to bind port {SOCKET_PORT}: {e}")); return; }
        };
        for stream in listener.incoming() {
            let Ok(stream) = stream else { continue };
            let tx   = cmd_tx.clone();
            let snap = snapshot.clone();
            std::thread::spawn(move || handle_connection(stream, tx, snap));
        }
    });
}

fn handle_connection(
    mut stream: std::net::TcpStream,
    tx:         std::sync::mpsc::Sender<SocketCmd>,
    snapshot:   Arc<Mutex<SimSnapshot>>,
) {
    let Ok(reader_stream) = stream.try_clone() else { return };
    let reader = BufReader::new(reader_stream);
    let _ = stream.write_all(b"3D Arm Console. Type 'help' for commands.\n> ");
    for line in reader.lines() {
        let Ok(line) = line else { break };
        let response = dispatch_command(line.trim(), &tx, &snapshot);
        let _ = stream.write_all(format!("{response}\n> ").as_bytes());
    }
}

fn dispatch_command(
    line:     &str,
    tx:       &std::sync::mpsc::Sender<SocketCmd>,
    snapshot: &Arc<Mutex<SimSnapshot>>,
) -> String {
    let parts: Vec<&str> = line.split_whitespace().collect();
    match parts.as_slice() {
        ["get"] => {
            let s = snapshot.lock().unwrap();
            serde_json::to_string_pretty(&*s).unwrap_or_else(|e| format!("error: {e}"))
        }
        ["set_target", x, y, z] => {
            match (x.parse::<f64>(), y.parse::<f64>(), z.parse::<f64>()) {
                (Ok(x), Ok(y), Ok(z)) => { let _ = tx.send(SocketCmd::SetTarget(x, y, z)); "ok".into() }
                _ => "error: expected set_target <x> <y> <z>".into(),
            }
        }
        ["pause"]  => { let _ = tx.send(SocketCmd::Pause);  "ok".into() }
        ["resume"] => { let _ = tx.send(SocketCmd::Resume); "ok".into() }
        ["reset"]  => { let _ = tx.send(SocketCmd::Reset);  "ok".into() }
        ["help"] => concat!(
            "commands:\n",
            "  get                   — JSON snapshot of all joint/EE values\n",
            "  set_target <x> <y> <z> — move IK target\n",
            "  pause / resume        — freeze/unfreeze sim\n",
            "  reset                 — reset arm to zero"
        ).into(),
        [] => String::new(),
        _  => format!("unknown: '{line}'  (try 'help')"),
    }
}

// ── constants ─────────────────────────────────────────────────────────────────

const LINK_RADIUS:      f32   = 0.04;
const JOINT_RADIUS:     f32   = 0.06;
const EE_RADIUS:        f32   = 0.09;
const IK_TOLERANCE_M:   f64   = 0.012;  // ~12mm: realistic SS error under gravity load
const TRAIL_LEN:        usize = 400;
const TARGET_SPEED:     f32   = 0.6;   // m/s when key held
const CARTESIAN_SPEED:  f64   = 0.3;   // m/s EE linear speed for Cartesian moves
const PLOT_HISTORY:   usize = 300;

// ── resources ─────────────────────────────────────────────────────────────────

#[derive(Resource)]
struct SimState {
    arm: ArmSim3d,
    target: DVec3,
    target_reached: bool,
    ee_trail: VecDeque<Vec3>,
    paused: bool,
    elapsed_s: f64,
    ik_valid: bool,
    active_choreo: Option<crate::arm::choreography::WaypointSeq>,
    cursor_hit: Option<Vec3>,  // world-space ray/plane intersection for gizmo preview
}

#[derive(Resource)]
struct OrbitCam {
    yaw: f32,
    pitch: f32,
    distance: f32,
    focus: Vec3,
    dragging: bool,
}

impl OrbitCam {
    fn camera_pos(&self) -> Vec3 {
        Vec3::new(
            self.focus.x + self.distance * self.pitch.cos() * self.yaw.sin(),
            self.focus.y + self.distance * self.pitch.sin(),
            self.focus.z + self.distance * self.pitch.cos() * self.yaw.cos(),
        )
    }
}

#[derive(Resource)]
struct VisualEntities {
    link1: Entity,
    link2: Entity,
    elbow: Entity,
    ee: Entity,
    target_sphere: Entity,
}

/// Ring-buffer of time-series data for all three joints.
#[derive(Resource)]
struct PlotHistory {
    // Index: 0=Pan, 1=Shoulder, 2=Elbow
    angle: [VecDeque<f32>; 3],
    sp:    [VecDeque<f32>; 3],
    err:   [VecDeque<f32>; 3],
    vel:   [VecDeque<f32>; 3],
    times: VecDeque<f32>,   // elapsed_s per sample (shared X axis)
    max_len: usize,
}

impl PlotHistory {
    fn new(max_len: usize) -> Self {
        Self {
            angle:   std::array::from_fn(|_| VecDeque::new()),
            sp:      std::array::from_fn(|_| VecDeque::new()),
            err:     std::array::from_fn(|_| VecDeque::new()),
            vel:     std::array::from_fn(|_| VecDeque::new()),
            times:   VecDeque::new(),
            max_len,
        }
    }

    fn push_clamped(buf: &mut VecDeque<f32>, val: f32, max_len: usize) {
        buf.push_back(val);
        if buf.len() > max_len { buf.pop_front(); }
    }

    fn push(&mut self, state: &SimState) {
        let ml = self.max_len;
        let joints = [&state.arm.pan, &state.arm.shoulder, &state.arm.elbow];
        for (i, j) in joints.iter().enumerate() {
            Self::push_clamped(&mut self.angle[i], j.angle_rad() as f32, ml);
            Self::push_clamped(&mut self.sp[i],    j.get_setpoint_rad() as f32, ml);
            Self::push_clamped(&mut self.err[i],   j.error_rad() as f32, ml);
            Self::push_clamped(&mut self.vel[i],   j.velocity_rad_s() as f32, ml);
        }
        Self::push_clamped(&mut self.times, state.elapsed_s as f32, ml);
    }
}

#[derive(Resource)]
struct UiState {
    selected_joint: usize,  // 0=Pan, 1=Shoulder, 2=Elbow
    tuning_tab: bool,        // false=Telemetry, true=Tuning
}

// ── marker components ─────────────────────────────────────────────────────────

#[derive(Component)]
struct MainCamera;

#[derive(Component)]
struct StatusText;

// ── entry point ───────────────────────────────────────────────────────────────

fn main() {
    let cfg = load_config();

    let motor_params = || MotorParams {
        r:  cfg.motor.resistance_ohm,
        kt: cfg.motor.kt_nm_per_amp,
        kv: cfg.motor.kv_v_per_rads,
        j:  cfg.motor.inertia_kg_m2,
        b:  cfg.motor.damping_nm_s_rad,
        coulomb_static_nm:  cfg.motor.coulomb_static_nm,
        coulomb_kinetic_nm: cfg.motor.coulomb_kinetic_nm,
        encoder: EncoderConfig {
            noise_std_rad:    cfg.motor.encoder_noise_std_rad,
            quantization_rad: cfg.motor.encoder_quantization_rad,
        },
        integration: if cfg.motor.use_rk4 {
            crate::motor::motor_sim::IntegrationMethod::RK4
        } else {
            crate::motor::motor_sim::IntegrationMethod::Euler
        },
    };

    let mut arm = ArmSim3d::new(
        cfg.pan.build(motor_params()),
        cfg.shoulder.build(motor_params()),
        cfg.elbow.build(motor_params()),
    );

    let initial_target = DVec3::from_array(cfg.sim.initial_target);
    let ik_valid = arm.set_target(initial_target);
    logger::info("[3D-IK]", format!(
        "Target ({:.2},{:.2},{:.2})  pan={:.1} sho={:.1} elb={:.1} deg",
        initial_target.x, initial_target.y, initial_target.z,
        arm.pan.get_setpoint_rad().to_degrees(),
        arm.shoulder.get_setpoint_rad().to_degrees(),
        arm.elbow.get_setpoint_rad().to_degrees(),
    ));

    let snapshot = Arc::new(Mutex::new(SimSnapshot::default()));
    let (cmd_tx, cmd_rx) = std::sync::mpsc::channel::<SocketCmd>();
    spawn_socket_server(cmd_tx, snapshot.clone());

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "3D Arm Simulator".into(),
                resolution: (1280.0, 960.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(FrameTimeDiagnosticsPlugin)
        .add_plugins(EguiPlugin)
        .insert_resource(SimState {
            arm,
            target: initial_target,
            target_reached: false,
            ee_trail: VecDeque::new(),
            paused: false,
            elapsed_s: 0.0,
            ik_valid,
            active_choreo: None,
            cursor_hit: None,
        })
        .insert_resource(OrbitCam {
            yaw: std::f32::consts::FRAC_PI_4,
            pitch: 0.45,
            distance: 4.5,
            focus: Vec3::new(0.6, 0.4, 0.2),
            dragging: false,
        })
        .insert_resource(PlotHistory::new(PLOT_HISTORY))
        .insert_resource(UiState { selected_joint: 0, tuning_tab: false })
        .insert_resource(SocketState { cmd_rx: Mutex::new(cmd_rx), snapshot })
        .add_systems(Startup, (
            setup,
            setup_egui.after(bevy_egui::EguiStartupSet::InitContexts),
        ))
        .add_systems(Update, (
            handle_socket_cmds,
            handle_keyboard,
            handle_mouse_target,
            handle_camera,
            step_sim,
            run_choreography,
            update_plot_history,
            update_visuals,
            draw_gizmos,
            update_status,
            draw_charts,
            push_snapshot,
        ).chain())
        .run();
}

// ── helpers ───────────────────────────────────────────────────────────────────

/// Place a Y-axis cylinder between two world points (translation + rotation only;
/// length must already be baked into the mesh half_height).
fn link_transform(start: Vec3, end: Vec3) -> Transform {
    let dir = end - start;
    let mid = (start + end) * 0.5;
    let rot = if dir.normalize_or_zero().dot(Vec3::Y).abs() > 0.9999 {
        if dir.y >= 0.0 { Quat::IDENTITY } else { Quat::from_rotation_x(std::f32::consts::PI) }
    } else {
        Quat::from_rotation_arc(Vec3::Y, dir.normalize())
    };
    Transform { translation: mid, rotation: rot, scale: Vec3::ONE }
}

fn set_target(state: &mut SimState, new_target: DVec3) {
    if state.arm.set_target(new_target) {
        state.arm.start_cartesian_move(new_target, CARTESIAN_SPEED);
        state.target = new_target;
        state.target_reached = false;
        state.ik_valid = true;
        state.ee_trail.clear();
    } else {
        state.ik_valid = false;
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    state: Res<SimState>,
    orbit: Res<OrbitCam>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        MainCamera,
        Transform::from_translation(orbit.camera_pos()).looking_at(orbit.focus, Vec3::Y),
    ));

    // Lighting
    commands.insert_resource(AmbientLight { color: Color::WHITE, brightness: 300.0 });
    commands.spawn((
        DirectionalLight {
            illuminance: 15_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.9, 0.5, 0.0)),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d { normal: Dir3::Y, half_size: Vec2::splat(6.0) })),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.10, 0.10, 0.10),
            perceptual_roughness: 1.0,
            ..default()
        })),
    ));

    // Base pedestal
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.09, 0.05))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.35, 0.35),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));

    // Shoulder joint (static)
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(JOINT_RADIUS * 1.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.8, 0.8),
            ..default()
        })),
    ));

    let [_, sho, elb, ee_p] = state.arm.forward_kinematics();
    let (sho, elb, ee_p) = (sho.as_vec3(), elb.as_vec3(), ee_p.as_vec3());

    let l1_hh = (state.arm.shoulder.length / 2.0) as f32;
    let l2_hh = (state.arm.elbow.length / 2.0) as f32;

    let link1 = commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(LINK_RADIUS, l1_hh))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.25, 0.55, 1.0),
            ..default()
        })),
        link_transform(sho, elb),
    )).id();

    let elbow = commands.spawn((
        Mesh3d(meshes.add(Sphere::new(JOINT_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.85, 0.15),
            ..default()
        })),
        Transform::from_translation(elb),
    )).id();

    let link2 = commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(LINK_RADIUS, l2_hh))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.25, 0.85, 0.45),
            ..default()
        })),
        link_transform(elb, ee_p),
    )).id();

    let ee = commands.spawn((
        Mesh3d(meshes.add(Sphere::new(EE_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(1.0, 0.45, 0.1),
            emissive: LinearRgba::new(0.4, 0.12, 0.0, 1.0),
            ..default()
        })),
        Transform::from_translation(ee_p),
    )).id();

    let target_sphere = commands.spawn((
        Mesh3d(meshes.add(Sphere::new(EE_RADIUS * 1.25))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.95, 0.15, 0.15, 0.5),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_translation(state.target.as_vec3()),
    )).id();

    commands.insert_resource(VisualEntities { link1, link2, elbow, ee, target_sphere });

    // Status panel
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            padding: UiRect::all(Val::Px(10.0)),
            ..default()
        },
        BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.75)),
    )).with_children(|p| {
        p.spawn((
            StatusText,
            Text::new("Initializing..."),
            TextFont { font_size: 13.0, ..default() },
            TextColor(Color::srgb(0.9, 0.9, 0.9)),
        ));
    });
}

// ── systems ───────────────────────────────────────────────────────────────────

fn handle_keyboard(
    keys: Res<ButtonInput<KeyCode>>,
    mut state: ResMut<SimState>,
    time: Res<Time>,
) {
    if keys.just_pressed(KeyCode::Space) {
        state.paused = !state.paused;
    }
    if keys.just_pressed(KeyCode::KeyR) {
        state.arm.reset();
        state.ee_trail.clear();
        state.elapsed_s = 0.0;
        state.target_reached = false;
        let t = state.target;
        state.arm.set_target(t);
        state.arm.start_cartesian_move(t, CARTESIAN_SPEED);
    }

    // Choreography shortcuts — 1-9 launch sequences, C cancels
    {
        use crate::arm::choreography as choreo;
        let launch: Option<choreo::WaypointSeq> =
            if keys.just_pressed(KeyCode::Digit1) {
                Some(choreo::figure_eight(0.25))
            } else if keys.just_pressed(KeyCode::Digit2) {
                Some(choreo::circle_sweep(0.25))
            } else if keys.just_pressed(KeyCode::Digit3) {
                Some(choreo::helix(0.25))
            } else if keys.just_pressed(KeyCode::Digit4) {
                Some(choreo::wave_hello(0.30))
            } else if keys.just_pressed(KeyCode::Digit5) {
                Some(choreo::pick_and_place(0.25))
            } else if keys.just_pressed(KeyCode::Digit6) {
                Some(choreo::triangle(0.25))
            } else if keys.just_pressed(KeyCode::Digit7) {
                Some(choreo::knock_knock(0.25))
            } else if keys.just_pressed(KeyCode::Digit8) {
                Some(choreo::slow_drift(0.25))
            } else if keys.just_pressed(KeyCode::Digit9) {
                Some(choreo::zorro(0.25))
            } else {
                None
            };
        if let Some(mut seq) = launch {
            logger::info("[Choreo]", format!("Starting: {}", seq.label));
            seq.start(&mut state.arm);
            state.active_choreo = Some(seq);
        }
        if keys.just_pressed(KeyCode::KeyC) {
            if state.active_choreo.is_some() {
                logger::info("[Choreo]", "Cancelled choreography");
            }
            state.active_choreo = None;
        }
    }

    // Move target with WASD (XZ plane) + Q/E (vertical)
    let dt = time.delta_secs();
    let spd = (TARGET_SPEED * dt) as f64;
    let mut delta = DVec3::ZERO;
    if keys.pressed(KeyCode::KeyW) { delta.x += spd; }
    if keys.pressed(KeyCode::KeyS) { delta.x -= spd; }
    if keys.pressed(KeyCode::KeyA) { delta.z -= spd; }
    if keys.pressed(KeyCode::KeyD) { delta.z += spd; }
    if keys.pressed(KeyCode::KeyE) { delta.y += spd; }
    if keys.pressed(KeyCode::KeyQ) { delta.y -= spd; }
    if delta != DVec3::ZERO {
        let new_t = state.target + delta;
        set_target(&mut state, new_t);
        // Manual move cancels any active choreography
        state.active_choreo = None;
    }
}

/// Shift+click (hold/drag) to place the target on a horizontal plane at the current target's Y.
/// The cursor's world-space hit point is also stored for the gizmo preview ring.
fn handle_mouse_target(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window>,
    cameras: Query<(&Camera, &GlobalTransform), With<MainCamera>>,
    mut contexts: EguiContexts,
    mut state: ResMut<SimState>,
) {
    // Always update cursor_hit for the hover gizmo (even without click).
    state.cursor_hit = None;

    let shift = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);

    // Only show hover preview while Shift is held.
    if !shift { return; }

    // Don't steal clicks from egui.
    if let Some(ctx) = contexts.try_ctx_mut() {
        if ctx.wants_pointer_input() { return; }
    }

    let Ok(window) = windows.get_single() else { return };
    let Ok((camera, cam_xf)) = cameras.get_single() else { return };
    let Some(cursor_pos) = window.cursor_position() else { return };

    // Cast ray from camera through cursor position.
    let Ok(ray) = camera.viewport_to_world(cam_xf, cursor_pos) else { return };

    // Intersect with horizontal plane at target's current Y height.
    let plane_y = state.target.y as f32;
    let denom = ray.direction.y;
    if denom.abs() < 1e-6 { return; }
    let t = (plane_y - ray.origin.y) / denom;
    if t < 0.0 || t > 50.0 { return; }  // behind camera or too far

    let hit = ray.origin + *ray.direction * t;
    state.cursor_hit = Some(hit);

    // Shift+click (or hold to drag) commits the hit as the new target.
    if mouse_buttons.pressed(MouseButton::Left) {
        let new_target = DVec3::new(hit.x as f64, state.target.y, hit.z as f64);
        set_target(&mut state, new_target);
    }
}

fn handle_camera(
    mouse_buttons: Res<ButtonInput<MouseButton>>,
    mut motion: EventReader<MouseMotion>,
    mut wheel: EventReader<MouseWheel>,
    mut orbit: ResMut<OrbitCam>,
    mut cam_query: Query<&mut Transform, With<MainCamera>>,
) {
    let Ok(mut cam) = cam_query.get_single_mut() else { return };

    // Scroll to zoom
    for ev in wheel.read() {
        orbit.distance = (orbit.distance - ev.y * orbit.distance * 0.1).clamp(0.4, 20.0);
    }

    // Left-drag to orbit
    if mouse_buttons.pressed(MouseButton::Left) {
        orbit.dragging = true;
        for ev in motion.read() {
            orbit.yaw   -= ev.delta.x * 0.007;
            orbit.pitch  = (orbit.pitch - ev.delta.y * 0.007)
                .clamp(-std::f32::consts::FRAC_PI_2 + 0.05, std::f32::consts::FRAC_PI_2 - 0.05);
        }
    } else {
        for _ in motion.read() {}
        orbit.dragging = false;
    }

    let pos = orbit.camera_pos();
    cam.translation = pos;
    cam.look_at(orbit.focus, Vec3::Y);
}

fn step_sim(mut state: ResMut<SimState>, time: Res<Time>) {
    if state.paused { return; }
    let dt = time.delta_secs_f64().min(0.05);
    if dt > 0.0 {
        state.arm.step(dt);
        state.elapsed_s += dt;
    }

    let ee = state.arm.ee_pos().as_vec3();
    state.ee_trail.push_back(ee);
    if state.ee_trail.len() > TRAIL_LEN { state.ee_trail.pop_front(); }

    if !state.target_reached && !state.arm.is_cartesian_active() {
        let dist = (ee - state.target.as_vec3()).length() as f64;
        if dist < IK_TOLERANCE_M {
            state.target_reached = true;
            logger::success("[Sim]", format!("Reached target — {:.2} mm error", dist * 1000.0));
        }
    }
}

/// Advance the active choreography sequence each frame.
fn run_choreography(mut state: ResMut<SimState>, time: Res<Time>) {
    if state.paused { return; }
    if state.active_choreo.is_none() { return; }
    let dt = time.delta_secs_f64();

    // Temporarily move the sequence out so we can borrow arm independently.
    let mut choreo = state.active_choreo.take().unwrap();

    let dist = if let Some(tgt) = choreo.current_target() {
        (state.arm.ee_pos() - tgt).length()
    } else {
        0.0
    };

    choreo.advance(&mut state.arm, dist, dt);

    // Keep state.target in sync so the crosshair follows the active waypoint.
    if let Some(tgt) = choreo.current_target() {
        state.target = tgt;
        state.target_reached = false;
        state.ik_valid = true;
    }

    if choreo.is_done() {
        logger::success("[Choreo]", "Sequence complete");
    } else {
        state.active_choreo = Some(choreo);
    }
}

/// Push the current joint values into the plot ring-buffers (only when running).
fn update_plot_history(state: Res<SimState>, mut history: ResMut<PlotHistory>) {
    if state.paused { return; }
    history.push(&state);
}

fn update_visuals(
    state: Res<SimState>,
    entities: Option<Res<VisualEntities>>,
    mut transforms: Query<&mut Transform>,
) {
    let Some(e) = entities else { return };

    let [_, sho, elb, ee_p] = state.arm.forward_kinematics();
    let (sho, elb, ee_p) = (sho.as_vec3(), elb.as_vec3(), ee_p.as_vec3());

    if let Ok([mut tl1, mut tl2, mut te, mut tee, mut ttgt]) = transforms.get_many_mut([
        e.link1, e.link2, e.elbow, e.ee, e.target_sphere,
    ]) {
        *tl1 = link_transform(sho, elb);
        *tl2 = link_transform(elb, ee_p);
        te.translation = elb;
        tee.translation = ee_p;
        ttgt.translation = state.target.as_vec3();
    }
}

fn draw_gizmos(state: Res<SimState>, mut gizmos: Gizmos) {
    // EE trail
    let trail: Vec<Vec3> = state.ee_trail.iter().copied().collect();
    let len = trail.len();
    for (i, pair) in trail.windows(2).enumerate() {
        let alpha = (i + 1) as f32 / len as f32;
        gizmos.line(pair[0], pair[1], Color::srgba(1.0, 0.75, 0.15, alpha * 0.9));
    }

    // Workspace outer/inner boundary spheres
    let l1 = state.arm.shoulder.length as f32;
    let l2 = state.arm.elbow.length as f32;
    gizmos.sphere(Isometry3d::IDENTITY, l1 + l2, Color::srgba(0.3, 0.3, 0.6, 0.15));
    if (l1 - l2).abs() > 0.01 {
        gizmos.sphere(Isometry3d::IDENTITY, (l1 - l2).abs(), Color::srgba(0.3, 0.3, 0.6, 0.15));
    }

    // Target crosshair ring
    let tp = state.target.as_vec3();
    let tc = if state.ik_valid { Color::srgb(0.95, 0.2, 0.2) } else { Color::srgb(0.5, 0.5, 0.5) };
    gizmos.circle(Isometry3d::new(tp, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)), 0.12, tc);
    gizmos.line(tp - Vec3::new(0.15, 0.0, 0.0), tp + Vec3::new(0.15, 0.0, 0.0), tc);
    gizmos.line(tp - Vec3::new(0.0, 0.15, 0.0), tp + Vec3::new(0.0, 0.15, 0.0), tc);
    gizmos.line(tp - Vec3::new(0.0, 0.0, 0.15), tp + Vec3::new(0.0, 0.0, 0.15), tc);

    // Cursor hover ring — shows where right-click will place the target.
    if let Some(hit) = state.cursor_hit {
        let hover_col = Color::srgba(1.0, 1.0, 0.3, 0.55);
        gizmos.circle(
            Isometry3d::new(hit, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
            0.08, hover_col,
        );
        gizmos.line(hit - Vec3::new(0.10, 0.0, 0.0), hit + Vec3::new(0.10, 0.0, 0.0), hover_col);
        gizmos.line(hit - Vec3::new(0.0, 0.0, 0.10), hit + Vec3::new(0.0, 0.0, 0.10), hover_col);
    }

    // Velocity arrows (from elbow and EE)
    let [_, _, elb, ee_p] = state.arm.forward_kinematics();
    let (elb, ee_p) = (elb.as_vec3(), ee_p.as_vec3());

    let t0 = state.arm.pan.angle_rad() as f32;
    let t1 = state.arm.shoulder.angle_rad() as f32;
    let t2 = state.arm.elbow.angle_rad() as f32;
    let v1 = state.arm.shoulder.velocity_rad_s() as f32;
    let v2 = state.arm.elbow.velocity_rad_s() as f32;

    // Tangent direction at elbow (perpendicular to link1 in its plane)
    let r_pan = Quat::from_rotation_y(-t0);
    let r_sho = r_pan * Quat::from_rotation_z(t1);
    let r_elb = r_sho * Quat::from_rotation_z(t2);
    let elb_tangent = r_sho * Vec3::new(-t1.sin(), t1.cos(), 0.0);
    let ee_tangent  = r_elb * Vec3::new(-(t1+t2).sin(), (t1+t2).cos(), 0.0);

    let arrow_col = Color::srgba(0.3, 1.0, 0.4, 0.8);
    let v1_arrow = elb_tangent * v1 * 0.4;
    if v1_arrow.length() > 0.01 { gizmos.line(elb, elb + v1_arrow, arrow_col); }
    let v2_arrow = ee_tangent * (v1 + v2) * 0.4;
    if v2_arrow.length() > 0.01 { gizmos.line(ee_p, ee_p + v2_arrow, arrow_col); }

    // Floor grid
    let grid_col = Color::srgba(0.3, 0.3, 0.3, 0.3);
    for i in -5i32..=5 {
        let f = i as f32 * 0.5;
        gizmos.line(Vec3::new(f, 0.0, -2.5), Vec3::new(f, 0.0, 2.5), grid_col);
        gizmos.line(Vec3::new(-2.5, 0.0, f), Vec3::new(2.5, 0.0, f), grid_col);
    }
}

fn update_status(
    state: Res<SimState>,
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<&mut Text, With<StatusText>>,
) {
    let Ok(mut text) = query.get_single_mut() else { return };

    let fps = diagnostics
        .get(&FrameTimeDiagnosticsPlugin::FPS)
        .and_then(|d| d.smoothed())
        .unwrap_or(0.0);

    let ee  = state.arm.ee_pos();
    let dist_mm = (ee - state.target).length() * 1000.0;

    let pan_a  = state.arm.pan.angle_rad().to_degrees();
    let sho_a  = state.arm.shoulder.angle_rad().to_degrees();
    let elb_a  = state.arm.elbow.angle_rad().to_degrees();
    let pan_sp = state.arm.pan.get_setpoint_rad().to_degrees();
    let sho_sp = state.arm.shoulder.get_setpoint_rad().to_degrees();
    let elb_sp = state.arm.elbow.get_setpoint_rad().to_degrees();
    let pan_e  = state.arm.pan.error_rad().to_degrees();
    let sho_e  = state.arm.shoulder.error_rad().to_degrees();
    let elb_e  = state.arm.elbow.error_rad().to_degrees();
    let pan_v  = state.arm.pan.velocity_rad_s();
    let sho_v  = state.arm.shoulder.velocity_rad_s();
    let elb_v  = state.arm.elbow.velocity_rad_s();

    let sim_s    = if state.paused { "PAUSED " } else { "RUNNING" };
    let ik_s     = if state.ik_valid { "" } else { "  [OUT OF RANGE]" };
    let tgt_s    = if state.target_reached { "REACHED" } else { "moving..." };
    let choreo_s = match &state.active_choreo {
        Some(c) => format!("  [CHOREO: {}]", c.label),
        None    => String::new(),
    };

    text.0 = format!(
        "[{sim_s}]  FPS:{fps:5.0}  t:{:.2}s{ik_s}{choreo_s}\n\
         -----------------------------------------\n\
         PAN  angle:{pan_a:+7.2}  sp:{pan_sp:+7.2}  err:{pan_e:+6.2} deg\n\
              vel:{pan_v:+8.4} r/s\n\
         SHO  angle:{sho_a:+7.2}  sp:{sho_sp:+7.2}  err:{sho_e:+6.2} deg\n\
              vel:{sho_v:+8.4} r/s\n\
         ELB  angle:{elb_a:+7.2}  sp:{elb_sp:+7.2}  err:{elb_e:+6.2} deg\n\
              vel:{elb_v:+8.4} r/s\n\
         -----------------------------------------\n\
         EE  ({:.3}, {:.3}, {:.3}) m\n\
         TGT ({:.3}, {:.3}, {:.3}) m  {:.2}mm  [{tgt_s}]\n\
         -----------------------------------------\n\
         WASD/QE: move target   Shift+Click: place target   Space: pause   R: reset\n\
         1-9: choreo  C: cancel   LDrag: orbit   Scroll: zoom",
        state.elapsed_s,
        ee.x, ee.y, ee.z,
        state.target.x, state.target.y, state.target.z,
        dist_mm,
    );
}

// ── chart panel ───────────────────────────────────────────────────────────────

fn setup_egui(mut contexts: EguiContexts) {
    let Some(ctx) = contexts.try_ctx_mut() else { return };
    let mut visuals = egui::Visuals::dark();
    visuals.selection.bg_fill = egui::Color32::from_rgb(50, 100, 200);
    visuals.widgets.active.bg_fill = egui::Color32::from_rgb(40, 80, 160);
    ctx.set_visuals(visuals);
}

fn time_plot_points(times: &VecDeque<f32>, vals: &VecDeque<f32>) -> PlotPoints {
    times.iter().zip(vals.iter())
        .map(|(&t, &v)| [t as f64, v as f64])
        .collect()
}

fn draw_charts(
    mut contexts: EguiContexts,
    history: Res<PlotHistory>,
    mut state: ResMut<SimState>,
    mut ui_state: ResMut<UiState>,
) {
    let Some(ctx) = contexts.try_ctx_mut() else { return };

    egui::SidePanel::right("charts_panel")
        .exact_width(380.0)
        .resizable(false)
        .show(ctx, |ui| {
            ui.add_space(6.0);
            ui.heading("Joint Telemetry");
            ui.separator();

            // Panel-level tab bar: Telemetry vs Tuning
            ui.horizontal(|ui| {
                if ui.selectable_label(!ui_state.tuning_tab, "Telemetry").clicked() {
                    ui_state.tuning_tab = false;
                }
                if ui.selectable_label(ui_state.tuning_tab, "Tuning").clicked() {
                    ui_state.tuning_tab = true;
                }
            });
            ui.separator();

            // Tab bar for joint selection
            ui.horizontal(|ui| {
                for (i, label) in ["Pan", "Shoulder", "Elbow"].iter().enumerate() {
                    if ui.selectable_label(ui_state.selected_joint == i, *label).clicked() {
                        ui_state.selected_joint = i;
                    }
                }
            });
            ui.separator();

            let ji = ui_state.selected_joint;

            egui::ScrollArea::vertical().show(ui, |ui| {
                if ui_state.tuning_tab {
                    // ── Tuning view ───────────────────────────────────────────
                    let joint = match ji {
                        0 => &mut state.arm.pan,
                        1 => &mut state.arm.shoulder,
                        _ => &mut state.arm.elbow,
                    };
                    let (mut kp, mut ki, mut kd, mut kf) = joint.get_gains();

                    ui.group(|ui| {
                        ui.label("PID Gains");
                        let mut changed = false;

                        ui.horizontal(|ui| {
                            ui.label("kp:");
                            changed |= ui.add(
                                egui::Slider::new(&mut kp, 0.0..=50.0).step_by(0.1),
                            ).changed();
                        });
                        ui.horizontal(|ui| {
                            ui.label("ki:");
                            changed |= ui.add(
                                egui::Slider::new(&mut ki, 0.0..=10.0).step_by(0.01),
                            ).changed();
                        });
                        ui.horizontal(|ui| {
                            ui.label("kd:");
                            changed |= ui.add(
                                egui::Slider::new(&mut kd, 0.0..=10.0).step_by(0.01),
                            ).changed();
                        });
                        ui.horizontal(|ui| {
                            ui.label("kf:");
                            changed |= ui.add(
                                egui::Slider::new(&mut kf, 0.0..=5.0).step_by(0.01),
                            ).changed();
                        });

                        if changed {
                            joint.set_gains(kp, ki, kd, kf);
                        }

                        ui.add_space(4.0);
                        if ui.button("Reset Integrator").clicked() {
                            joint.reset_integrator();
                        }
                    });

                    ui.add_space(4.0);
                    ui.label(egui::RichText::new("Changes apply immediately").weak().small());
                } else {
                    // ── Telemetry view ────────────────────────────────────────
                    // Angle vs Setpoint
                    ui.label("Angle vs Setpoint (rad)");
                    Plot::new(format!("angle_{ji}"))
                        .height(110.0)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .x_axis_label("t (s)")
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                Line::new(time_plot_points(&history.times, &history.angle[ji]))
                                    .color(egui::Color32::from_rgb(100, 180, 255))
                                    .name("angle"),
                            );
                            plot_ui.line(
                                Line::new(time_plot_points(&history.times, &history.sp[ji]))
                                    .color(egui::Color32::from_rgb(255, 200, 80))
                                    .name("setpoint"),
                            );
                        });

                    // Error
                    ui.label("Error (rad)");
                    Plot::new(format!("err_{ji}"))
                        .height(80.0)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .x_axis_label("t (s)")
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                Line::new(time_plot_points(&history.times, &history.err[ji]))
                                    .color(egui::Color32::from_rgb(255, 100, 100))
                                    .name("error"),
                            );
                        });

                    // Velocity
                    ui.label("Velocity (rad/s)");
                    Plot::new(format!("vel_{ji}"))
                        .height(80.0)
                        .allow_zoom(false)
                        .allow_scroll(false)
                        .x_axis_label("t (s)")
                        .show(ui, |plot_ui| {
                            plot_ui.line(
                                Line::new(time_plot_points(&history.times, &history.vel[ji]))
                                    .color(egui::Color32::from_rgb(100, 220, 130))
                                    .name("velocity"),
                            );
                        });
                }

                ui.separator();

                // Target position controls
                ui.label("Target position (m)");
                let mut tgt = [
                    state.target.x as f32,
                    state.target.y as f32,
                    state.target.z as f32,
                ];
                let mut changed = false;
                ui.horizontal(|ui| {
                    ui.label("X:");
                    changed |= ui.add(egui::DragValue::new(&mut tgt[0]).speed(0.01).range(0.0..=2.0)).changed();
                    ui.label("Y:");
                    changed |= ui.add(egui::DragValue::new(&mut tgt[1]).speed(0.01).range(0.0..=2.0)).changed();
                    ui.label("Z:");
                    changed |= ui.add(egui::DragValue::new(&mut tgt[2]).speed(0.01).range(-1.0..=1.0)).changed();
                });
                if changed {
                    let new_t = bevy::math::DVec3::new(tgt[0] as f64, tgt[1] as f64, tgt[2] as f64);
                    set_target(&mut state, new_t);
                }

                ui.separator();
                ui.label(egui::RichText::new("WASD/QE: move  Space: pause  R: reset  1-9: choreo  C: cancel").weak().small());
            });
        });
}

// ── socket systems ────────────────────────────────────────────────────────────

fn handle_socket_cmds(mut state: ResMut<SimState>, socket: Res<SocketState>) {
    let Ok(rx) = socket.cmd_rx.lock() else { return };
    while let Ok(cmd) = rx.try_recv() {
        match cmd {
            SocketCmd::SetTarget(x, y, z) => set_target(&mut state, DVec3::new(x, y, z)),
            SocketCmd::Pause              => state.paused = true,
            SocketCmd::Resume             => state.paused = false,
            SocketCmd::Reset              => {
                state.arm.reset();
                state.ee_trail.clear();
                state.elapsed_s = 0.0;
                state.target_reached = false;
                let t = state.target;
                state.arm.set_target(t);
                state.arm.start_cartesian_move(t, CARTESIAN_SPEED);
            }
        }
    }
}

fn push_snapshot(state: Res<SimState>, socket: Res<SocketState>) {
    let Ok(mut snap) = socket.snapshot.lock() else { return };
    let ee  = state.arm.ee_pos();
    let tgt = state.target;
    *snap = SimSnapshot {
        elapsed_s:      state.elapsed_s,
        paused:         state.paused,
        pan_angle_deg:  state.arm.pan.angle_rad().to_degrees(),
        pan_sp_deg:     state.arm.pan.get_setpoint_rad().to_degrees(),
        pan_vel_rads:   state.arm.pan.velocity_rad_s(),
        sho_angle_deg:  state.arm.shoulder.angle_rad().to_degrees(),
        sho_sp_deg:     state.arm.shoulder.get_setpoint_rad().to_degrees(),
        sho_vel_rads:   state.arm.shoulder.velocity_rad_s(),
        elb_angle_deg:  state.arm.elbow.angle_rad().to_degrees(),
        elb_sp_deg:     state.arm.elbow.get_setpoint_rad().to_degrees(),
        elb_vel_rads:   state.arm.elbow.velocity_rad_s(),
        ee:             [ee.x, ee.y, ee.z],
        target:         [tgt.x, tgt.y, tgt.z],
        dist_mm:        (ee - tgt).length() * 1000.0,
        target_reached: state.target_reached,
        ik_valid:       state.ik_valid,
    };
}
