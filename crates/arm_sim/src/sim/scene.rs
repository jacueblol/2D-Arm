use bevy::light::{AmbientLight, DirectionalLight, GlobalAmbientLight};
use bevy::prelude::*;

use super::resources::{SimState, VisualEntities};

const LINK_RADIUS: f32 = 0.04;
const JOINT_RADIUS: f32 = 0.06;
const EE_RADIUS: f32 = 0.09;

#[derive(Component)]
pub struct StatusText;

/// Place a Y-axis cylinder between two world points (translation + rotation
/// only — the cylinder's length must already be baked into its mesh via
/// `half_height`).
fn link_transform(start: Vec3, end: Vec3) -> Transform {
    let dir = end - start;
    let mid = (start + end) * 0.5;
    let rot = if dir.normalize_or_zero().dot(Vec3::Y).abs() > 0.9999 {
        if dir.y >= 0.0 {
            Quat::IDENTITY
        } else {
            Quat::from_rotation_x(std::f32::consts::PI)
        }
    } else {
        Quat::from_rotation_arc(Vec3::Y, dir.normalize())
    };
    Transform {
        translation: mid,
        rotation: rot,
        scale: Vec3::ONE,
    }
}

pub fn setup(
    mut commands: Commands,
    state: Res<SimState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Fixed camera — orbit control lands in M7.
    commands.spawn((
        Camera3d::default(),
        AmbientLight {
            color: Color::WHITE,
            brightness: 300.0,
            ..default()
        },
        Transform::from_xyz(3.2, 2.6, 3.2).looking_at(Vec3::new(0.6, 0.4, 0.2), Vec3::Y),
    ));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 80.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 15_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.9, 0.5, 0.0)),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d {
            normal: Dir3::Y,
            half_size: Vec2::splat(6.0),
        })),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.10, 0.10, 0.10),
            perceptual_roughness: 1.0,
            ..default()
        })),
    ));

    // Base pedestal
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.09, 0.10))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.35, 0.35),
            ..default()
        })),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));

    // Shoulder joint (static — the pan/shoulder pivot itself doesn't move
    // visually since it's at the origin).
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(JOINT_RADIUS * 1.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.8, 0.8),
            ..default()
        })),
    ));

    let [_, sho, elb, ee_p] = state.arm.forward_kinematics();
    let (sho, elb, ee_p) = (sho.as_vec3(), elb.as_vec3(), ee_p.as_vec3());

    let l1 = state.arm.shoulder.length as f32;
    let l2 = state.arm.elbow.length as f32;

    let link1 = commands
        .spawn((
            Mesh3d(meshes.add(Cylinder::new(LINK_RADIUS, l1))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.25, 0.55, 1.0),
                ..default()
            })),
            link_transform(sho, elb),
        ))
        .id();

    let elbow = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(JOINT_RADIUS))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.85, 0.15),
                ..default()
            })),
            Transform::from_translation(elb),
        ))
        .id();

    let link2 = commands
        .spawn((
            Mesh3d(meshes.add(Cylinder::new(LINK_RADIUS, l2))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.25, 0.85, 0.45),
                ..default()
            })),
            link_transform(elb, ee_p),
        ))
        .id();

    let ee = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(EE_RADIUS))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.45, 0.1),
                emissive: LinearRgba::new(0.4, 0.12, 0.0, 1.0),
                ..default()
            })),
            Transform::from_translation(ee_p),
        ))
        .id();

    let target_sphere = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(EE_RADIUS * 1.25))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.95, 0.15, 0.15, 0.5),
                alpha_mode: AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(state.target.as_vec3()),
        ))
        .id();

    commands.insert_resource(VisualEntities {
        link1,
        link2,
        elbow,
        ee,
        target_sphere,
    });

    // Minimal status overlay — grows into the full telemetry/tuning panel in M9.
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(12.0),
                left: Val::Px(12.0),
                padding: UiRect::all(Val::Px(10.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.75)),
        ))
        .with_children(|p| {
            p.spawn((
                StatusText,
                Text::new("Initializing..."),
                TextFont {
                    font_size: bevy::text::FontSize::Px(13.0),
                    ..default()
                },
                TextColor(Color::srgb(0.9, 0.9, 0.9)),
            ));
        });
}

pub fn update_visuals(
    state: Res<SimState>,
    entities: Option<Res<VisualEntities>>,
    mut transforms: Query<&mut Transform>,
) {
    let Some(e) = entities else { return };

    let [_, sho, elb, ee_p] = state.arm.forward_kinematics();
    let (sho, elb, ee_p) = (sho.as_vec3(), elb.as_vec3(), ee_p.as_vec3());

    if let Ok([mut tl1, mut tl2, mut te, mut tee, mut ttgt]) =
        transforms.get_many_mut([e.link1, e.link2, e.elbow, e.ee, e.target_sphere])
    {
        *tl1 = link_transform(sho, elb);
        *tl2 = link_transform(elb, ee_p);
        te.translation = elb;
        tee.translation = ee_p;
        ttgt.translation = state.target.as_vec3();
    }
}

pub fn update_status(state: Res<SimState>, mut query: Query<&mut Text, With<StatusText>>) {
    let Ok(mut text) = query.single_mut() else {
        return;
    };

    let pan_deg = state.arm.pan.angle_rad().to_degrees();
    let sho_deg = state.arm.shoulder.angle_rad().to_degrees();
    let elb_deg = state.arm.elbow.angle_rad().to_degrees();
    let ee = state.arm.ee_pos();

    text.0 = format!(
        "pan {pan_deg:6.1}°  shoulder {sho_deg:6.1}°  elbow {elb_deg:6.1}°\n\
         end-effector ({:.3}, {:.3}, {:.3})  target ({:.3}, {:.3}, {:.3})",
        ee.x, ee.y, ee.z, state.target.x, state.target.y, state.target.z,
    );
}
