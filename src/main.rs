use std::f32::consts::PI;

use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
    prelude::*,
    utils::HashMap,
};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_pancam::{PanCam, PanCamPlugin};

const RADIUS: f32 = 1.0;
const MASS: f32 = 50.0;
const SMOOTHING_RADIUS: f32 = 5.0;
const TARGET_DENSITY: f32 = 5000.0;
const PRESSURE_MULTIPLIER: f32 = 2.0;
const WIDTH: f32 = 100.0;
const HEIGHT: f32 = 100.0;
const GRAVITY: f32 = 10.0;
const DAMPING_FACTOR: f32 = 0.99;
const E: f32 = 0.01;

#[derive(Component)]
struct Velocity(Vec3);

#[derive(Resource)]
struct DensityCache {
    densities: HashMap<Entity, f32>,
}

#[derive(Resource)]
struct DragState {
    selected_entity: Option<Entity>,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(PanCamPlugin::default())
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        .add_plugins(LogDiagnosticsPlugin::default())
        .add_systems(Startup, setup)
        .insert_resource(DensityCache {
            densities: HashMap::new(),
        })
        .insert_resource(DragState {
            selected_entity: None,
        })
        .add_systems(
            Update,
            (
                cache_density_system,
                velocity_system,
                update_system,
                collision_system,
                boundary_collision_system,
                update_colors_system,
                mouse_input_system,
                time_control_system,
                mouse_object_spawn_system,
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn((
        Camera2d,
        PanCam {
            grab_buttons: vec![],
            ..default()
        },
    ));

    let square_size = 10;
    let spacing = SMOOTHING_RADIUS;

    for x in 0..square_size {
        for y in 0..square_size {
            let position = Vec3::new(
                x as f32 * spacing - (square_size as f32 * spacing / 2.0) + spacing / 2.0,
                y as f32 * spacing - (square_size as f32 * spacing / 2.0) + spacing / 2.0,
                0.0,
            );

            commands.spawn((
                Mesh2d(meshes.add(Circle::new(RADIUS))),
                MeshMaterial2d(materials.add(Color::hsl(0.5, 0.95, 0.7))),
                Transform::from_translation(position),
                Velocity(Vec3::ZERO),
            ));
        }
    }
}

fn hash_position(position: Vec3, cell_size: f32) -> (i32, i32) {
    (
        (position.x / cell_size).floor() as i32,
        (position.y / cell_size).floor() as i32,
    )
}

fn smoothing_kernel(radius: f32, distance: f32) -> f32 {
    if distance >= radius {
        0.0
    } else {
        let volume = (PI * radius.powi(4)) / 6.0;
        (radius - distance).powi(2) / volume
    }
}

fn smoothing_kernel_derivative(radius: f32, distance: f32) -> f32 {
    if distance > radius {
        0.0
    } else {
        let scale = 12.0 / (radius.powi(4) * PI);

        (distance - radius) * scale
    }
}

// fn calculate_density(point: Vec3, transforms: &[&Transform]) -> f32 {
//     transforms.iter().fold(0.0, |density, &transform| {
//         let distance = transform.translation.distance(point);
//         density + MASS * smoothing_kernel(SMOOTHING_RADIUS, distance)
//     })
// }

fn density_to_pressure(density: f32) -> f32 {
    (density - TARGET_DENSITY) * PRESSURE_MULTIPLIER
}

fn calculate_pressure_force(
    point: Vec3,
    point_cell: (i32, i32),
    spatial_hash: &HashMap<(i32, i32), Vec<(Entity, Vec3)>>,
    density: f32,
) -> Vec3 {
    let mut pressure_force = Vec3::ZERO;

    for dx in -1..=1 {
        for dy in -1..=1 {
            if let Some(neighbors) = spatial_hash.get(&(point_cell.0 + dx, point_cell.1 + dy)) {
                for &(_, neighbor_position) in neighbors {
                    let distance = neighbor_position.distance(point);

                    if distance <= f32::EPSILON || distance >= SMOOTHING_RADIUS || distance.is_nan()
                    {
                        continue;
                    }

                    let direction = (neighbor_position - point) / distance;
                    let slope = smoothing_kernel_derivative(SMOOTHING_RADIUS, distance);

                    pressure_force +=
                        -density_to_pressure(density) * direction * slope * MASS / density;
                }
            }
        }
    }

    pressure_force
}

fn cache_density_system(
    mut density_cache: ResMut<DensityCache>,
    transforms_query: Query<(Entity, &Transform)>,
) {
    let cell_size = SMOOTHING_RADIUS.powi(2);
    let mut spatial_hash: HashMap<(i32, i32), Vec<(Entity, Vec3)>> = HashMap::new();

    for (entity, transform) in transforms_query.iter() {
        let position = transform.translation;
        let cell = hash_position(position, cell_size);

        spatial_hash
            .entry(cell)
            .or_insert_with(Vec::new)
            .push((entity, position));
    }

    density_cache.densities.clear();

    for (entity, transform) in transforms_query.iter() {
        let position = transform.translation;
        let cell = hash_position(position, cell_size);

        let mut density = 0.0;

        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(neighbors) = spatial_hash.get(&(cell.0 + dx, cell.1 + dy)) {
                    for &(_, neighbor_position) in neighbors {
                        let distance = neighbor_position.distance(position);
                        if distance < SMOOTHING_RADIUS {
                            density += MASS * smoothing_kernel(SMOOTHING_RADIUS, distance);
                        }
                    }
                }
            }
        }

        density_cache.densities.insert(entity, density);
    }
}

fn velocity_system(
    time: Res<Time>,
    transforms_query: Query<(Entity, &Transform)>,
    mut velocities_query: Query<(Entity, &Transform, &mut Velocity)>,
    density_cache: Res<DensityCache>,
) {
    let delta_time = time.delta_secs().max(1e-6);
    let cell_size = SMOOTHING_RADIUS;

    let mut spatial_hash: HashMap<(i32, i32), Vec<(Entity, Vec3)>> = HashMap::new();
    for (entity, transform) in transforms_query.iter() {
        let position = transform.translation;
        let cell = hash_position(position, cell_size);

        spatial_hash
            .entry(cell)
            .or_insert_with(Vec::new)
            .push((entity, position));
    }

    for (entity, transform, mut velocity) in velocities_query.iter_mut() {
        let position = transform.translation;
        let cell = hash_position(position, cell_size);

        if let Some(&density) = density_cache.densities.get(&entity) {
            let density_safe = density.max(1e-6);
            let pressure_force =
                calculate_pressure_force(position, cell, &spatial_hash, density_safe);
            let pressure_acceleration = pressure_force / density_safe;

            if pressure_acceleration.is_finite() {
                velocity.0 += pressure_acceleration * delta_time;
            }

            velocity.0 += Vec3::new(0.0, -1.0, 0.0) * GRAVITY * delta_time;
            velocity.0 *= DAMPING_FACTOR;

            if !velocity.0.is_finite() {
                velocity.0 = Vec3::ZERO;
            }
        }
    }
}

fn update_system(time: Res<Time>, mut query: Query<(&mut Transform, &Velocity)>) {
    let delta_time = time.delta_secs().max(1e-6);

    for (mut transform, velocity) in query.iter_mut() {
        if velocity.0.is_finite() {
            transform.translation += velocity.0 * delta_time;
        }
    }
}

fn boundary_collision_system(mut query: Query<(&mut Transform, &mut Velocity)>) {
    for (mut transform, mut velocity) in query.iter_mut() {
        let position = transform.translation;

        if position.x < -WIDTH / 2.0 || position.x > WIDTH / 2.0 {
            velocity.0.x *= -DAMPING_FACTOR;
            transform.translation.x = position.x.clamp(-WIDTH / 2.0, WIDTH / 2.0);
        }

        if position.y < -HEIGHT / 2.0 || position.y > HEIGHT / 2.0 {
            velocity.0.y *= -DAMPING_FACTOR;
            transform.translation.y = position.y.clamp(-HEIGHT / 2.0, HEIGHT / 2.0);
        }
    }
}

fn collision_system(
    transforms_query: Query<(Entity, &Transform)>,
    mut velocities_query: Query<(Entity, &mut Velocity)>,
) {
    let mut collision_impulses: Vec<(Entity, Vec3)> = vec![];

    let transforms_and_positions: Vec<_> = transforms_query
        .iter()
        .map(|(entity, transform)| (entity, transform.translation))
        .collect();

    for i in 0..transforms_and_positions.len() {
        for j in (i + 1)..transforms_and_positions.len() {
            let (entity_a, position_a) = transforms_and_positions[i];
            let (entity_b, position_b) = transforms_and_positions[j];

            let distance = position_a.distance(position_b);

            if distance < 2.0 * RADIUS {
                let normal = (position_b - position_a).normalize();

                if let (Ok(velocity_a), Ok(velocity_b)) = (
                    velocities_query.get(entity_a),
                    velocities_query.get(entity_b),
                ) {
                    let relative_velocity = velocity_b.1 .0 - velocity_a.1 .0;
                    let velocity_along_normal = relative_velocity.dot(normal);

                    if velocity_along_normal > 0.0 {
                        continue;
                    }

                    let impulse = -(1.0 + E) * velocity_along_normal * MASS;

                    let impulse_a = impulse * normal * -1.0;
                    let impulse_b = impulse * normal;

                    collision_impulses.push((entity_a, impulse_a));
                    collision_impulses.push((entity_b, impulse_b));
                }
            }
        }
    }

    for (entity, impulse) in collision_impulses {
        if let Ok(mut velocity) = velocities_query.get_mut(entity) {
            velocity.1 .0 += impulse / MASS * DAMPING_FACTOR;
        }
    }
}

fn update_colors_system(
    density_cache: Res<DensityCache>,
    query: Query<(Entity, &mut MeshMaterial2d<ColorMaterial>)>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    for (entity, material_handle) in query.iter() {
        if let (Some(material), Some(density)) = (
            materials.get_mut(material_handle),
            density_cache.densities.get(&entity),
        ) {
            let hue = (density * 360.0) % 360.0;
            material.color = Color::hsl(hue, 0.95, 0.7);
        }
    }
}

fn mouse_input_system(
    mouse_input: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut drag_state: ResMut<DragState>,
    mut query: Query<(Entity, &mut Transform, &mut Velocity)>,
) {
    let window = windows.single();
    let (camera, camera_transform) = camera_query.single();
    static mut LAST_MOUSE_POSITION: Option<Vec2> = None;

    if let Some(cursor_position) = window.cursor_position() {
        unsafe {
            if mouse_input.just_pressed(MouseButton::Left) {
                for (entity, transform, _) in query.iter_mut() {
                    let position = camera.viewport_to_world_2d(camera_transform, cursor_position);
                    if let Ok(position) = position {
                        if transform.translation.truncate().distance(position) <= RADIUS {
                            drag_state.selected_entity = Some(entity);
                            LAST_MOUSE_POSITION = Some(cursor_position);
                            break;
                        }
                    }
                }
            } else if mouse_input.just_released(MouseButton::Left) {
                if let Some(entity) = drag_state.selected_entity {
                    if let Some(last_position) = LAST_MOUSE_POSITION {
                        if let Ok((_, _, mut velocity)) = query.get_mut(entity) {
                            let current_position = cursor_position;
                            let delta = current_position - last_position;
                            velocity.0 = Vec3::new(delta.x, delta.y, 0.0) * 10.0;
                        }
                    }
                }
                drag_state.selected_entity = None;
                LAST_MOUSE_POSITION = None;
            } else if let Some(entity) = drag_state.selected_entity {
                if let Ok((_, mut transform, _)) = query.get_mut(entity) {
                    if let Ok(world_position) =
                        camera.viewport_to_world_2d(camera_transform, cursor_position)
                    {
                        transform.translation.x = world_position.x;
                        transform.translation.y = world_position.y;
                        LAST_MOUSE_POSITION = Some(cursor_position);
                    }
                }
            }
        }
    }
}

fn time_control_system(input: Res<ButtonInput<KeyCode>>, mut time: ResMut<Time<Virtual>>) {
    if input.just_pressed(KeyCode::Space) {
        if time.is_paused() {
            time.unpause();
        } else {
            time.pause();
        }
    }
}

fn mouse_object_spawn_system(
    input: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let (camera, camera_transform) = camera_query.single();

    if let Some(cursor_position) = windows.single().cursor_position() {
        if input.just_pressed(KeyCode::KeyF) {
            if let Ok(world_position) =
                camera.viewport_to_world_2d(camera_transform, cursor_position)
            {
                commands.spawn((
                    Mesh2d(meshes.add(Circle::new(RADIUS))),
                    MeshMaterial2d(materials.add(Color::hsl(0.5, 0.95, 0.7))),
                    Transform::from_translation(Vec3::new(world_position.x, world_position.y, 0.0)),
                    Velocity(Vec3::ZERO),
                ));
            }
        }
    }
}
