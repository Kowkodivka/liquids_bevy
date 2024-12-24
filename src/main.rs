use std::f32::consts::PI;

use bevy::{prelude::*, utils::HashMap};
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_pancam::{PanCam, PanCamPlugin};

const RADIUS: f32 = 1.0;
const MASS: f32 = 1.0;
const SMOOTHING_RADIUS: f32 = 2.0;
const TARGET_DENSITY: f32 = 0.75;
const PRESSURE_MULTIPLIER: f32 = 2.0;
const WIDTH: f32 = 100.0;
const HEIGHT: f32 = 100.0;
const GRAVITY: f32 = 10.0;
const DAMPING_FACTOR: f32 = 0.995;

#[derive(Component)]
struct Velocity(Vec3);

#[derive(Resource)]
struct DensityCache {
    densities: HashMap<Entity, f32>,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(PanCamPlugin::default())
        .add_systems(Startup, setup)
        .insert_resource(DensityCache {
            densities: HashMap::new(),
        })
        .add_systems(
            Update,
            (
                cache_density_system,
                velocity_system,
                update_system,
                collision_system,
                boundary_collision_system,
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    commands.spawn((Camera2d, PanCam::default()));

    let square_size = 10;
    let spacing = 10.0;

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

fn smoothing_kernel(radius: f32, distance: f32) -> f32 {
    let volume = PI * radius.powi(8) / 4.0;
    let value = (radius * radius - distance * distance).max(0.0);
    value * value * value / volume
}

fn smoothing_kernel_derivative(radius: f32, distance: f32) -> f32 {
    if distance > radius {
        0.0
    } else {
        let f = radius * radius - distance * distance;
        let scale = -24.0 / (PI * radius.powi(8));

        scale * distance * f * f
    }
}

fn calculate_density(point: Vec3, transforms: &[&Transform]) -> f32 {
    transforms.iter().fold(0.0, |density, &transform| {
        let distance = transform.translation.distance(point);
        density + MASS * smoothing_kernel(SMOOTHING_RADIUS, distance)
    })
}

fn density_to_pressure(density: f32) -> f32 {
    (density - TARGET_DENSITY) * PRESSURE_MULTIPLIER
}

fn calculate_pressure_force(point: Vec3, transforms: &[&Transform], density: f32) -> Vec3 {
    transforms
        .iter()
        .fold(Vec3::ZERO, |mut pressure_force, &transform| {
            let distance = transform.translation.distance(point);

            if distance == 0.0 {
                return pressure_force;
            }

            let direction = (transform.translation - point) / distance;
            let slope = smoothing_kernel_derivative(SMOOTHING_RADIUS, distance);

            pressure_force += -density_to_pressure(density) * direction * slope * MASS / density;
            pressure_force
        })
}

fn cache_density_system(
    mut density_cache: ResMut<DensityCache>,
    transforms_query: Query<(Entity, &Transform)>,
) {
    let all_transforms: Vec<_> = transforms_query.iter().collect();

    // Очищаем кеш
    density_cache.densities.clear();

    for (entity, transform) in &all_transforms {
        let density = calculate_density(
            transform.translation,
            &all_transforms.iter().map(|(_, t)| *t).collect::<Vec<_>>(),
        );
        density_cache.densities.insert(*entity, density);
    }

    debug!(
        "Cached densities updated: {} entries",
        density_cache.densities.len()
    );
}

fn velocity_system(
    time: Res<Time>,
    transforms_query: Query<&Transform>,
    mut velocities_query: Query<(Entity, &Transform, &mut Velocity)>,
    density_cache: Res<DensityCache>,
) {
    let delta_time = time.delta_secs();

    let all_transforms: Vec<_> = transforms_query.iter().collect();

    for (entity, transform, mut velocity) in velocities_query.iter_mut() {
        let position = transform.translation;

        if let Some(&density) = density_cache.densities.get(&entity) {
            let pressure_force = calculate_pressure_force(position, &all_transforms, density);
            let pressure_acceleration = pressure_force / density;

            velocity.0 += pressure_acceleration * delta_time;
            velocity.0 += Vec3::new(0.0, -1.0, 0.0) * GRAVITY * delta_time;

            // Apply damping
            velocity.0 *= DAMPING_FACTOR;
        } else {
            error!("Missing density for entity {:?}", entity);
        }
    }
}

fn update_system(time: Res<Time>, mut query: Query<(&mut Transform, &Velocity)>) {
    let delta_time = time.delta_secs();

    for (mut transform, velocity) in query.iter_mut() {
        transform.translation += velocity.0 * delta_time;
    }
}

fn boundary_collision_system(mut query: Query<(&mut Transform, &mut Velocity)>) {
    for (mut transform, mut velocity) in query.iter_mut() {
        let position = transform.translation;

        if position.x <= -WIDTH / 2.0 || position.x >= WIDTH / 2.0 {
            velocity.0.x *= -1.0 * DAMPING_FACTOR;
            transform.translation.x = position.x.clamp(-WIDTH / 2.0, WIDTH / 2.0);
        }

        if position.y <= -HEIGHT / 2.0 || position.y >= HEIGHT / 2.0 {
            velocity.0.y *= -1.0 * DAMPING_FACTOR;
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

    // Сначала определяем коллизии и вычисляем импульсы
    for i in 0..transforms_and_positions.len() {
        for j in (i + 1)..transforms_and_positions.len() {
            let (entity_a, position_a) = transforms_and_positions[i];
            let (entity_b, position_b) = transforms_and_positions[j];

            let distance = position_a.distance(position_b);

            if distance < 2.0 * RADIUS {
                let normal = (position_b - position_a).normalize();

                // Проверяем наличие компонентов Velocity у обеих сущностей
                if let (Ok(velocity_a), Ok(velocity_b)) = (
                    velocities_query.get(entity_a),
                    velocities_query.get(entity_b),
                ) {
                    let relative_velocity = velocity_b.1 .0 - velocity_a.1 .0;
                    let velocity_along_normal = relative_velocity.dot(normal);

                    if velocity_along_normal > 0.0 {
                        continue;
                    }

                    let impulse = (2.0 * velocity_along_normal) / (2.0 * MASS);

                    let impulse_a = impulse * normal * -1.0;
                    let impulse_b = impulse * normal;

                    collision_impulses.push((entity_a, impulse_a));
                    collision_impulses.push((entity_b, impulse_b));
                }
            }
        }
    }

    // Затем применяем изменения
    for (entity, impulse) in collision_impulses {
        if let Ok(mut velocity) = velocities_query.get_mut(entity) {
            velocity.1 .0 += impulse;
        }
    }
}
