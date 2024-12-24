use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_pancam::{PanCam, PanCamPlugin};

const MASS: f32 = 1.0;
const SMOOTHING_RADIUS: f32 = 1.5;
const TARGET_DENSITY: f32 = 1000.0;
const PRESSURE_MULTIPLIER: f32 = 2000.0;
const WIDTH: f32 = 100.0;
const HEIGHT: f32 = 100.0;
const GRAVITY: f32 = 9.8;

#[derive(Component)]
struct Velocity(Vec3);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(PanCamPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (velocity_system, update_system, boundary_collision_system),
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
    let spacing = 1.0;

    for x in 0..square_size {
        for y in 0..square_size {
            let position = Vec3::new(
                x as f32 * spacing - (square_size as f32 * spacing / 2.0) + spacing / 2.0,
                y as f32 * spacing - (square_size as f32 * spacing / 2.0) + spacing / 2.0,
                0.0,
            );

            commands.spawn((
                Mesh2d(meshes.add(Circle::new(1.0))),
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

fn calculate_pressure_force(point: Vec3, transforms: &[&Transform]) -> Vec3 {
    let density = calculate_density(point, transforms);
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

fn velocity_system(
    time: Res<Time>,
    transforms_query: Query<&Transform>,
    mut velocities_query: Query<(&Transform, &mut Velocity)>,
) {
    let delta_time = time.delta_secs();

    let all_transforms: Vec<_> = transforms_query.iter().collect();

    for (transform, mut velocity) in velocities_query.iter_mut() {
        let position = transform.translation;
        let pressure_force = calculate_pressure_force(position, &all_transforms);

        velocity.0 += pressure_force * delta_time;
        velocity.0 += Vec3::new(0.0, -1.0, 0.0) * GRAVITY * delta_time
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
            velocity.0.x *= -1.0;
            transform.translation.x = position.x.clamp(-WIDTH / 2.0, WIDTH / 2.0);
        }

        if position.y <= -HEIGHT / 2.0 || position.y >= HEIGHT / 2.0 {
            velocity.0.y *= -1.0;
            transform.translation.y = position.y.clamp(-HEIGHT / 2.0, HEIGHT / 2.0);
        }
    }
}
