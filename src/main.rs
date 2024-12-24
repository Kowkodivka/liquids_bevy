use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_pancam::{PanCam, PanCamPlugin};

const MASS: f32 = 1.0;
const SMOOTHING_RADIUS: f32 = 10.0;
const TARGET_DENSITY: f32 = 5.0;
const PRESSURE_MULTIPLIER: f32 = 1.0;

#[derive(Component)]
struct Velocity(Vec3);

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(PanCamPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, simulation_step)
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

fn simulation_step(time: Res<Time>) {
    let delta_time = time.delta_secs();
}
