use std::f32::consts::PI;

use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_pancam::{PanCam, PanCamPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(WorldInspectorPlugin::new())
        .add_plugins(PanCamPlugin::default())
        .add_systems(Startup, setup)
        .add_systems(Update, handle_density_info)
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
            ));
        }
    }
}

fn smoothing_kernel(radius: f32, distance: f32) -> f32 {
    let volume = PI * radius.powi(8) / 4.0;
    let value = (radius * radius - distance * distance).max(0.0);
    value * value * value / volume
}

fn calculate_density(point: Vec3, query: Query<&Transform>) -> f32 {
    let mut density = 0.0;

    let mass = 1.0;
    let smoothing_radius = 10.0;

    for position in query.iter() {
        let distance = position.translation.distance(point);
        let influence = smoothing_kernel(smoothing_radius, distance);

        density += mass * influence
    }

    density
}

fn handle_density_info(
    keys: Res<ButtonInput<KeyCode>>,
    windows: Query<&Window>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    transforms_query: Query<&Transform>,
) {
    if keys.just_released(KeyCode::KeyE) {
        if let Some(world_position) = get_mouse_world_position(camera_query, windows) {
            let density = calculate_density(world_position.extend(0.0), transforms_query);
            println!("Density at {:?}: {}", world_position, density);
        }
    }
}

fn get_mouse_world_position(
    camera_query: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
) -> Option<Vec2> {
    let (camera, camera_transform) = camera_query.get_single().ok()?;
    let window = windows.get_single().ok()?;

    let cursor_position = window.cursor_position()?;
    Some(
        camera
            .viewport_to_world_2d(camera_transform, cursor_position)
            .unwrap(),
    )
}
