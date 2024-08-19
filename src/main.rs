use bevy::{diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin}, prelude::*, sprite::MaterialMesh2dBundle};
use bevy_egui::{egui, EguiContexts, EguiPlugin};
use rand::Rng;
use rayon::prelude::*;
use std::collections::HashMap;

const WINDOW_WIDTH: f32 = 1920.0;
const WINDOW_HEIGHT: f32 = 1080.0;
const FIXED_TIMESTEP: f32 = 1.0 / 60.0;
const BOID_COUNT: usize = 5000;
const ZOOM_FACTOR: f32 = 2.5;

#[derive(Component)]
struct Boid {
    velocity: Vec2,
}

#[derive(Resource)]
struct SimulationParams {
    separation_radius: f32,
    alignment_radius: f32,
    cohesion_radius: f32,
    separation_factor: f32,
    alignment_factor: f32,
    cohesion_factor: f32,
    linear_damping: f32,
    max_speed: f32,
    max_force: f32,
}

impl Default for SimulationParams {
    fn default() -> Self {
        SimulationParams {
            separation_radius: 15.0,
            alignment_radius: 55.0,
            cohesion_radius: 90.0,
            separation_factor: 1.0,
            alignment_factor: 0.5,
            cohesion_factor: 0.1,
            linear_damping: 0.05,
            max_speed: 150.0,
            max_force: 300.0,
        }
    }
}

#[derive(Resource, Default)]
struct PhysicsTime {
    accumulated_time: f32,
}

#[derive(Resource)]
struct SpatialHashGrid {
    cell_size: f32,
    grid: HashMap<(i32, i32), Vec<(Entity, Vec3, Vec2)>>,
}

impl SpatialHashGrid {
    fn new(cell_size: f32, zoom_factor: f32) -> Self {
        SpatialHashGrid {
            cell_size: cell_size * zoom_factor,
            grid: HashMap::new(),
        }
    }

    fn clear(&mut self) {
        self.grid.clear();
    }

    fn insert(&mut self, entity: Entity, position: Vec3, velocity: Vec2) {
        let cell = self.get_cell(position.truncate());
        self.grid
            .entry(cell)
            .or_insert_with(Vec::new)
            .push((entity, position, velocity));
    }

    fn get_cell(&self, position: Vec2) -> (i32, i32) {
        (
            (position.x / self.cell_size).floor() as i32,
            (position.y / self.cell_size).floor() as i32,
        )
    }

    fn get_nearby_entities(&self, position: Vec2, radius: f32) -> Vec<(Entity, Vec3, Vec2)> {
        let mut nearby = Vec::new();
        let cell_radius = (radius / self.cell_size).ceil() as i32;
        let center_cell = self.get_cell(position);

        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                let cell = (center_cell.0 + dx, center_cell.1 + dy);
                if let Some(entities) = self.grid.get(&cell) {
                    nearby.extend(entities);
                }
            }
        }

        nearby
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                resolution: (WINDOW_WIDTH, WINDOW_HEIGHT).into(),
                title: "Boids Simulation".to_string(),
                ..default()
            }),
            ..default()
        }))
        .add_plugin(EguiPlugin)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .insert_resource(SimulationParams::default())
        .insert_resource(PhysicsTime::default())
        .insert_resource(SpatialHashGrid::new(75.0, ZOOM_FACTOR)) // Increased cell size for larger window
        .add_startup_system(setup)
        .add_system(update_physics_time)
        .add_system(update_spatial_hash_grid)
        .add_system(boid_movement.after(update_spatial_hash_grid))
        .add_system(ui_system)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
    params: Res<SimulationParams>,
) {
    commands.spawn(Camera2dBundle {
        projection: OrthographicProjection {
            scale: ZOOM_FACTOR,
            ..default()
        },
        ..default()
    });

    let arrow_mesh = create_arrow_mesh();
    let arrow_handle = meshes.add(arrow_mesh);

    let mut rng = rand::thread_rng();

    for _ in 0..BOID_COUNT {
        let x = rng.gen_range(-WINDOW_WIDTH / 2.0..WINDOW_WIDTH / 2.0);
        let y = rng.gen_range(-WINDOW_HEIGHT / 2.0..WINDOW_HEIGHT / 2.0);
        let angle = rng.gen_range(0.0..std::f32::consts::TAU);

        // Create a velocity vector from the angle
        let velocity = Vec2::new(angle.cos(), angle.sin()) * params.max_speed * 0.5;

        commands.spawn((
            MaterialMesh2dBundle {
                mesh: arrow_handle.clone().into(),
                material: materials.add(ColorMaterial::from(Color::WHITE)),
                transform: Transform::from_translation(Vec3::new(x, y, 0.0))
                    .with_rotation(Quat::from_rotation_z(angle))
                    .with_scale(Vec3::splat(10.0)), // Adjust scale as needed
                ..default()
            },
            Boid { velocity },
        ));
    }
}

fn create_arrow_mesh() -> Mesh {
    let mut mesh = Mesh::new(bevy::render::render_resource::PrimitiveTopology::TriangleList);

    let vertices = vec![
        [0.0, 0.5, 0.0],    // Top
        [-0.25, -0.5, 0.0], // Bottom left
        [0.25, -0.5, 0.0],  // Bottom right
        [-0.25, -0.3, 0.0], // Inner left
        [0.25, -0.3, 0.0],  // Inner right
    ];

    let indices = vec![
        0, 1, 2, // Main triangle
        1, 3, 4, // Left wing
        1, 4, 2, // Right wing
    ];

    let normals = vec![[0.0, 0.0, 1.0]; 5];
    let uvs = vec![[0.0, 0.0]; 5];

    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, vertices);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
    mesh.set_indices(Some(bevy::render::mesh::Indices::U32(indices)));

    mesh
}

fn update_spatial_hash_grid(
    mut grid: ResMut<SpatialHashGrid>,
    query: Query<(Entity, &Transform, &Boid)>,
) {
    grid.clear();
    for (entity, transform, boid) in query.iter() {
        grid.insert(entity, transform.translation, boid.velocity);
    }
}
fn update_physics_time(time: Res<Time>, mut physics_time: ResMut<PhysicsTime>) {
    physics_time.accumulated_time += time.delta_seconds();
}

fn boid_movement(
    mut physics_time: ResMut<PhysicsTime>,
    params: Res<SimulationParams>,
    grid: Res<SpatialHashGrid>,
    mut boid_query: Query<(Entity, &mut Transform, &mut Boid)>,
) {
    while physics_time.accumulated_time >= FIXED_TIMESTEP {
        physics_time.accumulated_time -= FIXED_TIMESTEP;

        // Collect boid data
        let boid_data: Vec<(Entity, Vec3, Vec2)> = boid_query
            .iter()
            .map(|(entity, transform, boid)| (entity, transform.translation, boid.velocity))
            .collect();

        // Process boids in parallel
        let boid_updates: Vec<(Entity, Vec3, Vec2)> = boid_data
            .par_iter()
            .map(|&(entity, position, velocity)| {
                let nearby_entities =
                    grid.get_nearby_entities(position.truncate(), params.cohesion_radius);

                let (separation, alignment, cohesion, total) = nearby_entities
                    .into_iter()
                    .filter(|(other_entity, _, _)| *other_entity != entity)
                    .fold(
                        (Vec2::ZERO, Vec2::ZERO, Vec2::ZERO, 0),
                        |(mut sep, mut ali, mut coh, mut tot), (_, other_pos, other_velocity)| {
                            let distance = calculate_wrapped_distance(position, other_pos, ZOOM_FACTOR);

                            if distance < params.separation_radius && distance > 0.0 {
                                sep += calculate_wrapped_direction(position, other_pos, ZOOM_FACTOR) / distance;
                            }

                            if distance < params.alignment_radius {
                                ali += other_velocity;
                                tot += 1;
                            }

                            if distance < params.cohesion_radius {
                                coh += calculate_wrapped_position(position, other_pos, ZOOM_FACTOR).truncate();
                                tot += 1;
                            }

                            (sep, ali, coh, tot)
                        },
                    );

                let (alignment, cohesion) = if total > 0 {
                    (
                        alignment / total as f32,
                        (cohesion / total as f32 - position.truncate()).normalize(),
                    )
                } else {
                    (Vec2::ZERO, Vec2::ZERO)
                };

                let mut acceleration = Vec2::ZERO;
                acceleration += separation.normalize_or_zero() * params.separation_factor;
                acceleration += alignment.normalize_or_zero() * params.alignment_factor;
                acceleration += cohesion.normalize_or_zero() * params.cohesion_factor;

                let mut new_velocity = velocity + acceleration * FIXED_TIMESTEP * params.max_force;

                // Apply linear damping
                new_velocity *= 1.0 - params.linear_damping * FIXED_TIMESTEP;

                // Limit to max speed
                if new_velocity.length() > params.max_speed {
                    new_velocity = new_velocity.normalize() * params.max_speed;
                }

                let mut new_position = position + new_velocity.extend(0.0) * FIXED_TIMESTEP;

                // Wrap around screen edges (adjusted for zoom)
                new_position.x = wrap(new_position.x, -WINDOW_WIDTH / 2.0 * ZOOM_FACTOR, WINDOW_WIDTH / 2.0 * ZOOM_FACTOR);
                new_position.y = wrap(new_position.y, -WINDOW_HEIGHT / 2.0 * ZOOM_FACTOR, WINDOW_HEIGHT / 2.0 * ZOOM_FACTOR);

                (entity, new_position, new_velocity)
            })
            .collect();

        // Apply updates
        for (entity, new_position, new_velocity) in boid_updates {
            if let Ok((_, mut transform, mut boid)) = boid_query.get_mut(entity) {
                transform.translation = new_position;
                boid.velocity = new_velocity;

                // Update rotation to face the direction of movement
                if new_velocity != Vec2::ZERO {
                    let angle = Vec2::Y.angle_between(new_velocity);
                    transform.rotation = Quat::from_rotation_z(angle);
                }
            }
        }
    }
}

fn calculate_wrapped_distance(pos1: Vec3, pos2: Vec3, zoom_factor: f32) -> f32 {
    let dx = (pos1.x - pos2.x).abs();
    let dy = (pos1.y - pos2.y).abs();
    let wrapped_dx = dx.min(WINDOW_WIDTH * zoom_factor - dx);
    let wrapped_dy = dy.min(WINDOW_HEIGHT * zoom_factor - dy);
    (wrapped_dx * wrapped_dx + wrapped_dy * wrapped_dy).sqrt()
}

fn calculate_wrapped_direction(pos1: Vec3, pos2: Vec3, zoom_factor: f32) -> Vec2 {
    let mut direction = pos1.truncate() - pos2.truncate();

    if direction.x.abs() > WINDOW_WIDTH * zoom_factor / 2.0 {
        direction.x = -direction.x.signum() * (WINDOW_WIDTH * zoom_factor - direction.x.abs());
    }
    if direction.y.abs() > WINDOW_HEIGHT * zoom_factor / 2.0 {
        direction.y = -direction.y.signum() * (WINDOW_HEIGHT * zoom_factor - direction.y.abs());
    }

    direction
}

fn calculate_wrapped_position(pos1: Vec3, pos2: Vec3, zoom_factor: f32) -> Vec3 {
    let mut wrapped_pos = pos2;

    if (pos1.x - pos2.x).abs() > WINDOW_WIDTH * zoom_factor / 2.0 {
        wrapped_pos.x += WINDOW_WIDTH * zoom_factor * (pos1.x - pos2.x).signum();
    }
    if (pos1.y - pos2.y).abs() > WINDOW_HEIGHT * zoom_factor / 2.0 {
        wrapped_pos.y += WINDOW_HEIGHT * zoom_factor * (pos1.y - pos2.y).signum();
    }

    wrapped_pos
}

fn wrap(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        max - (min - value) % (max - min)
    } else if value >= max {
        min + (value - min) % (max - min)
    } else {
        value
    }
}

fn ui_system(mut egui_context: EguiContexts, mut params: ResMut<SimulationParams>) {
    egui::SidePanel::left("parameters_panel").show(egui_context.ctx_mut(), |ui| {
        ui.heading("Simulation Parameters");

        ui.add(
            egui::Slider::new(&mut params.separation_radius, 0.0..=100.0).text("Separation Radius"),
        );
        ui.add(
            egui::Slider::new(&mut params.alignment_radius, 0.0..=100.0).text("Alignment Radius"),
        );
        ui.add(egui::Slider::new(&mut params.cohesion_radius, 0.0..=200.0).text("Cohesion Radius"));
        ui.add(
            egui::Slider::new(&mut params.separation_factor, 0.0..=2.0).text("Separation Factor"),
        );
        ui.add(egui::Slider::new(&mut params.alignment_factor, 0.0..=2.0).text("Alignment Factor"));
        ui.add(egui::Slider::new(&mut params.cohesion_factor, 0.0..=2.0).text("Cohesion Factor"));
        ui.add(egui::Slider::new(&mut params.linear_damping, 0.0..=1.0).text("Linear Damping"));
        ui.add(egui::Slider::new(&mut params.max_speed, 0.0..=200.0).text("Max Speed"));
        ui.add(egui::Slider::new(&mut params.max_force, 0.0..=400.0).text("Max Force"));
    });
}
