use bevy::prelude::*;
use rand::Rng;

const WINDOW_WIDTH: f32 = 800.0;
const WINDOW_HEIGHT: f32 = 600.0;
const BOID_SPEED: f32 = 100.0;
const BOID_COUNT: usize = 100;
const FIXED_TIMESTEP: f32 = 1.0 / 60.0; // 60 Hz fixed update rate

#[derive(Component)]
struct Boid;

#[derive(Resource)]
struct SimulationParams {
    separation_radius: f32,
    alignment_radius: f32,
    cohesion_radius: f32,
    separation_factor: f32,
    alignment_factor: f32,
    cohesion_factor: f32,
}

impl Default for SimulationParams {
    fn default() -> Self {
        SimulationParams {
            separation_radius: 20.0,
            alignment_radius: 50.0,
            cohesion_radius: 100.0,
            separation_factor: 1.0,
            alignment_factor: 0.5,
            cohesion_factor: 0.3,
        }
    }
}

#[derive(Resource, Default)]
struct PhysicsTime {
    accumulated_time: f32,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .insert_resource(SimulationParams::default())
        .insert_resource(PhysicsTime::default())
        .add_startup_system(setup)
        .add_system(update_physics_time)
        .add_system(boid_movement.after(update_physics_time))
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn(Camera2dBundle::default());

    let mut rng = rand::thread_rng();

    for _ in 0..BOID_COUNT {
        let x = rng.gen_range(-WINDOW_WIDTH / 2.0..WINDOW_WIDTH / 2.0);
        let y = rng.gen_range(-WINDOW_HEIGHT / 2.0..WINDOW_HEIGHT / 2.0);
        let angle = rng.gen_range(0.0..std::f32::consts::TAU);

        commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::WHITE,
                    custom_size: Some(Vec2::new(10.0, 5.0)),
                    ..default()
                },
                transform: Transform::from_translation(Vec3::new(x, y, 0.0))
                    .with_rotation(Quat::from_rotation_z(angle)),
                ..default()
            },
            Boid,
        ));
    }
}

fn update_physics_time(time: Res<Time>, mut physics_time: ResMut<PhysicsTime>) {
    physics_time.accumulated_time += time.delta_seconds();
}

fn boid_movement(
    mut physics_time: ResMut<PhysicsTime>,
    params: Res<SimulationParams>,
    mut boid_query: Query<(&mut Transform, Entity), With<Boid>>,
) {
    while physics_time.accumulated_time >= FIXED_TIMESTEP {
        physics_time.accumulated_time -= FIXED_TIMESTEP;

        let boids: Vec<(Entity, Vec3, Vec3)> = boid_query
            .iter()
            .map(|(transform, entity)| {
                (entity, transform.translation, transform.up())
            })
            .collect();

        for (mut transform, entity) in boid_query.iter_mut() {
            let mut separation = Vec3::ZERO;
            let mut alignment = Vec3::ZERO;
            let mut cohesion = Vec3::ZERO;
            let mut total = 0;

            for (other_entity, other_pos, other_dir) in &boids {
                if *other_entity == entity {
                    continue;
                }

                let distance = transform.translation.distance(*other_pos);

                if distance < params.separation_radius {
                    separation += transform.translation - *other_pos;
                }

                if distance < params.alignment_radius {
                    alignment += *other_dir;
                    total += 1;
                }

                if distance < params.cohesion_radius {
                    cohesion += *other_pos;
                    total += 1;
                }
            }

            if total > 0 {
                alignment /= total as f32;
                cohesion /= total as f32;
                cohesion = (cohesion - transform.translation).normalize();
            }

            let mut velocity = transform.up();
            velocity += separation.normalize_or_zero() * params.separation_factor;
            velocity += alignment.normalize_or_zero() * params.alignment_factor;
            velocity += cohesion.normalize_or_zero() * params.cohesion_factor;

            velocity = velocity.normalize();

            transform.translation += velocity * BOID_SPEED * FIXED_TIMESTEP;
            transform.rotation = Quat::from_rotation_arc(Vec3::Y, velocity);

            // Wrap around screen edges
            transform.translation.x = wrap(transform.translation.x, -WINDOW_WIDTH / 2.0, WINDOW_WIDTH / 2.0);
            transform.translation.y = wrap(transform.translation.y, -WINDOW_HEIGHT / 2.0, WINDOW_HEIGHT / 2.0);
        }
    }
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
