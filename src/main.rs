use std::f32::consts::{PI, FRAC_PI_2};
use bevy::prelude::*;
use bevy::core_pipeline::Skybox;
use avian3d::prelude::*;

const CAM_OFFSET: Vec3 = Vec3::new(-2.0, 1.0, 5.0);

fn main() {
  App::new()
    .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
    .add_plugins(PhysicsDebugPlugin::default())
    .add_systems(Startup, setup)
    .add_systems(FixedUpdate, update_aircraft)
    .add_systems(FixedPostUpdate, update_camera.after(PhysicsSet::Sync))
    .run();
}

#[derive(Component)]
struct Aircraft;

fn setup(
  mut commands: Commands,
  mut meshes: ResMut<Assets<Mesh>>,
  mut materials: ResMut<Assets<StandardMaterial>>,
  assets: Res<AssetServer>,
) {
  let cube = meshes.add(Cuboid::new(1.0, 1.0, 3.0));
  let mat = materials.add(Color::srgb(0.7, 0.7, 0.8));

  commands.spawn((
    Aircraft,
    // Mesh3d(cube.clone()),
    // MeshMaterial3d(mat),
    Transform::IDENTITY,
    RigidBody::Dynamic,
    ExternalForce::new(Vec3::ZERO).with_persistence(false),
    ExternalTorque::new(Vec3::ZERO).with_persistence(false),
    // LinearVelocity(Vec3::new(0.0, 0.0, 30.0)),
    Mass::new(1000.0),
    Collider::cuboid(1.0, 1.0, 3.0),
  ));

  commands.spawn((
    DirectionalLight {
      illuminance: 32000.0,
      ..default()
    },
    Transform::from_xyz(0.0, 2.0, 0.0).with_rotation(Quat::from_rotation_x(-PI / 4.)),
  ));

  let skybox = assets.load("kloppenheim_06.ktx2");

  commands.spawn((
    Camera3d::default(),
    Transform::from_translation(CAM_OFFSET),
    Skybox {
      brightness: 250.0,
      image: skybox.clone(),
      ..default()
    },
    // EnvironmentMapLight {
    //     diffuse_map: skybox,
    //     intensity: 2000.0,
    //     ..default()
    // },
  ));
}

fn update_aircraft(
  mut query: Query<
    (
      &Transform,
      &LinearVelocity,
      &AngularVelocity,
      &CenterOfMass,
      &mut ExternalForce,
      &mut ExternalTorque,
    ),
    With<Aircraft>,
  >,
  mut gizmos: Gizmos,
  time: Res<Time>,
) {
  let (transform, linear_vel, angular_vel, center_of_mass, mut force, mut torque) =
    query.get_single_mut().unwrap();

  let wing_area = 20.0;
  let lift_coefficient = 1.2;
  let drag_coefficient = 0.05;
  let air_density = 1.225;

  let local_vel = transform.rotation.inverse() * linear_vel.0;
  let aoa = local_vel.y.atan2(-local_vel.z);
  let dynamic_pressure = 0.5 * air_density * linear_vel.length_squared();

  let lift = transform.up() * dynamic_pressure * lift_coefficient * wing_area;
  let drag = linear_vel.0.normalize_or_zero() * dynamic_pressure * drag_coefficient * wing_area;
  let thrust = transform.forward() * 1000.0;

  gizmos.ray(
    transform.translation,
    lift / 1000.0,
    Color::srgb(0.0, 1.0, 0.0),
  );
  gizmos.ray(
    transform.translation,
    drag / 1000.0,
    Color::srgb(0.0, 0.0, 1.0),
  );
  gizmos.ray(
    transform.translation,
    thrust / 1000.0,
    Color::srgb(1.0, 0.0, 0.0),
  );

  force.apply_force(lift + drag + thrust);

  info!("pos = {}", transform.translation);
  info!("rot = {:?}", transform.rotation.to_euler(EulerRot::XYZ));
  info!("linear_vel = {}", linear_vel.0);
  info!("aoa = {}", aoa);
  info!("lift = {}, drag = {}, thrust = {}", lift, drag, thrust);
}

fn update_camera(
  mut cam: Query<&mut Transform, (With<Camera3d>, Without<Aircraft>)>,
  target: Query<&Transform, (With<Aircraft>, Without<Camera3d>)>,
  time: Res<Time>,
) {
  let mut cam = cam.get_single_mut().unwrap();
  let target = target.get_single().unwrap();

  let t = (time.delta_secs() * 50.0).min(1.0);
  cam.translation = cam
    .translation
    .lerp(target.translation + target.rotation * CAM_OFFSET, t);
  let target_dir = (target.translation - cam.translation).normalize();
  cam.rotation = cam.rotation.slerp(
    // Quat::from_axis_angle(Vec3::Y, Vec3::NEG_Z.angle_between(target_dir)),
    Quat::from_rotation_arc(Vec3::NEG_Z, target_dir),
    t,
  );
}
