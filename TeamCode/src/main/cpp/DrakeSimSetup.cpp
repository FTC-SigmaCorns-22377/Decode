#include "DrakeSim.hpp"

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>

using namespace drake;
using namespace drake::geometry;
using namespace drake::math;
using namespace drake::multibody;
using namespace drake::systems;

DrakeSim::DrakeSim(const std::string &urdf_path) : urdf_path_(urdf_path) {
  BuildSimulator(nullptr);
}

DrakeSim::~DrakeSim() {}

void DrakeSim::BuildSimulator(const RobotState *state) {
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.005);
  plant_ = &plant;
  scene_graph_ = &scene_graph;

  Parser parser(plant_);
  parser.AddModels(urdf_path_);

  base_body_index_ = plant_->GetBodyByName("base_link").index();

  const double ground_size = 10.0;
  const double ground_depth = 1.0;
  RigidTransformd X_WG(Eigen::Vector3d(0, 0, -ground_depth / 2.0));
  Box box(ground_size, ground_size, ground_depth);
  plant_->RegisterCollisionGeometry(plant_->world_body(), X_WG, box,
                                    "ground_collision",
                                    CoulombFriction<double>(0.0, 0.0));
  plant_->RegisterVisualGeometry(plant_->world_body(), X_WG, box,
                                 "ground_visual",
                                 Vector4<double>(0.3, 0.3, 0.3, 1.0));

  plant_->mutable_gravity_field().set_gravity_vector(
      Eigen::Vector3d(0, 0, -9.81));

  // --- Field Elements ---
  const double L = 3.6576; // 12 ft
  const double h = 0.312;  // 12 in
  const double w_thick = 0.02;
  const CoulombFriction<double> wall_friction(0.3, 0.3);

  auto add_wall = [&](const std::string &name, const RigidTransformd &X_WW) {
    Box wall_shape((name == "N" || name == "S") ? L : w_thick,
                   (name == "N" || name == "S") ? w_thick : L, h);
    plant_->RegisterCollisionGeometry(plant_->world_body(), X_WW, wall_shape,
                                      "wall_" + name + "_col", wall_friction);
    plant_->RegisterVisualGeometry(plant_->world_body(), X_WW, wall_shape,
                                   "wall_" + name + "_vis",
                                   Vector4<double>(0.8, 0.8, 0.8, 0.3));
  };

  add_wall("N", RigidTransformd(Eigen::Vector3d(0, L / 2, h / 2)));
  add_wall("S", RigidTransformd(Eigen::Vector3d(0, -L / 2, h / 2)));
  add_wall("E", RigidTransformd(Eigen::Vector3d(L / 2, 0, h / 2)));
  add_wall("W", RigidTransformd(Eigen::Vector3d(-L / 2, 0, h / 2)));

  // Ramp STL
  const std::string ramp_stl_path =
      "TeamCode/src/main/assets/Ramp Assembly - am-5715 (1).stl";
  const std::string ramp_stl_path2 =
      "src/main/assets/Ramp Assembly - am-5715 (1).stl";
  std::string actual_ramp_path =
      std::filesystem::exists(ramp_stl_path) ? ramp_stl_path : ramp_stl_path2;

  if (std::filesystem::exists(actual_ramp_path)) {
    // Target corner: tx = -1.792859, ty = 1.792066 (North-West)
    // Rotation: X = PI/2

    RigidTransformd X_WR(RollPitchYaw<double>(M_PI / 2, 0, 0),
                         Eigen::Vector3d(-1.792859, 1.792066, 0));

    Mesh ramp_mesh(actual_ramp_path, 0.001);

    plant_->RegisterVisualGeometry(plant_->world_body(), X_WR, ramp_mesh,
                                   "ramp_blue_vis",
                                   Vector4<double>(0, 0, 1, 1));

    // Red Ramp (Mirrored)
    RigidTransformd X_WR_Red(RollPitchYaw<double>(M_PI / 2, 0, 0),
                             Eigen::Vector3d(1.792859, 1.792066, 0));

    plant_->RegisterVisualGeometry(plant_->world_body(), X_WR_Red, ramp_mesh,
                                   "ramp_red_vis", Vector4<double>(1, 0, 0, 1));
  }

  // Motor configs from RobotModelConstants (passed via SetMotorParameters)
  const double bare_motor_free_speed = motor_configs_.bare_motor_free_speed;
  const double bare_motor_stall_torque = motor_configs_.bare_motor_stall_torque;
  const MotorConfig drive_motor{bare_motor_free_speed / motor_configs_.drive_gear_ratio,
                                bare_motor_stall_torque * motor_configs_.drive_gear_ratio, 12.0};
  const MotorConfig spindexer_motor{bare_motor_free_speed / motor_configs_.spindexer_gear_ratio,
                                    bare_motor_stall_torque * motor_configs_.spindexer_gear_ratio, 12.0};
  const MotorConfig turret_motor{bare_motor_free_speed / motor_configs_.turret_gear_ratio,
                                 bare_motor_stall_torque * motor_configs_.turret_gear_ratio, 12.0};
  const MotorConfig spin_motor{bare_motor_free_speed / motor_configs_.intake_hood_gear_ratio,
                               bare_motor_stall_torque * motor_configs_.intake_hood_gear_ratio, 12.0};
  const MotorConfig flywheel_motor{bare_motor_free_speed / motor_configs_.flywheel_gear_ratio,
                                   bare_motor_stall_torque * motor_configs_.flywheel_gear_ratio, 12.0};

  mecanum_params_.lx = 0.2;
  mecanum_params_.ly = 0.2;
  mecanum_params_.wheel_radius = 0.048;
  mecanum_params_.mass = 15.0;
  mecanum_params_.rot_inertia = 0.5;
  mecanum_params_.drive_motor = drive_motor;

  const std::vector<std::string> actuator_joint_names = {
      "fl_wheel_joint", "bl_wheel_joint", "br_wheel_joint",
      "fr_wheel_joint", "intake_joint",   "spindexer_joint",
      "turret_joint",   "flywheel_joint", "hood_joint"};

  const std::vector<MotorConfig> actuator_motors = {
      drive_motor, drive_motor, drive_motor, drive_motor, spin_motor,
      spindexer_motor, turret_motor, flywheel_motor, spin_motor};

  joint_state_order_ = actuator_joint_names;

  for (const auto &name : actuator_joint_names) {
    const auto &joint = plant_->GetJointByName(name);
    const std::string actuator_name = name + "_actuator";
    plant_->AddJointActuator(actuator_name, joint);
  }

  // Contact geometry overrides
  auto add_contact = [&](const std::string &body_name, const Shape &shape,
                         const std::string &geom_name,
                         const CoulombFriction<double> &friction,
                         const RigidTransformd &X_BG =
                             RigidTransformd::Identity()) {
    const auto &body = plant_->GetBodyByName(body_name);
    plant_->RegisterCollisionGeometry(body, X_BG, shape, geom_name, friction);
  };

  // Override wheel friction to be 0 for manual mecanum physics
  // Radius changed to 0.048 to match kinematic model and prevent penetration
  add_contact("fl_wheel", Cylinder(0.048, 0.05), "fl_wheel_col",
              CoulombFriction<double>(0.0, 0.0),
              RigidTransformd(RollPitchYaw<double>(M_PI / 2, 0, 0),
                              Eigen::Vector3d::Zero()));
  add_contact("fr_wheel", Cylinder(0.048, 0.05), "fr_wheel_col",
              CoulombFriction<double>(0.0, 0.0),
              RigidTransformd(RollPitchYaw<double>(M_PI / 2, 0, 0),
                              Eigen::Vector3d::Zero()));
  add_contact("bl_wheel", Cylinder(0.048, 0.05), "bl_wheel_col",
              CoulombFriction<double>(0.0, 0.0),
              RigidTransformd(RollPitchYaw<double>(M_PI / 2, 0, 0),
                              Eigen::Vector3d::Zero()));
  add_contact("br_wheel", Cylinder(0.048, 0.05), "br_wheel_col",
              CoulombFriction<double>(0.0, 0.0),
              RigidTransformd(RollPitchYaw<double>(M_PI / 2, 0, 0),
                              Eigen::Vector3d::Zero()));

  add_contact("intake_link", Cylinder(0.03, 0.30), "intake_soft_contact",
              CoulombFriction<double>(1.2, 1.0));
  add_contact("spindexer_base_link", Cylinder(0.15, 0.01), "spindexer_contact",
              CoulombFriction<double>(0.7, 0.6));
  add_contact("flywheel_link", Cylinder(0.04, 0.02), "flywheel_contact",
              CoulombFriction<double>(1.3, 1.1));
  add_contact("transfer_ramp", Box(0.12, 0.08, 0.02), "transfer_contact",
              CoulombFriction<double>(0.6, 0.5));

  ball_instance_ = plant_->AddModelInstance("balls");
  ball_bodies_.clear();
  for (size_t i = 0; i < balls_.size(); ++i) {
    std::string name = "ball_" + std::to_string(i);
    const auto &ball_body = plant_->AddRigidBody(
        name, ball_instance_,
        SpatialInertia<double>::SolidSphereWithMass(0.05, 0.1));
    plant_->RegisterCollisionGeometry(ball_body, RigidTransformd::Identity(),
                                      Sphere(0.05), name + "_col",
                                      CoulombFriction<double>(0.5, 0.5));
    plant_->RegisterVisualGeometry(ball_body, RigidTransformd::Identity(),
                                   Sphere(0.05), name + "_vis",
                                   Vector4<double>(1, 0.5, 0, 1));
    ball_bodies_.push_back(ball_body.index());
  }

  plant_->Finalize();

  actuators_.clear();
  for (size_t i = 0; i < actuator_joint_names.size(); ++i) {
    const std::string actuator_name = actuator_joint_names[i] + "_actuator";
    const auto &actuator = plant_->GetJointActuatorByName(actuator_name);
    actuators_.push_back(
        {actuator.index(), static_cast<int>(i), actuator_motors[i]});
  }

  diagram_ = builder.Build();
  simulator_owner_ = std::make_unique<Simulator<double>>(*diagram_);
  simulator_ = simulator_owner_.get();
  simulator_->Initialize();

  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);
  auto default_context = plant_->CreateDefaultContext();
  const auto &base_body_for_default = plant_->GetBodyByName("base_link");
  const auto &fl_wheel_body = plant_->GetBodyByName("fl_wheel");
  const double wheel_center_z =
      plant_->CalcRelativeTransform(*default_context,
                                    base_body_for_default.body_frame(),
                                    fl_wheel_body.body_frame())
          .translation()
          .z();
  const double base_z = mecanum_params_.wheel_radius - wheel_center_z;

  if (state) {
    context.SetTime(state->time_sec);

    const auto &base_body = plant_->get_body(base_body_index_);
    RigidTransformd X_WB(RollPitchYaw<double>(state->roll, state->pitch,
                                             state->yaw),
                         Eigen::Vector3d(state->x, state->y, state->z));
    plant_->SetFreeBodyPose(&plant_context, base_body, X_WB);
    plant_->SetFreeBodySpatialVelocity(
        &plant_context, base_body,
        SpatialVelocity<double>(Eigen::Vector3d(state->wx, state->wy, state->wz),
                                Eigen::Vector3d(state->vx, state->vy, state->vz)));

    for (size_t i = 0; i < joint_state_order_.size(); ++i) {
      const auto &joint = plant_->GetJointByName(joint_state_order_[i]);
      if (joint.num_positions() == 1 &&
          i < state->joint_positions.size()) {
        Eigen::VectorXd q(1);
        q[0] = state->joint_positions[i];
        joint.SetPositions(&plant_context, q);
      }
      if (joint.num_velocities() == 1 &&
          i < state->joint_velocities.size()) {
        Eigen::VectorXd v(1);
        v[0] = state->joint_velocities[i];
        joint.SetVelocities(&plant_context, v);
      }
    }

    for (size_t i = 0; i < balls_.size(); ++i) {
      const auto &body = plant_->get_body(ball_bodies_[i]);
      const auto &ball = balls_[i];
      plant_->SetFreeBodyPose(
          &plant_context, body,
          RigidTransformd(Eigen::Vector3d(ball.x, ball.y, ball.z)));
      plant_->SetFreeBodySpatialVelocity(
          &plant_context, body,
          SpatialVelocity<double>(Eigen::Vector3d(ball.wx, ball.wy, ball.wz),
                                  Eigen::Vector3d(ball.vx, ball.vy, ball.vz)));
    }

    // Re-initialize the simulator after modifying context time and state
    // This updates Drake's internal time tracking to match the restored state
    simulator_->Initialize();
  } else {
    // Set initial pose
    const auto &base_body = plant_->get_body(base_body_index_);
    plant_->SetFreeBodyPose(&plant_context, base_body,
                            RigidTransformd(Eigen::Vector3d(0, 0, base_z)));

    // Initialize hood pitch to 45 degrees
    const auto &hood_joint = plant_->GetJointByName("hood_joint");
    Eigen::VectorXd q(1);
    q[0] = M_PI / 4.0;  // 45 degrees
    hood_joint.SetPositions(&plant_context, q);
  }

  const auto &base_body = plant_->get_body(base_body_index_);
  RigidTransformd X_WB = plant_->EvalBodyPoseInWorld(plant_context, base_body);
  SpatialVelocity<double> V_WB =
      plant_->EvalBodySpatialVelocityInWorld(plant_context, base_body);

  mecanum_state_.pos.x = X_WB.translation().x();
  mecanum_state_.pos.y = X_WB.translation().y();
  mecanum_state_.pos.theta = RollPitchYaw<double>(X_WB.rotation()).yaw_angle();

  Eigen::Vector3d v_world = V_WB.translational();
  double yaw = mecanum_state_.pos.theta;
  double c = std::cos(yaw), s = std::sin(yaw);

  mecanum_state_.vel.x = v_world.x() * c + v_world.y() * s;
  mecanum_state_.vel.y = -v_world.x() * s + v_world.y() * c;
  mecanum_state_.vel.theta = V_WB.rotational().z();
}

DrakeSim::RobotState DrakeSim::CaptureRobotState() const {
  RobotState state;
  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);
  const auto &base_body = plant_->get_body(base_body_index_);
  RigidTransformd X_WB = plant_->EvalBodyPoseInWorld(plant_context, base_body);
  SpatialVelocity<double> V_WB =
      plant_->EvalBodySpatialVelocityInWorld(plant_context, base_body);
  RollPitchYaw<double> rpy(X_WB.rotation());

  state.x = X_WB.translation().x();
  state.y = X_WB.translation().y();
  state.z = X_WB.translation().z();
  state.roll = rpy.roll_angle();
  state.pitch = rpy.pitch_angle();
  state.yaw = rpy.yaw_angle();
  state.wx = V_WB.rotational().x();
  state.wy = V_WB.rotational().y();
  state.wz = V_WB.rotational().z();
  state.vx = V_WB.translational().x();
  state.vy = V_WB.translational().y();
  state.vz = V_WB.translational().z();
  state.time_sec = context.get_time();

  state.joint_positions.resize(joint_state_order_.size());
  state.joint_velocities.resize(joint_state_order_.size());
  for (size_t i = 0; i < joint_state_order_.size(); ++i) {
    const auto &joint = plant_->GetJointByName(joint_state_order_[i]);
    if (joint.num_positions() == 1) {
      state.joint_positions[i] = joint.GetPositions(plant_context)[0];
    } else {
      state.joint_positions[i] = 0.0;
    }
    if (joint.num_velocities() == 1) {
      state.joint_velocities[i] = joint.GetVelocities(plant_context)[0];
    } else {
      state.joint_velocities[i] = 0.0;
    }
  }

  return state;
}

void DrakeSim::RefreshBallStates() {
  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);
  balls_.resize(ball_bodies_.size());
  for (size_t i = 0; i < ball_bodies_.size(); ++i) {
    const auto &body = plant_->get_body(ball_bodies_[i]);
    const auto &X_WB = plant_->EvalBodyPoseInWorld(plant_context, body);
    const auto &V_WB =
        plant_->EvalBodySpatialVelocityInWorld(plant_context, body);
    balls_[i].x = X_WB.translation().x();
    balls_[i].y = X_WB.translation().y();
    balls_[i].z = X_WB.translation().z();
    balls_[i].wx = V_WB.rotational().x();
    balls_[i].wy = V_WB.rotational().y();
    balls_[i].wz = V_WB.rotational().z();
    balls_[i].vx = V_WB.translational().x();
    balls_[i].vy = V_WB.translational().y();
    balls_[i].vz = V_WB.translational().z();
  }
}
