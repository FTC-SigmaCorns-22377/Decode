#include "DrakeSim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>

#include <drake/multibody/plant/externally_applied_spatial_force.h>

using namespace drake;
using namespace drake::geometry;
using namespace drake::math;
using namespace drake::multibody;
using namespace drake::systems;

namespace {
void RotateInPlace(double &x, double &y, double angle) {
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const double rx = c * x - s * y;
  const double ry = s * x + c * y;
  x = rx;
  y = ry;
}
} // namespace

DrakeSim::DrakeSim(const std::string &urdf_path) {
  DiagramBuilder<double> builder;

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.001);
  plant_ = &plant;
  scene_graph_ = &scene_graph;

  Parser parser(plant_);
  parser.AddModels(urdf_path);

  base_body_index_ = plant_->GetBodyByName("base_link").index();

  const double ground_size = 10.0;
  const double ground_depth = 1.0;
  RigidTransformd X_WG(Eigen::Vector3d(0, 0, -ground_depth / 2.0));
  Box box(ground_size, ground_size, ground_depth);
  plant_->RegisterCollisionGeometry(plant_->world_body(), X_WG, box,
                                    "ground_collision",
                                    CoulombFriction<double>(0.8, 0.8));
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

  // Motor configs derived from RobotModelConstants.
  const double bare_motor_free_speed = 617.84;
  const double bare_motor_stall_torque = 0.187;
  const MotorConfig drive_motor{bare_motor_free_speed / 19.2,
                                bare_motor_stall_torque * 19.2, 12.0};
  const MotorConfig spin_motor{bare_motor_free_speed / 10.0,
                               bare_motor_stall_torque * 10.0, 12.0};
  const MotorConfig flywheel_motor{bare_motor_free_speed / 13.7,
                                   bare_motor_stall_torque * 13.7, 12.0};

  mecanum_params_.lx = 0.2;
  mecanum_params_.ly = 0.2;
  mecanum_params_.wheel_radius = 0.048;
  mecanum_params_.mass = 15.0;
  mecanum_params_.rot_inertia = 0.5;
  mecanum_params_.drive_motor = drive_motor;
  base_z_ = mecanum_params_.wheel_radius;

  const std::vector<std::string> actuator_joint_names = {
      "fl_wheel_joint", "bl_wheel_joint", "br_wheel_joint",
      "fr_wheel_joint", "intake_joint",   "spindexer_joint",
      "turret_joint",   "flywheel_joint", "hood_joint"};

  // Store wheel joint indices for force calc
  wheel_joints_.clear();
  for (int i = 0; i < 4; ++i) {
    wheel_joints_.push_back(
        plant_->GetJointByName(actuator_joint_names[i]).index());
  }

  const std::vector<MotorConfig> actuator_motors = {
      drive_motor, drive_motor, drive_motor,    drive_motor, spin_motor,
      spin_motor,  spin_motor,  flywheel_motor, spin_motor};

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
  add_contact("hood_seg1", Box(0.01, 0.08, 0.04), "hood_seg1_contact",
              CoulombFriction<double>(0.4, 0.3));
  add_contact("hood_seg2", Box(0.01, 0.08, 0.04), "hood_seg2_contact",
              CoulombFriction<double>(0.4, 0.3));
  add_contact("hood_seg3", Box(0.01, 0.08, 0.04), "hood_seg3_contact",
              CoulombFriction<double>(0.4, 0.3));
  add_contact("transfer_ramp", Box(0.12, 0.08, 0.02), "transfer_contact",
              CoulombFriction<double>(0.6, 0.5));

  const ModelInstanceIndex ball_instance = plant_->AddModelInstance("balls");
  for (int i = 0; i < 10; ++i) {
    std::string name = "ball_" + std::to_string(i);
    const auto &ball_body = plant_->AddRigidBody(
        name, ball_instance,
        SpatialInertia<double>::SolidSphereWithMass(0.05, 0.1));
    plant_->RegisterCollisionGeometry(ball_body, RigidTransformd::Identity(),
                                      Sphere(0.05), name + "_col",
                                      CoulombFriction<double>(0.5, 0.5));
    plant_->RegisterVisualGeometry(ball_body, RigidTransformd::Identity(),
                                   Sphere(0.05), name + "_vis",
                                   Vector4<double>(1, 0.5, 0, 1));
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
  // simulator_->set_target_realtime_rate(1.0); // Remove limit for debugging
  simulator_->Initialize();

  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);
  for (int i = 0; i < 10; ++i) {
    std::string name = "ball_" + std::to_string(i);
    const auto &body = plant_->GetBodyByName(name);
    plant_->SetFreeBodyPose(&plant_context, body,
                            RigidTransformd(Eigen::Vector3d(0, 0, -100 - i)));
  }

  // Set initial pose
  const auto &base_body = plant_->get_body(base_body_index_);
  plant_->SetFreeBodyPose(&plant_context, base_body,
                          RigidTransformd(Eigen::Vector3d(0, 0, base_z_)));
}

DrakeSim::~DrakeSim() {}

double DrakeSim::MotorTorque(const MotorConfig &motor, double power,
                             double omega) const {
  const double clamped = std::clamp(power, -1.0, 1.0);
  const double voltage_scale = motor.v_ref > 0.0 ? (12.0 / motor.v_ref) : 1.0;
  return motor.stall_torque *
         (clamped * voltage_scale - omega / motor.free_speed);
}

// Replaced simple kinematic integration with physics-based forces
void DrakeSim::Step(double dt, const std::vector<double> &inputs) {
  if (dt <= 0.0)
    return;

  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

  // Sub-stepping
  const int steps = 10;
  const double sub_dt = dt / steps;

  for (int step = 0; step < steps; ++step) {
    Eigen::VectorXd velocities = plant_->GetVelocities(plant_context);
    Eigen::VectorXd torques = Eigen::VectorXd::Zero(plant_->num_actuators());

    // 1. Calculate Motor Torques for ALL actuators
    for (const auto &actuator : actuators_) {
      const double command =
          actuator.input_index < static_cast<int>(inputs.size())
              ? inputs[actuator.input_index]
              : 0.0;
      const JointActuator<double> &joint_actuator =
          plant_->get_joint_actuator(actuator.index);
      const Joint<double> &joint = joint_actuator.joint();
      double joint_vel = 0.0;
      if (joint.num_velocities() == 1) {
        joint_vel = velocities[joint.velocity_start()];
      }
      torques[actuator.index] = MotorTorque(actuator.motor, command, joint_vel);
    }
    plant_->get_actuation_input_port().FixValue(&plant_context, torques);

    // 2. Apply Ground Reaction Forces for Wheels (direct torque -> force)
    const std::vector<std::string> wheel_names = {"fl_wheel", "bl_wheel",
                                                  "br_wheel", "fr_wheel"};
    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
    const std::vector<Eigen::Vector3d> traction_dirs = {
        Eigen::Vector3d(inv_sqrt2, -inv_sqrt2, 0), // FL
        Eigen::Vector3d(inv_sqrt2, inv_sqrt2, 0),  // BL
        Eigen::Vector3d(inv_sqrt2, -inv_sqrt2, 0), // BR
        Eigen::Vector3d(inv_sqrt2, inv_sqrt2, 0)   // FR
    };

    std::vector<ExternallyAppliedSpatialForce<double>> spatial_forces;
    const auto &base_body = plant_->get_body(base_body_index_);
    RigidTransformd X_W_Base =
        plant_->EvalBodyPoseInWorld(plant_context, base_body);
    double yaw = RollPitchYaw<double>(X_W_Base.rotation()).yaw_angle();

    for (size_t i = 0; i < 4; ++i) {
      const auto &wheel_body = plant_->GetBodyByName(wheel_names[i]);
      RigidTransformd X_W_Wheel =
          plant_->EvalBodyPoseInWorld(plant_context, wheel_body);

      // Only generate traction if on/near ground
      if (X_W_Wheel.translation().z() < mecanum_params_.wheel_radius * 1.2) {
        // Get torque calculated in step 1
        double T = torques[actuators_[i].index];
        double f_mag = T / mecanum_params_.wheel_radius;

        // Clamp force for stability
        f_mag = std::clamp(f_mag, -200.0, 200.0);

        if (std::isnan(f_mag))
          f_mag = 0;

        Eigen::Vector3d traction_dir_world =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * traction_dirs[i];

        Eigen::Vector3d force_W = f_mag * traction_dir_world;

        // Apply to base_link at wheel center position (expressed in base frame)
        Eigen::Vector3d p_Wheel_W = X_W_Wheel.translation();
        Eigen::Vector3d p_BoBq_B = X_W_Base.inverse() * p_Wheel_W;

        spatial_forces.push_back(
            {base_body_index_, p_BoBq_B,
             SpatialForce<double>(Eigen::Vector3d::Zero(), force_W)});
      }
    }

    plant_->get_applied_spatial_force_input_port().FixValue(&plant_context,
                                                            spatial_forces);

    try {
      simulator_->AdvanceTo(context.get_time() + sub_dt);
    } catch (...) {
      // Silence exceptions during sub-stepping
    }
  }

  // Update internal state
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

std::vector<double> DrakeSim::GetState() {
  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

  Eigen::VectorXd q = plant_->GetPositions(plant_context);
  Eigen::VectorXd v = plant_->GetVelocities(plant_context);

  std::vector<double> state;
  state.reserve(6 + joint_state_order_.size() * 2);

  state.push_back(mecanum_state_.pos.x);
  state.push_back(mecanum_state_.pos.y);
  state.push_back(mecanum_state_.pos.theta);
  state.push_back(mecanum_state_.vel.x);
  state.push_back(mecanum_state_.vel.y);
  state.push_back(mecanum_state_.vel.theta);

  for (const auto &joint_name : joint_state_order_) {
    const auto &joint = plant_->GetJointByName(joint_name);
    if (joint.num_positions() == 1) {
      state.push_back(q[joint.position_start()]);
    } else {
      state.push_back(0.0);
    }
  }

  for (const auto &joint_name : joint_state_order_) {
    const auto &joint = plant_->GetJointByName(joint_name);
    if (joint.num_velocities() == 1) {
      state.push_back(v[joint.velocity_start()]);
    } else {
      state.push_back(0.0);
    }
  }

  // Add ball positions (x, y, z for each of the 10 balls)
  for (int i = 0; i < 10; ++i) {
    std::string name = "ball_" + std::to_string(i);
    const auto &body = plant_->GetBodyByName(name);
    const auto &X_WB = plant_->EvalBodyPoseInWorld(plant_context, body);
    const auto &p_WB = X_WB.translation();
    state.push_back(p_WB.x());
    state.push_back(p_WB.y());
    state.push_back(p_WB.z());
  }

  return state;
}

void DrakeSim::SpawnBall(double x, double y, double z) {
  if (ball_count_ >= 10) {
    std::cerr << "Ball pool exhausted!" << std::endl;
    return;
  }

  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

  std::string name = "ball_" + std::to_string(ball_count_);
  const auto &body = plant_->GetBodyByName(name);

  plant_->SetFreeBodyPose(&plant_context, body,
                          RigidTransformd(Eigen::Vector3d(x, y, z)));
  plant_->SetFreeBodySpatialVelocity(&plant_context, body,
                                     SpatialVelocity<double>::Zero());

  ball_count_++;
}
