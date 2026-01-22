#include "DrakeSim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/plant/externally_applied_spatial_force.h>

using namespace drake;
using namespace drake::math;
using namespace drake::multibody;

double DrakeSim::MotorTorque(const MotorConfig &motor, double power,
                             double omega) const {
  const double clamped = std::clamp(power, -1.0, 1.0);
  const double voltage_scale = motor.v_ref > 0.0 ? (12.0 / motor.v_ref) : 1.0;
  return motor.stall_torque *
         (clamped * voltage_scale - omega / motor.free_speed);
}

// Replaced simple kinematic integration with physics-based forces
void DrakeSim::Step(double dt, const std::vector<double> &inputs) {
  if (dt <= 0.0) {
    return;
  }

  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

  // Sub-stepping
  constexpr int kSubsteps = 10;
  const double sub_dt = dt / kSubsteps;

  static const std::array<const char *, 4> wheel_names = {
      "fl_wheel", "bl_wheel", "br_wheel", "fr_wheel"};
  const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
  static const std::array<Eigen::Vector3d, 4> traction_dirs = {
      Eigen::Vector3d(inv_sqrt2, -inv_sqrt2, 0), // FL
      Eigen::Vector3d(inv_sqrt2, inv_sqrt2, 0),  // BL
      Eigen::Vector3d(inv_sqrt2, -inv_sqrt2, 0), // BR
      Eigen::Vector3d(inv_sqrt2, inv_sqrt2, 0)   // FR
  };

  for (int step = 0; step < kSubsteps; ++step) {
    for (auto &force : last_wheel_forces_W_) {
      force = {0.0, 0.0, 0.0};
    }

    Eigen::VectorXd velocities = plant_->GetVelocities(plant_context);
    Eigen::VectorXd torques = Eigen::VectorXd::Zero(plant_->num_actuators());

    const auto &base_body = plant_->get_body(base_body_index_);
    RigidTransformd X_W_Base =
        plant_->EvalBodyPoseInWorld(plant_context, base_body);
    double yaw = RollPitchYaw<double>(X_W_Base.rotation()).yaw_angle();

    // Get current robot velocity
    SpatialVelocity<double> V_WB =
        plant_->EvalBodySpatialVelocityInWorld(plant_context, base_body);
    Eigen::Vector3d v_world = V_WB.translational();
    double omega_world = V_WB.rotational().z();

    // Transform world velocity to robot body frame
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    const double v_body_x = v_world.x() * c + v_world.y() * s;
    const double v_body_y = -v_world.x() * s + v_world.y() * c;
    const double omega_body = omega_world;

    const double l = mecanum_params_.lx + mecanum_params_.ly;
    const double l2 = std::sqrt(mecanum_params_.lx * mecanum_params_.lx +
                                mecanum_params_.ly * mecanum_params_.ly);
    const double r = mecanum_params_.wheel_radius;

    // ACCURATE MECANUM DYNAMICS MODEL
    // Based on MecanumDynamics.kt - uses inverse kinematics to get wheel velocities
    // from current robot velocity, then calculates motor torques

    // Inverse velocity kinematics: robot velocity → wheel velocities
    // [FL, BL, BR, FR] = inverseVel * [vx, vy, omega, 0]
    const std::array<double, 4> wheel_omegas = {
        (v_body_x - v_body_y - l * omega_body) / r,  // FL
        (v_body_x + v_body_y - l * omega_body) / r,  // BL
        (v_body_x - v_body_y + l * omega_body) / r,  // BR
        (v_body_x + v_body_y + l * omega_body) / r   // FR
    };

    // Get motor power commands
    std::array<double, 4> motor_powers = {0, 0, 0, 0};
    for (size_t i = 0; i < 4; ++i) {
      motor_powers[i] = (i < inputs.size()) ? std::clamp(inputs[i], -1.0, 1.0) : 0.0;
    }

    // Calculate motor torques using DC motor model
    // τ = τ_stall * (power - ω/ω_free)
    std::array<double, 4> motor_torques;
    for (size_t i = 0; i < 4; ++i) {
      motor_torques[i] = MotorTorque(mecanum_params_.drive_motor, motor_powers[i], wheel_omegas[i]);
    }

    // Forward acceleration kinematics: wheel torques → robot acceleration
    // This uses the forwardAcc matrix from MecanumDynamics.kt
    // acc = forwardAcc * torques
    // forwardAcc is scaled by (1/r)/sqrt(2) * [1/m, 1/m, 1/I]
    const double force_scale = (1.0 / r) / std::sqrt(2.0);
    const double acc_x = force_scale * (motor_torques[0] + motor_torques[1] + motor_torques[2] + motor_torques[3]) / mecanum_params_.mass;
    const double acc_y = force_scale * (-motor_torques[0] + motor_torques[1] - motor_torques[2] + motor_torques[3]) / mecanum_params_.mass;
    const double alpha = force_scale * (-l2 * motor_torques[0] - l2 * motor_torques[1] + l2 * motor_torques[2] + l2 * motor_torques[3]) / mecanum_params_.rot_inertia;

    // Transform acceleration from body frame to world frame
    const double acc_world_x = acc_x * c - acc_y * s;
    const double acc_world_y = acc_x * s + acc_y * c;

    // Apply acceleration by computing forces
    // F = m * a, τ = I * α
    const Eigen::Vector3d base_force(
        acc_world_x * mecanum_params_.mass,
        acc_world_y * mecanum_params_.mass,
        0.0
    );
    const Eigen::Vector3d base_torque(
        0.0,
        0.0,
        alpha * mecanum_params_.rot_inertia
    );

    // Apply spatial force to base
    std::vector<ExternallyAppliedSpatialForce<double>> spatial_forces;
    spatial_forces.push_back({
        base_body_index_,
        Eigen::Vector3d::Zero(),  // Applied at COM
        SpatialForce<double>(base_torque, base_force)
    });

    // Store wheel forces for visualization (convert torques to forces)
    for (size_t i = 0; i < 4; ++i) {
      const double f_mag = (motor_torques[i] * std::sqrt(2.0)) / r;
      Eigen::Vector3d traction_dir_world =
          Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * traction_dirs[i];
      Eigen::Vector3d force_W = f_mag * traction_dir_world;
      last_wheel_forces_W_[i] = {force_W.x(), force_W.y(), force_W.z()};
    }

    // Set wheel joint velocities for visualization
    for (size_t i = 0; i < wheel_omegas.size(); ++i) {
      const JointActuator<double> &joint_actuator =
          plant_->get_joint_actuator(actuators_[i].index);
      const Joint<double> &joint = joint_actuator.joint();
      if (joint.num_velocities() == 1) {
        velocities[joint.velocity_start()] = wheel_omegas[i];
      }
    }
    plant_->SetVelocities(&plant_context, velocities);

    // Calculate Motor Torques for NON-WHEEL actuators (intake, shooter, etc.)
    for (const auto &actuator : actuators_) {
      if (actuator.input_index >= 4) {  // Skip wheels (indices 0-3)
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
    }
    plant_->get_actuation_input_port().FixValue(&plant_context, torques);

    plant_->get_applied_spatial_force_input_port().FixValue(&plant_context,
                                                            spatial_forces);

    simulator_->AdvanceTo(context.get_time() + sub_dt);
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
