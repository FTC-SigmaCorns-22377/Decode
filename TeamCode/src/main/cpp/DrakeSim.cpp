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
    SpatialVelocity<double> V_WB =
        plant_->EvalBodySpatialVelocityInWorld(plant_context, base_body);
    Eigen::Vector3d v_world = V_WB.translational();
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    const double v_body_x = v_world.x() * c + v_world.y() * s;
    const double v_body_y = -v_world.x() * s + v_world.y() * c;
    const double omega_body = V_WB.rotational().z();
    const double l = mecanum_params_.lx + mecanum_params_.ly;
    const double r = mecanum_params_.wheel_radius;
    const std::array<double, 4> wheel_omegas = {
        (v_body_x - v_body_y - l * omega_body) / r,  // FL
        (v_body_x + v_body_y - l * omega_body) / r,  // BL
        (v_body_x - v_body_y + l * omega_body) / r,  // BR
        (v_body_x + v_body_y + l * omega_body) / r   // FR
    };

    for (size_t i = 0; i < wheel_omegas.size(); ++i) {
      const JointActuator<double> &joint_actuator =
          plant_->get_joint_actuator(actuators_[i].index);
      const Joint<double> &joint = joint_actuator.joint();
      if (joint.num_velocities() == 1) {
        velocities[joint.velocity_start()] = wheel_omegas[i];
      }
    }
    plant_->SetVelocities(&plant_context, velocities);

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
        if (actuator.input_index >= 0 && actuator.input_index < 4) {
          joint_vel = wheel_omegas[actuator.input_index];
        } else {
          joint_vel = velocities[joint.velocity_start()];
        }
      }
      torques[actuator.index] = MotorTorque(actuator.motor, command, joint_vel);
    }
    plant_->get_actuation_input_port().FixValue(&plant_context, torques);

    // 2. Apply Ground Reaction Forces for Wheels (direct torque -> force)
    std::vector<ExternallyAppliedSpatialForce<double>> spatial_forces;
    spatial_forces.reserve(4);

    for (size_t i = 0; i < wheel_names.size(); ++i) {
      const auto &wheel_body = plant_->GetBodyByName(wheel_names[i]);
      RigidTransformd X_W_Wheel =
          plant_->EvalBodyPoseInWorld(plant_context, wheel_body);

      // Only generate traction if on/near ground
      if (X_W_Wheel.translation().z() < mecanum_params_.wheel_radius * 1.2) {
        // Get torque calculated in step 1
        double T = torques[actuators_[i].index];
        double f_mag = (T * std::sqrt(2)) / mecanum_params_.wheel_radius;

        // force safety checks
        if (std::fabs(f_mag) > 200.0) {
          std::cout << "WARNING: clamping wheel force of " << f_mag
                    << std::endl;
          f_mag = std::clamp(f_mag, -200.0, 200.0);
        }
        if (std::isnan(f_mag)) {
          std::cout << "WARNING: wheel force is NaN. defaulting to 0."
                    << std::endl;
          f_mag = 0;
        }

        Eigen::Vector3d traction_dir_world =
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * traction_dirs[i];

        Eigen::Vector3d force_W = f_mag * traction_dir_world;

        // Apply to base_link at wheel center position (expressed in base frame)
        Eigen::Vector3d p_Wheel_W = X_W_Wheel.translation();
        Eigen::Vector3d p_BoBq_B = X_W_Base.inverse() * p_Wheel_W;

        spatial_forces.push_back(
            {base_body_index_, p_BoBq_B,
             SpatialForce<double>(Eigen::Vector3d::Zero(), force_W)});

        last_wheel_forces_W_[i] = {force_W.x(), force_W.y(), force_W.z()};
      }
    }

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
