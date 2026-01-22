#include "DrakeSim.hpp"

#include <vector>

#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>

using namespace drake;
using namespace drake::multibody;
using namespace drake::math;

std::vector<double> DrakeSim::GetState() {
  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

  Eigen::VectorXd q = plant_->GetPositions(plant_context);
  Eigen::VectorXd v = plant_->GetVelocities(plant_context);

  std::vector<double> state;
  state.reserve(7 + joint_state_order_.size() * 2 +
                last_wheel_forces_W_.size() * 3 + ball_bodies_.size() * 3);

  state.push_back(mecanum_state_.pos.x);
  state.push_back(mecanum_state_.pos.y);
  const auto &base_body = plant_->get_body(base_body_index_);
  const auto &X_WB = plant_->EvalBodyPoseInWorld(plant_context, base_body);
  state.push_back(X_WB.translation().z());
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
  
  // TODO: support joints with >1 velocities
  for (const auto &joint_name : joint_state_order_) {
    const auto &joint = plant_->GetJointByName(joint_name);
    if (joint.num_velocities() == 1) {
      state.push_back(v[joint.velocity_start()]);
    } else {
      state.push_back(0.0);
    }
  }

  for (const auto &force : last_wheel_forces_W_) {
    state.push_back(force[0]);
    state.push_back(force[1]);
    state.push_back(force[2]);
  }

  // Add ball positions (x, y, z for each ball)
  for (const auto &ball_index : ball_bodies_) {
    const auto &body = plant_->get_body(ball_index);
    const auto &X_WB = plant_->EvalBodyPoseInWorld(plant_context, body);
    const auto &p_WB = X_WB.translation();
    state.push_back(p_WB.x());
    state.push_back(p_WB.y());
    state.push_back(p_WB.z());
  }

  return state;
}

void DrakeSim::SpawnBall(double x, double y, double z) {
  RefreshBallStates();
  RobotState state = CaptureRobotState();

  BallState ball;
  ball.x = x;
  ball.y = y;
  ball.z = z;
  balls_.push_back(ball);

  BuildSimulator(&state);
}

void DrakeSim::SpawnBallWithVelocity(double x, double y, double z,
                                      double vx, double vy, double vz) {
  RefreshBallStates();
  RobotState state = CaptureRobotState();

  BallState ball;
  ball.x = x;
  ball.y = y;
  ball.z = z;
  ball.vx = vx;
  ball.vy = vy;
  ball.vz = vz;
  balls_.push_back(ball);

  BuildSimulator(&state);
}

void DrakeSim::RemoveBall(int index) {
  if (index < 0 || index >= static_cast<int>(balls_.size())) {
    return;
  }

  RefreshBallStates();
  RobotState state = CaptureRobotState();

  balls_.erase(balls_.begin() + index);

  BuildSimulator(&state);
}

void DrakeSim::SetPosition(double x, double y, double yaw) {
  auto &context = simulator_->get_mutable_context();
  auto &plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);
  const auto &base_body = plant_->get_body(base_body_index_);
  const auto &X_WB_current = plant_->EvalBodyPoseInWorld(plant_context, base_body);

  RigidTransformd X_WB(RollPitchYaw<double>(0.0, 0.0, yaw),
                       Eigen::Vector3d(x, y, X_WB_current.translation().z()));
  plant_->SetFreeBodyPose(&plant_context, base_body, X_WB);
  plant_->SetFreeBodySpatialVelocity(&plant_context, base_body,
                                     SpatialVelocity<double>::Zero());

  mecanum_state_.pos.x = x;
  mecanum_state_.pos.y = y;
  mecanum_state_.pos.theta = yaw;
  mecanum_state_.vel.x = 0.0;
  mecanum_state_.vel.y = 0.0;
  mecanum_state_.vel.theta = 0.0;
}

void DrakeSim::SetMecanumParameters(const std::vector<double>& params) {
  if (params.size() != 7) {
    return;
  }

  // params: [freeSpeed, stallTorque, lx, ly, wheelRadius, weight, rotInertia]
  mecanum_params_.drive_motor.free_speed = params[0];
  mecanum_params_.drive_motor.stall_torque = params[1];
  mecanum_params_.lx = params[2];
  mecanum_params_.ly = params[3];
  mecanum_params_.wheel_radius = params[4];
  mecanum_params_.mass = params[5];
  mecanum_params_.rot_inertia = params[6];
}
