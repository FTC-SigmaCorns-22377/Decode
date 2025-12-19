#include "DrakeSim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <vector>

#include <drake/geometry/shape_specification.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram_builder.h>

using namespace drake;
using namespace drake::geometry;
using namespace drake::math;
using namespace drake::multibody;
using namespace drake::systems;

namespace {
void RotateInPlace(double& x, double& y, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double rx = c * x - s * y;
    const double ry = s * x + c * y;
    x = rx;
    y = ry;
}
}  // namespace

DrakeSim::DrakeSim(const std::string& urdf_path) {
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
    plant_->RegisterCollisionGeometry(
        plant_->world_body(), X_WG, box, "ground_collision",
        CoulombFriction<double>(0.8, 0.8));
    plant_->RegisterVisualGeometry(
        plant_->world_body(), X_WG, box, "ground_visual",
        Vector4<double>(0.3, 0.3, 0.3, 1.0));

    plant_->mutable_gravity_field().set_gravity_vector(
        Eigen::Vector3d(0, 0, -9.81));

    // Motor configs derived from RobotModelConstants.
    const double bare_motor_free_speed = 617.84;
    const double bare_motor_stall_torque = 0.187;
    const MotorConfig drive_motor{
        bare_motor_free_speed / 19.2, bare_motor_stall_torque * 19.2, 12.0};
    const MotorConfig spin_motor{
        bare_motor_free_speed / 10.0, bare_motor_stall_torque * 10.0, 12.0};
    const MotorConfig flywheel_motor{
        bare_motor_free_speed / 13.7, bare_motor_stall_torque * 13.7, 12.0};

    mecanum_params_.lx = 0.2;
    mecanum_params_.ly = 0.2;
    mecanum_params_.wheel_radius = 0.048;
    mecanum_params_.mass = 15.0;
    mecanum_params_.rot_inertia = 0.5;
    mecanum_params_.drive_motor = drive_motor;
    base_z_ = mecanum_params_.wheel_radius;

    const std::vector<std::string> actuator_joint_names = {
        "fl_wheel_joint",
        "bl_wheel_joint",
        "br_wheel_joint",
        "fr_wheel_joint",
        "intake_joint",
        "spindexer_joint",
        "turret_joint",
        "flywheel_joint",
        "hood_joint"};

    const std::vector<MotorConfig> actuator_motors = {
        drive_motor,
        drive_motor,
        drive_motor,
        drive_motor,
        spin_motor,
        spin_motor,
        spin_motor,
        flywheel_motor,
        spin_motor};

    joint_state_order_ = actuator_joint_names;

    for (const auto& name : actuator_joint_names) {
        const auto& joint = plant_->GetJointByName(name);
        const std::string actuator_name = name + "_actuator";
        plant_->AddJointActuator(actuator_name, joint);
    }

    // Contact geometry overrides for intake/spindexer/flywheel/hood/transfer.
    auto add_contact = [&](const std::string& body_name,
                           const Shape& shape,
                           const std::string& geom_name,
                           const CoulombFriction<double>& friction,
                           const RigidTransformd& X_BG = RigidTransformd::Identity()) {
        const auto& body = plant_->GetBodyByName(body_name);
        plant_->RegisterCollisionGeometry(body, X_BG, shape, geom_name, friction);
    };

    add_contact("intake_link", Cylinder(0.03, 0.30), "intake_soft_contact",
                CoulombFriction<double>(1.2, 1.0));
    add_contact("spindexer_base_link", Cylinder(0.15, 0.01),
                "spindexer_contact", CoulombFriction<double>(0.7, 0.6));
    add_contact("flywheel_link", Cylinder(0.04, 0.02), "flywheel_contact",
                CoulombFriction<double>(1.3, 1.1));
    add_contact("hood_seg1", Box(0.01, 0.08, 0.04), "hood_seg1_contact",
                CoulombFriction<double>(0.4, 0.3));
    add_contact("hood_seg2", Box(0.01, 0.08, 0.04), "hood_seg2_contact",
                CoulombFriction<double>(0.4, 0.3));
    add_contact("hood_seg3", Box(0.01, 0.08, 0.04), "hood_seg3_contact",
                CoulombFriction<double>(0.4, 0.3));
    add_contact("transfer_ramp", Box(0.12, 0.08, 0.02),
                "transfer_contact", CoulombFriction<double>(0.6, 0.5));

    const ModelInstanceIndex ball_instance = plant_->AddModelInstance("balls");
    for (int i = 0; i < 10; ++i) {
        std::string name = "ball_" + std::to_string(i);
        const auto& ball_body = plant_->AddRigidBody(
            name, ball_instance, SpatialInertia<double>::SolidSphereWithMass(0.05, 0.1));
        plant_->RegisterCollisionGeometry(
            ball_body, RigidTransformd::Identity(), Sphere(0.05),
            name + "_col", CoulombFriction<double>(0.5, 0.5));
        plant_->RegisterVisualGeometry(
            ball_body, RigidTransformd::Identity(), Sphere(0.05),
            name + "_vis", Vector4<double>(1, 0.5, 0, 1));
    }

    plant_->Finalize();

    actuators_.clear();
    for (size_t i = 0; i < actuator_joint_names.size(); ++i) {
        const std::string actuator_name = actuator_joint_names[i] + "_actuator";
        const auto& actuator = plant_->GetJointActuatorByName(actuator_name);
        actuators_.push_back(
            {actuator.index(), static_cast<int>(i), actuator_motors[i]});
    }

    diagram_ = builder.Build();
    simulator_owner_ = std::make_unique<Simulator<double>>(*diagram_);
    simulator_ = simulator_owner_.get();
    simulator_->set_target_realtime_rate(1.0);
    simulator_->Initialize();

    auto& context = simulator_->get_mutable_context();
    auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);
    for (int i = 0; i < 10; ++i) {
        std::string name = "ball_" + std::to_string(i);
        const auto& body = plant_->GetBodyByName(name);
        plant_->SetFreeBodyPose(
            &plant_context, body,
            RigidTransformd(Eigen::Vector3d(0, 0, -100 - i)));
    }

    SetBasePoseAndVelocity(mecanum_state_);
}

DrakeSim::~DrakeSim() {}

double DrakeSim::MotorTorque(const MotorConfig& motor,
                             double power,
                             double omega) const {
    if (motor.free_speed <= 1e-6) {
        return 0.0;
    }
    const double clamped = std::clamp(power, -1.0, 1.0);
    const double voltage_scale = motor.v_ref > 0.0 ? (12.0 / motor.v_ref) : 1.0;
    return motor.stall_torque * (clamped * voltage_scale - omega / motor.free_speed);
}

DrakeSim::MecanumState DrakeSim::IntegrateMecanum(
    double dt,
    const std::array<double, 4>& inputs,
    const MecanumState& state) const {
    const double l = mecanum_params_.lx + mecanum_params_.ly;
    const double l2 =
        std::sqrt(mecanum_params_.lx * mecanum_params_.lx +
                  mecanum_params_.ly * mecanum_params_.ly);

    auto dx = [&](const std::array<double, 6>& x) {
        const Pose2d pos{x[3], x[4], x[5]};
        Pose2d vel{x[0], x[1], x[2]};

        RotateInPlace(vel.x, vel.y, -pos.theta);

        const double w_fl = (vel.x - vel.y - l * vel.theta) /
                            mecanum_params_.wheel_radius;
        const double w_bl = (vel.x + vel.y - l * vel.theta) /
                            mecanum_params_.wheel_radius;
        const double w_br = (vel.x - vel.y + l * vel.theta) /
                            mecanum_params_.wheel_radius;
        const double w_fr = (vel.x + vel.y + l * vel.theta) /
                            mecanum_params_.wheel_radius;

        const double t_fl = MotorTorque(mecanum_params_.drive_motor, inputs[0], w_fl);
        const double t_bl = MotorTorque(mecanum_params_.drive_motor, inputs[1], w_bl);
        const double t_br = MotorTorque(mecanum_params_.drive_motor, inputs[2], w_br);
        const double t_fr = MotorTorque(mecanum_params_.drive_motor, inputs[3], w_fr);

        const double scale = (1.0 / mecanum_params_.wheel_radius) /
                             std::sqrt(2.0);
        double ax = scale * (1.0 / mecanum_params_.mass) *
                    (t_fl + t_bl + t_br + t_fr);
        double ay = scale * (1.0 / mecanum_params_.mass) *
                    (-t_fl + t_bl - t_br + t_fr);
        const double alpha =
            scale * (1.0 / mecanum_params_.rot_inertia) *
            (-l2 * t_fl - l2 * t_bl + l2 * t_br + l2 * t_fr);

        RotateInPlace(ax, ay, pos.theta);

        return std::array<double, 6>{
            ax, ay, alpha, x[0], x[1], x[2]};
    };

    std::array<double, 6> x0{
        state.vel.x, state.vel.y, state.vel.theta,
        state.pos.x, state.pos.y, state.pos.theta};

    const auto k1 = dx(x0);
    std::array<double, 6> x1{};
    for (int i = 0; i < 6; ++i) {
        x1[i] = x0[i] + 0.5 * dt * k1[i];
    }
    const auto k2 = dx(x1);
    std::array<double, 6> x2{};
    for (int i = 0; i < 6; ++i) {
        x2[i] = x0[i] + 0.5 * dt * k2[i];
    }
    const auto k3 = dx(x2);
    std::array<double, 6> x3{};
    for (int i = 0; i < 6; ++i) {
        x3[i] = x0[i] + dt * k3[i];
    }
    const auto k4 = dx(x3);

    std::array<double, 6> xf{};
    for (int i = 0; i < 6; ++i) {
        xf[i] = x0[i] + (dt / 6.0) *
                          (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }

    MecanumState out;
    out.vel = {xf[0], xf[1], xf[2]};
    out.pos = {xf[3], xf[4], xf[5]};
    return out;
}

void DrakeSim::SetBasePoseAndVelocity(const MecanumState& state) {
    auto& context = simulator_->get_mutable_context();
    auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);
    const auto& base_body = plant_->get_body(base_body_index_);

    const RollPitchYaw<double> rpy(0.0, 0.0, state.pos.theta);
    const RigidTransformd X_WB(
        rpy.ToRotationMatrix(), Eigen::Vector3d(state.pos.x, state.pos.y, base_z_));
    plant_->SetFreeBodyPose(&plant_context, base_body, X_WB);

    const SpatialVelocity<double> V_WB(
        Eigen::Vector3d(0, 0, state.vel.theta),
        Eigen::Vector3d(state.vel.x, state.vel.y, 0.0));
    plant_->SetFreeBodySpatialVelocity(&plant_context, base_body, V_WB);
}

void DrakeSim::Step(double dt, const std::vector<double>& inputs) {
    if (dt <= 0.0) {
        return;
    }

    std::array<double, 4> drive_inputs{0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 4 && i < static_cast<int>(inputs.size()); ++i) {
        drive_inputs[i] = inputs[i];
    }
    mecanum_state_ = IntegrateMecanum(dt, drive_inputs, mecanum_state_);
    SetBasePoseAndVelocity(mecanum_state_);

    auto& context = simulator_->get_mutable_context();
    auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

    Eigen::VectorXd velocities = plant_->GetVelocities(plant_context);
    Eigen::VectorXd torques = Eigen::VectorXd::Zero(plant_->num_actuators());

    for (const auto& actuator : actuators_) {
        const double command =
            actuator.input_index < static_cast<int>(inputs.size())
                ? inputs[actuator.input_index]
                : 0.0;
        const JointActuator<double>& joint_actuator =
            plant_->get_joint_actuator(actuator.index);
        const Joint<double>& joint = joint_actuator.joint();
        double joint_vel = 0.0;
        if (joint.num_velocities() == 1) {
            joint_vel = velocities[joint.velocity_start()];
        }
        torques[actuator.index] = MotorTorque(actuator.motor, command, joint_vel);
    }

    plant_->get_actuation_input_port().FixValue(&plant_context, torques);
    simulator_->AdvanceTo(context.get_time() + dt);
    SetBasePoseAndVelocity(mecanum_state_);
}

std::vector<double> DrakeSim::GetState() {
    auto& context = simulator_->get_mutable_context();
    auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

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

    for (const auto& joint_name : joint_state_order_) {
        const auto& joint = plant_->GetJointByName(joint_name);
        if (joint.num_positions() == 1) {
            state.push_back(q[joint.position_start()]);
        } else {
            state.push_back(0.0);
        }
    }

    for (const auto& joint_name : joint_state_order_) {
        const auto& joint = plant_->GetJointByName(joint_name);
        if (joint.num_velocities() == 1) {
            state.push_back(v[joint.velocity_start()]);
        } else {
            state.push_back(0.0);
        }
    }

    return state;
}

void DrakeSim::SpawnBall(double x, double y, double z) {
    if (ball_count_ >= 10) {
        std::cerr << "Ball pool exhausted!" << std::endl;
        return;
    }

    auto& context = simulator_->get_mutable_context();
    auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_, &context);

    std::string name = "ball_" + std::to_string(ball_count_);
    const auto& body = plant_->GetBodyByName(name);

    plant_->SetFreeBodyPose(
        &plant_context, body, RigidTransformd(Eigen::Vector3d(x, y, z)));
    plant_->SetFreeBodySpatialVelocity(
        &plant_context, body, SpatialVelocity<double>::Zero());

    ball_count_++;
}
