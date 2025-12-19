#pragma once

#include <memory>
#include <vector>
#include <string>
#include <array>

#if defined(USE_DRAKE) && USE_DRAKE
#include <drake/systems/framework/diagram.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/scene_graph.h>
#endif

#if defined(USE_DRAKE) && USE_DRAKE
class DrakeSim {
public:
    DrakeSim(const std::string& urdf_path);
    ~DrakeSim();

    // inputs: [fl, bl, br, fr, intake, spindexer, turret, flywheel, hood] (power -1..1)
    void Step(double dt, const std::vector<double>& inputs);

    // returns: [x, y, yaw, vx, vy, omega, ... joint positions ..., ... joint velocities ...]
    std::vector<double> GetState();
    
    // Spawns a ball at the given position
    void SpawnBall(double x, double y, double z);

private:
    std::unique_ptr<drake::systems::Diagram<double>> diagram_;
    drake::systems::Simulator<double>* simulator_; 
    std::unique_ptr<drake::systems::Simulator<double>> simulator_owner_;
    drake::multibody::MultibodyPlant<double>* plant_;
    drake::geometry::SceneGraph<double>* scene_graph_;
    
    // Motor Parameters (Simple DC Motor Model)
    struct MotorConfig {
        double free_speed = 0.0;   // rad/s
        double stall_torque = 0.0; // Nm
        double v_ref = 12.0;
    };
    struct ActuatorInfo {
        drake::multibody::JointActuatorIndex index;
        int input_index = 0;
        MotorConfig motor;
    };
    struct Pose2d {
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
    };
    struct MecanumState {
        Pose2d pos;
        Pose2d vel;
    };
    struct MecanumParams {
        double lx = 0.2;
        double ly = 0.2;
        double wheel_radius = 0.048;
        double mass = 15.0;
        double rot_inertia = 0.5;
        MotorConfig drive_motor;
    };

    std::vector<ActuatorInfo> actuators_;
    std::vector<std::string> joint_state_order_;
    MecanumParams mecanum_params_;
    MecanumState mecanum_state_;
    drake::multibody::BodyIndex base_body_index_;
    double base_z_ = 0.05;

    int ball_count_ = 0;

    double MotorTorque(const MotorConfig& motor, double power, double omega) const;
    MecanumState IntegrateMecanum(double dt,
                                  const std::array<double, 4>& inputs,
                                  const MecanumState& state) const;
    void SetBasePoseAndVelocity(const MecanumState& state);
};
#else
class DrakeSim {
public:
    DrakeSim(const std::string& urdf_path);
    ~DrakeSim();

    void Step(double dt, const std::vector<double>& inputs);
    std::vector<double> GetState();
    void SpawnBall(double x, double y, double z);
};
#endif
