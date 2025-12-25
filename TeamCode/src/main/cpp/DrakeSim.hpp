#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

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

    // returns: [x, y, z, yaw, vx, vy, omega, ... joint positions ..., ... joint velocities ...,
    //           wheel forces (FL, BL, BR, FR; each as x,y,z), ... ball positions ...]
    std::vector<double> GetState();
    
    // Spawns a ball at the given position
    void SpawnBall(double x, double y, double z);
    void SetPosition(double x, double y, double yaw);

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
        double direction = 1.0;
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
    struct RobotState {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
        double time_sec = 0.0;
        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
    };
    struct BallState {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
    };

    std::vector<ActuatorInfo> actuators_;
    std::vector<std::string> joint_state_order_;
    MecanumParams mecanum_params_;
    MecanumState mecanum_state_;
    drake::multibody::BodyIndex base_body_index_;
    drake::multibody::ModelInstanceIndex ball_instance_;
    std::vector<drake::multibody::BodyIndex> ball_bodies_;
    std::vector<BallState> balls_;
    std::string urdf_path_;
    std::array<std::array<double, 3>, 4> last_wheel_forces_W_{};

    double MotorTorque(const MotorConfig& motor, double power, double omega) const;
    void BuildSimulator(const RobotState* state);
    RobotState CaptureRobotState() const;
    void RefreshBallStates();
};
#else
class DrakeSim {
public:
    DrakeSim(const std::string& urdf_path);
    ~DrakeSim();

    void Step(double dt, const std::vector<double>& inputs);
    std::vector<double> GetState();
    void SpawnBall(double x, double y, double z);
    void SetPosition(double x, double y, double yaw);
};
#endif
