#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>

#include <vector>
#include <mutex>

// Object layers
namespace Layers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING     = 1;
    static constexpr JPH::ObjectLayer SENSOR     = 2;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 3;
}

// Broad phase layers
namespace BroadPhaseLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr uint32_t NUM_LAYERS = 2;
}

// BroadPhaseLayerInterface implementation
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    uint32_t GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        switch (inLayer) {
            case Layers::NON_MOVING: return BroadPhaseLayers::NON_MOVING;
            default:                 return BroadPhaseLayers::MOVING;
        }
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer inLayer) const override {
        switch ((JPH::BroadPhaseLayer::Type)inLayer) {
            case 0: return "NON_MOVING";
            case 1: return "MOVING";
            default: return "UNKNOWN";
        }
    }
#endif
};

// ObjectVsBroadPhaseLayerFilter
class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        if (inLayer1 == Layers::NON_MOVING)
            return inLayer2 == BroadPhaseLayers::MOVING;
        return true;
    }
};

// ObjectLayerPairFilter
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override {
        if (inLayer1 == Layers::NON_MOVING && inLayer2 == Layers::NON_MOVING)
            return false;
        // Sensors only collide with moving objects
        if (inLayer1 == Layers::SENSOR || inLayer2 == Layers::SENSOR)
            return inLayer1 == Layers::MOVING || inLayer2 == Layers::MOVING;
        return true;
    }
};

struct BallInfo {
    JPH::BodyID bodyId;
    int color; // 0 = green, 1 = purple
};

struct IntakeInfo {
    JPH::BodyID rollerBodyId;
    float rollerOmega = 0.0f;  // set from Kotlin each frame
    float hingeAngle = -0.305f; // simulated hinge angle (rad), starts at rest
};

struct GoalInfo {
    // Goal triangle walls (3-sided structure in field corner)
    JPH::BodyID sideWallAId;    // along -X field wall (back wall)
    JPH::BodyID sideWallBId;    // along ±Z field wall (side wall)
    JPH::BodyID frontWallId;    // diagonal front face at lip height (hypotenuse)

    // Classifier ramp
    JPH::BodyID rampFloorId;    // sloped ramp surface
    JPH::BodyID rampRailAId;    // ramp side rail
    JPH::BodyID rampRailBId;    // ramp side rail
    JPH::BodyID gateId;         // kinematic gate (push to open)
    JPH::BodyID rampWallId;     // trapezoidal wall below ramp

    // Rectangular extension connecting triangle to ramp
    JPH::BodyID rectInnerWallId;  // along ±Z wall covering ramp section
    JPH::BodyID rectPerpWallId;   // perpendicular wall at triangle front, into the field
    JPH::BodyID cornerWallId;     // connects wall B (at wall) to wall A (offset) at corner

    // Lever (balance platform near goal opening)
    JPH::BodyID leverId;

    float zSign = 0.0f;         // +1 for red corner (+Z), -1 for blue corner (-Z)
    int scoredBalls = 0;
    float gateOpenAmount = 0.0f; // 0 = closed, up to GATE_TRAVEL = fully open
    float leverAngle = 0.0f;    // tilt in radians
};

// Forward declaration
class JoltWorld;

class IntakeContactListener : public JPH::ContactListener {
public:
    IntakeContactListener(JoltWorld* world) : world_(world) {}

    void OnContactAdded(const JPH::Body& body1, const JPH::Body& body2,
                        const JPH::ContactManifold& manifold,
                        JPH::ContactSettings& settings) override;

    void OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2,
                            const JPH::ContactManifold& manifold,
                            JPH::ContactSettings& settings) override;

private:
    void applyIntakeEffect(const JPH::Body& body1, const JPH::Body& body2,
                           const JPH::ContactManifold& manifold,
                           JPH::ContactSettings& settings);
    JoltWorld* world_;
};

class JoltWorld {
    friend class IntakeContactListener;

public:
    JoltWorld();
    ~JoltWorld();

    void step(float dt);
    void applyRobotForce(float fx, float fy, float tz);
    void setRobotPose(float x, float y, float theta);
    void getRobotState(float out[6]); // [x, y, theta, vx, vy, omega]

    int  spawnBall(float x, float y, float z, float vx, float vy, float vz, int color);
    void removeBall(int index);
    int  getBallCount() const;
    void getBallStates(float* out) const;  // [x,y,z] * count
    void getBallColors(int* out) const;

    // Intake API
    void setIntakeRollerOmega(float omega);
    void getIntakeState(float* out) const;  // [hingeAngle, rollerOmega]
    int  getPendingPickups(int* out, int max);

    // Goal API
    // [redScore, blueScore, redGateOpen, blueGateOpen, redLeverAngle, blueLeverAngle]
    void getGoalStates(float* out) const;

private:
    void buildField();
    void buildGoals();
    void buildGoalSide(GoalInfo& goal, float zSign);
    void createRobot();
    void createIntakeRoller();
    void updateIntake(float dt);
    void checkGoalScoring();
    void updateGates(float dt);
    void updateLevers(float dt);
    JPH::BodyID createStaticBox(JPH::Vec3 halfExtent, JPH::Vec3 position, float friction);
    JPH::BodyID createStaticBoxRotated(JPH::Vec3 halfExtent, JPH::Vec3 position, JPH::Quat rotation, float friction);

    // Jolt infrastructure
    JPH::TempAllocatorImpl*     tempAllocator_ = nullptr;
    JPH::JobSystemThreadPool*   jobSystem_     = nullptr;
    JPH::PhysicsSystem*         physicsSystem_ = nullptr;

    // Layer interfaces
    BPLayerInterfaceImpl                bpLayerInterface_;
    ObjectVsBroadPhaseLayerFilterImpl   objectVsBroadPhase_;
    ObjectLayerPairFilterImpl           objectLayerPair_;

    // Contact listener
    IntakeContactListener* contactListener_ = nullptr;

    // Collision group filter (prevents robot-roller collision)
    JPH::Ref<JPH::GroupFilterTable> robotRollerGroupFilter_;

    // Bodies
    JPH::BodyID robotBodyId_;
    IntakeInfo intake_;
    std::vector<BallInfo> balls_;

    // Pending pickups (ball indices to be picked up by Kotlin)
    std::vector<int> pendingPickups_;

    // Goals
    GoalInfo redGoal_;
    GoalInfo blueGoal_;

    // Field body IDs (ground + 4 walls) for cleanup
    std::vector<JPH::BodyID> fieldBodyIds_;

    // Queued forces (applied each step)
    float queuedFx_  = 0.0f;
    float queuedFy_  = 0.0f;
    float queuedTz_  = 0.0f;

    // Field constants
    static constexpr float FIELD_SIZE     = 3.6576f; // 12 feet in meters
    static constexpr float WALL_HEIGHT    = 0.312f;
    static constexpr float WALL_THICKNESS = 0.02f;
    static constexpr float WALL_FRICTION  = 0.3f;

    static constexpr float ROBOT_WIDTH    = 0.4f;
    static constexpr float ROBOT_LENGTH   = 0.4f;
    static constexpr float ROBOT_HEIGHT   = 0.15f;
    static constexpr float ROBOT_MASS     = 15.0f;

    static constexpr float BALL_RADIUS    = 0.0635f;
    static constexpr float BALL_MASS      = 0.0748f; // 0.165 lbs
    static constexpr float BALL_FRICTION  = 0.8f;

    // Intake sensor (legacy, kept for reference)
    static constexpr float INTAKE_OFFSET  = 0.2f; // offset in front of robot center = ROBOT_LENGTH/2
    static constexpr float INTAKE_WIDTH   = 0.3f;
    static constexpr float INTAKE_DEPTH   = 0.1f;
    static constexpr float INTAKE_HEIGHT  = 0.1f;

    // Intake roller physics
    static constexpr float INTAKE_BAR_LENGTH      = 0.127f;   // 5" bar length
    static constexpr float INTAKE_ROLLER_RADIUS    = 0.0254f;  // 2" diameter / 2
    static constexpr float INTAKE_MASS             = 0.227f;   // ~0.5 lb
    static constexpr float INTAKE_ROLLER_FRICTION  = 0.9f;     // rubber wheels
    static constexpr float INTAKE_REST_ANGLE       = -0.305f;  // ~-17.5 deg (roller hangs below chassis top)
    static constexpr float INTAKE_MIN_ANGLE        = -0.7f;    // gravity rest lower limit
    static constexpr float INTAKE_MAX_ANGLE        = 1.22f;    // ~70 deg, max ball push-up angle
    static constexpr float INTAKE_HINGE_FRICTION   = 0.01f;    // Nm pivot damping
    static constexpr float INTAKE_OMEGA_THRESHOLD  = 10.0f;    // min roller rad/s for ball pickup

    // Goal structure (from DECODE game manual)
    // 3-sided triangular structure in field corner, open top
    static constexpr float HALF_FIELD         = FIELD_SIZE / 2.0f; // 1.8288m
    static constexpr float GOAL_LEG           = 0.6858f;  // 27" each leg of the right triangle
    static constexpr float GOAL_WALL_THICK    = 0.03f;    // physics thickness (thicker than real 1cm for reliable collision)
    static constexpr float GOAL_LIP_HEIGHT    = 0.9843f;  // 38.75" top lip from tiles
    static constexpr float GOAL_TOTAL_HEIGHT  = 1.3716f;  // 54" max height at corner
    static constexpr float GOAL_BACKBOARD_EXT = 0.381f;   // 15" backboard above lip

    // Classifier ramp (aluminum extrusion channel, holds up to 9 artifacts)
    static constexpr float CRAMP_LENGTH       = 1.00f;    // ramp channel length
    static constexpr float CRAMP_WIDTH        = 0.16f;    // channel width (> ball diameter)
    static constexpr float CRAMP_START_H      = 0.49f;    // input end height (~halfway up goal)
    static constexpr float CRAMP_END_H        = 0.127f;   // output end height (~ball diameter)
    static constexpr float CRAMP_MID_H        = (CRAMP_START_H + CRAMP_END_H) / 2.0f;
    static constexpr float CRAMP_RAIL_HEIGHT  = 0.10f;    // side rail height
    static constexpr float CRAMP_RAIL_THICK   = 0.01f;    // side rail thickness
    static constexpr float CRAMP_WALL_THICK   = 0.01f;    // trapezoid wall below ramp

    // Gate (push-to-open, gravity-closed)
    static constexpr float GATE_CLOSED_H      = 0.14f;    // 5.5" contact area when closed
    static constexpr float GATE_OPEN_H        = 0.076f;   // 3" when open
    static constexpr float GATE_TRAVEL        = 0.15f;    // enough clearance for ball diameter
    static constexpr float GATE_WIDTH         = 0.16f;    // matches ramp width
    static constexpr float GATE_THICK         = 0.02f;

    // Lever (gate actuator arm at ramp output end)
    static constexpr float LEVER_LENGTH       = 0.25f;   // arm extension toward field (~10")
    static constexpr float LEVER_WIDTH        = 0.04f;   // narrow handle width
    static constexpr float LEVER_THICKNESS    = 0.02f;
};
