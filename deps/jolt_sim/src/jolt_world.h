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
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>

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
#else
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return ""; }
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

class JoltWorld {
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
    int  getIntakeOverlaps(int* out, int max);

private:
    void buildField();
    void createRobot();
    void createIntakeSensor();
    JPH::BodyID createStaticBox(JPH::Vec3 halfExtent, JPH::Vec3 position, float friction);

    // Jolt infrastructure
    JPH::TempAllocatorImpl*     tempAllocator_ = nullptr;
    JPH::JobSystemThreadPool*   jobSystem_     = nullptr;
    JPH::PhysicsSystem*         physicsSystem_ = nullptr;

    // Layer interfaces
    BPLayerInterfaceImpl                bpLayerInterface_;
    ObjectVsBroadPhaseLayerFilterImpl   objectVsBroadPhase_;
    ObjectLayerPairFilterImpl           objectLayerPair_;

    // Bodies
    JPH::BodyID robotBodyId_;
    JPH::BodyID intakeSensorId_;
    std::vector<BallInfo> balls_;

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
    static constexpr float BALL_MASS      = 0.5f;
    static constexpr float BALL_FRICTION  = 0.8f;

    static constexpr float INTAKE_OFFSET  = 0.2f; // offset in front of robot center
    static constexpr float INTAKE_WIDTH   = 0.3f;
    static constexpr float INTAKE_DEPTH   = 0.1f;
    static constexpr float INTAKE_HEIGHT  = 0.1f;
};
