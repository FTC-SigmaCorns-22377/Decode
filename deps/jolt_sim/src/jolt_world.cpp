#include "jolt_world.h"

#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/CollideShape.h>

#include <cstring>
#include <cstdarg>
#include <cmath>
#include <algorithm>
#include <iostream>

// Jolt requires a trace function and assert failure handler
static void TraceImpl(const char* inFmt, ...) {
    va_list args;
    va_start(args, inFmt);
    vfprintf(stderr, inFmt, args);
    fprintf(stderr, "\n");
    va_end(args);
}

#ifdef JPH_ENABLE_ASSERTS
static bool AssertFailedImpl(const char* inExpression, const char* inMessage,
                             const char* inFile, uint32_t inLine) {
    fprintf(stderr, "Jolt assert: %s:%u: (%s) %s\n",
            inFile, inLine, inExpression, inMessage ? inMessage : "");
    return true; // break into debugger
}
#endif

// Global init guard
static bool g_joltInitialized = false;

static void ensureJoltInit() {
    if (g_joltInitialized) return;
    JPH::RegisterDefaultAllocator();
    JPH::Trace = TraceImpl;
#ifdef JPH_ENABLE_ASSERTS
    JPH::AssertFailed = AssertFailedImpl;
#endif
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
    g_joltInitialized = true;
}

JoltWorld::JoltWorld() {
    ensureJoltInit();

    tempAllocator_ = new JPH::TempAllocatorImpl(10 * 1024 * 1024); // 10MB
    jobSystem_ = new JPH::JobSystemThreadPool(
        JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 1 // single thread for determinism
    );

    const uint32_t maxBodies       = 256;
    const uint32_t numBodyMutexes  = 0; // auto
    const uint32_t maxBodyPairs    = 256;
    const uint32_t maxContactConstraints = 256;

    physicsSystem_ = new JPH::PhysicsSystem();
    physicsSystem_->Init(maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints,
                         bpLayerInterface_, objectVsBroadPhase_, objectLayerPair_);

    buildField();
    createRobot();
    createIntakeSensor();
}

JoltWorld::~JoltWorld() {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // Remove balls
    for (auto& ball : balls_) {
        bodyInterface.RemoveBody(ball.bodyId);
        bodyInterface.DestroyBody(ball.bodyId);
    }
    balls_.clear();

    // Remove intake sensor
    bodyInterface.RemoveBody(intakeSensorId_);
    bodyInterface.DestroyBody(intakeSensorId_);

    // Remove robot
    bodyInterface.RemoveBody(robotBodyId_);
    bodyInterface.DestroyBody(robotBodyId_);

    // Field walls and ground are static - clean up too
    // (PhysicsSystem destructor handles remaining bodies)

    delete physicsSystem_;
    delete jobSystem_;
    delete tempAllocator_;
}

JPH::BodyID JoltWorld::createStaticBox(JPH::Vec3 halfExtent, JPH::Vec3 position, float friction) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // Jolt adds convexRadius on top of half-extents, so shrink half-extents to compensate
    constexpr float cr = 0.005f;
    JPH::Vec3 adjusted = halfExtent - JPH::Vec3(cr, cr, cr);
    // Clamp to avoid negative
    adjusted = JPH::Vec3::sMax(adjusted, JPH::Vec3::sZero());
    JPH::BoxShapeSettings shapeSettings(adjusted, cr);
    auto shapeResult = shapeSettings.Create();

    JPH::BodyCreationSettings bodySettings(
        shapeResult.Get(), position, JPH::Quat::sIdentity(),
        JPH::EMotionType::Static, Layers::NON_MOVING
    );
    bodySettings.mFriction = friction;

    JPH::BodyID id = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::DontActivate);
    return id;
}

void JoltWorld::buildField() {
    float halfField = FIELD_SIZE / 2.0f;
    float halfWallH = WALL_HEIGHT / 2.0f;
    float halfThick = WALL_THICKNESS / 2.0f;

    // Ground plane
    createStaticBox(
        JPH::Vec3(halfField, 0.01f, halfField),
        JPH::Vec3(0, -0.01f, 0),
        0.5f
    );

    // Walls: +X, -X, +Z, -Z (field is in XZ plane, Y is up)
    // +X wall
    createStaticBox(
        JPH::Vec3(halfThick, halfWallH, halfField),
        JPH::Vec3(halfField + halfThick, halfWallH, 0),
        WALL_FRICTION
    );
    // -X wall
    createStaticBox(
        JPH::Vec3(halfThick, halfWallH, halfField),
        JPH::Vec3(-halfField - halfThick, halfWallH, 0),
        WALL_FRICTION
    );
    // +Z wall
    createStaticBox(
        JPH::Vec3(halfField, halfWallH, halfThick),
        JPH::Vec3(0, halfWallH, halfField + halfThick),
        WALL_FRICTION
    );
    // -Z wall
    createStaticBox(
        JPH::Vec3(halfField, halfWallH, halfThick),
        JPH::Vec3(0, halfWallH, -halfField - halfThick),
        WALL_FRICTION
    );
}

void JoltWorld::createRobot() {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    constexpr float cr = 0.005f;
    JPH::Vec3 robotHalf(ROBOT_WIDTH / 2.0f - cr, ROBOT_HEIGHT / 2.0f - cr, ROBOT_LENGTH / 2.0f - cr);
    JPH::BoxShapeSettings shapeSettings(robotHalf, cr);
    auto shapeResult = shapeSettings.Create();

    JPH::BodyCreationSettings bodySettings(
        shapeResult.Get(),
        JPH::Vec3(0, ROBOT_HEIGHT / 2.0f, 0), // start at origin, on ground
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Dynamic,
        Layers::MOVING
    );

    bodySettings.mFriction = 0.5f;
    bodySettings.mLinearDamping = 0.0f;  // we apply our own drag
    bodySettings.mAngularDamping = 0.0f;
    bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    bodySettings.mMassPropertiesOverride.mMass = ROBOT_MASS;

    // Constrain to 2D: allow X,Z translation and Y rotation only
    bodySettings.mAllowedDOFs = JPH::EAllowedDOFs::TranslationX |
                                JPH::EAllowedDOFs::TranslationZ |
                                JPH::EAllowedDOFs::RotationY;

    robotBodyId_ = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
}

void JoltWorld::createIntakeSensor() {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    constexpr float cr = 0.005f;
    JPH::Vec3 intakeHalf(INTAKE_WIDTH / 2.0f - cr, INTAKE_HEIGHT / 2.0f - cr, INTAKE_DEPTH / 2.0f - cr);
    JPH::BoxShapeSettings shapeSettings(intakeHalf, cr);
    auto shapeResult = shapeSettings.Create();

    JPH::BodyCreationSettings bodySettings(
        shapeResult.Get(),
        JPH::Vec3(0, ROBOT_HEIGHT / 2.0f, INTAKE_OFFSET), // offset in front
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Kinematic,
        Layers::SENSOR
    );
    bodySettings.mIsSensor = true;

    intakeSensorId_ = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
}

void JoltWorld::step(float dt) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // Apply queued force and torque to robot
    if (queuedFx_ != 0.0f || queuedFy_ != 0.0f) {
        // Our sim: X=forward in field, Y=left in field
        // Jolt: X=X, Z=forward (we map sim Y -> Jolt -Z... actually let's keep it simple)
        // Convention: sim (x,y) maps to Jolt (x, z), sim theta maps to rotation about Y
        bodyInterface.AddForce(robotBodyId_, JPH::Vec3(queuedFx_, 0, queuedFy_));
    }
    if (queuedTz_ != 0.0f) {
        bodyInterface.AddTorque(robotBodyId_, JPH::Vec3(0, queuedTz_, 0));
    }
    queuedFx_ = queuedFy_ = queuedTz_ = 0.0f;

    // Update intake sensor position to match robot
    {
        JPH::Vec3 robotPos = bodyInterface.GetPosition(robotBodyId_);
        JPH::Quat robotRot = bodyInterface.GetRotation(robotBodyId_);

        // Offset the sensor in the robot's local forward direction (Z in Jolt)
        JPH::Vec3 localOffset(0, 0, INTAKE_OFFSET);
        JPH::Vec3 worldOffset = robotRot * localOffset;
        JPH::Vec3 sensorPos = robotPos + worldOffset;

        bodyInterface.MoveKinematic(intakeSensorId_, sensorPos, robotRot, dt);
    }

    // Step physics
    physicsSystem_->Update(dt, 1, tempAllocator_, jobSystem_);
}

void JoltWorld::applyRobotForce(float fx, float fy, float tz) {
    queuedFx_ = fx;
    queuedFy_ = fy;
    queuedTz_ = tz;
}

void JoltWorld::setRobotPose(float x, float y, float theta) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // sim (x,y,theta) -> Jolt (x, z, rotY)
    JPH::Vec3 pos(x, ROBOT_HEIGHT / 2.0f, y);
    JPH::Quat rot = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), theta);

    bodyInterface.SetPositionAndRotation(robotBodyId_, pos, rot, JPH::EActivation::Activate);
    bodyInterface.SetLinearVelocity(robotBodyId_, JPH::Vec3::sZero());
    bodyInterface.SetAngularVelocity(robotBodyId_, JPH::Vec3::sZero());
}

void JoltWorld::getRobotState(float out[6]) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    JPH::Vec3 pos = bodyInterface.GetPosition(robotBodyId_);
    JPH::Vec3 vel = bodyInterface.GetLinearVelocity(robotBodyId_);
    JPH::Vec3 angVel = bodyInterface.GetAngularVelocity(robotBodyId_);
    JPH::Quat rot = bodyInterface.GetRotation(robotBodyId_);

    // Extract Y rotation (heading) from quaternion
    // For rotation about Y: theta = 2*atan2(qy, qw)
    float theta = 2.0f * std::atan2(rot.GetY(), rot.GetW());

    // Jolt (x, z) -> sim (x, y)
    out[0] = pos.GetX();
    out[1] = pos.GetZ();
    out[2] = theta;
    out[3] = vel.GetX();
    out[4] = vel.GetZ();
    out[5] = angVel.GetY();
}

int JoltWorld::spawnBall(float x, float y, float z, float vx, float vy, float vz, int color) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    auto shapeSettings = JPH::SphereShapeSettings(BALL_RADIUS);
    auto shapeResult = shapeSettings.Create();

    // sim (x, y, z) -> Jolt (x, z_jolt, y_jolt) where z is up in sim -> y is up in Jolt
    JPH::BodyCreationSettings bodySettings(
        shapeResult.Get(),
        JPH::Vec3(x, z, y), // sim z (up) -> Jolt y (up)
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Dynamic,
        Layers::MOVING
    );

    bodySettings.mFriction = BALL_FRICTION;
    bodySettings.mRestitution = 0.5f;
    bodySettings.mLinearDamping = 0.5f;   // ground friction slows balls down
    bodySettings.mAngularDamping = 0.3f;  // rolling friction
    bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    bodySettings.mMassPropertiesOverride.mMass = BALL_MASS;

    JPH::BodyID ballId = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
    bodyInterface.SetLinearVelocity(ballId, JPH::Vec3(vx, vz, vy));

    int index = static_cast<int>(balls_.size());
    balls_.push_back({ballId, color});
    return index;
}

void JoltWorld::removeBall(int index) {
    if (index < 0 || index >= static_cast<int>(balls_.size())) return;

    auto& bodyInterface = physicsSystem_->GetBodyInterface();
    bodyInterface.RemoveBody(balls_[index].bodyId);
    bodyInterface.DestroyBody(balls_[index].bodyId);

    // Swap with last and pop (invalidates indices, but that's fine)
    balls_[index] = balls_.back();
    balls_.pop_back();
}

int JoltWorld::getBallCount() const {
    return static_cast<int>(balls_.size());
}

void JoltWorld::getBallStates(float* out) const {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    for (int i = 0; i < static_cast<int>(balls_.size()); i++) {
        JPH::Vec3 pos = bodyInterface.GetPosition(balls_[i].bodyId);
        // Jolt (x, y, z) -> sim (x, z_jolt, y_jolt)
        out[i * 3 + 0] = pos.GetX();
        out[i * 3 + 1] = pos.GetZ(); // Jolt Z -> sim Y
        out[i * 3 + 2] = pos.GetY(); // Jolt Y -> sim Z (up)
    }
}

void JoltWorld::getBallColors(int* out) const {
    for (int i = 0; i < static_cast<int>(balls_.size()); i++) {
        out[i] = balls_[i].color;
    }
}

int JoltWorld::getIntakeOverlaps(int* out, int max) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    JPH::Vec3 sensorPos = bodyInterface.GetPosition(intakeSensorId_);
    float sensorRadius = std::max({INTAKE_WIDTH, INTAKE_DEPTH, INTAKE_HEIGHT});

    int count = 0;
    for (int i = 0; i < static_cast<int>(balls_.size()) && count < max; i++) {
        JPH::Vec3 ballPos = bodyInterface.GetPosition(balls_[i].bodyId);
        float dist = (ballPos - sensorPos).Length();
        // Simple proximity check - ball center within sensor bounding region
        if (dist < sensorRadius + BALL_RADIUS) {
            out[count++] = i;
        }
    }
    return count;
}
