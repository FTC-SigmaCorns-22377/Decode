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

    const uint32_t maxBodies       = 512;
    const uint32_t numBodyMutexes  = 0; // auto
    const uint32_t maxBodyPairs    = 512;
    const uint32_t maxContactConstraints = 512;

    physicsSystem_ = new JPH::PhysicsSystem();
    physicsSystem_->Init(maxBodies, numBodyMutexes, maxBodyPairs, maxContactConstraints,
                         bpLayerInterface_, objectVsBroadPhase_, objectLayerPair_);

    buildField();
    buildGoals();
    createRobot();
    createIntakeRoller();

    contactListener_ = new IntakeContactListener(this);
    physicsSystem_->SetContactListener(contactListener_);
}

JoltWorld::~JoltWorld() {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // Remove balls
    for (auto& ball : balls_) {
        bodyInterface.RemoveBody(ball.bodyId);
        bodyInterface.DestroyBody(ball.bodyId);
    }
    balls_.clear();

    // Remove intake roller
    bodyInterface.RemoveBody(intake_.rollerBodyId);
    bodyInterface.DestroyBody(intake_.rollerBodyId);

    // Remove robot
    bodyInterface.RemoveBody(robotBodyId_);
    bodyInterface.DestroyBody(robotBodyId_);

    // Remove goal bodies
    auto removeGoal = [&](GoalInfo& g) {
        for (auto id : { g.sideWallAId, g.sideWallBId, g.frontWallId,
                         g.rectInnerWallId, g.rectPerpWallId, g.cornerWallId,
                         g.rampFloorId, g.rampRailAId, g.rampRailBId,
                         g.rampWallId, g.gateId, g.leverId }) {
            bodyInterface.RemoveBody(id);
            bodyInterface.DestroyBody(id);
        }
    };
    removeGoal(redGoal_);
    removeGoal(blueGoal_);

    // Remove field bodies (ground + walls)
    for (auto id : fieldBodyIds_) {
        bodyInterface.RemoveBody(id);
        bodyInterface.DestroyBody(id);
    }
    fieldBodyIds_.clear();

    delete contactListener_;
    delete physicsSystem_;
    delete jobSystem_;
    delete tempAllocator_;
}

JPH::BodyID JoltWorld::createStaticBox(JPH::Vec3 halfExtent, JPH::Vec3 position, float friction) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // Jolt adds convexRadius on top of half-extents, so shrink half-extents to compensate
    // Use a smaller convex radius for thin shapes
    float minHalf = std::min({halfExtent.GetX(), halfExtent.GetY(), halfExtent.GetZ()});
    float cr = std::min(0.005f, minHalf * 0.5f);
    cr = std::max(cr, 0.001f); // minimum convex radius
    JPH::Vec3 adjusted = halfExtent - JPH::Vec3(cr, cr, cr);
    adjusted = JPH::Vec3::sMax(adjusted, JPH::Vec3(0.001f, 0.001f, 0.001f));
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
    fieldBodyIds_.push_back(createStaticBox(
        JPH::Vec3(halfField, 0.01f, halfField),
        JPH::Vec3(0, -0.01f, 0),
        0.5f
    ));

    // Walls: +X, -X, +Z, -Z (field is in XZ plane, Y is up)
    fieldBodyIds_.push_back(createStaticBox(
        JPH::Vec3(halfThick, halfWallH, halfField),
        JPH::Vec3(halfField + halfThick, halfWallH, 0),
        WALL_FRICTION
    ));
    fieldBodyIds_.push_back(createStaticBox(
        JPH::Vec3(halfThick, halfWallH, halfField),
        JPH::Vec3(-halfField - halfThick, halfWallH, 0),
        WALL_FRICTION
    ));
    fieldBodyIds_.push_back(createStaticBox(
        JPH::Vec3(halfField, halfWallH, halfThick),
        JPH::Vec3(0, halfWallH, halfField + halfThick),
        WALL_FRICTION
    ));
    fieldBodyIds_.push_back(createStaticBox(
        JPH::Vec3(halfField, halfWallH, halfThick),
        JPH::Vec3(0, halfWallH, -halfField - halfThick),
        WALL_FRICTION
    ));
}

JPH::BodyID JoltWorld::createStaticBoxRotated(JPH::Vec3 halfExtent, JPH::Vec3 position,
                                               JPH::Quat rotation, float friction) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    float minHalf = std::min({halfExtent.GetX(), halfExtent.GetY(), halfExtent.GetZ()});
    float cr = std::min(0.005f, minHalf * 0.5f);
    cr = std::max(cr, 0.001f);
    JPH::Vec3 adjusted = halfExtent - JPH::Vec3(cr, cr, cr);
    adjusted = JPH::Vec3::sMax(adjusted, JPH::Vec3(0.001f, 0.001f, 0.001f));
    JPH::BoxShapeSettings shapeSettings(adjusted, cr);
    auto shapeResult = shapeSettings.Create();

    JPH::BodyCreationSettings bodySettings(
        shapeResult.Get(), position, rotation,
        JPH::EMotionType::Static, Layers::NON_MOVING
    );
    bodySettings.mFriction = friction;

    return bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::DontActivate);
}

void JoltWorld::buildGoalSide(GoalInfo& goal, float zSign) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();
    goal.zSign = zSign;

    // Goal sits in the corner where -X wall meets ±Z wall.
    // zSign: +1 for red (corner at -X, +Z), -1 for blue (corner at -X, -Z)
    // Triangle is offset from the ±Z wall by CRAMP_WIDTH so the ramp can sit flush against the wall.
    float cornerX = -HALF_FIELD;
    float cornerZ = zSign * HALF_FIELD;
    float goalOffset = CRAMP_WIDTH; // triangle inset from ±Z wall

    // Side wall A: along -X field wall, extending from corner toward field center (Z direction)
    // Uses full GOAL_TOTAL_HEIGHT at corner; physics uses max height (visual shows taper)
    float wallAMidZ = cornerZ - zSign * (goalOffset + GOAL_LEG / 2.0f);
    goal.sideWallAId = createStaticBox(
        JPH::Vec3(GOAL_WALL_THICK / 2.0f, GOAL_TOTAL_HEIGHT / 2.0f, GOAL_LEG / 2.0f),
        JPH::Vec3(cornerX + GOAL_WALL_THICK / 2.0f, GOAL_TOTAL_HEIGHT / 2.0f, wallAMidZ),
        0.3f
    );

    // Side wall B: flush against ±Z field wall, extending from corner in +X direction
    float wallBMidX = cornerX + GOAL_LEG / 2.0f;
    goal.sideWallBId = createStaticBox(
        JPH::Vec3(GOAL_LEG / 2.0f, GOAL_TOTAL_HEIGHT / 2.0f, GOAL_WALL_THICK / 2.0f),
        JPH::Vec3(wallBMidX, GOAL_TOTAL_HEIGHT / 2.0f, cornerZ - zSign * GOAL_WALL_THICK / 2.0f),
        0.3f
    );

    // Corner connector: along -X wall, bridges wall B (at ±Z wall) to wall A (offset)
    float cornerConnLen = goalOffset;
    float cornerConnMidZ = cornerZ - zSign * goalOffset / 2.0f;
    goal.cornerWallId = createStaticBox(
        JPH::Vec3(GOAL_WALL_THICK / 2.0f, GOAL_TOTAL_HEIGHT / 2.0f, cornerConnLen / 2.0f),
        JPH::Vec3(cornerX + GOAL_WALL_THICK / 2.0f, GOAL_TOTAL_HEIGHT / 2.0f, cornerConnMidZ),
        0.3f
    );

    // Front wall (diagonal hypotenuse): connects the open ends of the two legs
    // End of leg A: (cornerX, cornerZ - zSign * (goalOffset + GOAL_LEG))
    // End of leg B: (cornerX + GOAL_LEG, cornerZ - zSign * goalOffset)
    // Midpoint and rotation around Y axis
    float frontLength = GOAL_LEG * 1.4142f; // sqrt(2)
    float frontMidX = cornerX + GOAL_LEG / 2.0f;
    float frontMidZ = cornerZ - zSign * (goalOffset + GOAL_LEG / 2.0f);
    float frontAngle = -zSign * 3.14159f / 4.0f; // ±45° around Y (matches Three.js convention)

    JPH::Quat frontRot = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), frontAngle);
    goal.frontWallId = createStaticBoxRotated(
        JPH::Vec3(frontLength / 2.0f, GOAL_LIP_HEIGHT / 2.0f, GOAL_WALL_THICK / 2.0f),
        JPH::Vec3(frontMidX, GOAL_LIP_HEIGHT / 2.0f, frontMidZ),
        frontRot, 0.3f
    );

    // --- Classifier ramp ---
    // Runs alongside the ±Z field wall, extending in +X from the goal triangle.
    // Ramp is flush against the ±Z wall; the triangle is offset inward by CRAMP_WIDTH.
    // A rectangular extension bridges the offset triangle to the wall-hugging ramp.
    float rampStartX = cornerX + GOAL_LEG;                    // near goal exit
    float rampEndX = rampStartX + CRAMP_LENGTH;               // output end (gate)
    float rampMidX = (rampStartX + rampEndX) / 2.0f;

    // Ramp center: flush against ±Z wall (inner edge at wall)
    float rampZ = cornerZ - zSign * CRAMP_WIDTH / 2.0f;

    // --- Rectangular extension walls ---
    // Inner wall: along ±Z wall covering the ramp section (wall B covers the triangle area)
    goal.rectInnerWallId = createStaticBox(
        JPH::Vec3(CRAMP_LENGTH / 2.0f, GOAL_LIP_HEIGHT / 2.0f, GOAL_WALL_THICK / 2.0f),
        JPH::Vec3(rampMidX, GOAL_LIP_HEIGHT / 2.0f, cornerZ - zSign * GOAL_WALL_THICK / 2.0f),
        0.3f
    );
    // Perpendicular wall at X = cornerX + GOAL_LEG, connecting ±Z wall to the offset triangle
    float perpLen = goalOffset; // bridges from wall to offset triangle
    float perpMidZ = cornerZ - zSign * perpLen / 2.0f;
    goal.rectPerpWallId = createStaticBox(
        JPH::Vec3(GOAL_WALL_THICK / 2.0f, GOAL_LIP_HEIGHT / 2.0f, perpLen / 2.0f),
        JPH::Vec3(rampStartX, GOAL_LIP_HEIGHT / 2.0f, perpMidZ),
        0.3f
    );

    // Ramp slope: atan((START_H - END_H) / LENGTH)
    float rampSlopeAngle = std::atan2(CRAMP_START_H - CRAMP_END_H, CRAMP_LENGTH);

    // Ramp floor: slopes from CRAMP_START_H down to CRAMP_END_H along X
    JPH::Quat rampRot = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), -rampSlopeAngle);
    goal.rampFloorId = createStaticBoxRotated(
        JPH::Vec3(CRAMP_LENGTH / 2.0f, 0.005f, CRAMP_WIDTH / 2.0f),
        JPH::Vec3(rampMidX, CRAMP_MID_H, rampZ),
        rampRot, 0.4f
    );

    // Ramp side rails: flat on the ground, tall enough to reach the high end of the ramp
    float railHeight = CRAMP_START_H + CRAMP_RAIL_HEIGHT;
    goal.rampRailAId = createStaticBox(
        JPH::Vec3(CRAMP_LENGTH / 2.0f, railHeight / 2.0f, CRAMP_RAIL_THICK / 2.0f),
        JPH::Vec3(rampMidX, railHeight / 2.0f, rampZ - CRAMP_WIDTH / 2.0f),
        0.3f
    );
    goal.rampRailBId = createStaticBox(
        JPH::Vec3(CRAMP_LENGTH / 2.0f, railHeight / 2.0f, CRAMP_RAIL_THICK / 2.0f),
        JPH::Vec3(rampMidX, railHeight / 2.0f, rampZ + CRAMP_WIDTH / 2.0f),
        0.3f
    );

    // Solid fill below the ramp — height capped at ramp's lowest point (CRAMP_END_H)
    // so it never protrudes above the ramp surface
    float wallBelowH = CRAMP_END_H / 2.0f;
    goal.rampWallId = createStaticBox(
        JPH::Vec3(CRAMP_LENGTH / 2.0f, wallBelowH, CRAMP_WIDTH / 2.0f + CRAMP_WALL_THICK),
        JPH::Vec3(rampMidX, wallBelowH, rampZ),
        0.3f
    );

    // Gate: kinematic body at the output end of the ramp
    {
        float minHalf2 = std::min({GATE_THICK / 2.0f, GATE_CLOSED_H / 2.0f, GATE_WIDTH / 2.0f});
        float cr2 = std::min(0.005f, minHalf2 * 0.5f);
        cr2 = std::max(cr2, 0.001f);
        JPH::Vec3 gateHalf(GATE_THICK / 2.0f - cr2, GATE_CLOSED_H / 2.0f - cr2, GATE_WIDTH / 2.0f - cr2);
        gateHalf = JPH::Vec3::sMax(gateHalf, JPH::Vec3(0.001f, 0.001f, 0.001f));
        JPH::BoxShapeSettings ss(gateHalf, cr2);
        auto sr = ss.Create();

        // Position gate so its top clears a ball sitting on the ramp end
        float gateCenterY = CRAMP_END_H + GATE_CLOSED_H / 2.0f;
        JPH::BodyCreationSettings bs(
            sr.Get(),
            JPH::Vec3(rampEndX, gateCenterY, rampZ),
            JPH::Quat::sIdentity(),
            JPH::EMotionType::Kinematic, Layers::MOVING
        );
        bs.mFriction = 0.5f;
        goal.gateId = bodyInterface.CreateAndAddBody(bs, JPH::EActivation::Activate);
    }

    // Lever: see-saw at the ramp output end, extends toward field center.
    // Robot pushes outer end down → inner end lifts → balls flow out past gate.
    {
        // Box: LEVER_WIDTH along X (ramp direction), LEVER_LENGTH along Z (toward field)
        float minHalf = std::min({LEVER_WIDTH / 2.0f, LEVER_THICKNESS / 2.0f, LEVER_LENGTH / 2.0f});
        float cr = std::min(0.005f, minHalf * 0.5f);
        cr = std::max(cr, 0.001f);
        JPH::Vec3 leverHalf(LEVER_WIDTH / 2.0f - cr, LEVER_THICKNESS / 2.0f - cr, LEVER_LENGTH / 2.0f - cr);
        leverHalf = JPH::Vec3::sMax(leverHalf, JPH::Vec3(0.001f, 0.001f, 0.001f));
        JPH::BoxShapeSettings ss(leverHalf, cr);
        auto sr = ss.Create();

        // Centered at the field-center edge of the ramp, extending outward
        float leverZ = rampZ - zSign * (CRAMP_WIDTH / 2.0f + LEVER_LENGTH / 2.0f);
        float leverY = CRAMP_END_H;
        JPH::BodyCreationSettings bs(
            sr.Get(),
            JPH::Vec3(rampEndX, leverY, leverZ),
            JPH::Quat::sIdentity(),
            JPH::EMotionType::Kinematic, Layers::MOVING
        );
        bs.mFriction = 0.5f;
        goal.leverId = bodyInterface.CreateAndAddBody(bs, JPH::EActivation::Activate);
    }
}

void JoltWorld::buildGoals() {
    buildGoalSide(redGoal_, 1.0f);   // red: corner at +Z
    buildGoalSide(blueGoal_, -1.0f); // blue: corner at -Z
}

void JoltWorld::checkGoalScoring() {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    GoalInfo* goals[] = { &redGoal_, &blueGoal_ };

    for (auto* goal : goals) {
        float zs = goal->zSign;

        for (int i = static_cast<int>(balls_.size()) - 1; i >= 0; i--) {
            JPH::Vec3 bp = bodyInterface.GetPosition(balls_[i].bodyId);

            // Triangle check in XZ plane:
            // Triangle is offset from ±Z wall by CRAMP_WIDTH.
            // deltaX = distance from -X wall (toward field center)
            // deltaZ = distance from offset wall B (toward field center)
            // Inside triangle when: deltaX >= 0, deltaZ >= 0, deltaX + deltaZ <= GOAL_LEG
            float deltaX = bp.GetX() + HALF_FIELD;
            float deltaZ = HALF_FIELD - zs * bp.GetZ() - CRAMP_WIDTH;

            // Height check: ball is inside the goal (below lip, above ground)
            bool insideTriangle = deltaX >= 0.0f && deltaX <= GOAL_LEG
                               && deltaZ >= 0.0f && deltaZ <= GOAL_LEG
                               && (deltaX + deltaZ) <= GOAL_LEG;
            bool insideHeight = bp.GetY() > BALL_RADIUS && bp.GetY() < GOAL_LIP_HEIGHT;

            if (insideTriangle && insideHeight) {
                goal->scoredBalls++;
                bodyInterface.RemoveBody(balls_[i].bodyId);
                bodyInterface.DestroyBody(balls_[i].bodyId);
                balls_[i] = balls_.back();
                balls_.pop_back();
            }
        }
    }
}

void JoltWorld::updateGates(float dt) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    GoalInfo* goals[] = { &redGoal_, &blueGoal_ };

    for (auto* goal : goals) {
        JPH::Vec3 gatePos = bodyInterface.GetPosition(goal->gateId);
        JPH::Vec3 robotPos = bodyInterface.GetPosition(robotBodyId_);

        // Check if robot is near the gate and pushing it
        float dist = (robotPos - gatePos).Length();
        bool robotPushing = dist < (ROBOT_WIDTH / 2.0f + GATE_THICK + 0.05f);

        if (robotPushing && goal->gateOpenAmount < GATE_TRAVEL) {
            // Open the gate
            goal->gateOpenAmount += dt * 0.1f; // opens over ~0.5s
            goal->gateOpenAmount = std::min(goal->gateOpenAmount, GATE_TRAVEL);
        } else if (!robotPushing && goal->gateOpenAmount > 0.0f) {
            // Gravity closes the gate
            goal->gateOpenAmount -= dt * 0.08f;
            goal->gateOpenAmount = std::max(goal->gateOpenAmount, 0.0f);
        }

        // Move the gate: slides in +X direction when opening
        JPH::Vec3 newPos = gatePos;
        float origX = -HALF_FIELD + GOAL_LEG + CRAMP_LENGTH;
        float rampZ = goal->zSign * HALF_FIELD - goal->zSign * CRAMP_WIDTH / 2.0f;
        newPos.SetX(origX + goal->gateOpenAmount);
        newPos.SetZ(rampZ); // keep Z stable

        bodyInterface.MoveKinematic(goal->gateId, newPos, JPH::Quat::sIdentity(), dt);
    }
}

void JoltWorld::updateLevers(float dt) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();
    GoalInfo* goals[] = { &redGoal_, &blueGoal_ };

    for (auto* goal : goals) {
        JPH::Vec3 leverPos = bodyInterface.GetPosition(goal->leverId);
        JPH::Vec3 robotPos = bodyInterface.GetPosition(robotBodyId_);

        // Check if robot is near the lever's outer end (field-center side)
        float dist = (robotPos - leverPos).Length();
        bool robotPushing = dist < (ROBOT_WIDTH / 2.0f + LEVER_LENGTH / 2.0f + 0.05f);

        float targetAngle = 0.0f;
        if (robotPushing) {
            // Robot pushes outer end down → lever tilts so inner end goes up
            targetAngle = 0.35f;
        }

        float angleDiff = targetAngle - goal->leverAngle;
        goal->leverAngle += angleDiff * std::min(dt * 5.0f, 1.0f);

        // Tilt around X axis: -zSign makes the field-center end go DOWN
        // (for red zSign=+1, lever extends in -Z; negative X rotation lowers -Z end)
        JPH::Quat tilt = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), -goal->zSign * goal->leverAngle);
        bodyInterface.MoveKinematic(goal->leverId, leverPos, tilt, dt);
    }
}

void JoltWorld::getGoalStates(float* out) const {
    out[0] = static_cast<float>(redGoal_.scoredBalls);
    out[1] = static_cast<float>(blueGoal_.scoredBalls);
    out[2] = redGoal_.gateOpenAmount;
    out[3] = blueGoal_.gateOpenAmount;
    out[4] = redGoal_.leverAngle;
    out[5] = blueGoal_.leverAngle;
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

    // Collision group: robot is subGroup 0, intake roller is subGroup 1
    // They won't collide with each other but will collide with everything else
    robotRollerGroupFilter_ = new JPH::GroupFilterTable(2);
    robotRollerGroupFilter_->DisableCollision(0, 1);
    bodySettings.mCollisionGroup.SetGroupFilter(robotRollerGroupFilter_);
    bodySettings.mCollisionGroup.SetGroupID(0);
    bodySettings.mCollisionGroup.SetSubGroupID(0);

    robotBodyId_ = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
}

void JoltWorld::createIntakeRoller() {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // Create cylinder shape for the roller: axis along X (width direction)
    // CylinderShape has axis along Y by default, so rotate 90deg around Z
    float cylConvexRadius = std::min(0.005f, INTAKE_ROLLER_RADIUS * 0.3f);
    JPH::CylinderShapeSettings cylinderSettings(INTAKE_WIDTH / 2.0f - cylConvexRadius,
                                                  INTAKE_ROLLER_RADIUS - cylConvexRadius,
                                                  cylConvexRadius);
    auto cylinderResult = cylinderSettings.Create();

    // Rotate cylinder so its axis aligns with Jolt X (robot width direction)
    JPH::RotatedTranslatedShapeSettings rotatedSettings(
        JPH::Vec3::sZero(),
        JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), 3.14159265f / 2.0f),
        cylinderResult.Get()
    );
    auto rotatedResult = rotatedSettings.Create();

    // Compute initial roller position based on rest angle
    // Pivot is at top-front edge of chassis: (0, ROBOT_HEIGHT, ROBOT_LENGTH/2)
    float pivotY = ROBOT_HEIGHT;
    float pivotZ = ROBOT_LENGTH / 2.0f;
    float rollerY = pivotY + INTAKE_BAR_LENGTH * std::sin(INTAKE_REST_ANGLE);
    float rollerZ = pivotZ + INTAKE_BAR_LENGTH * std::cos(INTAKE_REST_ANGLE);

    // Use kinematic body that follows the robot (avoids hinge constraint instability)
    JPH::BodyCreationSettings bodySettings(
        rotatedResult.Get(),
        JPH::Vec3(0, rollerY, rollerZ),
        JPH::Quat::sIdentity(),
        JPH::EMotionType::Kinematic,
        Layers::MOVING
    );
    bodySettings.mFriction = INTAKE_ROLLER_FRICTION;

    intake_.rollerBodyId = bodyInterface.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
    intake_.hingeAngle = INTAKE_REST_ANGLE;
}

void JoltWorld::step(float dt) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // Apply queued force and torque to robot
    if (queuedFx_ != 0.0f || queuedFy_ != 0.0f) {
        // Sim frame: x=forward, y=left, z=up
        // Jolt frame: X=right, Y=up, Z=forward (robot faces +Z, intake along +Z)
        // Mapping: sim x -> Jolt +Z, sim y -> Jolt -X
        bodyInterface.AddForce(robotBodyId_, JPH::Vec3(-queuedFy_, 0, queuedFx_));
    }
    if (queuedTz_ != 0.0f) {
        bodyInterface.AddTorque(robotBodyId_, JPH::Vec3(0, queuedTz_, 0));
    }
    queuedFx_ = queuedFy_ = queuedTz_ = 0.0f;

    // Step physics
    physicsSystem_->Update(dt, 1, tempAllocator_, jobSystem_);

    // Post-step: check pickups, scoring, and mechanisms
    updateIntake(dt);
    checkGoalScoring();
    updateGates(dt);
    updateLevers(dt);
}

void JoltWorld::applyRobotForce(float fx, float fy, float tz) {
    queuedFx_ = fx;
    queuedFy_ = fy;
    queuedTz_ = tz;
}

void JoltWorld::setRobotPose(float x, float y, float theta) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    // sim (x,y,theta) -> Jolt (-y, height, x)
    JPH::Vec3 pos(-y, ROBOT_HEIGHT / 2.0f, x);
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

    // Jolt -> sim: sim x = Jolt Z, sim y = -Jolt X
    out[0] = pos.GetZ();
    out[1] = -pos.GetX();
    out[2] = theta;
    out[3] = vel.GetZ();
    out[4] = -vel.GetX();
    out[5] = angVel.GetY();
}

int JoltWorld::spawnBall(float x, float y, float z, float vx, float vy, float vz, int color) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    auto shapeSettings = JPH::SphereShapeSettings(BALL_RADIUS);
    auto shapeResult = shapeSettings.Create();

    // sim (x, y, z) -> Jolt (-y, z, x): sim x->Jolt Z, sim y->Jolt -X, sim z->Jolt Y
    JPH::BodyCreationSettings bodySettings(
        shapeResult.Get(),
        JPH::Vec3(-y, z, x),
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
    bodyInterface.SetLinearVelocity(ballId, JPH::Vec3(-vy, vz, vx));

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
        // Jolt -> sim: sim x = Jolt Z, sim y = -Jolt X, sim z = Jolt Y
        out[i * 3 + 0] = pos.GetZ();
        out[i * 3 + 1] = -pos.GetX();
        out[i * 3 + 2] = pos.GetY();
    }
}

void JoltWorld::getBallColors(int* out) const {
    for (int i = 0; i < static_cast<int>(balls_.size()); i++) {
        out[i] = balls_[i].color;
    }
}

// --- Intake API ---

void JoltWorld::setIntakeRollerOmega(float omega) {
    intake_.rollerOmega = omega;
}

void JoltWorld::getIntakeState(float* out) const {
    out[0] = intake_.hingeAngle;
    out[1] = intake_.rollerOmega;
}

void JoltWorld::updateIntake(float dt) {
    auto& bodyInterface = physicsSystem_->GetBodyInterface();

    JPH::Vec3 robotPos = bodyInterface.GetPosition(robotBodyId_);
    JPH::Quat robotRot = bodyInterface.GetRotation(robotBodyId_);

    // Pivot point in robot local frame
    float pivotY = ROBOT_HEIGHT;
    float pivotZ = ROBOT_LENGTH / 2.0f;

    // Simulate hinge: check if any ball is pushing the intake upward
    float pushAngle = 0.0f;
    for (int i = 0; i < static_cast<int>(balls_.size()); i++) {
        JPH::Vec3 ballPos = bodyInterface.GetPosition(balls_[i].bodyId);
        JPH::Vec3 localBall = robotRot.Conjugated() * (ballPos - robotPos);

        // Check if ball is in the intake zone (near the roller)
        float rollerY = pivotY + INTAKE_BAR_LENGTH * std::sin(intake_.hingeAngle);
        float rollerZ = pivotZ + INTAKE_BAR_LENGTH * std::cos(intake_.hingeAngle);

        float distToRoller = std::sqrt(
            (localBall.GetY() - rollerY) * (localBall.GetY() - rollerY) +
            (localBall.GetZ() - rollerZ) * (localBall.GetZ() - rollerZ));

        if (distToRoller < BALL_RADIUS + INTAKE_ROLLER_RADIUS + 0.02f) {
            // Ball is touching roller — compute push angle
            float ballAngle = std::atan2(localBall.GetY() - pivotY, localBall.GetZ() - pivotZ);
            if (ballAngle > intake_.hingeAngle) {
                pushAngle = std::max(pushAngle, ballAngle - intake_.hingeAngle);
            }
        }
    }

    // Apply push + gravity return
    if (pushAngle > 0.01f) {
        intake_.hingeAngle += pushAngle * std::min(dt * 10.0f, 1.0f);
    } else {
        // Gravity return toward rest angle
        float restError = INTAKE_REST_ANGLE - intake_.hingeAngle;
        intake_.hingeAngle += restError * std::min(dt * 3.0f, 1.0f);
    }
    intake_.hingeAngle = std::max(INTAKE_MIN_ANGLE, std::min(INTAKE_MAX_ANGLE, intake_.hingeAngle));

    // Move kinematic roller to follow robot
    float rollerY = pivotY + INTAKE_BAR_LENGTH * std::sin(intake_.hingeAngle);
    float rollerZ = pivotZ + INTAKE_BAR_LENGTH * std::cos(intake_.hingeAngle);
    JPH::Vec3 localRollerPos(0, rollerY, rollerZ);
    JPH::Vec3 worldRollerPos = robotPos + robotRot * localRollerPos;
    bodyInterface.MoveKinematic(intake_.rollerBodyId, worldRollerPos, robotRot, dt);

    // Check for ball pickups if roller is spinning
    if (std::abs(intake_.rollerOmega) < INTAKE_OMEGA_THRESHOLD) return;

    for (int i = static_cast<int>(balls_.size()) - 1; i >= 0; i--) {
        JPH::Vec3 ballPos = bodyInterface.GetPosition(balls_[i].bodyId);
        JPH::Vec3 localBall = robotRot.Conjugated() * (ballPos - robotPos);

        // Check if ball is near the front face of the chassis
        bool nearFrontFace = localBall.GetZ() > (ROBOT_LENGTH / 2.0f - BALL_RADIUS) &&
                             localBall.GetZ() < (ROBOT_LENGTH / 2.0f + INTAKE_BAR_LENGTH + BALL_RADIUS);
        bool withinWidth = std::abs(localBall.GetX()) < (INTAKE_WIDTH / 2.0f + BALL_RADIUS);
        bool withinHeight = localBall.GetY() > -BALL_RADIUS && localBall.GetY() < (ROBOT_HEIGHT + BALL_RADIUS * 2);

        if (nearFrontFace && withinWidth && withinHeight) {
            pendingPickups_.push_back(i);
        }
    }

    // Sort descending so swap-and-pop removal works correctly
    std::sort(pendingPickups_.begin(), pendingPickups_.end(), std::greater<int>());
}

int JoltWorld::getPendingPickups(int* out, int max) {
    int count = std::min(static_cast<int>(pendingPickups_.size()), max);
    for (int i = 0; i < count; i++) {
        out[i] = pendingPickups_[i];
    }
    pendingPickups_.clear();
    return count;
}

// --- Contact listener: conveyor belt effect ---

void IntakeContactListener::OnContactAdded(const JPH::Body& body1, const JPH::Body& body2,
                                            const JPH::ContactManifold& manifold,
                                            JPH::ContactSettings& settings) {
    applyIntakeEffect(body1, body2, manifold, settings);
}

void IntakeContactListener::OnContactPersisted(const JPH::Body& body1, const JPH::Body& body2,
                                                const JPH::ContactManifold& manifold,
                                                JPH::ContactSettings& settings) {
    applyIntakeEffect(body1, body2, manifold, settings);
}

void IntakeContactListener::applyIntakeEffect(const JPH::Body& body1, const JPH::Body& body2,
                                               const JPH::ContactManifold& manifold,
                                               JPH::ContactSettings& settings) {
    if (std::abs(world_->intake_.rollerOmega) < 1.0f) return;

    // Determine which body is the roller and which is the ball
    bool body1IsRoller = (body1.GetID() == world_->intake_.rollerBodyId);
    bool body2IsRoller = (body2.GetID() == world_->intake_.rollerBodyId);
    if (!body1IsRoller && !body2IsRoller) return;

    const JPH::Body& rollerBody = body1IsRoller ? body1 : body2;
    const JPH::Body& otherBody = body1IsRoller ? body2 : body1;

    // Check if the other body is a ball
    bool isBall = false;
    for (const auto& ball : world_->balls_) {
        if (ball.bodyId == otherBody.GetID()) {
            isBall = true;
            break;
        }
    }
    if (!isBall) return;

    // Compute conveyor belt direction: from roller toward chassis (in -Z local direction)
    // Must use NoLock interface since we're inside a physics callback
    auto& bodyInterface = world_->physicsSystem_->GetBodyInterfaceNoLock();
    JPH::Quat robotRot = bodyInterface.GetRotation(world_->robotBodyId_);

    // Robot forward is +Z in Jolt, so "toward chassis" is -Z in robot frame
    JPH::Vec3 pullDirection = robotRot * JPH::Vec3(0, 0, -1.0f);

    float surfaceSpeed = std::abs(world_->intake_.rollerOmega) * JoltWorld::INTAKE_ROLLER_RADIUS;

    // Set relative surface velocity to create conveyor belt effect
    // This pulls the ball toward the chassis
    settings.mRelativeLinearSurfaceVelocity = pullDirection * surfaceSpeed;
}
