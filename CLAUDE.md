# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FTC robotics competition code for team SigmaCorns (22377), season DECODE 2025-2026. The robot has a mecanum drivetrain, flywheel shooter, turret, hood, intake, transfer, and beam break sensors. Built on the FTC SDK with Kotlin as the primary language.

## Build & Run

```bash
# First-time setup (submodules, LFS, native libs)
./gradlew setupDeps

# Build and deploy to robot
./gradlew :TeamCode:installDebug

# Run all tests (JUnit 5, macOS uses -XstartOnFirstThread automatically)
# WARNING: Do NOT run the full test suite — some tests (JoltSimTest, RerunTest,
# RerunLoggingImageTest) use LWJGL visualizers that run indefinitely.
# Always run individual test classes instead.
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.FlywheelSimTest"

# Run a single test class
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.FlywheelSimTest"

# Rebuild native JNI libs for host tests
./gradlew :TeamCode:buildMecanumLtvOcpHost
./gradlew :TeamCode:buildJoltJni
./gradlew :TeamCode:buildDecodeEstimatorHost

# Rebuild native JNI libs for Android deployment
./gradlew :TeamCode:buildMecanumLtvOcp
./gradlew :TeamCode:buildDecodeEstimator

# Start trajopt server
./gradlew runTrajopt

# Sync trajopt assets to robot
./gradlew :TeamCode:syncTrajopt
```

## Commit Conventions

This repository uses [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/). All commit messages must follow the format:

```
<type>[optional scope]: <description>
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`

Breaking changes use `!` after type/scope or a `BREAKING CHANGE:` footer.

## Architecture

### IO Abstraction Layer (`sigmacorns.io`)

`SigmaIO` is the central hardware abstraction interface. All robot interactions go through it:
- `HardwareIO` — real FTC hardware (motors, servos, sensors, Limelight, pinpoint localizer)
- `SimIO` — lightweight simulation for unit tests
- `JoltSimIO` — physics-based simulation using Jolt Physics via JNI

### Command System (Kotlin Coroutines)

Instead of FTCLib commands or a traditional command-based framework, this project uses **Kotlin coroutines** for async robot commands. `PollableDispatcher` (in `sigmacorns.control.FSM.kt`) is a custom `CoroutineDispatcher` that runs on the main thread — coroutines only advance when `dispatcher.update()` is called in the opmode loop. This gives deterministic, single-threaded execution with async syntax. Commands are launched via `robot.scope.launch { ... }` and use `delay()` / `yield()` for timing. The `FSM` class extends this with a state machine that runs a coroutine per state and supports event-driven transitions.

### Subsystems (`sigmacorns.subsystem`)

Single-subsystem control only. Each subsystem takes `SigmaIO` (not `Robot`), manages its own hardware (motors/servos/sensors), and must not reference other subsystems. Input properties are set by coordinators each loop.

- `IntakeTransfer` — unified intake + transfer path (one mechanical system with two motors and a blocker servo that gates entry into the shooter). State enum: IDLE, INTAKING, REVERSING, TRANSFERRING.
- `Shooter` — unified flywheel + hood. Flywheel uses a deadbeat controller; hood computes launch angle from tuning data or projectile-motion trig fallback. Inputs (`targetDistance`, `recommendedHoodAngleDeg`, `flywheelTarget`) set by AimingSystem.
- `Turret` — dual-servo geared turret with PID, field-relative aiming, slew rate limiting
- `BeamBreak` — reads 3 beam break sensors to track ball count
- `Drivetrain` — mecanum drive math (stateless, takes IO as method param)

### Logic (`sigmacorns.logic`)

Cross-subsystem coordination. Logic classes take a `Robot` reference and orchestrate multiple subsystems:

- `AimingSystem` — full aiming pipeline: vision + GTSAM sensor fusion + turret targeting + shooter input feeding (hood/flywheel from adaptive tuner) + shoot-while-move compensation
- `IntakeCoordinator` — feeds beam break state into IntakeTransfer, auto-stops intake when full, coordinates blocker engagement on intake start, auto-shoot zone detection

### Robot (`sigmacorns.Robot`)

Top-level coordinator. Owns subsystems + logic, creates `PollableDispatcher` and `CoroutineScope`. The `update()` loop follows a strict pipeline: read sensors -> run coordinators (set subsystem inputs) -> run subsystem updates (write IO).

### Control (`sigmacorns.control`)

- `mpc/` — Model Predictive Control client that communicates with a trajopt solver running on the Limelight coprocessor over TCP
- `ltv/` — LTV (Linear Time-Varying) controller for trajectory tracking, backed by native C++ via JNI (`mecanum_LTV_OCP`)
- `localization/` — pose estimation with AprilTag fusion
- `PIDController`, `TrapezoidalProfile`, `SlewRateLimiter`, `FSM` — standard control primitives

### OpModes (`sigmacorns.opmode`)

- `SigmaOpMode` — base class for all team opmodes
- `auto/` — autonomous routines
- `test/` — hardware test opmodes
- `tune/` — tuning opmodes for PID, flywheel, etc.
- `MainTeleOp` — competition teleop

### Native Dependencies (`deps/`)

All are git submodules built via CMake, producing JNI shared libraries:
- `mecanum_LTV_OCP` — C++ LTV optimal control solver
- `mecanum_trajopt` — trajectory optimization server (Python, runs on Limelight)
- `jolt_sim` — Jolt Physics engine bindings for robot simulation
- `decode_estimator` — C++ state estimator
- `shooter_id_vision_system` — vision-based shooter identification

Host-built `.dylib`/`.so` files go to `TeamCode/src/test/jniLibs`; Android arm64 builds go to `TeamCode/src/main/jniLibs/arm64-v8a`.

### Tests (`TeamCode/src/test/kotlin/sigmacorns/`)

Tests use JUnit 5. The `java.library.path` is set to `src/test/jniLibs` for native lib loading. Some tests (e.g., `JoltSimTest`) use LWJGL for 3D visualization — run these individually as the visualizer can hang when run in batch.

## Key Details

- Kotlin 2.2.0, JVM target 1.8 for Android / 11 for tests
- `local.properties` contains SDK path and GitHub credentials for the Sloth Maven registry — never commit this file
- The Rust `rerun` crate in `TeamCode/src/main/rust/rerun` provides logging/visualization and is cross-compiled via `rust-android-gradle`
- Coordinates and poses use `sigmacorns.math.Pose2d` (x, y, heading)
- The Limelight coprocessor runs both vision pipelines and the MPC solver process, controlled via pipeline switching
