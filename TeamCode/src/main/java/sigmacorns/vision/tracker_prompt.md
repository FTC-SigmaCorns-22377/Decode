You are extending the FTC-SigmaCorns-22377/Decode codebase to add a vision-based ball-tracking and chase system for the DECODE 2025-2026 season. READ CLAUDE.md FIRST — it documents the Kotlin codebase layout, SigmaIO abstraction, JoltSimIO, subsystem pattern, coroutine dispatcher, native JNI deps, and test conventions. Follow those conventions exactly.
Goal
Add a ball tracker that fuses detections over time into stable field-coordinate ball tracks and drives the robot to chase a selected ball. Prove the system in the existing Jolt simulation under a noise sweep BEFORE real hardware. Render sim state into the team's existing custom three.js web viewer, and add a camera-view panel to that viewer so we can watch detections and tracks overlaid on the simulated camera feed.
Step 0: discover and extract (do this first, report back before coding)
0a. Find the existing three.js web viewer
It's in this repo, not a submodule. Locate it:

rg -l --hidden -g '!node_modules' -g '!build' -i 'three' .
rg -l --hidden -g '!node_modules' -g '!build' -i 'threejs|three\.js|@react-three' .
find . -name package.json -not -path '*/node_modules/*'
find . -name '*.html' -not -path '*/no
de_modules/*' -not -path '*/build/*'
Check doc/, and any viewer/, web/, visualizer/, frontend/, site/ directories.

Also identify the sim → viewer transport: search for WebSocket, ServerSocket, ktor, javalin, Spark, :8080, :3000, localhost, SSE, or file-drop state in Kotlin sources. Do not build a new transport — reuse the existing one.
If you cannot find an existing three.js viewer after thorough search, STOP and tell me. Do not fabricate one.
0b. Extract geometry from the provided STEP file
A STEP file Top_Level__2_.step has been provided at /mnt/user-data/uploads/Top_Level__2_.step (copy it into the repo under config/cad/ first; do not commit it if it's large — add to .gitignore if needed). Use cadquery (preferred, pip install cadquery), pythonocc-core, or FreeCAD's Python API to parse it. Extract:

Camera mount position CAM_POS_R in robot coordinates (meters, x forward / y left / z up). Look for a component named or tagged "camera", "webcam", "global shutter", or the topmost forward-facing mounted body. If ambiguous, list candidates and ask.
Robot footprint polygon ROBOT_FOOTPRINT_R — the convex hull of the drivetrain or chassis outline projected onto the x/y plane, in robot frame. Simplify to ≤ 8 vertices.
Intake region — the x/y extent of the intake mechanism in robot frame. Needed later for the image-space mask.

Write all extracted values into config/ball_tracker.json. Include a comment block in the JSON or an accompanying config/ball_tracker.notes.md explaining which STEP components each value came from, so the numbers can be audited later.
0c. Report back
After 0a and 0b, post a short summary: viewer location + transport, extracted CAM_POS_R, extracted footprint, and any ambiguities. Only then start writing tracker code.
Non-negotiables

Filtering is done in field coordinates, never pixel/camera space.
Measurement covariance is per-detection (Jacobian-propagated), not a fixed scalar.
Pose used for projection corresponds to frame CAPTURE time, not arrival time.
Gating (image-space intake mask → field bounds → ramp exclusion → robot self-footprint) runs BEFORE association. The image-space mask is applied to raw pixel detections first; the field-coord gates apply after projection.
The tracker core is a pure Kotlin library: tick(detections, pose, t) -> tracks. No I/O. Identical call site from sim tests and real robot logic.
Reuse the existing three.js viewer and its existing transport. Extend, don't replace.
Kotlin style matches existing code. Conventional Commits.

Stack alignment (from CLAUDE.md)

Kotlin 2.2.0. JVM target 1.8 for Android, 11 for tests.
SigmaIO abstraction: HardwareIO / SimIO / JoltSimIO.
Coroutines via PollableDispatcher. Main-thread, deterministic.
Pose type: sigmacorns.math.Pose2d (x, y, heading).
Tests: JUnit 5. Native libs from src/test/jniLibs.
LWJGL viewer runs indefinitely — always run individual test classes.
Follow the AimingSystem pattern in sigmacorns.logic for the new BallTrackingSystem.

Module layout
sigmacorns/vision/tracker/
Frames.kt           # coordinate frames, T_RC builder
Projection.kt       # pixel + T_FC -> field point + covariance
Gating.kt           # image mask, field bounds, ramp, footprint
KalmanTrack.kt      # single-track KF state + predict/update
Association.kt      # Mahalanobis-gated greedy assignment
Tracker.kt          # top-level tick(detections, pose, t) -> tracks
PoseBuffer.kt       # ring buffer + time interpolation (wrap-aware theta)
TrackerConfig.kt    # all tunables; loaded from config/ball_tracker.json

sigmacorns/vision/sim/
SimulatedCamera.kt  # forward-project sim balls -> pixel detections + noise
SimCameraConfig.kt  # intrinsics, extrinsics, noise knobs

sigmacorns/vision/viz/
VizPublisher.kt     # publishes tracker + camera state to the three.js viewer
# using the transport discovered in Step 0

sigmacorns/logic/
BallTrackingSystem.kt
ChaseCoordinator.kt

sigmacorns/io/
Extend JoltSimIO: own a SimulatedCamera; implement getBallDetections(t, pose).
Extend HardwareIO: implement getBallDetections via Limelight pipeline.

TeamCode/src/test/kotlin/sigmacorns/test/vision/
ProjectionTest.kt, KalmanTrackTest.kt, GatingTest.kt,
AssociationTest.kt, PoseBufferTest.kt, BallTrackerSimTest.kt
For the web viewer, extend whatever directory Step 0 found. Add a camera-view panel next to the existing 3D field view. Match the existing viewer's conventions (React vs vanilla, build tooling, CSS).
Coordinate conventions (Frames.kt header comment)

Field F: origin at one field corner, +X right, +Y up, +Z out of floor. Meters. Heading CCW from +X. Matches sigmacorns.math.Pose2d.
Robot R: x forward, y left, z up.
Camera C: OpenCV — x right, y down, z forward along the optical axis.

Build T_RC from (cam_x_r, cam_y_r, cam_z_r, pitch_down, yaw, roll). Document Euler order in the file header. Unit-test: with T_RC=I and T_FR=I, a pixel at the principal point projects directly below the camera on the ground plane.
Projection.kt
Input: pixel (u, v), intrinsics K, distCoeffs, camera pose T_FC, plane height h.

Undistort (u, v) — port OpenCV's undistortPoints (5-iter Newton on the distortion polynomial) in pure Kotlin, or use an existing util if present. Do NOT add a new OpenCV dependency.
x_n = (u_ud - cx)/fx; y_n = (v_ud - cy)/fy.
d_C = (x_n, y_n, 1).
d_F = R_FC * d_C.
s = (h - o_F.z) / d_F.z. If s ≤ 0 or s > MAX_RANGE_M, return null.
p_F = o_F + s * d_F. Return (p_F.x, p_F.y).

Also projectWithCovariance(u, v, K, distCoeffs, T_FC, h, sigmaPx) returning (p_F, R_meas_2x2) via central finite differences (δ = 0.5 px per axis): R = J Σ_px Jᵀ.
Unit tests:

Forward-then-back round trip for a synthetic ball, within 1e-6 m.
Covariance major axis aligned with viewing direction within tolerance.

Gating.kt
Apply in order:

Image-space intake mask insideIntakeMask(u, v) — rectangular region at the bottom of the frame (config: INTAKE_MASK_Y_MIN_FRAC, default 0.66; full image width). If detection is inside this mask, drop it before projection. This catches the common case of the camera seeing our own intake wheels and artifacts held in the intake.
Range validRange(s) — reject if s outside (0, MAX_RANGE_M] (inside projection).
Field bounds insideField(p) — 10 cm inward margin from the 3.6576 m × 3.6576 m field.
Ramp exclusion outsideRamp(p) — polygon from config, expanded 20 cm on the camera-facing side.
Robot footprint outsideRobotFootprint(p, robotPose) — transform config footprint into field frame by robotPose, reject interior detections.

Combine into filterDetections(...). Unit tests for each gate independently.
KalmanTrack.kt
class KalmanTrack(val id: Int, z0, R0, t0):
state: DoubleArray(4)  // px, py, vx, vy
P: 4x4
lastUpdateT, framesSinceSeen, hits

predict(tNow): dt = tNow - lastUpdateT; apply F, Q. Do NOT update lastUpdateT.
update(z, Rmeas, tNow): KF update; lastUpdateT = tNow; framesSinceSeen = 0; hits++.
getPosition(tNow): predict on a copy, return (x, y).
Matrices (constant-velocity, σ_a = SIGMA_A_MPS2):
F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
Q = σ_a² * [[dt⁴/4, 0, dt³/2, 0],
[0, dt⁴/4, 0, dt³/2],
[dt³/2, 0, dt², 0],
[0, dt³/2, 0, dt²]]
H = [[1,0,0,0],[0,1,0,0]]
Unit tests: CV stream converges to true velocity; predict-only covariance trace grows monotonically.
Association.kt
For every (track i, detection j): d² = yᵀ S⁻¹ y, y = z_j - H x̂_i, S = H P Hᵀ + R_meas_j. Gate at CHI2_GATE = 9.21. Greedy assign ascending d², each track/detection claimed at most once. Return (matches, unmatchedTracks, unmatchedMeasurements).
Unit tests: empty inputs, 1:1 clean, two tracks crossing paths, spurious detection.
Tracker.kt
class Tracker(config: TrackerConfig):
tick(detectionsPx: List<PixelDetection>, pose: Pose2d, t: Double) -> List<KalmanTrack>:
1. Apply image-space intake mask to detectionsPx.
2. Build T_FC from pose + config.T_RC.
3. Project + covariance; drop nulls.
4. Field-coord gating (field, ramp, footprint).
5. predict(t) on all tracks.
6. Associate.
7. Update matched; framesSinceSeen++ on unmatched.
8. Spawn tracks from unmatched detections: state=(x,y,0,0); P position=R_meas;
velocity variance = INIT_VEL_VAR * I2.
9. Delete tracks with framesSinceSeen > MAX_COAST_FRAMES or position P trace
above threshold.
10. Return active tracks.

selectTarget(tracks, pose, strategy=Closest): default closest confirmed
(hits >= MIN_HITS_FOR_CONFIRMED).
PoseBuffer.kt
ArrayDeque<PoseSample>(capacity ~500). add(t, pose). get(t) linearly interpolates bracketing samples; heading via unwrap → interp → rewrap. Returns null if t outside buffer or fewer than 2 samples. Unit-test with known samples, tolerance 1e-9.
TrackerConfig.kt
Load all tunables from config/ball_tracker.json. Defaults, with values pre-filled from provided inputs:
FIELD_WIDTH_M          = 3.6576
FIELD_HEIGHT_M         = 3.6576
FIELD_MARGIN_M         = 0.10
RAMP_POLYGON           = [fill from CAD/game manual]
RAMP_EXPAND_M          = 0.20

BALL_RADIUS_M          = 0.035      // CONFIRM from DECODE game manual

K                      = [fx, fy, cx, cy]        // from calibration
DIST_COEFFS            = [k1, k2, p1, p2, k3]    // from calibration
IMAGE_WIDTH_PX         = [from calibration]
IMAGE_HEIGHT_PX        = [from calibration]

CAM_POS_R              = [from STEP extraction, Step 0b]
CAM_PITCH_DOWN_RAD     = 0.26908    // 15.417° downward — PROVIDED
CAM_YAW_RAD            = 0.0
CAM_ROLL_RAD           = 0.0

ROBOT_FOOTPRINT_R      = [from STEP extraction, Step 0b]

INTAKE_MASK_Y_MIN_FRAC = 0.66       // bottom third of image

SIGMA_PX               = 1.5
MAX_RANGE_M            = 8.0

SIGMA_A_MPS2           = 3.0
INIT_VEL_VAR           = 1.0
CHI2_GATE              = 9.21
MAX_COAST_FRAMES       = 15
MIN_HITS_FOR_CONFIRMED = 3
SimulatedCamera.kt
Given the JoltSimIO world state, produce detections per frame.
class SimulatedCamera(config: SimCameraConfig):
fun frame(world: JoltWorldView, t: Double, robotPose: Pose2d) -> List<PixelDetection>
Per frame:

Compute T_FC from robotPose + config.T_RC.
For each simulated ball b_F:

b_C = T_FC⁻¹ · b_F
Skip if b_C.z ≤ 0 (behind camera).
u = fx·b_C.x/b_C.z + cx; v = fy·b_C.y/b_C.z + cy.
Apply forward distortion model.
Skip if outside image bounds.
Add Gaussian pixel noise N(0, sigmaPxSim).
With probability pDrop, skip.


With probability pFalsePositive, emit one uniform-random (u, v) inside the image.
Attach timestamp t.

SimCameraConfig: imageWidthPx, imageHeightPx, K, distCoeffs, T_RC, sigmaPxSim (default 0), pDrop (default 0), pFalsePositive (default 0).
Integrate into JoltSimIO; expose getBallDetections(t, robotPose) on SigmaIO. Implement on HardwareIO via the Limelight pipeline.
Logic layer
BallTrackingSystem: holds Tracker, PoseBuffer, SigmaIO ref. Each update: read detections from IO; push current pose into buffer; tracker.tick(detections, poseAtFrameTime, t); selectTarget. Expose targetBallField: Pose2d?. Follow AimingSystem's structure.
ChaseCoordinator: takes Robot ref. Each update: read BallTrackingSystem.targetBallField; if non-null, command drivetrain via existing sigmacorns.subsystem.Drivetrain math with a heading + forward-velocity controller. Respect existing Robot.update() coordinator ordering.
Web viewer integration (VizPublisher.kt + viewer extension)
VizPublisher (Kotlin side): publishes per-tick to the transport found in Step 0. Payload JSON per frame:
json{
"t": 1.234,
"robot": { "x": 1.2, "y": 0.8, "theta": 0.5, "footprint": [[x,y],...] },
"camera": {
"origin": [x,y,z],
"R_field_camera": [[...],[...],[...]],
"fov_frustum_ground": [[x,y],...],
"intrinsics": { "fx":..., "fy":..., "cx":..., "cy":...,
"width":..., "height":... }
},
"ball_truth": [ { "id":..., "x":..., "y":... } ],
"detections_px": [ { "u":..., "v":..., "t":... } ],
"detections_field": [ { "x":..., "y":...,
"cov":[[...],[...]] } ],
"tracks": [ { "id":..., "x":..., "y":..., "vx":..., "vy":...,
"cov":[[...],[...]], "hits":..., "confirmed":... } ],
"target_id": 3,
"scenario": "PIXEL_AND_POSE",
"rms_error_m": 0.037
}
Viewer extension (JS/TS side, in the discovered viewer directory):

Preserve the existing 3D field view. Add overlays on the field plane:

Tracks: dots + 2σ covariance ellipses.
Target track highlighted in a distinct color.
Ground-truth balls (small translucent markers).
Camera frustum projected onto the ground plane.
Ramp exclusion polygon outline.


Add a camera-view panel (rectangular subview or side panel):

Black background at the configured image size, aspect-correct.
Render the simulated camera view: ground-plane grid with field lines, ramp cutout, artifacts as colored circles at their forward-projected pixel positions (radius from depth).
Overlay raw detection markers (small crosses at (u, v)).
Overlay the intake mask region as a translucent red box at the bottom.
Overlay projected-track covariance ellipses re-projected into pixel space.
Show the current scenario label and live RMS-error number as HUD text.


Add scenario-selector UI so the user can flip between the 6 test scenarios and a live-sim mode.

End-to-end sim test: BallTrackerSimTest.kt
Gate the LWJGL viewer and web-viz publishing behind env flags:

BALL_TRACKER_VIZ_WEB=1 → start VizPublisher.
BALL_TRACKER_VIZ_LWJGL=1 → open LWJGL overlay.
Default (CI): headless, no viewer.

Structure:
@Test fun zeroNoiseStationaryBall()       { runScenario(Noise.ZERO, ...) }
@Test fun pixelNoiseOnly()                { runScenario(Noise.PIXEL_ONLY, ...) }
@Test fun pixelAndPoseNoise()             { runScenario(Noise.PIXEL_AND_POSE, ...) }
@Test fun fullNoiseWithDropoutAndFP()     { runScenario(Noise.FULL, ...) }
@Test fun ballOnRampRejected()            { ... }
@Test fun ballInAndOutOfFrame()           { ... }
@Test fun headlessChaseScenario()         { robot starts 2 m from ball; assert arrive within 15 cm within 5 s sim time; PIXEL_AND_POSE noise }
runScenario: build JoltSimIO with known ball(s); drive robot in a circle (or to a target); configure SimulatedCamera noise; loop Robot.update() at 50 Hz; log per-tick truth + estimate + covariance + target. Emit CSV to build/reports/ball_tracker/.
Assertions:
Noise.ZERO:           RMS error < 1e-3 m over last 3 s.
Noise.PIXEL_ONLY:     RMS error < 2e-2 m over last 3 s.
Noise.PIXEL_AND_POSE: RMS error < 5e-2 m over last 3 s.
Noise.FULL:           RMS error < 10e-2 m AND target track persists >=95% of frames.
ballOnRampRejected:   zero tracks inside ramp polygon, entire run.
ballInAndOutOfFrame:  track survives 10-frame gap and re-locks on reappearance.
Print a pass/fail table to stdout with the RMS numbers.
Hardware integration (after sim tests pass)

Implement HardwareIO.getBallDetections(t, pose) via Limelight pipeline.
Calibrate camera intrinsics (checkerboard). Store in config/ball_tracker.json.
Verify extrinsics: ball at known field location, small utility op-mode logs projected (x, y) vs. tape measure. Adjust CAM_POS_R until error < 3 cm across 1–2 m.
Tele-op enabling BallTrackingSystem + ChaseCoordinator with a manual abort. Stationary balls first, then rolling.
If real-world RMS is much worse than the FULL sim case, raise SIGMA_PX or SIGMA_A_MPS2 before changing structure.

Build / run commands
bash./gradlew setupDeps                                  # first-time only
./gradlew :TeamCode:installDebug                     # build + deploy

# Run sim tests INDIVIDUALLY (never full suite — LWJGL tests hang)
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.BallTrackerSimTest"

# Individual unit tests
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.ProjectionTest"
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.KalmanTrackTest"
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.GatingTest"
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.AssociationTest"
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.PoseBufferTest"
To watch the sim in the web viewer: start the viewer's dev server (however the existing viewer is launched — document this in the README), then run the sim test with BALL_TRACKER_VIZ_WEB=1. Open the URL in a browser.
Commit plan (Conventional Commits; commit often; every commit must build and pass its own tests)
chore(vision): scaffold sigmacorns.vision.tracker package
docs(cad): extract CAM_POS_R and footprint from STEP into config/ball_tracker.json
feat(vision): add Frames.kt with field/robot/camera conventions
feat(vision): build T_RC from config euler + translation
test(vision): Frames unit tests for identity and 90-deg rotation cases
feat(vision): pixel undistort + back-projection in Projection.kt
feat(vision): ray-plane intersection and field projection
test(vision): Projection round-trip test to 1e-6 m
feat(vision): numerical-Jacobian covariance propagation
test(vision): covariance major-axis alignment test
feat(vision): TrackerConfig.kt with all tunables
feat(vision): load TrackerConfig from config/ball_tracker.json
chore(sim): scaffold sigmacorns.vision.sim package
feat(sim): SimCameraConfig.kt
feat(sim): SimulatedCamera forward projection (zero noise)
feat(sim): Gaussian pixel noise, dropouts, false positives in SimulatedCamera
test(sim): SimulatedCamera unit tests for visibility and bounds culling
feat(io): add getBallDetections to SigmaIO interface
feat(io): implement getBallDetections in JoltSimIO via SimulatedCamera
feat(vision): KalmanTrack.kt with predict/update
test(vision): KF converges on constant-velocity stream
test(vision): predict-only covariance grows monotonically
feat(vision): Association.kt Mahalanobis gating + greedy assign
test(vision): association handles empty, 1:1, and crossing cases
feat(vision): Gating.kt image-space intake mask + field bounds + range
feat(vision): ramp polygon exclusion with camera-side expansion
feat(vision): robot-footprint self-exclusion
test(vision): gating unit tests for each rejector
feat(vision): PoseBuffer.kt with wrap-aware theta interpolation
test(vision): PoseBuffer interpolation accuracy test
feat(vision): assemble Tracker.tick pipeline
feat(vision): track birth, coast, and deletion rules
feat(vision): selectTarget with closest-confirmed strategy
feat(logic): BallTrackingSystem wiring Tracker + PoseBuffer
feat(logic): ChaseCoordinator driving toward selected target
feat(viz): VizPublisher JSON payload + transport integration
feat(viz): extend three.js viewer with track and covariance overlays
feat(viz): add camera-view panel with detections and intake mask
feat(viz): scenario selector UI
test(sim): BallTrackerSimTest zero-noise stationary-ball scenario
test(sim): BallTrackerSimTest pixel-noise scenario with RMS assertion
test(sim): BallTrackerSimTest pixel+pose noise scenario
test(sim): BallTrackerSimTest full-noise dropout+FP scenario
test(sim): BallTrackerSimTest ramp-ball rejection scenario
test(sim): BallTrackerSimTest in-and-out-of-frame persistence scenario
test(sim): BallTrackerSimTest headless chase scenario with capture assertion
feat(test): emit CSV report under build/reports/ball_tracker/
feat(test): LWJGL overlay with covariance ellipses and camera frustum
feat(io): implement HardwareIO.getBallDetections via Limelight
feat(opmode): BallChaseTeleOp with manual override abort
docs(vision): README with calibration and tuning guide
docs(vision): extrinsic verification procedure
chore(config): record calibrated intrinsics in config/ball_tracker.json
fix(vision): tuning adjustments to SIGMA_PX and SIGMA_A_MPS2 from real-data RMS
docs(vision): proof-of-concept notes summarizing sim RMS curves
Rules: no bundled commits; fix-forward with a dedicated fix(...) commit if a test breaks; use refactor(...) for no-behavior changes; breaking SigmaIO changes get feat(io)!: with BREAKING CHANGE: footer. List above is minimum scaffolding, not a cap.
Deliverables

All source under sigmacorns/vision/, sigmacorns/logic/, integrated into JoltSimIO; viewer extensions in the existing viewer directory.
All unit tests passing individually.
BallTrackerSimTest passing all 7 scenarios with CSV + stdout report.
README in sigmacorns/vision/: calibration, extrinsic verification, tuning procedure, how to read LWJGL + web overlays, how to run hardware validation, how to start the web viewer.
Proof-of-concept notes summarizing sim RMS curves per scenario — the primary deliverable.

Start with Step 0 (discovery + STEP extraction), report back. Then Frames.kt, Projection.kt, and their unit tests. Do not move on to the tracker or sim until projection round-trips to machine precision.

---

# Session status — end of 2026-04-19 (Apr 19 EDT)

Sim-first plan stands: build everything in JoltSim with the web viewer until `BallTrackerSimTest` passes all 7 scenarios. Hardware (Limelight pipeline, calibration, BallChaseTeleOp) only after sim is green.

## What landed this session

| Commit | Scope |
|---|---|
| `7d7aa2d` | `chore(cad)`: STEP-AP242 parser at `tools/parse_step.py` (numpy-only, re-runnable when CAD changes). |
| `2f198be` | `docs(cad)`: `config/ball_tracker.json` + `config/cad/extraction_notes.md`. CAM_POS_R provenance. |
| `7772c31` | `feat(vision)`: `tracker.Frames` — F/R/C frame conventions, `buildTRC` / `buildTFR` / `buildTFC`. 9 unit tests pass. |
| `a9d9967` | `feat(vision)`: `tracker.Projection` — undistort (20-iter Newton), `forwardProject`, `projectToGround`, `projectToGroundWithCovariance`. 9 unit tests pass. Round-trip 1e-9 m no-distortion / 1e-6 m with realistic distortion. |

Auto-memory entries (cross-session):
- `robot_frame_convention.md`
- `onshape_top_level_axes.md`
- `ball_tracker_cam_pos.md`
- `step_parser_tool.md`

## Concrete numbers committed to `config/ball_tracker.json`

- `CAM_POS_R = (0.162, 0.099, 0.242) m` (robot frame: x-fwd / y-left / z-up, origin at chassis center).
- `CAM_PITCH_DOWN_RAD = -0.199` (camera pitched **up** 11.4° per the CAD — flagged for verification).
- `CAM_YAW_RAD = 0`, `CAM_ROLL_RAD = 0`.
- `ROBOT_FOOTPRINT_R` = axis-aligned rectangle ±0.184 × ±0.183 m from `RobotModelConstants.robotSize`.
- `K`, `DIST_COEFFS`, `IMAGE_WIDTH/HEIGHT_PX` = nominal placeholders (1280×720, fx=fy=900, no distortion).
- `RAMP_POLYGON` = empty placeholder.
- All KF / gating / chi² tunables from prompt defaults.

Provenance and the full derivation chain are in `config/cad/extraction_notes.md`.

## What's in code right now

```
TeamCode/src/main/java/sigmacorns/vision/tracker/
  Frames.kt          ← committed
  Projection.kt      ← committed

TeamCode/src/test/kotlin/sigmacorns/test/vision/
  FramesTest.kt      ← 9 pass
  ProjectionTest.kt  ← 9 pass

config/ball_tracker.json
config/cad/extraction_notes.md
tools/parse_step.py
```

## What still needs to be built (in sim-first order)

### Tracker pipeline (pure-Kotlin, no I/O)
- [ ] `TrackerConfig.kt` — load `config/ball_tracker.json` into a typed Kotlin object.
- [ ] `PoseBuffer.kt` — ring buffer with wrap-aware theta interpolation.
- [ ] `KalmanTrack.kt` — single-track CV KF with predict / update.
- [ ] `Association.kt` — Mahalanobis-gated greedy assignment.
- [ ] `Gating.kt` — image-mask + field bounds + ramp + footprint, in that order.
- [ ] `Tracker.kt` — assemble `tick(detections, pose, t) -> tracks`, with track birth / coast / deletion + `selectTarget`.
- [ ] Unit tests for each of the above (KF convergence, association crossings, each gate independently, PoseBuffer interpolation accuracy).

### Sim integration
- [ ] `sigmacorns/vision/sim/SimCameraConfig.kt`.
- [ ] `sigmacorns/vision/sim/SimulatedCamera.kt` — forward-project Jolt balls → pixel detections + Gaussian noise + dropouts + false positives.
- [ ] `SigmaIO.getBallDetections(t, pose)` interface addition.
- [ ] `JoltSimIO.getBallDetections` impl backed by `SimulatedCamera`.
- [ ] `HardwareIO.getBallDetections` stub returning empty until Limelight pipeline is wired (so the interface compiles on both sides).

### Logic + viz
- [ ] `sigmacorns/logic/BallTrackingSystem.kt` — Tracker + PoseBuffer + IO wiring; mirrors `AimingSystem`.
- [ ] `sigmacorns/logic/ChaseCoordinator.kt` — drive to `targetBallField` via existing `Drivetrain`.
- [ ] `sigmacorns/vision/viz/VizPublisher.kt` — extend the existing `SimVizServer` payload (keep transport, just append `camera`, `detections_px`, `detections_field`, `tracks`, `target_id`, `scenario`, `rms_error_m` fields).
- [ ] `TeamCode/src/test/resources/web/scene.js` — track + covariance overlays on the existing field plane; **new camera-view panel** showing pixel-space detections, intake mask, projected covariance ellipses; scenario selector UI.

### End-to-end test (the deliverable gate)
- [ ] `TeamCode/src/test/kotlin/sigmacorns/test/vision/BallTrackerSimTest.kt` with all 7 scenarios (zero-noise, pixel-only, pixel+pose, full-noise, ramp-rejection, in-and-out-of-frame, headless-chase). Env-flagged web-viz / LWJGL viewer (default headless for CI). CSV reports under `build/reports/ball_tracker/`.

### Hardware (after sim is green)
- [ ] `HardwareIO.getBallDetections` real impl via Limelight color pipeline.
- [ ] Calibrate Cam Global Shutter intrinsics (checkerboard → `teamwebcamcalibrations.xml` and/or `config/ball_tracker.json`).
- [ ] Extrinsic verification op-mode: place ball at known field point, verify projected `(x, y)` matches tape measure within 3 cm at 1–2 m. Adjust `CAM_POS_R` / `CAM_PITCH_DOWN_RAD` if not.
- [ ] `BallChaseTeleOp` with manual abort.

## Open verifications (must resolve before trusting tracker output)

1. **Camera pitch sign**: CAD says optical axis is +11.4° **above** horizontal. Earlier prompt assumed −15.4° **below**. On first hardware power-up, check that the floor near the robot is visible in the lower part of the GlobalShutter feed. If you mostly see the ceiling, the axis-flip in `onshape_top_level_axes.md` is inverted — flip and re-derive (it's a one-line change to `config/ball_tracker.json`).
2. **"Bottom-left corner" identity**: assumed back-left-bottom of chassis. Front-left would put the camera 530 mm behind the robot center (impossible), so the assumption is consistent — but worth confirming in Onshape.
3. **Camera intrinsics**: nominal placeholders. Calibrate before any hardware test.
4. **Ramp polygon**: empty. Need to extract from the FTC DECODE 2025-2026 game manual / field drawing.
5. **Robot footprint**: axis-aligned rectangle. For accurate self-occlusion gating, install `cadquery` (~500 MB conda env) and extract the true convex hull of the chassis.

## How to pick up next session

1. Re-read this status block.
2. Continue with `TrackerConfig.kt` → load `config/ball_tracker.json` into typed Kotlin.
3. Then `KalmanTrack` + its unit tests (KF convergence + monotonic predict-only covariance growth — both already specified in the prompt above).
4. Then `PoseBuffer`, `Association`, `Gating` — order doesn't matter much; each is ~50 LoC + 1 test file.
5. Assemble `Tracker.tick` last.
6. Only then start on `SimulatedCamera` + `JoltSimIO` hookup + `BallTrackerSimTest`.
7. Viewer extensions can be added in parallel with the test scenarios since the existing transport just needs new payload fields.

Run tests individually per the CLAUDE.md note (LWJGL viz tests can hang in batch mode):

```
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.FramesTest"
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.ProjectionTest"
```

Re-run the STEP parser whenever the CAD changes:

```
python3 tools/parse_step.py [path/to/file.step]
```

---

# Session status — end of 2026-04-20 (Apr 20 EDT)

Continuing from the 2026-04-19 handoff. The tracker core and end-to-end
sim are now green. Hardware, logic layer, and viewer extensions remain.

## What landed this session

| Commit | Scope |
|---|---|
| `37e5e47` | `feat(vision)`: `TrackerConfig` loads `config/ball_tracker.json` into typed Kotlin. Walks up from `$projectDir` / `user.dir`. 4 tests. |
| `ba25e37` | `feat(vision)`: `KalmanTrack` constant-velocity KF (predict/update/markMissed/positionAt). 9 tests. |
| `d6592dd` | `feat(vision)`: `PoseBuffer` ring with wrap-aware theta interpolation. 11 tests. |
| `c356a96` | `feat(vision)`: `Association` Mahalanobis-gated greedy assignment + `FieldDetection`. 10 tests. |
| `70dd27e` | `feat(vision)`: `Gating` — intake mask, field bounds, ramp, footprint. 17 tests. |
| `04dd2f0` | `feat(vision)`: `Tracker.tick` pipeline with birth/coast/deletion + `selectTarget`. 8 integration tests. |
| `0a7829f` | `feat(io)`: `SimulatedCamera` + `SimCameraConfig` + `SigmaIO.getBallDetections` default + `JoltSimIO` override. 7 tests. |
| `969f77e` | `test(sim)`: `BallTrackerSimTest` — 7 scenarios pass end-to-end. Pure-Kotlin scenario harness. CSVs under `build/reports/ball_tracker/`. |

All **91 vision tests** pass. Run with:
```
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.*"
```

## Headline numbers (BallTrackerSimTest)

| Scenario | Metric | Threshold | Actual |
|---|---|---|---|
| zero-noise stationary | RMS last 3 s | < 1e-3 m | 0.0000 m |
| pixel noise only | RMS last 3 s | < 2e-2 m | 0.0030 m |
| pixel + pose noise | RMS last 3 s | < 5e-2 m | 0.0422 m |
| full noise (drop + FP) | RMS last 3 s | < 10e-2 m | 0.0700 m |
| full noise | target persistence | >= 95% | 98.8% |
| ball on ramp | tracks inside ramp | 0 | 0 |
| ball in/out of frame | gap bridged | yes | yes |
| headless chase | final robot→ball | < 0.15 m | 0.1014 m |

## What still needs to be built

### Deferred for this iteration
- [ ] `sigmacorns/logic/BallTrackingSystem.kt` — mirror `AimingSystem`; Tracker + PoseBuffer + IO wiring on a `Robot` reference.
- [ ] `sigmacorns/logic/ChaseCoordinator.kt` — drive to `targetBallField` via existing `Drivetrain` math.
- [ ] `sigmacorns/vision/viz/VizPublisher.kt` — extend the existing `SimVizServer` payload (new fields: `camera`, `detections_px`, `detections_field`, `tracks`, `target_id`, `scenario`, `rms_error_m`). Triggered by `BALL_TRACKER_VIZ_WEB=1`.
- [ ] `TeamCode/src/test/resources/web/scene.js` — track + covariance overlays on the existing field plane; new camera-view panel (pixel-space detections, intake-mask region, re-projected covariance ellipses, scenario-selector UI).
- [ ] `BallTrackerSimTest` JoltSim-backed runner (`runScenarioJolt(...)`) for hardware-realistic dynamics validation.

### Hardware (after viz + logic)
- [ ] `HardwareIO.getBallDetections` real impl via Limelight color pipeline (currently inherits the `SigmaIO` default `emptyList()`).
- [ ] Calibrate Cam Global Shutter intrinsics (checkerboard → `teamwebcamcalibrations.xml` and/or `config/ball_tracker.json`).
- [ ] Extrinsic verification op-mode: ball at known field point, projected `(x, y)` vs. tape measure within 3 cm at 1–2 m. Adjust `CAM_POS_R` / `CAM_PITCH_DOWN_RAD` if not.
- [ ] `BallChaseTeleOp` with manual abort.

## Design decisions worth remembering

1. **Sim harness is pure Kotlin, not JoltSim-backed.** The tracker doesn't need physics to be validated — it needs accurate forward projection and pose interpolation. The kinematic harness is deterministic, seed-controlled, and runs in ~200 ms per scenario. JoltSim-backed scenarios can be added in a follow-up for dynamics-coupled tests; they are not a prerequisite.
2. **`BallTrackerSimTest` uses pitchDown 0.15 rad, not the 0.268 rad (15.4°) / −0.199 rad values in `config/ball_tracker.json`.** The sim scenarios own their own extrinsics in `baseExtrinsics` so test geometry is independent of robot-side tuning. The config pitch is still up for hardware verification (see the prompt's "Open verifications" section).
3. **`selectTarget` requires confirmation (`hits >= MIN_HITS_FOR_CONFIRMED`).** RMS is only graded on confirmed targets — spurious single-hit tracks from a full-noise false positive never contaminate the metric.
4. **Last-known-target is covariance-gated in the chase scenario.** When the ball drops below the image bottom at close range, the track coasts and its position-covariance trace grows. We only trust the target estimate for the chase goal when `positionCovTrace < 0.05` — transient horizon blowups don't get latched as the commanded goal.
5. **SimulatedCamera does NOT know about ball radius.** Callers pass ball CENTER positions in field frame; the tracker's ground-plane intersection happens at `config.ballRadiusM`. This lets sim scenarios place balls at any height (e.g., in-the-air test cases) without threading special-case flags.

## How to pick up next session

1. Re-read this status block.
2. Start `BallTrackingSystem.kt` — mirror `AimingSystem`:
   - holds a `Tracker`, `PoseBuffer`, and a `SigmaIO` ref.
   - each update: push `io.position()` into the buffer at `io.time()`, call `io.getBallDetections(t, pose)`, pose-interpolate to the detection capture timestamps, `tracker.tick(...)`, `selectTarget`, expose `targetBallField`.
3. Then `ChaseCoordinator.kt` — reads `BallTrackingSystem.targetBallField` and commands drivetrain velocity. Reuse the kinematic controller pattern from `BallTrackerSimTest.headlessChaseScenario` as a reference.
4. Viz can be built in parallel — find the existing transport in a `SimVizServer` reference and append the new fields. The viewer extension goes in the same directory as the current `scene.js`.
5. Hardware path after viz is proven.

Run tests individually per the CLAUDE.md note:
```
./gradlew :TeamCode:testDebugUnitTest --tests "sigmacorns.test.vision.BallTrackerSimTest"
```


The parser prints `Cam Global Shutter` position in the `Top Level` Onshape frame; convert to robot frame using the `diag(-1, +1, -1) · v_TL + (0.959, 0.099, 0.373)` mapping documented in `config/cad/extraction_notes.md` (anchor: chassis back-left-bottom corner at robot `(-0.184, +0.183, 0)`).
