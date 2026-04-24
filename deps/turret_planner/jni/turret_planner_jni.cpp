#include <jni.h>
#include <cstring>
#include <new>

#include "turret_planner/ballistics.h"
#include "turret_planner/flight_time.h"
#include "turret_planner/robust_shot.h"
#include "turret_planner/robust_3shot.h"
#include "turret_planner/path_scan.h"
#include "turret_planner/preposition.h"
#include "turret_planner/zone_tracker.h"
#include "turret_planner/mecanum_model.h"

// ---------------------------------------------------------------------------
// Config-array layouts (match TurretPlannerBridge.kt constants)
//   physConfig  [3]:  g, rH, dragK
//   bounds      [6]:  thetaMin, thetaMax, phiMin, phiMax, vExitMax, omegaMax
//   weights     [3]:  wTheta, wPhi, wOmega
//   omegaCoeffs [6]:  c0..c5
//   zoneConfig  [8]:  nx, ny, d, Q_process, p_low, p_high, alpha_lpf, omega_idle
//
// Path layout (5 floats per sample): t, x, y, vx, vy
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline PhysicsConfig unpack_physics(const jfloat* p) {
    return PhysicsConfig{p[0], p[1], p[2]};
}

static inline TurretBounds unpack_bounds(const jfloat* p) {
    return TurretBounds{p[0], p[1], p[2], p[3], p[4], p[5]};
}

static inline TurretWeights unpack_weights(const jfloat* p) {
    return TurretWeights{p[0], p[1], p[2], p[3]};
}

// Reads [N, phi_0, v_exit_0, omega_0, phi_1, ...] into OmegaMapParams for IDW.
static inline OmegaMapParams unpack_omega(const jfloat* p) {
    OmegaMapParams om;
    int n = static_cast<int>(p[0]);
    n = std::min(n, OmegaMapParams::MAX_POINTS);
    om.n = n;
    float phi_min = 1e9f, phi_max = -1e9f;
    float v_min   = 1e9f, v_max   = -1e9f;
    for (int i = 0; i < n; ++i) {
        om.phi[i]    = p[1 + i*3 + 0];
        om.v_exit[i] = p[1 + i*3 + 1];
        om.omega[i]  = p[1 + i*3 + 2];
        phi_min = std::min(phi_min, om.phi[i]);
        phi_max = std::max(phi_max, om.phi[i]);
        v_min   = std::min(v_min,   om.v_exit[i]);
        v_max   = std::max(v_max,   om.v_exit[i]);
    }
    om.phi_scale = (phi_max > phi_min + 1e-6f) ? 1.f/(phi_max - phi_min) : 1.f;
    om.v_scale   = (v_max   > v_min   + 1e-6f) ? 1.f/(v_max   - v_min)   : 1.f;
    return om;
}

static inline ZoneConfig unpack_zone(const jfloat* p) {
    return ZoneConfig{p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]};
}

static inline TurretState unpack_turret(float theta, float phi, float omega) {
    return TurretState{theta, phi, omega};
}

// Scoped float-array accessor.
//
// NOTE: we intentionally use GetFloatArrayElements / ReleaseFloatArrayElements
// rather than GetPrimitiveArrayCritical. The critical variants forbid any
// other JNI calls while the region is held, which is incompatible with
// allocating the result array (NewFloatArray) before releasing inputs.
// GetFloatArrayElements returns a pinned (or copied) pointer that remains
// valid across other JNI calls.
struct ScopedArray {
    JNIEnv*     env;
    jfloatArray arr;
    jfloat*     ptr;
    ScopedArray(JNIEnv* e, jfloatArray a)
        : env(e), arr(a)
        , ptr(a ? e->GetFloatArrayElements(a, nullptr) : nullptr) {}
    ~ScopedArray() { if (ptr) env->ReleaseFloatArrayElements(arr, ptr, JNI_ABORT); }
    operator jfloat*() const { return ptr; }
};

// Create a float array result on the JVM heap and fill it from a local buffer.
static jfloatArray make_result(JNIEnv* env, const float* buf, int n) {
    jfloatArray arr = env->NewFloatArray(n);
    if (arr) env->SetFloatArrayRegion(arr, 0, n, buf);
    return arr;
}

// ---------------------------------------------------------------------------
// JNI: solve
// Returns [theta, phi, vExit, omega]
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_solve(
    JNIEnv* env, jobject,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat targetX, jfloat targetY, jfloat targetZ,
    jfloat robotVx, jfloat robotVy,
    jfloat T,
    jfloatArray jPhys,
    jfloatArray jOmega)
{
    ScopedArray phys(env, jPhys);
    ScopedArray omC(env, jOmega);
    if (!phys || !omC) return nullptr;

    PhysicsConfig  cfg = unpack_physics(phys);
    OmegaMapParams om  = unpack_omega(omC);

    ShotParams p = ballistics_solve(
        turretX, turretY, turretZ,
        targetX, targetY, targetZ,
        robotVx, robotVy, T, cfg, om);

    float buf[4] = {p.theta, p.phi, p.v_exit, p.omega_flywheel};
    return make_result(env, buf, 4);
}

// ---------------------------------------------------------------------------
// JNI: shotError
// Forward-simulate a shot and return the horizontal miss distance (m).
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloat JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_shotError(
    JNIEnv* env, jobject,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat targetX, jfloat targetY, jfloat targetZ,
    jfloat robotVx, jfloat robotVy,
    jfloat theta, jfloat phi, jfloat vExit, jfloat omega,
    jfloat T,
    jfloatArray jPhys)
{
    ScopedArray ph(env, jPhys);
    if (!ph) return 1e9f;
    ShotParams p{theta, phi, vExit, omega};
    return ballistics_shot_error(
        turretX, turretY, turretZ,
        targetX, targetY, targetZ,
        robotVx, robotVy, p, T,
        unpack_physics(ph));
}

// ---------------------------------------------------------------------------
// JNI: optimalTCold
// Returns [T*, tau, theta, phi, vExit, omega, feasible(0|1)]
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_optimalTCold(
    JNIEnv* env, jobject,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat targetX, jfloat targetY, jfloat targetZ,
    jfloat robotVx, jfloat robotVy,
    jfloat curTheta, jfloat curPhi, jfloat curOmega,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega,
    jfloat tol)
{
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!w || !b || !ph || !om) return nullptr;

    FlightTimeResult r = flight_time_cold(
        turretX, turretY, turretZ,
        targetX, targetY, targetZ,
        robotVx, robotVy,
        unpack_turret(curTheta, curPhi, curOmega),
        unpack_weights(w), unpack_bounds(b), unpack_physics(ph), unpack_omega(om),
        tol);

    float buf[7] = {r.T_star, r.tau,
                    r.params.theta, r.params.phi, r.params.v_exit,
                    r.params.omega_flywheel,
                    r.feasible ? 1.f : 0.f};
    return make_result(env, buf, 7);
}

// ---------------------------------------------------------------------------
// JNI: optimalTWarm
// Returns [T*, tau, theta, phi, vExit, omega, feasible(0|1)]
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_optimalTWarm(
    JNIEnv* env, jobject,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat targetX, jfloat targetY, jfloat targetZ,
    jfloat robotVx, jfloat robotVy,
    jfloat TInit,
    jfloat curTheta, jfloat curPhi, jfloat curOmega,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega)
{
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!w || !b || !ph || !om) return nullptr;

    FlightTimeResult r = flight_time_warm(
        turretX, turretY, turretZ,
        targetX, targetY, targetZ,
        robotVx, robotVy, TInit,
        unpack_turret(curTheta, curPhi, curOmega),
        unpack_weights(w), unpack_bounds(b), unpack_physics(ph), unpack_omega(om));

    float buf[7] = {r.T_star, r.tau,
                    r.params.theta, r.params.phi, r.params.v_exit,
                    r.params.omega_flywheel,
                    r.feasible ? 1.f : 0.f};
    return make_result(env, buf, 7);
}

// ---------------------------------------------------------------------------
// JNI: findEarliestShot
// path: flat FloatArray, 5 floats/sample [t, x, y, vx, vy]
// Returns [found(0|1), tPath, T*, theta, phi, vExit, omega]
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_findEarliestShot(
    JNIEnv* env, jobject,
    jfloatArray jPath, jint nSamples,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat robotVx, jfloat robotVy,
    jfloat targetZ,
    jfloat tNow,
    jfloat curTheta, jfloat curPhi, jfloat curOmega,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega)
{
    ScopedArray path(env, jPath);
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!path || !w || !b || !ph || !om) return nullptr;

    // PathSample is 5 floats: t, x, y, vx, vy — same layout as our flat array.
    static_assert(sizeof(PathSample) == 5 * sizeof(float), "PathSample layout mismatch");
    const PathSample* samples = reinterpret_cast<const PathSample*>(path.ptr);

    EarliestShotResult r = path_scan_earliest(
        samples, (int)nSamples,
        turretX, turretY, turretZ,
        robotVx, robotVy, targetZ, tNow,
        unpack_turret(curTheta, curPhi, curOmega),
        unpack_weights(w), unpack_bounds(b), unpack_physics(ph), unpack_omega(om));

    float buf[7] = {r.found ? 1.f : 0.f, r.t_path, r.T_star,
                    r.params.theta, r.params.phi, r.params.v_exit,
                    r.params.omega_flywheel};
    return make_result(env, buf, 7);
}

// ---------------------------------------------------------------------------
// JNI: computePreposition
// Returns [theta, phi, omega, expectedEarliestT]
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_computePreposition(
    JNIEnv* env, jobject,
    jfloatArray jPath, jint nSamples,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat robotVx, jfloat robotVy,
    jfloat targetZ,
    jfloat curTheta, jfloat curPhi, jfloat curOmega,
    jfloat tAvailable,
    jfloat lambdaDecay,
    jint   kSamples,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega)
{
    ScopedArray path(env, jPath);
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!path || !w || !b || !ph || !om) return nullptr;

    const PathSample* samples = reinterpret_cast<const PathSample*>(path.ptr);

    PrepositionResult r = preposition_compute(
        samples, (int)nSamples,
        turretX, turretY, turretZ,
        robotVx, robotVy, targetZ,
        unpack_turret(curTheta, curPhi, curOmega),
        tAvailable, lambdaDecay, (int)kSamples,
        unpack_weights(w), unpack_bounds(b), unpack_physics(ph), unpack_omega(om));

    float buf[4] = {r.target.theta, r.target.phi, r.target.omega_flywheel,
                    r.expected_earliest_t};
    return make_result(env, buf, 4);
}

// ---------------------------------------------------------------------------
// JNI: robustShot
// Returns [feasible, T1, T2, J,
//          s1.theta, s1.phi, s1.vExit, s1.omega,
//          s2.theta, s2.phi, s2.vExit, s2.omega]
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_robustShot(
    JNIEnv* env, jobject,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat t1X, jfloat t1Y, jfloat t1Z,
    jfloat t2X, jfloat t2Y, jfloat t2Z,
    jfloat robotVx, jfloat robotVy,
    jfloat dropFraction,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega,
    jfloat tol,
    jint   maxIter)
{
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!w || !b || !ph || !om) return nullptr;

    RobustShotResult r = flight_time_robust(
        turretX, turretY, turretZ,
        t1X, t1Y, t1Z,
        t2X, t2Y, t2Z,
        robotVx, robotVy, dropFraction,
        unpack_weights(w), unpack_bounds(b),
        unpack_physics(ph), unpack_omega(om),
        tol, (int)maxIter);

    float buf[12] = {
        r.feasible ? 1.f : 0.f,
        r.T1, r.T2, r.J,
        r.s1.theta, r.s1.phi, r.s1.v_exit, r.s1.omega_flywheel,
        r.s2.theta, r.s2.phi, r.s2.v_exit, r.s2.omega_flywheel
    };
    return make_result(env, buf, 12);
}

// ---------------------------------------------------------------------------
// JNI: robustAdjust
// Same return shape as robustShot. Minimizes
//   J = J_Δ(cur → s1) + J_Δ(s1_reduced → s2)
// so the caller can plan a pair of shots that minimize total time to fire
// both balls from a known current turret state.
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_robustAdjust(
    JNIEnv* env, jobject,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat t1X, jfloat t1Y, jfloat t1Z,
    jfloat t2X, jfloat t2Y, jfloat t2Z,
    jfloat robotVx, jfloat robotVy,
    jfloat curTheta, jfloat curPhi, jfloat curOmega,
    jfloat dropFraction,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega,
    jfloat tol,
    jint   maxIter)
{
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!w || !b || !ph || !om) return nullptr;

    RobustShotResult r = flight_time_robust_adjust(
        turretX, turretY, turretZ,
        t1X, t1Y, t1Z,
        t2X, t2Y, t2Z,
        robotVx, robotVy,
        unpack_turret(curTheta, curPhi, curOmega),
        dropFraction,
        unpack_weights(w), unpack_bounds(b),
        unpack_physics(ph), unpack_omega(om),
        tol, (int)maxIter);

    float buf[12] = {
        r.feasible ? 1.f : 0.f,
        r.T1, r.T2, r.J,
        r.s1.theta, r.s1.phi, r.s1.v_exit, r.s1.omega_flywheel,
        r.s2.theta, r.s2.phi, r.s2.v_exit, r.s2.omega_flywheel
    };
    return make_result(env, buf, 12);
}

// ---------------------------------------------------------------------------
// JNI: computeRobustPreposition
// Same return shape as computePreposition: [theta, phi, omega, expectedEarliestT]
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_computeRobustPreposition(
    JNIEnv* env, jobject,
    jfloatArray jPath, jint nSamples,
    jfloat turretX, jfloat turretY, jfloat turretZ,
    jfloat robotVx, jfloat robotVy,
    jfloat targetZ,
    jfloat curTheta, jfloat curPhi, jfloat curOmega,
    jfloat tAvailable,
    jfloat lambdaDecay,
    jint   kSamples,
    jfloat dropFraction,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega)
{
    ScopedArray path(env, jPath);
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!path || !w || !b || !ph || !om) return nullptr;

    const PathSample* samples = reinterpret_cast<const PathSample*>(path.ptr);

    PrepositionResult r = preposition_robust_compute(
        samples, (int)nSamples,
        turretX, turretY, turretZ,
        robotVx, robotVy, targetZ,
        unpack_turret(curTheta, curPhi, curOmega),
        tAvailable, lambdaDecay, (int)kSamples, dropFraction,
        unpack_weights(w), unpack_bounds(b), unpack_physics(ph), unpack_omega(om));

    float buf[4] = {r.target.theta, r.target.phi, r.target.omega_flywheel,
                    r.expected_earliest_t};
    return make_result(env, buf, 4);
}

// ---------------------------------------------------------------------------
// JNI: robust3ShotPlan
// trajectory: flat FloatArray, 6 floats/state [t, x, y, heading, vx, vy]
// Returns [feasible, idx1, idx2, idx3, J, J_12, J_23, T1, T2, T3,
//          s1.theta, s1.phi, s1.vExit, s1.omega,
//          s2.theta, s2.phi, s2.vExit, s2.omega,
//          s3.theta, s3.phi, s3.vExit, s3.omega]   = 22 floats
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_robust3ShotPlan(
    JNIEnv* env, jobject,
    jfloatArray jTrajectory, jint nStates,
    jfloat turretZ,
    jfloat targetX, jfloat targetY, jfloat targetZ,
    jfloat curTheta, jfloat curPhi, jfloat curOmega,
    jint   nBalls,
    jfloat tRemaining,
    jfloat transferTime,
    jfloat dropFraction,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega,
    jfloat tol,
    jint   maxIter)
{
    ScopedArray traj(env, jTrajectory);
    ScopedArray w(env, jWeights), b(env, jBounds),
                        ph(env, jPhys),   om(env, jOmega);
    if (!traj || !w || !b || !ph || !om) return nullptr;

    static_assert(sizeof(FutureState) == 6 * sizeof(float), "FutureState layout mismatch");
    const FutureState* states = reinterpret_cast<const FutureState*>(traj.ptr);

    Robust3ShotResult r = robust_3shot_plan(
        states, (int)nStates,
        turretZ,
        targetX, targetY, targetZ,
        unpack_turret(curTheta, curPhi, curOmega),
        (int)nBalls,
        tRemaining, transferTime, dropFraction,
        unpack_weights(w), unpack_bounds(b),
        unpack_physics(ph), unpack_omega(om),
        tol, (int)maxIter);

    float buf[22] = {
        r.feasible ? 1.f : 0.f,
        float(r.idx1), float(r.idx2), float(r.idx3),
        r.J, r.J_12, r.J_23,
        r.T1, r.T2, r.T3,
        r.s1.theta, r.s1.phi, r.s1.v_exit, r.s1.omega_flywheel,
        r.s2.theta, r.s2.phi, r.s2.v_exit, r.s2.omega_flywheel,
        r.s3.theta, r.s3.phi, r.s3.v_exit, r.s3.omega_flywheel
    };
    return make_result(env, buf, 22);
}

// ---------------------------------------------------------------------------
// ZoneTracker native handle
// ---------------------------------------------------------------------------

extern "C" JNIEXPORT jlong JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_createZoneTracker(
    JNIEnv* env, jobject,
    jfloatArray jZone,
    jfloatArray jWeights,
    jfloatArray jBounds,
    jfloatArray jPhys,
    jfloatArray jOmega)
{
    ScopedArray z(env, jZone), w(env, jWeights),
                        b(env, jBounds), ph(env, jPhys), om(env, jOmega);
    if (!z || !w || !b || !ph || !om) return 0L;

    ZoneTracker* tracker = new(std::nothrow) ZoneTracker(
        unpack_zone(z),
        unpack_weights(w),
        unpack_bounds(b),
        unpack_physics(ph),
        unpack_omega(om));

    return reinterpret_cast<jlong>(tracker);
}

// Returns [urgency, urgencyFiltered, effort, tau, targetTheta, targetPhi, targetOmega, shouldFire(0|1)]
extern "C" JNIEXPORT jfloatArray JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_updateZoneTracker(
    JNIEnv* env, jobject,
    jlong handle,
    jfloat robotX,   jfloat robotY,   jfloat robotHeading,
    jfloat robotVx,  jfloat robotVy,  jfloat robotOmega,
    jfloat turretTheta, jfloat turretPhi, jfloat turretOmega,
    jfloat targetX, jfloat targetY, jfloat targetZ,
    jfloat dt)
{
    ZoneTracker* tracker = reinterpret_cast<ZoneTracker*>(handle);
    if (!tracker) return nullptr;

    RobotState robot{robotX, robotY, robotHeading, robotVx, robotVy, robotOmega};
    TurretState turret{turretTheta, turretPhi, turretOmega};

    ZoneTrackerState s = tracker->update(robot, turret, targetX, targetY, targetZ, dt);

    float buf[8] = {
        s.urgency, s.urgency_filtered, s.effort, s.tau,
        s.target.theta, s.target.phi, s.target.omega_flywheel,
        s.should_fire ? 1.f : 0.f
    };
    return make_result(env, buf, 8);
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_resetZoneTracker(
    JNIEnv*, jobject,
    jlong handle)
{
    ZoneTracker* tracker = reinterpret_cast<ZoneTracker*>(handle);
    if (tracker) tracker->reset();
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_destroyZoneTracker(
    JNIEnv*, jobject,
    jlong handle)
{
    delete reinterpret_cast<ZoneTracker*>(handle);
}

// ---------------------------------------------------------------------------
// JNI: omegaMapEval
// Evaluate the IDW omega map at (phi, vExit). Returns the interpolated omega.
// ---------------------------------------------------------------------------
extern "C" JNIEXPORT jfloat JNICALL
Java_sigmacorns_control_aim_TurretPlannerBridge_omegaMapEval(
    JNIEnv* env, jobject,
    jfloat phi, jfloat vExit,
    jfloatArray jOmega)
{
    ScopedArray omC(env, jOmega);
    if (!omC) return 0.f;
    OmegaMapParams om = unpack_omega(omC);
    return omega_map_eval(om, phi, vExit);
}
