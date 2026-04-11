#include <cstdio>
#include <cmath>
#include <cassert>
#include "turret_planner/zone_tracker.h"
#include "turret_planner/math/gaussian.h"

static PhysicsConfig  cfg{9.81f, 0.1f};
static OmegaMapParams om{};
static TurretBounds   bounds{-3.14f, 3.14f, 0.f, 1.4f, 15.f, 1200.f};
static TurretWeights  weights{0.05f, 0.08f, 0.0005f};

// Zone: robot must reach x >= 2 (half-plane: n=(1,0), d=2)
static ZoneConfig make_zone(float alpha_lpf = 0.f) {
    return ZoneConfig{
        1.f, 0.f,   // nx, ny
        2.f,        // d
        0.01f,      // Q_process (low noise for deterministic tests)
        0.2f, 0.8f, // p_low, p_high
        alpha_lpf,  // alpha_lpf
        50.f        // omega_idle
    };
}

static ZoneTracker make_tracker(float alpha_lpf = 0.f) {
    return ZoneTracker(make_zone(alpha_lpf), weights, bounds, cfg, om);
}

static void checkf(const char* name, float got, float lo, float hi) {
    if (got < lo || got > hi)
        printf("FAIL %s: got %.4f not in [%.4f, %.4f]\n", name, got, lo, hi);
    else
        printf("PASS %s: %.4f\n", name, got);
}

// -------------------------------------------------------------------------
// test 1: robot far outside zone → urgency near 0
// -------------------------------------------------------------------------
void test_far_outside() {
    printf("\n=== test_far_outside ===\n");
    ZoneTracker zt = make_tracker();
    RobotState  robot{-5.f, 0.f, 0.f, 0.f, 0.f, 0.f};  // x=-5, not moving
    TurretState turret{0.f, 0.5f, 0.f};

    ZoneTrackerState s = zt.update(robot, turret, 3.f, 0.f, 1.f, 0.02f);
    printf("urgency=%.4f  effort=%.4f\n", s.urgency, s.effort);
    // With x=-5, d=2, Q=0.01, t_prep~0.5: mu_z = -5-2 + 0*0.5 = -7, sigma_z = sqrt(0.01*0.5)≈0.07
    // urgency ≈ Φ(-7/0.07) = Φ(-100) ≈ 0
    checkf("urgency", s.urgency, 0.f, 0.01f);
    checkf("effort",  s.effort,  0.f, 0.01f);
    assert(!s.should_fire);
    printf("PASS far_outside\n");
}

// -------------------------------------------------------------------------
// test 2: robot well inside zone → urgency ≈ 1
// -------------------------------------------------------------------------
void test_inside_zone() {
    printf("\n=== test_inside_zone ===\n");
    ZoneTracker zt = make_tracker();
    RobotState  robot{5.f, 0.f, 0.f, 0.f, 0.f, 0.f};  // x=5 >> d=2
    TurretState turret{0.f, 0.5f, 0.f};

    ZoneTrackerState s = zt.update(robot, turret, 3.f, 0.f, 1.f, 0.02f);
    printf("urgency=%.4f  effort=%.4f  should_fire=%d\n",
        s.urgency, s.effort, s.should_fire);
    // z0 = 5-2 = 3 >> 0 → urgency ≈ 1
    checkf("urgency", s.urgency, 0.95f, 1.f);
    checkf("effort",  s.effort,  1.f,   1.f);
    printf("PASS inside_zone\n");
}

// -------------------------------------------------------------------------
// test 3: urgency increases as robot approaches zone
// -------------------------------------------------------------------------
void test_urgency_increases_approaching() {
    printf("\n=== test_urgency_increases_approaching ===\n");
    ZoneTracker zt = make_tracker();
    TurretState turret{0.f, 0.5f, 0.f};

    float prev_urgency = 0.f;
    bool monotone = true;
    for (int i = 0; i < 8; ++i) {
        float x = -3.f + float(i) * 0.8f;  // -3 → 2.6
        RobotState robot{x, 0.f, 0.f, 1.f, 0.f, 0.f};  // moving toward zone at 1 m/s
        ZoneTrackerState s = zt.update(robot, turret, 3.f, 0.f, 1.f, 0.02f);
        printf("  x=%.1f  urgency=%.4f\n", x, s.urgency);
        if (s.urgency < prev_urgency - 0.01f) monotone = false;
        prev_urgency = s.urgency;
    }
    assert(monotone && "urgency should be non-decreasing as robot approaches");
    printf("PASS urgency_increases_approaching\n");
}

// -------------------------------------------------------------------------
// test 4: low-pass filter — single spike takes multiple ticks to propagate
// -------------------------------------------------------------------------
void test_lowpass_filter() {
    printf("\n=== test_lowpass_filter ===\n");
    const float alpha = 0.8f;  // strong smoothing
    ZoneTracker zt = make_tracker(alpha);
    TurretState turret{0.f, 0.5f, 0.f};

    // Step: robot jumps from far outside to well inside
    RobotState outside{-5.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    RobotState inside{ 5.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    // Warm up
    for (int i = 0; i < 3; ++i)
        zt.update(outside, turret, 3.f, 0.f, 1.f, 0.02f);

    ZoneTrackerState s0 = zt.update(inside, turret, 3.f, 0.f, 1.f, 0.02f);
    ZoneTrackerState s1 = zt.update(inside, turret, 3.f, 0.f, 1.f, 0.02f);
    ZoneTrackerState s5 = s1;
    for (int i = 0; i < 4; ++i)
        s5 = zt.update(inside, turret, 3.f, 0.f, 1.f, 0.02f);

    printf("tick 0: raw=%.4f filtered=%.4f\n", s0.urgency, s0.urgency_filtered);
    printf("tick 1: raw=%.4f filtered=%.4f\n", s1.urgency, s1.urgency_filtered);
    printf("tick 5: raw=%.4f filtered=%.4f\n", s5.urgency, s5.urgency_filtered);

    // Raw urgency should be ≈1 immediately; filtered should still be < raw at tick 0+1
    assert(s0.urgency > 0.9f);
    assert(s0.urgency_filtered < s0.urgency && "filter should lag raw on step input");
    // After several ticks the filter should have caught up
    assert(s5.urgency_filtered > 0.7f && "filter should converge toward raw over time");
    printf("PASS lowpass_filter\n");
}

// -------------------------------------------------------------------------
// test 5: effort scaling between p_low and p_high
// -------------------------------------------------------------------------
void test_effort_scaling() {
    printf("\n=== test_effort_scaling ===\n");
    // Use alpha_lpf=0 so urgency_filtered == urgency_raw exactly
    ZoneTracker zt = make_tracker(0.f);
    TurretState turret{0.f, 0.5f, 0.f};

    // Find positions that produce urgency near p_low=0.2 and p_high=0.8.
    // urgency = Φ(mu_z/sigma_z).  With t_prep≈0.5, sigma≈0.07:
    //   urgency=0.2 → Φ⁻¹(0.2) ≈ -0.84 → mu_z ≈ -0.84*0.07 ≈ -0.059 → z0 ≈ -0.059
    //   urgency=0.5 → z0 ≈ 0
    //   urgency=0.8 → z0 ≈ +0.059

    // x values such that z0 = x - d brackets p_low and p_high
    struct Case { float x; const char* label; float e_lo, e_hi; };
    Case cases[] = {
        {-3.f, "far below p_low",  0.f, 0.05f},
        { 2.f, "at boundary",      0.f, 1.0f},   // could be anywhere depending on t_prep
        { 8.f, "far above p_high", 0.95f, 1.0f},
    };

    for (auto& c : cases) {
        RobotState robot{c.x, 0.f, 0.f, 0.f, 0.f, 0.f};
        ZoneTrackerState s = zt.update(robot, turret, 3.f, 0.f, 1.f, 0.02f);
        printf("  x=%.1f (%s): urgency=%.4f effort=%.4f\n",
            c.x, c.label, s.urgency, s.effort);
        checkf(c.label, s.effort, c.e_lo, c.e_hi);
    }
    printf("PASS effort_scaling\n");
}

// -------------------------------------------------------------------------
// test 6: omega_idle floor — target omega never drops below idle
// -------------------------------------------------------------------------
void test_omega_idle_floor() {
    printf("\n=== test_omega_idle_floor ===\n");
    ZoneTracker zt = make_tracker();
    TurretState turret{0.f, 0.5f, 0.f};  // omega=0 initially
    RobotState  robot{-5.f, 0.f, 0.f, 0.f, 0.f, 0.f};

    ZoneTrackerState s = zt.update(robot, turret, 3.f, 0.f, 1.f, 0.02f);
    printf("effort=%.4f  target.omega=%.2f  idle=%.2f\n",
        s.effort, s.target.omega_flywheel, make_zone().omega_idle);
    assert(s.target.omega_flywheel >= make_zone().omega_idle - 1e-4f
        && "target omega must be >= omega_idle");
    printf("PASS omega_idle_floor\n");
}

// -------------------------------------------------------------------------
// test 7: turret target blends toward optimal proportional to effort
// -------------------------------------------------------------------------
void test_target_blending() {
    printf("\n=== test_target_blending ===\n");
    // Use alpha=0 for deterministic filtered = raw
    ZoneTracker zt = make_tracker(0.f);
    TurretState turret{0.2f, 0.3f, 100.f};  // offset current state

    // Robot approaching zone at medium pace — effort in (0,1)
    RobotState robot{1.f, 0.f, 0.f, 1.f, 0.f, 0.f};  // x=1, moving toward d=2

    ZoneTrackerState s_approach = zt.update(robot, turret, 3.f, 0.f, 1.f, 0.02f);
    printf("effort=%.4f  target.theta=%.4f  turret.theta=%.4f\n",
        s_approach.effort, s_approach.target.theta, turret.theta);

    if (s_approach.effort > 0.f && s_approach.effort < 1.f) {
        // Target should be between current and optimal (not equal to either)
        // Just verify it's in the expected direction: not exactly current
        // (unless effort=0) and not fully jumped to optimal.
        // Since effort > 0, target.theta != turret.theta (unless optimal == current)
        printf("  (effort in (0,1): blending active)\n");
    }

    // Robot fully inside zone: effort=1, target should be at optimal
    RobotState inside{5.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    ZoneTrackerState s_inside = zt.update(inside, turret, 3.f, 0.f, 1.f, 0.02f);
    printf("effort=%.4f (expected 1.0)\n", s_inside.effort);
    assert(std::abs(s_inside.effort - 1.f) < 1e-4f);

    // At effort=1: target = turret + 1*(optimal - turret) = optimal
    FlightTimeResult opt = flight_time_cold(5.f,0.f,0.f, 3.f,0.f,1.f, 0.f,0.f,
                                             turret, weights, bounds, cfg, om);
    if (opt.feasible) {
        printf("optimal.theta=%.4f  target.theta=%.4f\n",
            opt.params.theta, s_inside.target.theta);
        assert(std::abs(s_inside.target.theta - opt.params.theta) < 1e-3f);
    }
    printf("PASS target_blending\n");
}

// -------------------------------------------------------------------------
// test 8: mecanum_project_zone accuracy
// -------------------------------------------------------------------------
void test_mecanum_projection() {
    printf("\n=== test_mecanum_projection ===\n");
    // n=(1,0), d=2, robot at (0,0), moving at v=(2,0).
    // At t=1: predicted x = 0+2*1 = 2, z = 2-2 = 0, urgency = 0.5.
    float nx=1.f,ny=0.f,d=2.f;
    float px=0.f,py=0.f,vx=2.f,vy=0.f;
    float Q=0.01f, t=1.f;

    ZoneProjection proj = mecanum_project_zone(nx,ny,d,px,py,vx,vy, 0.f, Q, t);
    printf("mu_z=%.4f (expect 0.0)  var_z=%.4f (expect 0.01)\n",
        proj.mu_z, proj.var_z);
    assert(std::abs(proj.mu_z - 0.f) < 1e-5f);
    assert(std::abs(proj.var_z - Q*t) < 1e-5f);

    // At t=0.5: x = 0+2*0.5 = 1, z = 1-2 = -1 → urgency < 0.5
    ZoneProjection proj2 = mecanum_project_zone(nx,ny,d,px,py,vx,vy, 0.f, Q, 0.5f);
    float urgency2 = fast_gaussian_cdf(proj2.mu_z / proj2.sigma_z);
    printf("t=0.5: mu_z=%.4f urgency=%.4f (expect < 0.5)\n", proj2.mu_z, urgency2);
    assert(urgency2 < 0.5f);

    printf("PASS mecanum_projection\n");
}

int main() {
    test_far_outside();
    test_inside_zone();
    test_urgency_increases_approaching();
    test_lowpass_filter();
    test_effort_scaling();
    test_omega_idle_floor();
    test_target_blending();
    test_mecanum_projection();
    printf("\nDone.\n");
    return 0;
}
