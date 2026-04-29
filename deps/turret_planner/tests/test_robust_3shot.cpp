#include <cstdio>
#include <cmath>
#include <cassert>
#include <algorithm>
#include "turret_planner/robust_3shot.h"
#include "turret_planner/ballistics.h"
#include "turret_planner/flywheel_model.h"

// Use a non-trivial omega map so the flywheel-drop arm actually matters:
//   omega = 800 + 40*v + 50*phi
static PhysicsConfig  cfg{9.81f, 0.1f, 0.f};
static OmegaMapParams make_om3() {
    OmegaMapParams m{};
    m.n = 4;
    m.phi[0] = 0.3f; m.v_exit[0] = 5.f; m.omega[0] = 800.f + 40.f*5.f + 50.f*0.3f;
    m.phi[1] = 0.3f; m.v_exit[1] = 9.f; m.omega[1] = 800.f + 40.f*9.f + 50.f*0.3f;
    m.phi[2] = 1.2f; m.v_exit[2] = 5.f; m.omega[2] = 800.f + 40.f*5.f + 50.f*1.2f;
    m.phi[3] = 1.2f; m.v_exit[3] = 9.f; m.omega[3] = 800.f + 40.f*9.f + 50.f*1.2f;
    m.phi_scale = 1.f / (1.2f - 0.3f);
    m.v_scale   = 1.f / (9.f  - 5.f);
    return m;
}
static OmegaMapParams om = make_om3();
static TurretBounds   bounds{-3.14f, 3.14f, 0.f, 1.4f, 15.f, 2000.f};
static TurretWeights  weights{0.05f, 0.08f, 0.0005f};

// Build a constant-velocity trajectory
static void make_trajectory(FutureState* out, int n,
                            float x0, float y0, float vx, float vy,
                            float t_start, float dt) {
    for (int i = 0; i < n; ++i) {
        float t = t_start + i * dt;
        out[i].t = t;
        out[i].x = x0 + vx * t;
        out[i].y = y0 + vy * t;
        out[i].heading = 0.f;
        out[i].vx = vx;
        out[i].vy = vy;
    }
}

void test_static_1ball() {
    printf("\n=== test_static_1ball ===\n");
    const int N = 20;
    FutureState traj[N];
    make_trajectory(traj, N, 0.f, 0.f, 0.f, 0.f, 0.f, 0.05f);

    TurretState cur{0.f, 0.3f, 900.f};
    float target_z = 1.5f;

    Robust3ShotResult r = robust_3shot_plan(
        traj, N, 0.f,
        3.f, 0.f, target_z,
        cur, 1, 0.f, 0.2f, 0.005f,
        weights, bounds, cfg, om);

    printf("feasible=%d  idx1=%d  T1=%.4f  J=%.6f\n",
           r.feasible, r.idx1, r.T1, r.J);
    printf("  s1: theta=%.4f phi=%.4f vExit=%.3f\n",
           r.s1.theta, r.s1.phi, r.s1.v_exit);

    assert(r.feasible);
    // With t_remaining=0, solver can pick any trajectory sample
    assert(r.idx1 >= 0);
    printf("PASS static_1ball\n");
}

void test_static_2ball() {
    printf("\n=== test_static_2ball ===\n");
    const int N = 30;
    FutureState traj[N];
    make_trajectory(traj, N, 0.f, 0.f, 0.f, 0.f, 0.f, 0.05f);

    TurretState cur{0.f, 0.3f, 900.f};
    float target_z = 1.5f;

    Robust3ShotResult r = robust_3shot_plan(
        traj, N, 0.f,
        3.f, 0.f, target_z,
        cur, 2, 0.f, 0.2f, 0.005f,
        weights, bounds, cfg, om);

    printf("feasible=%d  idx1=%d  T1=%.4f T2=%.4f  J=%.6f\n",
           r.feasible, r.idx1, r.T1, r.T2, r.J);
    printf("  s1: theta=%.4f phi=%.4f vExit=%.3f omega=%.2f\n",
           r.s1.theta, r.s1.phi, r.s1.v_exit, r.s1.omega_flywheel);
    printf("  s2: theta=%.4f phi=%.4f vExit=%.3f omega=%.2f\n",
           r.s2.theta, r.s2.phi, r.s2.v_exit, r.s2.omega_flywheel);

    assert(r.feasible);
    printf("PASS static_2ball\n");
}

void test_static_3ball() {
    printf("\n=== test_static_3ball ===\n");
    const int N = 40;
    FutureState traj[N];
    make_trajectory(traj, N, 0.f, 0.f, 0.f, 0.f, 0.f, 0.05f);

    TurretState cur{0.f, 0.3f, 900.f};
    float target_z = 1.5f;

    Robust3ShotResult r = robust_3shot_plan(
        traj, N, 0.f,
        3.f, 0.f, target_z,
        cur, 3, 0.f, 0.2f, 0.005f,
        weights, bounds, cfg, om);

    printf("feasible=%d  T1=%.4f T2=%.4f T3=%.4f  J=%.6f\n",
           r.feasible, r.T1, r.T2, r.T3, r.J);
    printf("  s1: theta=%.4f phi=%.4f vExit=%.3f omega=%.2f\n",
           r.s1.theta, r.s1.phi, r.s1.v_exit, r.s1.omega_flywheel);
    printf("  s2: theta=%.4f phi=%.4f vExit=%.3f omega=%.2f\n",
           r.s2.theta, r.s2.phi, r.s2.v_exit, r.s2.omega_flywheel);
    printf("  s3: theta=%.4f phi=%.4f vExit=%.3f omega=%.2f\n",
           r.s3.theta, r.s3.phi, r.s3.v_exit, r.s3.omega_flywheel);

    assert(r.feasible);
    printf("PASS static_3ball\n");
}

void test_moving_trajectory() {
    printf("\n=== test_moving_trajectory ===\n");
    // Robot starts 5m away and drives toward the target at 0.5 m/s.
    // Shots should be delayed to get closer (better geometry).
    const int N = 40;
    FutureState traj[N];
    // Moving in +x toward target at (3, 0)
    make_trajectory(traj, N, -2.f, 0.f, 0.5f, 0.f, 0.f, 0.05f);

    TurretState cur{0.f, 0.3f, 900.f};
    float target_z = 1.5f;

    Robust3ShotResult r = robust_3shot_plan(
        traj, N, 0.f,
        3.f, 0.f, target_z,
        cur, 2, 0.f, 0.2f, 0.005f,
        weights, bounds, cfg, om);

    printf("feasible=%d  idx1=%d  t1=%.3f  J=%.6f\n",
           r.feasible, r.idx1, r.idx1 >= 0 ? traj[r.idx1].t : -1.f, r.J);

    assert(r.feasible);
    printf("PASS moving_trajectory\n");
}

void test_transfer_timing() {
    printf("\n=== test_transfer_timing ===\n");
    // Verify that shot timing respects t_remaining constraint.
    // t_remaining = time until next ball can exit = 0.3s
    const int N = 40;
    FutureState traj[N];
    make_trajectory(traj, N, 0.f, 0.f, 0.f, 0.f, 0.f, 0.05f);

    TurretState cur{0.f, 0.3f, 900.f};
    float transfer_time = 0.2f;
    float t_remaining = 0.3f;  // next ball exits in 0.3s

    Robust3ShotResult r = robust_3shot_plan(
        traj, N, 0.f,
        3.f, 0.f, 1.5f,
        cur, 3, t_remaining, transfer_time, 0.005f,
        weights, bounds, cfg, om);

    if (r.feasible) {
        float t1 = traj[r.idx1].t;
        printf("t_remaining=%.2f\n", t_remaining);
        printf("shot1 at t=%.3f (min %.3f)\n", t1, t_remaining);
        assert(t1 >= t_remaining - 1e-3f);
    }
    printf("PASS transfer_timing\n");
}

void test_no_backward_shot() {
    printf("\n=== test_no_backward_shot ===\n");
    // Robot is close to the target and moving toward it at high speed.
    // Without the directional constraint, the solver could find a backwards-
    // pointing solution where theta ≈ π (turret faces away from goal).
    const int N = 40;
    FutureState traj[N];
    // Robot at (1, 0) moving at vx=2 m/s toward target at (3, 0).
    // At T > 0.5s the velocity correction can flip theta.
    make_trajectory(traj, N, 1.f, 0.f, 2.f, 0.f, 0.f, 0.05f);

    TurretState cur{0.f, 0.3f, 900.f};
    float target_z = 1.5f;

    Robust3ShotResult r = robust_3shot_plan(
        traj, N, 0.f,
        3.f, 0.f, target_z,
        cur, 1, 0.f, 0.2f, 0.005f,
        weights, bounds, cfg, om);

    printf("feasible=%d  idx1=%d  T1=%.4f  J=%.6f\n",
           r.feasible, r.idx1, r.T1, r.J);
    if (r.feasible) {
        printf("  s1: theta=%.4f phi=%.4f vExit=%.3f\n",
               r.s1.theta, r.s1.phi, r.s1.v_exit);
        // Goal is in the +x direction (theta ≈ 0 from early trajectory points).
        // The shot theta must not be backwards (|theta| must be < π/2).
        float goal_angle = std::atan2(0.f - 0.f, 3.f - 1.f);  // atan2(0, 2) = 0
        float d = std::abs(r.s1.theta - goal_angle);
        if (d > 3.14159265f) d = 6.28318530f - d;
        printf("  angle deviation from goal: %.4f rad (%.1f deg)\n",
               d, d * 180.f / 3.14159265f);
        assert(d < 1.5707963f && "shot must not point backwards from goal");
    }
    printf("PASS no_backward_shot\n");
}

int main() {
    test_static_1ball();
    test_static_2ball();
    test_static_3ball();
    test_moving_trajectory();
    test_transfer_timing();
    test_no_backward_shot();
    printf("\nDone.\n");
    return 0;
}
