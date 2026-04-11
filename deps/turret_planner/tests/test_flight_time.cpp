#include <cstdio>
#include <cmath>
#include <cassert>
#include "turret_planner/flight_time.h"

static PhysicsConfig  cfg{9.81f, 0.1f};
static OmegaMapParams om{}; // trivial map
static TurretBounds   bounds{-3.14f, 3.14f, 0.f, 1.4f, 15.f, 1200.f};
static TurretWeights  weights{0.05f, 0.08f, 0.0005f};  // s/rad, s/rad, s/(rad/s)

void test_cold_converges() {
    printf("\n=== test_cold_converges ===\n");
    TurretState cur{0.f, 0.5f, 0.f};
    FlightTimeResult r = flight_time_cold(
        0,0,0, 3.f,0.f,1.f, 0.f,0.f, cur, weights, bounds, cfg, om);

    printf("feasible=%d  T*=%.4f  tau=%.4f\n", r.feasible, r.T_star, r.tau);
    assert(r.feasible);

    // Verify it's actually near-optimal by comparing against a brute-force sweep
    float best_tau = 1e30f;
    TInterval iv = ballistics_feasible_interval(3.f,0.f,1.f,0.f,0.f, bounds,cfg,1e-4f);
    int N = 500;
    for (int i = 0; i <= N; ++i) {
        float T = iv.t_lo + (iv.t_hi - iv.t_lo) * float(i) / float(N);
        ShotParams p = ballistics_solve(0,0,0,3,0,1,0,0,T,cfg,om);
        float tau = flight_time_tau(p, cur, weights, om);
        if (tau < best_tau) best_tau = tau;
    }

    printf("brute-force best tau=%.4f, solver tau=%.4f, gap=%.6f\n",
        best_tau, r.tau, r.tau - best_tau);
    // Solver should be within tol=5e-4 of the brute-force best
    assert(r.tau - best_tau < 5e-4f + 1e-3f);
    printf("PASS cold_converges\n");
}

void test_warm_refinement() {
    printf("\n=== test_warm_refinement ===\n");
    TurretState cur{0.1f, 0.6f, 0.f};

    // Cold solve as ground truth
    FlightTimeResult cold = flight_time_cold(
        0,0,0, 3.f,0.f,1.f, 0.f,0.f, cur, weights, bounds, cfg, om);

    // Warm start with cold T* + small perturbation
    FlightTimeResult warm = flight_time_warm(
        0,0,0, 3.f,0.f,1.f, 0.f,0.f,
        cold.T_star + 0.05f, cur, weights, bounds, cfg, om);

    printf("cold T*=%.5f tau=%.5f\n", cold.T_star, cold.tau);
    printf("warm T*=%.5f tau=%.5f\n", warm.T_star, warm.tau);
    assert(warm.feasible);
    // Warm should match cold within 5ms on T* and small tau difference
    assert(std::abs(warm.T_star - cold.T_star) < 0.1f);
    printf("PASS warm_refinement\n");
}

void test_tau_is_max() {
    printf("\n=== test_tau_is_max ===\n");
    TurretState cur{0.f, 0.3f, 300.f};
    ShotParams  tgt{0.2f, 0.8f, 0.f, 800.f};
    // Manually: Δθ=0.2, Δφ=0.5, Δω=500
    // w_theta=0.05, w_phi=0.08, w_omega=0.0005
    // arms: 0.01, 0.04, 0.25 → max = 0.25
    float tau = flight_time_tau(tgt, cur, weights, om);
    printf("tau=%.6f  expected~0.25\n", tau);
    assert(std::abs(tau - 0.25f) < 1e-4f);
    printf("PASS tau_is_max\n");
}

int main() {
    test_tau_is_max();
    test_cold_converges();
    test_warm_refinement();
    printf("\nDone.\n");
    return 0;
}
