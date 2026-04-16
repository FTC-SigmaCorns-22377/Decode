#include <cstdio>
#include <cmath>
#include <cassert>
#include <algorithm>
#include "turret_planner/robust_shot.h"
#include "turret_planner/ballistics.h"
#include "turret_planner/flywheel_model.h"

// Use a non-trivial omega map so the flywheel-drop arm actually matters:
//   omega = 800 + 40*v + 50*phi
static PhysicsConfig  cfg{9.81f, 0.1f};
static OmegaMapParams om{{800.f, 40.f, 50.f, 0.f, 0.f, 0.f}};
static TurretBounds   bounds{-3.14f, 3.14f, 0.f, 1.4f, 15.f, 2000.f};
static TurretWeights  weights{0.05f, 0.08f, 0.0005f};

static float robust_move_cost(const ShotParams& s1, const ShotParams& s2,
                              float drop_fraction) {
    float om1 = omega_map_eval(om, s1.phi, s1.v_exit) * (1.f - drop_fraction);
    float om2 = omega_map_eval(om, s2.phi, s2.v_exit);
    float d_omega = std::abs(om1 - om2);
    float d_phi   = std::abs(s1.phi   - s2.phi);
    float d_theta = std::abs(s1.theta - s2.theta);
    return std::max({weights.w_omega * d_omega,
                     weights.w_phi   * d_phi,
                     weights.w_theta * d_theta});
}

void test_robust_matches_grid() {
    printf("\n=== test_robust_matches_grid ===\n");
    const float drop_fraction = 0.005f;  // ~0.5% proportional drop

    // Two distinct targets (different xyz → different theta, phi).
    float t1x = 3.0f, t1y = 0.0f, t1z = 1.5f;
    float t2x = 2.5f, t2y = 1.2f, t2z = 1.5f;

    TInterval iv1 = ballistics_feasible_interval(
        t1x, t1y, t1z, 0.f, 0.f, bounds, cfg, 1e-4f);
    TInterval iv2 = ballistics_feasible_interval(
        t2x, t2y, t2z, 0.f, 0.f, bounds, cfg, 1e-4f);
    assert(iv1.t_lo < iv1.t_hi && iv2.t_lo < iv2.t_hi);
    printf("iv1=[%.3f, %.3f]  iv2=[%.3f, %.3f]\n",
           iv1.t_lo, iv1.t_hi, iv2.t_lo, iv2.t_hi);

    // Brute-force 21x21 grid over the feasible rectangle.
    const int N = 20;
    float best_grid_cost = 1e30f;
    float best_T1 = iv1.t_lo, best_T2 = iv2.t_lo;
    for (int i = 0; i <= N; ++i) {
        float T1 = iv1.t_lo + (iv1.t_hi - iv1.t_lo) * float(i) / float(N);
        ShotParams s1 = ballistics_solve(0,0,0, t1x, t1y, t1z, 0, 0, T1, cfg, om);
        if (!ballistics_is_feasible(s1, T1, bounds, cfg)) continue;
        for (int j = 0; j <= N; ++j) {
            float T2 = iv2.t_lo + (iv2.t_hi - iv2.t_lo) * float(j) / float(N);
            ShotParams s2 = ballistics_solve(0,0,0, t2x, t2y, t2z, 0, 0, T2, cfg, om);
            if (!ballistics_is_feasible(s2, T2, bounds, cfg)) continue;
            float cost = robust_move_cost(s1, s2, drop_fraction);
            if (cost < best_grid_cost) {
                best_grid_cost = cost;
                best_T1 = T1;
                best_T2 = T2;
            }
        }
    }
    assert(best_grid_cost < 1e29f && "grid should find a feasible sample");
    printf("grid best: cost=%.6f  T1=%.4f  T2=%.4f\n",
           best_grid_cost, best_T1, best_T2);

    // Solver.
    RobustShotResult r = flight_time_robust(
        0,0,0, t1x, t1y, t1z, t2x, t2y, t2z,
        0, 0, drop_fraction, weights, bounds, cfg, om,
        /*tol=*/0.01f, /*maxIter=*/200);

    printf("solver: feasible=%d  T1=%.4f  T2=%.4f  J=%.6f\n",
           r.feasible, r.T1, r.T2, r.J);
    printf("  s1 theta=%.4f phi=%.4f vExit=%.3f omega=%.2f\n",
           r.s1.theta, r.s1.phi, r.s1.v_exit, r.s1.omega_flywheel);
    printf("  s2 theta=%.4f phi=%.4f vExit=%.3f omega=%.2f\n",
           r.s2.theta, r.s2.phi, r.s2.v_exit, r.s2.omega_flywheel);
    assert(r.feasible);

    // Recompute cost from the solver output (J should match robust_move_cost).
    float solver_cost = robust_move_cost(r.s1, r.s2, drop_fraction);
    printf("solver cost (recomputed): %.6f   solver J: %.6f\n", solver_cost, r.J);
    assert(std::abs(solver_cost - r.J) < 1e-4f);

    // Must be within 5% (with a small absolute floor) of the grid minimum.
    float tolerance = std::max(1e-4f, best_grid_cost * 0.05f);
    float gap = solver_cost - best_grid_cost;
    printf("gap=%.6f  tolerance=%.6f\n", gap, tolerance);
    assert(solver_cost <= best_grid_cost + tolerance);
    printf("PASS robust_matches_grid\n");
}

void test_infeasible_interval() {
    printf("\n=== test_infeasible_interval ===\n");
    // Target way too far — no feasible T.
    RobustShotResult r = flight_time_robust(
        0,0,0,  50.f, 0.f, 1.f,  3.f, 0.f, 1.f,
        0, 0, 5.f, weights, bounds, cfg, om);
    printf("feasible=%d J=%.3g\n", r.feasible, r.J);
    assert(!r.feasible);
    printf("PASS infeasible_interval\n");
}

// Helper: reference objective for flight_time_robust_adjust.
static float adjust_cost(const ShotParams& s1, const ShotParams& s2,
                         const TurretState& cur, float drop_fraction) {
    // A = J_Δ(cur, s1) with Δθ wrap.
    float a_d_theta = std::abs(s1.theta - cur.theta);
    if (a_d_theta > 3.14159265f) a_d_theta = 6.28318530f - a_d_theta;
    float a_d_phi   = std::abs(s1.phi   - cur.phi);
    float a_d_omega = std::abs(s1.omega_flywheel - cur.omega_flywheel);
    float A = std::max({weights.w_theta * a_d_theta,
                        weights.w_phi   * a_d_phi,
                        weights.w_omega * a_d_omega});
    // B = J_Δ(s1_reduced, s2)
    float b_om1 = s1.omega_flywheel * (1.f - drop_fraction);
    float b_d_omega = std::abs(b_om1 - s2.omega_flywheel);
    float b_d_phi   = std::abs(s1.phi   - s2.phi);
    float b_d_theta = std::abs(s1.theta - s2.theta);
    float B = std::max({weights.w_omega * b_d_omega,
                        weights.w_phi   * b_d_phi,
                        weights.w_theta * b_d_theta});
    return A + B;
}

void test_adjust_matches_grid() {
    printf("\n=== test_adjust_matches_grid ===\n");
    const float drop_fraction = 0.005f;  // ~0.5% proportional drop
    // Current turret state deliberately offset from any optimal shot so the
    // A term (cur → s1 slew) is non-trivial.
    TurretState cur{-0.3f, 0.4f, 900.f};

    float t1x = 3.0f, t1y = 0.0f, t1z = 1.5f;
    float t2x = 2.5f, t2y = 1.2f, t2z = 1.5f;

    TInterval iv1 = ballistics_feasible_interval(t1x, t1y, t1z, 0,0, bounds, cfg, 1e-4f);
    TInterval iv2 = ballistics_feasible_interval(t2x, t2y, t2z, 0,0, bounds, cfg, 1e-4f);
    assert(iv1.t_lo < iv1.t_hi && iv2.t_lo < iv2.t_hi);

    // Brute-force 21x21 grid.
    const int N = 20;
    float best_grid_cost = 1e30f;
    float best_T1 = iv1.t_lo, best_T2 = iv2.t_lo;
    for (int i = 0; i <= N; ++i) {
        float T1 = iv1.t_lo + (iv1.t_hi - iv1.t_lo) * float(i) / float(N);
        ShotParams s1 = ballistics_solve(0,0,0, t1x, t1y, t1z, 0, 0, T1, cfg, om);
        if (!ballistics_is_feasible(s1, T1, bounds, cfg)) continue;
        for (int j = 0; j <= N; ++j) {
            float T2 = iv2.t_lo + (iv2.t_hi - iv2.t_lo) * float(j) / float(N);
            ShotParams s2 = ballistics_solve(0,0,0, t2x, t2y, t2z, 0, 0, T2, cfg, om);
            if (!ballistics_is_feasible(s2, T2, bounds, cfg)) continue;
            float cost = adjust_cost(s1, s2, cur, drop_fraction);
            if (cost < best_grid_cost) {
                best_grid_cost = cost;
                best_T1 = T1;
                best_T2 = T2;
            }
        }
    }
    assert(best_grid_cost < 1e29f);
    printf("grid best: cost=%.6f  T1=%.4f  T2=%.4f\n", best_grid_cost, best_T1, best_T2);

    RobustShotResult r = flight_time_robust_adjust(
        0,0,0, t1x, t1y, t1z, t2x, t2y, t2z,
        0, 0, cur, drop_fraction, weights, bounds, cfg, om,
        /*tol=*/0.01f, /*maxIter=*/200);

    printf("solver: feasible=%d  T1=%.4f  T2=%.4f  J=%.6f\n",
           r.feasible, r.T1, r.T2, r.J);
    assert(r.feasible);

    float solver_cost = adjust_cost(r.s1, r.s2, cur, drop_fraction);
    printf("solver cost (recomputed): %.6f   solver J: %.6f\n", solver_cost, r.J);
    assert(std::abs(solver_cost - r.J) < 1e-4f);

    float tolerance = std::max(1e-4f, best_grid_cost * 0.05f);
    float gap = solver_cost - best_grid_cost;
    printf("gap=%.6f  tolerance=%.6f\n", gap, tolerance);
    assert(solver_cost <= best_grid_cost + tolerance);
    printf("PASS adjust_matches_grid\n");
}

void test_adjust_same_target_drop_zero() {
    printf("\n=== test_adjust_same_target_drop_zero ===\n");
    // With drop_fraction=0 and target1==target2, the B term is minimized at T1=T2
    // and the objective degenerates to A = J_Δ(cur → s1). The solver should
    // pick (T1, T2) with T1 ≈ T2 and a small J.
    TurretState cur{0.1f, 0.5f, 700.f};
    float tx = 3.0f, ty = 0.2f, tz = 1.5f;

    RobustShotResult r = flight_time_robust_adjust(
        0,0,0, tx, ty, tz, tx, ty, tz,
        0, 0, cur, /*drop_fraction=*/0.f, weights, bounds, cfg, om,
        /*tol=*/0.005f, /*maxIter=*/100);

    printf("feasible=%d  T1=%.4f  T2=%.4f  J=%.6f\n", r.feasible, r.T1, r.T2, r.J);
    assert(r.feasible);

    // B should be ~0 at T1 == T2.
    float B_direct;
    {
        float b_d_omega = std::abs(r.s1.omega_flywheel - r.s2.omega_flywheel);
        float b_d_phi   = std::abs(r.s1.phi   - r.s2.phi);
        float b_d_theta = std::abs(r.s1.theta - r.s2.theta);
        B_direct = std::max({weights.w_omega * b_d_omega,
                             weights.w_phi   * b_d_phi,
                             weights.w_theta * b_d_theta});
    }
    printf("B at solution = %.6f\n", B_direct);
    assert(B_direct < 0.05f && "B should be ~0 with matching targets and zero drop");
    printf("PASS adjust_same_target_drop_zero\n");
}

int main() {
    test_robust_matches_grid();
    test_infeasible_interval();
    test_adjust_matches_grid();
    test_adjust_same_target_drop_zero();
    printf("\nDone.\n");
    return 0;
}
