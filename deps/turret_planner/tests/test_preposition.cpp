#include <cstdio>
#include <cmath>
#include <cassert>
#include "turret_planner/preposition.h"

static PhysicsConfig  cfg{9.81f, 0.1f};
static OmegaMapParams om{};
static TurretBounds   bounds{-3.14f, 3.14f, 0.f, 1.4f, 15.f, 1200.f};
static TurretWeights  weights{0.05f, 0.08f, 0.0005f};

static void check(const char* name, float got, float expected, float tol) {
    float err = std::abs(got - expected);
    if (err > tol)
        printf("FAIL %s: got %.4f expected %.4f (err %.2e)\n", name, got, expected, err);
    else
        printf("PASS %s (err %.2e)\n", name, err);
}

// -------------------------------------------------------------------------
// test 1: stationary target, generous t_available.
// All samples see the same target (3, 0, 1).  The centroid should converge
// to the T* shot for that target, and the reachable projection shouldn't clamp.
// -------------------------------------------------------------------------
void test_stationary_target() {
    printf("\n=== test_stationary_target ===\n");
    const int K = 8;
    PathSample path[K];
    for (int i = 0; i < K; ++i) {
        path[i].t = i * 0.1f;
        path[i].x = 3.f; path[i].y = 0.f;
        path[i].vx = path[i].vy = 0.f;
    }

    TurretState cur{0.f, 0.f, 0.f};

    // Single-sample T* as reference
    FlightTimeResult ref = flight_time_cold(0,0,0, 3.f,0.f,1.f, 0.f,0.f,
                                             cur, weights, bounds, cfg, om);
    assert(ref.feasible);

    PrepositionResult r = preposition_compute(
        path, K,
        0.f,0.f,0.f, 0.f,0.f, 1.f,
        cur, /*t_available=*/5.f, /*lambda_decay=*/0.f, /*k_samples=*/K,
        weights, bounds, cfg, om);

    printf("ref theta=%.4f phi=%.4f om=%.2f\n",
        ref.params.theta, ref.params.phi, ref.params.omega_flywheel);
    printf("pre theta=%.4f phi=%.4f om=%.2f\n",
        r.target.theta, r.target.phi, r.target.omega_flywheel);

    // All samples see same target → centroid == single-sample result
    check("theta", r.target.theta, ref.params.theta, 1e-3f);
    check("phi",   r.target.phi,   ref.params.phi,   1e-3f);
}

// -------------------------------------------------------------------------
// test 2: t_available so small the turret can barely move.
// Result must stay near current state.
// -------------------------------------------------------------------------
void test_clamped_by_t_available() {
    printf("\n=== test_clamped_by_t_available ===\n");
    const int K = 8;
    PathSample path[K];
    for (int i = 0; i < K; ++i) {
        path[i].t = i * 0.1f;
        path[i].x = 3.f; path[i].y = 1.f;
        path[i].vx = path[i].vy = 0.f;
    }

    TurretState cur{0.2f, 0.6f, 400.f};  // some non-zero current state

    PrepositionResult r_free = preposition_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 0.f, K, weights, bounds, cfg, om);

    PrepositionResult r_tight = preposition_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 0.001f, 0.f, K, weights, bounds, cfg, om);

    printf("free   theta=%.4f phi=%.4f\n", r_free.target.theta, r_free.target.phi);
    printf("tight  theta=%.4f phi=%.4f (cur=%.4f, %.4f)\n",
        r_tight.target.theta, r_tight.target.phi, cur.theta, cur.phi);

    // With t_available=1ms, reachable range is tiny: result must be ≈ current
    float max_theta_change = 0.001f / weights.w_theta;  // rad
    float max_phi_change   = 0.001f / weights.w_phi;
    assert(std::abs(r_tight.target.theta - cur.theta) <= max_theta_change + 1e-5f);
    assert(std::abs(r_tight.target.phi   - cur.phi)   <= max_phi_change   + 1e-5f);
    printf("PASS clamped_by_t_available\n");
}

// -------------------------------------------------------------------------
// test 3: exponential weighting — high lambda means result dominated by sample 0.
// Use a path with two distinct shot angles.
// -------------------------------------------------------------------------
void test_exponential_weighting() {
    printf("\n=== test_exponential_weighting ===\n");
    // Two-segment path: first half points toward (3, 0, 1), second half toward (3, 3, 1).
    const int K = 8;
    PathSample path[K];
    for (int i = 0; i < K; ++i) {
        path[i].t = i * 0.1f;
        path[i].x = 3.f;
        path[i].y = (i < K/2) ? 0.f : 3.f;  // theta changes at midpoint
        path[i].vx = path[i].vy = 0.f;
    }

    TurretState cur{0.f, 0.f, 0.f};

    // Uniform weighting (lambda=0): centroid of both halves
    PrepositionResult r_uniform = preposition_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 0.f, K, weights, bounds, cfg, om);

    // Heavy decay (lambda=5): dominated by first few samples (y=0)
    PrepositionResult r_decay = preposition_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 5.f, K, weights, bounds, cfg, om);

    // Reference: T* for sample 0 (y=0, theta≈0)
    FlightTimeResult ref0 = flight_time_cold(0,0,0, 3.f,0.f,1.f, 0.f,0.f,
                                              cur, weights, bounds, cfg, om);

    printf("uniform theta=%.4f   decay theta=%.4f   ref0 theta=%.4f\n",
        r_uniform.target.theta, r_decay.target.theta, ref0.params.theta);

    // High-decay result should be closer to ref0 (theta≈0) than uniform result
    assert(std::abs(r_decay.target.theta - ref0.params.theta) <
           std::abs(r_uniform.target.theta - ref0.params.theta) + 1e-3f);
    printf("PASS exponential_weighting\n");
}

// -------------------------------------------------------------------------
// test 4: empty / all-infeasible path → returns current state
// -------------------------------------------------------------------------
void test_no_feasible_samples() {
    printf("\n=== test_no_feasible_samples ===\n");
    const int K = 5;
    PathSample path[K];
    for (int i = 0; i < K; ++i) {
        path[i].t = i * 0.1f;
        path[i].x = 50.f;  // way too far, v_exit needed >> max
        path[i].y = 0.f;
        path[i].vx = path[i].vy = 0.f;
    }

    TurretState cur{0.3f, 0.7f, 500.f};
    PrepositionResult r = preposition_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 0.f, K, weights, bounds, cfg, om);

    printf("theta=%.4f (expect %.4f)\n", r.target.theta, cur.theta);
    assert(r.target.theta == cur.theta && "should return current state when no feasible samples");
    printf("PASS no_feasible_samples\n");
}

// -------------------------------------------------------------------------
// test 5: preposition_robust_compute runs cleanly over a feasible path and
// produces a feasible target. θ is largely determined by target position,
// so it should roughly agree with plain preposition; φ and ω can differ
// because the two functions minimize different objectives per sample
// (plain: τ from current state; robust: transition cost to next shot).
// -------------------------------------------------------------------------
void test_robust_feasibility() {
    printf("\n=== test_robust_feasibility ===\n");
    OmegaMapParams om_rich{{800.f, 40.f, 50.f, 0.f, 0.f, 0.f}};

    const int K = 4;
    PathSample path[K];
    for (int i = 0; i < K; ++i) {
        path[i].t = i * 0.1f;
        path[i].x = 3.f + 0.1f * i;
        path[i].y = 0.2f * i;
        path[i].vx = path[i].vy = 0.f;
    }

    TurretState cur{0.f, 0.3f, 500.f};

    PrepositionResult plain = preposition_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 0.f, K, weights, bounds, cfg, om_rich);
    PrepositionResult rob0 = preposition_robust_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 0.f, K, /*omega_drop=*/0.f, weights, bounds, cfg, om_rich);

    printf("plain theta=%.4f phi=%.4f om=%.2f\n",
        plain.target.theta, plain.target.phi, plain.target.omega_flywheel);
    printf("rob0  theta=%.4f phi=%.4f om=%.2f\n",
        rob0.target.theta, rob0.target.phi, rob0.target.omega_flywheel);

    // θ is driven by target geometry more than by the objective, so the
    // two should agree closely.
    assert(std::abs(rob0.target.theta - plain.target.theta) < 0.05f);
    // φ is inside its bounds.
    assert(rob0.target.phi >= bounds.phi_min && rob0.target.phi <= bounds.phi_max);
    // ω is non-negative and within its bound.
    assert(rob0.target.omega_flywheel >= 0.f
        && rob0.target.omega_flywheel <= bounds.omega_max);
    printf("PASS robust_feasibility\n");
}

// -------------------------------------------------------------------------
// test 6: a non-zero omega_drop should produce a different preposition
// (sanity check that the drop parameter actually flows through).
// -------------------------------------------------------------------------
void test_robust_drop_shifts_target() {
    printf("\n=== test_robust_drop_shifts_target ===\n");
    OmegaMapParams om_rich{{800.f, 40.f, 50.f, 0.f, 0.f, 0.f}};

    const int K = 4;
    PathSample path[K];
    for (int i = 0; i < K; ++i) {
        path[i].t = i * 0.1f;
        path[i].x = 3.f + 0.1f * i;
        path[i].y = 0.2f * i;
        path[i].vx = path[i].vy = 0.f;
    }
    TurretState cur{0.f, 0.3f, 500.f};

    PrepositionResult r0 = preposition_robust_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 0.f, K, 0.f, weights, bounds, cfg, om_rich);
    PrepositionResult r1 = preposition_robust_compute(
        path, K, 0,0,0, 0,0, 1.f,
        cur, 10.f, 0.f, K, 30.f, weights, bounds, cfg, om_rich);

    printf("drop=0  theta=%.4f phi=%.4f om=%.2f\n",
        r0.target.theta, r0.target.phi, r0.target.omega_flywheel);
    printf("drop=30 theta=%.4f phi=%.4f om=%.2f\n",
        r1.target.theta, r1.target.phi, r1.target.omega_flywheel);

    // Something (theta, phi, or omega) should differ meaningfully.
    float d_theta = std::abs(r0.target.theta - r1.target.theta);
    float d_phi   = std::abs(r0.target.phi   - r1.target.phi);
    float d_om    = std::abs(r0.target.omega_flywheel - r1.target.omega_flywheel);
    printf("|dtheta|=%.4f  |dphi|=%.4f  |dom|=%.4f\n", d_theta, d_phi, d_om);
    assert(d_theta + d_phi + d_om > 1e-3f
           && "non-zero omega_drop should change the preposition target");
    printf("PASS robust_drop_shifts_target\n");
}

int main() {
    test_stationary_target();
    test_clamped_by_t_available();
    test_exponential_weighting();
    test_no_feasible_samples();
    test_robust_feasibility();
    test_robust_drop_shifts_target();
    printf("\nDone.\n");
    return 0;
}
