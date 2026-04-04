#include <cstdio>
#include <cmath>
#include <cassert>
#include "turret_planner/path_scan.h"

// Shared config
static PhysicsConfig  cfg{9.81f, 0.1f};
static OmegaMapParams om{};
// Tight v_exit cap to create a clear infeasible→feasible transition
static TurretBounds   bounds{-3.14f, 3.14f, 0.f, 1.4f, 8.f, 1200.f};
static TurretWeights  weights{0.05f, 0.08f, 0.0005f};
static TurretState    cur{0.f, 0.5f, 0.f};

// Build a linear path: n samples, x from x0 to x1, y=0, t[i]=i*dt.
static void make_path(PathSample* path, int n, float x0, float x1, float dt) {
    for (int i = 0; i < n; ++i) {
        float alpha = (n > 1) ? float(i) / float(n - 1) : 0.f;
        path[i].t  = i * dt;
        path[i].x  = x0 + alpha * (x1 - x0);
        path[i].y  = 0.f;
        path[i].vx = 0.f;
        path[i].vy = 0.f;
    }
}

// -------------------------------------------------------------------------
// test 1: infeasible → feasible transition found mid-path
// Target starts far (x=10, v_exit too high), ends near (x=3, feasible).
// Turret at (0,0,0).
// -------------------------------------------------------------------------
void test_infeasible_to_feasible() {
    printf("\n=== test_infeasible_to_feasible ===\n");
    const int N = 20;
    PathSample path[N];
    make_path(path, N, 10.f, 3.f, 0.1f);

    EarliestShotResult r = path_scan_earliest(
        path, N,
        0.f, 0.f, 0.f,   // turret at origin
        0.f, 0.f,         // robot stationary
        1.f,              // target z
        0.f,              // t_now = start
        cur, weights, bounds, cfg, om);

    printf("found=%d  t_path=%.3f  sample=%d  T*=%.4f  feasible=%d\n",
        r.found, r.t_path, r.sample_index, r.T_star, r.found);
    assert(r.found && "expected to find a shot");
    // The early samples (x=10) must be infeasible and the transition should
    // occur well before the path end (x=3).
    assert(r.t_path < path[N-1].t && "transition should be before end of path");
    assert(r.t_path > 0.f && "transition should not be at the very first sample (x=10 is far)");
    printf("PASS infeasible_to_feasible\n");
}

// -------------------------------------------------------------------------
// test 2: first sample already feasible — should return immediately
// -------------------------------------------------------------------------
void test_first_sample_feasible() {
    printf("\n=== test_first_sample_feasible ===\n");
    const int N = 10;
    PathSample path[N];
    make_path(path, N, 3.f, 3.f, 0.1f);  // all at x=3 (feasible)

    EarliestShotResult r = path_scan_earliest(
        path, N,
        0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f,
        cur, weights, bounds, cfg, om);

    printf("found=%d  t_path=%.3f  sample=%d\n", r.found, r.t_path, r.sample_index);
    assert(r.found);
    assert(r.t_path == path[0].t && "should be the first sample");
    printf("PASS first_sample_feasible\n");
}

// -------------------------------------------------------------------------
// test 3: all samples infeasible — should return found=false
// -------------------------------------------------------------------------
void test_all_infeasible() {
    printf("\n=== test_all_infeasible ===\n");
    const int N = 10;
    PathSample path[N];
    make_path(path, N, 15.f, 12.f, 0.1f);  // all too far for v_max=8

    EarliestShotResult r = path_scan_earliest(
        path, N,
        0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f,
        cur, weights, bounds, cfg, om);

    printf("found=%d\n", r.found);
    assert(!r.found && "expected no feasible shot");
    printf("PASS all_infeasible\n");
}

// -------------------------------------------------------------------------
// test 4: t_now filtering — skip past samples
// -------------------------------------------------------------------------
void test_t_now_filtering() {
    printf("\n=== test_t_now_filtering ===\n");
    const int N = 20;
    PathSample path[N];
    make_path(path, N, 10.f, 3.f, 0.1f);  // transition somewhere in the middle

    // Without filtering: finds the transition
    EarliestShotResult r_all = path_scan_earliest(
        path, N, 0,0,0, 0,0, 1.f, 0.f, cur, weights, bounds, cfg, om);

    // With t_now just after the transition: should find same or later point
    float t_now = r_all.t_path + 0.05f;  // 50ms after transition
    EarliestShotResult r_late = path_scan_earliest(
        path, N, 0,0,0, 0,0, 1.f, t_now, cur, weights, bounds, cfg, om);

    printf("no-filter: t_path=%.3f   with t_now=%.3f: t_path=%.3f\n",
        r_all.t_path, t_now, r_late.found ? r_late.t_path : -1.f);
    assert(r_late.found);
    assert(r_late.t_path >= t_now - 1e-4f && "filtered result must be at or after t_now");
    printf("PASS t_now_filtering\n");
}

// -------------------------------------------------------------------------
// test 5: bisection gives sub-sample precision
// -------------------------------------------------------------------------
void test_bisection_precision() {
    printf("\n=== test_bisection_precision ===\n");
    const int N = 5;   // coarse path — large sample spacing
    PathSample path[N];
    make_path(path, N, 10.f, 3.f, 0.5f);  // 0.5s spacing

    // No bisection
    EarliestShotResult r_coarse = path_scan_earliest(
        path, N, 0,0,0, 0,0, 1.f, 0.f, cur, weights, bounds, cfg, om,
        /*bisect_iters=*/0);

    // With bisection
    EarliestShotResult r_fine = path_scan_earliest(
        path, N, 0,0,0, 0,0, 1.f, 0.f, cur, weights, bounds, cfg, om,
        /*bisect_iters=*/5);

    printf("coarse t_path=%.4f   fine t_path=%.4f   diff=%.4f\n",
        r_coarse.found ? r_coarse.t_path : -1.f,
        r_fine.found   ? r_fine.t_path   : -1.f,
        (r_coarse.found && r_fine.found) ? r_coarse.t_path - r_fine.t_path : 0.f);

    assert(r_coarse.found && r_fine.found);
    // Bisection should give an equal or earlier feasible point than coarse snap-to-sample.
    assert(r_fine.t_path <= r_coarse.t_path + 1e-4f);
    // And the fine result should actually be feasible.
    assert(ballistics_is_feasible(r_fine.params, r_fine.T_star, bounds, cfg));
    printf("PASS bisection_precision\n");
}

int main() {
    test_infeasible_to_feasible();
    test_first_sample_feasible();
    test_all_infeasible();
    test_t_now_filtering();
    test_bisection_precision();
    printf("\nDone.\n");
    return 0;
}
