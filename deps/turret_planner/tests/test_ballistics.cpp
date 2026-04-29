#include <cstdio>
#include <cmath>
#include <cassert>
#include <array>
#include "turret_planner/ballistics.h"
#include "turret_planner/flywheel_model.h"

// Utility: check two floats are close
static void check(const char* name, float got, float expected, float tol = 1e-3f) {
    float err = std::abs(got - expected);
    if (err > tol) {
        printf("FAIL %s: got %.6f expected %.6f (err %.2e)\n", name, got, expected, err);
    } else {
        printf("PASS %s: %.6f (err %.2e)\n", name, got, err);
    }
}

// Cross-check: verify ballistics_solve matches the Kotlin logic exactly.
// We compute by hand for a simple stationary case (vR=0):
//   Target at (3,0,1) from turret at (0,0,0), T=0.5s, g=9.81, r_h=0.1
//   dx=3, dy=0, dz=1
//   b = 3/0.5 = 6.0,  c = 0,  a = -0.1/0.5 = -0.2
//   theta = atan2(0, 6) = 0
//   B = -0.2 + 1.0*6.0 + 0 = 5.8
//   C = 1/0.5 + 0.5*9.81*0.5 = 2.0 + 2.4525 = 4.4525
//   v² = 5.8²+4.4525²-0.2² = 33.64+19.8248-0.04 = 53.4248 → v = 7.3093
//   phi = atan2(C*v+B*a, B*v-C*a)
//       = atan2(4.4525*7.3093 + 5.8*(-0.2), 5.8*7.3093 - 4.4525*(-0.2))
//       = atan2(32.538 - 1.16, 42.394 + 0.8905)
//       = atan2(31.378, 43.284)
//       = 0.6243 rad (~35.8°)
void test_solve_basic() {
    printf("\n=== test_solve_basic ===\n");
    PhysicsConfig cfg{9.81f, 0.1f};
    OmegaMapParams om{}; // all zeros → omega = 0
    ShotParams p = ballistics_solve(0,0,0, 3,0,1, 0,0, 0.5f, cfg, om);

    check("theta", p.theta, 0.f, 1e-4f);
    check("v_exit", p.v_exit, 7.3093f, 5e-3f);
    // Reference phi computed via std::atan2 (higher precision); fast_atan2 error < 5e-6 rad.
    // The 3e-3 gap here is vs our hand-computed reference 0.6243, not vs ground truth.
    {
        float B = -0.1f/0.5f + 1.f*6.f, C = 1.f/0.5f + 0.5f*9.81f*0.5f;
        float v = std::sqrt(B*B + C*C - (0.1f/0.5f)*(0.1f/0.5f));
        float phi_ref = std::atan2(C*v + B*(-0.2f), B*v - C*(-0.2f));
        check("phi vs ref", p.phi, phi_ref, 1e-4f);
    }
}

// Verify dθ/dT matches the analytic simplified formula via finite difference.
// Uses double-precision FD to avoid float cancellation noise.
void test_dtheta_dT() {
    printf("\n=== test_dtheta_dT ===\n");
    double dx = 2.0, dy = 1.5, vRx = 0.3, vRy = -0.2;
    double T = 0.4, eps = 1e-7;

    // Analytic (float)
    PhysicsConfig cfg{9.81f, 0.1f};
    float analytic = ballistics_dtheta_dT(float(dx), float(dy), float(vRx), float(vRy), float(T), cfg);

    // FD in double for accuracy
    auto eval_theta_d = [&](double t) {
        double b = dx/t - vRx, c = dy/t - vRy;
        return std::atan2(c, b);
    };
    double fd = (eval_theta_d(T + eps) - eval_theta_d(T - eps)) / (2.0 * eps);

    check("dtheta_dT", analytic, float(fd), 1e-4f);
}

// Verify the full derivative suite via double-precision finite differences.
void test_derivs_fd() {
    printf("\n=== test_derivs_fd ===\n");
    PhysicsConfig cfg{9.81f, 0.15f};
    // Set up a small IDW omega map: omega = 30*v - 10*phi + 500 (approx) via 3 points.
    OmegaMapParams om{};
    om.n = 3;
    om.phi[0] = 0.4f; om.v_exit[0] = 6.f; om.omega[0] = 500.f + 30.f*6.f - 10.f*0.4f;
    om.phi[1] = 0.6f; om.v_exit[1] = 8.f; om.omega[1] = 500.f + 30.f*8.f - 10.f*0.6f;
    om.phi[2] = 0.5f; om.v_exit[2] = 7.f; om.omega[2] = 500.f + 30.f*7.f - 10.f*0.5f;
    om.phi_scale = 1.f / (0.6f - 0.4f);
    om.v_scale   = 1.f / (8.f - 6.f);

    float tx=0,ty=0,tz=0, gx=2.5f,gy=1.f,gz=0.8f, vRx=0.2f,vRy=-0.1f;
    float T = 0.45f;
    double eps = 1e-7;

    ShotDerivatives derivs;
    ballistics_solve_with_derivs(tx,ty,tz, gx,gy,gz, vRx,vRy, T, cfg, om, &derivs);

    // Compute FD in double via direct formula replication (IDW omega at each perturbed T)
    auto eval_d = [&](double t) -> std::array<double,4> {
        double dx = gx-tx, dy = gy-ty, dz = gz-tz;
        double inv_t = 1.0/t;
        double b = dx*inv_t - vRx, c = dy*inv_t - vRy;
        double a = -cfg.r_h*inv_t;
        double theta = std::atan2(c,b);
        double st = std::sin(theta), ct = std::cos(theta);
        double B = a + ct*b + st*c;
        double C = dz*inv_t + 0.5*cfg.g*t;
        double vsq = B*B + C*C - a*a;
        double v = (vsq>0) ? std::sqrt(vsq) : 0.0;
        double phi = std::atan2(C*v+B*a, B*v-C*a);
        double om_val = omega_map_eval(om, float(phi), float(v));
        return {theta, phi, v, om_val};
    };
    auto p_p = eval_d(T+eps), p_m = eval_d(T-eps);
    double fd_dtheta = (p_p[0]-p_m[0])/(2.0*eps);
    double fd_dphi   = (p_p[1]-p_m[1])/(2.0*eps);
    double fd_dv     = (p_p[2]-p_m[2])/(2.0*eps);
    double fd_dom    = (p_p[3]-p_m[3])/(2.0*eps);

    // Allow 1% relative tolerance: float analytic vs double FD
    auto check_rel = [&](const char* name, float got, double expected) {
        float tol = std::max(1e-3f, 0.01f * std::abs(float(expected)));
        check(name, got, float(expected), tol);
    };
    check_rel("dtheta_dT", derivs.dtheta_dT, fd_dtheta);
    check_rel("dphi_dT",   derivs.dphi_dT,   fd_dphi);
    check_rel("dvexit_dT", derivs.dvexit_dT, fd_dv);
    check_rel("domega_dT", derivs.domega_dT, fd_dom);
}

// Verify feasible_interval contains a broad range and exclude T values
// where the lob check fails.
void test_feasible_interval() {
    printf("\n=== test_feasible_interval ===\n");
    PhysicsConfig cfg{9.81f, 0.1f};
    TurretBounds bounds{-3.14f, 3.14f, 0.f, 1.4f, 15.f, 1e6f};

    TInterval iv = ballistics_feasible_interval(3.f, 0.f, 1.f, 0.f, 0.f, bounds, cfg, 1e-4f);
    printf("  feasible T: [%.4f, %.4f]\n", iv.t_lo, iv.t_hi);
    assert(iv.t_lo < iv.t_hi && "Expected a non-empty feasible interval");
    assert(iv.t_lo > 0.f);
    printf("PASS feasible_interval basic\n");

    // Check that the midpoint is actually feasible
    float T_mid = 0.5f * (iv.t_lo + iv.t_hi);
    OmegaMapParams om{};
    ShotParams p = ballistics_solve(0,0,0, 3,0,1, 0,0, T_mid, cfg, om);
    bool feas = ballistics_is_feasible(p, T_mid, bounds, cfg, om);
    printf("%s is_feasible at midpoint (T=%.4f)\n", feas ? "PASS" : "FAIL", T_mid);
}

// Verify Lipschitz bounds are not violated by sampling.
void test_lipschitz_bounds() {
    printf("\n=== test_lipschitz_bounds ===\n");
    PhysicsConfig cfg{9.81f, 0.1f};
    float dx=3.f, dy=0.f, dz=1.f, vRx=0.f, vRy=0.f;

    TurretBounds bounds{-3.14f, 3.14f, 0.f, 1.4f, 15.f, 1e6f};
    TInterval iv = ballistics_feasible_interval(dx, dy, dz, vRx, vRy, bounds, cfg, 1e-4f);
    float t_lo = iv.t_lo, t_hi = iv.t_hi;

    LipschitzBounds lip = ballistics_lipschitz(dx, dy, dz, vRx, vRy, t_lo, t_hi, cfg);

    OmegaMapParams om{};
    int N = 1000;
    float max_d_theta = 0.f, max_d_phi = 0.f, max_d_v = 0.f;
    for (int i = 0; i < N; ++i) {
        float t1 = t_lo + (t_hi - t_lo) * float(i)   / float(N);
        float t2 = t_lo + (t_hi - t_lo) * float(i+1) / float(N);
        ShotParams s1 = ballistics_solve(0,0,0, dx,dy,dz, vRx,vRy, t1, cfg, om);
        ShotParams s2 = ballistics_solve(0,0,0, dx,dy,dz, vRx,vRy, t2, cfg, om);
        float dt = t2 - t1;
        max_d_theta = std::max(max_d_theta, std::abs(s2.theta - s1.theta) / dt);
        max_d_phi   = std::max(max_d_phi,   std::abs(s2.phi   - s1.phi)   / dt);
        max_d_v     = std::max(max_d_v,     std::abs(s2.v_exit - s1.v_exit) / dt);
    }

    printf("L_theta: bound=%.4f  observed=%.4f  ratio=%.2f  %s\n",
        lip.l_theta, max_d_theta, lip.l_theta/max_d_theta,
        lip.l_theta >= max_d_theta ? "PASS" : "FAIL");
    printf("L_phi:   bound=%.4f  observed=%.4f  ratio=%.2f  %s\n",
        lip.l_phi, max_d_phi, lip.l_phi/max_d_phi,
        lip.l_phi >= max_d_phi ? "PASS" : "FAIL");
    printf("L_vexit: bound=%.4f  observed=%.4f  ratio=%.2f  %s\n",
        lip.l_vexit, max_d_v, lip.l_vexit/max_d_v,
        lip.l_vexit >= max_d_v ? "PASS" : "FAIL");
}

int main() {
    test_solve_basic();
    test_dtheta_dT();
    test_derivs_fd();
    test_feasible_interval();
    test_lipschitz_bounds();
    printf("\nDone.\n");
    return 0;
}
