#include "turret_planner/zone_tracker.h"
#include "turret_planner/mecanum_model.h"
#include "turret_planner/math/gaussian.h"
#include "turret_planner/math/fast_trig.h"
#include <cmath>
#include <algorithm>

ZoneTracker::ZoneTracker(const ZoneConfig&     zone,
                         const TurretWeights&  weights,
                         const TurretBounds&   bounds,
                         const PhysicsConfig&  physics,
                         const OmegaMapParams& omega)
    : zone_(zone)
    , weights_(weights)
    , bounds_(bounds)
    , physics_(physics)
    , omega_(omega)
    , urgency_lpf_(0.f)
    , tau_last_(0.5f)   // conservative initial look-ahead
    , armed_(false)
{}

ZoneTrackerState ZoneTracker::update(
    const RobotState&  robot,
    const TurretState& turret,
    float target_x, float target_y, float target_z,
    float /*dt*/)
{
    ZoneTrackerState out{};

    // -----------------------------------------------------------------------
    // 1. Probabilistic zone-entry estimate.
    //    Look-ahead time = τ from previous tick (updated below once T* is known).
    //    z(t) = n·p(t) - d, propagated as a Gaussian.
    // -----------------------------------------------------------------------
    float t_prep = tau_last_;  // best estimate of how long until turret is ready

    ZoneProjection proj = mecanum_project_zone(
        zone_.nx, zone_.ny, zone_.d,
        robot.x,  robot.y,
        robot.vx, robot.vy,
        0.f,              // initial position variance (trust odometry)
        zone_.Q_process,
        t_prep
    );

    // urgency = P(robot inside zone when turret is ready) = Φ(μ_z / σ_z)
    float urgency_raw = fast_gaussian_cdf(proj.mu_z / proj.sigma_z);

    // -----------------------------------------------------------------------
    // 2. Low-pass filter (hysteresis)
    // -----------------------------------------------------------------------
    urgency_lpf_ = zone_.alpha_lpf * urgency_lpf_
                 + (1.f - zone_.alpha_lpf) * urgency_raw;

    float effort = std::clamp(
        (urgency_lpf_ - zone_.p_low) / (zone_.p_high - zone_.p_low + 1e-9f),
        0.f, 1.f
    );

    // -----------------------------------------------------------------------
    // 3. Predict the boundary-crossing position.
    //    Solve n·(p + v*t) = d  →  t_cross = -z0 / (n·v).
    //    Use t_cross to predict where the robot will be when it enters the zone.
    // -----------------------------------------------------------------------
    float z0      = zone_.nx * robot.x + zone_.ny * robot.y - zone_.d;
    float n_dot_v = zone_.nx * robot.vx + zone_.ny * robot.vy;
    float t_cross = (std::abs(n_dot_v) > 1e-6f) ? (-z0 / n_dot_v) : 1e6f;
    t_cross = std::clamp(t_cross, 0.f, 5.f);

    float px = robot.x + robot.vx * t_cross;
    float py = robot.y + robot.vy * t_cross;

    // Turret pivot assumed at robot origin; caller adjusts target_z / turret_z
    // to account for physical offsets.
    const float turret_x = px, turret_y = py, turret_z = 0.f;

    // -----------------------------------------------------------------------
    // 4. Compute T* and blend turret target proportional to effort.
    // -----------------------------------------------------------------------
    TurretState target_state = turret;
    target_state.omega_flywheel = std::max(turret.omega_flywheel, zone_.omega_idle);

    float tau_new = tau_last_;  // will be updated if feasible

    if (effort > 1e-3f) {
        FlightTimeResult res = flight_time_cold(
            turret_x, turret_y, turret_z,
            target_x, target_y, target_z,
            robot.vx, robot.vy,
            turret, weights_, bounds_, physics_, omega_);

        if (res.feasible) {
            tau_new = res.tau;   // update look-ahead for next tick

            float e = effort;
            target_state.theta = turret.theta
                               + e * (res.params.theta - turret.theta);
            target_state.phi   = turret.phi
                               + e * (res.params.phi   - turret.phi);
            target_state.omega_flywheel = std::max(
                turret.omega_flywheel + e * (res.params.omega_flywheel - turret.omega_flywheel),
                zone_.omega_idle
            );
        }
    }

    tau_last_ = tau_new;

    // -----------------------------------------------------------------------
    // 5. Fire check: in zone + feasible shot from current position.
    // -----------------------------------------------------------------------
    bool in_zone    = (z0 >= 0.f);
    bool should_fire = false;
    if (in_zone) {
        FlightTimeResult res = flight_time_warm(
            robot.x, robot.y, 0.f,
            target_x, target_y, target_z,
            robot.vx, robot.vy,
            (tau_last_ > 0.f) ? tau_last_ : 0.5f,
            turret, weights_, bounds_, physics_, omega_);
        should_fire = res.feasible;
    }

    out.urgency          = urgency_raw;
    out.urgency_filtered = urgency_lpf_;
    out.effort           = effort;
    out.tau              = tau_last_;
    out.target           = target_state;
    out.should_fire      = should_fire;
    return out;
}

void ZoneTracker::reset() {
    urgency_lpf_ = 0.f;
    tau_last_    = 0.5f;
    armed_       = false;
}
