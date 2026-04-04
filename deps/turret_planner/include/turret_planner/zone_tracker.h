#pragma once
#include "types.h"
#include "flight_time.h"
#include "mecanum_model.h"

// ---------------------------------------------------------------------------
// Probabilistic zone-entry pre-positioning controller.
//
// Models the robot's progress toward a half-plane boundary
//   z(t) = n·p(t) - d
// as a 1D Gaussian with accumulated process noise. The probability of having
// entered the zone by time τ(T*) (turret readiness time) drives a graduated
// urgency signal that blends the turret toward the predicted crossing-point shot.
// ---------------------------------------------------------------------------

struct ZoneTrackerState {
    float urgency;          // raw zone-entry probability Φ(μ_z / σ_z)
    float urgency_filtered; // low-passed urgency (hysteresis)
    float effort;           // [0,1] graduated effort: 0 = idle, 1 = full pre-position
    float tau;              // current readiness time estimate (s), used as look-ahead
    TurretState target;     // recommended turret state
    bool  should_fire;      // true when in zone and shot is feasible
};

class ZoneTracker {
public:
    ZoneTracker(const ZoneConfig&     zone,
                const TurretWeights&  weights,
                const TurretBounds&   bounds,
                const PhysicsConfig&  physics,
                const OmegaMapParams& omega);

    // Call once per control tick (dt = elapsed seconds since last call).
    ZoneTrackerState update(
        const RobotState&  robot,
        const TurretState& turret,
        float target_x, float target_y, float target_z,
        float dt
    );

    void reset();

    // Read-only access for diagnostics.
    float urgency_filtered() const { return urgency_lpf_; }
    float last_tau()         const { return tau_last_; }

private:
    ZoneConfig     zone_;
    TurretWeights  weights_;
    TurretBounds   bounds_;
    PhysicsConfig  physics_;
    OmegaMapParams omega_;

    float urgency_lpf_;
    float tau_last_;    // τ from previous tick — used as look-ahead for Gaussian
    bool  armed_;
};
