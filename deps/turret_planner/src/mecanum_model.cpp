#include "turret_planner/mecanum_model.h"
#include "turret_planner/math/fast_trig.h"
#include <algorithm>

ZoneProjection mecanum_project_zone(
    float nx, float ny, float d,
    float px, float py,
    float vx, float vy,
    float cov_pos_0,
    float Q_pos,
    float t_pred)
{
    // Predicted position mean at t_pred
    float mu_x = px + vx * t_pred;
    float mu_y = py + vy * t_pred;

    // Signed distance mean: n·μ_p - d
    float mu_z = nx * mu_x + ny * mu_y - d;

    // Variance: initial + accumulated process noise
    float var_z = cov_pos_0 + Q_pos * t_pred;
    if (var_z < 1e-9f) var_z = 1e-9f;

    ZoneProjection proj;
    proj.mu_z   = mu_z;
    proj.var_z  = var_z;
    proj.sigma_z = fast_sqrt(var_z);
    return proj;
}
