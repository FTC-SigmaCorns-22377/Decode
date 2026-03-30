# Optimized Shoot-While-Move: Mathematical Specification

This document specifies the ballistic trajectory solver for shoot-while-move.
A valid shot trajectory depends on exit velocity `v_exit`, hood pitch `phi`, and turret yaw `theta`.
Given the target position and robot position + velocity, a valid shot has **1 degree of freedom**,
which we parameterize by travel time `T`. This enables optimizing which shot to take.

## Notation

| Symbol | Description |
|--------|-------------|
| `v_exit` | Ball exit velocity magnitude (m/s) |
| `phi` | Hood pitch angle (rad), 0 = horizontal, pi/2 = vertical |
| `theta` | Turret yaw angle (rad), field-frame |
| `T` | Ball travel time from launch to target (s) |
| `(x_target, y_target, z_target)` | Goal position in field frame (m) |
| `(x_turret, y_turret, z_turret)` | Turret/launch position in field frame (m) |
| `(v_rx, v_ry)` | Robot velocity in field frame (m/s) |
| `r_h` | Hood arc radius (m) - distance from pivot to ball release |
| `g` | Gravitational acceleration (9.81 m/s^2) |
| `omega` | Flywheel angular velocity (rad/s) |
| `phi_0, omega_0, theta_0` | Current actuator states |

---

## 1. Fixed Travel Time Subproblem

**Problem:** Given a travel time `T`, find shot parameters `(v_exit, phi, theta)` so the ball
collides with the target.

**Decision variables:** `(v_exit, phi, theta)`

**Parameters:** `(x_target, y_target, z_target, x_turret, y_turret, z_turret, v_rx, v_ry)`

### 1a. Without Hood Arc

The projectile equations in 3D with robot velocity:

```
v_exit * cos(phi) * cos(theta) = (x_target - x_0) / T - v_rx
v_exit * cos(phi) * sin(theta) = (y_target - y_0) / T - v_ry
v_exit * sin(phi)              = (z_target - z_0) / T + g*T/2
```

This is of the form:

```
v_exit * cos(phi) * cos(theta) = a
v_exit * cos(phi) * sin(theta) = b
v_exit * sin(phi)              = c
```

With the closed-form solution:

```
theta  = atan2(b, a)
phi    = atan2(c, sqrt(a^2 + b^2))
v_exit = sqrt(a^2 + b^2 + c^2)
```

### 1b. With Hood Arc

The hood arc offsets the initial launch position by `r_h` as a function of `phi`:

```
x_0 = x_turret - r_h * sin(phi) * cos(theta)
y_0 = y_turret - r_h * sin(phi) * sin(theta)
z_0 = z_turret + r_h * cos(phi)
```

Substituting into the projectile equations:

```
v_exit*cos(phi)*cos(theta) = (x_target - x_turret + r_h*sin(phi)*cos(theta)) / T - v_rx
v_exit*cos(phi)*sin(theta) = (y_target - y_turret + r_h*sin(phi)*sin(theta)) / T - v_ry
v_exit*sin(phi)            = (z_target - z_turret - r_h*cos(phi)) / T + g*T/2
```

This is of the form:

```
v_exit*cos(phi)*cos(theta) = a*sin(phi)*cos(theta) + b
v_exit*cos(phi)*sin(theta) = a*sin(phi)*sin(theta) + c
v_exit*sin(phi)            = a*cos(phi) + d
```

**Solving for theta:** Factor out `cos(theta)` and `sin(theta)`:

```
cos(theta) * (v_exit*cos(phi) - a*sin(phi)) = b
sin(theta) * (v_exit*cos(phi) - a*sin(phi)) = c
```

Therefore: `theta = atan2(c, b)` -- **same theta as the no-hood-arc case**.

**Solving for phi and v_exit:** Once theta is known, rotate the reference frame by `-theta`.
The equations reduce to:

```
v_exit*cos(phi) - A*sin(phi) = B
v_exit*sin(phi) + A*cos(phi) = C
```

where:

```
A = r_h / T
B = ((x_target-x_turret)*cos(theta) + (y_target-y_turret)*sin(theta)) / T
    - v_rx*cos(theta) - v_ry*sin(theta)
C = (z_target - z_turret) / T + g*T/2
```

This is equivalent to the matrix equation `R(phi) * [v_exit, A]^T = [B, C]^T`.

Squaring and adding both equations:

```
v_exit^2 + A^2 = B^2 + C^2
v_exit = sqrt(B^2 + C^2 - A^2)
```

Solving the 2x2 system for cos(phi) and sin(phi):

```
cos(phi) = (B*v_exit + C*A) / (B^2 + C^2)
sin(phi) = (C*v_exit - B*A) / (B^2 + C^2)
phi = atan2(C*v_exit - B*A, B*v_exit + C*A)
```

### Final Procedure (with hood arc)

1. `theta = atan2((y_target-y_0)/T - v_ry, (x_target-x_0)/T - v_rx)`
2. Compute rotated frame constants:
   - `A = r_h / T`
   - `B = ((x_target-x_turret)*cos(theta) + (y_target-y_turret)*sin(theta)) / T - v_rx*cos(theta) - v_ry*sin(theta)`
   - `C = (z_target - z_turret) / T + g*T/2`
3. `v_exit = sqrt(B^2 + C^2 - A^2)`
4. `phi = atan2(C*v_exit - B*A, B*v_exit + C*A)`

---

## 2. Transition Cost Function

It takes time for the robot's actuators to reach new targets. Define `J_delta` as the
time to transition from the current state to the shot parameters for travel time `T`.

Define `omega(phi, v_exit)` as the flywheel speed needed for a given exit velocity
and hood angle (calibrated from real-world data).

```
J_delta(T, omega_0, phi_0, theta_0) = max(
    Q_phi   * |phi(T)   - phi_0|,
    Q_omega * |omega(phi(T), v_exit(T)) - omega_0|,
    Q_theta * |theta(T) - theta_0|
)
```

Where `Q_phi`, `Q_omega`, `Q_theta` are settling time coefficients:
the time for a unit change in hood angle, flywheel speed, and turret angle respectively.

---

## 3. Optimal Shot Adjustment

**Objective:** Find `T` that minimizes transition time from current actuator state.

```
minimize    J_delta(T, omega_0, phi_0, theta_0)
   T

subject to  phi_min    <= phi(T)                    <= phi_max
            omega_min  <= omega(phi(T), v_exit(T))  <= omega_max
```

This has only 1 decision variable and can be warm-started from the previous solution.
Golden-section search on a small interval around the last solved `T` is effective.

---

## 4. Robust Shot Target

When preparing a multi-ball shot sequence, target parameters for the first shot
that minimize recovery time for the second shot, accounting for flywheel speed
loss `Delta_omega` after firing.

```
minimize      J_delta(T_2, omega(phi(T_1), v(T_1)) - Delta_omega, phi(T_1), theta(T_1))
  T_1, T_2

subject to    phi_min   <= phi(T_i)                   <= phi_max     for i in {1, 2}
              omega_min <= omega(phi(T_i), v_exit(T_i)) <= omega_max  for i in {1, 2}
```

This requires a 2D sweep and is computed once per shot sequence (not per frame).

---

## 5. Bounds on T

### Loose bounds

1. Find `v_max` from the flywheel map: max exit velocity at any valid phi
2. `T_min = sqrt(dx^2 + dy^2) / v_max`
3. `T_max`: sweep until constraints become invalid

### Tighter T_min bounds

Line search offline for maximum `v` where `omega(phi, v) < omega_max` to find `v_max`.

Increasing hood angle moves the launch point away from the target, so use phi=0
(launch point directly above robot) for a slight underestimate of T_min.

The projectile equations with phi=0 simplify to a quartic in T:

```
-g^2/4 * T^4 + (v_max^2 + |v_r|^2 - dz*g) * T^2
    + 2*T*(dx + dy) - |delta|^2 = 0
```

Find the smallest positive root via Newton's method (warm-started from the loose bound)
or direct quartic solution.

If the phi at this T_min is within bounds, use it. Otherwise solve the phi-limited
version (fixing phi = phi_min or phi_max) which yields a different quartic.

---

## 6. Lipschitz Constants

Over an interval `0 < T_min <= T <= T_max`, Lipschitz constants bound how fast each
parameter changes with T, enabling efficient interval pruning during search.

### For theta

```
d(theta)/dT = (-v_rx + v_ry) / |delta_xy - T*v_r|^2

L_theta(T_min, T_max) = |(-v_rx + v_ry)| / |delta_xy - clamp(T_min, T*, T_max)*v_r|^2
```

where `T* = (delta_xy . v_r) / |v_r|^2`

### For A, B, C (rotated frame constants)

```
dA/dT = -r_h / T^2                    L_A = r_h / T_min^2
dB/dT = L_theta * (|delta_xy|/T_min + |v_r|) + |delta_xy| / T_min^2
dC/dT = -dz / T^2 + g/2              L_C = dz / T_min^2 + g/2
```

### For v_exit and phi

Derived from the coupled differential equations:

```
d(phi)/dT = (1/cos(phi)) * dC/dT - (1 + 1/sin(phi)) * dA/dT - dB/dT
d(v)/dT   = (A*cos(phi) + sin(phi))/cos(phi) * d(phi)/dT
            + (1/cos(phi)) * (dA/dT + dB/dT * sin(phi))
```

These are used for future interval-search optimization and are not required
for the golden-section search implementation.

---

## Code Mapping

| Math Symbol | Code Location | Field/Method |
|-------------|---------------|--------------|
| `(x_target, y_target)` | `FieldLandmarks.kt` | `goalPosition(blue)` |
| `z_target` | `HoodConfig` | `goalHeight` |
| `z_turret` | `HoodConfig` | `launchHeight` |
| `(v_rx, v_ry)` | `SigmaIO` | `velocity().v` |
| `omega` | `FlywheelMap.kt` | `getOmega(vExit)` |
| `phi` limits | `HoodConfig` | `minAngleDeg`, `maxAngleDeg` |
| `omega` limits | `AdaptiveTuner` | `MIN/MAX_FLYWHEEL_SPEED` |
| `theta` limits | `TurretServoConfig` | `minAngle`, `maxAngle` |
| `Q_phi, Q_omega, Q_theta` | `BallisticConstants` | Tunable coefficients |
