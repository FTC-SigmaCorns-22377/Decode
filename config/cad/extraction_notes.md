# STEP extraction notes — Cam Global Shutter mount

Source: `~/Downloads/Top Level (2).step` (152 MB, AP242, exported from Onshape).
Parser: `tools/parse_step.py` (in-repo, re-runnable; see memory entry
`step_parser_tool.md`).
Date recorded: 2026-04-19.

## Final derived values (committed to `config/ball_tracker.json`)

```
CAM_POS_R          = (0.162, 0.099, 0.242)  m   # robot frame: x fwd, y left, z up
CAM_PITCH_DOWN_RAD = -0.199                 rad # camera pitched UP 11.4° (NOT down — see "Open verifications" below)
CAM_YAW_RAD        = 0.0
CAM_ROLL_RAD       = 0.0
ROBOT_FOOTPRINT_R  = axis-aligned rectangle ±0.184 × ±0.183 m
                     (from RobotModelConstants.robotSize = (0.367, 0.365))
```

## Inputs

1. **From `tools/parse_step.py`** — Cam Global Shutter (NAUO579) walked up the
   assembly chain `Cam Global Shutter ← global stutter assembly <1> ← full DT <1> ← Top Level`.
   Composed transform (in `Top Level` frame, units: metres):

   ```
   position : (0.797085, 0.000000, 0.130711)
   rotation : [[ -6.4e-17  -0.197357  -0.980332 ]
               [ -1.0       -3.7e-17    7.3e-17  ]
               [ -5.1e-17    0.980332  -0.197357 ]]
   ```

   Columns of the rotation matrix = camera-local OpenCV axes
   (image-right, image-down, optical-axis) expressed in `Top Level` frame.

2. **From the user (Onshape Measure tool)** — camera position relative to the
   chassis "back-left-bottom corner" (in the same `Top Level` Onshape global
   frame as the STEP), in millimetres:

   ```
   (-346.081, -83.748, -242.268)
   ```

3. **From `RobotModelConstants.kt`** — robot frame x-fwd / y-left / z-up,
   origin at chassis geometric center, `robotSize = (0.367, 0.365)` m.
   So back-left-bottom corner of the chassis in robot frame is
   `(-0.184, +0.183, 0.000)` m.

## Derivation

### Step A — locate the corner in the Top Level frame

```
corner_TL = camera_TL  -  user_offset
          = (0.797, 0.000, 0.131)  -  (-0.346, -0.084, -0.242)
          = (1.143, 0.084, 0.373)  m
```

### Step B — pick the axis mapping `Top Level → robot`

For the camera's rotation matrix to describe a physically-mounted camera
(image-right pointing one way, image-down a perpendicular way, optical-axis
forward), only one axis-flip choice is internally consistent:

```
v_robot = diag(-1, +1, -1)  ·  v_TL  +  t
         (TL +X = robot back, TL +Y = robot left, TL +Z = robot down)
```

Sanity-check under this mapping:
- camera image-right `(0, -1, 0)` in TL  →  `(0, -1, 0)` in robot = robot right ✓
- camera image-down `(-0.197, 0, +0.980)` in TL  →  `(+0.197, 0, -0.980)` in robot = mostly robot down ✓
- optical axis `(-0.980, 0, -0.197)` in TL  →  `(+0.980, 0, +0.197)` in robot = forward (and slightly up) ✓ in direction, but see "Open verifications".

### Step C — solve for the translation `t`

```
corner_robot       = R · corner_TL + t
(-0.184, +0.183, 0) = diag(-1,+1,-1) · (1.143, 0.084, 0.373) + t
                   = (-1.143, +0.084, -0.373) + t

t = (0.959, 0.099, 0.373)  m
```

### Step D — apply to the camera

```
camera_robot = R · camera_TL + t
             = (-1, +1, -1) · (0.797, 0.000, 0.131) + (0.959, 0.099, 0.373)
             = (-0.797, 0.000, -0.131) + (0.959, 0.099, 0.373)
             = (0.162, 0.099, 0.242)  m   ✓
```

### Step E — extract Euler angles

Camera rotation in robot frame = `diag(-1, +1, -1) · R_TL`. From its columns:
- optical axis `(+0.980, 0, +0.197)` — forward, pitched **UP** by `asin(0.197) ≈ 11.4°`.
- image-down `(+0.197, 0, -0.980)` — straight down with a small forward component (matches the up-tilt).
- image-right `(0, -1, 0)` — robot right.

So `CAM_PITCH_DOWN_RAD = -asin(0.197) = -0.199` rad (negative because the
optical axis is *above* the horizon), `CAM_YAW_RAD = 0`, `CAM_ROLL_RAD = 0`.

## Open verifications (please sanity-check before tracker is run on real hardware)

1. **Pitch sign.** The CAD says the optical axis is 11.4° **above** horizontal.
   The original prompt assumed 15.4° below horizontal. Possibilities:
   - The mount changed since the prompt was written — the new value is the truth.
   - "Bottom-left corner" in the user's measurement is actually a top corner,
     in which case the axis-flip on Z is inverted. Under that alternative,
     the camera ends up at `(0.162, 0.099, 0.148)` m and pitched +11.4° down,
     matching expectations more closely.

   The two interpretations are observable: power up the camera and look at a
   live feed from the GlobalShutter pipeline. If the floor near the robot is
   visible in the lower part of the image, the current numbers are correct.
   If you mostly see the ceiling and far field, flip `CAM_PITCH_DOWN_RAD` and
   adjust `CAM_POS_R[2]`.

2. **"Bottom-left corner" identity.** I assumed back-left-bottom (the only
   choice that puts the camera *inside* the chassis along +X). If the user
   meant front-left-bottom, the camera ends up at `(-0.530, +0.099, +0.242)` m
   (530 mm behind the robot center) — physically impossible.

3. **Camera intrinsics (`K`, `DIST_COEFFS`, `IMAGE_*_PX`)** are placeholders
   in `config/ball_tracker.json` (nominal 1280×720, fx=fy=900, no distortion).
   Calibrate the actual Cam Global Shutter before any hardware test.

4. **`ROBOT_FOOTPRINT_R`** is an axis-aligned rectangle from
   `RobotModelConstants.robotSize`. For accurate self-occlusion gating,
   replace with the convex hull of the chassis (including bumpers, projecting
   intake, etc.) — extractable via `cadquery` once installed.

5. **`RAMP_POLYGON`** is empty. Fill from the FTC DECODE 2025-2026 game
   manual / field drawings.

## Re-running the parser

```
python3 tools/parse_step.py [path/to/file.step]
```

Default path: `~/Downloads/Top Level (2).step`. Output for the camera will be
the row labelled `NAUO579 Cam Global Shutter` — values in metres in the
`Top Level` frame.
