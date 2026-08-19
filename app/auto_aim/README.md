# Armor auto-aim module

## Scope

Version 0.1 supports robot armor in alliance matches. `Tracker` rejects
`outpost`, `base` and `not_armor` observations. No energy-mechanism source is
included.

## Modules

- `armor`: common observation types and keypoints.
- `classifier`: 32x32 armor-number classification.
- `detector`: traditional light-bar detector retained as a fallback component.
- `yolo` and `yolos/yolov8`: v0.1 neural detector backend.
- `solver`: PnP, coordinate transforms and yaw reprojection optimization.
- `tracker`: target state machine and observation association.
- `target`: 11-state whole-vehicle EKF model, including two-armor balance infantry.
- `planner`: original `standard_mpc` trajectory generation and TinyMPC yaw/pitch control.
- `aimer` and `shooter`: retained traditional control path; not used by the current MPC entry point.
- `multithread`: latest-frame YOLO worker.

## Coordinate and unit contract

- Position: metres.
- Angles: radians inside C++; configuration offsets/tolerances marked `deg`
  are converted on load.
- Time: `std::chrono::steady_clock` timestamps and seconds for floating-point
  intervals.
- The camera matrix, distortion coefficients and camera-to-gimbal transform
  must come from the same physical camera installation.

## Required validation before enabling fire

1. Confirm image timestamp and IMU quaternion interpolation direction.
2. Verify PnP depth and reprojection on recorded images.
3. Confirm world/gimbal yaw and pitch sign conventions against the lower board.
4. Tune Tracker and Planner values through offline replay.
5. Verify MPC angle, angular-velocity and angular-acceleration outputs on a disabled-friction-wheel rig.
6. Enable `[planner].auto_fire` only after the previous checks pass.
