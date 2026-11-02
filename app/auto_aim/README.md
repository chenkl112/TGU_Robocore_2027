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
- The YOLO tensor contract is fixed at NHWC `1x480x640x3` before OpenVINO
  preprocessing and NCHW `1x3x480x640` inside the model. The model output is
  `1x14x6300`.

## Required validation before enabling fire

1. Confirm image timestamp and IMU quaternion interpolation direction.
2. Verify PnP depth and reprojection on recorded images.
3. Confirm world/gimbal yaw and pitch sign conventions against the lower board.
4. Tune Tracker and Planner values through offline replay.
5. Verify MPC angle, angular-velocity and angular-acceleration outputs on a disabled-friction-wheel rig.
6. Enable `[planner].auto_fire` only after the previous checks pass.

## Fire and freshness checks

`Planner::plan` accepts optional `io::GimbalState` feedback. Calls without feedback can
produce a control plan but always return `fire = false`. Live feedback includes a local
receive timestamp; both yaw and pitch must be within `planner.fire_thresh` of the
interception reference. Gimbal rechecks the latest feedback against the outgoing command.

`Target::timestamp()` is the predicted state time. `observation_timestamp()` is the time
of the last accepted measurement; prediction, empty frames and rejected measurements do
not refresh it. The live entry point and Planner enforce `planner.max_target_age` using
that measurement time. Every MPC sample solves its own ballistic interception time.

The bundled YOLOv8 pose head emits semantic keypoints in `LT, LB, RB, RT` order.
The decoder maps channels to PnP's `LT, RT, RB, LB` order and preserves their
identities under roll and perspective. It checks finiteness and convexity without
sorting image coordinates or edge lengths. A replacement model must use the same
semantic channel contract, checked by `auto_aim.model_keypoint_contract` using the
bundled weights and generated armor images.

Projection regressions cover small and big armor at 3 m, roll angles 0/±20/±45/±70
degrees, and view angles 0/35/70/80 degrees. OpenCV IPPE results that are nonfinite
or behind the camera are retried with the iterative PnP solver using the same points.

The retained `Shooter` path requires both yaw and pitch command stability and actual
gimbal alignment before returning `true`. The live MPC entry point does not call this
legacy path; its Planner and Gimbal checks independently require current yaw/pitch
feedback before allowing fire. A disconnected Gimbal enters IDLE and retries the
configured serial port at `gimbal.reconnect_delay_ms` until valid feedback returns.
