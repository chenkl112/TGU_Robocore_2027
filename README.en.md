# TGU Robocore 2027

An experimental armor auto-aim program for alliance-match robots, built on Robocore with YOLOv8, whole-vehicle EKF tracking and TinyMPC gimbal control. Calibration and closed-loop validation must be completed on the actual hardware.

[简体中文](README.md) | [English](README.en.md)

The `auto-aim-v0.1` branch supports hero, engineer, infantry and sentry robots. It does not target outposts, bases or energy mechanisms.

```text
USB camera → YOLOv8 → number classification → PnP / armor yaw optimization
           → whole-vehicle EKF → Planner trajectory → TinyMPC → gimbal command
```

## Build and numeric check

Run these commands from the root of this branch. Use Ubuntu 24.04 or a Linux environment with GCC 13+ and C++20 `std::format` support.

1. Install the system dependencies, including CMake 3.16+, OpenCV, Eigen3, Boost.System and Aravis 0.8:

   ```bash
   sudo apt update
   sudo apt install -y build-essential cmake ninja-build pkg-config g++ \
     python3-venv python3-pip libboost-all-dev libopencv-dev \
     libeigen3-dev libaravis-dev aravis-tools aravis-tools-cli libusb-1.0-0-dev
   ```

2. Install the pinned OpenVINO packages:

   ```bash
   python3 -m venv .venv-openvino
   .venv-openvino/bin/python -m pip install -r requirements-openvino.txt
   ```

   CMake searches this virtual environment for OpenVINO. To use a system installation, add `-DOpenVINO_DIR=your-package-cmake-directory` to the configuration command below.

3. Build and run the MPC numeric check, which needs no camera or serial device:

   ```bash
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ./build/planner_test --config-path=config/auto_aim.toml
   ```

   A successful check exits with status 0 and prints angles, angular velocities and angular accelerations. It checks for finite control outputs; it does not validate detection, tracking or the hardware control loop.

## Run with hardware

Edit [config/auto_aim.toml](config/auto_aim.toml) first. Replace the camera intrinsics, distortion and camera-to-gimbal transform, and check the serial protocol. The bundled calibration is a migration reference.

```bash
./build/auto_aim_sentry config/auto_aim.toml
```

The program opens the gimbal serial port before the camera. Missing devices or serial permissions prevent startup. Common settings are:

| Setting | Default | Purpose |
|---|---|---|
| `camera.source` | `0` | Camera index, device path, video file or stream address |
| `camera.width` / `camera.height` | `0` / `0` | Keep the camera's default capture resolution |
| `camera.fps` | `200.0` | Requested rate, clamped to 1–200 FPS; nonpositive values select 200 |
| `tracker.enemy_color` | `red` | Enemy color; check against your team's side before running |
| `tracker.priority_mode` | `1` | Mode 1 prioritizes robots 3/4; mode 2 prioritizes robot 2 |
| `gimbal.com_port` | `/dev/ttyUSB0` | Gimbal serial device |
| `planner.auto_fire` | `false` | Automatic fire switch |

Keep automatic fire disabled until the serial protocol, coordinate signs, calibration and MPC outputs have been checked on a test rig. The hardware entry point uses Planner/TinyMPC. `Aimer` and `Shooter` remain in the source but are not called by this entry point.

## Offline replay

Replay requires an `.avi` video and a `.txt` pose record sharing the same prefix. Recordings are not bundled. Set `REPLAY_PREFIX` to your recording prefix, then check that both files exist:

```bash
: "${REPLAY_PREFIX:?Set the recording path prefix without an extension}"
test -f "${REPLAY_PREFIX}.avi" && test -f "${REPLAY_PREFIX}.txt" && \
  ./build/auto_aim_test --config-path=config/auto_aim.toml "$REPLAY_PREFIX"
```

## Detection models

The default model is [assets/yolov8_640x480.xml](assets/yolov8_640x480.xml) with its matching `.bin` weights. The detector checks the `640 × 480` input and `1 × 14 × 6300` output shapes at startup. YOLOv8 is the only supported backend.

Capture resolution can differ from network resolution: frames are resized with their aspect ratio preserved, then padded on the right or bottom to `640 × 480`. The original `416 × 416` IR is retained for provenance and conversion; the current detector rejects it. To regenerate the default model:

```bash
.venv-openvino/bin/python tools/convert_yolov8_640x480.py
```

Conversion overwrites the default `640 × 480` model and runs a zero-input inference check on the CPU.

## Hikrobot USB permissions

The original framework's Hikrobot camera test may require this udev rule:

```bash
sudo tee /etc/udev/rules.d/99-hikrobot.rules <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", ATTRS{idProduct}=="0001", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Code and model sources

The auto-aim implementation is adapted from TongjiSuperPower `sp_vision_25`. Its MIT license is preserved in [LICENSES/sp_vision_25-MIT.txt](LICENSES/sp_vision_25-MIT.txt). Model licensing is documented separately in the [third-party notices](LICENSES/README.md); the upstream MIT license does not license every file in this repository.

See the [auto-aim module notes](app/auto_aim/README.md) for module boundaries, coordinates and units, and the [v0.1 review](docs/auto_aim_v0.1_review.md) for validation gaps and planned work.
