# TGU Robocore 2027

## Armor auto-aim v0.1

This branch provides the first Robocore-integrated armor auto-aim baseline for
alliance matches. It detects and tracks robot armor only. Outpost, base and
energy-mechanism targets are outside the v0.1 scope.

The armor pipeline is:

```text
USB camera -> YOLOv8 -> armor classifier -> PnP/yaw optimization
           -> tracker/EKF -> ballistic/MPC planner -> gimbal command
```

The implementation is adapted from TongjiSuperPower `sp_vision_25`. Its MIT
license is retained in `LICENSES/sp_vision_25-MIT.txt`.

### v0.1 scope

- YOLOv8 armor detection with OpenVINO.
- Robot targets: hero, engineer, infantry and sentry.
- PnP pose solving and armor yaw optimization.
- Whole-vehicle EKF tracking and MPC command planning.
- Existing Robocore serial, logger, TOML and Foxglove infrastructure retained.
- No outpost targeting and no energy-mechanism algorithm.
- Automatic fire is disabled by default and additionally gated by measured
  gimbal yaw/pitch error when enabled.

### Dependencies

- Ubuntu 24.04 or a distribution providing GCC 13+
- CMake 3.16+
- C++20 compiler with `std::format` support (GCC 13+ recommended)
- Boost.System
- OpenCV
- Eigen3
- OpenVINO Runtime development package
- Aravis 0.8

For the dependencies available from Ubuntu:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build g++ \
  python3-venv python3-pip libboost-all-dev libopencv-dev \
  libeigen3-dev libaravis-dev aravis-tools aravis-tools-cli \
  libusb-1.0-0-dev
```

Install the pinned OpenVINO build in a project-local virtual environment. The
2026.3 wheel uses the same libstdc++ ABI as Ubuntu 24.04's OpenCV packages and
can load the checked-in OpenVINO 2024 IR model.

```bash
python3 -m venv .venv-openvino
.venv-openvino/bin/python -m pip install -r requirements-openvino.txt
```

CMake automatically discovers this local installation. To use an official
system installation instead, pass its package directory explicitly with
`-DOpenVINO_DIR=/path/to/openvino/cmake`.

### Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### Run

Review `config/auto_aim.toml` first. The checked-in calibration is a migration
reference and must be replaced with calibration from the actual camera/gimbal
pair.

```bash
./build/auto_aim_sentry config/auto_aim.toml
```

Offline replay:

```bash
./build/auto_aim_test --config-path=config/auto_aim.toml assets/demo/demo
```

Replay video and pose text are not bundled in v0.1.

### Safety and limitations

- `[fire].enabled` defaults to `false`. Enable it only after verifying the
  serial protocol, coordinate convention and calibration on a test rig.
- Camera source, requested resolution and frame rate are configured under
  `[camera]` in `config/auto_aim.toml`.
- YOLOv5 and YOLO11 are not included in v0.1; YOLOv8 is the only selectable
  detector backend.
- Entering a buff mode does not run a buff detector; the program sends a
  disabled control command instead.

See `app/auto_aim/README.md` for module boundaries and validation targets, and
`docs/auto_aim_v0.1_review.md` for the Chinese review, known risks and roadmap.

### Existing Hikrobot USB rule

The original framework's Hikrobot camera test may require:

```bash
sudo tee /etc/udev/rules.d/99-hikrobot.rules <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", ATTRS{idProduct}=="0001", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```
