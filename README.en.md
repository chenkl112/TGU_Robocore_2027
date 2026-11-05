# TGU Robocore 2027

An experimental armor auto-aim program for alliance-match robots, built on Robocore with YOLOv8, whole-vehicle EKF tracking and TinyMPC gimbal control. The current `auto-aim-v0.1` branch supports hero, engineer, infantry and sentry robots. It does not target outposts, bases or energy mechanisms. Calibration and closed-loop behavior still need validation on the actual hardware.

[简体中文](README.md) | [English](README.en.md)

```text
Image → YOLOv8 → number classification → size resolution / PnP / yaw refinement
      → whole-vehicle EKF → target snapshot → ballistic interception and TinyMPC → command
```

Live operation and offline replay share the tracking and planning implementations. Replay processes recorded timestamps in sequence, supports headless operation and exports per-frame CSV results.

## Build and test

Run these commands from the repository root. Use a compiler with C++20 `std::format`, such as GCC 13 on Ubuntu 24.04. The CTest commands below require CMake 3.20+.

Install the dependencies for algorithm tests:

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build libeigen3-dev libopencv-dev
```

For geometry, tracking and planning work, use the core build. It does not search for OpenVINO, Boost or Aravis and does not start hardware devices:

```bash
cmake -S . -B build/core -G Ninja -DCMAKE_BUILD_TYPE=Debug \
  -DBUILD_TESTING=ON \
  -DROBOCORE_WITH_VISION=OFF -DROBOCORE_WITH_IO=OFF -DROBOCORE_WITH_FOXGLOVE=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_OpenVINO=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_Boost=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PkgConfig=TRUE
cmake --build build/core -j4
ctest --test-dir build/core --output-on-failure -j4
./build/core/planner_test --config-path=config/auto_aim.toml
```

Detection, replay and the hardware entry point also need Boost headers and the pinned OpenVINO packages:

```bash
sudo apt install -y libboost-dev python3-venv python3-pip
python3 -m venv .venv-openvino
.venv-openvino/bin/python -m pip install -r requirements-openvino.txt

cmake -S . -B build/full -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/full -j4
ctest --test-dir build/full --output-on-failure -j4
```

CMake uses an explicit `OpenVINO_DIR` first, then searches `.venv-openvino`. Use `-j2` if build memory is limited.

| Option | Default | Controls |
|---|---|---|
| `ROBOCORE_WITH_VISION` | `ON` | OpenVINO detection, synchronous replay and model tests |
| `ROBOCORE_WITH_IO` | `ON` | Serial, gimbal and USB camera; the live entry point requires both vision and IO |
| `ROBOCORE_WITH_FOXGLOVE` | `ON` | Foxglove SDK adapter; the current auto-aim entry point does not publish Foxglove data |
| `ROBOCORE_BUILD_DEMOS` | `ON` | Diagnostic programs such as `planner_test` |
| `ROBOCORE_BUILD_CAMERA_DEMO` | `OFF` | The separate Aravis camera demo |

Aravis is only required for its camera demo. Install `pkg-config libaravis-dev` and configure with `-DROBOCORE_BUILD_CAMERA_DEMO=ON` to build it. Tests and applications link the same production libraries defined in [RobocoreTargets.cmake](cmake/RobocoreTargets.cmake).

## Configuration and hardware operation

Edit [config/auto_aim.toml](config/auto_aim.toml) before using hardware. Replace the reference intrinsics, distortion and extrinsics with calibration for the actual camera/gimbal installation, and check the lower-board protocol.

```bash
./build/full/auto_aim_sentry config/auto_aim.toml
```

The entry point calls `load_auto_aim_config()` once to parse TOML, validate fields and convert units. Modules receive typed configuration values and validate those values when constructed directly. Missing calibration arrays, incorrect types and invalid ranges produce errors identifying the affected field.

Without a resource base, relative paths retain their original meaning relative to the process working directory. To run from another directory, add this to `config/auto_aim.toml`:

```toml
[paths]
base_dir = ".."
```

`base_dir` is relative to the configuration file's directory, so this example selects the repository root. Relative model and camera-file paths use that base; parsed model paths are absolute. Absolute paths and camera stream URLs keep their meaning. Configurations without `base_dir` continue to use the working directory.

| Setting | Bundled default | Meaning |
|---|---|---|
| `camera.source` | `0` | Camera index, device path, video file or stream URL |
| `camera.width` / `camera.height` | `0` / `0` | Keep the device's default resolution |
| `camera.fps` | `200.0` | Requested rate clamped to 1–200 FPS; nonpositive values select 200 |
| `tracker.enemy_color` | `red` | Armor color to track |
| `tracker.priority_mode` | `1` | Mode 1 prioritizes robots 3/4; mode 2 prioritizes robot 2 |
| `gimbal.com_port` | `/dev/ttyUSB0` | Gimbal serial port |
| `gimbal.reconnect_delay_ms` | `250` | Reconnect interval after a disconnection |
| `planner.max_target_age` | `0.1` | Maximum age of the last accepted observation, in seconds |
| `planner.auto_fire` | `false` | Automatic fire switch |

Startup opens the serial port and waits for valid feedback. A serial open failure or no valid packet within three seconds causes startup to fail. The camera worker retries unavailable capture devices; a serial disconnection or stale feedback during operation disables control. Automatic fire also requires fresh gimbal feedback, a matching session, a visible interception plate and alignment. Software regressions do not establish hardware calibration or closed-loop accuracy.

## Offline replay and CSV

A recording consists of an `.avi` video and a `.txt` file with the same prefix. Each text line corresponds to one frame and contains `seconds q_w q_x q_y q_z`. Time uses finite recording-clock seconds in `[0, 1e9]`; processed timestamps must increase strictly. Quaternions must be finite with a valid norm. Team recordings are not bundled; regression tests generate their own images and poses.

Set `REPLAY_PREFIX` to your recording path without an extension:

```bash
: "${REPLAY_PREFIX:?Set the recording path prefix without an extension}"
./build/full/auto_aim_test --config-path=config/auto_aim.toml \
  --device=CPU --headless=true --output=replay.csv "$REPLAY_PREFIX"
```

`--start-index=100 --end-index=199` processes that inclusive range; `--end-index=0` reads to the end. Seeking skips the corresponding pose records. Omit `--headless=true` to display frames, or omit `--output` to skip CSV export.

Each processed frame runs detection, tracking and planning once using the recording clock. CSV rows include detection count, tracking state, target identity, the 11-state vector, state and observation times, control outputs and the stop reason. The CLI supplies no live gimbal feedback, so `fire` remains `0`. Repeated runs in the same environment with the same model and input can be compared; bitwise agreement across inference devices is not guaranteed.

CSV output is written to a temporary file in the destination directory and atomically replaces the destination on normal completion. Recording, model or processing failures return a nonzero exit status and preserve an existing CSV. The output cannot overwrite the recording, configuration or referenced model files.

## Detection models

The default detector uses [yolov8_640x480.xml](assets/yolov8_640x480.xml) and its matching `.bin`; number classification uses [tiny_resnet.onnx](assets/tiny_resnet.onnx). The detection model expects NCHW `1×3×480×640` input and `1×14×6300` output. The classifier must return nine finite class scores.

Images retain their aspect ratio when resized, with black padding on the right or bottom to reach `640×480`. Each frame computes its effective ROI from the original configuration and current image size. Both corners and boxes are restored to source-image coordinates. Each detector instance reuses its inference request and input buffer and belongs to a single thread.

The original `416×416` IR is retained for provenance and conversion and is rejected by the current detector. Regenerating the default model overwrites the `640×480` files and runs a CPU zero-input check:

```bash
.venv-openvino/bin/python tools/convert_yolov8_640x480.py
```

## Development checks

The [module notes](app/auto_aim/README.md) document interfaces, coordinates, physical assumptions and thread ownership. The [2026-11-05 refactor record](docs/refactor_2026-11-05.md) explains changes and validation limits; the [review from that date](docs/review_2026-11-05.md) describes the preceding baseline.

Project-owned C++ follows `.clang-format`, checked with `clang-format 23.1.5`:

```bash
python3 -m venv .venv-format
.venv-format/bin/python -m pip install clang-format==23.1.5
python3 tools/check_format.py --check --clang-format .venv-format/bin/clang-format
```

Use `--fix` to apply formatting. New or revised interface comments use Chinese and explain units, coordinates, timestamps and threading. Third-party implementations are excluded from this formatting check.

Standalone serial and gimbal tests with AddressSanitizer/UBSan require neither cameras nor OpenVINO:

```bash
cmake -S tests/io -B build/io-asan -G Ninja -DCMAKE_BUILD_TYPE=Debug -DIO_SANITIZER=address
cmake --build build/io-asan -j4
ctest --test-dir build/io-asan --output-on-failure
```

## Code and model sources

The auto-aim implementation is adapted from TongjiSuperPower `sp_vision_25`. Its MIT license is preserved in [LICENSES/sp_vision_25-MIT.txt](LICENSES/sp_vision_25-MIT.txt). Model licensing is documented in the [third-party notices](LICENSES/README.md); that upstream license does not license every file in this repository.

The historical section of [项目结构与代码风格](项目结构与代码风格.md) preserves the initial framework layout and recommendations. The [v0.1 review](docs/auto_aim_v0.1_review.md) and [2026-11-02 review](docs/review_2026-11-02.md) remain records of their respective versions.
