# TGU Robocore 2027

面向联盟赛机器人装甲板的实验性自瞄程序，基于 Robocore 框架，使用 YOLOv8、整车 EKF 和 TinyMPC 完成检测、跟踪与云台控制。当前 `auto-aim-v0.1` 支持英雄、工程、步兵和哨兵，不处理前哨站、基地和能量机关。实车标定和闭环效果仍需在实际设备上验证。

[简体中文](README.md) | [English](README.en.md)

```text
图像 → YOLOv8 → 数字分类 → 尺寸判定 / PnP / 朝向优化
     → 整车 EKF → 目标快照 → 弹道拦截与 TinyMPC → 云台指令
```

实时程序和离线回放共用跟踪、规划实现。离线回放按录包时间逐帧执行，可以不打开窗口，并输出用于比较的 CSV。

## 编译与自检

以下命令在仓库根目录执行。需要支持 C++20 `std::format` 的编译器，例如 Ubuntu 24.04 的 GCC 13；下面的 CTest 命令需要 CMake 3.20+。

先安装算法测试所需依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build libeigen3-dev libopencv-dev
```

只改几何、跟踪或规划时，可以使用纯算法构建。这个配置不查找 OpenVINO、Boost 或 Aravis，也不启动设备：

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

检测、回放和正式入口还需要 Boost 头文件及仓库固定版本的 OpenVINO：

```bash
sudo apt install -y libboost-dev python3-venv python3-pip
python3 -m venv .venv-openvino
.venv-openvino/bin/python -m pip install -r requirements-openvino.txt

cmake -S . -B build/full -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/full -j4
ctest --test-dir build/full --output-on-failure -j4
```

CMake 优先使用显式传入的 `OpenVINO_DIR`，否则查找 `.venv-openvino` 中的 CMake 包。内存较紧张时将构建并行数降为 `-j2`。

| 开关 | 默认值 | 控制的内容 |
|---|---|---|
| `ROBOCORE_WITH_VISION` | `ON` | OpenVINO 检测、同步回放及模型测试 |
| `ROBOCORE_WITH_IO` | `ON` | 串口、云台和 USB 相机；正式入口需要同时开启视觉和设备 |
| `ROBOCORE_WITH_FOXGLOVE` | `ON` | Foxglove SDK 适配器；当前自瞄入口未发布 Foxglove 数据 |
| `ROBOCORE_BUILD_DEMOS` | `ON` | `planner_test` 等诊断程序 |
| `ROBOCORE_BUILD_CAMERA_DEMO` | `OFF` | 单独的 Aravis 相机演示 |

Aravis 仅供相机演示使用。需要该程序时安装 `pkg-config libaravis-dev`，重新配置时加 `-DROBOCORE_BUILD_CAMERA_DEMO=ON`。生产库和测试共用 [CMake 目标定义](cmake/RobocoreTargets.cmake)，不再分别维护两套算法源文件清单。

## 配置和实车运行

运行前修改 [config/auto_aim.toml](config/auto_aim.toml)，用实际相机与云台组合的内参、畸变和外参替换参考值，并核对下位机协议。

```bash
./build/full/auto_aim_sentry config/auto_aim.toml
```

入口通过 `load_auto_aim_config()` 一次读取 TOML、检查字段并转换单位，再把各模块需要的配置结构传入构造函数。直接用配置结构构造模块也会检查参数；缺失的标定数组、错误类型和非法范围会带字段名报错。

默认资源路径相对程序启动目录解析，因此默认配置适合在仓库根目录运行。若要从其他目录启动，可以在 `config/auto_aim.toml` 增加：

```toml
[paths]
base_dir = ".."
```

`base_dir` 自身相对配置文件所在目录解析；这里指向仓库根目录。相对模型路径和相机文件路径随后以它为基准，加载后的模型路径是绝对路径。绝对路径和相机流 URL 保持原意；未设置 `base_dir` 的旧配置继续使用启动目录。

| 配置项 | 仓库默认值 | 含义 |
|---|---|---|
| `camera.source` | `0` | 相机索引，也可用设备路径、视频文件或流 URL |
| `camera.width` / `camera.height` | `0` / `0` | 使用设备默认分辨率 |
| `camera.fps` | `200.0` | 请求帧率限制在 1–200 FPS；非正值按 200 处理 |
| `tracker.enemy_color` | `red` | 待跟踪的装甲颜色 |
| `tracker.priority_mode` | `1` | 模式 1 优先 3/4 号，模式 2 优先 2 号 |
| `gimbal.com_port` | `/dev/ttyUSB0` | 云台串口 |
| `gimbal.reconnect_delay_ms` | `250` | 断开后的重连间隔 |
| `planner.max_target_age` | `0.1` | 最后一次有效观测的最长使用时间，单位秒 |
| `planner.auto_fire` | `false` | 自动开火开关 |

启动先打开串口并等待首个有效反馈；串口打开失败或 3 秒内没有有效包会退出。相机缺失时采集线程尝试重新打开；运行中的串口断连或反馈过期会使输出停控。自动开火还要求新鲜的实际云台反馈、当前会话一致、可见的拦截装甲及对齐条件成立。现有测试不替代标定、协议方向和实机闭环检查。

## 离线回放与 CSV

录包由同名前缀的 `.avi` 和 `.txt` 组成。文本每行对应一帧，格式为 `时间秒 q_w q_x q_y q_z`。时间使用录包时钟的有限非负秒数（上限 `1e9`），所处理帧必须严格递增；四元数必须有限且具有有效范数。仓库未附带本队实机录包，回归测试使用生成的图像和位姿。

将 `REPLAY_PREFIX` 设为自己的路径前缀后执行：

```bash
: "${REPLAY_PREFIX:?请设置录包路径前缀，不含扩展名}"
./build/full/auto_aim_test --config-path=config/auto_aim.toml \
  --device=CPU --headless=true --output=replay.csv "$REPLAY_PREFIX"
```

`--start-index=100 --end-index=199` 处理包含两端的帧范围；`--end-index=0` 表示读到视频末尾。视频跳帧时同步跳过位姿记录。省略 `--headless=true` 可查看图像，省略 `--output` 则不保存 CSV。

每个处理帧只执行一次检测、跟踪和规划，规划使用录包时间。CSV 包含检测数、跟踪状态、目标身份、11 维状态、状态时间、最后观测时间、控制输出和停止原因。CLI 没有实际云台反馈，因此 CSV 的 `fire` 始终为 `0`。同一环境、模型和输入可用于重复回放对比，不承诺跨推理设备的逐位相同输出。

CSV 先写入输出目录中的临时文件，正常结束后原子替换目标文件。读包、模型或处理失败时返回非零并保留已有 CSV；输出也不能覆盖录包、配置或配置引用的模型文件。

## 检测模型

默认检测器使用 [yolov8_640x480.xml](assets/yolov8_640x480.xml) 及同名 `.bin`，数字分类使用 [tiny_resnet.onnx](assets/tiny_resnet.onnx)。检测模型输入为 NCHW `1×3×480×640`，输出为 `1×14×6300`；分类器输出必须是九个有限的类别分数。

采集图像按比例缩放，右侧或下方补黑到 `640×480`。ROI 每帧按当前分辨率重新裁剪，角点和检测框都会恢复到原图坐标。一个检测实例复用自己的推理请求和输入缓冲区，由单个线程使用。

原始 `416×416` IR 保留用于追溯和转换，当前检测器不会接受它。重新生成默认模型会覆盖现有 `640×480` 文件，并执行一次 CPU 零输入检查：

```bash
.venv-openvino/bin/python tools/convert_yolov8_640x480.py
```

## 开发与验证入口

模块接口、坐标、物理假设和线程所有权见 [自瞄模块说明](app/auto_aim/README.md)。本轮修改与验证边界见 [2026-11-05 重构记录](docs/refactor_2026-11-05.md)；[同日审查](docs/review_2026-11-05.md) 描述修改前的基线问题。

自有 C++ 使用仓库的 `.clang-format`，检查器要求 `clang-format 23.1.5`：

```bash
python3 -m venv .venv-format
.venv-format/bin/python -m pip install clang-format==23.1.5
python3 tools/check_format.py --check --clang-format .venv-format/bin/clang-format
```

修改后可使用 `--fix` 应用格式。新增和修改的接口注释使用中文，重点写清单位、坐标、时间和线程契约。第三方源码不进入这项全仓格式检查。

串口与云台的独立 AddressSanitizer/UBSan 测试不需要相机或 OpenVINO：

```bash
cmake -S tests/io -B build/io-asan -G Ninja -DCMAKE_BUILD_TYPE=Debug -DIO_SANITIZER=address
cmake --build build/io-asan -j4
ctest --test-dir build/io-asan --output-on-failure
```

## 代码与模型来源

自瞄实现改编自 TongjiSuperPower 的 `sp_vision_25`，原 MIT 许可证保留在 [LICENSES/sp_vision_25-MIT.txt](LICENSES/sp_vision_25-MIT.txt)。模型许可见 [第三方说明](LICENSES/README.md)；该 MIT 许可证不代表仓库所有内容均采用 MIT。

框架初期的目录和开发建议保留在 [项目结构与代码风格](项目结构与代码风格.md) 的历史部分。此前的 [v0.1 评审](docs/auto_aim_v0.1_review.md) 和 [2026-11-02 审查](docs/review_2026-11-02.md) 继续作为对应版本的记录。
