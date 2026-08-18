# TGU Robocore 2027

[English](README.md) | [简体中文](README.zh-CN.md)

## 装甲板自瞄 v0.1

本分支提供首个适配 Robocore 框架的联盟赛装甲板自瞄基线。当前版本只检测和跟踪机器人装甲板，不包含前哨站、基地和能量机关目标。

装甲板自瞄处理流程：

```text
USB 相机 -> YOLOv8 -> 装甲板数字分类 -> PnP/装甲板朝向角优化
         -> 目标跟踪/EKF -> 弹道解算/MPC 规划 -> 云台控制指令
```

本实现改编自 TongjiSuperPower 的 `sp_vision_25`。原项目的 MIT 许可证保留在 `LICENSES/sp_vision_25-MIT.txt`。

### v0.1 功能范围

- 使用 OpenVINO 运行 YOLOv8 装甲板检测。
- 支持英雄、工程、步兵和哨兵机器人目标。
- 支持 PnP 位姿解算和装甲板朝向角优化。
- 支持整车 EKF 跟踪和 MPC 控制指令规划。
- 保留 Robocore 原有的串口、日志、TOML 和 Foxglove 基础设施。
- 不包含前哨站瞄准和能量机关算法。
- 自动开火默认关闭；启用后还会根据实测云台 yaw/pitch 误差进行二次限制。

### 环境依赖

- Ubuntu 24.04，或其他提供 GCC 13 及以上版本的发行版
- CMake 3.16 及以上版本
- 支持 `std::format` 的 C++20 编译器，推荐 GCC 13 及以上版本
- Boost.System
- OpenCV
- Eigen3
- OpenVINO Runtime 开发包
- Aravis 0.8

在 Ubuntu 上安装系统依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build g++ \
  python3-venv python3-pip libboost-all-dev libopencv-dev \
  libeigen3-dev libaravis-dev aravis-tools aravis-tools-cli \
  libusb-1.0-0-dev
```

在项目目录中创建独立的 Python 虚拟环境，并安装固定版本的 OpenVINO。OpenVINO 2026.3 wheel 与 Ubuntu 24.04 的 OpenCV 软件包使用相同的 libstdc++ ABI，同时可以加载仓库中由 OpenVINO 2024 生成的 IR 模型。

```bash
python3 -m venv .venv-openvino
.venv-openvino/bin/python -m pip install -r requirements-openvino.txt
```

CMake 会自动发现上述项目本地安装。如果需要使用系统中的 OpenVINO，可以手动指定其 CMake 软件包目录：

```bash
cmake -S . -B build -DOpenVINO_DIR=/path/to/openvino/cmake
```

### 编译

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### 运行

运行前请先检查 `config/auto_aim.toml`。仓库中的标定参数仅作为迁移参考，必须替换为实际相机和云台组合的标定结果。

启动实车程序：

```bash
./build/auto_aim_sentry config/auto_aim.toml
```

运行离线回放：

```bash
./build/auto_aim_test --config-path=config/auto_aim.toml assets/demo/demo
```

v0.1 仓库中未包含回放视频和对应的位姿文本。

### 安全事项与当前限制

- `[fire].enabled` 默认为 `false`。只有在测试台上确认串口协议、坐标系约定和标定参数正确后才能启用。
- 相机来源、分辨率和帧率在 `config/auto_aim.toml` 的 `[camera]` 配置段中设置。
- v0.1 不包含 YOLOv5 和 YOLO11，唯一可选的检测后端是 YOLOv8。
- 切换到能量机关模式不会启动能量机关检测器，程序只会发送禁用控制的指令。

模块边界和验证要求见 `app/auto_aim/README.md`；已知风险、代码评审和后续规划见 `docs/auto_aim_v0.1_review.md`。

### 海康相机 USB 权限规则

运行原框架中的海康相机测试时，可能需要配置以下 udev 规则：

```bash
sudo tee /etc/udev/rules.d/99-hikrobot.rules <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", ATTRS{idProduct}=="0001", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```
