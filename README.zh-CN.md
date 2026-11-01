# TGU Robocore 2027

面向联盟赛机器人装甲板的实验性自瞄程序，基于 Robocore 框架，使用 YOLOv8、整车 EKF 和 TinyMPC 完成检测、跟踪与云台控制。实车标定和闭环验证仍需在实际设备上完成。

[简体中文](README.md) | [English](README.en.md)

当前分支为 `auto-aim-v0.1`，支持英雄、工程、步兵和哨兵，不处理前哨站、基地和能量机关。

```text
USB 相机 → YOLOv8 → 数字分类 → PnP / 装甲板朝向角优化
         → 整车 EKF → Planner 目标轨迹 → TinyMPC → 云台指令
```

## 编译与自检

以下命令在本分支的仓库根目录执行。需要 Ubuntu 24.04 或能提供 GCC 13+ 的 Linux 环境，编译器须支持 C++20 的 `std::format`。

1. 安装系统依赖，包括 CMake 3.16+、OpenCV、Eigen3、Boost.System 和 Aravis 0.8：

   ```bash
   sudo apt update
   sudo apt install -y build-essential cmake ninja-build pkg-config g++ \
     python3-venv python3-pip libboost-all-dev libopencv-dev \
     libeigen3-dev libaravis-dev aravis-tools aravis-tools-cli libusb-1.0-0-dev
   ```

2. 安装仓库固定版本的 OpenVINO：

   ```bash
   python3 -m venv .venv-openvino
   .venv-openvino/bin/python -m pip install -r requirements-openvino.txt
   ```

   CMake 会从这个虚拟环境查找 OpenVINO。使用系统安装时，在下面的配置命令中添加 `-DOpenVINO_DIR=软件包的cmake目录`。

3. 编译并运行不依赖相机、串口的 MPC 数值自检：

   ```bash
   cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
   cmake --build build -j
   ./build/planner_test --config-path=config/auto_aim.toml
   ```

   自检成功时返回 0，并输出角度、角速度和角加速度。它只检查能否生成有限的控制量，不代表检测、跟踪或实车闭环已经通过验证。

## 实车运行

先修改 [config/auto_aim.toml](config/auto_aim.toml)，替换相机内参、畸变和相机到云台外参，并核对串口协议。仓库中的标定值仅供迁移参考。

```bash
./build/auto_aim_sentry config/auto_aim.toml
```

程序会先打开云台串口，再打开相机；缺少设备或串口权限时无法完成启动。常用配置如下：

| 配置项 | 默认值 | 用途 |
|---|---|---|
| `camera.source` | `0` | 相机索引，也可填写设备路径、视频文件或流地址 |
| `camera.width` / `camera.height` | `0` / `0` | 使用相机默认采集分辨率 |
| `camera.fps` | `200.0` | 请求帧率；后端限制在 1–200 FPS，非正值按 200 处理 |
| `tracker.enemy_color` | `red` | 敌方颜色，运行前按己方阵营检查 |
| `tracker.priority_mode` | `1` | 模式 1 优先 3/4 号，模式 2 优先 2 号 |
| `gimbal.com_port` | `/dev/ttyUSB0` | 云台串口设备 |
| `planner.auto_fire` | `false` | 自动开火开关 |

自动开火保持关闭，直到串口协议、坐标方向、标定和 MPC 输出都已在测试台上确认。正式入口使用 Planner/TinyMPC；`Aimer`、`Shooter` 保留在源码中，当前入口不调用它们。

## 离线回放

离线回放需要同名前缀的 `.avi` 视频和 `.txt` 位姿记录，仓库没有附带录包。将 `REPLAY_PREFIX` 设置为自己的录包前缀，运行前确认两个文件都存在：

```bash
: "${REPLAY_PREFIX:?请先设置录包路径前缀，不含扩展名}"
test -f "${REPLAY_PREFIX}.avi" && test -f "${REPLAY_PREFIX}.txt" && \
  ./build/auto_aim_test --config-path=config/auto_aim.toml "$REPLAY_PREFIX"
```

## 检测模型

默认模型为 [assets/yolov8_640x480.xml](assets/yolov8_640x480.xml) 及其同名 `.bin` 权重。检测器启动时校验输入 `640 × 480` 和输出 `1 × 14 × 6300`，目前只支持 YOLOv8。

相机采集分辨率可以与模型输入不同：图像按比例缩放后，在右侧或下方补黑到 `640 × 480`。原始 `416 × 416` IR 仅用于追溯和转换，不能直接交给当前检测器。需要重新转换时执行：

```bash
.venv-openvino/bin/python tools/convert_yolov8_640x480.py
```

转换会覆盖默认的 `640 × 480` 模型，并用 CPU 做一次零输入推理检查。

## 海康相机 USB 权限

原框架的海康相机测试可能需要以下 udev 规则：

```bash
sudo tee /etc/udev/rules.d/99-hikrobot.rules <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="2bdf", ATTRS{idProduct}=="0001", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 代码与模型来源

自瞄实现改编自 TongjiSuperPower 的 `sp_vision_25`，原 MIT 许可证保留在 [LICENSES/sp_vision_25-MIT.txt](LICENSES/sp_vision_25-MIT.txt)。模型另有许可信息，见 [第三方说明](LICENSES/README.md)；该 MIT 许可证不代表仓库所有内容均采用 MIT。

模块划分、坐标和单位约定见 [自瞄模块说明](app/auto_aim/README.md)。实机验证缺口和后续计划见 [v0.1 评审](docs/auto_aim_v0.1_review.md)。
