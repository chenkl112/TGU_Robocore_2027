# P1 修复分支提交闸门

审查对象为合入 `auto-aim-v0.1` 的 P1 修复，基准为 `9e0f1aa`。按 `commit-gate` 技能检查 README、环境耦合和隐私，检查范围包括本次新增文件、全仓已跟踪文本以及本地可达提交历史中的凭据。

目标是已有的公开仓库 `chenkl112/TGU_Robocore_2027`，按用户要求将已审查的修复快进合入并推送 `auto-aim-v0.1`，核对远端后删除临时修复分支，保留现有可见性。技能中新建私有仓库的流程不适用于这次已有公开仓库的分支推送。

## 格式闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 首屏说明用途、目标读者和实验状态 | PASS | README.md:1、README.md:3；README.en.md:3 |
| 安装、构建、自检步骤齐全 | PASS | README.md:18、README.md:27、README.md:36 |
| 快速开始复验 | PASS | README.md:39；本轮从空构建目录配置 Ninja Release，完整构建成功，CTest 42/42 通过，planner_test 返回 0 |
| 中英文步骤一致 | PASS | README.md:39、README.zh-CN.md:39、README.en.md:39；三份均包含构建、CTest 和 MPC 自检 |
| 相对链接与代码块完整 | PASS | tests/io/README.md:63 已改为现存审查报告；README、docs、测试说明的相对链接及代码块检查通过 |
| 验证声明有证据 | PASS | tests/infra/README.md:52 已移除本轮无法证实的旧 TSan 结果，仅保留实际执行的 Debug/Release 结果 |
| 硬件及录包前提写明 | PASS | README.md:49、README.md:55、README.md:71；docs/review_2026-11-02.md:55 |

## 耦合闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 本机用户目录、磁盘路径、file URI | PASS | 对全仓已跟踪文本逐行扫描，无未批准命中；CMakeLists.txt:22 使用项目目录查找可选依赖 |
| 构建依赖没有绑定本机临时目录 | PASS | CMakeLists.txt:16、CMakeLists.txt:20、README.md:34；本次验证用的临时依赖目录没有写入仓库 |
| 项目及上游标识保留有理由 | PASS | .publish-allow:7、.publish-allow:8、.publish-allow:9、.publish-allow:10 |
| 私有文档入口和内部服务链接 | PASS | 全仓已跟踪文本扫描无未批准命中；新增报告和测试说明使用仓内相对链接 |
| 新增文件范围 | PASS | 仅源代码、配置、测试和文档；build 由 .gitignore:7 排除，没有新增日志、环境文件、数据库或二进制 |

## 隐私闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 当前文本中的凭据、私钥和带密码连接串 | PASS | 对全仓已跟踪文本逐行扫描，无命中；扫描输出仅记录位置和类型 |
| 本地可达历史中的凭据 | PASS | 对 git log --all -p 的新增行扫描，无私钥或凭据命中 |
| 内网地址、联系方式及身份号码 | PASS | 当前文本没有未批准命中；测试运行服务使用回环地址，见 tests/auto_aim/runtime_tests.py:48 |
| 第三方版权邮箱 | PASS | 命中 tools/foxglove/include/foxglove/expected.hpp:3、tools/tomlpp.hpp:28、tools/tomlpp.hpp:9084；分别由 .publish-allow:11、.publish-allow:12、.publish-allow:13 说明，保留版权通知 |
| 二进制扫描边界 | PASS | .publish-allow:15；模型和 Foxglove 库是基线已有文件，本次未新增或替换，未将二进制文本扫描当作其内容审计 |

可上传。三道闸均通过，无未处理阻断项。提交信息使用 `fix(auto_aim): 修复全量审查发现的P1问题`。推送后需核对远端分支提交与本地 HEAD 一致，并回读仓库可见性。

## 二次审查补修的提交闸门

本轮基于 `auto-aim-v0.1` 的 `30e62b8`，提交用户指出的 2 项 P1 和 3 项 P2 修复，目标分支仍为 `auto-aim-v0.1`。

### 格式闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 行为与回归结果同步 | PASS | docs/review_2026-11-02.md:59、docs/review_2026-11-02.md:69；Release 49/49、串口/云台 ASan+UBSan 21/21 通过 |
| 接口说明与实现一致 | PASS | io/serial/README.md:29 说明消费前校验；io/serial/serial.hpp:89 在验证失败时只消费一个字节；app/auto_aim/README.md:46 说明缺失反馈时禁射 |
| 文档链接、代码块和空白 | PASS | README、docs、模块及测试说明的相对链接和代码块检查通过；git diff --check 无错误 |

### 耦合闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 本机路径和私有服务 | PASS | 全仓已跟踪文本复扫无未批准命中；新增用例以临时目录生成标定，测试结束清理，见 tests/auto_aim/auto_aim_regression_tests.cpp |
| 依赖与生成物 | PASS | tests/auto_aim/CMakeLists.txt:29、tests/io/CMakeLists.txt:51 注册新场景；.gitignore:7 排除构建产物，本轮没有新增二进制、日志或环境配置 |

### 隐私闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 当前文本及历史凭据 | PASS | 对全仓文本及本地可达历史新增行复扫，未发现凭据、私钥或带密码连接串 |
| 公开版权联系方式 | PASS | 仍仅命中 .publish-allow:11、.publish-allow:12、.publish-allow:13 已批准的 3 处第三方版权邮箱；没有新增例外 |

可上传。三道闸通过，提交信息使用 `fix(auto_aim): 补齐发弹校验与截击观测同步`。推送后核对远端提交与本地 HEAD 一致。

## 强透视角点修复的提交闸门

本轮基于 `auto-aim-v0.1` 的 `43de5cd`，目标仍为该分支。模型和二进制权重没有修改。

### 格式闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 模型约定与修复行为可核对 | PASS | app/auto_aim/yolos/keypoints.hpp:11 说明原始通道及 PnP 点序；docs/review_2026-11-02.md:80 说明替代短边排序的原因 |
| 回归证据与表述一致 | PASS | docs/review_2026-11-02.md:84、docs/review_2026-11-02.md:88；Release 构建成功、CTest 51/51 通过 |
| 链接、代码块和空白 | PASS | README、docs 和模块说明的相对链接及代码块检查通过；git diff --check 无错误 |

### 耦合闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 本机路径和私有服务 | PASS | 全仓已跟踪文本复扫无未批准命中；模型路径由 tests/auto_aim/CMakeLists.txt:44 传入 |
| 新增文件范围 | PASS | 仅新增模型解码头文件和模型通道测试源码；构建产物由 .gitignore:7 排除，未新增二进制、日志或环境文件 |

### 隐私闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 当前文本与历史凭据 | PASS | 全仓已跟踪文本与本地可达历史新增行复扫，无凭据、私钥或带密码连接串 |
| 第三方版权例外 | PASS | 仍仅命中 .publish-allow:11、.publish-allow:12、.publish-allow:13 已批准的 3 处邮箱，无新增例外 |

可上传。三道闸通过，提交信息使用 `fix(auto_aim): 按模型语义保留强透视角点对应`。推送后核对远端提交和仓库可见性。

## 余留风险补修的提交闸门

本轮基于 `auto-aim-v0.1` 的 `71f5853`，补修图中指出的 Shooter pitch 对准和 Gimbal 自动重连；目标仍为 `auto-aim-v0.1`。

### 格式闸

| 项目 | 结果 | 证据 |
|---|---|---|
| README 配置说明同步 | PASS | README.md、README.zh-CN.md、README.en.md 的 gimbal 表格包含 `reconnect_delay_ms` |
| 行为说明与实现一致 | PASS | app/auto_aim/README.md 说明 Shooter yaw/pitch 和重连行为；docs/review_2026-11-02.md 更新余留风险状态 |
| 验证声明有证据 | PASS | Release CTest 53/53、串口/云台 ASan+UBSan 22/22；gimbal.reconnect 重复 5 次通过 |

### 耦合闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 本机路径和临时文件 | PASS | 新增测试使用临时目录和伪终端，测试退出清理；无本机绝对路径进入提交 |
| 构建产物和日志 | PASS | `.gitignore:7` 排除 build；提交前只暂存源代码、配置、文档和测试 |

### 隐私闸

| 项目 | 结果 | 证据 |
|---|---|---|
| 凭据、私钥、内部地址 | PASS | 全仓文本和本地可达历史复扫无未批准命中 |
| 第三方版权邮箱 | PASS | 仅保留 `.publish-allow:11`、`.publish-allow:12`、`.publish-allow:13` 的既有版权例外 |

可上传。三道闸通过，提交信息使用 `fix(auto_aim): 补齐发弹校验与串口自动重连`。
