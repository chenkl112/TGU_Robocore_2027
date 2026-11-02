# P1 修复分支提交闸门

审查对象为 `fix/full-review-p1`，基于 `auto-aim-v0.1` 的 `9e0f1aa`。按 `commit-gate` 技能检查 README、环境耦合和隐私，检查范围包括本次新增文件、全仓已跟踪文本以及本地可达提交历史中的凭据。

目标是已有的公开仓库 `chenkl112/TGU_Robocore_2027`，只发布修复分支，保留现有可见性。技能中新建私有仓库的流程不适用于这次已有公开仓库的分支推送。

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
