# 2026-11-05 提交前检查

检查对象为 `auto-aim-v0.1` 上以 `e454a18` 为基线的自瞄重构、测试、文档和 CI 改动。目标是现有公开仓库的同名分支，保持现有可见性。提交前源码摘要与 [最终验证记录](refactor_2026-11-05_validation.txt) 一致，已有 Release 83/83、核心 Debug 39/39、I/O ASan＋UBSan 30/30 通过的证据；本次检查未重复运行这些测试。

## 格式闸

| 检查项 | 结果 | 证据 |
|---|---|---|
| 用途、受众和成熟度 | PASS | README.md:3、README.en.md:3 明确实验性自瞄程序及实机验证边界 |
| 构建和运行入口 | PASS | README.md:28、README.md:46、README.md:65；CMakeLists.txt:4、task/CMakeLists.txt:30 对应现有选项和程序 |
| 配置和命令真实 | PASS | README.md:84、README.md:104；config/auto_aim.toml:12、task/test/auto_aim_test.cpp:32；引用的模型、脚本和配置均存在 |
| 中英文同步 | PASS | README.md、README.en.md 的 9 个 Bash/TOML 代码块一致，仅录包错误提示翻译不同；README.zh-CN.md:3 指向主中文文档 |
| 文档链接及归属 | PASS | README.md:150、README.en.md:150、LICENSES/README.md:3 保留上游归属，不声明全仓 MIT；所查 38 个相对链接目标存在 |
| 验证表述与代码格式 | PASS | docs/refactor_2026-11-05.md:64 记录最终测试配置与限制；96 个自有 C++ 文件通过固定版本格式检查，git diff --check 通过 |

## 耦合闸

| 检查项 | 结果 | 证据 |
|---|---|---|
| 本机路径和私有服务 | PASS | 全部待提交文本未命中用户目录绝对路径、file/vscode URI、私有文档域名或内网 IP；app/auto_aim/config.hpp:128–129 明确资源路径解析契约 |
| 示例与测试路径 | PASS | tests/auto_aim/config_tests.cpp 使用临时目录和合成路径；README.md:77 通过 paths.base_dir 描述可迁移的资源定位方式 |
| 项目名和上游归属 | PASS | .publish-allow:7–10 明确保留公开项目名、头文件保护宏和上游版权引用的理由 |
| 分支和仓库 | PASS | origin 对应用户指定的公开仓库；当前分支及上游均为 auto-aim-v0.1；不会改写其他分支或改变仓库可见性 |

## 隐私闸

| 检查项 | 结果 | 证据 |
|---|---|---|
| 当前文本中的凭据和个人数据 | PASS | 提交前全仓文本扫描未发现私钥、访问令牌、含密码连接串、手机号或内网地址；只命中以下三处已登记版权邮箱 |
| 第三方版权联系方式 | PASS | tools/foxglove/include/foxglove/expected.hpp:3、tools/tomlpp.hpp:28、tools/tomlpp.hpp:9084；由 .publish-allow:11–13 逐项说明，保留原版权通知 |
| 本地可达历史中的凭据 | PASS | 对 41 个可达提交的补丁扫描私钥、GitHub/API/AWS/Slack 令牌及含密码连接串，未命中；扫描输出仅记录位置与类别 |
| 新增文件和二进制边界 | PASS | 未发现待提交的 .env、日志、数据库或私钥文件；无新增二进制文件，原有模型和 Foxglove SDK 无改动；二进制内容不属于文本密钥扫描结论 |

可上传。
