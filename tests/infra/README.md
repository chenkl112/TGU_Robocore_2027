# 通用基础设施测试

从仓库根目录运行。需要 CMake 3.16+、构建工具和支持 C++20 `std::format` 的编译器及标准库（例如 GCC 13 与 libstdc++ 13）。配置阶段会检查 `std::format`，不支持时直接报错。

这个入口只编译 `tools/logger.cpp` 和测试程序，链接标准线程库，不加载根目录的 CMake 工程，无需安装 OpenCV、OpenVINO、Eigen、串口库或设备 SDK。

```bash
cmake -S tests/infra -B build/infra-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/infra-debug
ctest --test-dir build/infra-debug --output-on-failure

cmake -S tests/infra -B build/infra-release -DCMAKE_BUILD_TYPE=Release
cmake --build build/infra-release
ctest --test-dir build/infra-release --output-on-failure
```

CMake 3.16–3.19 的 CTest 不支持 `--test-dir`，请改为进入对应构建目录后执行 `ctest --output-on-failure`。多配置生成器需在构建时指定 `--config Debug` / `Release`，运行 CTest 时指定 `-C Debug` / `Release`。

每组有 7 个测试，各自启动进程，创建独立临时目录并在正常结束或断言失败时清理。测试以内容检查和非零退出码判断失败，Release 下同样执行检查。

| 场景 | 检查内容 |
| --- | --- |
| `logger.reinitialize` | 同一路径反复初始化，前后记录完整保留 |
| `logger.switch_file` | A 切到 B 后，后续记录只写入 B |
| `logger.toggle_file` | 禁用时关闭并刷新旧文件，禁用期间无新记录，重新启用后追加 |
| `logger.invalid_path` | 无效路径向标准错误报告路径与原因，控制台仍可用，改为有效路径后恢复 |
| `logger.levels` | Debug 至 Off 的全部阈值、文件与控制台过滤、等级标记及 stdout/stderr 分流 |
| `logger.concurrent` | 6 个写线程与 1 个初始化线程按轮次同时开始，150 轮共 2700 条记录，每条恰好一次且格式完整 |
| `logger.debug_macro` | Debug 输出 DEBUG；Release 不输出 DEBUG，也不求值其参数；其余日志宏继续输出 |

文件内容检查前通过禁用文件输出关闭流，不要求 Logger 每条日志立即刷新。现有 `task/test/test_logger.cpp` 仍是演示程序。

支持 ThreadSanitizer 的环境可额外运行并发用例：

```bash
cmake -S tests/infra -B build/infra-tsan \
  -DCMAKE_BUILD_TYPE=Debug -DINFRA_ENABLE_TSAN=ON
cmake --build build/infra-tsan
ctest --test-dir build/infra-tsan -R '^logger\.concurrent$' --output-on-failure
```

ThreadSanitizer 需要编译器运行库及兼容的运行环境；编译成功不代表检查已执行。若启动时报 `unexpected memory mapping` 等运行库错误，应记录为未完成动态数据竞争检查，普通并发用例通过也不能替代这一检查。

若系统允许 `setarch`，可尝试仅为测试进程关闭地址随机化后重试，不修改系统全局设置：

```bash
setarch x86_64 -R ctest --test-dir build/infra-tsan -R '^logger\.concurrent$' --output-on-failure
```

本轮基于 `9e0f1aa`，修复日志等级读取与初始化之间的锁保护问题，以及重复初始化、切换路径、禁用输出、打开失败后的文件流生命周期问题。保留 `task / app / io / tools` 分层、Logger 接口、日志格式及 Release 的 DEBUG 宏行为。验证范围仅限日志基础设施，不覆盖自动选靶、瞄准预测、发射控制，也不表示此前全部 P1/P2 已解决。

2026-11-02 在 Linux x86_64、GCC 13.3.0、CMake 3.28.3 上验证：独立 Debug 入口和顶层 Release 入口各 7/7 通过。本轮没有执行 ThreadSanitizer，普通并发回归不能替代动态数据竞争检查。
