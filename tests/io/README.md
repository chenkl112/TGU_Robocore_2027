# 串口与设备生命周期测试

为维护者验证串口静默、断线、并发关闭和重开行为的独立测试入口。Linux x86_64 上的 Debug / Release 回归已通过；项目仍处于实验阶段，实际设备和驱动需要另行验证。

需要 CMake 3.20+、支持 `std::format` 的 C++20 编译器（例如 GCC 13）、Boost 1.69+ 头文件、Eigen3 和线程库。测试使用 Linux 伪终端，不需要真实串口、相机、OpenCV、OpenVINO、ROS 或 Gazebo，也不需要 root 权限。

在仓库根目录执行：

```bash
cmake -S tests/io -B build/io-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/io-debug
ctest --test-dir build/io-debug --output-on-failure

cmake -S tests/io -B build/io-release -DCMAKE_BUILD_TYPE=Release
cmake --build build/io-release
ctest --test-dir build/io-release --output-on-failure
```

自定义依赖目录可通过 `-DBoost_ROOT=...` 和 `-DEigen3_DIR=...` 指定。入口只编译串口、日志、CRC、Gimbal 和测试文件，不加载根目录工程。Gimbal 使用的时间差仍按原先的 `std::chrono::duration<double>` 计算，不再为这个运算间接依赖包含 OpenCV 的数学工具实现。

每个场景作为独立进程运行，失败返回非零；CTest 另设进程超时，防止旧版死锁使整个测试永久等待。测试自行关闭伪终端并清理临时配置。

| 场景 | 验证内容 |
| --- | --- |
| `serial.read_timeout`、`serial.close_read` | 无数据时有界返回，另一个线程关闭后可以结束接收 |
| `serial.fragmented`、`serial.subscriptions` | 半包、多包和同类型多个回调都能正确组包 |
| `serial.isolated_ports`、`serial.reopen` | 两个端口不能合成一个包，重开后旧半包不残留且订阅保留 |
| `serial.failed_open` | 端口配置失败无描述符泄漏，随后可以重新打开并发送 |
| `serial.disconnect` | 对端关闭后识别断线，继续调用接收时不会忙循环 |
| `serial.write_timeout`、`serial.close_write` | 对端不读取造成背压时，超时或主动关闭均能结束发送 |
| `serial.queued_write` | 旧会话排队中的发送不能进入新打开的端口 |
| `serial.callback_close`、`serial.callback_throw` | 回调中发送/关闭不死锁，回调异常关闭当前会话 |
| `serial.concurrent` | 收发、查询、关闭、重开和注册回调交错执行 |
| `gimbal.silent_start` | 对端保持打开且静默时，3 秒首包超时包含线程清理后仍能返回 |
| `gimbal.idle_shutdown` | 收到有效的空闲反馈后，对端静默，析构可以结束接收线程 |

## Sanitizer

ASan/UBSan 检查内存和未定义行为，检测到错误即失败：

```bash
cmake -S tests/io -B build/io-asan -DCMAKE_BUILD_TYPE=Debug -DIO_SANITIZER=address
cmake --build build/io-asan
ctest --test-dir build/io-asan --output-on-failure
```

TSan 使用独立构建目录，不能与 ASan 混用：

```bash
cmake -S tests/io -B build/io-tsan -DCMAKE_BUILD_TYPE=Debug -DIO_SANITIZER=thread
cmake --build build/io-tsan
ctest --test-dir build/io-tsan --output-on-failure
```

若 Linux x86_64 上的 GCC 运行库报告 `unexpected memory mapping`，且系统允许 `setarch`，可尝试仅对测试进程关闭地址随机化，不改系统全局配置：

```bash
setarch x86_64 -R ctest --test-dir build/io-tsan --output-on-failure
```

若环境禁止 `setarch`，应记录 TSan 未完成，不能把普通回归替代为数据竞争检查。GCC 13 对 Boost.Asio 内部 `atomic_thread_fence` 仍有检测能力警告；当前封装用互斥锁同步自身状态和端口访问，测试通过不代表对 Boost 整体做出了并发正确性证明。

修复项和验证范围见 [自瞄分支审查报告](../../docs/review_2026-11-02.md)。
