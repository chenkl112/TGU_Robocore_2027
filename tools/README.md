# ROBOCORE::TOOLS

一个用于Robocore框架的工具库，目前有以下工具
- CRC校验
- Logger日志
- toml++

## 简介与使用说明

### 1、crc

### 2、logger
一个轻量级、无第三方依赖的 C++ 日志系统，支持：

- 编译期裁剪（Release 下移除 DEBUG 日志）
- 运行时日志等级控制
- 控制台 + 文件输出
- 多线程安全
- 模块化日志
---
详细使用示例见 `task/test/test_logger.cpp`，此处仅做简单介绍：

#### 1. 日志系统的初始化

```cpp
tools::LoggerConfig cfg{
    .level = tools::LogLevel::Debug, // 日志最低输出等级（低于该等级将被过滤）
    .enable_console = true,          // 是否输出到控制台
    .enable_file = false,            // 是否输出到文件
    .file_path = "logs.txt"          // 日志文件路径（enable_file=true 时生效）
};

tools::Logger::instance().init(cfg);
```

#### 2. 日志宏的使用

日志通过宏调用，统一格式如下：

```cpp
LOG_INFO(module, fmt, ...);
```
- module：模块名称（用于标识日志来源，如 serial / vision / planner 等）
- fmt：格式化字符串（使用 {} 作为占位符）
- ...：可选参数，对应 {} 依次填充

#### 3. 模块名称推荐写法

建议为每个文件定义模块名：
```c++
static constexpr const char* MODULE = "TEST";

LOG_INFO(MODULE, "init success");
LOG_WARN(MODULE, "value={}", 42);
```

> [!NOTE]
>
> - {} 的数量需与参数数量一致
> - LOG_DEBUG 在 Release 模式下会被编译期移除（不会执行）
> - 格式化依赖 std::format（需 C++20 编译支持）

### 3、tomlpp