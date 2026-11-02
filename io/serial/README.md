# 结构体串口通信

`io::Serial` 在 Linux 上提供有期限的串口收发和按结构体解析的数据订阅。需要 C++20、Boost.Asio 和项目日志模块。端口无数据时 `spin_once()` 最多等待 50 ms；发送遇到背压时，250 ms 超时会关闭当前会话。

包类型必须可直接复制、包含至少两个字节的固定帧头，并能放入解析缓冲区。协议的字节序、打包方式和 CRC 范围需要与下位机一致。

```cpp
struct __attribute__((packed)) Feedback {
    uint8_t head[2] = {'S', 'P'};
    uint32_t value;
    uint16_t crc16;
};

io::Serial serial;
if (!serial.open(device_path, 115200)) {
    throw std::runtime_error("cannot open serial device");
}
serial.recv<Feedback>(
    [](const Feedback& packet) {
        // 使用已通过校验的 packet.value。
    },
    [](const Feedback& packet) {
        return tools::check_crc16(
            reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    });
while (serial.is_open()) serial.spin_once();
```

`recv<T>(callback, validate)` 的第二个参数是消费整帧前的校验器。返回 `false` 时只丢弃一个字节并继续查找帧头，保留假帧头或坏 CRC 后面的合法包。只在 `callback` 中校验 CRC 已经太晚：解析器此前可能消费了与合法包重叠的假候选。

不带第二个参数的 `recv<T>(callback)` 保留无校验协议的行为，不自动猜测 CRC 算法。云台和串口演示入口都显式提供 CRC16 校验器。每个订阅有独立解析器；端口重开会清空半包，但保留订阅和校验器。用户回调可发送或关闭串口，不能在回调里递归调用 `spin_once()`。

发送前由调用者填写帧头、内容和 CRC，再调用 `send(packet)`。驱动不自动修改包。返回值是实际写入的字节数，应与包大小比较；超时、断开或关闭可能产生部分写入。

独立伪终端测试及 Sanitizer 命令见 [测试说明](../../tests/io/README.md)。
