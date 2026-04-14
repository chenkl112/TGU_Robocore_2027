# io::Serial —— 结构体驱动串口通信库
一个**轻量级、非异步**的 C++ 串口通信库，基于 `boost::asio` 实现，支持：
- ✔ 结构体直接收发
- ✔ 自动帧头同步（来自结构体）
- ✔ 环形缓冲区（防粘包 / 半包）
- ✔ 回调机制（事件驱动）
- ✔ 适用于高速连续流（1000Hz+）

## 特性
### 1、完全结构体驱动
用户只需要定义结构体：
```cpp
struct __attribute__((packed)) RecvPackage {
    uint8_t head[2] = {'A', 'B'};
    uint32_t data;
    uint16_t crc16;
};
```
无需额外协议配置，驱动会自动从结构体读取head自动完成同步与校验。当然用户也可以再从外部回调中加入CRC校验逻辑进行校验

### 2、自动防粘包
内部实现：
环形缓冲区（RingBuffer） 流式解析（逐字节） 自动丢弃错误数据

## 依赖
- C++17
- Boost.Asio
```bash
sudo apt install libboost-all-dev 
```
