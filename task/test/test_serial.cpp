#include <iostream>

#include "io/serial/serial.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"

struct __attribute__((packed)) RecvPackage {
    uint8_t head[2] = {'T', 'G'};
    uint32_t data;
    uint16_t crc16;
};

struct __attribute__((packed)) SendPackage {
    uint8_t head[2] = {'T', 'G'};
    uint32_t data = 0;
    uint16_t crc16;
} sendpkg;

int main() {
    tools::LoggerConfig cfg{.level = tools::LogLevel::Debug,
                            .enable_console = true,
                            .enable_file = false,
                            .file_path = "logs.txt"};
    tools::Logger::instance().init(cfg);
    static constexpr const char* MODULE = "MAIN";

    io::Serial serial;

    serial.open("/dev/ttyACM0", 2000000);

    serial.recv<RecvPackage>([](const auto& pkt) { LOG_INFO(MODULE, "{}", pkt.data); },
                             [](const RecvPackage& pkt) {
                                 return tools::check_crc16(reinterpret_cast<const uint8_t*>(&pkt),
                                                           sizeof(pkt));
                             });

    while (true) {
        serial.spin_once();
        sendpkg.data++;
        sendpkg.crc16 =
            tools::get_crc16(reinterpret_cast<const uint8_t*>(&sendpkg), sizeof(sendpkg) - 2);
        serial.send(sendpkg);
    }
}
