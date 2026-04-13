#include <iostream>

#include "io/serial/serial.hpp"
#include "tools/crc.hpp"

struct __attribute__((packed)) RecvPackage {
    uint8_t head[2] = {'T', 'G'};
    uint32_t data;
    uint16_t crc16;
};

struct __attribute__((packed)) SendPackage {
    uint8_t head[2] = {'T', 'G'};
    uint32_t data = 0;
    uint16_t crc16;
}sendpkg;

int main() {
    io::Serial serial;

    serial.open("/dev/ttyACM0", 2000000);

    serial.recv<RecvPackage>([](const auto& pkt){
        if (!tools::check_crc16(reinterpret_cast<const uint8_t*>(&pkt),sizeof(RecvPackage))){
            std::cerr << "CRC Error" << std::endl;
            return;
        }
        std::cout << pkt.data << std::endl;
    });

    while (true) {
        serial.spinOnce();
        sendpkg.data++;
        serial.send(sendpkg);
    }
}