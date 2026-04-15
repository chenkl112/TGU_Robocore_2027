//
// Created by Interweave on 2026/4/14.
//

#include "tools/foxglove_comm.hpp"
#include <iostream>

int main() {
    tools::FoxGloveComm comm("0.0.0.0", 8765);

    if (!comm.ok()) {
        std::cerr << "foxglove init failed\n";
        return -1;
    }

    std::cout << "foxglove ready at " << comm.host() << ":" << comm.port() << "\n";
    return 0;
}
