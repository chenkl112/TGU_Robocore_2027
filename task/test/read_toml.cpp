//
// Created by Interweave on 2026/4/09.
//

#include <iomanip>
#include <iostream>

#include "tools/tomlpp.hpp"

int main() {
    std::cout << std::setprecision(17);
    auto config = toml::parse_file("../../config/testconfig.toml");

    std::string title = config["title"].value_or("default");

    double exposure_ms = config["camera"]["exposure_ms"].value_or(0);
    double gain = config["camera"]["gain"].value_or(0.0);

    std::cout << title << std::endl;
    std::cout << "exposure_ms: " << exposure_ms << std::endl;
    std::cout << "gain: " << gain << std::endl;

    auto arr1 = config["camera"]["camera_matrix"].as_array();
    double K[3][3];
    for (int i = 0; i < 9; i++) {
        K[i / 3][i % 3] = (*arr1)[i].value_or(0.0);
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << K[i][j] << " ";
        }
        std::cout << std::endl;
    }

    auto arr = config["camera"]["distort_coeffs"].as_array();

    for (size_t i = 0; i < 5; i++) {
        double val = (*arr)[i].value_or(0.0);
        std::cout << val << std::endl;
    }

    return 0;
}
