#include "regression_fixtures.hpp"
#include "tools/logger.hpp"

#include <iostream>
#include <string>

using namespace auto_aim_test;

bool run_planner_regressions(const std::string&, const std::string&, Clock::time_point);
bool run_tracker_regressions(const std::string&, const std::string&, Clock::time_point);
bool run_geometry_runtime_regressions(const std::string&, const std::string&, Clock::time_point);
bool run_time_contract_regressions(const std::string&, const std::string&, Clock::time_point);

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 3, "Expected scenario and config path");
        const std::string scenario = argv[1], config = argv[2];
        const auto now = Clock::now();
        if (!run_planner_regressions(scenario, config, now) &&
            !run_tracker_regressions(scenario, config, now) &&
            !run_geometry_runtime_regressions(scenario, config, now) &&
            !run_time_contract_regressions(scenario, config, now)) {
            throw std::runtime_error("Unknown scenario");
        }
        std::cout << "PASS " << scenario << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
