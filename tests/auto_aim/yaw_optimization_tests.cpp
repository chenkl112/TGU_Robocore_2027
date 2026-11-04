#include "app/auto_aim/solver.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
using namespace app::auto_aim;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

double error(const Solver& solver, const Armor& armor, double yaw) {
    const auto projected = solver.reproject_armor(armor.xyz_in_world, yaw, armor.type);
    double result = 0;
    for (std::size_t i = 0; i < projected.size(); ++i) {
        result += cv::norm(projected[i] - armor.points[i]);
    }
    return result;
}
}

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected config path");
        Solver solver(argv[1]);
        int samples = 0;
        int improved = 0;
        double largest_yaw_error = 0;
        for (auto type : {ArmorType::small, ArmorType::big}) {
            for (double gimbal_yaw_deg : {0., 170., -170.}) {
                const double gimbal_yaw = gimbal_yaw_deg * CV_PI / 180;
                const Eigen::Quaterniond q(Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ()));
                solver.set_R_gimbal2world(q);
                for (double bearing_deg : {-12., 0., 12.}) {
                    const double bearing = bearing_deg * CV_PI / 180;
                    const Eigen::Vector3d position = q * Eigen::Vector3d(3 * std::cos(bearing),
                                                                          3 * std::sin(bearing), 0.05);
                    for (double slant_deg : {-85., -80., -45., 0., 45., 80., 85.}) {
                        const double yaw = tools::limit_rad(gimbal_yaw + bearing + slant_deg * CV_PI / 180);
                        const auto points = solver.reproject_armor(position, yaw, type);
                        for (bool noisy : {false, true}) {
                            Armor armor(1, 1.0f, {}, points);
                            // Hero big armor enters yaw optimization; balance armor deliberately does not.
                            armor.type = type;
                            armor.name = type == ArmorType::big ? ArmorName::one : ArmorName::three;
                            if (noisy) {
                                armor.points[0] += cv::Point2f(0.25f, -0.15f);
                                armor.points[1] += cv::Point2f(-0.15f, 0.20f);
                                armor.points[2] += cv::Point2f(0.10f, -0.25f);
                                armor.points[3] += cv::Point2f(-0.20f, 0.10f);
                            }
                            require(solver.solve(armor), "Valid side-view armor failed to solve");
                            const double before = error(solver, armor, armor.yaw_raw);
                            const double after = error(solver, armor, armor.ypr_in_world[0]);
                            require(std::isfinite(before) && std::isfinite(after) && after <= before + 1e-6,
                                    "Yaw optimization increased reprojection error relative to PnP");
                            if (after + 1e-5 < before) ++improved;
                            if (!noisy) {
                                const double yaw_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - yaw));
                                largest_yaw_error = std::max(largest_yaw_error, yaw_error);
                                require((armor.xyz_in_world - position).norm() < 1e-3,
                                        "Side-view armor position changed");
                                require(yaw_error < 0.05 * CV_PI / 180,
                                        "Yaw optimization clipped a correct side-view or wrapped PnP heading");
                            }
                            ++samples;
                        }
                    }
                }
            }
        }
        require(improved > 0, "Yaw search never improved any noisy observation");
        std::cout << "PASS yaw optimization: " << samples << " observations, " << improved
                  << " improved; largest exact yaw error=" << largest_yaw_error * 180 / CV_PI << " deg\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
