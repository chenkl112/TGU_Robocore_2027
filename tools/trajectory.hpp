#ifndef TGU_ROBOCORE_2027_TRAJECTORY_HPP
#define TGU_ROBOCORE_2027_TRAJECTORY_HPP
#pragma once

namespace tools {

struct Trajectory {
    bool unsolvable = true;
    double fly_time = 0.0;
    double pitch = 0.0;  // positive = pointing upward

    Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools

#endif  // TGU_ROBOCORE_2027_TRAJECTORY_HPP
