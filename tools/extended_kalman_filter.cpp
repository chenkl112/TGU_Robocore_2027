#include "extended_kalman_filter.hpp"

#include <numeric>

namespace tools {

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add)
    : x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add) {
    data["residual_yaw"] = 0.0;
    data["residual_pitch"] = 0.0;
    data["residual_distance"] = 0.0;
    data["residual_angle"] = 0.0;
    data["nis"] = 0.0;
    data["nees"] = 0.0;
    data["nis_fail"] = 0.0;
    data["nees_fail"] = 0.0;
    data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
    return predict(F, Q, [&](const Eigen::VectorXd& x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
    const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f) {
    P = F * P * F.transpose() + Q;
    x = f(x);
    return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract) {
    return update(z, H, R, [&](const Eigen::VectorXd& x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
    const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract) {
    const Eigen::VectorXd innovation = z_subtract(z, h(x));
    const Eigen::MatrixXd S = H * P * H.transpose() + R;
    const auto s_ldlt = S.ldlt();
    const Eigen::MatrixXd K =
        P * H.transpose() * s_ldlt.solve(Eigen::MatrixXd::Identity(S.rows(), S.cols()));

    x = x_add(x, K * innovation);
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

    const double nis = innovation.dot(s_ldlt.solve(innovation));
    constexpr double nis_threshold = 9.4877;  // chi-square, dof=4, 95%

    data["nis_fail"] = 0;
    data["nees_fail"] = 0;
    if (nis > nis_threshold) {
        ++nis_count_;
        data["nis_fail"] = 1;
    }
    total_count_++;
    last_nis = nis;

    recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

    if (recent_nis_failures.size() > window_size) {
        recent_nis_failures.pop_front();
    }

    int recent_failures =
        std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
    double recent_rate =
        static_cast<double>(recent_failures) / recent_nis_failures.size();

    data["residual_yaw"] = innovation[0];
    data["residual_pitch"] = innovation[1];
    data["residual_distance"] = innovation[2];
    data["residual_angle"] = innovation[3];
    data["nis"] = nis;
    data["nees"] = 0.0;
    data["recent_nis_failures"] = recent_rate;

    return x;
}

}  // namespace tools
