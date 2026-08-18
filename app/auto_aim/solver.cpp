#include "solver.hpp"

#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

constexpr double LIGHTBAR_LENGTH = 56e-3;
constexpr double BIG_ARMOR_WIDTH = 230e-3;
constexpr double SMALL_ARMOR_WIDTH = 135e-3;

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

Solver::Solver(const std::string& config_path)
    : R_gimbal2world_(Eigen::Matrix3d::Identity()) {
    auto config = toml::parse_file(config_path);

    auto read_matrix_3x3 = [&](const char* key) {
        auto data = config["camera"][key].as_array();
        if (!data || data->size() < 9) {
            LOG_WARN(MODULE, "Missing or invalid config key: {}", key);
            return Eigen::Matrix3d::Identity();
        }
        Eigen::Matrix3d m;
        for (int i = 0; i < 9; ++i)
            m(i / 3, i % 3) = (*data)[i].value_or(0.0);
        return m;
    };

    auto read_vector_3 = [&](const char* key) {
        auto data = config["camera"][key].as_array();
        if (!data || data->size() < 3) {
            LOG_WARN(MODULE, "Missing or invalid config key: {}", key);
            return Eigen::Vector3d::Zero();
        }
        return Eigen::Vector3d(
            (*data)[0].value_or(0.0), (*data)[1].value_or(0.0), (*data)[2].value_or(0.0));
    };

    R_gimbal2imubody_ = read_matrix_3x3("R_gimbal2imubody");
    R_camera2gimbal_ = read_matrix_3x3("R_camera2gimbal");
    t_camera2gimbal_ = read_vector_3("t_camera2gimbal");

    Eigen::Matrix3d camera_matrix = read_matrix_3x3("camera_matrix");
    cv::eigen2cv(camera_matrix, camera_matrix_);

    auto distort_data = config["camera"]["distort_coeffs"].as_array();
    Eigen::Matrix<double, 1, 5> distort_coeffs;
    if (distort_data && distort_data->size() >= 5) {
        for (int i = 0; i < 5; ++i) distort_coeffs[i] = (*distort_data)[i].value_or(0.0);
    } else {
        distort_coeffs.setZero();
    }
    cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond& q) {
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ =
        R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool Solver::solve(Armor& armor) const {
    if (armor.points.size() != 4) {
        LOG_WARN(MODULE, "PnP requires 4 armor points, got {}", armor.points.size());
        return false;
    }

    const auto& object_points =
        (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    cv::Vec3d rvec, tvec;
    const bool solved = cv::solvePnP(
        object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
        cv::SOLVEPNP_IPPE);
    if (!solved || tvec[2] <= 0) {
        LOG_WARN(MODULE, "PnP failed or returned non-positive depth");
        return false;
    }

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

    auto is_balance = (armor.type == ArmorType::big) &&
                      (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                       armor.name == ArmorName::five);
    if (is_balance) return true;

    optimize_yaw(armor);
    return true;
}

std::vector<cv::Point2f> Solver::reproject_armor(
    const Eigen::Vector3d& xyz_in_world, double yaw, ArmorType type) const {
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    constexpr double pitch = 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    const Eigen::Matrix3d R_armor2world{
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch},
        {-sin_pitch, 0, cos_pitch}};

    const Eigen::Vector3d& t_armor2world = xyz_in_world;
    Eigen::Matrix3d R_armor2camera =
        R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    Eigen::Vector3d t_armor2camera =
        R_camera2gimbal_.transpose() *
        (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    cv::Vec3d rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    std::vector<cv::Point2f> image_points;
    const auto& object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    cv::projectPoints(
        object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    return image_points;
}

void Solver::optimize_yaw(Armor& armor) const {
    Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

    constexpr int SEARCH_RANGE = 140;
    auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

    auto min_error = 1e10;
    auto best_yaw = armor.ypr_in_world[0];

    for (int i = 0; i < SEARCH_RANGE; i++) {
        double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
        auto error = armor_reprojection_error(armor, yaw);

        if (error < min_error) {
            min_error = error;
            best_yaw = yaw;
        }
    }

    armor.yaw_raw = armor.ypr_in_world[0];
    armor.ypr_in_world[0] = best_yaw;
}

double Solver::armor_reprojection_error(const Armor& armor, double yaw) const {
    auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type);
    auto error = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
    return error;
}

std::vector<cv::Point2f> Solver::world2pixel(
    const std::vector<cv::Point3f>& worldPoints) {
    Eigen::Matrix3d R_world2camera =
        R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
    Eigen::Vector3d t_world2camera =
        -R_camera2gimbal_.transpose() * t_camera2gimbal_;

    cv::Mat rvec, tvec;
    cv::eigen2cv(R_world2camera, rvec);
    cv::eigen2cv(t_world2camera, tvec);

    std::vector<cv::Point3f> valid_world_points;
    for (const auto& world_point : worldPoints) {
        Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
        Eigen::Vector3d camera_point =
            R_world2camera * world_point_eigen + t_world2camera;
        if (camera_point.z() > 0) {
            valid_world_points.push_back(world_point);
        }
    }
    if (valid_world_points.empty()) {
        return {};
    }
    std::vector<cv::Point2f> pixelPoints;
    cv::projectPoints(
        valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
    return pixelPoints;
}

}  // namespace app::auto_aim
