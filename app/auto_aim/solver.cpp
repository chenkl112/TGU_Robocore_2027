#include "solver.hpp"

#include <cmath>
#include <array>
#include <limits>
#include <stdexcept>
#include <utility>
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
    max_reprojection_error_ = config["solver"]["max_reprojection_error"].value_or(3.0);
    min_type_error_gap_ = config["solver"]["min_type_error_gap"].value_or(0.25);
    if (!std::isfinite(max_reprojection_error_) || max_reprojection_error_ <= 0 ||
        !std::isfinite(min_type_error_gap_) || min_type_error_gap_ <= 0) {
        throw std::invalid_argument("solver reprojection thresholds must be finite and positive");
    }

    auto read_matrix_3x3 = [&](const char* key) -> Eigen::Matrix3d {
        auto data = config["camera"][key].as_array();
        if (!data || data->size() != 9) {
            throw std::invalid_argument(std::string("camera.") + key + " must contain 9 values");
        }
        Eigen::Matrix3d m;
        for (int i = 0; i < 9; ++i) {
            auto value = (*data)[i].value<double>();
            if (!value) throw std::invalid_argument(std::string("invalid camera.") + key);
            m(i / 3, i % 3) = *value;
        }
        if (!m.allFinite()) throw std::invalid_argument(std::string("nonfinite camera.") + key);
        return m;
    };

    auto read_vector_3 = [&](const char* key) -> Eigen::Vector3d {
        auto data = config["camera"][key].as_array();
        if (!data || data->size() != 3) {
            throw std::invalid_argument(std::string("camera.") + key + " must contain 3 values");
        }
        Eigen::Vector3d result;
        for (int i = 0; i < 3; ++i) {
            auto value = (*data)[i].value<double>();
            if (!value || !std::isfinite(*value)) throw std::invalid_argument(std::string("invalid camera.") + key);
            result[i] = *value;
        }
        return result;
    };

    R_gimbal2imubody_ = read_matrix_3x3("R_gimbal2imubody");
    R_camera2gimbal_ = read_matrix_3x3("R_camera2gimbal");
    t_camera2gimbal_ = read_vector_3("t_camera2gimbal");
    for (const auto& rotation : {R_gimbal2imubody_, R_camera2gimbal_}) {
        if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-3 ||
            std::abs(rotation.determinant() - 1) > 1e-3) {
            throw std::invalid_argument("camera extrinsic rotation must be orthonormal with determinant +1");
        }
    }

    Eigen::Matrix3d camera_matrix = read_matrix_3x3("camera_matrix");
    if (camera_matrix(0, 0) <= 0 || camera_matrix(1, 1) <= 0 ||
        (camera_matrix.row(2) - Eigen::RowVector3d(0, 0, 1)).norm() > 1e-9) {
        throw std::invalid_argument("invalid camera intrinsics");
    }
    cv::eigen2cv(camera_matrix, camera_matrix_);

    auto distort_data = config["camera"]["distort_coeffs"].as_array();
    Eigen::Matrix<double, 1, 5> distort_coeffs;
    if (!distort_data || distort_data->size() != 5) {
        throw std::invalid_argument("camera.distort_coeffs must contain 5 values");
    }
    for (int i = 0; i < 5; ++i) {
        auto value = (*distort_data)[i].value<double>();
        if (!value || !std::isfinite(*value)) throw std::invalid_argument("invalid camera.distort_coeffs");
        distort_coeffs[i] = *value;
    }
    cv::eigen2cv(distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond& q) {
    if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6) {
        throw std::invalid_argument("invalid gimbal quaternion");
    }
    Eigen::Matrix3d R_imubody2imuabs = q.normalized().toRotationMatrix();
    R_gimbal2world_ =
        R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool Solver::solve(Armor& armor) const {
    double error;
    return solve_pose(armor, error) && finish_pose(armor);
}

bool Solver::solve_observation(Armor& armor, std::optional<ArmorType> tracked_type) const {
    if (!is_robot_target(armor.name)) return false;
    const auto known_type = armor_type_from_name(armor.name);
    if (known_type != ArmorType::unknown) {
        auto candidate = armor;
        candidate.type = known_type;
        double error;
        if (!solve_pose(candidate, error) || error > max_reprojection_error_ ||
            !finish_pose(candidate)) return false;
        armor = std::move(candidate);
        return true;
    }

    std::array<Armor, 2> candidates{armor, armor};
    std::array<double, 2> errors;
    errors.fill(std::numeric_limits<double>::infinity());
    for (int i = 0; i < 2; ++i) {
        candidates[i].type = i == 0 ? ArmorType::big : ArmorType::small;
        double error;
        if (solve_pose(candidates[i], error) && error <= max_reprojection_error_) errors[i] = error;
    }
    const int best = errors[0] < errors[1] ? 0 : 1;
    if (!std::isfinite(errors[best])) return false;
    int selected = best;
    if (std::abs(errors[0] - errors[1]) < min_type_error_gap_) {
        // Near-affine projections do not reliably distinguish the two sizes.
        // Never promote a pixel aspect ratio to a confirmed vehicle identity.
        if (!tracked_type || *tracked_type == ArmorType::unknown) return false;
        selected = *tracked_type == ArmorType::big ? 0 : 1;
    }
    if (!finish_pose(candidates[selected])) return false;
    armor = std::move(candidates[selected]);
    return true;
}

bool Solver::solve_pose(Armor& armor, double& reprojection_error) const {
    if (armor.type != ArmorType::big && armor.type != ArmorType::small) return false;
    if (armor.points.size() != 4) {
        LOG_WARN(MODULE, "PnP requires 4 armor points, got {}", armor.points.size());
        return false;
    }

    const auto& object_points =
        (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    for (const auto& point : armor.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    }
    if (std::abs(cv::contourArea(armor.points)) < 1.0) return false;
    cv::Vec3d rvec, tvec;
    bool solved = false;
    for (const auto method : {cv::SOLVEPNP_IPPE, cv::SOLVEPNP_ITERATIVE}) {
        try {
            solved = cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_,
                                  rvec, tvec, false, method);
        } catch (const cv::Exception&) {
            solved = false;
        }
        if (solved && cv::checkRange(rvec) && cv::checkRange(tvec) && tvec[2] > 0) break;
        // OpenCV IPPE can report success with NaNs for valid planar poses.
        // Retry the same physical correspondences with its iterative solver.
        solved = false;
    }
    if (!solved) {
        LOG_WARN(MODULE, "PnP failed or returned nonfinite/non-positive depth");
        return false;
    }

    // Compare both physical sizes using the full PnP pose before imposing the
    // yaw optimizer's upright-armor model. Units are pixels in the source image.
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, projected);
    reprojection_error = cv::norm(armor.points, projected, cv::NORM_L2) / 2.0;
    if (!std::isfinite(reprojection_error)) return false;

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

    armor.yaw_raw = armor.ypr_in_world[0];
    return armor.xyz_in_world.allFinite() && armor.ypr_in_world.allFinite() &&
           armor.ypd_in_world.allFinite();
}

bool Solver::finish_pose(Armor& armor) const {
    auto is_balance = (armor.type == ArmorType::big) &&
                      (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                       armor.name == ArmorName::five);
    if (is_balance) return armor.xyz_in_world.allFinite() && armor.ypr_in_world.allFinite();

    optimize_yaw(armor);
    return armor.xyz_in_world.allFinite() && armor.ypr_in_world.allFinite() && armor.ypd_in_world.allFinite();
}

std::vector<cv::Point2f> Solver::reproject_armor(
    const Eigen::Vector3d& xyz_in_world, double yaw, ArmorType type) const {
    if (type != ArmorType::big && type != ArmorType::small) {
        throw std::invalid_argument("armor reprojection requires a resolved physical size");
    }
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
    armor.yaw_raw = armor.ypr_in_world[0];
    auto best_yaw = armor.yaw_raw;
    auto min_error = armor_reprojection_error(armor, best_yaw);
    if (!std::isfinite(min_error)) return;

    // The visible yaw interval follows the target's camera ray, not the optical
    // axis. Keep PnP as a candidate so the grid can never worsen a valid pose.
    const Eigen::Vector3d camera_in_world = R_gimbal2world_ * t_camera2gimbal_;
    const Eigen::Vector3d camera_to_armor = armor.xyz_in_world - camera_in_world;
    const double view_yaw = std::atan2(camera_to_armor.y(), camera_to_armor.x());
    for (int offset_deg = -90; offset_deg <= 90; ++offset_deg) {
        const double yaw = tools::limit_rad(view_yaw + offset_deg * CV_PI / 180.0);
        auto error = armor_reprojection_error(armor, yaw);

        if (std::isfinite(error) && error < min_error) {
            min_error = error;
            best_yaw = yaw;
        }
    }

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
