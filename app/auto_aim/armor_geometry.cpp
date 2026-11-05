#include "armor_geometry.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/core/eigen.hpp>
#include <stdexcept>

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {
namespace {
// 灯条中心端点的物理尺寸。模型原点位于装甲中心，x 指向背面，y 向左，z 向上。
// 顺序与图像角点相同：左上、右上、右下、左下。
constexpr double LIGHTBAR_LENGTH = 56e-3;
constexpr double BIG_ARMOR_WIDTH = 230e-3;
constexpr double SMALL_ARMOR_WIDTH = 135e-3;

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{{0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
                                                {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{{0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
                                                  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
                                                  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

} // namespace

ArmorGeometry::ArmorGeometry(const CalibrationConfig& calibration, double max_balance_pitch)
    : R_gimbal2imubody_(calibration.R_gimbal2imubody),
      R_camera2gimbal_(calibration.R_camera2gimbal), t_camera2gimbal_(calibration.t_camera2gimbal),
      max_balance_pitch_(max_balance_pitch) {
    validate(calibration);
    if (!std::isfinite(max_balance_pitch_) || max_balance_pitch_ <= 0 ||
        max_balance_pitch_ >= CV_PI / 2) {
        throw std::invalid_argument("solver.max_balance_pitch must be between 0 and pi/2 radians");
    }
    cv::eigen2cv(calibration.camera_matrix, camera_matrix_);
    cv::eigen2cv(calibration.distort_coeffs, distort_coeffs_);
}

Eigen::Matrix3d ArmorGeometry::R_gimbal2world() const {
    return R_gimbal2world_;
}

void ArmorGeometry::set_R_gimbal2world(const Eigen::Quaterniond& q) {
    if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-6) {
        throw std::invalid_argument("invalid gimbal quaternion");
    }
    Eigen::Matrix3d R_imubody2imuabs = q.normalized().toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

bool ArmorGeometry::solve_size_pose(Armor& armor, double& reprojection_error,
                                    MountingModel model_kind) const {
    armor.pose_valid = false;
    const bool allow_small_tilt = model_kind == MountingModel::TiltedSmall;
    // 平地步兵的装甲横边与世界系水平面平行，小装甲安装俯仰为 15°，平衡步兵允许俯仰。
    // 比较尺寸前先拟合这些物理约束，否则自由姿态的小装甲可通过倾斜灯条并缩短距离，
    // 错误地解释正面的大装甲。TiltedSmall 仅用于否决不可靠的尺寸判断。
    const bool balance = armor.type == ArmorType::big;
    const int dimensions = allow_small_tilt ? 6 : balance ? 5 : 4;
    const double mounting_pitch = 15.0 * CV_PI / 180.0;
    const double min_pitch = allow_small_tilt ? 0 : -max_balance_pitch_;
    const double max_pitch = allow_small_tilt ? 30 * CV_PI / 180 : max_balance_pitch_;
    const Eigen::Matrix3d world_to_camera =
        R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
    const Eigen::Vector3d camera_in_world = R_gimbal2world_ * t_camera2gimbal_;
    const auto& model = balance ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    std::vector<cv::Point3d> object_points;
    for (const auto& point : model)
        object_points.emplace_back(point.x, point.y, point.z);

    auto residual = [&](const Eigen::VectorXd& pose, Eigen::Matrix<double, 8, 1>& result) {
        const double pitch = dimensions >= 5 ? pose[4] : mounting_pitch;
        const double roll = allow_small_tilt ? pose[5] : 0;
        const auto rotation = tools::rotation_matrix({pose[3], pitch, roll});
        const Eigen::Vector3d translation = world_to_camera * (pose.head<3>() - camera_in_world);
        if (!translation.allFinite() || translation.z() <= 0)
            return false;
        cv::Mat rmat, rvec;
        cv::eigen2cv(Eigen::Matrix3d(world_to_camera * rotation), rmat);
        cv::Rodrigues(rmat, rvec);
        std::vector<cv::Point2d> projected;
        cv::projectPoints(object_points, rvec,
                          cv::Vec3d(translation.x(), translation.y(), translation.z()),
                          camera_matrix_, distort_coeffs_, projected);
        for (int i = 0; i < 4; ++i) {
            result[2 * i] = projected[i].x - armor.points[i].x;
            result[2 * i + 1] = projected[i].y - armor.points[i].y;
        }
        return result.allFinite();
    };

    Eigen::VectorXd best_pose(dimensions);
    double best_error = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d ray = armor.xyz_in_world - camera_in_world;
    const double view_yaw = std::atan2(ray.y(), ray.x());
    // 远距离角点噪声可能让 IPPE 选中镜像法向；两条平面姿态分支采用相同约束拟合。
    for (bool reflected : {false, true}) {
        Eigen::VectorXd pose(dimensions);
        pose.head<3>() = armor.xyz_in_world;
        pose[3] = reflected ? 2 * view_yaw - armor.ypr_in_world[0] : armor.ypr_in_world[0];
        if (dimensions >= 5)
            pose[4] =
                std::clamp((reflected ? -1 : 1) * armor.ypr_in_world[1], min_pitch, max_pitch);
        if (allow_small_tilt)
            pose[5] = armor.ypr_in_world[2];
        Eigen::Matrix<double, 8, 1> errors;
        if (!residual(pose, errors))
            continue;
        double cost = errors.squaredNorm();
        // 使用有限差分雅可比的阻尼最小二乘，残差下降时减小阻尼，否则缩小步长。
        double damping = 1e-3;
        for (int iteration = 0; iteration < 30; ++iteration) {
            Eigen::MatrixXd jacobian(8, dimensions);
            constexpr double increment = 1e-5;
            bool valid = true;
            for (int column = 0; column < dimensions; ++column) {
                auto perturbed = pose;
                perturbed[column] += increment;
                Eigen::Matrix<double, 8, 1> changed;
                if (!residual(perturbed, changed)) {
                    valid = false;
                    break;
                }
                jacobian.col(column) = (changed - errors) / increment;
            }
            if (!valid)
                break;
            Eigen::MatrixXd normal = jacobian.transpose() * jacobian;
            normal.diagonal().array() += damping * normal.diagonal().array().max(1.0);
            Eigen::VectorXd gradient = jacobian.transpose() * errors;
            Eigen::VectorXd step = normal.ldlt().solve(-gradient);
            if (!step.allFinite())
                break;
            if (dimensions >= 5 && ((pose[4] <= min_pitch + 1e-9 && step[4] < 0) ||
                                    (pose[4] >= max_pitch - 1e-9 && step[4] > 0))) {
                // 到达俯仰边界后固定该维度，继续优化其余坐标，避免重复提出越界步长。
                normal.row(4).setZero();
                normal.col(4).setZero();
                normal(4, 4) = 1;
                gradient[4] = 0;
                step = normal.ldlt().solve(-gradient);
                if (!step.allFinite())
                    break;
            }
            Eigen::VectorXd candidate = pose + step;
            if (dimensions >= 5)
                candidate[4] = std::clamp(candidate[4], min_pitch, max_pitch);
            Eigen::Matrix<double, 8, 1> changed;
            if (residual(candidate, changed) && changed.squaredNorm() < cost) {
                const double improvement = cost - changed.squaredNorm();
                pose = std::move(candidate);
                errors = changed;
                cost = errors.squaredNorm();
                damping = std::max(1e-9, damping / 3);
                if (improvement < 1e-8 || step.norm() < 1e-8)
                    break;
            } else {
                damping *= 10;
                if (damping > 1e9)
                    break;
            }
        }
        if (cost < best_error) {
            best_error = cost;
            best_pose = pose;
        }
    }
    if (!std::isfinite(best_error))
        return false;
    reprojection_error = std::sqrt(best_error) / 2;
    armor.xyz_in_world = best_pose.head<3>();
    armor.ypr_in_world = {tools::limit_rad(best_pose[3]),
                          dimensions >= 5 ? best_pose[4] : mounting_pitch,
                          allow_small_tilt ? best_pose[5] : 0};
    armor.yaw_raw = armor.ypr_in_world[0];
    return update_derived_pose(armor);
}

bool ArmorGeometry::solve_pose(Armor& armor, double& reprojection_error,
                               PoseRefinement refinement) const {
    armor.pose_valid = false;
    if (armor.type != ArmorType::big && armor.type != ArmorType::small)
        return false;

    const auto& object_points =
        (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    for (const auto& point : armor.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            return false;
    }
    if (std::abs(cv::contourArea(armor.points)) < 1.0)
        return false;
    // OpenCV 的 array 视图是行向量，投影输出是列向量；显式统一形状后再比较残差。
    const cv::Mat image_points = cv::Mat(armor.points).reshape(2, 4);
    cv::Vec3d rvec, tvec;
    bool solved = false;
    for (const auto method : {cv::SOLVEPNP_IPPE, cv::SOLVEPNP_ITERATIVE}) {
        try {
            solved = cv::solvePnP(object_points, image_points, camera_matrix_, distort_coeffs_,
                                  rvec, tvec, false, method);
        } catch (const cv::Exception&) {
            solved = false;
        }
        if (solved && cv::checkRange(rvec) && cv::checkRange(tvec) && tvec[2] > 0)
            break;
        // IPPE 对合法平面偶尔返回 NaN；保持角点对应关系，改用迭代 PnP 重试。
        solved = false;
    }
    if (!solved) {
        LOG_WARN(MODULE, "PnP failed or returned nonfinite/non-positive depth");
        return false;
    }

    // 原图像素 RMS 只评价 PnP 初值是否可用；尺寸可信度由安装模型的残差判断。
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, projected);
    reprojection_error = cv::norm(image_points, cv::Mat(projected), cv::NORM_L2) / 2.0;
    if (!std::isfinite(reprojection_error))
        return false;
    if (refinement == PoseRefinement::LeastSquares) {
        // IPPE 仅提供平面初值，先做最小二乘细化，再用残差排除尺寸假设。
        auto refined_rvec = rvec;
        auto refined_tvec = tvec;
        try {
            cv::solvePnPRefineLM(object_points, image_points, camera_matrix_, distort_coeffs_,
                                 refined_rvec, refined_tvec);
            if (cv::checkRange(refined_rvec) && cv::checkRange(refined_tvec) &&
                refined_tvec[2] > 0) {
                cv::projectPoints(object_points, refined_rvec, refined_tvec, camera_matrix_,
                                  distort_coeffs_, projected);
                const double error = cv::norm(image_points, cv::Mat(projected), cv::NORM_L2) / 2.0;
                if (error < reprojection_error) {
                    rvec = refined_rvec;
                    tvec = refined_tvec;
                    reprojection_error = error;
                }
            }
        } catch (const cv::Exception&) {
            // 细化失败时保留已经验证过的 PnP 初值。
        }
    }

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    const Eigen::Vector3d xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

    armor.yaw_raw = armor.ypr_in_world[0];
    return update_derived_pose(armor);
}

bool ArmorGeometry::finish_pose(Armor& armor) const {
    const bool balance = armor.type == ArmorType::big &&
                         (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                          armor.name == ArmorName::five);
    if (!balance)
        optimize_yaw(armor);
    armor.pose_valid = update_derived_pose(armor);
    return armor.pose_valid;
}

bool ArmorGeometry::update_derived_pose(Armor& armor) const {
    // 所有几何步骤只产生世界系位姿，其余缓存统一从此入口派生。
    armor.xyz_in_gimbal = R_gimbal2world_.transpose() * armor.xyz_in_world;
    armor.ypr_in_gimbal = tools::eulers(
        Eigen::Matrix3d(R_gimbal2world_.transpose() * tools::rotation_matrix(armor.ypr_in_world)),
        2, 1, 0);
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
    return armor.xyz_in_world.allFinite() && armor.ypr_in_world.allFinite() &&
           armor.xyz_in_gimbal.allFinite() && armor.ypr_in_gimbal.allFinite() &&
           armor.ypd_in_world.allFinite();
}

std::vector<cv::Point2f> ArmorGeometry::reproject_armor(const Eigen::Vector3d& xyz_in_world,
                                                        double yaw, ArmorType type) const {
    if (type != ArmorType::big && type != ArmorType::small) {
        throw std::invalid_argument("armor reprojection requires a resolved physical size");
    }
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    constexpr double pitch = 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    const Eigen::Matrix3d R_armor2world{{cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
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
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    return image_points;
}

void ArmorGeometry::optimize_yaw(Armor& armor) const {
    armor.yaw_raw = armor.ypr_in_world[0];
    auto best_yaw = armor.yaw_raw;
    auto min_error = armor_reprojection_error(armor, best_yaw);
    if (!std::isfinite(min_error))
        return;

    // 可见 yaw 区间以相机到目标的视线为中心；保留原始 PnP 候选，避免网格搜索增大残差。
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

double ArmorGeometry::armor_reprojection_error(const Armor& armor, double yaw) const {
    auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type);
    auto error = 0.0;
    for (int i = 0; i < 4; i++)
        error += cv::norm(armor.points[i] - image_points[i]);
    return error;
}

std::vector<cv::Point2f>
ArmorGeometry::world2pixel(const std::vector<cv::Point3f>& worldPoints) const {
    Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
    Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

    cv::Mat rvec, tvec;
    cv::eigen2cv(R_world2camera, rvec);
    cv::eigen2cv(t_world2camera, tvec);

    std::vector<cv::Point3f> valid_world_points;
    for (const auto& world_point : worldPoints) {
        Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
        Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;
        if (camera_point.z() > 0) {
            valid_world_points.push_back(world_point);
        }
    }
    if (valid_world_points.empty()) {
        return {};
    }
    std::vector<cv::Point2f> pixelPoints;
    cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
    return pixelPoints;
}

} // namespace app::auto_aim
