/// @file excavator_fk.cpp
/// @brief 挖掘机正/逆运动学实现

#include "dig_trajectory_planner/excavator_fk.h"

#include <algorithm>
#include <cmath>

namespace dig_trajectory_planner {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;
}  // namespace

ExcavatorFK::ExcavatorFK(double l_boom, double l_arm, double l_bucket,
                         const JointLimits& limits)
    : l_boom_(l_boom), l_arm_(l_arm), l_bucket_(l_bucket), limits_(limits) {}

Point2d ExcavatorFK::forwardKinematics(double boom, double arm, double bucket) const {
    // 转换为弧度
    double boom_rad = boom * kDeg2Rad;
    double arm_rad = arm * kDeg2Rad;
    double bucket_rad = bucket * kDeg2Rad;

    Point2d result;
    result.x = l_boom_ * std::cos(boom_rad) +
               l_arm_ * std::cos(boom_rad + arm_rad) +
               l_bucket_ * std::cos(boom_rad + arm_rad + bucket_rad);
    result.z = l_boom_ * std::sin(boom_rad) +
               l_arm_ * std::sin(boom_rad + arm_rad) +
               l_bucket_ * std::sin(boom_rad + arm_rad + bucket_rad);
    return result;
}

Point2d ExcavatorFK::forwardKinematics(const JointAngles& angles) const {
    return forwardKinematics(angles.boom, angles.arm, angles.bucket);
}

bool ExcavatorFK::inverseStart(double x, double z, double bucket_angle,
                                JointAngles& out) const {
    // 参考 dp.m 的 inverse_start 函数
    // 已知齿尖坐标和 bucket 角，反解 boom 和 arm 角

    double bucket_rad = bucket_angle * kDeg2Rad;

    // 计算铲斗相关角度
    double theta_FQV;
    if (bucket_angle <= 0) {
        theta_FQV = M_PI + bucket_rad;
    } else {
        theta_FQV = M_PI - bucket_rad;
    }

    // FV: 斗杆末端到齿尖的距离
    double FV = std::sqrt(l_arm_ * l_arm_ + l_bucket_ * l_bucket_ -
                          2 * std::cos(theta_FQV) * l_arm_ * l_bucket_);

    // CV: 基座到齿尖的距离
    double CV = std::sqrt(x * x + z * z);

    // 余弦定理求解
    double cos_FCV = (l_boom_ * l_boom_ + x * x + z * z - FV * FV) /
                     (2 * l_boom_ * std::sqrt(x * x + z * z));
    double cos_CFV = (l_boom_ * l_boom_ + FV * FV - CV * CV) /
                     (2 * l_boom_ * FV);

    // 检查是否有解
    if (cos_FCV < -1.0 || cos_FCV > 1.0 || cos_CFV < -1.0 || cos_CFV > 1.0) {
        return false;
    }

    double theta_FCV = std::acos(cos_FCV);
    double theta_CFV = std::acos(cos_CFV);

    // 计算 boom 角
    double boom_angle = theta_FCV + std::atan2(z, std::sqrt(x * x));

    // 计算 arm 角
    double theta_VFQ = std::acos((FV * FV + l_arm_ * l_arm_ - l_bucket_ * l_bucket_) /
                                 (2 * FV * l_arm_));
    double arm_angle;
    if (bucket_angle <= 0) {
        arm_angle = theta_CFV + theta_VFQ - M_PI;
    } else {
        arm_angle = theta_CFV - theta_VFQ - M_PI;
    }

    // 转换为度
    boom_angle *= kRad2Deg;
    arm_angle *= kRad2Deg;

    // 检查关节限位
    if (boom_angle < limits_.boom_limit[0] || boom_angle > limits_.boom_limit[1] ||
        arm_angle < limits_.arm_limit[0] || arm_angle > limits_.arm_limit[1] ||
        bucket_angle < limits_.bucket_limit[0] || bucket_angle > limits_.bucket_limit[1]) {
        return false;
    }

    out.boom = boom_angle;
    out.arm = arm_angle;
    out.bucket = bucket_angle;
    return true;
}

bool ExcavatorFK::inverseEnd(double x, double z, JointAngles& out) const {
    // 参考 dp.m 的 inverse_end 函数
    // 固定 theta_end = -180 deg（铲斗朝下）

    constexpr double theta_end = -180.0;

    double l_CV = std::sqrt(x * x + z * z);
    double theta_dig = std::atan2(-z, x) * kRad2Deg;
    double theta_CVQ = -theta_end - theta_dig;

    double l_CQ = std::sqrt(l_CV * l_CV + l_bucket_ * l_bucket_ -
                            2 * l_CV * l_bucket_ * std::cos(theta_CVQ * kDeg2Rad));

    // 检查三角不等式
    if (l_boom_ + l_arm_ - l_CQ <= 0 ||
        l_CV + l_CQ - l_bucket_ <= 0 ||
        l_boom_ + l_CQ - l_arm_ <= 0 ||
        l_arm_ + l_CQ - l_boom_ <= 0 ||
        l_CQ + l_arm_ - l_boom_ <= 0) {
        return false;
    }

    double theta_CFQ = std::acos((l_boom_ * l_boom_ + l_arm_ * l_arm_ - l_CQ * l_CQ) /
                                 (2 * l_boom_ * l_arm_)) * kRad2Deg;
    double theta_2 = theta_CFQ - 180.0;

    double theta_VCQ = std::acos((l_CV * l_CV + l_CQ * l_CQ - l_bucket_ * l_bucket_) /
                                 (2 * l_CQ * l_CV)) * kRad2Deg;
    double theta_FCQ = std::acos((l_boom_ * l_boom_ + l_CQ * l_CQ - l_arm_ * l_arm_) /
                                 (2 * l_boom_ * l_CQ)) * kRad2Deg;

    double theta_1 = theta_VCQ + theta_FCQ + (90.0 - theta_dig) - 90.0;

    double boom = theta_1;
    double arm = theta_2;
    double bucket = theta_end - theta_2 - theta_1;

    // 检查关节限位
    if (boom < limits_.boom_limit[0] || boom > limits_.boom_limit[1] ||
        arm < limits_.arm_limit[0] || arm > limits_.arm_limit[1] ||
        bucket < limits_.bucket_limit[0] || bucket > limits_.bucket_limit[1]) {
        return false;
    }

    out.boom = boom;
    out.arm = arm;
    out.bucket = bucket;
    return true;
}

bool ExcavatorFK::isInLimits(const JointAngles& angles) const {
    return angles.boom >= limits_.boom_limit[0] && angles.boom <= limits_.boom_limit[1] &&
           angles.arm >= limits_.arm_limit[0] && angles.arm <= limits_.arm_limit[1] &&
           angles.bucket >= limits_.bucket_limit[0] && angles.bucket <= limits_.bucket_limit[1];
}

void ExcavatorFK::clampToLimits(JointAngles& angles) const {
    angles.boom = std::clamp(angles.boom, limits_.boom_limit[0], limits_.boom_limit[1]);
    angles.arm = std::clamp(angles.arm, limits_.arm_limit[0], limits_.arm_limit[1]);
    angles.bucket = std::clamp(angles.bucket, limits_.bucket_limit[0], limits_.bucket_limit[1]);
}

}  // namespace dig_trajectory_planner
