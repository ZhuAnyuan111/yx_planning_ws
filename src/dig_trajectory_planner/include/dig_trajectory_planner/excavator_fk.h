#pragma once

/// @file excavator_fk.h
/// @brief 挖掘机正/逆运动学工具（纯 C++，无 ROS 依赖）。
///
/// 功能：
/// - 正运动学：关节角 -> 齿尖坐标
/// - 逆运动学（起点）：齿尖坐标 + bucket_angle -> [boom, arm]
/// - 逆运动学（终点）：齿尖坐标 -> [boom, arm, bucket]
/// - 关节限位校验
///
/// 坐标系约定：
/// - 关节角单位为度 (deg)
/// - X 为水平方向（远离挖掘机为正），Z 为垂直方向（向上为正）
/// - boom 角：动臂与水平面夹角，上仰为正
/// - arm 角：斗杆相对于动臂的夹角，内收为负
/// - bucket 角：铲斗相对于斗杆的夹角，内翻为负

#include <array>

namespace dig_trajectory_planner {

/// 2D 点
struct Point2d {
    double x = 0.0;
    double z = 0.0;
};

/// 三关节角 (deg)
struct JointAngles {
    double boom = 0.0;    ///< 动臂角 (deg)
    double arm = 0.0;     ///< 斗杆角 (deg)
    double bucket = 0.0;  ///< 铲斗角 (deg)
};

/// 关节限位
struct JointLimits {
    std::array<double, 2> boom_limit{-45.44, 55.28};
    std::array<double, 2> arm_limit{-138.69, -37.35};
    std::array<double, 2> bucket_limit{-133.48, 30.53};
};

/// 挖掘机运动学类
class ExcavatorFK {
public:
    /// 构造函数
    /// @param l_boom 动臂长度 (m)
    /// @param l_arm 斗杆长度 (m)
    /// @param l_bucket 铲斗长度 (m)
    /// @param limits 关节限位（可选，使用默认值）
    ExcavatorFK(double l_boom, double l_arm, double l_bucket,
                const JointLimits& limits = JointLimits{});

    /// 正运动学：关节角 -> 齿尖坐标
    /// @param boom 动臂角 (deg)
    /// @param arm 斗杆角 (deg)
    /// @param bucket 铲斗角 (deg)
    /// @return 齿尖坐标 (x, z)
    Point2d forwardKinematics(double boom, double arm, double bucket) const;

    /// 正运动学：JointAngles -> 齿尖坐标
    Point2d forwardKinematics(const JointAngles& angles) const;

    /// 逆运动学（起点）：齿尖坐标 + bucket_angle -> [boom, arm]
    /// @param x 齿尖 X 坐标
    /// @param z 齿尖 Z 坐标
    /// @param bucket_angle 铲斗角 (deg)
    /// @param[out] out 输出关节角
    /// @return 是否有解
    bool inverseStart(double x, double z, double bucket_angle, JointAngles& out) const;

    /// 逆运动学（终点）：齿尖坐标 -> [boom, arm, bucket]
    /// 固定 bucket 角为 -180 deg（铲斗朝下）
    /// @param x 齿尖 X 坐标
    /// @param z 齿尖 Z 坐标
    /// @param[out] out 输出关节角
    /// @return 是否有解
    bool inverseEnd(double x, double z, JointAngles& out) const;

    /// 检查关节角是否在限位内
    bool isInLimits(const JointAngles& angles) const;

    /// 将关节角裁剪到限位内
    void clampToLimits(JointAngles& angles) const;

    /// 获取动臂长度
    double getBoomLength() const { return l_boom_; }
    /// 获取斗杆长度
    double getArmLength() const { return l_arm_; }
    /// 获取铲斗长度
    double getBucketLength() const { return l_bucket_; }
    /// 获取关节限位
    const JointLimits& getLimits() const { return limits_; }

private:
    double l_boom_;    ///< 动臂长度 (m)
    double l_arm_;     ///< 斗杆长度 (m)
    double l_bucket_;  ///< 铲斗长度 (m)
    JointLimits limits_;  ///< 关节限位
};

}  // namespace dig_trajectory_planner
