#pragma once

/// @file trajectory_postprocess.h
/// @brief 挖掘轨迹后处理工具（纯 C++，无 ROS 依赖）。
///
/// 功能：
/// - 三次多项式插值：将离散关节角序列插值为连续轨迹
/// - 关节速度限幅：检查并调整超限速度
/// - 可达性校验：验证齿尖轨迹合理性

#include <vector>

#include "dig_trajectory_planner/excavator_fk.h"

namespace dig_trajectory_planner {

/// 后处理参数
struct PostprocessParams {
    double dt = 0.1;                    ///< 插值时间步长 (s)
    double max_boom_vel = 30.0;         ///< 动臂最大角速度 (deg/s)
    double max_arm_vel = 45.0;          ///< 斗杆最大角速度 (deg/s)
    double max_bucket_vel = 60.0;       ///< 铲斗最大角速度 (deg/s)
    bool check_reachability = true;     ///< 是否检查可达性
};

/// 带时间戳的关节角
struct TimedJointAngles {
    double time = 0.0;
    JointAngles angles;
};

/// 轨迹后处理器
class TrajectoryPostprocess {
public:
    /// 构造函数
    explicit TrajectoryPostprocess(const PostprocessParams& params = PostprocessParams{});

    /// 插值离散关节角序列
    /// @param waypoints 离散关节角序列（10个点）
    /// @return 带时间戳的连续轨迹
    std::vector<TimedJointAngles> interpolate(const std::vector<JointAngles>& waypoints) const;

    /// 检查并限幅关节速度
    /// @param trajectory 带时间戳的轨迹
    /// @return 调整后的轨迹
    std::vector<TimedJointAngles> clampVelocity(
        const std::vector<TimedJointAngles>& trajectory) const;

    /// 可达性校验
    /// @param trajectory 带时间戳的轨迹
    /// @param fk 运动学模型
    /// @return 是否全部可达
    bool checkReachability(const std::vector<TimedJointAngles>& trajectory,
                           const ExcavatorFK& fk) const;

    /// 完整后处理流程
    /// @param waypoints 离散关节角序列
    /// @param fk 运动学模型
    /// @return 后处理后的轨迹
    std::vector<TimedJointAngles> process(const std::vector<JointAngles>& waypoints,
                                          const ExcavatorFK& fk) const;

    /// 获取参数
    const PostprocessParams& getParams() const { return params_; }

    /// 设置参数
    void setParams(const PostprocessParams& params) { params_ = params; }

private:
    /// 三次多项式插值（单段）
    void interpolateSegment(const JointAngles& start,
                            const JointAngles& end,
                            double duration,
                            std::vector<TimedJointAngles>& output) const;

    PostprocessParams params_;
};

}  // namespace dig_trajectory_planner
