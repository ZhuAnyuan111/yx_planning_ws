/// @file trajectory_postprocess.cpp
/// @brief 挖掘轨迹后处理实现

#include "dig_trajectory_planner/trajectory_postprocess.h"

#include <algorithm>
#include <cmath>

namespace dig_trajectory_planner {

TrajectoryPostprocess::TrajectoryPostprocess(const PostprocessParams& params)
    : params_(params) {}

std::vector<TimedJointAngles> TrajectoryPostprocess::interpolate(
    const std::vector<JointAngles>& waypoints) const {
    std::vector<TimedJointAngles> result;

    if (waypoints.empty()) return result;

    // 添加起点
    TimedJointAngles start;
    start.time = 0.0;
    start.angles = waypoints[0];
    result.push_back(start);

    // 逐段插值
    for (size_t i = 1; i < waypoints.size(); ++i) {
        // 根据最大速度计算段时长
        double boom_delta = std::abs(waypoints[i].boom - waypoints[i - 1].boom);
        double arm_delta = std::abs(waypoints[i].arm - waypoints[i - 1].arm);
        double bucket_delta = std::abs(waypoints[i].bucket - waypoints[i - 1].bucket);

        double duration = std::max({
            boom_delta / params_.max_boom_vel,
            arm_delta / params_.max_arm_vel,
            bucket_delta / params_.max_bucket_vel,
            0.1  // 最小段时长
        });

        interpolateSegment(waypoints[i - 1], waypoints[i], duration, result);
    }

    return result;
}

void TrajectoryPostprocess::interpolateSegment(const JointAngles& start,
                                               const JointAngles& end,
                                               double duration,
                                               std::vector<TimedJointAngles>& output) const {
    int n_steps = static_cast<int>(duration / params_.dt);
    if (n_steps < 1) n_steps = 1;

    double start_time = output.empty() ? 0.0 : output.back().time;

    for (int i = 1; i <= n_steps; ++i) {
        double t = static_cast<double>(i) / n_steps;

        // 三次多项式插值（平滑加减速）
        // s(t) = 3t^2 - 2t^3 (S曲线)
        double s = 3 * t * t - 2 * t * t * t;

        TimedJointAngles point;
        point.time = start_time + i * params_.dt;
        point.angles.boom = start.boom + s * (end.boom - start.boom);
        point.angles.arm = start.arm + s * (end.arm - start.arm);
        point.angles.bucket = start.bucket + s * (end.bucket - start.bucket);

        output.push_back(point);
    }
}

std::vector<TimedJointAngles> TrajectoryPostprocess::clampVelocity(
    const std::vector<TimedJointAngles>& trajectory) const {
    if (trajectory.size() < 2) return trajectory;

    std::vector<TimedJointAngles> result = trajectory;

    // 检查并调整速度
    for (size_t i = 1; i < result.size(); ++i) {
        double dt = result[i].time - result[i - 1].time;
        if (dt <= 0) continue;

        double boom_vel = std::abs(result[i].angles.boom - result[i - 1].angles.boom) / dt;
        double arm_vel = std::abs(result[i].angles.arm - result[i - 1].angles.arm) / dt;
        double bucket_vel = std::abs(result[i].angles.bucket - result[i - 1].angles.bucket) / dt;

        // 如果超速，拉伸时间
        double scale = 1.0;
        if (boom_vel > params_.max_boom_vel) {
            scale = std::max(scale, boom_vel / params_.max_boom_vel);
        }
        if (arm_vel > params_.max_arm_vel) {
            scale = std::max(scale, arm_vel / params_.max_arm_vel);
        }
        if (bucket_vel > params_.max_bucket_vel) {
            scale = std::max(scale, bucket_vel / params_.max_bucket_vel);
        }

        if (scale > 1.0) {
            // 调整后续所有点的时间
            double time_offset = (scale - 1.0) * dt;
            for (size_t j = i; j < result.size(); ++j) {
                result[j].time += time_offset;
            }
        }
    }

    return result;
}

bool TrajectoryPostprocess::checkReachability(const std::vector<TimedJointAngles>& trajectory,
                                              const ExcavatorFK& fk) const {
    for (const auto& point : trajectory) {
        if (!fk.isInLimits(point.angles)) {
            return false;
        }

        // 正运动学计算，检查是否有效
        Point2d pos = fk.forwardKinematics(point.angles);
        if (std::isnan(pos.x) || std::isnan(pos.z) ||
            std::isinf(pos.x) || std::isinf(pos.z)) {
            return false;
        }
    }
    return true;
}

std::vector<TimedJointAngles> TrajectoryPostprocess::process(
    const std::vector<JointAngles>& waypoints,
    const ExcavatorFK& fk) const {
    // 1. 插值
    auto trajectory = interpolate(waypoints);

    // 2. 速度限幅
    trajectory = clampVelocity(trajectory);

    // 3. 可达性校验
    if (params_.check_reachability) {
        if (!checkReachability(trajectory, fk)) {
            // 可达性失败，返回空轨迹
            return {};
        }
    }

    return trajectory;
}

}  // namespace dig_trajectory_planner
