/// @file dig_planner_dp.cpp
/// @brief 基于动态规划的挖掘轨迹优化器实现

#include "dig_trajectory_planner/dig_planner_dp.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace dig_trajectory_planner {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kDeg2Rad = M_PI / 180.0;
}  // namespace

DigPlannerDP::DigPlannerDP(const DigPlannerParams& params) : params_(params) {}

DigPlannerResult DigPlannerDP::plan(const TerrainProfile& terrain,
                                    const ExcavatorFK& fk,
                                    double x_start,
                                    double x_end,
                                    double start_z_offset,
                                    double end_z_offset,
                                    double bucket_angle) const {
    DigPlannerResult result;
    result.success = false;

    // 生成候选终点列表（5个）
    auto x_end_list = generateEndPoints(x_start, x_end);

    // 存储每个终点的结果
    std::vector<double> volume_set(x_end_list.size(), kInf);
    std::vector<std::vector<JointAngles>> trajectory_set(x_end_list.size());

    // 遍历每个候选终点
    for (size_t n = 0; n < x_end_list.size(); ++n) {
        double x_end_curr = x_end_list[n];

        // 生成起终点关节角
        JointAngles start_point, end_point;
        double z_start;
        if (!generateStartEndPoints(terrain, fk, x_start, x_end_curr,
                                    start_z_offset, end_z_offset, bucket_angle,
                                    start_point, end_point, z_start)) {
            continue;  // 逆解失败，跳过此终点
        }

        // 生成动臂角度列表
        auto boom_list = generateBoomList(start_point, end_point);
        double boom_up_range = end_point.boom - start_point.boom;

        // 生成 arm/bucket 耦合增益列表
        std::vector<double> arm_gain_list, bucket_gain_list;
        generateCouplingGains(start_point, end_point, arm_gain_list, bucket_gain_list);

        int n_boom = static_cast<int>(boom_list.size());
        int n_arm = static_cast<int>(arm_gain_list.size());
        int n_bucket = static_cast<int>(bucket_gain_list.size());
        int n_states = n_boom * n_arm * n_bucket;

        // DP 矩阵
        int n_steps = params_.dig_steps;
        std::vector<std::vector<double>> cost_matrix(n_steps, std::vector<double>(n_states, kInf));
        std::vector<std::vector<double>> volume_matrix(n_steps, std::vector<double>(n_states, 0.0));
        std::vector<std::vector<int>> traj_matrix(n_steps, std::vector<int>(n_states, 0));

        // 初始状态：动臂下降段中间位置
        int init_boom_idx = static_cast<int>(params_.boom_down_steps);
        int init_arm_idx = n_arm / 2;
        int init_bucket_idx = n_bucket / 2;
        int init_state = init_boom_idx * n_arm * n_bucket + init_arm_idx * n_bucket + init_bucket_idx;

        cost_matrix[0][init_state] = 0.0;
        volume_matrix[0][init_state] = 0.0;
        traj_matrix[0][init_state] = init_state;

        // DP 递推
        for (int step = 1; step < n_steps; ++step) {
            for (int s_curr = 0; s_curr < n_states; ++s_curr) {
                // 解码当前状态
                int boom_idx = s_curr / (n_arm * n_bucket);
                int arm_idx = (s_curr / n_bucket) % n_arm;
                int bucket_idx = s_curr % n_bucket;

                JointAngles after;
                after.boom = boom_list[boom_idx];
                after.arm = start_point.arm + arm_gain_list[arm_idx] * (step + 1) / n_steps;
                after.bucket = start_point.bucket + bucket_gain_list[bucket_idx] * (step + 1) / n_steps;

                // 遍历所有可能的前驱状态
                for (int s_prev = 0; s_prev < n_states; ++s_prev) {
                    if (cost_matrix[step - 1][s_prev] >= kInf) continue;

                    // 解码前驱状态
                    int prev_boom_idx = s_prev / (n_arm * n_bucket);
                    int prev_arm_idx = (s_prev / n_bucket) % n_arm;
                    int prev_bucket_idx = s_prev % n_bucket;

                    JointAngles before;
                    before.boom = boom_list[prev_boom_idx];
                    before.arm = start_point.arm + arm_gain_list[prev_arm_idx] * step / n_steps;
                    before.bucket = start_point.bucket + bucket_gain_list[prev_bucket_idx] * step / n_steps;

                    // 硬约束：动臂单步变化限幅
                    double boom_delta = std::abs(after.boom - before.boom);
                    if (boom_delta > params_.max_boom_step) continue;

                    // 硬约束：前N步禁止动臂后退
                    if (step <= params_.no_retreat_steps && after.boom > before.boom) continue;

                    // 计算挖掘量
                    double dig_vol = calcDigVolume(before, after, terrain, fk);
                    if (dig_vol >= kInf) continue;

                    // 计算代价
                    double cost = calcTransitionCost(step, before, after, dig_vol,
                                                     start_point, end_point, terrain, fk,
                                                     z_start, boom_up_range);
                    if (cost >= kInf) continue;

                    // 更新 DP 矩阵
                    double total_cost = cost_matrix[step - 1][s_prev] + cost;
                    if (total_cost < cost_matrix[step][s_curr]) {
                        cost_matrix[step][s_curr] = total_cost;
                        volume_matrix[step][s_curr] = volume_matrix[step - 1][s_prev] + dig_vol;
                        traj_matrix[step][s_curr] = s_prev;
                    }
                }
            }
        }

        // 回溯最优轨迹
        int min_idx = 0;
        double min_cost = kInf;
        for (int s = 0; s < n_states; ++s) {
            if (cost_matrix[n_steps - 1][s] < min_cost) {
                min_cost = cost_matrix[n_steps - 1][s];
                min_idx = s;
            }
        }

        if (min_cost >= kInf) continue;

        // 回溯轨迹
        std::vector<JointAngles> trajectory(n_steps + 1);
        trajectory[n_steps] = end_point;

        int curr_state = min_idx;
        for (int step = n_steps - 1; step >= 0; --step) {
            int boom_idx = curr_state / (n_arm * n_bucket);
            int arm_idx = (curr_state / n_bucket) % n_arm;
            int bucket_idx = curr_state % n_bucket;

            trajectory[step].boom = boom_list[boom_idx];
            trajectory[step].arm = start_point.arm + arm_gain_list[arm_idx] * (step + 1) / n_steps;
            trajectory[step].bucket = start_point.bucket + bucket_gain_list[bucket_idx] * (step + 1) / n_steps;

            if (step > 0) {
                curr_state = traj_matrix[step][curr_state];
            }
        }

        volume_set[n] = volume_matrix[n_steps - 1][min_idx];
        trajectory_set[n] = trajectory;
    }

    // 选择最优结果
    int best_idx = 0;
    double best_volume = -kInf;
    for (size_t n = 0; n < volume_set.size(); ++n) {
        if (volume_set[n] < kInf && volume_set[n] > best_volume) {
            best_volume = volume_set[n];
            best_idx = static_cast<int>(n);
        }
    }

    if (best_volume > -kInf) {
        result.trajectory = trajectory_set[best_idx];
        result.total_volume = volume_set[best_idx];
        result.success = true;
    }

    return result;
}

std::vector<double> DigPlannerDP::generateEndPoints(double x_start, double x_end) const {
    std::vector<double> x_end_list;
    double x_end_1 = x_end;
    double x_end_2 = (x_start - x_end > 3.0) ? (x_start - 3.0) : x_end;

    for (int i = 0; i < 5; ++i) {
        double t = static_cast<double>(i) / 4.0;
        x_end_list.push_back(x_end_1 + t * (x_end_2 - x_end_1));
    }
    return x_end_list;
}

bool DigPlannerDP::generateStartEndPoints(const TerrainProfile& terrain,
                                          const ExcavatorFK& fk,
                                          double x_start,
                                          double x_end,
                                          double start_z_offset,
                                          double end_z_offset,
                                          double bucket_angle,
                                          JointAngles& start_point,
                                          JointAngles& end_point,
                                          double& z_start) const {
    double z_start_terrain;
    if (!terrain.getHeight(x_start, z_start_terrain)) return false;

    // 起点逆解，失败时自动后退
    double x_start_adj = x_start;
    for (int retry = 0; retry < 10; ++retry) {
        double z_terrain;
        if (!terrain.getHeight(x_start_adj, z_terrain)) return false;

        if (fk.inverseStart(x_start_adj, z_terrain + start_z_offset, bucket_angle, start_point)) {
            break;
        }
        x_start_adj -= 0.5;
        if (x_start_adj < 5.0) return false;
    }

    Point2d start_pos = fk.forwardKinematics(start_point);
    z_start = start_pos.z;

    // 终点逆解
    double z_end_terrain;
    if (!terrain.getHeight(x_end, z_end_terrain)) return false;

    if (!fk.inverseEnd(x_end, z_end_terrain + end_z_offset, end_point)) {
        return false;
    }

    // 保证终点 boom 角 >= 起点 boom 角 + 5
    if (end_point.boom < start_point.boom + 5.0) {
        end_point.boom = start_point.boom + 5.0;
    }

    return true;
}

std::vector<double> DigPlannerDP::generateBoomList(const JointAngles& start,
                                                   const JointAngles& end) const {
    std::vector<double> boom_list;

    // 下降段
    double boom_down_total = params_.boom_down_steps * params_.boom_down_diff;
    for (int i = 0; i <= static_cast<int>(params_.boom_down_steps); ++i) {
        double boom = start.boom - boom_down_total + i * params_.boom_down_diff;
        boom_list.push_back(boom);
    }

    // 上升段（去掉重复的起始点）
    double boom_up_total = end.boom - start.boom;
    for (int i = 1; i <= static_cast<int>(params_.boom_up_steps); ++i) {
        double t = static_cast<double>(i) / params_.boom_up_steps;
        double boom = start.boom + t * boom_up_total;
        boom_list.push_back(boom);
    }

    return boom_list;
}

void DigPlannerDP::generateCouplingGains(const JointAngles& start,
                                         const JointAngles& end,
                                         std::vector<double>& arm_gains,
                                         std::vector<double>& bucket_gains) const {
    double arm_range = end.arm - start.arm;
    double bucket_range = end.bucket - start.bucket;

    arm_gains.clear();
    bucket_gains.clear();

    for (int i = 0; i < params_.arm_gains; ++i) {
        double t = static_cast<double>(i) / (params_.arm_gains - 1);
        arm_gains.push_back(t * arm_range);
    }

    for (int i = 0; i < params_.bucket_gains; ++i) {
        double t = static_cast<double>(i) / (params_.bucket_gains - 1);
        bucket_gains.push_back(t * bucket_range);
    }
}

double DigPlannerDP::calcDigVolume(const JointAngles& before,
                                   const JointAngles& after,
                                   const TerrainProfile& terrain,
                                   const ExcavatorFK& fk) const {
    double total_volume = 0.0;
    int n_intervals = params_.volume_interval;

    // 离散化为多个子步
    for (int i = 0; i < n_intervals; ++i) {
        double t1 = static_cast<double>(i) / n_intervals;
        double t2 = static_cast<double>(i + 1) / n_intervals;

        JointAngles angles1, angles2;
        angles1.boom = before.boom + t1 * (after.boom - before.boom);
        angles1.arm = before.arm + t1 * (after.arm - before.arm);
        angles1.bucket = before.bucket + t1 * (after.bucket - before.bucket);

        angles2.boom = before.boom + t2 * (after.boom - before.boom);
        angles2.arm = before.arm + t2 * (after.arm - before.arm);
        angles2.bucket = before.bucket + t2 * (after.bucket - before.bucket);

        Point2d p1 = fk.forwardKinematics(angles1);
        Point2d p2 = fk.forwardKinematics(angles2);

        // 检查是否在地形以下
        double z_terrain1, z_terrain2;
        if (!terrain.getHeight(p1.x, z_terrain1) || !terrain.getHeight(p2.x, z_terrain2)) {
            return kInf;  // 超出地形范围
        }

        if (p1.z > z_terrain1 || p2.z > z_terrain2) {
            continue;  // 齿尖在地面以上，无贡献
        }

        // 检查方向：齿尖从远到近才计为正
        if (p2.x > p1.x) {
            continue;  // 齿尖向远处移动，土方量为负
        }

        // 梯形积分计算截面面积
        double dx = std::abs(p2.x - p1.x);
        double dz1 = z_terrain1 - p1.z;
        double dz2 = z_terrain2 - p2.z;
        double area = 0.5 * (dz1 + dz2) * dx;
        total_volume += area;
    }

    return total_volume;
}

double DigPlannerDP::calcTransitionCost(int step,
                                        const JointAngles& before,
                                        const JointAngles& after,
                                        double dig_volume,
                                        const JointAngles& start_point,
                                        const JointAngles& end_point,
                                        const TerrainProfile& terrain,
                                        const ExcavatorFK& fk,
                                        double z_start,
                                        double boom_up_range) const {
    Point2d after_pos = fk.forwardKinematics(after);

    double z_terrain;
    if (!terrain.getHeight(after_pos.x, z_terrain)) {
        return kInf;
    }

    // Cost 1: 挖掘量偏差（归一化）
    double x_diff = start_point.boom - end_point.boom;
    double z_diff = terrain.getHeight(end_point.boom, z_terrain) ? z_terrain - z_start : 0.0;
    double v_ref_local = (params_.v_ref - x_diff * z_diff / 2) / x_diff *
                         (start_point.boom - after.boom) +
                         z_diff / x_diff * std::pow(start_point.boom - after.boom, 2) / 2;
    double cost_volume = std::abs(v_ref_local - dig_volume) / std::max(params_.v_ref, 0.01);

    // Cost 2: 飞齿惩罚（硬约束）
    double cost_ground = (after_pos.z > z_terrain) ? 1.0 : 0.0;

    // Cost 3: 深度约束（硬约束）
    double cost_depth = (after_pos.z < z_terrain - params_.min_depth) ? 1.0 : 0.0;

    // Cost 4: 动臂变化率（软约束）
    double boom_delta = std::abs(after.boom - before.boom);
    double cost_boom_rate = boom_delta / params_.max_boom_step;

    // Cost 5: 联动约束（硬约束）
    double cost_coupling = 0.0;

    // 到达重心前禁止动臂下降
    if (before.boom + before.arm > -90.0 && after.boom < before.boom) {
        cost_coupling = 1.0;
    }

    // 铲斗上翻时禁止动臂下降
    if (after.boom < before.boom && after.bucket > before.bucket) {
        cost_coupling = 1.0;
    }

    // 加权求和
    double total_cost = params_.w_volume * cost_volume +
                        params_.w_ground * cost_ground +
                        params_.w_depth * cost_depth +
                        params_.w_boom_rate * cost_boom_rate +
                        params_.w_coupling * cost_coupling;

    return total_cost;
}

}  // namespace dig_trajectory_planner
