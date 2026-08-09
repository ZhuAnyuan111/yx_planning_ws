#pragma once

/// @file dig_planner_dp.h
/// @brief 基于动态规划的挖掘轨迹优化器（纯 C++，无 ROS 依赖）。
///
/// 核心改进（相比 dp.m）：
/// - 三关节联合优化：boom + arm/bucket 耦合参数搜索
/// - 归一化代价函数：硬/软约束分离，量级统一
/// - 全约束生效：所有定义的约束都参与优化
/// - 高精度体积计算：Interval=10~20，梯形积分
///
/// 算法流程：
/// 1. 输入地形、起终点、参考挖掘量
/// 2. 逆解生成起终点关节角
/// 3. 构建 boom 状态列表 + arm/bucket 耦合增益列表
/// 4. DP 递推搜索最优状态序列
/// 5. 回溯最优轨迹
/// 6. 输出关节角序列（10个点）

#include <vector>

#include "dig_trajectory_planner/excavator_fk.h"
#include "dig_trajectory_planner/terrain_profile.h"

namespace dig_trajectory_planner {

/// DP 规划器参数
struct DigPlannerParams {
    // 状态空间
    int boom_states = 16;       ///< 动臂离散状态数
    int arm_gains = 4;          ///< 斗杆耦合增益离散数
    int bucket_gains = 4;       ///< 铲斗耦合增益离散数
    int dig_steps = 9;          ///< DP 递推步数

    // 代价权重
    double w_volume = 1.0;      ///< 挖掘量偏差权重
    double w_ground = 10.0;     ///< 飞齿惩罚权重（硬约束）
    double w_depth = 5.0;       ///< 深度约束权重（硬约束）
    double w_boom_rate = 0.5;   ///< 动臂变化率权重
    double w_coupling = 2.0;    ///< 联动约束权重（硬约束）

    // 体积计算
    int volume_interval = 15;   ///< 挖掘量计算子步数

    // 地形
    double terrain_resolution = 0.05;  ///< 地形分辨率 (m)

    // 参考挖掘量
    double v_ref = 0.5;         ///< 参考挖掘量 (m^2, 2D截面面积)

    // 动臂状态生成
    double boom_down_steps = 3;      ///< 动臂下降步数
    double boom_down_diff = 2.0;     ///< 动臂下降每步角度 (deg)
    double boom_up_steps = 12;       ///< 动臂上升步数

    // 约束参数
    double min_depth = 0.1;          ///< 最小挖掘深度 (m)
    double max_boom_step = 4.0;      ///< 动臂单步最大变化 (deg)
    int no_retreat_steps = 2;        ///< 前N步禁止动臂后退
};

/// DP 规划器结果
struct DigPlannerResult {
    std::vector<JointAngles> trajectory;  ///< 最优轨迹（dig_steps + 1 个点）
    double total_volume = 0.0;            ///< 总挖掘量 (m^2)
    double total_cost = 0.0;              ///< 总代价
    bool success = false;                 ///< 是否成功
};

/// 基于动态规划的挖掘轨迹优化器
class DigPlannerDP {
public:
    /// 构造函数
    explicit DigPlannerDP(const DigPlannerParams& params = DigPlannerParams{});

    /// 规划挖掘轨迹
    /// @param terrain 地形剖面
    /// @param fk 运动学模型
    /// @param x_start 起点 X 坐标
    /// @param x_end 终点 X 坐标
    /// @param start_z_offset 起点 Z 偏移（相对于地形）
    /// @param end_z_offset 终点 Z 偏移（相对于地形）
    /// @param bucket_angle 铲斗角 (deg)
    /// @return 规划结果
    DigPlannerResult plan(const TerrainProfile& terrain,
                          const ExcavatorFK& fk,
                          double x_start,
                          double x_end,
                          double start_z_offset,
                          double end_z_offset,
                          double bucket_angle) const;

    /// 获取参数
    const DigPlannerParams& getParams() const { return params_; }

    /// 设置参数
    void setParams(const DigPlannerParams& params) { params_ = params; }

private:
    /// 生成候选终点列表
    std::vector<double> generateEndPoints(double x_start, double x_end) const;

    /// 生成起终点关节角
    bool generateStartEndPoints(const TerrainProfile& terrain,
                                const ExcavatorFK& fk,
                                double x_start,
                                double x_end,
                                double start_z_offset,
                                double end_z_offset,
                                double bucket_angle,
                                JointAngles& start_point,
                                JointAngles& end_point,
                                double& z_start) const;

    /// 生成动臂角度列表
    std::vector<double> generateBoomList(const JointAngles& start,
                                         const JointAngles& end) const;

    /// 生成 arm/bucket 耦合增益列表
    void generateCouplingGains(const JointAngles& start,
                               const JointAngles& end,
                               std::vector<double>& arm_gains,
                               std::vector<double>& bucket_gains) const;

    /// 计算单次转移的挖掘量
    double calcDigVolume(const JointAngles& before,
                         const JointAngles& after,
                         const TerrainProfile& terrain,
                         const ExcavatorFK& fk) const;

    /// 计算单次转移的代价
    double calcTransitionCost(int step,
                              const JointAngles& before,
                              const JointAngles& after,
                              double dig_volume,
                              const JointAngles& start_point,
                              const JointAngles& end_point,
                              const TerrainProfile& terrain,
                              const ExcavatorFK& fk,
                              double z_start,
                              double boom_up_range) const;

    DigPlannerParams params_;
};

}  // namespace dig_trajectory_planner
