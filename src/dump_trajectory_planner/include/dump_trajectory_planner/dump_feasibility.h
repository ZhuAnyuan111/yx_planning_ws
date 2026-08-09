#pragma once

/// @file dump_feasibility.h
/// @brief 卸载点可行性检查（真实 IK 判定 + 可选的包络粗筛）。
///
/// 采用 kinematics::KinematicsSolver::swing_center_inverse_by_bucket_attitude
/// 求解卸载点在给定铲斗姿态角下的关节角；若求解成功且通过关节限位则视为可达。
/// 求解得到的关节角同时作为后续航路点生成的终点（unload_joint）。

#include <string>

#include <kinematics/kinematics.hpp>

namespace dump_trajectory_planner {

/// 3 维笛卡尔点（base 系）
struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// 卸载点可行性检查参数
struct FeasibilityParams {
  // ---- 可选包络粗筛（IK 前的快速过滤） ----
  bool enable_envelope_prefilter = true;
  double reach_min = 3.0;         // 水平半径最小值 (m)
  double reach_max = 12.0;        // 水平半径最大值 (m)
  double dump_height_min = 1.0;   // 卸载点最低高度 (m)
  double dump_height_max = 7.5;   // 卸载点最高高度 (m)

  // ---- IK 姿态角 ----
  double bucket_attitude_deg = -90.0;  // 卸载姿态（deg），负值表示齿尖朝下
};

/// 检查结果
struct FeasibilityResult {
  bool reachable = false;
  std::string reason;
  kinematics::JointState unload_joint;  // 卸载点关节角（rad），仅在 reachable=true 时有效
};

/// 卸载点可行性检查
/// @param unload_point 卸载点笛卡尔位置 (base 系)
/// @param params       可行性参数
/// @param solver       运动学求解器
FeasibilityResult CheckReachable(const Point3& unload_point,
                                 const FeasibilityParams& params,
                                 const kinematics::KinematicsSolver& solver);

}  // namespace dump_trajectory_planner
