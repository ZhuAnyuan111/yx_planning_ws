#pragma once

/// @file waypoint_generator.h
/// @brief 卸载轨迹关键航路点生成器（关节空间，照 MATLAB 方案）。
///
/// 6 关键航路点（关节角，rad）：
///   WP1 —— 挖掘终止点（起点，姿态角修正 bucket）
///   WP2 —— 动臂提升点（swing/arm/bucket 保持，仅 boom 提升；提升幅度由 swing 差插值决定）
///   WP3 —— 中间过渡点（swing 走一半或最小步长；boom/arm 按系数混合）
///   WP4 —— 厢上过渡点（xy 由 ComputeMiddleUpXY 决定，铰接点高度=truck_top+bias，铲斗长=0 IK 求解）
///   WP5 —— 除铲斗到位（swing 到位，boom/arm 按 P5 系数混合到 unload；铲斗未翻）
///   WP6 —— 终点（unload_joint 完整）
///
/// 段时间 t_array（绝对时刻，t1=0）：由各段主导关节速度决定，见 .cpp 实现。

#include <string>
#include <vector>

#include <kinematics/kinematics.hpp>

#include "dump_trajectory_planner/dump_feasibility.h"  // Point3

namespace dump_trajectory_planner {

/// 卡车位姿（挖机 base 系）
/// - center_x/center_y/center_z : 卡车中心坐标（m），z 为 RTK 高度
/// - rtk_heading_deg   : RTK 方位角 α（deg，北0顺时针）
/// - yaw_rad           : 卡车航向角 β（rad），来自 RTK 方位角经
///                       β = (180° + α_rtk) mod 360° 转换
struct TruckPose {
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;    // RTK 高度，用于计算卡车框高度（center_z - 1.0）
  double rtk_heading_deg = 0.0;  // RTK 方位角 α（deg）
  double yaw_rad = 0.0;
};

/// 航路点生成参数（角度类字段单位统一为 deg，速度单位 deg/s）
struct WaypointParams {
  // 姿态角上限（deg）：boom+arm+bucket 之和的最大姿态
  double attitude_angle_deg = 25.0;

  // WP4 厢上过渡点：铰接点高度 = truck_top_height + wp4_height_bias
  // （wp4_height_bias 为斗杆-铲斗铰接点相对卡车表面的高度偏置）
  double wp4_height_bias = 1.0;

  // 卡车框高度偏移（m）：卡车框高度 = truck_center_z - truck_height_offset
  double truck_height_offset = 1.0;

  // WP4 IK 求解的铲斗关节角（deg），用于 swing_center_inverse_by_bucket_angle
  double wp4_bucket_angle_deg = 25.0;

  // WP3 swing 最小步长（deg）：|median - swing4| ≤ 该值时改为 swing4 ± 该值
  double wp3_swing_step_deg = 5.0;

  // WP2 boom 提升幅度插值（相对 swing 差）
  double p3_min_z0_swing_deg = 10.0;    // swing 差 ≤ 此值 → boom 直接提到 WP4 高度
  double p3_max_z0_swing_deg = 40.0;    // swing 差 ≥ 此值 → boom 只做最小提升
  double max_boom_lift24_deg = 20.0;    // WP2 与 WP4 的 boom 最大差值

  // WP3 swing 速度插值（相对 swing2→swing3 差）
  double p3_min_swing_deg = 15.0;
  double p3_max_swing_deg = 60.0;
  double min_p3_swing_vel_dps = 5.0;
  double max_p3_swing_vel_dps = 15.0;

  // WP3 混合系数
  double p3_boom_cof = 0.5;  // boom3 = cof·boom2 + (1-cof)·boom4
  double p3_arm_cof = 0.5;   // arm3  = cof·arm2  + (1-cof)·arm4

  // WP5 混合系数
  double p5_boom_cof = 0.5;
  double p5_arm_cof = 0.5;

  // 各段主导关节速度（deg/s）
  double wp2_vel_boom_dps = 8.0;
  double wp4_vel_swing_dps = 15.0;
  double wp4_vel_arm_dps = 10.0;
  double wp5_vel_boom_dps = 8.0;
  double wp5_vel_arm_dps = 10.0;
  double wp5_vel_bkt_dps = 15.0;

  // 铲斗关节限位（deg）
  double bucket_limit_lower_deg = -170.0;
  double bucket_limit_upper_deg = 30.0;

  // ---- WP4 入厢点（回转圆策略）----
  double truck_box_length = 5.0;   // 卡车厢长 L（m）
  double truck_box_width = 2.6;    // 卡车厢宽 W（m）
  double wp4_R_coff = 0.5;         // 回转圆半径混合系数：R = coff·R_dig_end + (1-coff)·R_unload
};

/// 生成结果
struct DumpWaypointResult {
  bool success = false;
  std::string message;
  std::vector<kinematics::JointState> waypoints;  // WP1..WP6（rad）
  std::vector<double> t_array;                    // 绝对时刻 t1..t6（秒），t1=0
};

/// 生成 6 关键航路点及段时间
/// @param start_joint  挖掘终止时的实时关节角（rad）
/// @param unload_joint 卸载点关节角（rad），来自 CheckReachable
/// @param unload_point 卸载点笛卡尔（base 系，m）
/// @param truck_pose   卡车位姿（base 系），用于 WP4 入厢点计算
/// @param solver       运动学求解器（用于 dig_end FK 与 WP4 IK）
/// @param params       生成参数
DumpWaypointResult GenerateDumpWaypoints(
    const kinematics::JointState& start_joint,
    const kinematics::JointState& unload_joint,
    const Point3& unload_point,
    const TruckPose& truck_pose,
    const kinematics::KinematicsSolver& solver,
    const WaypointParams& params);

/// 计算 WP4 入厢点 xy（base 系），照 MATLAB `calc_entry_point_circle_strategy`：
/// 1. 回转圆半径 R = coff·|dig_end| + (1-coff)·|unload|；
/// 2. 将回转圆与卡车厢矩形 4 条边求交点；
/// 3. 有交点 → 沿 rot_dir 方向从 dig_end swing 出发角度差最小的一个；
/// 4. 无交点 → 回退到矩形最近边缘的圆上投影点。
///
/// rot_dir 由 dig_end swing 到 unload swing 的最短路径自动判定。
/// 当 R 过小或退化时，回退为 unload_point 的 xy。
void ComputeMiddleUpXY(const Point3& unload_point,
                       const kinematics::JointState& start_joint,
                       const kinematics::JointState& unload_joint,
                       const TruckPose& truck_pose,
                       const kinematics::KinematicsSolver& solver,
                       const WaypointParams& params,
                       double& out_x,
                       double& out_y);

/// 计算 WP2→WP3 段回转速度（deg/s）：回转幅度 |swing3-swing2| 越小 → 速度越慢
/// （小回转慢速起步，抑制满载起步激励回转台欠阻尼振荡）。
/// 供生成器 t3_dur 与在线 seg1 段时间共用，保证两处速度基准一致。
double Wp3SwingVelDps(const WaypointParams& params, double swing_diff_deg);

}  // namespace dump_trajectory_planner
