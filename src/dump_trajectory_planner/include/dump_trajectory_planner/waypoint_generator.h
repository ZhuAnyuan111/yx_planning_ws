#pragma once

/// @file waypoint_generator.h
/// @brief 卸载轨迹关键航路点生成器（关节空间，简化 4 航路点版）。
///
/// 4 关键航路点（关节角，rad）：
///   WP1 —— 挖掘终止点（起点，姿态角修正 bucket）
///   WP4 —— 厢上过渡点（xy 由 ComputeMiddleUpXY 决定，铰接点高度=truck_top+bias，IK 求解）
///   WP5 —— 卸载过渡点（swing 到位，boom 保持=WP4.boom 不下降，arm 按 P5 系数混合到 unload）
///   WP6 —— 终点（unload_joint 完整）
///
/// WP1→WP4 由节点端的两阶段策略驱动（boom 阶跃 + PCHIP），无需中间航路点。
/// 段时间 t_array（绝对时刻，t1=0）：由各段主导关节速度决定，见 .cpp 实现。

#include <string>
#include <vector>

#include <kinematics/kinematics.hpp>

#include "dump_trajectory_planner/dump_feasibility.h"  // Point3

namespace dump_trajectory_planner {

/// 卡车位姿（挖机 base 系）
/// - center_x/center_y/center_z : 卡车中心坐标（m），z 为 RTK 高度
/// - rtk_heading_deg   : RTK 方位角 α（deg，北0顺时针）
///   - yaw_rad           : 卡车航向角 β（rad），来自 RTK 方位角经
///                       β = (180° − α_rtk) mod 360° 转换
///                       （北180/东90/南0/西270 逆时针增大，与
///                        truck_dump_planner 6 方向闭环验证一致）
struct TruckPose {
  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;    // RTK 高度，用于计算卡车框高度（center_z - 1.0）
  double rtk_heading_deg = 0.0;  // RTK 方位角 α（deg）
  double yaw_rad = 0.0;
};

/// 航路点生成参数（角度类字段单位统一为 deg，速度单位 deg/s）
struct WaypointParams {
  // 卸载途中维持的铲斗姿态角目标（boom+arm+bucket，有符号 deg）。
  // 单一真源 = online_plan/bucket_attitude_target_deg（节点 LoadParams 里赋值到此），
  // 用于 AdjustBucketByAttitude 修正 WP1/WP4/WP5 的 bucket。
  double bucket_attitude_target_deg = -180.0;

  // WP4 厢上过渡点：铰接点高度 = truck_top_height + wp4_height_bias
  // （wp4_height_bias 为斗杆-铲斗铰接点相对卡车表面的高度偏置）
  double wp4_height_bias = 1.0;

  // 卡车框高度偏移（m）：卡车框高度 = truck_center_z - truck_height_offset
  double truck_height_offset = 1.0;

  // WP4 IK 求解的铲斗关节角（deg），用于 swing_center_inverse_by_bucket_angle
  double wp4_bucket_angle_deg = 25.0;

  // WP5 混合系数（p5_boom_cof 已停用：WP5.boom 恒取 WP4.boom，Seg1 boom 保持）
  double p5_boom_cof = 0.5;
  double p5_arm_cof = 0.5;

  // 各段主导关节速度（deg/s）——命名按"段+关节"语义（旧 wp2/wp4/wp5_ 编号已废弃）：
  double seg0_boom_vel_dps = 8.0;      // Seg0(WP1→WP4) boom 抬升速率（亦用于 Phase1 超时估算）
  double swing_vel_dps = 15.0;         // swing 回转速率（Seg0 与 Seg1 共用）
  double seg0_arm_vel_dps = 10.0;      // Seg0 Phase1 超时估算用的 arm 速率
  double unload_boom_vel_dps = 8.0;    // Seg1+Seg2 boom 速率
  double unload_arm_vel_dps = 10.0;    // Seg1+Seg2 arm 速率
  double unload_bucket_vel_dps = 15.0; // Seg2 bucket 速率

  // 铲斗关节限位（deg）
  double bucket_limit_lower_deg = -170.0;
  double bucket_limit_upper_deg = 30.0;

  // ---- WP4 入厢点（回转圆策略）----
  double truck_box_length = 5.0;   // 卡车厢长 L（m）
  double truck_box_width = 2.6;    // 卡车厢宽 W（m）
  double wp4_R_coff = 0.5;         // 回转圆半径混合系数：R = coff·R_dig_end + (1-coff)·R_unload

  // ---- 铲斗宽度碰撞（配置空间外扩 / Minkowski 膨胀卡车框）----
  // 卡车框矩形按 outset = 0.5·bucket_width + bucket_wall_clearance 向外膨胀后再与回转圆求交：
  // 把有宽度的铲斗退化为其宽度中心线（回转圆上的铰接点），障碍物（卡车框）膨胀铲斗半宽，
  // 于是入厢点 = 铲斗边缘刚好接触真实卡车框时的中心位置（而非把铲斗当点、边缘会超框刮帮）。
  double bucket_width = 2.0;          // 铲斗宽度 W_b（m），YAML 可调
  double bucket_wall_clearance = 0.1; // 额外安全间隙（m），叠加在 W_b/2 之上；=0 即边缘贴合框线
};

/// 回转圆入厢点计算的中间量（供可视化/调试）。
/// 由 ComputeMiddleUpXY 通过可选出参回填；不改变 xy 计算行为本身。
struct EntryPointDebug {
  bool valid = false;   // ComputeMiddleUpXY 是否执行过
  int flag = 0;         // 0=退化(R 过小); 1=圆-矩形交点; 2=fallback 最近边投影
  double coff = 0.0;    // wp4_R_coff
  double R1 = 0.0;      // |dig_end| 半径（m）
  double R2 = 0.0;      // |unload| 半径（m）
  double R = 0.0;       // 混合回转圆半径（m）
  int rot_dir = 0;      // +1=CCW, -1=CW（由 dig_end→unload 最短路径判定）
  // 配置空间外扩：卡车框每边实际外扩量（m）= 0.5·bucket_width + bucket_wall_clearance；
  // 供可视化画"膨胀禁区"外框（回转圆与之外扩求交，交点=铲斗边缘接触真实框的中心位置）
  double outset = 0.0;
  // dig_end / unload 参考点（base 系 xy）与其 swing（deg）
  double dig_end_x = 0.0, dig_end_y = 0.0, dig_end_swing = 0.0;
  double unload_x = 0.0, unload_y = 0.0, unload_swing = 0.0;
  // 全部圆-矩形候选交点（base 系，z=0）+ 各自 swing（deg）+ 沿 rot_dir 的角度差（deg）
  std::vector<Point3> candidates;
  std::vector<double> cand_swing;
  std::vector<double> cand_diff;
  int chosen_idx = -1;  // 选中的候选交点下标（flag==1 时有效）
  // 最终入厢点（base 系 xy）+ swing + 角度差
  double out_x = 0.0, out_y = 0.0, out_swing = 0.0, out_diff = 0.0;
  // fallback（flag==2）：矩形最近边点（base 系），投影终点即 out_x/out_y
  double fb_edge_x = 0.0, fb_edge_y = 0.0;
};

/// 生成结果
struct DumpWaypointResult {
  bool success = false;
  std::string message;
  std::vector<kinematics::JointState> waypoints;  // WP1,WP4,WP5,WP6（rad）
  std::vector<double> t_array;                    // 绝对时刻 t1..t4（秒），t1=0
  EntryPointDebug entry_dbg;                      // WP4 回转圆策略中间量（调试用）
};

/// 生成 4 关键航路点及段时间
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
                       double& out_y,
                       EntryPointDebug* dbg = nullptr);

}  // namespace dump_trajectory_planner
