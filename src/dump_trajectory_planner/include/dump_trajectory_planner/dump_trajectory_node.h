#pragma once

/// @file dump_trajectory_node.h
/// @brief 卸载阶段关节空间轨迹规划与执行节点（在线规划版，简化 4 航路点）。
///
/// 采用基于航路点的在线规划方案：
///   - 激活时生成 WP1/WP4/WP5/WP6 四个关键航路点
///   - Seg0(WP1→WP4): boom + swing 双阶跃 + 高度门控 + PCHIP 平滑收尾的两阶段策略
///   - Seg1(WP4→WP5): PCHIP 插值，回转到卸载方位
///   - Seg2(WP5→WP6): PCHIP 插值，最终卸载姿态
///   - Seg0 Phase1 期间 swing 到位即检查铲斗高度门控，未达标则 boom boost 重试
///   - 全部插值统一使用 PCHIP（保形，无超调）
///
/// 触发协议：
///   /Sys_SeqAction          (std_msgs/Float64) data==4.0 触发卸载阶段（在线轨迹执行）
///   /Sys_SUn_FlagUnloadPlan (std_msgs/Float64) data==1.0 触发预校验（卸载点分布+航路点计算，
///                            只算不发轨迹；成功后发 /Sys_unloadPlanFinish=1）
///
/// 输入话题：
///   /truck_center_unloadpoint   (geometry_msgs/Quaternion) x/y=卡车近点(base系,m) w=RTK方位角(deg)
///   /truck_center_unloadpoint_2 (geometry_msgs/Quaternion) x/y=卡车远点(base系,m)，用于动态卸载点选取(可选)
///   /Sys_SUn_BucketNumber       (std_msgs/Float64)         data=当前装载斗数，决定选取最远/中间/最近卸载点
///   /joints_angle               (geometry_msgs/Quaternion) x/y/z=boom/arm/bucket (deg)
///   /heading2swing_topic        (std_msgs/Float32)         data=swing (deg)
///
/// 输出话题：
///   /RefDeviceTraj_Unload (geometry_msgs/Pose)   执行指令流（本类直发）
///     Orientation.x/y/z/w = swing/boom/arm/bucket (deg)
///     Position.x = 1.0 执行中 / 0.0 阶段结束
///   /Sys_RUn_UnloadPointSequence (std_msgs/Float32MultiArray)  离线轨迹序列（本类直发，
///     适配原程序节点，后续可能移除）：预校验成功后发布 2000x5 列优先一维数组（latched）
///   其余状态/统计/适配类话题（/Swing_topic、/RealBktPosXYZ、/UPFeasible、
///   /loadTraj_*、/Sys_RUn_UnloadEndAngle、/Sys_unloadPlanFinish、
///   /Sys_RUn_FlagUnloadExcuteFinish、/UT_PlanningPulse_topic）统一由
///   StatusReporter 发布，协议详见 status_reporter.h。

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

#include <kinematics/kinematics.hpp>

#include "dump_trajectory_planner/dump_feasibility.h"
#include "dump_trajectory_planner/dump_visualizer.h"
#include "dump_trajectory_planner/status_reporter.h"
#include "dump_trajectory_planner/velocity_estimator.h"
#include "dump_trajectory_planner/waypoint_generator.h"

namespace dump_trajectory_planner {

/// 卸载执行阶段（在线规划状态机）
enum class DumpPhase {
  kIdle,              // 空闲等待
  kPlanNextSegment,   // 在线规划当前段局部轨迹
  kExecutingSegment,  // 逐帧发布当前段轨迹
  kWaitBucketClear,   // 等待铲斗高于卡车上表面（段间门控）
  kGateBoost,         // 门控超时后自动 boom 提升小段
  kDone               // 完成
};

class DumpTrajectoryNode {
 public:
  DumpTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  // ==================== 初始化 ====================
  void LoadParams();
  void SetupTopics();

  // ==================== 订阅回调 ====================
  void ActivateCallback(const std_msgs::Float64::ConstPtr& msg);
  void PlanUnloadPointCallback(const std_msgs::Float64::ConstPtr& msg);
  void TruckPoseCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void TruckPoseFarCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void BucketNumberCallback(const std_msgs::Float64::ConstPtr& msg);
  void JointsAngleCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void SwingCallback(const std_msgs::Float32::ConstPtr& msg);

  // ==================== 定时器 ====================
  void TimerCallback(const ros::TimerEvent& event);

  // ==================== 在线规划方法 ====================
  /// @brief 为当前段生成局部轨迹（基于实际关节角）
  void PlanCurrentSegment();

  /// 段推进判定结果
  enum class SegResult {
    kContinue,  // 继续执行当前段
    kComplete,  // 当前段完成，推进到下一段
    kEnterGate  // 段超时且铲斗高度门控未达标 → 转入高度门控（boost 重试/耗尽终止）
  };

  /// @brief 判定当前段推进结果（反馈驱动 + 超时兜底）
  SegResult CheckSegmentProgress();

  /// @brief 按段索引计算“归一化段误差” = max_j(err_j/tol_j)（逐关节达标判据）
  /// <=1.0 即该段所有主导关节各自达标；seg0=swing+boom, seg1=swing, seg2=arm+bucket
  double ComputeSegError(int seg_idx) const;

  /// @brief 卡车框顶高（base 系, m）= 卡车 RTK 中心高 center_z − truck_height_offset
  /// （与 waypoint_generator 中 WP4 框高计算逻辑一致，动态随 RTK 高度变化）
  double TruckTopHeight() const;

  /// @brief 检查铲斗是否高于卡车框顶（齿尖高 > 框顶高 + 裕量）
  bool IsBucketAboveTruck() const;

  /// @brief 启动门控自动 boom 提升小段（boom 抬升 + arm 联动保姿态）
  /// @return false=boom 已达上限无法提升，调用方应终止卸载
  bool StartGateBoost();

  /// @brief 推进到下一段或完成
  void AdvanceToNextSegment();

  /// @brief 计算各航路点的目标切线（Catmull-Rom，门控点/端点为0）
  void ComputeWaypointTangents();

  /// @brief 判断某航路点是否为“飞越点”（内部非门控点）。
  /// 飞越点处指令位置/速度保持连续、不停稳（整段卸载平滑过渡）；
  /// 首点、门控点、末点为“停止点”（实时到位 + 确认保持，可停）。
  bool IsFlythroughWaypoint(int wp_idx) const;

  /// @brief 动态卸载点选取：在近点与远点之间生成候选，验证可达性，按斗数选取
  /// @param[out] fr 选中点的可行性结果（含 unload_joint）
  /// @return true=选取成功（unload_point_ 已更新）；false=无候选可达
  bool SelectUnloadPoint(FeasibilityResult& fr);

  /// @brief 预校验公共流程：输入校验→卸载点选取→可达性→航路点生成→限位校验→
  ///        统计量/终点角发布→段时长/切线计算→可视化。只算不发执行轨迹。
  /// @return true=预校验通过（waypoints_ 已缓存）；false=任一环节失败
  bool PlanWaypoints();

  /// @brief 发布离线轨迹序列 /Sys_RUn_UnloadPointSequence（适配原程序节点，后续可能移除）。
  /// 2000x5 列优先 reshape 成一维 Float32MultiArray：
  ///   row0 = 计算成功标志（5 列全 1）；row1..1999 = 轨迹（前4列 swing/boom/arm/bucket deg，
  ///   第5列段标志）；航路点 PCHIP 插值到实际时长，不足补末点。
  void PublishOfflineSequence();

  // ==================== 工具方法 ====================
  bool CheckInputsValid();

  /// @brief 校验航路点关节限位，超限则 clamp
  /// @return true=在限位内，或修正量 ≤ waypoint_clamp_abort_deg_（已 clamp 容错）；
  ///         false=修正量超过阈值（上游航路点不可信），调用方应终止激活
  bool ValidateWaypointsJointLimits();

  /// @brief 发布执行指令帧（/RefDeviceTraj_Unload，含 swing 回包与首帧零速头）
  void PublishTrajectoryFrame(const kinematics::JointState& q, bool running);
  void StopExecution();

  /// @brief 由当前关节角 FK 计算铲斗齿尖坐标，更新门控高度并上报 /RealBktPosXYZ
  /// 需 swing 与 joints 两路反馈均有效（FK 需要全部 4 关节角）
  void UpdateBucketPos();

  // ==================== 成员变量 ====================
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  // ---- 运动学求解器 ----
  std::unique_ptr<kinematics::KinematicsSolver> solver_;

  // ---- 组件 ----
  StatusReporter reporter_;       // 状态/统计/适配类话题统一出口
  VelocityEstimator vel_estimator_;  // 双路关节速度估计（段间平滑）
  DumpVisualizer visualizer_;     // RViz 联调可视化（卡车包络/臂架/回转圆/轨迹/状态文本）

  // 订阅
  ros::Subscriber sub_activate_;
  ros::Subscriber sub_plan_unload_point_;  // /Sys_SUn_FlagUnloadPlan（预校验）
  ros::Subscriber sub_truck_pose_;
  ros::Subscriber sub_truck_pose_far_;   // /truck_center_unloadpoint_2（卡车远点）
  ros::Subscriber sub_bucket_number_;    // /Sys_SUn_BucketNumber（装载斗数）
  ros::Subscriber sub_joints_angle_;
  ros::Subscriber sub_swing_;

  // 发布（仅执行指令流，其余见 StatusReporter）
  ros::Publisher pub_trajectory_;   // /RefDeviceTraj_Unload
  ros::Publisher pub_unload_seq_;   // /Sys_RUn_UnloadPointSequence（离线序列，适配原程序）

  // 定时器
  ros::Timer timer_;

  // ---- 参数 ----
  FeasibilityParams feasibility_params_;
  WaypointParams waypoint_params_;
  double publish_rate_hz_ = 10.0;
  double total_timeout_sec_ = 30.0;

  // 段完成判定：各段独立容差（仅检查主导关节，deg）
  double seg0_swing_tolerance_deg_ = 5.0;       // WP1→WP4：回转
  double seg0_boom_tolerance_deg_ = 5.0;        // WP1→WP4：动臂
  bool seg0_height_gate_ = true;                // WP1→WP4：是否同时要求铲斗高于卡车上表面
  double seg1_swing_tolerance_deg_ = 5.0;       // WP4→WP5：仅判断回转
  double seg2_arm_tolerance_deg_ = 10.0;        // WP5→WP6：斗杆
  double seg2_bucket_tolerance_deg_ = 20.0;     // WP5→WP6：铲斗
  double segment_timeout_factor_ = 2.0;    // 段超时倍率（段时间 × 该值）
  int segment_confirm_frames_ = 3;         // 段完成需连续确认帧数（防抖）
  int seg0_confirm_frames_ = 8;            // Seg0 专用确认帧数（覆盖 boom 阶跃后臂架液压振荡模态）
  double bucket_clear_margin_m_ = 0.3;     // 铲斗需超出卡车上表面的裕量 (m)
  int bucket_clear_seg_idx_ = 0;           // 哪一段完成后触发铲斗高度门控（默认段0即WP1→WP4完成后）
  double waypoint_clamp_abort_deg_ = 5.0;  // 限位校验 clamp 修正量超过该值则终止激活 (deg)
  double offline_seq_dt_sec_ = 0.01;       // 离线轨迹序列采样步长 (s)

  // 门控自动提升参数
  double gate_timeout_sec_ = 4.0;          // 门控等待超时 (s)，超时自动追加 boom 提升
  double gate_boost_boom_deg_ = 3.0;       // 每次自动提升的 boom 角度 (deg)
  double gate_boost_tolerance_deg_ = 1.0;  // boost 完成判定容差 (deg)，必须 < gate_boost_boom_deg_，否则首帧即误判完成
  int gate_max_boost_count_ = 3;           // 最大自动提升次数，超过则终止卸载

  // ---- Seg0 两阶段策略参数 ----
  double seg0_switch_threshold_deg_ = 3.0; // swing 到达 Phase1 目标 ±阈值(deg)后检查高度门控，通过则切 Phase2
  double seg0_phase1_swing_dps_ = 15.0;    // Phase1 复合回转指令速率(deg/s)：boom 随动举升期间 swing 向 WP4 回转的速率（独立可调，实际受泵流量限制会滞后）
  double seg0_min_step_factor_ = 2.0;      // 小行程退化因子：boom 抬升量 < 该值×切换阈值 时退化为单段 PCHIP
  double seg0_swing_offset_deg_ = 15.0;    // swing Phase1 阶跃偏移角(deg)：目标 = WP4.swing − 偏移（沿回转方向）
  double bucket_attitude_target_deg_ = -180.0;  // 铲斗姿态角目标（boom+arm+bucket），超过时调整 bucket

  // ---- 状态机 ----
  DumpPhase phase_ = DumpPhase::kIdle;

  // ---- 全局计时 ----
  ros::Time execution_start_time_;

  // ---- 段执行追踪 ----
  int current_seg_idx_ = 0;                // 当前执行段索引（0-based）
  int total_segments_ = 0;                 // 总段数
  ros::Time seg_start_time_;               // 当前段开始时间
  std::vector<double> segment_times_;      // 各段预期时长 (s)
  double seg_planned_time_ = 0.0;          // 当前段在线实际规划时长 (s)，超时判据取其与标称段时长的较大者
  int confirm_count_ = 0;                  // 段完成连续确认计数（防抖）

  // ---- 航路点缓存 ----
  std::vector<kinematics::JointState> waypoints_;  // WP1,WP4,WP5,WP6
  std::vector<kinematics::JointState> wp_tangents_; // 各航路点目标切线 (rad/s)

  // ---- 当前段局部轨迹 ----
  std::vector<kinematics::JointState> seg_trajectory_;
  size_t seg_frame_idx_ = 0;

  // ---- Seg0 两阶段状态 ----
  bool seg0_phase1_complete_ = false;      // Phase 1(回转+boom随动) 是否已完成，进入 Phase 2(PCHIP)
  kinematics::JointState seg0_phase1_cmd_; // Phase 1 指令：boom=随swing进度插值抬升, arm 保持, swing=阶跃到"卡车附近"(WP4−offset), bucket 维持姿态
  double seg0_phase1_timeout_sec_ = 0.0;   // Phase 1 独立超时(s)=行程/wp2_vel_boom_dps×factor；超时转入高度门控 boost 重试
  // Phase1 boom 随动插值状态（progress 单调不减，超调回摆不回退指令）
  double seg0_p1_sw_start_rad_ = 0.0;      // Phase1 swing 起点（周期展开后）
  double seg0_p1_boom_start_rad_ = 0.0;    // boom 插值起点（=WP1.boom）
  double seg0_p1_boom_end_rad_ = 0.0;      // boom 插值终点（=段终点航路点.boom）
  double seg0_p1_progress_ = 0.0;          // swing 回转进度 [0,1]（单调锁定）

  // ---- 门控自动提升状态 ----
  ros::Time gate_start_time_;              // 进入门控（或上次提升完成）的时刻
  int gate_boost_count_ = 0;               // 本次卸载已自动提升次数
  kinematics::JointState boost_cmd_;       // 提升小段当前指令位姿
  kinematics::JointState boost_target_;    // 提升小段目标位姿
  ros::Time boost_start_time_;             // 提升小段开始时刻
  kinematics::JointState gate_hold_cmd_;   // 进入门控时锁存的指令位姿（等待期间恒定发布，防止液压保压沉降被逐帧追认）

  // ---- 输入缓存 ----
  Point3 unload_point_;
  bool unload_valid_ = false;

  // ---- 动态卸载点选取 ----
  Point3 unload_point_far_;                // 卡车远点（/truck_center_unloadpoint_2）
  bool unload_far_valid_ = false;
  int bucket_number_ = 0;                  // 当前装载斗数（/Sys_SUn_BucketNumber）
  bool bucket_number_valid_ = false;
  bool unload_selector_enable_ = true;     // 是否启用动态选取（false 或未收到远点时回退单点）
  int unload_candidates_count_ = 21;       // 候选点数量（近点到远点线性插值）
  int unload_bucket_far_count_ = 3;        // 前 N 斗选最远可达点
  int unload_bucket_mid_count_ = 3;        // 接下来 N 斗选中间可达点
  int unload_bucket_near_count_ = 2;       // 最后 N 斗选最近可达点，超出默认选中间

  // 卡车位姿：从 /truck_center_unloadpoint 获取
  // x/y=卡车中心(base系,m)，w=RTK方位角α(deg)
  // 卸载点取卡车中心 x/y，z=0
  // 内部存 base 系 yaw β = (180 − α) mod 360 (rad)
  TruckPose truck_pose_;
  bool truck_pose_valid_ = false;

  // 实时关节角（rad）：swing 独立话题，boom/arm/bucket 合并话题
  kinematics::JointState current_joint_;
  bool swing_valid_ = false;
  bool joints_valid_ = false;

  // 铲斗齿尖位置（由关节角 FK 实时计算，见 UpdateBucketPos）
  double bucket_height_ = 0.0;             // 齿尖 z (m)，门控判据
  bool bucket_pos_valid_ = false;
};

}  // namespace dump_trajectory_planner
