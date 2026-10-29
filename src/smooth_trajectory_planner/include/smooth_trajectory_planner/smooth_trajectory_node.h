#pragma once

/// @file smooth_trajectory_node.h
/// @brief 车厢抹平轨迹规划与分阶段执行节点类声明。
///
/// 功能：无人挖机装满车后，用铲斗沿车厢上表面从车尾向车头推抹，
///       刮平卡车框内高出部分。
///
/// 激活：/Sys_SeqAction (std_msgs/Float64) data==6.0 触发；非 6.0 时停止/复位。
///
/// 几何：
///   - 由卡车位姿（中心 + RTK 方位角）确定 base 系航向 β=(180−α) mod 360；
///   - 方向向量 dir=(cosβ, sinβ)；
///   - 车尾起点 tail = center − (L/2 − inset)·dir；
///   - 车头终点 head = center + (L/2 − inset)·dir；
///   - 目标高度 z = (center_z − truck_height_offset) + smooth_height_bias；
///   - 对 tail/head 两点分别做 IK，取 swing / arm 分量作为起终点关节目标。
///
/// 两阶段：
///   Phase1：从当前实时位置限速逼近车尾起点 (swing_start, arm_start)，
///           连续 confirm_frames 帧进入容差 → 进入 Phase2；
///   Phase2：限速推抹至车头终点 (swing_end, arm_end)，
///           arm 到位（可选同时校验 swing）连续 confirm_frames 帧 → 抹平结束。
///   boom/bucket 每帧填充当前实测角（不主动命令）。
///
/// 输入话题：
///   /joints_angle             (geometry_msgs/Quaternion) x=boom, y=arm, z=bucket（度）
///   /heading2swing_topic      (std_msgs/Float32)         data=swing（度）
///   /truck_center_unloadpoint (geometry_msgs/Quaternion) x/y/z=中心(z=RTK高), w=RTK方位角α
///   /Sys_SeqAction            (std_msgs/Float64)         data==6.0 激活抹平
///
/// 输出话题：
///   /RefDeviceTraj_Smooth (geometry_msgs/Pose)
///     Orientation: x=swing, y=boom, z=arm, w=bucket（度）
///     Position.x: 1=执行中, 0=结束/空闲
///   /FlagSmoothFinish (std_msgs/Bool) true=抹平完成

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <kinematics/kinematics.hpp>

#include <memory>

#include "smooth_trajectory_planner/angle_utils.h"

namespace smooth_trajectory_planner {

/// 抹平执行阶段
enum class SmoothPhase {
  kIdle,    // 空闲等待激活
  kPhase1,  // 阶段1：进入车厢尾起点
  kPhase2,  // 阶段2：推抹至车厢头终点
  kDone,    // 正常完成
  kFailed   // 异常中止（超时 / 输入失效 / IK 失败）
};

class SmoothTrajectoryNode {
 public:
  SmoothTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  // ==================== 初始化 ====================
  void LoadParams();
  void SetupTopics();

  // ==================== 订阅回调 ====================
  void JointsCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void SwingCallback(const std_msgs::Float32::ConstPtr& msg);
  void TruckPoseCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void ActivateCallback(const std_msgs::Float64::ConstPtr& msg);

  // ==================== 定时器与阶段执行 ====================
  void TimerCallback(const ros::TimerEvent& event);
  void ExecutePhase1();
  void ExecutePhase2();

  // ==================== 几何与 IK ====================
  /// 激活时计算抹平起终点关节目标（含 IK 失败降级，恒不拒绝激活）：
  ///   主降级：端点距离收缩重试；兜底：最大能力扫掠；最终：原地保持。
  bool ComputeSmoothTargets();
  /// 对单点 (x, y, z) 做 IK，输出 swing/arm（度），失败返回 false。
  /// verbose=true 时失败打印 ROS_ERROR（重试探测期间应传 false 防刷屏）。
  bool SolvePointIk(double x, double y, double z,
                    double& swing_deg, double& arm_deg, bool verbose = true);
  /// 对沿 dir 偏移 s（相对卡车中心）、高度 z 的点做 IK
  bool SolveOffsetIk(double s, double z, double dir_x, double dir_y,
                     double& swing_deg, double& arm_deg);
  /// 端点收缩重试：从 s_start 朝中心(0)按 shrink_step_m_ 收缩，
  /// 返回首个可达 s（最贴近原端点）；全程不可达返回 false
  bool ShrinkProbe(double s_start, double z, double dir_x, double dir_y,
                   double& s_out, double& swing_deg, double& arm_deg);
  /// 最大能力扫掠：在 [-half, half] 密采样，取可达点中 s 最小/最大者，
  /// 跨度 >= min_smooth_span_m_ 时成功
  bool MaxSweepProbe(double half, double z, double dir_x, double dir_y,
                     double& s_tail_out, double& s_head_out,
                     double& sw_tail, double& ar_tail,
                     double& sw_head, double& ar_head);
  /// 调试打印最终起终点关节目标（仅 enable_debug_log_ 时）
  void LogTargets(double z);

  // ==================== 工具方法 ====================
  void PublishTrajectoryFrame(double swing, double boom, double arm,
                              double bucket, bool running);
  void PublishFinish(bool finished);
  /// 限速逼近：从 current 朝 target 推进不超过 max_step，返回新值
  double StepTowards(double current, double target, double max_step) const;
  void StopExecution();                     // 外部撤销：停止并回空闲
  void AbortExecution(const char* reason);  // 异常中止：保持末指令并置失败
  bool CheckInputsValid();                  // 激活前置校验
  bool CheckInputsFresh();                  // 执行期输入新鲜度校验

  // ==================== 成员变量 ====================
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  // 运动学求解器
  std::unique_ptr<kinematics::KinematicsSolver> solver_;

  // 订阅
  ros::Subscriber sub_joints_;      // /joints_angle
  ros::Subscriber sub_swing_;       // /heading2swing_topic
  ros::Subscriber sub_truck_pose_;  // /truck_center_unloadpoint
  ros::Subscriber sub_activate_;    // /Sys_SeqAction

  // 发布
  ros::Publisher pub_trajectory_;  // /RefDeviceTraj_Smooth (geometry_msgs/Pose)
  ros::Publisher pub_finish_;      // /FlagSmoothFinish (std_msgs/Bool)

  // 定时器（激活时 start，非执行态 stop）
  ros::Timer timer_;

  // ---- 抹平范围/几何参数 ----
  double truck_height_offset_ = 0.5;   // 车厢上表面高度偏移 (m)
  double smooth_height_bias_ = 0.0;    // IK 目标高度裕量 (m)
  double truck_box_length_ = 5.0;      // 卡车厢长 L (m)
  double box_inset_ = 0.3;             // 车厢头尾内缩裕量 (m)
  double bucket_angle_deg_ = 25.0;     // IK 固定铲斗关节角 (deg)
  bool use_pivot_as_target_ = true;    // IK 目标点语义（true=铰接点）

  // ---- IK 失败降级（端点收缩重试 / 最大能力扫掠）----
  double shrink_step_m_ = 0.1;         // 收缩/采样步长 (m)
  double min_smooth_span_m_ = 0.5;     // 最小有效抹平跨度 (m)，低于此视为无意义

  // ---- 发布频率与限速 ----
  double publish_rate_ = 10.0;         // 逐帧发布频率 (Hz)
  double swing_vel_dps_ = 15.0;        // swing 命令限速 (deg/s)
  double arm_vel_dps_ = 15.0;          // arm 命令限速 (deg/s)

  // ---- 到位容差与确认帧数 ----
  double swing_tolerance_deg_ = 5.0;   // swing 到位容差 (deg)
  double arm_tolerance_deg_ = 5.0;     // arm 到位容差 (deg)
  int confirm_frames_ = 3;             // 到位需连续确认帧数
  bool phase2_check_swing_ = true;     // Phase2 是否同时校验 swing 到位

  // ---- 超时 (s) ----
  double phase1_timeout_sec_ = 8.0;    // Phase1 超时
  double phase2_timeout_sec_ = 10.0;   // Phase2 超时
  double total_timeout_sec_ = 13.0;    // 全局执行超时
  double input_timeout_sec_ = 1.0;     // 输入话题最大允许静默时长

  // ---- 调试 ----
  bool enable_debug_log_ = false;
  // 调试日志去重：记录上一次打印的到位确认计数，仅在跳变时打印事件
  int dbg_p1_cnt_ = -1;
  int dbg_p2_cnt_ = -1;

  // ---- 状态机 ----
  SmoothPhase phase_ = SmoothPhase::kIdle;

  // ---- 全局/阶段计时 ----
  ros::Time execution_start_time_;     // 触发时刻（全局超时基准）
  ros::Time phase_start_time_;         // 当前阶段起始时刻
  ros::Time last_timer_time_;          // 上一帧定时器时刻（计算 dt）

  // ---- 卡车位姿缓存 ----
  double truck_center_x_ = 0.0;
  double truck_center_y_ = 0.0;
  double truck_center_z_ = 0.0;        // RTK 高度
  double truck_rtk_heading_deg_ = 0.0; // RTK 方位角 α
  double truck_yaw_rad_ = 0.0;         // base 系航向 β (rad)
  bool truck_pose_valid_ = false;
  ros::Time last_truck_time_;

  // ---- 抹平起终点关节目标（激活时 IK 计算，度）----
  double swing_start_deg_ = 0.0;       // 车尾起点 swing
  double arm_start_deg_ = 0.0;         // 车尾起点 arm
  double swing_end_deg_ = 0.0;         // 车头终点 swing
  double arm_end_deg_ = 0.0;           // 车头终点 arm

  // ---- 当前命令值（限速逼近的内部状态，度）----
  double cmd_swing_deg_ = 0.0;
  double cmd_arm_deg_ = 0.0;

  // ---- 到位确认计数 ----
  int phase1_confirm_count_ = 0;
  int phase2_confirm_count_ = 0;

  // ---- 实时关节角缓存（度）----
  double swing_deg_ = 0.0;
  double boom_deg_ = 0.0;
  double arm_deg_ = 0.0;
  double bucket_deg_ = 0.0;
  bool joints_valid_ = false;
  bool swing_valid_ = false;
  ros::Time last_joints_time_;
  ros::Time last_swing_time_;

  // ---- 完成标志当前发布状态 ----
  bool finish_flag_ = false;
};

}  // namespace smooth_trajectory_planner
