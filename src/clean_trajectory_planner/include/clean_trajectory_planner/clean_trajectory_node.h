#pragma once

/// @file clean_trajectory_node.h
/// @brief 清台（地面平整/搭台）轨迹规划与执行节点类声明。
///
/// 功能：挖掘装车结束、卡车驶离后，决策层 StagingCycleFsm 进入搭台/清台循环。
///       本节点以中心线(/GP2mid/front_lane_deg)±左右偏移展开多条"径向条带"，
///       逐条带遍历清台区域：每条带由复位节点摆到起点后，命令斗杆内收刮平。
///
/// 单步执行 + 跨激活记忆：节点每次激活只完成"当前条带"的斗杆内收，靠决策层
///       staging 外层循环（复位→清台→挖掘→卸载→复位…）驱动逐条带推进；
///       节点用 strip_index_ 跨激活记忆遍历进度。
///
/// 起点姿态（发 /clean_digPoint，供复位搭台模式摆位）：
///   - swing = 条带角 θ_i；arm = start_arm_deg（最大外伸极限）；bucket = start_bucket_deg(0°)；
///   - boom 由清台目标高度 z（/Sys_SCo_Traj position.y）经 FK 一维二分反解，使齿尖落到 z。
///
/// 内收执行：激活后命令斗杆姿态角(boom+arm)限速逼近 end_arm_attitude_deg；swing 保持 θ_i；
///   boom/bucket 每帧填实测角（内收过程由控制层 boom 齿尖高度闭环维持贴地）。
///
/// 激活：/Sys_SLe_FlagGroundLevelExcute (std_msgs/Float64) ==1.0 执行 / ==0.0 停止；
///       /Sys_SeqAction==7.0 为并行指示（仅日志）。
///
/// 输入话题：
///   /joints_angle                 (geometry_msgs/Quaternion) x=boom,y=arm,z=bucket（度）
///   /heading2swing_topic          (std_msgs/Float32)         data=swing（度）
///   /Sys_SCo_Traj                 (geometry_msgs/Pose)       position.y=清台目标高度 z(m)
///   /Sys_SLe_FlagGroundLevelExcute(std_msgs/Float64)         ==1.0 激活 / ==0.0 停止
///   /Sys_SeqAction                (std_msgs/Float64)         ==7.0 清台阶段（指示）
///   /GP2mid/front_lane_deg        (ROS param, double)        中心线 swing 角（度）
///
/// 输出话题：
///   /clean_digPoint       (geometry_msgs/Quaternion, latched) x=swing,y=boom,z=arm,w=bucket（度）
///   /RefDeviceTraj_CleanUp(geometry_msgs/Pose, 10Hz)
///     Orientation: x=swing,y=boom,z=arm,w=bucket（度）；Position.x: 1=执行中,0=结束/空闲
///   /FlagCleanFinish      (std_msgs/Float32, latched)
///     1.0=本条带完成且还有剩余（决策接续挖掘循环）；2.0=全部条带完成（最终收尾）；0.0=未完成

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <kinematics/kinematics.hpp>

#include <memory>
#include <string>
#include <vector>

#include "clean_trajectory_planner/angle_utils.h"

namespace clean_trajectory_planner {

/// 清台执行阶段
enum class CleanPhase {
  kIdle,      // 空闲 armed：已发布当前条带 /clean_digPoint，等待激活
  kRetract,   // 激活后：斗杆内收刮平
  kStripDone, // 本条带完成：已上报 /FlagCleanFinish，等待决策层撤销(flag=0)
  kFailed     // 异常中止（超时 / 输入失效）
};

class CleanTrajectoryNode {
 public:
  CleanTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  // ==================== 初始化 ====================
  void LoadParams();
  void SetupTopics();

  // ==================== 订阅回调 ====================
  void JointsCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void SwingCallback(const std_msgs::Float32::ConstPtr& msg);
  void HeightCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void ActivateCallback(const std_msgs::Float64::ConstPtr& msg);
  void SeqActionCallback(const std_msgs::Float64::ConstPtr& msg);

  // ==================== 定时器与执行 ====================
  void TimerCallback(const ros::TimerEvent& event);
  void IdleRefresh();       // armed 态：节流重读 front_lane / 重发 /clean_digPoint
  void StartRetract();      // kIdle -> kRetract
  void ExecuteRetract();    // kRetract 每帧：限速内收 + 到位判定
  void CompleteStrip();     // 本条带完成：推进/收尾 + 上报 /FlagCleanFinish

  // ==================== 条带几何与 boom 反解 ====================
  /// 读 front_lane_deg 生成 strips_（swing 角序列）；保留并 clamp strip_index_
  void GenerateStrips();
  /// 计算第 i 条带起点姿态（swing=θ_i, arm=start_arm, bucket=start_bucket, boom=反解）
  /// 成功返回 true 并填充输出；boom 反解失败返回 false
  bool ComputeStripStartPose(int i, double& swing_deg, double& boom_deg,
                             double& arm_deg, double& bucket_deg);
  /// 给定齿尖目标高度 z(m)、固定 arm/bucket(deg)，二分反解 boom(deg)，
  /// 在 [boom_lower, boom_upper] 内使 FK 高度 = z；无解返回 false
  bool SolveBoomForHeight(double z_target, double arm_deg, double bucket_deg,
                          double& boom_deg);
  /// 计算并发布当前条带 /clean_digPoint（latched）；成功置 published 标志
  void PublishCleanDigPoint();

  // ==================== 工具方法 ====================
  void PublishTrajectoryFrame(double swing, double boom, double arm,
                              double bucket, bool running);
  void PublishCleanFinish(float value);
  /// 限速逼近：从 current 朝 target 推进不超过 max_step，返回新值
  double StepTowards(double current, double target, double max_step) const;
  void StopExecution();                     // 外部撤销(flag=0)：停止内收回空闲
  void AbortExecution(const char* reason);  // 异常中止：保持末指令并置失败
  bool CheckInputsValid();                  // 激活前置校验（joints/swing 有效且新鲜）
  bool CheckInputsFresh();                  // 执行期输入新鲜度校验
  double CurrentStripAngleDeg() const;      // 当前条带 swing 角（归一化）

  // ==================== 成员变量 ====================
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  // 运动学求解器（仅用其 FK + 几何/限位）
  std::unique_ptr<kinematics::KinematicsSolver> solver_;

  // 订阅
  ros::Subscriber sub_joints_;    // /joints_angle
  ros::Subscriber sub_swing_;     // /heading2swing_topic
  ros::Subscriber sub_height_;    // /Sys_SCo_Traj (position.y=地面高度 z)
  ros::Subscriber sub_activate_;  // /Sys_SLe_FlagGroundLevelExcute
  ros::Subscriber sub_seq_;       // /Sys_SeqAction (并行指示, 仅日志)

  // 发布
  ros::Publisher pub_trajectory_;  // /RefDeviceTraj_CleanUp (geometry_msgs/Pose)
  ros::Publisher pub_finish_;      // /FlagCleanFinish (std_msgs/Float32, latched)
  ros::Publisher pub_clean_point_; // /clean_digPoint (geometry_msgs/Quaternion, latched)

  // 定时器（常开：kRetract 执行 / kIdle 刷新）
  ros::Timer timer_;

  // ---- 区域/条带参数 ----
  std::string front_lane_param_;
  std::string ground_height_topic_;
  double left_offset_deg_ = 20.0;
  double right_offset_deg_ = 20.0;
  double strip_step_deg_ = 5.0;
  // 遍历方向（运行时决定，非参数）：全新遍历时依据当前实时 swing 选择最近边界为起点。
  //   false=从 lo(=lane-left) 递增；true=从 hi(=lane+right) 递减。
  bool traverse_descend_ = false;

  // ---- 起点姿态 / 内收终点 ----
  double start_arm_deg_ = -30.0;     // 斗杆最大外伸极限角（关节角）
  double start_bucket_deg_ = 0.0;    // 起点铲斗角（固定 0°）
  // 内收终点「斗杆姿态角」(boom+arm, deg)：内收过程每帧由实测 boom 反推斗杆关节角，
  //   使斗杆绝对倾角恒定（区别于旧的固定关节角 end_arm_deg）。
  double end_arm_attitude_deg_ = -60.0;

  // ---- boom 高度反解 ----
  double boom_solve_tol_m_ = 0.01;
  int boom_solve_max_iter_ = 60;
  double height_offset_m_ = 0.0;
  bool height_target_at_pivot_ = false;
  double height_eps_m_ = 0.02;

  // ---- 发布频率 ----
  double publish_rate_ = 10.0;
  double idle_refresh_sec_ = 1.0;

  // ---- 命令限速 (deg/s) ----
  double arm_vel_dps_ = 15.0;
  double swing_vel_dps_ = 15.0;

  // ---- 到位容差与确认帧数 ----
  double arm_tolerance_deg_ = 5.0;
  double swing_tolerance_deg_ = 5.0;
  int confirm_frames_ = 3;
  bool retract_check_swing_ = false;

  // ---- 超时 (s) ----
  double strip_timeout_sec_ = 8.0;
  double total_timeout_sec_ = 12.0;
  double input_timeout_sec_ = 1.0;

  // ---- 调试 ----
  bool enable_debug_log_ = false;
  int dbg_confirm_cnt_ = -1;  // 到位计数跳变去重

  // ---- 状态机 ----
  CleanPhase phase_ = CleanPhase::kIdle;

  // ---- 条带遍历进度（跨激活记忆）----
  std::vector<double> strips_;   // 各条带 swing 角(deg)
  int strip_index_ = 0;
  double front_lane_deg_ = 0.0;  // 上次生成条带用的中心线角

  // ---- 计时 ----
  ros::Time execution_start_time_;  // 本次激活时刻（全局超时基准）
  ros::Time strip_start_time_;      // 本条带内收起始时刻
  ros::Time last_timer_time_;       // 上一帧定时器时刻（计算 dt）
  ros::Time last_idle_refresh_time_;

  // ---- 实时关节角缓存（度）----
  double swing_deg_ = 0.0;
  double boom_deg_ = 0.0;
  double arm_deg_ = 0.0;
  double bucket_deg_ = 0.0;
  bool joints_valid_ = false;
  bool swing_valid_ = false;
  ros::Time last_joints_time_;
  ros::Time last_swing_time_;

  // ---- 清台目标高度缓存 ----
  double z_target_ = 0.0;  // 已含 height_offset_m_
  bool height_valid_ = false;
  ros::Time last_height_time_;

  // ---- /clean_digPoint 发布状态与当前条带起点姿态缓存 ----
  bool clean_digpoint_published_ = false;
  double published_lane_ = 1e9;  // 上次发布时中心线角（防抖比较）
  double published_z_ = 1e9;     // 上次发布时高度（防抖比较）
  double start_swing_deg_ = 0.0; // 当前条带起点 swing
  double start_boom_deg_ = 0.0;  // 当前条带起点反解 boom

  // ---- 当前命令值（限速逼近内部状态，度）----
  double cmd_swing_deg_ = 0.0;
  double cmd_arm_deg_ = 0.0;

  // ---- 到位确认计数 ----
  int retract_confirm_count_ = 0;

  // ---- 完成标志当前发布值 ----
  float clean_finish_value_ = 0.0f;

  // ---- /Sys_SeqAction 最近值（仅日志/调试）----
  double last_seq_action_ = 0.0;
};

}  // namespace clean_trajectory_planner
