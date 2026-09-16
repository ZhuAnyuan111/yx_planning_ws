#pragma once

/// @file reset_trajectory_node.h
/// @brief 复位轨迹规划与分阶段执行节点类声明。
///
/// 策略：
///   阶段1（kSwing）：回转直接发目标终点角，动臂/斗杆/铲斗锁定触发时刻的值。
///                    持续监测实时回转反馈，等待已回转 > threshold 度。
///   阶段2（kArm）：  回转保持目标角，动臂/斗杆/铲斗的进度由回转剩余行程映射：
///                    t = (已回转 - threshold) / (总行程 - threshold)，t∈[0,1]
///                    当回转到达或超调目标角时，直接发布精确目标值。
///
/// 输入话题：
///   /joints_angle             (geometry_msgs/Quaternion) x=boom, y=arm, z=bucket（度）
///   /heading2swing_topic      (std_msgs/Float32)         data=swing（度）
///   /GP_dig_joints            (geometry_msgs/Quaternion) 挖掘循环模式目标：
///                             x=swing,y=boom,z=arm,w=bucket（度）
///   /clean_digPoint           (geometry_msgs/Quaternion) 搭台模式目标：
///                             x=swing,y=boom,z=arm,w=bucket（度）
///   /Sys_SRe_FlagResetExcute  (std_msgs/Float64)         data==1 挖掘循环复位 /
///                             ==2 搭台复位 / ==0 退出
///
/// 目标姿态由上位机预计算后经对应模式话题直接给出（四关节角）：
///   swing=x, boom=y, arm=z, bucket=w
///
/// 输出话题：
///   /RefDeviceTraj_Reset (geometry_msgs/Pose)
///     Orientation: x=swing, y=boom, z=arm, w=bucket（度）
///     Position.x: 1=执行中, 0=阶段结束

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <string>

#include "reset_trajectory_planner/angle_utils.h"

namespace reset_trajectory_planner {

/// 复位执行阶段
enum class ResetPhase {
  kIdle,   // 空闲等待
  kSwing,  // 阶段1：回转先行，臂架保持
  kArm,    // 阶段2：臂架跟随回转映射
  kDone,   // 正常完成
  kFailed  // 异常中止（超时 / 输入失效）
};

/// 工作模式（决定复位目标来源）
enum class ResetMode {
  kNone,      // 未激活
  kDigCycle,  // 挖掘循环模式：目标源 /GP_dig_joints
  kBench      // 搭台模式：目标源 /clean_digPoint
};

class ResetTrajectoryNode {
 public:
  ResetTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  // ==================== 初始化 ====================
  void LoadParams();
  void SetupTopics();

  // ==================== 订阅回调 ====================
  void JointsCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void SwingCallback(const std_msgs::Float32::ConstPtr& msg);
  void GPDigJointsCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void CleanDigPointCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void ActivateCallback(const std_msgs::Float64::ConstPtr& msg);
  void TruckCenterCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
  void BucketTerrainCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  void RealBktPosCallback(const geometry_msgs::Point::ConstPtr& msg);

  // ==================== 阶段切换 ====================
  void EnterSwingPhase();
  void EnterArmPhase();

  // ==================== 定时器与阶段执行 ====================
  void TimerCallback(const ros::TimerEvent& event);
  void ExecuteSwingPhase();
  void ExecuteArmPhase();

  // ==================== 工具方法 ====================
  void PublishTrajectoryFrame(double swing, double boom, double arm,
                              double bucket, bool running);
  void StopExecution();                     // 外部撤销：停止并置结束位
  void AbortExecution(const char* reason);  // 异常中止：保持末指令并置结束位
  bool CheckInputsValid(bool goal_src_valid, const char* src_topic);  // 激活前置校验：数据是否曾到达
  bool CheckInputsFresh();                  // 执行期校验：数据是否仍在更新
  bool IsPhysicalGoalReached();             // 物理状态提前结束判定
  double UpdateFilteredTraveled();          // 更新并返回抗抖动已行程（有副作用）

  // ==================== 成员变量 ====================
  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;

  // 订阅
  ros::Subscriber sub_joints_;
  ros::Subscriber sub_swing_;            // /heading2swing_topic
  ros::Subscriber sub_gp_dig_joints_;    // /GP_dig_joints（挖掘循环模式目标）
  ros::Subscriber sub_clean_dig_point_;  // /clean_digPoint（搭台模式目标）
  ros::Subscriber sub_activate_;         // /Sys_SRe_FlagResetExcute
  ros::Subscriber sub_truck_center_;
  ros::Subscriber sub_bucket_terrain_;
  ros::Subscriber sub_real_bkt_pos_;

  // 发布
  ros::Publisher pub_trajectory_;   // /RefDeviceTraj_Reset (geometry_msgs/Pose)

  // 定时器
  ros::Timer timer_;

  // ---- 参数 ----
  double swing_threshold_deg_ = 10.0;   // 阶段1→2切换阈值（度）
  double swing_timeout_sec_ = 15.0;     // 阶段1最大等待时间
  double total_timeout_sec_ = 30.0;     // 全局执行超时（从触发开始计）
  double truck_safe_margin_deg_ = 60.0; // truck_safe 判定：与卡车中心航向的最小角距（度）
  double rs_threshold_5_ = 5.0;         // RSThreshold(5)：bucket_height_gd 结束阈值
  double input_timeout_sec_ = 1.0;      // 输入话题最大允许静默时长（秒）
  bool enable_debug_log_ = false;       // 调试日志开关（yaml 配置）

  // ---- 状态机 ----
  ResetPhase phase_ = ResetPhase::kIdle;

  // ---- 全局计时 ----
  ros::Time execution_start_time_;       // 触发时刻（全局超时基准）

  // ---- 阶段1相关 ----
  double swing_start_deg_ = 0.0;        // 触发时的回转起始角
  double total_swing_travel_ = 0.0;     // 总回转行程（方向性）
  int swing_direction_ = -1;            // 回转方向：1=顺时针，-1=逆时针
  ros::Time swing_phase_start_time_;
  double hold_boom_deg_ = 0.0;          // 阶段1锁定的臂架角度（保持目标）
  double hold_arm_deg_ = 0.0;
  double hold_bucket_deg_ = 0.0;

  // ---- 反向抖动保护 ----
  double max_traveled_ = 0.0;           // 已行程历史最大值（单调递增）
  double last_swing_deg_ = 0.0;         // 上一帧回转读数
  bool has_last_swing_ = false;

  // ---- 阶段2相关 ----
  double arm_start_boom_deg_ = 0.0;     // 阶段2起始时刻的臂架角度
  double arm_start_arm_deg_ = 0.0;
  double arm_start_bucket_deg_ = 0.0;
  double arm_remaining_swing_ = 0.0;    // 阶段2对应的回转剩余行程（度）

  // ---- 实时关节角缓存（度）----
  double swing_deg_ = 0.0;
  double boom_deg_ = 0.0;
  double arm_deg_ = 0.0;
  double bucket_deg_ = 0.0;
  bool joints_valid_ = false;
  bool swing_valid_ = false;

  // ---- 挖掘循环模式目标关节角（/GP_dig_joints, 度）----
  double gp_swing_deg_ = 0.0;
  double gp_boom_deg_ = 0.0;
  double gp_arm_deg_ = 0.0;
  double gp_bucket_deg_ = 0.0;
  bool gp_goal_valid_ = false;

  // ---- 搭台模式目标关节角（/clean_digPoint, 度）----
  double clean_swing_deg_ = 0.0;
  double clean_boom_deg_ = 0.0;
  double clean_arm_deg_ = 0.0;
  double clean_bucket_deg_ = 0.0;
  bool clean_goal_valid_ = false;

  // ---- 当前工作模式（激活时确定，用于日志与目标源选择）----
  ResetMode current_mode_ = ResetMode::kNone;

  // ---- 目标快照（激活瞬间锁定，执行期一律使用）----
  // 防止挖掘点话题中途刷新导致回转指令跳变而进度基准仍为旧值
  double exec_goal_swing_deg_ = 0.0;
  double exec_goal_boom_deg_ = 0.0;
  double exec_goal_arm_deg_ = 0.0;
  double exec_goal_bucket_deg_ = 0.0;

  // ---- 末次发布指令（异常中止时保持，避免中止瞬间跳向目标）----
  double last_cmd_swing_deg_ = 0.0;
  double last_cmd_boom_deg_ = 0.0;
  double last_cmd_arm_deg_ = 0.0;
  double last_cmd_bucket_deg_ = 0.0;
  bool has_last_cmd_ = false;

  // ---- 输入新鲜度时间戳 ----
  ros::Time last_joints_time_;
  ros::Time last_swing_time_;

  // ---- 物理提前结束判定相关状态 ----
  double truck_center_swing_deg_ = 0.0;  // 卡车中心对应的挖机航向角（度）
  bool truck_center_valid_ = false;
  double bucket_height_gd_ = 0.0;        // /bucket_terrain_delta_position .z
  bool bucket_terrain_valid_ = false;
  double real_bkt_pos_z_ = 0.0;          // /RealBktPosXYZ .z（齿尖离地高度）
  bool real_bkt_pos_valid_ = false;
};

}  // namespace reset_trajectory_planner
