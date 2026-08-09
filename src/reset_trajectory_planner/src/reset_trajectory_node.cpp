#include "reset_trajectory_planner/reset_trajectory_node.h"

#include <geometry_msgs/Pose.h>

#include <algorithm>
#include <cmath>

#include "reset_trajectory_planner/angle_utils.h"

namespace reset_trajectory_planner {

namespace {
constexpr double kPublishRate = 10.0;  // 10Hz 逐帧发布
constexpr double kActivateValue = 2.0; // /Sys_SeqAction 激活值
}  // namespace

// ==================== 构造函数 ====================

ResetTrajectoryNode::ResetTrajectoryNode(ros::NodeHandle& nh,
                                         ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  LoadParams();
  SetupTopics();

  timer_ = nh_.createTimer(ros::Duration(1.0 / kPublishRate),
                           &ResetTrajectoryNode::TimerCallback, this,
                           /*oneshot=*/false, /*autostart=*/false);

  ROS_INFO("[reset_traj] ready. swing_threshold=%.1f deg, swing_timeout=%.1fs, total_timeout=%.1fs",
           swing_threshold_deg_, swing_timeout_sec_, total_timeout_sec_);
}

// ==================== 初始化 ====================

void ResetTrajectoryNode::LoadParams() {
  swing_threshold_deg_ = pnh_.param("swing_threshold_deg", 10.0);
  swing_timeout_sec_ = pnh_.param("swing_timeout_sec", 15.0);
  total_timeout_sec_ = pnh_.param("total_timeout_sec", 30.0);
  truck_safe_margin_deg_ = pnh_.param("truck_safe_margin_deg", 60.0);
  rs_threshold_5_ = pnh_.param("rs_threshold_5", 5.0);
  input_timeout_sec_ = pnh_.param("input_timeout_sec", 1.0);
  enable_debug_log_ = pnh_.param("enable_debug_log", false);

  if (enable_debug_log_) {
    ROS_INFO("[reset_traj] debug log ENABLED");
  }
}

void ResetTrajectoryNode::SetupTopics() {
  sub_joints_ = nh_.subscribe("/joints_angle", 1,
                              &ResetTrajectoryNode::JointsCallback, this);
  sub_swing_ = nh_.subscribe("/Swing_topic", 1,
                             &ResetTrajectoryNode::SwingCallback, this);
  sub_dig_point_ = nh_.subscribe(
      "/dig_point", 1,
      &ResetTrajectoryNode::DigPointCallback, this);
  sub_activate_ = nh_.subscribe("/Sys_SeqAction", 1,
                                &ResetTrajectoryNode::ActivateCallback, this);
  sub_truck_center_ = nh_.subscribe(
      "/truck_center_point", 1,
      &ResetTrajectoryNode::TruckCenterCallback, this);
  sub_bucket_terrain_ = nh_.subscribe(
      "/bucket_terrain_delta_position", 1,
      &ResetTrajectoryNode::BucketTerrainCallback, this);
  sub_real_bkt_pos_ = nh_.subscribe(
      "/RealBktPosXYZ", 1,
      &ResetTrajectoryNode::RealBktPosCallback, this);

  pub_trajectory_ = nh_.advertise<geometry_msgs::Pose>(
      "/RefDeviceTraj_Reset", 1);
}

// ==================== 订阅回调 ====================

void ResetTrajectoryNode::JointsCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  boom_deg_ = msg->x;
  arm_deg_ = msg->y;
  bucket_deg_ = msg->z;
  const bool first_frame = !joints_valid_;
  joints_valid_ = true;
  last_joints_time_ = ros::Time::now();
  if (first_frame) {
    ROS_INFO("[reset_traj] first /joints_angle: boom=%.2f arm=%.2f bucket=%.2f deg",
             boom_deg_, arm_deg_, bucket_deg_);
  }
}

void ResetTrajectoryNode::SwingCallback(
    const geometry_msgs::Point::ConstPtr& msg) {
  swing_deg_ = msg->x;
  const bool first_frame = !swing_valid_;
  swing_valid_ = true;
  last_swing_time_ = ros::Time::now();
  if (first_frame) {
    ROS_INFO("[reset_traj] first /Swing_topic: swing=%.2f deg", swing_deg_);
  }
}

/// /dig_point: 上位机已预计算好的四关节目标角（度）
///   x = swing 目标角
///   y = boom  目标角
///   z = arm   目标角
///   w = bucket目标角
void ResetTrajectoryNode::DigPointCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  const bool first_frame = !dig_point_valid_;
  dig_swing_deg_ = msg->x;
  goal_swing_deg_ = msg->x;
  goal_boom_deg_ = msg->y;
  goal_arm_deg_ = msg->z;
  goal_bucket_deg_ = msg->w;
  goal_valid_ = true;
  dig_point_valid_ = true;

  if (first_frame) {
    ROS_INFO("[reset_traj] first /dig_point: swing=%.2f boom=%.2f arm=%.2f "
             "bucket=%.2f deg",
             goal_swing_deg_, goal_boom_deg_, goal_arm_deg_, goal_bucket_deg_);
  } else if (enable_debug_log_) {
    ROS_INFO_THROTTLE(1.0,
        "[reset_traj][dbg] /dig_point: swing=%.2f boom=%.2f arm=%.2f "
        "bucket=%.2f deg",
        goal_swing_deg_, goal_boom_deg_, goal_arm_deg_, goal_bucket_deg_);
  }
}

/// 卡车中心位置（x/y/z）→ 挖机航向角
/// truck_center_swingAngle = rad2deg(atan2(y, x)) + 180
void ResetTrajectoryNode::TruckCenterCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  const double angle_deg = std::atan2(msg->y, msg->x) * 180.0 / M_PI + 180.0;
  truck_center_swing_deg_ = NormalizeAngle(angle_deg);
  const bool first_frame = !truck_center_valid_;
  truck_center_valid_ = true;
  if (first_frame) {
    ROS_INFO("[reset_traj] first /truck_center_point: truck_swing=%.2f deg (from x=%.3f y=%.3f)",
             truck_center_swing_deg_, msg->x, msg->y);
  }
}

void ResetTrajectoryNode::BucketTerrainCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
  bucket_height_gd_ = msg->point.z;
  bucket_terrain_valid_ = true;
}

void ResetTrajectoryNode::RealBktPosCallback(
    const geometry_msgs::Point::ConstPtr& msg) {
  real_bkt_pos_z_ = msg->z;
  real_bkt_pos_valid_ = true;
}

void ResetTrajectoryNode::ActivateCallback(
    const std_msgs::Float64::ConstPtr& msg) {
  if (msg->data != kActivateValue) {
    // 非复位激活信号，若正在执行则停止
    if (phase_ != ResetPhase::kIdle && phase_ != ResetPhase::kDone) {
      StopExecution();
    }
    return;
  }

  // 已在执行中，忽略重复触发
  if (phase_ == ResetPhase::kSwing || phase_ == ResetPhase::kArm) return;

  ROS_INFO("[reset_traj] activation received (Sys_SeqAction=2)");

  if (!CheckInputsValid()) return;

  // 记录全局执行起始时刻
  execution_start_time_ = ros::Time::now();

  // 锁定目标快照：执行期间一律使用快照值，避免挖掘点话题中途刷新
  // 导致回转指令跳变而进度基准（起始角/方向/总行程）仍为旧值
  exec_goal_swing_deg_ = goal_swing_deg_;
  exec_goal_boom_deg_ = goal_boom_deg_;
  exec_goal_arm_deg_ = goal_arm_deg_;
  exec_goal_bucket_deg_ = goal_bucket_deg_;

  // 记录起始回转角，判断方向和总行程（使用方向性角度差处理 360° 回绕）
  swing_start_deg_ = swing_deg_;
  swing_direction_ = DetermineDirection(swing_start_deg_, exec_goal_swing_deg_);
  total_swing_travel_ = CalculateAngleDiff(swing_start_deg_,
                                          exec_goal_swing_deg_,
                                          swing_direction_);

  // 重置反向抖动保护状态
  max_traveled_ = 0.0;
  last_swing_deg_ = swing_start_deg_;
  has_last_swing_ = true;

  // 重置末次指令缓存
  has_last_cmd_ = false;

  ROS_INFO("[reset_traj] swing: %.1f -> %.1f (travel=%.1f deg, dir=%s)",
           swing_start_deg_, exec_goal_swing_deg_, total_swing_travel_,
           swing_direction_ == 1 ? "CW" : "CCW");
  ROS_INFO("[reset_traj] arm goal: boom=%.1f arm=%.1f bucket=%.1f",
           exec_goal_boom_deg_, exec_goal_arm_deg_, exec_goal_bucket_deg_);

  if (total_swing_travel_ <= swing_threshold_deg_) {
    ROS_INFO("[reset_traj] swing travel <= threshold, skip to arm phase");
    EnterArmPhase();
  } else {
    EnterSwingPhase();
  }

  timer_.start();
}

// ==================== 阶段切换 ====================

void ResetTrajectoryNode::EnterSwingPhase() {
  phase_ = ResetPhase::kSwing;
  swing_phase_start_time_ = ros::Time::now();
  // 锁定当前臂架角度作为阶段1保持目标（让控制器有明确纠偏信号）
  hold_boom_deg_ = boom_deg_;
  hold_arm_deg_ = arm_deg_;
  hold_bucket_deg_ = bucket_deg_;
  PublishTrajectoryFrame(exec_goal_swing_deg_, hold_boom_deg_, hold_arm_deg_,
                         hold_bucket_deg_, true);
  ROS_INFO("[reset_traj] -> kSwing: hold boom=%.1f arm=%.1f bucket=%.1f, waiting swing > %.1f deg",
           hold_boom_deg_, hold_arm_deg_, hold_bucket_deg_, swing_threshold_deg_);
}

void ResetTrajectoryNode::EnterArmPhase() {
  phase_ = ResetPhase::kArm;
  // 取此刻的实时关节角作为臂架映射起点
  arm_start_boom_deg_ = boom_deg_;
  arm_start_arm_deg_ = arm_deg_;
  arm_start_bucket_deg_ = bucket_deg_;
  // 阶段2对应的回转剩余行程（跳过阶段1的场景下可能为负，clamp 到 0）
  arm_remaining_swing_ =
      std::max(0.0, total_swing_travel_ - swing_threshold_deg_);

  ROS_INFO("[reset_traj] -> kArm: boom %.1f->%.1f, arm %.1f->%.1f, "
           "bucket %.1f->%.1f (mapped to remaining swing %.1f deg)",
           arm_start_boom_deg_, exec_goal_boom_deg_,
           arm_start_arm_deg_, exec_goal_arm_deg_,
           arm_start_bucket_deg_, exec_goal_bucket_deg_,
           arm_remaining_swing_);
}

// ==================== 定时器与阶段执行 ====================

void ResetTrajectoryNode::TimerCallback(const ros::TimerEvent& /*event*/) {
  // 非执行态：停表退出
  if (phase_ != ResetPhase::kSwing && phase_ != ResetPhase::kArm) {
    timer_.stop();
    return;
  }

  // 全局超时检查（统一覆盖阶段1与阶段2，防任一阶段卡死时永久运行）
  const double total_elapsed =
      (ros::Time::now() - execution_start_time_).toSec();
  if (total_elapsed > total_timeout_sec_) {
    AbortExecution("total execution timeout");
    return;
  }

  // 输入新鲜度检查（防传感器断流后基于陈旧反馈继续下发指令）
  if (!CheckInputsFresh()) {
    AbortExecution("input data stale (joints/swing feedback lost)");
    return;
  }

  switch (phase_) {
    case ResetPhase::kSwing:
      ExecuteSwingPhase();
      break;
    case ResetPhase::kArm:
      ExecuteArmPhase();
      break;
    default:
      break;
  }
}

/// 阶段1：发布 swing=目标, boom/arm/bucket=锁定值（固定保持目标）
void ResetTrajectoryNode::ExecuteSwingPhase() {
  // 阶段1超时检查
  const double elapsed = (ros::Time::now() - swing_phase_start_time_).toSec();
  if (elapsed > swing_timeout_sec_) {
    AbortExecution("swing phase timeout");
    return;
  }

  // 发布锁定值（非实时值），控制器有明确保持目标
  PublishTrajectoryFrame(exec_goal_swing_deg_, hold_boom_deg_, hold_arm_deg_,
                         hold_bucket_deg_, true);

  // 检查回转进度（反向抖动保护后的已行程）
  const double traveled = UpdateFilteredTraveled();
  // 周期性进度打印（1Hz），便于排查"为什么一直不切换"
  if (enable_debug_log_) {
    ROS_INFO_THROTTLE(1.0,
        "[reset_traj][dbg] kSwing: swing=%.1f traveled=%.1f/%.1f deg "
        "(max=%.1f), elapsed=%.1fs/%.1fs",
        swing_deg_, traveled, swing_threshold_deg_, max_traveled_,
        elapsed, swing_timeout_sec_);
  }
  if (traveled >= swing_threshold_deg_) {
    ROS_INFO("[reset_traj] swing traveled %.1f deg (>= %.1f), entering arm phase",
             traveled, swing_threshold_deg_);
    EnterArmPhase();
  }
}

/// 阶段2：swing=目标保持，boom/arm/bucket 按回转剩余行程映射
void ResetTrajectoryNode::ExecuteArmPhase() {
  const double traveled = UpdateFilteredTraveled();

  // 物理状态提前结束判定：
  // (truck_safe && bucket_height_gd < RSThreshold(5)) && RealBktPosXYZ(3) < 0
  if (IsPhysicalGoalReached()) {
    PublishTrajectoryFrame(exec_goal_swing_deg_, exec_goal_boom_deg_,
                           exec_goal_arm_deg_, exec_goal_bucket_deg_, false);
    timer_.stop();
    phase_ = ResetPhase::kDone;
    ROS_INFO("[reset_traj] physical goal reached (truck_safe & height & bucket_z<0), done");
    return;
  }

  // 到达或超调 → 发精确目标点，Position.x=0 表示阶段结束
  if (traveled >= total_swing_travel_) {
    PublishTrajectoryFrame(exec_goal_swing_deg_, exec_goal_boom_deg_,
                           exec_goal_arm_deg_, exec_goal_bucket_deg_, false);
    timer_.stop();
    phase_ = ResetPhase::kDone;
    ROS_INFO("[reset_traj] swing reached/overshot target (traveled=%.1f >= total=%.1f), done",
             traveled, total_swing_travel_);
    return;
  }

  // 按回转剩余行程映射臂架进度 t ∈ [0, 1]
  double t = 1.0;
  if (arm_remaining_swing_ > 1e-3) {
    t = (traveled - swing_threshold_deg_) / arm_remaining_swing_;
    t = std::max(0.0, std::min(1.0, t));
  }

  const double boom_now =
      arm_start_boom_deg_ + t * (exec_goal_boom_deg_ - arm_start_boom_deg_);
  const double arm_now =
      arm_start_arm_deg_ + t * (exec_goal_arm_deg_ - arm_start_arm_deg_);
  const double bucket_now =
      arm_start_bucket_deg_ +
      t * (exec_goal_bucket_deg_ - arm_start_bucket_deg_);

  // 周期性进度打印（1Hz），便于排查"臂架为什么走这么慢/快"
  if (enable_debug_log_) {
    ROS_INFO_THROTTLE(1.0,
        "[reset_traj][dbg] kArm: t=%.3f traveled=%.1f/%.1f deg, "
        "cmd=[s=%.1f b=%.1f a=%.1f bk=%.1f]",
        t, traveled, total_swing_travel_,
        exec_goal_swing_deg_, boom_now, arm_now, bucket_now);
  }

  PublishTrajectoryFrame(exec_goal_swing_deg_, boom_now, arm_now, bucket_now,
                         true);
}

// ==================== 工具方法 ====================

void ResetTrajectoryNode::PublishTrajectoryFrame(double swing, double boom,
                                                 double arm, double bucket,
                                                 bool running) {
  geometry_msgs::Pose msg;
  // Position.x: 1=执行中, 0=阶段结束
  msg.position.x = running ? 1.0 : 0.0;
  msg.position.y = 0.0;
  msg.position.z = 0.0;
  // Orientation: swing/boom/arm/bucket（角度制）
  msg.orientation.x = swing;
  msg.orientation.y = boom;
  msg.orientation.z = arm;
  msg.orientation.w = bucket;
  pub_trajectory_.publish(msg);

  // 缓存末次指令：异常中止时保持该值，避免中止瞬间跳向目标
  last_cmd_swing_deg_ = swing;
  last_cmd_boom_deg_ = boom;
  last_cmd_arm_deg_ = arm;
  last_cmd_bucket_deg_ = bucket;
  has_last_cmd_ = true;
}

void ResetTrajectoryNode::StopExecution() {
  if (phase_ == ResetPhase::kSwing || phase_ == ResetPhase::kArm) {
    // 外部撤销：保持末次指令并置结束位（不跳向目标）
    if (has_last_cmd_) {
      PublishTrajectoryFrame(last_cmd_swing_deg_, last_cmd_boom_deg_,
                             last_cmd_arm_deg_, last_cmd_bucket_deg_, false);
    }
    timer_.stop();
    phase_ = ResetPhase::kIdle;
    ROS_INFO("[reset_traj] execution stopped by external command");
  }
}

/// 异常中止：保持末次指令姿态并置 Position.x=0，状态置 kFailed。
/// 不发布目标值，避免中止瞬间给控制器一个大阶跃指令。
void ResetTrajectoryNode::AbortExecution(const char* reason) {
  const double elapsed = (ros::Time::now() - execution_start_time_).toSec();
  const double traveled = max_traveled_;
  ROS_WARN("[reset_traj] ABORT after %.1fs: %s (traveled=%.1f/%.1f deg, "
           "last_cmd=[s=%.1f b=%.1f a=%.1f bk=%.1f])",
           elapsed, reason, traveled, total_swing_travel_,
           last_cmd_swing_deg_, last_cmd_boom_deg_,
           last_cmd_arm_deg_, last_cmd_bucket_deg_);
  if (has_last_cmd_) {
    PublishTrajectoryFrame(last_cmd_swing_deg_, last_cmd_boom_deg_,
                           last_cmd_arm_deg_, last_cmd_bucket_deg_, false);
  }
  timer_.stop();
  phase_ = ResetPhase::kFailed;
}

bool ResetTrajectoryNode::CheckInputsValid() {
  if (!joints_valid_ || !swing_valid_) {
    ROS_WARN("[reset_traj] joints data not received, abort");
    return false;
  }
  if (!goal_valid_) {
    ROS_WARN("[reset_traj] /dig_point not received yet, abort");
    return false;
  }
  return true;
}

/// 执行期输入新鲜度校验：/joints_angle 与 /Swing_topic 均须在
/// input_timeout_sec_ 内有更新，否则视为传感器断流。
/// input_timeout_sec_ <= 0 表示关闭该检查。
bool ResetTrajectoryNode::CheckInputsFresh() {
  if (input_timeout_sec_ <= 0.0) return true;
  const ros::Time now = ros::Time::now();
  const double joints_age = (now - last_joints_time_).toSec();
  const double swing_age = (now - last_swing_time_).toSec();
  if (joints_age > input_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "[reset_traj] /joints_angle stale (%.2fs)",
                      joints_age);
    return false;
  }
  if (swing_age > input_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "[reset_traj] /Swing_topic stale (%.2fs)",
                      swing_age);
    return false;
  }
  return true;
}

/// 物理状态提前结束判定：
///   (truck_safe && bucket_height_gd < RSThreshold(5)) && RealBktPosXYZ(3) < 0
/// truck_safe = 当前回转与卡车中心航向的最短角距 > truck_safe_margin_deg
/// 三个依赖话题任一未接收 → 不启用提前结束（仅靠回转到位判定）
bool ResetTrajectoryNode::IsPhysicalGoalReached() {
  if (!truck_center_valid_ || !bucket_terrain_valid_ || !real_bkt_pos_valid_) {
    if (enable_debug_log_) {
      ROS_INFO_THROTTLE(2.0,
          "[reset_traj][dbg] physical goal check: disabled "
          "(truck=%d terrain=%d bkt=%d)",
          truck_center_valid_, bucket_terrain_valid_, real_bkt_pos_valid_);
    }
    return false;
  }
  // 最短角距（已处理 360° 回绕）
  const double cw_dist = CalculateAngleDiff(truck_center_swing_deg_,
                                            swing_deg_, 1);
  const double ccw_dist = CalculateAngleDiff(truck_center_swing_deg_,
                                             swing_deg_, -1);
  const double angular_dist = std::min(cw_dist, ccw_dist);
  const bool truck_safe = angular_dist > truck_safe_margin_deg_;
  const bool height_ok = bucket_height_gd_ < rs_threshold_5_;
  const bool bucket_z_ok = real_bkt_pos_z_ < 0.0;

  if (enable_debug_log_) {
    ROS_INFO_THROTTLE(1.0,
        "[reset_traj][dbg] physical goal: truck_swing=%.1f cur_swing=%.1f "
        "ang_dist=%.1f (margin=%.1f) safe=%d | height=%.2f (thr=%.1f) ok=%d | "
        "bkt_z=%.2f ok=%d",
        truck_center_swing_deg_, swing_deg_, angular_dist,
        truck_safe_margin_deg_, truck_safe,
        bucket_height_gd_, rs_threshold_5_, height_ok,
        real_bkt_pos_z_, bucket_z_ok);
  }

  return truck_safe && height_ok && bucket_z_ok;
}

/// 更新并返回抗抖动已行程（有副作用：更新 max_traveled_ / last_swing_deg_）
///   方案1：上限截断——超过总行程且超出绝对裕度，视为 360° 回绕伪值
///   方案6：方向一致性——逐帧反向且幅度 < 1° 视为噪声
///   方案3：累计非递减——max_traveled_ 单调递增，拒绝回退
double ResetTrajectoryNode::UpdateFilteredTraveled() {
  const double raw_traveled = CalculateAngleDiff(swing_start_deg_, swing_deg_,
                                                 swing_direction_);

  // 方案1：上限截断。同时要求超过 1.5 倍总行程与 30° 绝对裕度，
  // 避免小行程场景（如总行程 12°）下正常液压超调被误判为回绕伪值
  const double wrap_limit = std::max(total_swing_travel_ * 1.5,
                                     total_swing_travel_ + 30.0);
  if (raw_traveled > wrap_limit) {
    if (enable_debug_log_) {
      ROS_INFO("[reset_traj][dbg] filter: wrap-limit hit "
               "(raw=%.1f > limit=%.1f), freeze at max=%.1f",
               raw_traveled, wrap_limit, max_traveled_);
    }
    last_swing_deg_ = swing_deg_;
    has_last_swing_ = true;
    return max_traveled_;
  }

  // 方案6：方向一致性检查
  if (has_last_swing_) {
    const double frame_cw = CalculateAngleDiff(last_swing_deg_, swing_deg_, 1);
    const double frame_ccw = CalculateAngleDiff(last_swing_deg_, swing_deg_, -1);
    const int frame_direction = (frame_cw <= frame_ccw) ? 1 : -1;
    const double frame_delta = std::min(frame_cw, frame_ccw);
    // 逐帧方向与总方向相反且幅度 < 1° → 传感器噪声，忽略本帧进度更新
    if (frame_direction != swing_direction_ && frame_delta < 1.0) {
      if (enable_debug_log_) {
        ROS_INFO("[reset_traj][dbg] filter: reverse-noise rejected "
                 "(frame_dir=%s delta=%.2f deg), freeze at max=%.1f",
                 frame_direction == 1 ? "CW" : "CCW",
                 frame_delta, max_traveled_);
      }
      last_swing_deg_ = swing_deg_;
      return max_traveled_;
    }
  }

  // 方案3：累计非递减——仅当 raw 超过历史最大值时才更新
  if (raw_traveled > max_traveled_) {
    max_traveled_ = raw_traveled;
  }
  last_swing_deg_ = swing_deg_;
  has_last_swing_ = true;
  return max_traveled_;
}

}  // namespace reset_trajectory_planner
