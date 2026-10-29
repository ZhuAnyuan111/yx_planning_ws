#include "clean_trajectory_planner/clean_trajectory_node.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>

#include <algorithm>
#include <cmath>

#include "clean_trajectory_planner/angle_utils.h"

namespace clean_trajectory_planner {

namespace {
/// /Sys_SLe_FlagGroundLevelExcute 执行值（决策层 StagingCycleFsm::GROUND_LEVEL_EXECUTE 发布）
constexpr double kExecuteValue = 1.0;
/// /Sys_SeqAction 清台阶段指示值（仅日志，不作为触发门控）
constexpr double kSeqActionClean = 7.0;

/// swing 限速逼近（wrap-aware，处理 0°/360° 跨界），返回归一化后的新命令值
double StepSwingTowards(double current, double target, double max_step) {
  const double diff = SignedShortestDiffDeg(current, target);
  if (std::abs(diff) <= max_step) return NormalizeAngle(target);
  return NormalizeAngle(current + (diff > 0.0 ? max_step : -max_step));
}

/// 阶段名（日志可读性，避免打印裸枚举整数）
const char* PhaseName(CleanPhase p) {
  switch (p) {
    case CleanPhase::kIdle:      return "Idle";
    case CleanPhase::kRetract:   return "Retract";
    case CleanPhase::kStripDone: return "StripDone";
    case CleanPhase::kFailed:    return "Failed";
  }
  return "?";
}
}  // namespace

// ==================== 构造函数 ====================

CleanTrajectoryNode::CleanTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  // 先建 solver：LoadParams 中 start_arm_deg 默认值依赖 kinematics 限位
  solver_ = std::make_unique<kinematics::KinematicsSolver>(pnh_, "kinematics");

  LoadParams();
  SetupTopics();

  // 定时器常开：kRetract 执行内收 / kIdle 节流刷新 /clean_digPoint
  timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                           &CleanTrajectoryNode::TimerCallback, this,
                           /*oneshot=*/false, /*autostart=*/true);

  GenerateStrips();          // best-effort：中心线可能未就绪，IdleRefresh 会重试
  PublishCleanFinish(0.0f);  // 初始未完成

  ROS_INFO("[clean] ready. region=[lane-%.1f, lane+%.1f] step=%.1f dir=nearest-boundary | "
           "start(arm=%.1f bkt=%.1f) end_arm_attitude=%.1f | arm_vel=%.1f tol=%.1f confirm=%d | "
           "lane_param=%s height_topic=%s",
           left_offset_deg_, right_offset_deg_, strip_step_deg_,
           start_arm_deg_, start_bucket_deg_, end_arm_attitude_deg_, arm_vel_dps_,
           arm_tolerance_deg_, confirm_frames_, front_lane_param_.c_str(),
           ground_height_topic_.c_str());
}

// ==================== 初始化 ====================

void CleanTrajectoryNode::LoadParams() {
  // 区域/条带
  front_lane_param_ =
      pnh_.param<std::string>("clean/front_lane_param", "/GP2mid/front_lane_deg");
  ground_height_topic_ =
      pnh_.param<std::string>("clean/ground_height_topic", "/Sys_SCo_Traj");
  left_offset_deg_ = pnh_.param("clean/left_offset_deg", 20.0);
  right_offset_deg_ = pnh_.param("clean/right_offset_deg", 20.0);
  strip_step_deg_ = pnh_.param("clean/strip_step_deg", 5.0);
  // 遍历方向不再由参数指定：GenerateStrips 依据当前实时 swing 自动选最近边界为起点

  // 起点姿态 / 内收终点（start_arm 默认取 arm 最大外伸极限 arm_upper）
  start_arm_deg_ = pnh_.param("clean/start_arm_deg", solver_->limits().arm_upper);
  start_bucket_deg_ = pnh_.param("clean/start_bucket_deg", 0.0);
  // end_arm_attitude_deg 为「斗杆姿态角」(boom+arm)，非斗杆关节角
  end_arm_attitude_deg_ = pnh_.param("clean/end_arm_attitude_deg", -60.0);

  // boom 高度反解
  boom_solve_tol_m_ = pnh_.param("clean/boom_solve_tol_m", 0.01);
  boom_solve_max_iter_ = pnh_.param("clean/boom_solve_max_iter", 60);
  height_offset_m_ = pnh_.param("clean/height_offset_m", 0.0);
  height_target_at_pivot_ = pnh_.param("clean/height_target_at_pivot", false);
  height_eps_m_ = pnh_.param("clean/height_eps_m", 0.02);

  // 频率/限速/容差/超时
  publish_rate_ = pnh_.param("publish_rate", 10.0);
  idle_refresh_sec_ = pnh_.param("idle_refresh_sec", 1.0);
  arm_vel_dps_ = pnh_.param("arm_vel_dps", 15.0);
  swing_vel_dps_ = pnh_.param("swing_vel_dps", 15.0);
  arm_tolerance_deg_ = pnh_.param("arm_tolerance_deg", 5.0);
  swing_tolerance_deg_ = pnh_.param("swing_tolerance_deg", 5.0);
  confirm_frames_ = pnh_.param("confirm_frames", 3);
  retract_check_swing_ = pnh_.param("retract_check_swing", false);
  strip_timeout_sec_ = pnh_.param("strip_timeout_sec", 8.0);
  total_timeout_sec_ = pnh_.param("total_timeout_sec", 12.0);
  input_timeout_sec_ = pnh_.param("input_timeout_sec", 1.0);
  enable_debug_log_ = pnh_.param("enable_debug_log", false);

  // 兜底防呆
  if (publish_rate_ <= 0.0) publish_rate_ = 10.0;
  if (idle_refresh_sec_ < 0.05) idle_refresh_sec_ = 0.05;
  if (confirm_frames_ < 1) confirm_frames_ = 1;
  if (strip_step_deg_ <= 1e-6) strip_step_deg_ = 1.0;
  if (boom_solve_max_iter_ < 1) boom_solve_max_iter_ = 60;
  if (boom_solve_tol_m_ <= 0.0) boom_solve_tol_m_ = 0.01;
  if (height_eps_m_ < 0.0) height_eps_m_ = 0.0;
  if (enable_debug_log_) ROS_INFO("[clean] debug log ENABLED");
}

void CleanTrajectoryNode::SetupTopics() {
  sub_joints_ = nh_.subscribe("/joints_angle", 1,
                              &CleanTrajectoryNode::JointsCallback, this);
  sub_swing_ = nh_.subscribe("/heading2swing_topic", 1,
                             &CleanTrajectoryNode::SwingCallback, this);
  sub_height_ = nh_.subscribe(ground_height_topic_, 1,
                              &CleanTrajectoryNode::HeightCallback, this);
  sub_activate_ = nh_.subscribe("/Sys_SLe_FlagGroundLevelExcute", 1,
                                &CleanTrajectoryNode::ActivateCallback, this);
  sub_seq_ = nh_.subscribe("/Sys_SeqAction", 1,
                           &CleanTrajectoryNode::SeqActionCallback, this);

  pub_trajectory_ = nh_.advertise<geometry_msgs::Pose>("/RefDeviceTraj_CleanUp", 1);
  pub_finish_ = nh_.advertise<std_msgs::Float32>("/FlagCleanFinish", 1, /*latch=*/true);
  pub_clean_point_ =
      nh_.advertise<geometry_msgs::Quaternion>("/clean_digPoint", 1, /*latch=*/true);
}

// ==================== 订阅回调 ====================

void CleanTrajectoryNode::JointsCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  boom_deg_ = msg->x;
  arm_deg_ = msg->y;
  bucket_deg_ = msg->z;
  const bool first_frame = !joints_valid_;
  joints_valid_ = true;
  last_joints_time_ = ros::Time::now();
  if (first_frame) {
    ROS_INFO("[clean] first /joints_angle: boom=%.2f arm=%.2f bucket=%.2f deg",
             boom_deg_, arm_deg_, bucket_deg_);
  }
}

void CleanTrajectoryNode::SwingCallback(const std_msgs::Float32::ConstPtr& msg) {
  swing_deg_ = msg->data;
  const bool first_frame = !swing_valid_;
  swing_valid_ = true;
  last_swing_time_ = ros::Time::now();
  if (first_frame) {
    ROS_INFO("[clean] first /heading2swing_topic: swing=%.2f deg", swing_deg_);
  }
}

/// /Sys_SCo_Traj (geometry_msgs/Pose)：position.y = 清台目标地面相对高度 z(m)。
/// 注：该话题既有语义为参考回转角(orientation.x)，本节点只取 position.y。
void CleanTrajectoryNode::HeightCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  const bool first_frame = !height_valid_;
  z_target_ = msg->position.y + height_offset_m_;
  height_valid_ = true;
  last_height_time_ = ros::Time::now();
  if (first_frame) {
    ROS_INFO("[clean] first %s: 清台目标高度 z=%.3f m (raw position.y=%.3f, offset=%.3f)",
             ground_height_topic_.c_str(), z_target_, msg->position.y, height_offset_m_);
  }
}

/// /Sys_SLe_FlagGroundLevelExcute (Float64)：==1.0 执行清台 / ==0.0 停止。
/// 与 reset 节点同模式：以电平信号为主触发，seq_action 仅作并行指示。
void CleanTrajectoryNode::ActivateCallback(const std_msgs::Float64::ConstPtr& msg) {
  const double v = msg->data;
  if (v == kExecuteValue) {
    // 仅空闲态接受激活；kRetract/kStripDone/kFailed 忽略决策层持续重发的 1.0，
    // 防止反复重启同一条带（kStripDone->kIdle 由下方 flag=0 分支处理）。
    if (phase_ == CleanPhase::kIdle) StartRetract();
    return;
  }

  // v == 0.0（或其他）：决策层撤销/结束本阶段
  if (phase_ == CleanPhase::kRetract) {
    StopExecution();
  } else if (phase_ == CleanPhase::kStripDone) {
    phase_ = CleanPhase::kIdle;
    ROS_INFO("[clean] 决策层已确认本条带完成(flag=0) -> kIdle，armed 于条带 %d/%d (swing=%.1f)",
             strip_index_ + 1, static_cast<int>(strips_.size()), CurrentStripAngleDeg());
  } else if (phase_ == CleanPhase::kFailed) {
    phase_ = CleanPhase::kIdle;
    ROS_INFO("[clean] 从异常态恢复 -> kIdle，重试条带 %d/%d",
             strip_index_ + 1, static_cast<int>(strips_.size()));
  }
}

void CleanTrajectoryNode::SeqActionCallback(const std_msgs::Float64::ConstPtr& msg) {
  last_seq_action_ = msg->data;
  if (enable_debug_log_ && msg->data == kSeqActionClean) {
    ROS_INFO_THROTTLE(2.0, "[clean][dbg] /Sys_SeqAction=7.0（清台阶段指示）");
  }
}

// ==================== 定时器与执行 ====================

void CleanTrajectoryNode::TimerCallback(const ros::TimerEvent& /*event*/) {
  switch (phase_) {
    case CleanPhase::kRetract:
      ExecuteRetract();
      break;
    case CleanPhase::kIdle:
      IdleRefresh();
      break;
    case CleanPhase::kStripDone:
    case CleanPhase::kFailed:
    default:
      // 等待决策层撤销(flag=0)后由 ActivateCallback 转回 kIdle
      break;
  }
}

/// armed 空闲态：节流重读中心线、按最新高度刷新当前条带 /clean_digPoint
void CleanTrajectoryNode::IdleRefresh() {
  const ros::Time now = ros::Time::now();
  if (!last_idle_refresh_time_.isZero() &&
      (now - last_idle_refresh_time_).toSec() < idle_refresh_sec_) {
    return;
  }
  last_idle_refresh_time_ = now;

  // 重读中心线：变化（或条带为空）则重新生成
  double theta_c = 0.0;
  if (nh_.getParam(front_lane_param_, theta_c)) {
    if (strips_.empty() || std::abs(theta_c - front_lane_deg_) > 1e-6) {
      GenerateStrips();
    }
  } else {
    ROS_WARN_THROTTLE(5.0, "[clean] 未读到中心线参数 %s", front_lane_param_.c_str());
  }

  if (strips_.empty()) return;

  if (!height_valid_) {
    ROS_WARN_THROTTLE(2.0, "[clean] 等待清台目标高度 %s(position.y)",
                      ground_height_topic_.c_str());
    return;
  }

  // 防抖：仅在未发布过 / 中心线变化 / 高度变化超死区时重算重发
  const bool need = !clean_digpoint_published_ ||
                    std::abs(front_lane_deg_ - published_lane_) > 1e-6 ||
                    std::abs(z_target_ - published_z_) > height_eps_m_;
  if (need) PublishCleanDigPoint();
}

/// kIdle -> kRetract：校验输入与起点姿态就绪后启动本条带内收
void CleanTrajectoryNode::StartRetract() {
  if (!CheckInputsValid()) return;

  if (strips_.empty()) {
    GenerateStrips();
    if (strips_.empty()) {
      ROS_WARN_THROTTLE(2.0, "[clean] 无有效条带（中心线未就绪），拒绝激活");
      return;
    }
  }

  // 起点姿态必须已就绪（复位据此摆位）；否则尝试即时补发，仍失败则拒绝激活
  if (!clean_digpoint_published_) {
    PublishCleanDigPoint();
    if (!clean_digpoint_published_) {
      ROS_WARN_THROTTLE(2.0,
          "[clean] 当前条带起点姿态未就绪（boom 反解失败/高度缺失），拒绝激活");
      return;
    }
  }

  // 命令值从当前实测出发，平滑逼近（swing 保持条带角，斗杆姿态角内收到 end_arm_attitude_deg）
  cmd_swing_deg_ = NormalizeAngle(swing_deg_);
  cmd_arm_deg_ = arm_deg_;
  retract_confirm_count_ = 0;
  dbg_confirm_cnt_ = -1;

  execution_start_time_ = ros::Time::now();
  strip_start_time_ = execution_start_time_;
  last_timer_time_ = execution_start_time_;
  phase_ = CleanPhase::kRetract;

  PublishCleanFinish(0.0f);  // 进入内收：清完成标志

  ROS_INFO("[clean] -> 内收条带 %d/%d：swing=%.1f, 斗杆姿态角->%.1f (当前 boom+arm=%.1f) "
           "(boom/bucket 锁实测, z=%.3f)",
           strip_index_ + 1, static_cast<int>(strips_.size()),
           CurrentStripAngleDeg(), end_arm_attitude_deg_, boom_deg_ + arm_deg_, z_target_);
}

/// kRetract 每帧：限速内收 arm + 流式发布 + 到位判定
void CleanTrajectoryNode::ExecuteRetract() {
  const ros::Time tnow = ros::Time::now();

  const double total_elapsed = (tnow - execution_start_time_).toSec();
  if (total_elapsed > total_timeout_sec_) {
    AbortExecution("total execution timeout");
    return;
  }
  const double strip_elapsed = (tnow - strip_start_time_).toSec();
  if (strip_elapsed > strip_timeout_sec_) {
    AbortExecution("strip retract timeout");
    return;
  }
  if (!CheckInputsFresh()) {
    AbortExecution("input data stale (joints/swing feedback lost)");
    return;
  }

  double dt = (tnow - last_timer_time_).toSec();
  last_timer_time_ = tnow;
  if (dt <= 0.0 || dt > 0.5) dt = 1.0 / publish_rate_;

  // 限速内收 arm：终点为固定「斗杆姿态角」(boom+arm)。因 boom 每帧填实测角且内收中
  //   由高度闭环变化，故斗杆关节角目标须每帧由实测 boom 反推：
  //   arm_joint_target = end_arm_attitude_deg_ − boom_实测，并裁剪到关节限位。
  const kinematics::JointLimitsDeg& lim = solver_->limits();
  const double arm_joint_target = std::max(
      lim.arm_lower, std::min(lim.arm_upper, end_arm_attitude_deg_ - boom_deg_));
  cmd_arm_deg_ = StepTowards(cmd_arm_deg_, arm_joint_target, arm_vel_dps_ * dt);
  const double theta = CurrentStripAngleDeg();
  cmd_swing_deg_ = StepSwingTowards(cmd_swing_deg_, theta, swing_vel_dps_ * dt);

  PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_, true);

  // 到位判定：以实测「斗杆姿态角」(boom+arm) 逼近 end_arm_attitude_deg_ 为主判据
  //   （可选同时校验 swing），连续 confirm_frames 帧
  const double arm_attitude_now = boom_deg_ + arm_deg_;
  const bool arm_ok =
      std::abs(arm_attitude_now - end_arm_attitude_deg_) <= arm_tolerance_deg_;
  const bool swing_ok = !retract_check_swing_ ||
      ShortestAngularDistanceDeg(swing_deg_, theta) <= swing_tolerance_deg_;

  if (enable_debug_log_) {
    ROS_INFO_THROTTLE(0.5,
        "[clean][dbg] retract: fb(sw=%.1f ar=%.1f att=%.1f) tgt(sw=%.1f att=%.1f) "
        "cmd(sw=%.1f ar=%.1f) err(att=%.1f) ok(a=%d s=%d) cnt=%d/%d "
        "dt=%.3f elapsed=%.1f/%.1f",
        swing_deg_, arm_deg_, arm_attitude_now, theta, end_arm_attitude_deg_,
        cmd_swing_deg_, cmd_arm_deg_,
        arm_attitude_now - end_arm_attitude_deg_, arm_ok, swing_ok,
        retract_confirm_count_, confirm_frames_, dt, strip_elapsed, strip_timeout_sec_);
    if (retract_confirm_count_ != dbg_confirm_cnt_) {
      dbg_confirm_cnt_ = retract_confirm_count_;
      ROS_INFO("[clean][dbg] retract confirm %d/%d (arm_ok=%d swing_ok=%d)",
               retract_confirm_count_, confirm_frames_, arm_ok, swing_ok);
    }
  }

  if (arm_ok && swing_ok) {
    if (++retract_confirm_count_ >= confirm_frames_) CompleteStrip();
  } else {
    retract_confirm_count_ = 0;
  }
}

/// 本条带内收到位：发末帧 + 推进/收尾 + 上报 /FlagCleanFinish，转 kStripDone
void CleanTrajectoryNode::CompleteStrip() {
  // 末帧 position.x=0（结束位）
  PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_, false);

  const int done_idx = strip_index_;
  const bool is_last = (strip_index_ >= static_cast<int>(strips_.size()) - 1);
  phase_ = CleanPhase::kStripDone;

  if (!is_last) {
    strip_index_++;
    // 为下一条带反解 boom 并发布 /clean_digPoint（供下次复位摆位）
    PublishCleanDigPoint();
    PublishCleanFinish(1.0f);
    ROS_INFO("[clean] 条带 %d/%d 完成 -> FlagCleanFinish=1.0；"
             "已发布下一条带(%d/%d, swing=%.1f) /clean_digPoint",
             done_idx + 1, static_cast<int>(strips_.size()),
             strip_index_ + 1, static_cast<int>(strips_.size()),
             CurrentStripAngleDeg());
  } else {
    PublishCleanFinish(2.0f);
    ROS_INFO("[clean] 全部 %d 条带完成 -> FlagCleanFinish=2.0（最终收尾）",
             static_cast<int>(strips_.size()));
    // 为下一轮 staging 复位到 strip0：重置进度、重读中心线、重发 strip0 /clean_digPoint
    strip_index_ = 0;
    GenerateStrips();
    PublishCleanDigPoint();
  }
}

// ==================== 条带几何与 boom 反解 ====================

void CleanTrajectoryNode::GenerateStrips() {
  double theta_c = 0.0;
  if (!nh_.getParam(front_lane_param_, theta_c)) {
    ROS_WARN_THROTTLE(5.0, "[clean] 未读到中心线参数 %s，暂不生成条带",
                      front_lane_param_.c_str());
    return;  // 保留旧 strips_
  }
  if (strip_step_deg_ <= 1e-6) strip_step_deg_ = 1.0;

  const double lo = theta_c - left_offset_deg_;
  const double hi = theta_c + right_offset_deg_;
  const int n = static_cast<int>(std::floor((hi - lo) / strip_step_deg_ + 1e-9)) + 1;

  strips_.clear();
  if (n > 0) {
    strips_.reserve(n);
    for (int i = 0; i < n; ++i) strips_.push_back(lo + i * strip_step_deg_);
    // 遍历方向：仅在「全新遍历」(strip_index_==0) 时依据当前实时 swing 决定——
    //   取离当前 swing 最近的区域边界作为起点（hi 更近则从 hi 递减）。
    //   中途重生成(strip_index_>0，如中心线变化)沿用已锁定方向，避免进度错乱。
    if (strip_index_ == 0) {
      if (swing_valid_) {
        const double d_lo = ShortestAngularDistanceDeg(swing_deg_, lo);
        const double d_hi = ShortestAngularDistanceDeg(swing_deg_, hi);
        traverse_descend_ = (d_hi < d_lo);
      } else {
        traverse_descend_ = false;  // 无 swing 反馈：默认从 lo 递增
      }
    }
    if (traverse_descend_) std::reverse(strips_.begin(), strips_.end());
  }
  front_lane_deg_ = theta_c;

  // 保留遍历进度，clamp 到新区间
  if (strip_index_ < 0) strip_index_ = 0;
  if (!strips_.empty() && strip_index_ >= static_cast<int>(strips_.size())) {
    strip_index_ = static_cast<int>(strips_.size()) - 1;
  }

  if (enable_debug_log_) {
    ROS_INFO("[clean][dbg] 生成条带 %d 条：center=%.1f range=[%.1f,%.1f] step=%.1f "
             "dir=%s(cur_swing=%.1f) -> swing[0]=%.1f ... swing[%d]=%.1f",
             static_cast<int>(strips_.size()), theta_c, lo, hi, strip_step_deg_,
             traverse_descend_ ? "hi->lo" : "lo->hi", swing_deg_,
             strips_.empty() ? 0.0 : strips_.front(),
             static_cast<int>(strips_.size()) - 1,
             strips_.empty() ? 0.0 : strips_.back());
  } else {
    ROS_INFO("[clean] 生成清台条带 %d 条（center=%.1f, [%.1f,%.1f], step=%.1f, 方向=%s）",
             static_cast<int>(strips_.size()), theta_c, lo, hi, strip_step_deg_,
             traverse_descend_ ? "hi->lo" : "lo->hi");
  }
}

bool CleanTrajectoryNode::ComputeStripStartPose(int i, double& swing_deg,
                                                double& boom_deg, double& arm_deg,
                                                double& bucket_deg) {
  if (i < 0 || i >= static_cast<int>(strips_.size())) return false;
  if (!height_valid_) {
    ROS_WARN_THROTTLE(2.0, "[clean] 等待清台目标高度 %s(position.y)，暂无法反解 boom",
                      ground_height_topic_.c_str());
    return false;
  }
  double boom = 0.0;
  if (!SolveBoomForHeight(z_target_, start_arm_deg_, start_bucket_deg_, boom)) {
    return false;  // SolveBoomForHeight 内部已打印 ERROR
  }
  swing_deg = NormalizeAngle(strips_[i]);
  boom_deg = boom;
  arm_deg = start_arm_deg_;
  bucket_deg = start_bucket_deg_;
  return true;
}

bool CleanTrajectoryNode::SolveBoomForHeight(double z_target, double arm_deg,
                                             double bucket_deg, double& boom_deg) {
  const kinematics::LinkGeometry& g = solver_->geometry();
  const kinematics::JointLimitsDeg& lim = solver_->limits();
  const double arm_rad = Deg2Rad(arm_deg);
  const double bkt_rad = Deg2Rad(bucket_deg);
  // 齿尖高度 FK（与 swing 无关）：
  //   z = pivot_z + Lb·sin(boom) + La·sin(boom+arm) [+ Lbt·sin(boom+arm+bucket)]
  // height_target_at_pivot=true 时目标点取斗杆-铲斗铰接点（齿长项置 0）
  const double tooth_len = height_target_at_pivot_ ? 0.0 : g.bucket_tooth_length;
  auto fz = [&](double boom_rad) {
    return g.boom_pivot_z +
           g.boom_length * std::sin(boom_rad) +
           g.arm_length * std::sin(boom_rad + arm_rad) +
           tooth_len * std::sin(boom_rad + arm_rad + bkt_rad);
  };

  const double lo = lim.boom_lower;
  const double hi = lim.boom_upper;
  const double z_lo = fz(Deg2Rad(lo));
  const double z_hi = fz(Deg2Rad(hi));
  double f_lo = z_lo - z_target;
  const double f_hi = z_hi - z_target;

  // 端点同号 → z 超出该 arm/bucket 下 boom 限位可达高度范围 → 无解
  if (f_lo * f_hi > 0.0) {
    ROS_ERROR("[clean] boom 反解失败：目标高度 z=%.3f 超出可达范围 [%.3f, %.3f] "
              "(arm=%.1f bucket=%.1f, boom∈[%.1f,%.1f], pivot=%s)",
              z_target, z_lo, z_hi, arm_deg, bucket_deg, lo, hi,
              height_target_at_pivot_ ? "铰接点" : "齿尖");
    return false;
  }

  // 二分：FK z 对 boom 单调，端点已夹逼出根
  double a = lo, b = hi;
  for (int it = 0; it < boom_solve_max_iter_; ++it) {
    const double m = 0.5 * (a + b);
    const double fm = fz(Deg2Rad(m)) - z_target;
    if (std::abs(fm) <= boom_solve_tol_m_ || (b - a) < 1e-7) {
      boom_deg = m;
      if (enable_debug_log_) {
        ROS_INFO("[clean][dbg] boom 反解收敛：z=%.3f -> boom=%.2f (iter=%d, 残差=%.4f)",
                 z_target, boom_deg, it, fm);
      }
      return true;
    }
    if (f_lo * fm <= 0.0) {
      b = m;
    } else {
      a = m;
      f_lo = fm;
    }
  }
  boom_deg = 0.5 * (a + b);  // 迭代上限兜底（区间已极小）
  if (enable_debug_log_) {
    ROS_INFO("[clean][dbg] boom 反解达迭代上限：z=%.3f -> boom=%.2f", z_target, boom_deg);
  }
  return true;
}

void CleanTrajectoryNode::PublishCleanDigPoint() {
  double sw = 0.0, boom = 0.0, arm = 0.0, bkt = 0.0;
  if (!ComputeStripStartPose(strip_index_, sw, boom, arm, bkt)) {
    clean_digpoint_published_ = false;
    return;  // ComputeStripStartPose 内部已打印原因
  }
  geometry_msgs::Quaternion msg;
  msg.x = sw;    // swing
  msg.y = boom;  // boom
  msg.z = arm;   // arm
  msg.w = bkt;   // bucket
  pub_clean_point_.publish(msg);

  clean_digpoint_published_ = true;
  start_swing_deg_ = sw;
  start_boom_deg_ = boom;
  published_lane_ = front_lane_deg_;
  published_z_ = z_target_;

  ROS_INFO("[clean] /clean_digPoint 条带 %d/%d: swing=%.1f boom=%.1f arm=%.1f "
           "bucket=%.1f (z=%.3f)",
           strip_index_ + 1, static_cast<int>(strips_.size()), sw, boom, arm, bkt,
           z_target_);
}

// ==================== 工具方法 ====================

void CleanTrajectoryNode::PublishTrajectoryFrame(double swing, double boom,
                                                 double arm, double bucket,
                                                 bool running) {
  geometry_msgs::Pose msg;
  msg.position.x = running ? 1.0 : 0.0;
  msg.position.y = 0.0;
  msg.position.z = 0.0;
  msg.orientation.x = swing;
  msg.orientation.y = boom;
  msg.orientation.z = arm;
  msg.orientation.w = bucket;
  pub_trajectory_.publish(msg);
}

void CleanTrajectoryNode::PublishCleanFinish(float value) {
  std_msgs::Float32 msg;
  msg.data = value;
  pub_finish_.publish(msg);
  clean_finish_value_ = value;
}

double CleanTrajectoryNode::StepTowards(double current, double target,
                                        double max_step) const {
  const double diff = target - current;
  if (std::abs(diff) <= max_step) return target;
  return current + (diff > 0.0 ? max_step : -max_step);
}

double CleanTrajectoryNode::CurrentStripAngleDeg() const {
  if (strips_.empty() || strip_index_ < 0 ||
      strip_index_ >= static_cast<int>(strips_.size())) {
    return 0.0;
  }
  return NormalizeAngle(strips_[strip_index_]);
}

/// 外部撤销(flag=0)：保持末次命令并置结束位（不跳向目标），回 kIdle 重试同一条带
void CleanTrajectoryNode::StopExecution() {
  PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_, false);
  PublishCleanFinish(0.0f);
  phase_ = CleanPhase::kIdle;
  ROS_INFO("[clean] 内收被外部撤销(flag=0)，保持末指令并回 kIdle（条带 %d/%d 不推进）",
           strip_index_ + 1, static_cast<int>(strips_.size()));
}

/// 异常中止：保持末次命令姿态并置 position.x=0，/FlagCleanFinish=0.0，状态置 kFailed。
/// 不跳向目标，避免中止瞬间给控制器大阶跃；等待决策层撤销(flag=0)后恢复 kIdle 重试。
void CleanTrajectoryNode::AbortExecution(const char* reason) {
  const double elapsed = (ros::Time::now() - execution_start_time_).toSec();
  ROS_WARN("[clean] ABORT after %.1fs: %s (phase=%s, strip=%d/%d, "
           "fb[sw=%.1f ar=%.1f] cmd[sw=%.1f ar=%.1f])",
           elapsed, reason, PhaseName(phase_), strip_index_ + 1,
           static_cast<int>(strips_.size()), swing_deg_, arm_deg_,
           cmd_swing_deg_, cmd_arm_deg_);
  PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_, false);
  PublishCleanFinish(0.0f);
  phase_ = CleanPhase::kFailed;
}

bool CleanTrajectoryNode::CheckInputsValid() {
  if (!joints_valid_ || !swing_valid_) {
    ROS_WARN_THROTTLE(2.0, "[clean] waiting feedback: joints=%d swing=%d",
                      joints_valid_, swing_valid_);
    return false;
  }
  if (input_timeout_sec_ > 0.0) {
    const ros::Time now = ros::Time::now();
    if ((now - last_joints_time_).toSec() > input_timeout_sec_) {
      ROS_WARN_THROTTLE(2.0, "[clean] /joints_angle stale");
      return false;
    }
    if ((now - last_swing_time_).toSec() > input_timeout_sec_) {
      ROS_WARN_THROTTLE(2.0, "[clean] /heading2swing_topic stale");
      return false;
    }
  }
  return true;
}

bool CleanTrajectoryNode::CheckInputsFresh() {
  if (input_timeout_sec_ <= 0.0) return true;
  const ros::Time now = ros::Time::now();
  const double joints_age = (now - last_joints_time_).toSec();
  const double swing_age = (now - last_swing_time_).toSec();
  if (joints_age > input_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "[clean] /joints_angle stale (%.2fs)", joints_age);
    return false;
  }
  if (swing_age > input_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "[clean] /heading2swing_topic stale (%.2fs)", swing_age);
    return false;
  }
  return true;
}

}  // namespace clean_trajectory_planner
