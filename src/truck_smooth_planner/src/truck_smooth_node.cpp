#include "truck_smooth_planner/truck_smooth_node.h"

#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <cmath>

#include "truck_smooth_planner/angle_utils.h"

namespace truck_smooth_planner {

namespace {
/// /Sys_SeqAction 抹平激活值（决策层 DiggingCycleFsm::LEVEL_TRUCK 发布）
constexpr double kActivateValue = 6.0;

/// swing 限速逼近（wrap-aware，处理 0°/360° 跨界），返回归一化后的新命令值
double StepSwingTowards(double current, double target, double max_step) {
  const double diff = SignedShortestDiffDeg(current, target);
  if (std::abs(diff) <= max_step) return NormalizeAngle(target);
  return NormalizeAngle(current + (diff > 0.0 ? max_step : -max_step));
}

/// 阶段名（日志可读性，避免打印裸枚举整数）
const char* PhaseName(SmoothPhase p) {
  switch (p) {
    case SmoothPhase::kIdle:   return "Idle";
    case SmoothPhase::kPhase1: return "Phase1";
    case SmoothPhase::kPhase2: return "Phase2";
    case SmoothPhase::kDone:   return "Done";
    case SmoothPhase::kFailed: return "Failed";
  }
  return "?";
}
}  // namespace

// ==================== 构造函数 ====================

TruckSmoothNode::TruckSmoothNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  LoadParams();

  solver_ = std::make_unique<kinematics::KinematicsSolver>(pnh_, "kinematics");

  SetupTopics();

  timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                           &TruckSmoothNode::TimerCallback, this,
                           /*oneshot=*/false, /*autostart=*/false);

  PublishFinish(false);

  ROS_INFO("[truck_smooth] ready. box_len=%.2f inset=%.2f height_off=%.2f "
           "bias=%.2f bkt=%.1f deg | vel(swing/arm)=%.1f/%.1f dps | "
           "tol(swing/arm)=%.1f/%.1f deg confirm=%d",
           truck_box_length_, box_inset_, truck_height_offset_,
           smooth_height_bias_, bucket_angle_deg_, swing_vel_dps_, arm_vel_dps_,
           swing_tolerance_deg_, arm_tolerance_deg_, confirm_frames_);
}

// ==================== 初始化 ====================

void TruckSmoothNode::LoadParams() {
  // 抹平范围/几何
  truck_height_offset_ = pnh_.param("smooth/truck_height_offset", 0.5);
  smooth_height_bias_ = pnh_.param("smooth/smooth_height_bias", 0.0);
  truck_box_length_ = pnh_.param("smooth/truck_box_length", 5.0);
  box_inset_ = pnh_.param("smooth/box_inset", 0.3);
  bucket_angle_deg_ = pnh_.param("smooth/bucket_angle_deg", 25.0);
  use_pivot_as_target_ = pnh_.param("smooth/use_pivot_as_target", true);
  shrink_step_m_ = pnh_.param("smooth/shrink_step_m", 0.1);
  min_smooth_span_m_ = pnh_.param("smooth/min_smooth_span_m", 0.5);

  // 发布频率与限速
  publish_rate_ = pnh_.param("publish_rate", 10.0);
  swing_vel_dps_ = pnh_.param("swing_vel_dps", 15.0);
  arm_vel_dps_ = pnh_.param("arm_vel_dps", 15.0);

  // 容差与确认帧数
  swing_tolerance_deg_ = pnh_.param("swing_tolerance_deg", 5.0);
  arm_tolerance_deg_ = pnh_.param("arm_tolerance_deg", 5.0);
  confirm_frames_ = pnh_.param("confirm_frames", 3);
  phase2_check_swing_ = pnh_.param("phase2_check_swing", true);

  // 超时
  phase1_timeout_sec_ = pnh_.param("phase1_timeout_sec", 8.0);
  phase2_timeout_sec_ = pnh_.param("phase2_timeout_sec", 10.0);
  total_timeout_sec_ = pnh_.param("total_timeout_sec", 13.0);
  input_timeout_sec_ = pnh_.param("input_timeout_sec", 1.0);

  enable_debug_log_ = pnh_.param("enable_debug_log", false);

  if (publish_rate_ <= 0.0) publish_rate_ = 10.0;
  if (confirm_frames_ < 1) confirm_frames_ = 1;
  if (shrink_step_m_ <= 0.0) shrink_step_m_ = 0.1;
  if (min_smooth_span_m_ < 0.0) min_smooth_span_m_ = 0.0;
  if (enable_debug_log_) ROS_INFO("[truck_smooth] debug log ENABLED");
}

void TruckSmoothNode::SetupTopics() {
  sub_joints_ = nh_.subscribe("/joints_angle", 1,
                              &TruckSmoothNode::JointsCallback, this);
  sub_swing_ = nh_.subscribe("/heading2swing_topic", 1,
                             &TruckSmoothNode::SwingCallback, this);
  sub_truck_pose_ = nh_.subscribe("/truck_center_unloadpoint", 1,
                                  &TruckSmoothNode::TruckPoseCallback, this);
  sub_activate_ = nh_.subscribe("/Sys_SeqAction", 1,
                                &TruckSmoothNode::ActivateCallback, this);

  pub_trajectory_ = nh_.advertise<geometry_msgs::Pose>(
      "/RefDeviceTraj_Smooth", 1);
  pub_finish_ = nh_.advertise<std_msgs::Bool>("/FlagSmoothFinish", 1, true);
}

// ==================== 订阅回调 ====================

void TruckSmoothNode::JointsCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  boom_deg_ = msg->x;
  arm_deg_ = msg->y;
  bucket_deg_ = msg->z;
  const bool first_frame = !joints_valid_;
  joints_valid_ = true;
  last_joints_time_ = ros::Time::now();
  if (first_frame) {
    ROS_INFO("[truck_smooth] first /joints_angle: boom=%.2f arm=%.2f "
             "bucket=%.2f deg", boom_deg_, arm_deg_, bucket_deg_);
  }
}

void TruckSmoothNode::SwingCallback(const std_msgs::Float32::ConstPtr& msg) {
  swing_deg_ = msg->data;
  const bool first_frame = !swing_valid_;
  swing_valid_ = true;
  last_swing_time_ = ros::Time::now();
  if (first_frame) {
    ROS_INFO("[truck_smooth] first /heading2swing_topic: swing=%.2f deg",
             swing_deg_);
  }
}

void TruckSmoothNode::TruckPoseCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  // x/y/z = 卡车中心坐标（z 为 RTK 高度）；w = RTK 方位角 α (deg, 北0顺时针)
  // base 系卡车航向 β = (180 − α) mod 360 → rad
  const bool first_frame = !truck_pose_valid_;
  truck_center_x_ = msg->x;
  truck_center_y_ = msg->y;
  truck_center_z_ = msg->z;
  truck_rtk_heading_deg_ = msg->w;
  double beta_deg = std::fmod(180.0 - msg->w, 360.0);
  if (beta_deg < 0.0) beta_deg += 360.0;
  truck_yaw_rad_ = Deg2Rad(beta_deg);
  truck_pose_valid_ = true;
  last_truck_time_ = ros::Time::now();

  if (first_frame) {
    ROS_INFO("[truck_smooth] first /truck_center_unloadpoint: (%.2f, %.2f, "
             "%.2f) m, rtk_heading=%.2f deg -> yaw_base=%.2f deg",
             truck_center_x_, truck_center_y_, truck_center_z_,
             truck_rtk_heading_deg_, Rad2Deg(truck_yaw_rad_));
  }
}

/// /Sys_SeqAction (std_msgs/Float64): data==6.0 激活抹平；其余值停止/复位
void TruckSmoothNode::ActivateCallback(const std_msgs::Float64::ConstPtr& msg) {
  if (msg->data != kActivateValue) {
    if (phase_ == SmoothPhase::kDone) {
      // 抹平完成后跳出本阶段：完成标志复位，回到空闲
      PublishFinish(false);
      phase_ = SmoothPhase::kIdle;
      ROS_INFO("[truck_smooth] smooth stage exited, finish flag reset to false");
    } else if (phase_ != SmoothPhase::kIdle) {
      StopExecution();
    }
    return;
  }

  // 仅空闲态接受激活；执行中/已完成/已失败均忽略重复触发。
  // 防止完成后或 L3 失败后被决策层持续重发的 6.0 反复重启/刷屏；
  // kDone/kFailed 回到 kIdle 的转换由上方非激活分支（seq_action!=6.0）处理。
  if (phase_ != SmoothPhase::kIdle) return;

  ROS_INFO("[truck_smooth] activation received (Sys_SeqAction=6)");

  if (!CheckInputsValid()) return;

  // 由卡车位姿计算抹平起终点 IK 目标（含降级：L1 端点收缩 / L2 最大扫掠，
  // 两者均仍正常执行抹平）。输入暂缺或 L3 整体不可达时返回 false，
  // 精确原因与状态（kIdle 重试 / kFailed 上报）已在内部处理。
  if (!ComputeSmoothTargets()) return;

  // 初始化命令值为当前实时位置，从原地平滑逼近起点
  cmd_swing_deg_ = NormalizeAngle(swing_deg_);
  cmd_arm_deg_ = arm_deg_;
  phase1_confirm_count_ = 0;
  phase2_confirm_count_ = 0;
  dbg_p1_cnt_ = -1;
  dbg_p2_cnt_ = -1;

  execution_start_time_ = ros::Time::now();
  phase_start_time_ = execution_start_time_;
  last_timer_time_ = execution_start_time_;
  phase_ = SmoothPhase::kPhase1;

  PublishFinish(false);
  timer_.start();

  ROS_INFO("[truck_smooth] -> Phase1: move to tail start (swing=%.1f arm=%.1f) "
           "| Phase2 target head (swing=%.1f arm=%.1f)",
           swing_start_deg_, arm_start_deg_, swing_end_deg_, arm_end_deg_);
}

// ==================== 几何与 IK ====================

bool TruckSmoothNode::ComputeSmoothTargets() {
  if (!truck_pose_valid_) {
    ROS_WARN_THROTTLE(2.0,
        "[truck_smooth] truck pose not received, cannot compute targets");
    return false;
  }

  const double dir_x = std::cos(truck_yaw_rad_);
  const double dir_y = std::sin(truck_yaw_rad_);
  const double half = truck_box_length_ * 0.5 - box_inset_;

  // 车厢上表面高度 + 刮削裕量
  const double z_top = truck_center_z_ - truck_height_offset_;
  const double z = z_top + smooth_height_bias_;

  if (enable_debug_log_) {
    ROS_INFO("[truck_smooth][dbg] compute targets: center(%.2f,%.2f,%.2f) "
             "yaw_base=%.2f deg dir(%.3f,%.3f) half=%.2f z_top=%.2f z=%.2f | "
             "fb sw=%.1f boom=%.1f ar=%.1f bkt=%.1f",
             truck_center_x_, truck_center_y_, truck_center_z_,
             Rad2Deg(truck_yaw_rad_), dir_x, dir_y, half, z_top, z,
             swing_deg_, boom_deg_, arm_deg_, bucket_deg_);
  }

  // 抹平起终点用「沿 dir 相对中心的有符号偏移 s」参数化：
  //   车尾起点 s=-half（Phase1 目标），车头终点 s=+half（Phase2 目标）
  double s_tail = -half, s_head = +half;
  double sw_start = 0.0, ar_start = 0.0;
  double sw_end = 0.0, ar_end = 0.0;

  // ---------- 首选 + 主降级：端点距离收缩重试 ----------
  // tail 从 -half 朝中心收缩，head 从 +half 朝中心收缩，各取首个可达点。
  // 因 s_tail∈[-half,0]、s_head∈[0,half]，恒有 s_tail<=s_head。
  const bool tail_ok =
      ShrinkProbe(-half, z, dir_x, dir_y, s_tail, sw_start, ar_start);
  const bool head_ok =
      ShrinkProbe(+half, z, dir_x, dir_y, s_head, sw_end, ar_end);

  if (tail_ok && head_ok && (s_head - s_tail) >= min_smooth_span_m_) {
    const bool full_span =
        (std::abs(s_tail + half) < 1e-6) && (std::abs(s_head - half) < 1e-6);
    if (full_span) {
      ROS_INFO("[truck_smooth] IK ok at full box span [-%.2f, %.2f] m, z=%.2f",
               half, half, z);
    } else {
      ROS_WARN("[truck_smooth] 端点收缩降级：抹平区间缩至 [%.2f, %.2f] m"
               "（原 [-%.2f, %.2f]，跨度 %.2f m），z=%.2f",
               s_tail, s_head, -half, half, s_head - s_tail, z);
    }
    swing_start_deg_ = NormalizeAngle(sw_start);
    arm_start_deg_ = ar_start;
    swing_end_deg_ = NormalizeAngle(sw_end);
    arm_end_deg_ = ar_end;
    LogTargets(z);
    return true;
  }

  // ---------- 兜底：最大能力扫掠 ----------
  // 端点收缩失败（某侧从端点到中心整体不可达），改为在整段密采样，
  // 取可达点中 s 最小/最大者作为起终点，做挖机能力范围内的最大幅度扫掠。
  ROS_WARN("[truck_smooth] 端点收缩失败(tail_ok=%d head_ok=%d)，"
           "降级为最大能力扫掠", tail_ok, head_ok);
  double sw_t2 = 0.0, ar_t2 = 0.0, sw_h2 = 0.0, ar_h2 = 0.0;
  if (MaxSweepProbe(half, z, dir_x, dir_y, s_tail, s_head,
                    sw_t2, ar_t2, sw_h2, ar_h2)) {
    ROS_WARN("[truck_smooth] 最大能力扫掠区间 [%.2f, %.2f] m（跨度 %.2f m），"
             "z=%.2f；原始车厢头尾点超出可达域，请现场复核卡车停靠位姿",
             s_tail, s_head, s_head - s_tail, z);
    swing_start_deg_ = NormalizeAngle(sw_t2);
    arm_start_deg_ = ar_t2;
    swing_end_deg_ = NormalizeAngle(sw_h2);
    arm_end_deg_ = ar_h2;
    LogTargets(z);
    return true;
  }

  // ---------- 最终保底：z 平面整体不可达 → 不谎报成功 ----------
  // 收缩(L1)/扫掠(L2)均失败：车厢上表面整体超出可达域。
  // 此时绝不进入两阶段流程——否则起终点=当前位姿会瞬间判定到位、
  // 谎报 FlagSmoothFinish=true（“假成功”），让决策层误以为已抹平。
  // 处置：上报未完成(false) + 置 kFailed，由决策层超时兜底，等待现场调整车位。
  ROS_ERROR("[truck_smooth] 车厢上表面 z=%.2f 整体超出可达域，无法有效抹平；"
            "拒绝执行并上报未完成(FlagSmoothFinish=false)。"
            "请现场复核卡车停靠位置/高度", z);
  PublishFinish(false);
  phase_ = SmoothPhase::kFailed;
  return false;
}

void TruckSmoothNode::LogTargets(double z) {
  if (!enable_debug_log_) return;
  ROS_INFO("[truck_smooth][dbg] z=%.2f | start(sw=%.1f ar=%.1f) "
           "end(sw=%.1f ar=%.1f)",
           z, swing_start_deg_, arm_start_deg_, swing_end_deg_, arm_end_deg_);
}

bool TruckSmoothNode::SolvePointIk(double x, double y, double z,
                                   double& swing_deg, double& arm_deg,
                                   bool verbose) {
  kinematics::SwingBaseIkRequest req;
  req.target_pose.x = x;
  req.target_pose.y = y;
  req.target_pose.z = z;
  req.target_pose.alpha = 0.0;
  req.bucket_angle_deg = bucket_angle_deg_;
  // use_pivot_as_target: 目标点为斗杆-铲斗铰接点（override=0）；否则为齿尖
  if (use_pivot_as_target_) req.bucket_tooth_length_override = 0.0;

  const auto ik = solver_->swing_center_inverse_by_bucket_angle(req);
  if (!ik.success) {
    if (verbose) {
      ROS_ERROR("[truck_smooth] IK failed at (%.2f, %.2f, %.2f): %s",
                x, y, z, ik.message.c_str());
    }
    return false;
  }
  swing_deg = Rad2Deg(ik.q.swing);
  arm_deg = Rad2Deg(ik.q.arm);
  return true;
}

bool TruckSmoothNode::SolveOffsetIk(double s, double z,
                                    double dir_x, double dir_y,
                                    double& swing_deg, double& arm_deg) {
  const double x = truck_center_x_ + s * dir_x;
  const double y = truck_center_y_ + s * dir_y;
  // 探测期间静默（不打印 ROS_ERROR），避免重试刷屏
  return SolvePointIk(x, y, z, swing_deg, arm_deg, /*verbose=*/false);
}

bool TruckSmoothNode::ShrinkProbe(double s_start, double z,
                                  double dir_x, double dir_y,
                                  double& s_out,
                                  double& swing_deg, double& arm_deg) {
  const double sign = (s_start >= 0.0) ? 1.0 : -1.0;
  double s_abs = std::abs(s_start);
  int steps = 0;
  while (true) {
    const double s = sign * s_abs;
    double sw = 0.0, ar = 0.0;
    if (SolveOffsetIk(s, z, dir_x, dir_y, sw, ar)) {
      s_out = s;
      swing_deg = sw;
      arm_deg = ar;
      if (enable_debug_log_) {
        ROS_INFO("[truck_smooth][dbg] shrink probe: s_start=%.2f -> s=%.2f "
                 "(steps=%d) sw=%.1f ar=%.1f", s_start, s, steps, sw, ar);
      }
      return true;
    }
    if (s_abs <= 1e-9) break;             // 已收缩到中心仍不可达
    s_abs = std::max(0.0, s_abs - shrink_step_m_);
    ++steps;
  }
  if (enable_debug_log_) {
    ROS_INFO("[truck_smooth][dbg] shrink probe FAILED: s_start=%.2f, "
             "no reachable point from end to center", s_start);
  }
  return false;
}

bool TruckSmoothNode::MaxSweepProbe(double half, double z,
                                    double dir_x, double dir_y,
                                    double& s_tail_out, double& s_head_out,
                                    double& sw_tail, double& ar_tail,
                                    double& sw_head, double& ar_head) {
  bool any = false;
  double s_min = 0.0, s_max = 0.0;
  double sw_min = 0.0, ar_min = 0.0, sw_max = 0.0, ar_max = 0.0;
  int n_total = 0, n_ok = 0;
  // 在 [-half, half] 以 shrink_step_m_ 密采样，收集可达点中 s 最小/最大者
  for (double s = -half; s <= half + 1e-9; s += shrink_step_m_) {
    ++n_total;
    double sw = 0.0, ar = 0.0;
    if (!SolveOffsetIk(s, z, dir_x, dir_y, sw, ar)) continue;
    ++n_ok;
    if (!any) {
      s_min = s_max = s;
      sw_min = sw_max = sw;
      ar_min = ar_max = ar;
      any = true;
    } else {
      if (s < s_min) { s_min = s; sw_min = sw; ar_min = ar; }
      if (s > s_max) { s_max = s; sw_max = sw; ar_max = ar; }
    }
  }
  if (enable_debug_log_) {
    ROS_INFO("[truck_smooth][dbg] max sweep: samples=%d reachable=%d "
             "s_range=[%.2f, %.2f] span=%.2f (need>=%.2f)",
             n_total, n_ok, any ? s_min : 0.0, any ? s_max : 0.0,
             any ? (s_max - s_min) : 0.0, min_smooth_span_m_);
  }
  if (any && (s_max - s_min) >= min_smooth_span_m_) {
    s_tail_out = s_min;
    s_head_out = s_max;
    sw_tail = sw_min;
    ar_tail = ar_min;
    sw_head = sw_max;
    ar_head = ar_max;
    return true;
  }
  return false;
}

// ==================== 定时器与阶段执行 ====================

void TruckSmoothNode::TimerCallback(const ros::TimerEvent& /*event*/) {
  if (phase_ != SmoothPhase::kPhase1 && phase_ != SmoothPhase::kPhase2) {
    timer_.stop();
    return;
  }

  // 全局超时（覆盖两阶段，防卡死）
  const double total_elapsed =
      (ros::Time::now() - execution_start_time_).toSec();
  if (total_elapsed > total_timeout_sec_) {
    AbortExecution("total execution timeout");
    return;
  }

  // 输入新鲜度（joints/swing 反馈断流保护）
  if (!CheckInputsFresh()) {
    AbortExecution("input data stale (joints/swing feedback lost)");
    return;
  }

  switch (phase_) {
    case SmoothPhase::kPhase1:
      ExecutePhase1();
      break;
    case SmoothPhase::kPhase2:
      ExecutePhase2();
      break;
    default:
      break;
  }
}

/// 阶段1：从当前实时位置限速逼近车尾起点 (swing_start, arm_start)
void TruckSmoothNode::ExecutePhase1() {
  const double elapsed = (ros::Time::now() - phase_start_time_).toSec();
  if (elapsed > phase1_timeout_sec_) {
    AbortExecution("phase1 (enter tail start) timeout");
    return;
  }

  const ros::Time tnow = ros::Time::now();
  double dt = (tnow - last_timer_time_).toSec();
  last_timer_time_ = tnow;
  if (dt <= 0.0 || dt > 0.5) dt = 1.0 / publish_rate_;

  // 限速逼近（boom/bucket 填当前实测角）
  cmd_swing_deg_ = StepSwingTowards(cmd_swing_deg_, swing_start_deg_,
                                    swing_vel_dps_ * dt);
  cmd_arm_deg_ = StepTowards(cmd_arm_deg_, arm_start_deg_, arm_vel_dps_ * dt);

  PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_,
                         true);

  // 到位判定：以实时反馈角为准，连续 confirm_frames 帧进入容差
  const bool swing_ok =
      ShortestAngularDistanceDeg(swing_deg_, swing_start_deg_) <=
      swing_tolerance_deg_;
  const bool arm_ok =
      std::abs(arm_deg_ - arm_start_deg_) <= arm_tolerance_deg_;

  if (enable_debug_log_) {
    ROS_INFO_THROTTLE(0.5,
        "[truck_smooth][dbg] P1: fb(sw=%.1f ar=%.1f) tgt(sw=%.1f ar=%.1f) "
        "cmd(sw=%.1f ar=%.1f) err(sw=%.1f ar=%.1f) ok(s=%d a=%d) cnt=%d/%d "
        "dt=%.3f elapsed=%.1f/%.1f",
        swing_deg_, arm_deg_, swing_start_deg_, arm_start_deg_,
        cmd_swing_deg_, cmd_arm_deg_,
        SignedShortestDiffDeg(swing_deg_, swing_start_deg_),
        arm_deg_ - arm_start_deg_, swing_ok, arm_ok,
        phase1_confirm_count_, confirm_frames_,
        dt, elapsed, phase1_timeout_sec_);
    if (phase1_confirm_count_ != dbg_p1_cnt_) {
      dbg_p1_cnt_ = phase1_confirm_count_;
      ROS_INFO("[truck_smooth][dbg] P1 confirm %d/%d (swing_ok=%d arm_ok=%d)",
               phase1_confirm_count_, confirm_frames_, swing_ok, arm_ok);
    }
  }

  if (swing_ok && arm_ok) {
    if (++phase1_confirm_count_ >= confirm_frames_) {
      // 进入阶段2
      phase_ = SmoothPhase::kPhase2;
      phase_start_time_ = ros::Time::now();
      last_timer_time_ = phase_start_time_;
      phase2_confirm_count_ = 0;
      ROS_INFO("[truck_smooth] -> Phase2: push smooth to head end "
               "(swing=%.1f arm=%.1f)", swing_end_deg_, arm_end_deg_);
    }
  } else {
    phase1_confirm_count_ = 0;
  }
}

/// 阶段2：限速推抹至车头终点 (swing_end, arm_end)
void TruckSmoothNode::ExecutePhase2() {
  const double elapsed = (ros::Time::now() - phase_start_time_).toSec();
  if (elapsed > phase2_timeout_sec_) {
    AbortExecution("phase2 (push to head end) timeout");
    return;
  }

  const ros::Time tnow = ros::Time::now();
  double dt = (tnow - last_timer_time_).toSec();
  last_timer_time_ = tnow;
  if (dt <= 0.0 || dt > 0.5) dt = 1.0 / publish_rate_;

  cmd_swing_deg_ = StepSwingTowards(cmd_swing_deg_, swing_end_deg_,
                                    swing_vel_dps_ * dt);
  cmd_arm_deg_ = StepTowards(cmd_arm_deg_, arm_end_deg_, arm_vel_dps_ * dt);

  PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_,
                         true);

  // 到位判定：arm 为主判据（可选同时校验 swing）
  const bool arm_ok =
      std::abs(arm_deg_ - arm_end_deg_) <= arm_tolerance_deg_;
  const bool swing_ok =
      !phase2_check_swing_ ||
      ShortestAngularDistanceDeg(swing_deg_, swing_end_deg_) <=
          swing_tolerance_deg_;

  if (enable_debug_log_) {
    ROS_INFO_THROTTLE(0.5,
        "[truck_smooth][dbg] P2: fb(sw=%.1f ar=%.1f) tgt(sw=%.1f ar=%.1f) "
        "cmd(sw=%.1f ar=%.1f) err(sw=%.1f ar=%.1f) ok(s=%d a=%d) cnt=%d/%d "
        "dt=%.3f elapsed=%.1f/%.1f",
        swing_deg_, arm_deg_, swing_end_deg_, arm_end_deg_,
        cmd_swing_deg_, cmd_arm_deg_,
        SignedShortestDiffDeg(swing_deg_, swing_end_deg_),
        arm_deg_ - arm_end_deg_, swing_ok, arm_ok,
        phase2_confirm_count_, confirm_frames_,
        dt, elapsed, phase2_timeout_sec_);
    if (phase2_confirm_count_ != dbg_p2_cnt_) {
      dbg_p2_cnt_ = phase2_confirm_count_;
      ROS_INFO("[truck_smooth][dbg] P2 confirm %d/%d (swing_ok=%d arm_ok=%d)",
               phase2_confirm_count_, confirm_frames_, swing_ok, arm_ok);
    }
  }

  if (arm_ok && swing_ok) {
    if (++phase2_confirm_count_ >= confirm_frames_) {
      // 抹平完成：发末帧 Position.x=0 + FlagSmoothFinish=true
      PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_,
                             bucket_deg_, false);
      PublishFinish(true);
      timer_.stop();
      phase_ = SmoothPhase::kDone;
      ROS_INFO("[truck_smooth] smooth FINISHED (head end reached), "
               "FlagSmoothFinish=true");
    }
  } else {
    phase2_confirm_count_ = 0;
  }
}

// ==================== 工具方法 ====================

void TruckSmoothNode::PublishTrajectoryFrame(double swing, double boom,
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

void TruckSmoothNode::PublishFinish(bool finished) {
  std_msgs::Bool msg;
  msg.data = finished;
  pub_finish_.publish(msg);
  finish_flag_ = finished;
}

double TruckSmoothNode::StepTowards(double current, double target,
                                    double max_step) const {
  const double diff = target - current;
  if (std::abs(diff) <= max_step) return target;
  return current + (diff > 0.0 ? max_step : -max_step);
}

void TruckSmoothNode::StopExecution() {
  if (phase_ == SmoothPhase::kPhase1 || phase_ == SmoothPhase::kPhase2 ||
      phase_ == SmoothPhase::kFailed) {
    // 外部撤销：保持末次命令并置结束位（不跳向目标）
    PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_,
                           false);
    PublishFinish(false);
    timer_.stop();
    phase_ = SmoothPhase::kIdle;
    ROS_INFO("[truck_smooth] execution stopped by external command");
  }
}

/// 异常中止：保持末次命令姿态并置 Position.x=0，FlagSmoothFinish=false，
/// 状态置 kFailed。不跳向目标，避免中止瞬间给控制器大阶跃。
void TruckSmoothNode::AbortExecution(const char* reason) {
  const double elapsed = (ros::Time::now() - execution_start_time_).toSec();
  ROS_WARN("[truck_smooth] ABORT after %.1fs: %s (phase=%s, fb[sw=%.1f "
           "ar=%.1f] cmd[sw=%.1f ar=%.1f])", elapsed, reason,
           PhaseName(phase_), swing_deg_, arm_deg_,
           cmd_swing_deg_, cmd_arm_deg_);
  PublishTrajectoryFrame(cmd_swing_deg_, boom_deg_, cmd_arm_deg_, bucket_deg_,
                         false);
  PublishFinish(false);
  timer_.stop();
  phase_ = SmoothPhase::kFailed;
}

bool TruckSmoothNode::CheckInputsValid() {
  if (!joints_valid_ || !swing_valid_) {
    ROS_WARN_THROTTLE(2.0,
        "[truck_smooth] waiting feedback: joints=%d swing=%d",
        joints_valid_, swing_valid_);
    return false;
  }
  if (!truck_pose_valid_) {
    ROS_WARN_THROTTLE(2.0,
        "[truck_smooth] waiting /truck_center_unloadpoint");
    return false;
  }
  return true;
}

bool TruckSmoothNode::CheckInputsFresh() {
  if (input_timeout_sec_ <= 0.0) return true;
  const ros::Time now = ros::Time::now();
  const double joints_age = (now - last_joints_time_).toSec();
  const double swing_age = (now - last_swing_time_).toSec();
  if (joints_age > input_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "[truck_smooth] /joints_angle stale (%.2fs)",
                      joints_age);
    return false;
  }
  if (swing_age > input_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "[truck_smooth] /heading2swing_topic stale (%.2fs)",
                      swing_age);
    return false;
  }
  return true;
}

}  // namespace truck_smooth_planner
