#include "dump_trajectory_planner/dump_trajectory_node.h"

#include <algorithm>
#include <cmath>

#include "dump_trajectory_planner/angle_utils.h"
#include "dump_trajectory_planner/cubic_hermite_interpolator.h"

namespace dump_trajectory_planner {

namespace {
/// 激活值
constexpr double kActivateValue = 4.0;
/// 段最小时长 (s)
constexpr double kMinSegmentTime = 0.5;

/// 状态机阶段名（可视化状态文本用）
const char* PhaseName(DumpPhase p) {
  switch (p) {
    case DumpPhase::kIdle: return "IDLE";
    case DumpPhase::kPlanNextSegment: return "PLAN_SEG";
    case DumpPhase::kExecutingSegment: return "EXEC_SEG";
    case DumpPhase::kWaitBucketClear: return "GATE_WAIT";
    case DumpPhase::kGateBoost: return "GATE_BOOST";
    case DumpPhase::kDone: return "DONE";
  }
  return "?";
}
}  // namespace

// 角度工具函数统一由 angle_utils.h 提供

// ==================== 构造函数 ====================

DumpTrajectoryNode::DumpTrajectoryNode(ros::NodeHandle& nh,
                                       ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), reporter_(nh), visualizer_(nh, pnh) {
  LoadParams();

  // 构造运动学求解器（参数从 pnh_/kinematics/... 加载）
  solver_ = std::make_unique<kinematics::KinematicsSolver>(pnh_, "kinematics");

  SetupTopics();

  timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                           &DumpTrajectoryNode::TimerCallback, this,
                           /*oneshot=*/false, /*autostart=*/false);

  // 初始完成标志=1（空闲，可接收激活）
  reporter_.ReportFinishFlag(1.0);

  ROS_INFO("[dump_traj] ready (online-plan mode). publish_rate=%.1fHz, "
           "total_timeout=%.1fs, seg_tols=[%.1f,%.1f,%.1f,%.1f,%.1f/%.1f]deg, "
           "truck_top_height=%.2fm",
           publish_rate_hz_, total_timeout_sec_,
           seg0_boom_tolerance_deg_, seg1_swing_tolerance_deg_,
           seg2_swing_tolerance_deg_, seg3_swing_tolerance_deg_,
           seg4_arm_tolerance_deg_, seg4_bucket_tolerance_deg_,
           truck_top_height_m_);
}

// ==================== 初始化 ====================

void DumpTrajectoryNode::LoadParams() {
  // ---- 可行性检查参数 ----
  feasibility_params_.enable_envelope_prefilter =
      pnh_.param("feasibility/enable_envelope_prefilter", true);
  feasibility_params_.reach_min =
      pnh_.param("feasibility/reach_min", 3.0);
  feasibility_params_.reach_max =
      pnh_.param("feasibility/reach_max", 12.0);
  feasibility_params_.dump_height_min =
      pnh_.param("feasibility/dump_height_min", 1.0);
  feasibility_params_.dump_height_max =
      pnh_.param("feasibility/dump_height_max", 7.5);
  feasibility_params_.bucket_attitude_deg =
      pnh_.param("feasibility/bucket_attitude_deg", -90.0);

  // ---- 航路点生成参数 ----
  waypoint_params_.attitude_angle_deg =
      pnh_.param("waypoint/attitude_angle_deg", 25.0);
  waypoint_params_.wp4_height_bias =
      pnh_.param("waypoint/wp4_height_bias", 1.0);
  // 卡车框高度偏移：卡车框高度 = truck_center_z - truck_height_offset
  waypoint_params_.truck_height_offset =
      pnh_.param("waypoint/truck_height_offset", 1.0);
  waypoint_params_.wp4_bucket_angle_deg =
      pnh_.param("waypoint/wp4_bucket_angle_deg", 25.0);
  waypoint_params_.wp3_swing_step_deg =
      pnh_.param("waypoint/wp3_swing_step_deg", 5.0);
  waypoint_params_.p3_min_z0_swing_deg =
      pnh_.param("waypoint/p3_min_z0_swing_deg", 10.0);
  waypoint_params_.p3_max_z0_swing_deg =
      pnh_.param("waypoint/p3_max_z0_swing_deg", 40.0);
  waypoint_params_.max_boom_lift24_deg =
      pnh_.param("waypoint/max_boom_lift24_deg", 20.0);
  waypoint_params_.p3_min_swing_deg =
      pnh_.param("waypoint/p3_min_swing_deg", 15.0);
  waypoint_params_.p3_max_swing_deg =
      pnh_.param("waypoint/p3_max_swing_deg", 60.0);
  waypoint_params_.min_p3_swing_vel_dps =
      pnh_.param("waypoint/min_p3_swing_vel_dps", 5.0);
  waypoint_params_.max_p3_swing_vel_dps =
      pnh_.param("waypoint/max_p3_swing_vel_dps", 15.0);
  waypoint_params_.p3_boom_cof =
      pnh_.param("waypoint/p3_boom_cof", 0.5);
  waypoint_params_.p3_arm_cof =
      pnh_.param("waypoint/p3_arm_cof", 0.5);
  waypoint_params_.p5_boom_cof =
      pnh_.param("waypoint/p5_boom_cof", 0.5);
  waypoint_params_.p5_arm_cof =
      pnh_.param("waypoint/p5_arm_cof", 0.5);
  waypoint_params_.wp2_vel_boom_dps =
      pnh_.param("waypoint/wp2_vel_boom_dps", 8.0);
  waypoint_params_.wp4_vel_swing_dps =
      pnh_.param("waypoint/wp4_vel_swing_dps", 15.0);
  waypoint_params_.wp4_vel_arm_dps =
      pnh_.param("waypoint/wp4_vel_arm_dps", 10.0);
  waypoint_params_.wp5_vel_boom_dps =
      pnh_.param("waypoint/wp5_vel_boom_dps", 8.0);
  waypoint_params_.wp5_vel_arm_dps =
      pnh_.param("waypoint/wp5_vel_arm_dps", 10.0);
  waypoint_params_.wp5_vel_bkt_dps =
      pnh_.param("waypoint/wp5_vel_bkt_dps", 15.0);
  waypoint_params_.bucket_limit_lower_deg =
      pnh_.param("waypoint/bucket_limit_lower_deg", -170.0);
  waypoint_params_.bucket_limit_upper_deg =
      pnh_.param("waypoint/bucket_limit_upper_deg", 30.0);

  // ---- WP4 入厢点（回转圆策略）----
  waypoint_params_.truck_box_length =
      pnh_.param("waypoint/truck_box_length", 5.0);
  waypoint_params_.truck_box_width =
      pnh_.param("waypoint/truck_box_width", 2.6);
  waypoint_params_.wp4_R_coff =
      pnh_.param("waypoint/wp4_R_coff", 0.5);

  // ---- 全局 ----
  publish_rate_hz_ = pnh_.param("publish_rate", 10.0);
  total_timeout_sec_ = pnh_.param("total_timeout_sec", 30.0);

  // ---- 在线规划参数 ----
  seg0_boom_tolerance_deg_ = pnh_.param("online_plan/seg0_boom_deg", 2.0);
  seg1_swing_tolerance_deg_ = pnh_.param("online_plan/seg1_swing_deg", 2.0);
  seg2_swing_tolerance_deg_ = pnh_.param("online_plan/seg2_swing_deg", 2.0);
  seg2_height_gate_ = pnh_.param("online_plan/seg2_height_gate", true);
  seg3_swing_tolerance_deg_ = pnh_.param("online_plan/seg3_swing_deg", 2.0);
  seg4_arm_tolerance_deg_ = pnh_.param("online_plan/seg4_arm_deg", 2.0);
  seg4_bucket_tolerance_deg_ = pnh_.param("online_plan/seg4_bucket_deg", 2.0);
  segment_timeout_factor_ = pnh_.param("online_plan/segment_timeout_factor", 2.0);
  segment_confirm_frames_ = pnh_.param("online_plan/segment_confirm_frames", 3);
  truck_top_height_m_ = pnh_.param("online_plan/truck_top_height_m", 3.5);
  bucket_clear_margin_m_ = pnh_.param("online_plan/bucket_clear_margin_m", 0.3);
  bucket_clear_seg_idx_ = pnh_.param("online_plan/bucket_clear_seg_idx", 1);
  gate_timeout_sec_ = pnh_.param("online_plan/gate_timeout_sec", 4.0);
  gate_boost_boom_deg_ = pnh_.param("online_plan/gate_boost_boom_deg", 3.0);
  gate_max_boost_count_ = pnh_.param("online_plan/gate_max_boost_count", 3);
  waypoint_clamp_abort_deg_ =
      pnh_.param("online_plan/waypoint_clamp_abort_deg", 5.0);
}

void DumpTrajectoryNode::SetupTopics() {
  sub_activate_ = nh_.subscribe("/Sys_SeqAction", 1,
                                &DumpTrajectoryNode::ActivateCallback, this);
  sub_truck_pose_ = nh_.subscribe(
      "/truck_center_unloadpoint", 1,
      &DumpTrajectoryNode::TruckPoseCallback, this);
  sub_joints_angle_ = nh_.subscribe(
      "/joints_angle", 1,
      &DumpTrajectoryNode::JointsAngleCallback, this);
  sub_swing_ = nh_.subscribe(
      "/heading2swing_topic", 1,
      &DumpTrajectoryNode::SwingCallback, this);

  // 输出：仅执行指令流，状态/统计/适配类话题统一由 StatusReporter 发布
  pub_trajectory_ = nh_.advertise<geometry_msgs::Pose>(
      "/RefDeviceTraj_Dump", 1);
}

// ==================== 订阅回调 ====================

void DumpTrajectoryNode::TruckPoseCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  // x/y/z = 卡车中心坐标；w = RTK方位角 α (deg, 北0顺时针)
  // base 系卡车航向 β = (180 + α) mod 360 → rad
  truck_pose_.center_x = msg->x;
  truck_pose_.center_y = msg->y;
  truck_pose_.center_z = msg->z;      // RTK 高度，用于计算卡车框高度
  truck_pose_.rtk_heading_deg = msg->w;  // RTK 方位角 α
  double beta_deg = std::fmod(180.0 + msg->w, 360.0);
  if (beta_deg < 0.0) beta_deg += 360.0;
  truck_pose_.yaw_rad = Deg2Rad(beta_deg);
  truck_pose_valid_ = true;

  // 卸载点取卡车中心 x/y，z=0
  unload_point_ = {msg->x, msg->y, 0.0};
  unload_valid_ = true;
}

void DumpTrajectoryNode::JointsAngleCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  // x/y/z = boom/arm/bucket 单位 deg，转 rad 存 JointState
  current_joint_.boom = Deg2Rad(msg->x);
  current_joint_.arm = Deg2Rad(msg->y);
  current_joint_.bucket = Deg2Rad(msg->z);
  joints_valid_ = true;

  // 更新 boom/arm/bucket 历史缓存（swing 分量由独立历史维护）
  vel_estimator_.PushJoints(ros::Time::now().toSec(), current_joint_);

  // 臂架关节变化影响齿尖位置 → FK 重算并上报
  UpdateBucketPos();
}

void DumpTrajectoryNode::SwingCallback(
    const std_msgs::Float32::ConstPtr& msg) {
  // data = swing 单位 deg
  current_joint_.swing = Deg2Rad(static_cast<double>(msg->data));
  swing_valid_ = true;

  // swing 独立历史（与 joints 话题频率无关，各自时间戳）
  vel_estimator_.PushSwing(ros::Time::now().toSec(), current_joint_.swing);

  // 实时转发为 /Swing_topic（deg 原值，保持下游协议兼容）
  reporter_.ReportSwingForward(static_cast<double>(msg->data));

  // swing 变化影响齿尖 x/y → FK 重算并上报
  UpdateBucketPos();
}

void DumpTrajectoryNode::UpdateBucketPos() {
  if (!swing_valid_ || !joints_valid_) return;  // FK 需全部 4 关节角
  const auto pose = solver_->swing_center_forward(current_joint_);
  bucket_height_ = pose.z;
  bucket_pos_valid_ = true;

  reporter_.ReportBucketPos(pose.x, pose.y, pose.z);
}

// ==================== 激活 ====================

void DumpTrajectoryNode::ActivateCallback(
    const std_msgs::Float64::ConstPtr& msg) {
  if (msg->data != kActivateValue) {
    if (phase_ != DumpPhase::kIdle && phase_ != DumpPhase::kDone) {
      StopExecution();
    }
    return;
  }

  if (phase_ != DumpPhase::kIdle && phase_ != DumpPhase::kDone) return;

  ROS_INFO("[dump_traj] activation received (Sys_SeqAction=4)");

  // 打印矿卡信息
  ROS_INFO("[dump_traj] truck info: center=(%.2f, %.2f, %.2f) m, rtk_heading=%.2f deg, "
           "yaw_base=%.2f deg, box=%.2fx%.2f m",
           truck_pose_.center_x, truck_pose_.center_y, truck_pose_.center_z,
           truck_pose_.rtk_heading_deg, Rad2Deg(truck_pose_.yaw_rad),
           waypoint_params_.truck_box_length, waypoint_params_.truck_box_width);

  if (!CheckInputsValid()) return;

  // 可行性检查（IK 求解 unload_joint）
  const FeasibilityResult fr =
      CheckReachable(unload_point_, feasibility_params_, *solver_);
  // 发布卸载点校验结果（latched，上位机据此判断是否换点重试）
  reporter_.ReportFeasible(fr.reachable);
  if (!fr.reachable) {
    ROS_WARN("[dump_traj] unload point not reachable: %s", fr.reason.c_str());
    reporter_.ReportPulse(false, false);  // error_code=1 卡车位置无法卸载
    return;
  }
  ROS_INFO("[dump_traj] unload point (%.2f, %.2f, %.2f) reachable, "
           "unload_joint(deg) swing=%.2f boom=%.2f arm=%.2f bkt=%.2f",
           unload_point_.x, unload_point_.y, unload_point_.z,
           Rad2Deg(fr.unload_joint.swing), Rad2Deg(fr.unload_joint.boom),
           Rad2Deg(fr.unload_joint.arm), Rad2Deg(fr.unload_joint.bucket));

  // 生成航路点（只生成 WP1-WP6，不做全轨迹插值）
  const auto wpts = GenerateDumpWaypoints(current_joint_, fr.unload_joint,
                                           unload_point_, truck_pose_,
                                           *solver_, waypoint_params_);
  if (!wpts.success) {
    // WP4 航路点可达性失败（当前生成器唯一失败源即 WP4 IK）→ error_flag=false
    ROS_WARN("[dump_traj] generate waypoints failed: %s", wpts.message.c_str());
    reporter_.ReportPulse(true, false);  // error_code=2 轨迹规划失败
    return;
  }

  // 打印 WP1-WP6 航路点（deg）
  for (size_t i = 0; i < wpts.waypoints.size(); ++i) {
    const auto& wp = wpts.waypoints[i];
    ROS_INFO("[dump_traj] WP%zu: swing=%.2f boom=%.2f arm=%.2f bucket=%.2f (deg), t=%.2fs",
             i + 1, Rad2Deg(wp.swing), Rad2Deg(wp.boom),
             Rad2Deg(wp.arm), Rad2Deg(wp.bucket), wpts.t_array[i]);
  }

  // 缓存航路点并展开 swing 角度（处理 0°/360° 边界）
  waypoints_ = wpts.waypoints;
  UnwrapSwingSequence(waypoints_);
  total_segments_ = static_cast<int>(waypoints_.size()) - 1;

  // 校验航路点关节限位（超限 clamp；修正量超阈值则判定上游不可信，终止）
  if (!ValidateWaypointsJointLimits()) {
    ROS_WARN("[dump_traj] waypoint clamp fix exceeds threshold (%.1f deg), abort",
             waypoint_clamp_abort_deg_);
    waypoints_.clear();
    reporter_.ReportPulse(true, false);  // 限位校验失败归入 error_code=2
    return;
  }

  // 航路点已定稿（含 clamp 修正）→ 发布统计量：动臂最大角度 / 齿尖最高 z
  {
    double max_boom_deg = -1e9;
    double max_z = -1e9;
    for (const auto& q : waypoints_) {
      max_boom_deg = std::max(max_boom_deg, Rad2Deg(q.boom));
      max_z = std::max(max_z, solver_->swing_center_forward(q).z);
    }
    reporter_.ReportWaypointStats(max_boom_deg, max_z);
    ROS_INFO("[dump_traj] waypoint stats: max_boom=%.1f deg, max_z=%.2f m",
             max_boom_deg, max_z);
  }

  // 计算各段预期时长（从 t_array 差分得出）
  segment_times_.clear();
  for (int i = 0; i < total_segments_; ++i) {
    double dt = wpts.t_array[i + 1] - wpts.t_array[i];
    segment_times_.push_back(std::max(dt, kMinSegmentTime));
  }

  // 计算各航路点目标切线（用于段间平滑过渡，依赖 segment_times_）
  ComputeWaypointTangents();

  ROS_INFO("[dump_traj] waypoints generated (online-plan mode):");
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    const auto& q = waypoints_[i];
    ROS_INFO("[dump_traj]   WP%zu t=%.2f  swing=%.2f boom=%.2f arm=%.2f bkt=%.2f",
             i + 1, wpts.t_array[i],
             Rad2Deg(q.swing), Rad2Deg(q.boom), Rad2Deg(q.arm),
             Rad2Deg(q.bucket));
  }

  // 可视化：卡车场景 + 规划结果（激活时发布一次，供 RViz 联调）
  visualizer_.PublishTruckScene(truck_pose_, waypoint_params_.truck_box_length,
                                waypoint_params_.truck_box_width,
                                truck_top_height_m_,
                                truck_top_height_m_ + bucket_clear_margin_m_,
                                unload_point_);
  visualizer_.PublishPlan(waypoints_, current_joint_, fr.unload_joint,
                          unload_point_, truck_pose_, waypoint_params_,
                          *solver_);

  // 初始化执行状态
  execution_start_time_ = ros::Time::now();
  current_seg_idx_ = 0;
  gate_boost_count_ = 0;
  phase_ = DumpPhase::kPlanNextSegment;
  timer_.start();
  reporter_.ReportFinishFlag(0.0);  // 进入执行：完成标志=0
  reporter_.ReportPulse(true, true);  // error_code=0 校验全部通过
}

// ==================== 定时器 ====================

void DumpTrajectoryNode::TimerCallback(const ros::TimerEvent& /*event*/) {
  // 可视化状态文本（随定时器高频刷新，联调时直观查看状态机）
  {
    char status_buf[160];
    snprintf(status_buf, sizeof(status_buf),
             "phase=%s  seg=%d/%d  boost=%d/%d\nbkt_z=%.2f (need>%.2f)",
             PhaseName(phase_), current_seg_idx_, total_segments_,
             gate_boost_count_, gate_max_boost_count_,
             bucket_height_, truck_top_height_m_ + bucket_clear_margin_m_);
    visualizer_.PublishStatus(status_buf);
  }

  // 全局超时检查
  const double elapsed = (ros::Time::now() - execution_start_time_).toSec();
  if (elapsed > total_timeout_sec_ && phase_ != DumpPhase::kIdle &&
      phase_ != DumpPhase::kDone) {
    ROS_WARN("[dump_traj] total execution timeout (%.1fs), abort", elapsed);
    PublishTrajectoryFrame(current_joint_, false);
    timer_.stop();
    phase_ = DumpPhase::kIdle;
    reporter_.ReportExecutionAbort();  // error_code=3：与正常完成区分
    reporter_.ReportFinishFlag(1.0);
    visualizer_.ClearSegmentTrajectory();
    return;
  }

  switch (phase_) {
    case DumpPhase::kPlanNextSegment:
      PlanCurrentSegment();
      break;

    case DumpPhase::kExecutingSegment: {
      // 判定当前段推进结果
      const SegResult seg_result = CheckSegmentProgress();
      if (seg_result == SegResult::kComplete) {
        AdvanceToNextSegment();
        return;
      }
      if (seg_result == SegResult::kEnterGate) {
        // 转入高度门控：清空速度历史（停车后速度归零），等待/boost 重试跨越
        vel_estimator_.Clear();
        gate_hold_cmd_ = current_joint_;  // 锁存当前姿态，门控期间恒定发布
        gate_start_time_ = ros::Time::now();
        phase_ = DumpPhase::kWaitBucketClear;
        return;
      }
      // kContinue：逐帧发布
      if (seg_frame_idx_ < seg_trajectory_.size()) {
        PublishTrajectoryFrame(seg_trajectory_[seg_frame_idx_], true);
        ++seg_frame_idx_;
      } else {
        // 轨迹帧已发完，但关节还未到达目标，保持发布最后一帧
        PublishTrajectoryFrame(seg_trajectory_.back(), true);
      }
      break;
    }

    case DumpPhase::kWaitBucketClear:
      if (IsBucketAboveTruck()) {
        ROS_INFO("[dump_traj] bucket cleared truck top (height=%.2fm > %.2fm), "
                 "advancing to segment %d",
                 bucket_height_, truck_top_height_m_ + bucket_clear_margin_m_,
                 current_seg_idx_);
        phase_ = DumpPhase::kPlanNextSegment;
      } else {
        const double gate_elapsed =
            (ros::Time::now() - gate_start_time_).toSec();
        if (gate_elapsed > gate_timeout_sec_) {
          // 门控超时：自动追加 boom 提升重试；次数耗尽则终止卸载
          if (gate_boost_count_ >= gate_max_boost_count_) {
            ROS_WARN("[dump_traj] bucket still below truck top after %d boosts "
                     "(height=%.2fm, need > %.2fm), abort",
                     gate_boost_count_, bucket_height_,
                     truck_top_height_m_ + bucket_clear_margin_m_);
            PublishTrajectoryFrame(current_joint_, false);
            timer_.stop();
            phase_ = DumpPhase::kIdle;
            reporter_.ReportExecutionAbort();  // error_code=3：门控失败中止
            reporter_.ReportFinishFlag(1.0);
            visualizer_.ClearSegmentTrajectory();
          } else if (StartGateBoost()) {
            ++gate_boost_count_;
          } else {
            ROS_WARN("[dump_traj] boom already at upper limit, cannot boost, "
                     "abort");
            PublishTrajectoryFrame(current_joint_, false);
            timer_.stop();
            phase_ = DumpPhase::kIdle;
            reporter_.ReportExecutionAbort();  // error_code=3：boom 到限中止
            reporter_.ReportFinishFlag(1.0);
            visualizer_.ClearSegmentTrajectory();
          }
        } else {
          // 等待期间恒定发布锁存姿态（不跟随反馈，避免液压保压沉降被逐帧追认成下沉指令）
          PublishTrajectoryFrame(gate_hold_cmd_, true);
        }
      }
      break;

    case DumpPhase::kGateBoost: {
      // 指令朝目标步进（按 boom 提升限速，arm 联动保姿态）
      const double step =
          Deg2Rad(waypoint_params_.wp2_vel_boom_dps) / publish_rate_hz_;
      if (boost_cmd_.boom < boost_target_.boom) {
        boost_cmd_.boom = std::min(boost_cmd_.boom + step, boost_target_.boom);
      }
      if (boost_cmd_.arm > boost_target_.arm) {  // arm 反向联动（Δarm=-Δboom）
        boost_cmd_.arm = std::max(boost_cmd_.arm - step, boost_target_.arm);
      }
      PublishTrajectoryFrame(boost_cmd_, true);

      // 完成判定：仅检查动臂是否到达提升目标
      const double boom_err = std::abs(
          Rad2Deg(current_joint_.boom - boost_target_.boom));
      if (boom_err <= seg0_boom_tolerance_deg_) {
        ROS_INFO("[dump_traj] gate boost %d done (boom=%.1f deg), "
                 "re-checking bucket height",
                 gate_boost_count_, Rad2Deg(current_joint_.boom));
        gate_hold_cmd_ = boost_cmd_;  // 锁存抬升后的指令位姿
        gate_start_time_ = ros::Time::now();
        phase_ = DumpPhase::kWaitBucketClear;
        break;
      }
      // 超时兜底：指令走完但反馈未跟随 → 回门控以当前高度重新判定
      const double boost_expected = gate_boost_boom_deg_ /
          std::max(waypoint_params_.wp2_vel_boom_dps, 1e-6);
      const double boost_timeout = boost_expected * segment_timeout_factor_;
      if ((ros::Time::now() - boost_start_time_).toSec() > boost_timeout) {
        ROS_WARN("[dump_traj] gate boost timeout (boom_err=%.2fdeg), "
                 "re-checking bucket height anyway",
                 std::abs(Rad2Deg(current_joint_.boom - boost_target_.boom)));
        gate_hold_cmd_ = boost_cmd_;  // 锁存抬升后的指令位姿
        gate_start_time_ = ros::Time::now();
        phase_ = DumpPhase::kWaitBucketClear;
      }
      break;
    }

    case DumpPhase::kDone:
    case DumpPhase::kIdle:
      timer_.stop();
      break;
  }
}

// ==================== 在线规划方法 ====================

void DumpTrajectoryNode::PlanCurrentSegment() {
  if (current_seg_idx_ >= total_segments_) {
    // 所有段完成
    PublishTrajectoryFrame(waypoints_.back(), false);
    phase_ = DumpPhase::kDone;
    timer_.stop();
    reporter_.ReportFinishFlag(1.0);
    visualizer_.ClearSegmentTrajectory();
    ROS_INFO("[dump_traj] all segments complete, done");
    return;
  }

  // 起点：取当前实际关节角（消除累积误差）
  kinematics::JointState seg_start = current_joint_;
  const kinematics::JointState& seg_end = waypoints_[current_seg_idx_ + 1];

  // 将起点 swing 展开到与目标航路点同一周期（处理 0°/360° 跨越）
  double swing_diff_raw = seg_start.swing - seg_end.swing;
  swing_diff_raw = NormalizeRadToPi(swing_diff_raw);
  seg_start.swing = seg_end.swing + swing_diff_raw;

  // 计算段时间：基于实际剩余角度差
  double max_time = 0.0;
  double swing_diff = std::abs(Rad2Deg(seg_end.swing - seg_start.swing));
  double boom_diff = std::abs(Rad2Deg(seg_end.boom - seg_start.boom));
  double arm_diff = std::abs(Rad2Deg(seg_end.arm - seg_start.arm));
  double bkt_diff = std::abs(Rad2Deg(seg_end.bucket - seg_start.bucket));

  // 根据段索引使用对应的主导关节速度
  if (current_seg_idx_ == 0) {
    // WP1→WP2: 动臂提升段
    max_time = boom_diff / waypoint_params_.wp2_vel_boom_dps;
  } else if (current_seg_idx_ == 1) {
    // WP2→WP3: 回转主导，回转速度随回转幅度插值（小回转→慢速），
    // 与生成器 t3_dur 严格一致（swing 用插值速度，第二项用 boom 提升速度）
    double wp3_vel_swing = Wp3SwingVelDps(waypoint_params_, swing_diff);
    double t_swing = swing_diff / std::max(wp3_vel_swing, 1e-6);
    double t_boom = boom_diff / waypoint_params_.wp2_vel_boom_dps;
    max_time = std::max(t_swing, t_boom);
  } else if (current_seg_idx_ == 2) {
    // WP3→WP4: 回转主导段（回转到厢上过渡方位）
    double t_swing = swing_diff / waypoint_params_.wp4_vel_swing_dps;
    double t_arm = arm_diff / waypoint_params_.wp4_vel_arm_dps;
    max_time = std::max(t_swing, t_arm);
  } else if (current_seg_idx_ == 3) {
    // WP4→WP5: 回转到卸载方位（swing 是本段完成判定关节，必须计入段时间），
    // 同时 boom/arm 混合过渡 —— 取三者最大者，防止指令回转速度超设计值
    double t_swing = swing_diff / waypoint_params_.wp4_vel_swing_dps;
    double t_boom = boom_diff / waypoint_params_.wp5_vel_boom_dps;
    double t_arm = arm_diff / waypoint_params_.wp5_vel_arm_dps;
    max_time = std::max({t_swing, t_boom, t_arm});
  } else {
    // WP5→WP6: 臂架主导段
    double t_boom = boom_diff / waypoint_params_.wp5_vel_boom_dps;
    double t_arm = arm_diff / waypoint_params_.wp5_vel_arm_dps;
    double t_bkt = bkt_diff / waypoint_params_.wp5_vel_bkt_dps;
    max_time = std::max({t_boom, t_arm, t_bkt});
  }
  max_time = std::max(max_time, kMinSegmentTime);

  // 确定起点切线：从反馈估计实际速度
  // 第一段静止启动；其它段用历史估计（门控后若 history 已清空则自然返回0）
  kinematics::JointState v_start{};
  if (current_seg_idx_ > 0) {
    v_start = vel_estimator_.Estimate();
  }

  // 确定终点切线：从预计算的目标切线取
  const kinematics::JointState& v_end = wp_tangents_[current_seg_idx_ + 1];

  // 使用带显式切线的 Hermite 插值生成局部轨迹
  seg_trajectory_ = InterpolateSegmentWithTangents(
      seg_start, seg_end, v_start, v_end, max_time, 1.0 / publish_rate_hz_);
  seg_frame_idx_ = 0;
  seg_start_time_ = ros::Time::now();
  seg_planned_time_ = max_time;  // 记录在线实际段时长，供超时判据使用
  confirm_count_ = 0;  // 重置段完成防抖计数

  // 可视化：更新当前段轨迹折线
  visualizer_.PublishSegmentTrajectory(seg_trajectory_, *solver_);

  ROS_INFO("[dump_traj] segment %d planned online: %zu frames, %.2fs "
           "(v_start: sw=%.2f bm=%.2f | v_end: sw=%.2f bm=%.2f)",
           current_seg_idx_, seg_trajectory_.size(), max_time,
           Rad2Deg(v_start.swing), Rad2Deg(v_start.boom),
           Rad2Deg(v_end.swing), Rad2Deg(v_end.boom));

  phase_ = DumpPhase::kExecutingSegment;
}

DumpTrajectoryNode::SegResult DumpTrajectoryNode::CheckSegmentProgress() {
  // 1. 主判定：仅检查该段指定的主导关节（需连续 N 帧确认，防抖动）
  double seg_err = ComputeSegError(current_seg_idx_);
  double tol = 0.0;
  switch (current_seg_idx_) {
    case 0: tol = seg0_boom_tolerance_deg_; break;
    case 1: tol = seg1_swing_tolerance_deg_; break;
    case 2: tol = seg2_swing_tolerance_deg_; break;
    case 3: tol = seg3_swing_tolerance_deg_; break;
    default: tol = std::max(seg4_arm_tolerance_deg_, seg4_bucket_tolerance_deg_); break;
  }

  if (seg_err <= tol) {
    ++confirm_count_;
    if (confirm_count_ >= segment_confirm_frames_) {
      ROS_INFO("[dump_traj] segment %d complete (err=%.2fdeg <= tol=%.1fdeg, "
               "confirmed %d frames)",
               current_seg_idx_, seg_err, tol, confirm_count_);
      return SegResult::kComplete;
    }
  } else {
    confirm_count_ = 0;
  }

  // 2. 兜底：段超时处理
  double seg_elapsed = (ros::Time::now() - seg_start_time_).toSec();
  // 超时基准取"标称段时长"与"在线实际规划段时长"的较大者：
  // 反馈起点残差大时在线段时长会更长，若仍用标称值会把正常段误判超时强推
  double seg_base_time =
      std::max(segment_times_[current_seg_idx_], seg_planned_time_);
  double seg_timeout = seg_base_time * segment_timeout_factor_;
  if (seg_elapsed > seg_timeout) {
    // 安全优先：seg2 高度门控未达标时超时不强推（防止铲斗高度不足跨越卡车），
    // 转入高度门控复用 boost 重试机制；次数耗尽由门控状态统一终止
    if (current_seg_idx_ == 2 && seg2_height_gate_ && !IsBucketAboveTruck()) {
      ROS_ERROR("[dump_traj] segment 2 timeout with bucket below truck top "
                "(height=%.2fm <= %.2fm), entering height gate for boost-retry "
                "instead of forcing advance",
                bucket_height_, truck_top_height_m_ + bucket_clear_margin_m_);
      return SegResult::kEnterGate;
    }
    ROS_WARN("[dump_traj] segment %d timeout (%.1fs > %.1fs), err=%.2fdeg, "
             "forcing advance",
             current_seg_idx_, seg_elapsed, seg_timeout, seg_err);
    return SegResult::kComplete;
  }

  return SegResult::kContinue;
}

double DumpTrajectoryNode::ComputeSegError(int seg_idx) const {
  const kinematics::JointState& target = waypoints_[seg_idx + 1];
  const kinematics::JointState& cur = current_joint_;

  switch (seg_idx) {
    case 0: {
      // WP1→WP2：仅判断动臂
      return std::abs(Rad2Deg(cur.boom - target.boom));
    }
    case 1: {
      // WP2→WP3：仅判断回转（最短角距）
      return ShortestAngularDistanceDeg(Rad2Deg(cur.swing), Rad2Deg(target.swing));
    }
    case 2: {
      // WP3→WP4：回转 + 高度门控
      double swing_err = ShortestAngularDistanceDeg(
          Rad2Deg(cur.swing), Rad2Deg(target.swing));
      if (seg2_height_gate_ && !IsBucketAboveTruck()) {
        // 铲斗未高于卡车上表面，返回大值阻止完成
        return 1e6;
      }
      return swing_err;
    }
    case 3: {
      // WP4→WP5：仅判断回转
      return ShortestAngularDistanceDeg(Rad2Deg(cur.swing), Rad2Deg(target.swing));
    }
    default: {
      // WP5→WP6：斗杆 + 铲斗
      double arm_err = std::abs(Rad2Deg(cur.arm - target.arm));
      double bkt_err = std::abs(Rad2Deg(cur.bucket - target.bucket));
      return std::max(arm_err, bkt_err);
    }
  }
}

bool DumpTrajectoryNode::IsBucketAboveTruck() const {
  if (!bucket_pos_valid_) {
    ROS_WARN_THROTTLE(2.0,
                      "[dump_traj] waiting for joint feedback (bucket FK)...");
    return false;
  }
  return bucket_height_ > (truck_top_height_m_ + bucket_clear_margin_m_);
}

bool DumpTrajectoryNode::StartGateBoost() {
  const auto& limits = solver_->limits();
  const double bo_deg = Rad2Deg(current_joint_.boom);
  const double bo_target =
      std::min(bo_deg + gate_boost_boom_deg_, limits.boom_upper);
  const double delta = bo_target - bo_deg;
  if (delta < 1e-3) {
    return false;  // boom 已达上限，无法再抬升
  }
  // arm 反向联动保持铲斗姿态（Δarm = -Δboom），限位优先
  const double ar_target =
      std::max(limits.arm_lower,
               std::min(limits.arm_upper, Rad2Deg(current_joint_.arm) - delta));

  boost_target_ = current_joint_;
  boost_target_.boom = Deg2Rad(bo_target);
  boost_target_.arm = Deg2Rad(ar_target);
  boost_cmd_ = current_joint_;
  boost_start_time_ = ros::Time::now();
  phase_ = DumpPhase::kGateBoost;

  ROS_INFO("[dump_traj] gate timeout (%.1fs), auto boost %d/%d: "
           "boom %.1f -> %.1f deg",
           gate_timeout_sec_, gate_boost_count_ + 1, gate_max_boost_count_,
           bo_deg, bo_target);
  return true;
}

void DumpTrajectoryNode::AdvanceToNextSegment() {
  // 检查是否需要铲斗高度门控
  if (current_seg_idx_ == bucket_clear_seg_idx_) {
    current_seg_idx_++;
    // 立即检测：如果铲斗已经高于卡车，直接通过不停等
    if (IsBucketAboveTruck()) {
      ROS_INFO("[dump_traj] segment %d gate pre-cleared (height=%.2fm > %.2fm), "
               "proceeding immediately",
               current_seg_idx_ - 1, bucket_height_,
               truck_top_height_m_ + bucket_clear_margin_m_);
      // 不清空两路历史缓存，保留速度估计（允许平滑过渡）
      if (current_seg_idx_ >= total_segments_) {
        PublishTrajectoryFrame(waypoints_.back(), false);
        phase_ = DumpPhase::kDone;
        timer_.stop();
        reporter_.ReportFinishFlag(1.0);
        visualizer_.ClearSegmentTrajectory();
      } else {
        phase_ = DumpPhase::kPlanNextSegment;
      }
    } else {
      ROS_INFO("[dump_traj] segment %d done, entering bucket-clear gate "
               "(need height > %.2fm)",
               current_seg_idx_ - 1,
               truck_top_height_m_ + bucket_clear_margin_m_);
      // 门控等待期间清空两路历史缓存（停止后速度为0）
      vel_estimator_.Clear();
      gate_hold_cmd_ = current_joint_;  // 锁存当前姿态，门控期间恒定发布
      gate_start_time_ = ros::Time::now();
      phase_ = DumpPhase::kWaitBucketClear;
    }
    return;
  }

  current_seg_idx_++;

  if (current_seg_idx_ >= total_segments_) {
    // 所有段完成
    PublishTrajectoryFrame(waypoints_.back(), false);
    phase_ = DumpPhase::kDone;
    timer_.stop();
    reporter_.ReportFinishFlag(1.0);
    visualizer_.ClearSegmentTrajectory();
    ROS_INFO("[dump_traj] all segments complete, done");
  } else {
    // 进入下一段规划
    phase_ = DumpPhase::kPlanNextSegment;
  }
}

void DumpTrajectoryNode::ComputeWaypointTangents() {
  const size_t n = waypoints_.size();
  wp_tangents_.assign(n, kinematics::JointState{});

  if (n < 3) return;

  // 用 Catmull-Rom 差分计算内部航路点切线
  for (size_t i = 1; i + 1 < n; ++i) {
    double dt = segment_times_[i - 1] + segment_times_[i];  // t_{i+1} - t_{i-1}
    if (dt < 1e-9) continue;

    // 检查该点是否是门控点后的第一个点（需停止→切线=0）
    bool is_gate_stop = (static_cast<int>(i) == bucket_clear_seg_idx_ + 1);
    if (is_gate_stop) continue;  // 保持 0

    wp_tangents_[i].swing =
        (waypoints_[i + 1].swing - waypoints_[i - 1].swing) / dt;
    wp_tangents_[i].boom =
        (waypoints_[i + 1].boom - waypoints_[i - 1].boom) / dt;
    wp_tangents_[i].arm =
        (waypoints_[i + 1].arm - waypoints_[i - 1].arm) / dt;
    wp_tangents_[i].bucket =
        (waypoints_[i + 1].bucket - waypoints_[i - 1].bucket) / dt;
  }
  // 端点 WP1(idx=0) 和 WP6(idx=n-1) 保持 0（起停静止）
}

// ==================== 工具方法 ====================

bool DumpTrajectoryNode::CheckInputsValid() {
  if (!swing_valid_) {
    ROS_WARN("[dump_traj] /Swing_topic not received, abort");
    return false;
  }
  if (!joints_valid_) {
    ROS_WARN("[dump_traj] /joints_angle not received, abort");
    return false;
  }
  if (!truck_pose_valid_) {
    ROS_WARN("[dump_traj] /truck_center_unloadpoint not received, abort");
    return false;
  }
  return true;
}

bool DumpTrajectoryNode::ValidateWaypointsJointLimits() {
  const auto& limits = solver_->limits();
  bool all_ok = true;

  // 单个关节校验：超限 clamp；修正量超阈值 → 上游不可信，标记终止
  auto clamp_joint = [&](double& q_rad, double lo, double hi,
                         size_t wp_idx, const char* name) {
    const double deg = Rad2Deg(q_rad);
    if (deg >= lo && deg <= hi) return;
    const double clamped = std::max(lo, std::min(hi, deg));
    const double fix = std::abs(clamped - deg);
    if (fix > waypoint_clamp_abort_deg_) {
      ROS_ERROR("[dump_traj] WP%zu %s=%.1f deg out of [%.1f, %.1f], "
                "clamp fix %.1f deg > abort threshold %.1f deg",
                wp_idx + 1, name, deg, lo, hi, fix,
                waypoint_clamp_abort_deg_);
      all_ok = false;
    } else {
      ROS_WARN("[dump_traj] WP%zu %s=%.1f deg out of [%.1f, %.1f], "
               "clamped to %.1f",
               wp_idx + 1, name, deg, lo, hi, clamped);
    }
    q_rad = Deg2Rad(clamped);
  };

  for (size_t i = 0; i < waypoints_.size(); ++i) {
    auto& q = waypoints_[i];
    clamp_joint(q.boom, limits.boom_lower, limits.boom_upper, i, "boom");
    clamp_joint(q.arm, limits.arm_lower, limits.arm_upper, i, "arm");
    clamp_joint(q.bucket, limits.bucket_lower, limits.bucket_upper, i, "bucket");
  }

  return all_ok;
}

void DumpTrajectoryNode::PublishTrajectoryFrame(
    const kinematics::JointState& q, bool running) {
  geometry_msgs::Pose msg;
  // Position.x: running(1) / done(0)
  msg.position.x = running ? 1.0 : 0.0;
  msg.position.y = 0.0;
  msg.position.z = 0.0;
  // Orientation.x/y/z/w = swing/boom/arm/bucket (deg)
  // swing 回包到 [0°, 360°)，处理展开后超出范围的情况
  msg.orientation.x = WrapTo360Deg(Rad2Deg(q.swing));
  msg.orientation.y = Rad2Deg(q.boom);
  msg.orientation.z = Rad2Deg(q.arm);
  msg.orientation.w = Rad2Deg(q.bucket);
  pub_trajectory_.publish(msg);

  // 可视化：当前指令臂架姿态（随指令帧高频发布）
  visualizer_.PublishExcavator(solver_->geometry(), q);
}

void DumpTrajectoryNode::StopExecution() {
  if (phase_ != DumpPhase::kIdle && phase_ != DumpPhase::kDone) {
    PublishTrajectoryFrame(current_joint_, false);
    timer_.stop();
    phase_ = DumpPhase::kIdle;
    reporter_.ReportFinishFlag(1.0);  // 撤销终止：执行流程结束
    visualizer_.ClearSegmentTrajectory();
    ROS_INFO("[dump_traj] execution stopped");
  }
}

}  // namespace dump_trajectory_planner
