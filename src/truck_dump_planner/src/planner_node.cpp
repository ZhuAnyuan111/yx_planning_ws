#include "truck_dump_planner/planner_node.h"

namespace truck_dump_planner {

namespace {
const char* StateToString(PlannerState s) {
  switch (s) {
    case PlannerState::kIdle:  return "IDLE";
    case PlannerState::kDig:   return "DIG";
    case PlannerState::kDump:  return "DUMP";
    case PlannerState::kReset: return "RESET";
  }
  return "UNKNOWN";
}
}  // namespace

PlannerNode::PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  LoadParams();
  viz_ = std::make_unique<EnvelopeVisualizer>(frame_id_);

  // 发布
  pub_markers_ =
      nh_.advertise<visualization_msgs::MarkerArray>("truck_envelope_markers",
                                                     1, true);
  pub_trajectory_ =
      nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers",
                                                     1, true);
  pub_feedback_ =
      nh_.advertise<std_msgs::String>("planner_feedback", 1);

  // 订阅
  sub_pose_ = nh_.subscribe("truck_pose", 1,
                            &PlannerNode::PoseCallback, this);
  sub_joints_ = nh_.subscribe("/joints_angle", 1,
                              &PlannerNode::JointsCallback, this);
  sub_swing_ = nh_.subscribe("/Swing_topic", 1,
                             &PlannerNode::SwingCallback, this);
  sub_activate_ = nh_.subscribe("planner_activate", 1,
                                &PlannerNode::ActivateCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1),
                           &PlannerNode::TimerCallback, this);

  ROS_INFO("[planner] ready. frame=%s, waiting for activation...",
           frame_id_.c_str());
}

void PlannerNode::LoadParams() {
  p_.box_length = pnh_.param("truck/box_length", 5.0);
  p_.box_width = pnh_.param("truck/box_width", 2.6);
  p_.box_height = pnh_.param("truck/box_height", 1.8);

  excavator_xy_ = {pnh_.param("excavator/x", 0.0),
                   pnh_.param("excavator/y", 0.0)};

  frame_id_ = pnh_.param<std::string>("frame_id", "base");

  std::string axis_str =
      pnh_.param<std::string>("axis_convention", "north_west");
  conv_ = (axis_str == "east_north") ? AxisConvention::kEastNorth
                                     : AxisConvention::kNorthWest;

  // 挖掘参数
  dig_params_.dig_point.x = pnh_.param("dig/point_x", 3.0);
  dig_params_.dig_point.y = pnh_.param("dig/point_y", 0.0);
  dig_params_.dig_point.z = pnh_.param("dig/point_z", -0.5);
  dig_params_.approach_height = pnh_.param("dig/approach_height", 1.0);
  dig_params_.dig_depth = pnh_.param("dig/depth", 0.5);

  // 复位参数
  reset_params_.home_position.x = pnh_.param("reset/home_x", 2.0);
  reset_params_.home_position.y = pnh_.param("reset/home_y", 0.0);
  reset_params_.home_position.z = pnh_.param("reset/home_z", 1.5);
  reset_params_.safe_height = pnh_.param("reset/safe_height", 3.0);
}

void PlannerNode::JointsCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  boom_deg_ = msg->x;
  arm_deg_ = msg->y;
  bucket_deg_ = msg->z;
}

void PlannerNode::SwingCallback(
    const geometry_msgs::Point::ConstPtr& msg) {
  swing_deg_ = msg->x;
}

void PlannerNode::PoseCallback(
    const geometry_msgs::Quaternion::ConstPtr& msg) {
  const double rtk_bearing = Wrap360(msg->w);
  last_env_ = BuildTruckEnvelope(msg->x, msg->y, msg->z,
                                  rtk_bearing, p_, conv_);
  env_valid_ = true;

  auto markers = viz_->BuildEnvelopeMarkers(last_env_, excavator_xy_,
                                            ros::Time::now());
  pub_markers_.publish(markers);
}

void PlannerNode::ActivateCallback(
    const std_msgs::String::ConstPtr& msg) {
  const std::string& cmd = msg->data;
  ROS_INFO("[planner] received activation: '%s'", cmd.c_str());
  if (cmd == "dig") {
    TransitionTo(PlannerState::kDig);
  } else if (cmd == "dump") {
    TransitionTo(PlannerState::kDump);
  } else if (cmd == "reset") {
    TransitionTo(PlannerState::kReset);
  } else if (cmd == "stop") {
    TransitionTo(PlannerState::kIdle);
    PublishFeedback("idle");
  } else {
    ROS_WARN("[planner] unknown activation: %s", cmd.c_str());
  }
}

void PlannerNode::TransitionTo(PlannerState new_state) {
  if (new_state == state_) return;
  ROS_INFO("[planner] %s -> %s",
           StateToString(state_), StateToString(new_state));
  state_ = new_state;
  current_trajectory_.clear();

  Vec3 bucket = GetBucketPosition();

  switch (state_) {
    case PlannerState::kDig: {
      auto result = PlanDigTrajectory(bucket, dig_params_);
      if (result.success) {
        current_trajectory_ = std::move(result.waypoints);
        ROS_INFO("[planner] dig trajectory: %zu waypoints",
                 current_trajectory_.size());
        PublishFeedback("dig_done");
      } else {
        ROS_WARN("[planner] dig plan failed: %s", result.message.c_str());
        PublishFeedback("dig_failed");
        state_ = PlannerState::kIdle;
      }
      break;
    }
    case PlannerState::kDump: {
      // 卸载规划占位（后续接入 dump_planner + kinematics）
      ROS_INFO("[planner] dump planning placeholder");
      PublishFeedback("dump_done");
      break;
    }
    case PlannerState::kReset: {
      auto result = PlanResetTrajectory(bucket, reset_params_);
      if (result.success) {
        current_trajectory_ = std::move(result.waypoints);
        ROS_INFO("[planner] reset trajectory: %zu waypoints",
                 current_trajectory_.size());
        PublishFeedback("reset_done");
      } else {
        ROS_WARN("[planner] reset plan failed: %s", result.message.c_str());
        PublishFeedback("reset_failed");
        state_ = PlannerState::kIdle;
      }
      break;
    }
    case PlannerState::kIdle:
      break;
  }
}

void PlannerNode::PublishFeedback(const std::string& status) {
  std_msgs::String msg;
  msg.data = status;
  pub_feedback_.publish(msg);
  ROS_INFO("[planner] feedback -> '%s'", status.c_str());
}

void PlannerNode::TimerCallback(const ros::TimerEvent& /*event*/) {
  if (current_trajectory_.empty()) return;

  // 发布轨迹可视化（LINE_STRIP）
  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = ros::Time::now();
  m.ns = "trajectory";
  m.id = 0;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.05;
  m.color.r = 0.0;
  m.color.g = 0.6;
  m.color.b = 1.0;
  m.color.a = 1.0;
  m.pose.orientation.w = 1.0;
  for (const auto& pt : current_trajectory_) {
    geometry_msgs::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = pt.z;
    m.points.push_back(p);
  }
  arr.markers.push_back(m);
  pub_trajectory_.publish(arr);

  ROS_INFO_THROTTLE(2.0, "[planner] state=%s trajectory=%zu pts",
                    StateToString(state_), current_trajectory_.size());
}

Vec3 PlannerNode::GetBucketPosition() const {
  // 简化占位：用固定位置代替 FK（后续接入 kinematics 后用 FK 计算）
  // TODO: 接入 KinematicsSolver::swing_center_forward(cur_q) 得到真实位置
  return Vec3{2.0, 0.0, 1.5};
}

}  // namespace truck_dump_planner
