#include "dump_trajectory_planner/dump_visualizer.h"

#include <cmath>

namespace dump_trajectory_planner {

namespace {

constexpr double kPi = 3.14159265358979323846;

geometry_msgs::Point Pt(double x, double y, double z) {
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void SetColor(visualization_msgs::Marker& m, double r, double g, double b,
              double a = 1.0) {
  m.color.r = static_cast<float>(r);
  m.color.g = static_cast<float>(g);
  m.color.b = static_cast<float>(b);
  m.color.a = static_cast<float>(a);
}

/// 臂架铰点链（与 KinematicsSolver::swing_center_forward 严格同源）
struct LinkPoints {
  geometry_msgs::Point origin;      // 回转中心 (0,0,0)
  geometry_msgs::Point boom_pivot;  // 动臂销轴
  geometry_msgs::Point boom_arm;    // 动臂·斗杆铰
  geometry_msgs::Point arm_bucket;  // 斗杆·铲斗铰
  geometry_msgs::Point tip;         // 铲斗齿尖
};

LinkPoints ComputeLinkPoints(const kinematics::LinkGeometry& g,
                             const kinematics::JointState& q) {
  const double th_b = q.boom;
  const double th_a = q.boom + q.arm;
  const double th_k = q.boom + q.arm + q.bucket;
  const double cs = std::cos(q.swing);
  const double sn = std::sin(q.swing);

  auto make = [&](double radial, double z) {
    return Pt(cs * radial, sn * radial + g.boom_pivot_y, z);
  };

  LinkPoints lp;
  lp.origin = Pt(0.0, 0.0, 0.0);
  lp.boom_pivot = make(g.boom_pivot_x, g.boom_pivot_z);

  const double r1 = g.boom_pivot_x + g.boom_length * std::cos(th_b);
  const double z1 = g.boom_pivot_z + g.boom_length * std::sin(th_b);
  lp.boom_arm = make(r1, z1);

  const double r2 = r1 + g.arm_length * std::cos(th_a);
  const double z2 = z1 + g.arm_length * std::sin(th_a);
  lp.arm_bucket = make(r2, z2);

  lp.tip = make(r2 + g.bucket_tooth_length * std::cos(th_k),
                z2 + g.bucket_tooth_length * std::sin(th_k));
  return lp;
}

/// 车厢四角（base 系）：center ± L/2·forward ± W/2·right
/// forward=(cosβ,sinβ)，right=(sinβ,-cosβ)（与 truck_dump_planner 约定一致）
void BoxCorners(const TruckPose& truck, double length, double width,
                geometry_msgs::Point out[4]) {
  const double fx = std::cos(truck.yaw_rad);
  const double fy = std::sin(truck.yaw_rad);
  const double rx = fy;
  const double ry = -fx;
  const double hl = 0.5 * length;
  const double hw = 0.5 * width;
  // 顺序：左前-右前-右后-左后（相对航向）
  const double s[4][2] = {{+hl, -hw}, {+hl, +hw}, {-hl, +hw}, {-hl, -hw}};
  for (int i = 0; i < 4; ++i) {
    out[i] = Pt(truck.center_x + s[i][0] * fx + s[i][1] * rx,
                truck.center_y + s[i][0] * fy + s[i][1] * ry,
                0.0);
  }
}

}  // namespace

// ==================== 构造函数 ====================

DumpVisualizer::DumpVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  enabled_ = pnh.param("visualization/enable", true);
  pnh.param<std::string>("visualization/frame_id", frame_id_, "base");

  pub_truck_ = nh.advertise<visualization_msgs::MarkerArray>("dump_viz/truck", 1);
  pub_excavator_ = nh.advertise<visualization_msgs::MarkerArray>("dump_viz/excavator", 1);
  pub_plan_ = nh.advertise<visualization_msgs::MarkerArray>("dump_viz/plan", 1);
  pub_segment_ = nh.advertise<visualization_msgs::MarkerArray>("dump_viz/segment", 1);
  pub_status_ = nh.advertise<visualization_msgs::Marker>("dump_viz/status", 1);

  ROS_INFO("[dump_viz] visualizer %s (frame=%s)",
           enabled_ ? "enabled" : "disabled", frame_id_.c_str());
}

visualization_msgs::Marker DumpVisualizer::MakeBase(const std::string& ns,
                                                    int id, int type) const {
  visualization_msgs::Marker m;
  m.header.frame_id = frame_id_;
  m.header.stamp = ros::Time::now();
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.frame_locked = true;
  return m;
}

// ==================== 卡车场景 ====================

void DumpVisualizer::PublishTruckScene(const TruckPose& truck,
                                       double box_length, double box_width,
                                       double truck_top_z, double gate_z,
                                       const Point3& unload_point) {
  if (!enabled_) return;

  visualization_msgs::MarkerArray arr;

  geometry_msgs::Point corners[4];
  BoxCorners(truck, box_length, box_width, corners);

  // ---- 车厢 3D 线框（底圈 + 顶圈 + 4 竖棱）----
  {
    auto m = MakeBase("truck_box", 0, visualization_msgs::Marker::LINE_LIST);
    SetColor(m, 1.0, 0.55, 0.0);  // 橙色
    m.scale.x = 0.06;
    for (int i = 0; i < 4; ++i) {
      const int j = (i + 1) % 4;
      geometry_msgs::Point b0 = corners[i];
      geometry_msgs::Point b1 = corners[j];
      geometry_msgs::Point t0 = b0; t0.z = truck_top_z;
      geometry_msgs::Point t1 = b1; t1.z = truck_top_z;
      m.points.push_back(b0);  m.points.push_back(b1);   // 底圈
      m.points.push_back(t0);  m.points.push_back(t1);   // 顶圈
      m.points.push_back(b0);  m.points.push_back(t0);   // 竖棱
    }
    arr.markers.push_back(m);
  }

  // ---- 航向箭头（中心 → 前方 0.6L）----
  {
    auto m = MakeBase("truck_heading", 1, visualization_msgs::Marker::ARROW);
    SetColor(m, 1.0, 0.9, 0.1);  // 黄色
    m.scale.x = 0.12;  // 杆径
    m.scale.y = 0.28;  // 头宽
    m.scale.z = 0.28;
    m.points.push_back(Pt(truck.center_x, truck.center_y, truck_top_z));
    m.points.push_back(Pt(truck.center_x + 0.6 * box_length * std::cos(truck.yaw_rad),
                          truck.center_y + 0.6 * box_length * std::sin(truck.yaw_rad),
                          truck_top_z));
    arr.markers.push_back(m);
  }

  // ---- 车厢中心球 ----
  {
    auto m = MakeBase("truck_center", 2, visualization_msgs::Marker::SPHERE);
    SetColor(m, 1.0, 0.1, 0.1);
    m.scale.x = m.scale.y = m.scale.z = 0.25;
    m.pose.position = Pt(truck.center_x, truck.center_y, truck_top_z);
    arr.markers.push_back(m);
  }

  // ---- 门控高度参考矩形（gate_z，较车厢外扩 0.3m，青色）----
  {
    geometry_msgs::Point gc[4];
    BoxCorners(truck, box_length + 0.6, box_width + 0.6, gc);
    auto m = MakeBase("gate_height", 3, visualization_msgs::Marker::LINE_STRIP);
    SetColor(m, 0.0, 0.9, 0.9, 0.9);
    m.scale.x = 0.04;
    for (int i = 0; i <= 4; ++i) {
      geometry_msgs::Point p = gc[i % 4];
      p.z = gate_z;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);

    auto t = MakeBase("gate_height", 4, visualization_msgs::Marker::TEXT_VIEW_FACING);
    SetColor(t, 0.0, 0.9, 0.9);
    t.scale.z = 0.3;
    char buf[32];
    snprintf(buf, sizeof(buf), "gate %.2fm", gate_z);
    t.text = buf;
    t.pose.position = Pt(gc[1].x, gc[1].y, gate_z + 0.25);
    arr.markers.push_back(t);
  }

  // ---- 卸载点（绿球 + 垂线 + 标签）----
  {
    auto m = MakeBase("unload_point", 5, visualization_msgs::Marker::SPHERE);
    SetColor(m, 0.1, 0.9, 0.2);
    m.scale.x = m.scale.y = m.scale.z = 0.28;
    m.pose.position = Pt(unload_point.x, unload_point.y, unload_point.z);
    arr.markers.push_back(m);

    auto ln = MakeBase("unload_point", 6, visualization_msgs::Marker::LINE_STRIP);
    SetColor(ln, 0.1, 0.9, 0.2, 0.5);
    ln.scale.x = 0.03;
    ln.points.push_back(Pt(unload_point.x, unload_point.y, 0.0));
    ln.points.push_back(Pt(unload_point.x, unload_point.y, unload_point.z));
    arr.markers.push_back(ln);

    auto t = MakeBase("unload_point", 7, visualization_msgs::Marker::TEXT_VIEW_FACING);
    SetColor(t, 0.1, 0.9, 0.2);
    t.scale.z = 0.3;
    char buf[64];
    snprintf(buf, sizeof(buf), "unload (%.1f,%.1f,%.1f)",
             unload_point.x, unload_point.y, unload_point.z);
    t.text = buf;
    t.pose.position = Pt(unload_point.x, unload_point.y, unload_point.z + 0.4);
    arr.markers.push_back(t);
  }

  pub_truck_.publish(arr);
}

// ==================== 挖掘机臂架 ====================

void DumpVisualizer::PublishExcavator(const kinematics::LinkGeometry& geom,
                                      const kinematics::JointState& q) {
  if (!enabled_) return;

  visualization_msgs::MarkerArray arr;
  const LinkPoints lp = ComputeLinkPoints(geom, q);

  // ---- 连杆线段：origin→boom_pivot→boom_arm→arm_bucket→tip ----
  {
    auto m = MakeBase("links", 0, visualization_msgs::Marker::LINE_LIST);
    SetColor(m, 0.85, 0.65, 0.1);  // 工程黄
    m.scale.x = 0.12;
    const geometry_msgs::Point* chain[5] = {
        &lp.origin, &lp.boom_pivot, &lp.boom_arm, &lp.arm_bucket, &lp.tip};
    for (int i = 0; i + 1 < 5; ++i) {
      m.points.push_back(*chain[i]);
      m.points.push_back(*chain[i + 1]);
    }
    arr.markers.push_back(m);
  }

  // ---- 铰点球 ----
  {
    auto m = MakeBase("joints", 1, visualization_msgs::Marker::SPHERE_LIST);
    SetColor(m, 0.45, 0.45, 0.45);
    m.scale.x = m.scale.y = m.scale.z = 0.18;
    m.points.push_back(lp.boom_pivot);
    m.points.push_back(lp.boom_arm);
    m.points.push_back(lp.arm_bucket);
    arr.markers.push_back(m);
  }

  // ---- 齿尖球（红，门控高度观察点）----
  {
    auto m = MakeBase("bucket_tip", 2, visualization_msgs::Marker::SPHERE);
    SetColor(m, 1.0, 0.15, 0.15);
    m.scale.x = m.scale.y = m.scale.z = 0.24;
    m.pose.position = lp.tip;
    arr.markers.push_back(m);
  }

  pub_excavator_.publish(arr);
}

// ==================== 规划结果 ====================

void DumpVisualizer::PublishPlan(
    const std::vector<kinematics::JointState>& waypoints,
    const kinematics::JointState& start_joint,
    const kinematics::JointState& unload_joint,
    const Point3& unload_point,
    const TruckPose& truck,
    const WaypointParams& params,
    const kinematics::KinematicsSolver& solver) {
  if (!enabled_ || waypoints.size() < 2) return;

  visualization_msgs::MarkerArray arr;

  // ---- WP 球体 + 标签 + 连线 ----
  auto wp_line = MakeBase("wp_path", 0, visualization_msgs::Marker::LINE_STRIP);
  SetColor(wp_line, 1.0, 1.0, 1.0, 0.55);
  wp_line.scale.x = 0.04;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    const auto pose = solver.swing_center_forward(waypoints[i]);
    const geometry_msgs::Point p = Pt(pose.x, pose.y, pose.z);
    wp_line.points.push_back(p);

    auto m = MakeBase("wp", static_cast<int>(i) * 2,
                      visualization_msgs::Marker::SPHERE);
    if (i == 0) {
      SetColor(m, 0.1, 0.9, 0.2);                    // WP1 起点：绿
    } else if (i == waypoints.size() - 1) {
      SetColor(m, 1.0, 0.2, 0.2);                    // WP6 终点：红
    } else if (i == 3) {
      SetColor(m, 0.2, 0.5, 1.0);                    // WP4 厢上过渡：蓝
    } else {
      SetColor(m, 1.0, 0.8, 0.0);                    // 其余：黄
    }
    m.scale.x = m.scale.y = m.scale.z = 0.24;
    m.pose.position = p;
    arr.markers.push_back(m);

    auto t = MakeBase("wp_label", static_cast<int>(i) * 2 + 1,
                      visualization_msgs::Marker::TEXT_VIEW_FACING);
    SetColor(t, 1.0, 1.0, 1.0);
    t.scale.z = 0.32;
    t.text = "WP" + std::to_string(i + 1);
    t.pose.position = Pt(p.x, p.y, p.z + 0.4);
    arr.markers.push_back(t);
  }
  arr.markers.push_back(wp_line);

  // ---- WP4 回转圆（与 CalcEntryPointCircleStrategy 同式）----
  {
    const auto dig_end = solver.swing_center_forward(start_joint);
    const double r_dig = std::hypot(dig_end.x, dig_end.y);
    const double r_unload = std::hypot(unload_point.x, unload_point.y);
    const double R = params.wp4_R_coff * r_dig +
                     (1.0 - params.wp4_R_coff) * r_unload;
    if (R > 1e-6) {
      auto m = MakeBase("swing_circle", 100,
                        visualization_msgs::Marker::LINE_STRIP);
      SetColor(m, 0.0, 0.9, 0.9, 0.8);
      m.scale.x = 0.05;
      constexpr int kSeg = 72;
      for (int i = 0; i <= kSeg; ++i) {
        const double th = 2.0 * kPi * static_cast<double>(i) / kSeg;
        m.points.push_back(Pt(R * std::cos(th), R * std::sin(th), 0.05));
      }
      arr.markers.push_back(m);

      auto t = MakeBase("swing_circle", 101,
                        visualization_msgs::Marker::TEXT_VIEW_FACING);
      SetColor(t, 0.0, 0.9, 0.9);
      t.scale.z = 0.3;
      char buf[32];
      snprintf(buf, sizeof(buf), "R=%.2fm", R);
      t.text = buf;
      t.pose.position = Pt(R, 0.0, 0.35);
      arr.markers.push_back(t);
    }
  }

  // ---- 入厢点（WP4 齿尖 xy，蓝球 + 垂线）----
  if (waypoints.size() > 3) {
    const auto wp4 = solver.swing_center_forward(waypoints[3]);
    auto m = MakeBase("entry_point", 102, visualization_msgs::Marker::SPHERE);
    SetColor(m, 0.2, 0.5, 1.0);
    m.scale.x = m.scale.y = m.scale.z = 0.2;
    m.pose.position = Pt(wp4.x, wp4.y, wp4.z);
    arr.markers.push_back(m);

    auto ln = MakeBase("entry_point", 103,
                       visualization_msgs::Marker::LINE_STRIP);
    SetColor(ln, 0.2, 0.5, 1.0, 0.5);
    ln.scale.x = 0.03;
    ln.points.push_back(Pt(wp4.x, wp4.y, 0.05));
    ln.points.push_back(Pt(wp4.x, wp4.y, wp4.z));
    arr.markers.push_back(ln);
  }

  // 抑制未使用参数告警（truck/unload_joint 预留扩展，如后续绘制车厢局部坐标系）
  (void)truck;
  (void)unload_joint;

  pub_plan_.publish(arr);
}

// ==================== 当前段轨迹 ====================

void DumpVisualizer::PublishSegmentTrajectory(
    const std::vector<kinematics::JointState>& traj,
    const kinematics::KinematicsSolver& solver) {
  if (!enabled_) return;

  if (traj.size() < 2) {
    ClearSegmentTrajectory();
    return;
  }

  visualization_msgs::MarkerArray arr;
  auto m = MakeBase("seg_traj", 0, visualization_msgs::Marker::LINE_STRIP);
  SetColor(m, 1.0, 0.9, 0.2, 0.95);  // 亮黄
  m.scale.x = 0.07;
  m.points.reserve(traj.size());
  for (const auto& q : traj) {
    const auto pose = solver.swing_center_forward(q);
    m.points.push_back(Pt(pose.x, pose.y, pose.z));
  }
  arr.markers.push_back(m);
  pub_segment_.publish(arr);
}

void DumpVisualizer::ClearSegmentTrajectory() {
  if (!enabled_) return;
  visualization_msgs::MarkerArray arr;
  auto m = MakeBase("seg_traj", 0, visualization_msgs::Marker::LINE_STRIP);
  m.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(m);
  pub_segment_.publish(arr);
}

// ==================== 状态文本 ====================

void DumpVisualizer::PublishStatus(const std::string& text) {
  if (!enabled_) return;
  auto m = MakeBase("status", 0, visualization_msgs::Marker::TEXT_VIEW_FACING);
  SetColor(m, 1.0, 1.0, 1.0);
  m.scale.z = 0.5;
  m.text = text;
  m.pose.position = Pt(0.0, 0.0, 7.5);
  pub_status_.publish(m);
}

}  // namespace dump_trajectory_planner
