#include "dump_trajectory_planner/waypoint_generator.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "dump_trajectory_planner/angle_utils.h"

namespace dump_trajectory_planner {

namespace {

inline double Clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
inline double Sign(double x) { return (x > 0.0) - (x < 0.0); }

/// 线性插值：给定 (x1,y1) 与 (x2,y2)，返回 x 处的 y（x 会先被 clamp 到 [x1,x2]）
inline double LinearInterp(double x1, double x2, double y1, double y2,
                            double x) {
  if (std::abs(x2 - x1) < 1e-9) return y1;
  const double xl = std::min(x1, x2);
  const double xh = std::max(x1, x2);
  const double xc = Clampd(x, xl, xh);
  const double t = (xc - x1) / (x2 - x1);
  return y1 + t * (y2 - y1);
}

/// 铲斗姿态角限制：若 boom+arm+bkt ≥ attitude 上限，则 bkt = attitude - boom - arm（并 clamp 到限位）
inline double AdjustBucketByAttitude(double boom_deg, double arm_deg,
                                      double bkt_deg,
                                      double attitude_deg,
                                      double bkt_lower, double bkt_upper) {
  if (boom_deg + arm_deg + bkt_deg >= attitude_deg) {
    const double v = attitude_deg - boom_deg - arm_deg;
    return Clampd(v, bkt_lower, bkt_upper);
  }
  return bkt_deg;
}

/// 把 (swing_deg, boom_deg, arm_deg, bkt_deg) 打包为 kinematics::JointState（rad）
inline kinematics::JointState PackDeg(double sw, double bo, double ar,
                                       double bk) {
  kinematics::JointState q;
  q.swing = Deg2Rad(sw);
  q.boom = Deg2Rad(bo);
  q.arm = Deg2Rad(ar);
  q.bucket = Deg2Rad(bk);
  return q;
}

}  // namespace

// ==================== ComputeMiddleUpXY（回转圆策略） ====================

namespace {

/// MATLAB calculate_swing(x, y)：
///   swing_angle = atan2d(y, x)
///   swing_angle < 0 → +360
///   swing_angle += 180（与挖机 swing=180 时大臂指向 -x 约定对齐）
///   swing_angle > 360 → -360
inline double CalcSwingDeg(double x, double y) {
  double a = std::atan2(y, x) * 180.0 / kPi;
  if (a < 0.0) a += 360.0;
  a += 180.0;
  if (a > 360.0) a -= 360.0;
  return a;
}

/// MATLAB angle_diff：沿 rot_dir 方向从 A 到 B 的角度差（deg，[0, 360)）
/// rot_dir > 0 : CCW； rot_dir <= 0 : CW
inline double AngleDiffDeg(double a_deg, double b_deg, int rot_dir) {
  auto mod360 = [](double v) {
    v = std::fmod(v, 360.0);
    if (v < 0.0) v += 360.0;
    return v;
  };
  const double ts = mod360(a_deg);
  const double te = mod360(b_deg);
  const double ccw = mod360(te - ts);
  const double cw = mod360(ts - te);
  return (rot_dir > 0) ? ccw : cw;
}

/// MATLAB seg_check：点 P 到线段 AB 的最近点 Q 与距离平方；
/// 若 d² < best_d2 则更新 nearest / best_d2
inline void SegCheck(double px, double py,
                      double ax, double ay,
                      double bx, double by,
                      double& nx, double& ny,
                      double& best_d2) {
  const double abx = bx - ax;
  const double aby = by - ay;
  const double apx = px - ax;
  const double apy = py - ay;
  const double den = abx * abx + aby * aby;
  double qx, qy;
  if (den < 1e-9) {
    qx = ax;
    qy = ay;
  } else {
    double t = (apx * abx + apy * aby) / den;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
    qx = ax + t * abx;
    qy = ay + t * aby;
  }
  const double dx = qx - px;
  const double dy = qy - py;
  const double d2 = dx * dx + dy * dy;
  if (d2 < best_d2) {
    best_d2 = d2;
    nx = qx;
    ny = qy;
  }
}

/// 入厢点返回结果
struct EntryPointResult {
  int flag = 0;    // 0=失败(R 过小或退化); 1=圆-矩形交点; 2=fallback 最近边投影
  double x = 0.0;
  double y = 0.0;
};

/// MATLAB calc_entry_point_circle_strategy 的 C++ 翻译
/// - 参考 O 旋转中心固定在挖机 base 系原点 (0,0)；
/// - truck_yaw_rad 为卡车坐标系相对 base 系的旋转角 β（rad）；
/// - R_WT = [[cy,-sy],[sy,cy]]，将卡车本地坐标变换到 base 系。
EntryPointResult CalcEntryPointCircleStrategy(double dig_end_x, double dig_end_y,
                                              double unload_x, double unload_y,
                                              double R_coff,
                                              double truck_cx, double truck_cy,
                                              double truck_yaw_rad,
                                              double L, double W,
                                              int rot_dir) {
  EntryPointResult r;

  // ---- 1. 回转圆半径 ----
  const double R1 = std::hypot(dig_end_x, dig_end_y);
  const double R2 = std::hypot(unload_x, unload_y);
  const double R = R_coff * R1 + (1.0 - R_coff) * R2;
  if (R < 1e-6) {
    r.flag = 0;
    return r;
  }

  // ---- 2. 坐标变换 (base → truck-local) ----
  const double half_L = 0.5 * L;
  const double half_W = 0.5 * W;
  const double xmin = -half_L, xmax = half_L;
  const double ymin = -half_W, ymax = half_W;
  const double cy = std::cos(truck_yaw_rad);
  const double sy = std::sin(truck_yaw_rad);
  // O_local = R_WT' * (O - truck_center)；O = [0,0]，故代入化简
  const double dxO = -truck_cx;
  const double dyO = -truck_cy;
  const double Ox_l = cy * dxO + sy * dyO;
  const double Oy_l = -sy * dxO + cy * dyO;

  // ---- 3. 圆-矩形 4 边交点（truck-local）----
  struct Pt2 { double x, y; };
  Pt2 inter[8];
  int n_inter = 0;
  const double R2sq = R * R;

  auto push_inter = [&](double x, double y) {
    if (n_inter < 8) {
      inter[n_inter++] = {x, y};
    }
  };

  // x = xmin / xmax
  for (double xw : {xmin, xmax}) {
    const double b = Ox_l - xw;
    const double D2 = R2sq - b * b;
    if (D2 >= 0.0) {
      const double s = std::sqrt(D2);
      const double y1 = Oy_l + s;
      const double y2 = Oy_l - s;
      if (y1 >= ymin && y1 <= ymax) push_inter(xw, y1);
      if (y2 >= ymin && y2 <= ymax) push_inter(xw, y2);
    }
  }
  // y = ymin / ymax
  for (double yw : {ymin, ymax}) {
    const double d = Oy_l - yw;
    const double D2 = R2sq - d * d;
    if (D2 >= 0.0) {
      const double s = std::sqrt(D2);
      const double x1 = Ox_l + s;
      const double x2 = Ox_l - s;
      if (x1 >= xmin && x1 <= xmax) push_inter(x1, yw);
      if (x2 >= xmin && x2 <= xmax) push_inter(x2, yw);
    }
  }

  // ---- 4. 有交点：沿 rot_dir 取最小角度差 ----
  if (n_inter > 0) {
    const double dig_end_swing = CalcSwingDeg(dig_end_x, dig_end_y);
    double best_diff = 361.0;
    double best_x = unload_x, best_y = unload_y;
    bool found = false;
    for (int i = 0; i < n_inter; ++i) {
      const double ix = inter[i].x;
      const double iy = inter[i].y;
      if (ix * ix + iy * iy <= 0.0) continue;  // MATLAB: > 0 方可用
      // truck-local → base: R_WT * [ix; iy] + truck_center
      const double wx = cy * ix - sy * iy + truck_cx;
      const double wy = sy * ix + cy * iy + truck_cy;
      const double sw = CalcSwingDeg(wx, wy);
      const double diff = AngleDiffDeg(dig_end_swing, sw, rot_dir);
      if (diff < best_diff) {
        best_diff = diff;
        best_x = wx;
        best_y = wy;
        found = true;
      }
    }
    if (found) {
      r.flag = 1;
      r.x = best_x;
      r.y = best_y;
      return r;
    }
  }

  // ---- 5. fallback: 最近边缘 → 圆上投影 ----
  double best_d2 = 1e12;
  double nx = 0.0, ny = 0.0;
  SegCheck(Ox_l, Oy_l, xmin, ymin, xmin, ymax, nx, ny, best_d2);
  SegCheck(Ox_l, Oy_l, xmax, ymin, xmax, ymax, nx, ny, best_d2);
  SegCheck(Ox_l, Oy_l, xmin, ymin, xmax, ymin, nx, ny, best_d2);
  SegCheck(Ox_l, Oy_l, xmin, ymax, xmax, ymax, nx, ny, best_d2);

  const double vx = nx - Ox_l;
  const double vy = ny - Oy_l;
  const double vn = std::hypot(vx, vy);
  if (vn < 1e-6) {
    r.flag = 0;
    return r;
  }
  const double px_local = Ox_l + R * vx / vn;
  const double py_local = Oy_l + R * vy / vn;
  // truck-local → base
  r.x = cy * px_local - sy * py_local + truck_cx;
  r.y = sy * px_local + cy * py_local + truck_cy;
  r.flag = 2;
  return r;
}

}  // namespace

void ComputeMiddleUpXY(const Point3& unload_point,
                       const kinematics::JointState& start_joint,
                       const kinematics::JointState& /*unload_joint*/,
                       const TruckPose& truck_pose,
                       const kinematics::KinematicsSolver& solver,
                       const WaypointParams& p,
                       double& out_x,
                       double& out_y) {
  // dig_end 由 start_joint 的 FK 得到（铲齿尖 XY）
  const auto dig_end = solver.swing_center_forward(start_joint);
  const double dig_end_x = dig_end.x;
  const double dig_end_y = dig_end.y;

  // rot_dir 自动判定：从 dig_end swing 到 unload swing 的最短路径
  const double sw_dig = CalcSwingDeg(dig_end_x, dig_end_y);
  const double sw_unl = CalcSwingDeg(unload_point.x, unload_point.y);
  const double ccw = AngleDiffDeg(sw_dig, sw_unl, +1);
  const double cw = AngleDiffDeg(sw_dig, sw_unl, -1);
  const int rot_dir = (ccw <= cw) ? 1 : -1;

  const auto res = CalcEntryPointCircleStrategy(
      dig_end_x, dig_end_y, unload_point.x, unload_point.y,
      p.wp4_R_coff,
      truck_pose.center_x, truck_pose.center_y, truck_pose.yaw_rad,
      p.truck_box_length, p.truck_box_width,
      rot_dir);

  if (res.flag == 0) {
    // 退化：直接用卸载点 xy
    out_x = unload_point.x;
    out_y = unload_point.y;
  } else {
    out_x = res.x;
    out_y = res.y;
  }
}

// ==================== GenerateDumpWaypoints ====================
// 简化版：4 航路点 {WP1, WP4, WP5, WP6}，去掉 WP2/WP3 中间过渡点。
// WP1→WP4 由节点端的两阶段策略（boom 阶跃 + PCHIP）驱动，无需中间航路点。

DumpWaypointResult GenerateDumpWaypoints(
    const kinematics::JointState& start_joint,
    const kinematics::JointState& unload_joint,
    const Point3& unload_point,
    const TruckPose& truck_pose,
    const kinematics::KinematicsSolver& solver,
    const WaypointParams& p) {
  DumpWaypointResult result;

  // 转到 deg 空间
  const double sw1 = Rad2Deg(start_joint.swing);
  const double bo1 = Rad2Deg(start_joint.boom);
  const double ar1 = Rad2Deg(start_joint.arm);
  const double bk1_raw = Rad2Deg(start_joint.bucket);
  const double sw_unload = Rad2Deg(unload_joint.swing);
  const double bo_unload = Rad2Deg(unload_joint.boom);
  const double ar_unload = Rad2Deg(unload_joint.arm);
  const double bk_unload = Rad2Deg(unload_joint.bucket);

  const double attitude = p.attitude_angle_deg;
  const double bkt_lo = p.bucket_limit_lower_deg;
  const double bkt_hi = p.bucket_limit_upper_deg;

  // ---------- WP1: 起点（姿态角修正 bucket）----------
  const double bk1 = AdjustBucketByAttitude(bo1, ar1, bk1_raw, attitude,
                                             bkt_lo, bkt_hi);

  // ---------- WP4: 厢上过渡点，先 IK ----------
  double mx = 0.0, my = 0.0;
  ComputeMiddleUpXY(unload_point, start_joint, unload_joint, truck_pose, solver,
                    p, mx, my);

  // 卡车框高度 = 卡车中心 RTK 高度 - 偏移量
  const double truck_top_height = truck_pose.center_z - p.truck_height_offset;

  kinematics::SwingBaseIkRequest ik_req;
  ik_req.target_pose.x = mx;
  ik_req.target_pose.y = my;
  ik_req.target_pose.z = truck_top_height + p.wp4_height_bias;
  ik_req.target_pose.alpha = 0.0;
  ik_req.bucket_angle_deg = p.wp4_bucket_angle_deg;
  ik_req.bucket_tooth_length_override = 0.0;
  const auto ik = solver.swing_center_inverse_by_bucket_angle(ik_req);
  if (!ik.success) {
    std::ostringstream oss;
    oss << "WP4 IK failed: " << ik.message;
    result.message = oss.str();
    return result;
  }
  const double sw4 = Rad2Deg(ik.q.swing);
  const double bo4 = Rad2Deg(ik.q.boom);
  const double ar4 = Rad2Deg(ik.q.arm);

  // ---------- swing 展开（处理 0°/360° 跨界）----------
  // sw4 相对 sw1 展开：保证 |sw4u - sw1| ≤ 180°
  const double sw4u = sw1 + SignedShortestDiffDeg(sw1, sw4);

  // sw_unload 相对 sw4u 展开：保证 swing 链相邻差均 ≤ 180°
  const double sw_unl_u = sw4u + SignedShortestDiffDeg(sw4u, sw_unload);

  // ---------- WP4: bucket 姿态限制（基于 WP1 的 bucket）----------
  const double bk4 = AdjustBucketByAttitude(bo4, ar4, bk1, attitude,
                                             bkt_lo, bkt_hi);

  // ---------- WP5: swing 到位，boom/arm 混合，bucket 姿态限制且不倒退 ----------
  const double sw5 = sw_unl_u;
  const double bo5 = p.p5_boom_cof * bo4 + (1.0 - p.p5_boom_cof) * bo_unload;
  const double ar5 = p.p5_arm_cof * ar4 + (1.0 - p.p5_arm_cof) * ar_unload;
  double bk5 = AdjustBucketByAttitude(bo5, ar5, bk4, attitude, bkt_lo, bkt_hi);
  if (bk5 < bk4) bk5 = bk4;  // 防止铲斗倒退

  // ---------- WP6: 终点 ----------
  const double sw6 = sw_unl_u;
  const double bo6 = bo_unload;
  const double ar6 = ar_unload;
  const double bk6 = bk_unload;

  // ---------- 段时间（4 航路点 → 3 段）----------
  const double t1 = 0.0;

  // Seg0: WP1→WP4（boom+swing 联动，段时间取主导关节）
  const double t2_dur = std::max(
      std::abs(bo4 - bo1) / std::max(p.wp2_vel_boom_dps, 1e-6),
      std::abs(sw4u - sw1) / std::max(p.wp4_vel_swing_dps, 1e-6));
  const double t2 = t1 + t2_dur;

  // Seg1: WP4→WP5（swing 到位 + boom/arm 过渡）
  const double t3_dur = std::max({
      std::abs(sw5 - sw4u) / std::max(p.wp4_vel_swing_dps, 1e-6),
      std::abs(bo5 - bo4) / std::max(p.wp5_vel_boom_dps, 1e-6),
      std::abs(ar5 - ar4) / std::max(p.wp5_vel_arm_dps, 1e-6)});
  const double t3 = t2 + t3_dur;

  // Seg2: WP5→WP6（arm/bucket/boom 均在运动）
  const double t4_dur = std::max({
      std::abs(bk6 - bk5) / std::max(p.wp5_vel_bkt_dps, 1e-6),
      std::abs(bo6 - bo5) / std::max(p.wp5_vel_boom_dps, 1e-6),
      std::abs(ar6 - ar5) / std::max(p.wp5_vel_arm_dps, 1e-6)});
  const double t4 = t3 + t4_dur;

  // ---------- 打包（4 航路点）----------
  result.waypoints = {
      PackDeg(sw1, bo1, ar1, bk1),       // WP1: 起点
      PackDeg(sw4u, bo4, ar4, bk4),      // WP4: 厢上过渡点
      PackDeg(sw5, bo5, ar5, bk5),       // WP5: 卸载过渡点
      PackDeg(sw6, bo6, ar6, bk6),       // WP6: 终点
  };
  result.t_array = {t1, t2, t3, t4};
  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace dump_trajectory_planner
