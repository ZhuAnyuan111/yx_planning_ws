#include "dump_trajectory_planner/angle_utils.h"

#include <algorithm>
#include <cmath>

namespace dump_trajectory_planner {

double NormalizeRadToPi(double rad) {
  while (rad > kPi) rad -= kTwoPi;
  while (rad <= -kPi) rad += kTwoPi;
  return rad;
}

double ShortestAngularDistanceDeg(double a_deg, double b_deg) {
  return std::abs(SignedShortestDiffDeg(a_deg, b_deg));
}

double SignedShortestDiffDeg(double a_deg, double b_deg) {
  double d = std::fmod(b_deg - a_deg, 360.0);
  if (d > 180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

double WrapTo360Deg(double deg) {
  deg = std::fmod(deg, 360.0);
  if (deg < 0.0) deg += 360.0;
  return deg;
}

void UnwrapSwingSequence(std::vector<kinematics::JointState>& waypoints) {
  for (size_t i = 1; i < waypoints.size(); ++i) {
    double diff = waypoints[i].swing - waypoints[i - 1].swing;
    diff = NormalizeRadToPi(diff);
    waypoints[i].swing = waypoints[i - 1].swing + diff;
  }
}

double MaxJointErrorDeg(const kinematics::JointState& a,
                        const kinematics::JointState& b) {
  double err = 0.0;
  // swing：使用最短角距离，处理 0°/360° 边界
  err = std::max(err, ShortestAngularDistanceDeg(Rad2Deg(a.swing),
                                                 Rad2Deg(b.swing)));
  err = std::max(err, std::abs(Rad2Deg(a.boom - b.boom)));
  err = std::max(err, std::abs(Rad2Deg(a.arm - b.arm)));
  err = std::max(err, std::abs(Rad2Deg(a.bucket - b.bucket)));
  return err;
}

}  // namespace dump_trajectory_planner
