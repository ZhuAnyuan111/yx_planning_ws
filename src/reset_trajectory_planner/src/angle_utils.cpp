#include "reset_trajectory_planner/angle_utils.h"

#include <cmath>

namespace reset_trajectory_planner {

double NormalizeAngle(double angle) {
  angle = std::fmod(angle, 360.0);
  if (angle < 0.0) angle += 360.0;
  return angle;
}

double CalculateAngleDiff(double a, double b, int direction) {
  a = NormalizeAngle(a);
  b = NormalizeAngle(b);
  if (direction == 1) {
    // 顺时针：从 a 出发角度减小方向到达 b
    return (b <= a) ? (a - b) : (a + (360.0 - b));
  } else {
    // 逆时针：从 a 出发角度增大方向到达 b
    return (b >= a) ? (b - a) : ((360.0 - a) + b);
  }
}

int DetermineDirection(double start, double goal) {
  const double cw = CalculateAngleDiff(start, goal, 1);
  const double ccw = CalculateAngleDiff(start, goal, -1);
  return (cw <= ccw) ? 1 : -1;
}

}  // namespace reset_trajectory_planner