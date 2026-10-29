#include "clean_trajectory_planner/angle_utils.h"

#include <cmath>

namespace clean_trajectory_planner {

double NormalizeAngle(double angle) {
  angle = std::fmod(angle, 360.0);
  if (angle < 0.0) angle += 360.0;
  return angle;
}

double SignedShortestDiffDeg(double a_deg, double b_deg) {
  double d = std::fmod(b_deg - a_deg, 360.0);
  if (d > 180.0) d -= 360.0;
  if (d < -180.0) d += 360.0;
  return d;
}

double ShortestAngularDistanceDeg(double a_deg, double b_deg) {
  return std::abs(SignedShortestDiffDeg(a_deg, b_deg));
}

}  // namespace clean_trajectory_planner
