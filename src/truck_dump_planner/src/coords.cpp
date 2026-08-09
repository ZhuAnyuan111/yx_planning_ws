#include "truck_dump_planner/coords.h"
#include <cmath>
 
namespace truck_dump_planner {

double Wrap360(double deg) {
  return deg - 360.0 * std::floor(deg / 360.0);
}

double RtkBearingToExcavatorHeading(double rtk_bearing_deg) {
  return Wrap360(180.0 - rtk_bearing_deg);
}

double ExcavatorHeadingToRtkBearing(double heading_deg) {
  return Wrap360(180.0 - heading_deg);
}

void HeadingToUnitVector(double heading_deg, AxisConvention conv,
                         double& dx, double& dy) {
  // 正北方向在指定 xy 约定下的单位向量 (nx, ny)
  double nx, ny;
  switch (conv) {
    case AxisConvention::kNorthWest:
      nx = 1.0;  ny = 0.0;   // x 轴朝北 -> 北=(1,0)
      break;
    case AxisConvention::kEastNorth:
    default:
      nx = 0.0;  ny = 1.0;   // y 轴朝北 -> 北=(0,1)
      break;
  }
  // β=180 对应正北；θ_geo = β - 180 为“相对正北逆时针”的地理方向角
  double theta = (heading_deg - 180.0) * kDeg2Rad;
  double c = std::cos(theta);
  double s = std::sin(theta);
  // 将北方向 (nx,ny) 绕原点逆时针旋转 θ_geo 得到目标方向
  dx = c * nx - s * ny;
  dy = s * nx + c * ny;
  double norm = std::hypot(dx, dy);
  if (norm > 1e-12) {
    dx /= norm;
    dy /= norm;
  }
}

void RightUnitVector(double dx, double dy, double& rx, double& ry) {
  // 顺时针旋转 90°：(x,y) -> (y,-x)
  rx = dy;
  ry = -dx;
}

}  // namespace truck_dump_planner
