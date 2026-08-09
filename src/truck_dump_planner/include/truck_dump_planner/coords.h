#ifndef TRUCK_DUMP_PLANNER_COORDS_H
#define TRUCK_DUMP_PLANNER_COORDS_H

namespace truck_dump_planner {

// 角度常量
constexpr double kDeg2Rad = 0.017453292519943295;   // pi/180
constexpr double kRad2Deg = 57.29577951308232;      // 180/pi

// 把角度归一化到 [0, 360)
double Wrap360(double deg);

// 挖掘机系 / RTK 航向角转换
//   挖掘机系航向 β：北180、东90、南0、西270，逆时针增大
//   RTK 方位角   α：北0、东90、南180、西270，顺时针增大
//   β = (180 - α) mod 360
double RtkBearingToExcavatorHeading(double rtk_bearing_deg);
double ExcavatorHeadingToRtkBearing(double heading_deg);

// 挖掘机坐标系 xy 轴地理朝向约定
enum class AxisConvention {
  kEastNorth = 0,  // x=东, y=北 (ENU 式，默认)
  kNorthWest = 1,  // x=北, y=西
};

// 挖掘机系航向 β -> xy 平面车头方向单位向量 (dx, dy)
// （把 β=180/90/0/270 对应北/东/南/西 分解到指定 xy 约定下）
void HeadingToUnitVector(double heading_deg, AxisConvention conv,
                         double& dx, double& dy);

// 车头方向 -> 车体右侧单位向量（xy 平面内顺时针旋转 90°）：(dy, -dx)
void RightUnitVector(double dx, double dy, double& rx, double& ry);

}  // namespace truck_dump_planner

#endif  // TRUCK_DUMP_PLANNER_COORDS_H
