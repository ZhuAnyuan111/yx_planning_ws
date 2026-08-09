#ifndef TRUCK_DUMP_PLANNER_TRUCK_MODEL_H
#define TRUCK_DUMP_PLANNER_TRUCK_MODEL_H

#include <array>
#include <vector>
#include "truck_dump_planner/coords.h"

namespace truck_dump_planner {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// 卡车车厢几何参数（单位：米 / 度）
// 说明：只包络车厢，不考虑车头；提供的卡车中心点即车厢中心。
struct TruckParams {
  double box_length = 5.0;   // 车厢长（纵轴，沿航向方向）
  double box_width = 2.6;    // 车厢宽（横轴，垂直航向）
  double box_height = 1.8;   // 车厢侧壁高度
};

// 卡车车厢在挖掘机坐标系下的包络
struct TruckEnvelope {
  Vec2 center;                       // 卡车中心点=车厢中心（挖掘机系）
  double z = 0.0;                    // 卡车地面高度
  double heading_beta = 0.0;         // 挖掘机系航向 β（度）
  double heading_rtk = 0.0;          // RTK 方位角 α（度，记录用）
  std::array<Vec2, 4> box_corners;   // 车厢四角: 前左/后左/后右/前右
  Vec2 box_center;                   // 车厢中心（=center）
  double box_top_z = 0.0;            // 车厢顶面高度
  Vec2 forward_dir;                  // 车厢纵轴正向（航向）单位向量
  Vec2 right_dir;                    // 车厢右侧单位向量
};

// 在挖掘机坐标系下构建卡车车厢包络
// truck_x/y/z: 卡车中心点位置（=车厢中心，挖掘机系）
// rtk_bearing_deg: 卡车 RTK 方位角 α（北0顺时针）
TruckEnvelope BuildTruckEnvelope(double truck_x, double truck_y, double truck_z,
                                 double rtk_bearing_deg,
                                 const TruckParams& params,
                                 AxisConvention conv);

// 射线法判断点是否在多边形内（碰撞/合法性检查）
bool PointInPolygon(const Vec2& p, const std::vector<Vec2>& poly);

}  // namespace truck_dump_planner

#endif  // TRUCK_DUMP_PLANNER_TRUCK_MODEL_H
