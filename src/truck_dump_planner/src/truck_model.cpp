#include "truck_dump_planner/truck_model.h"
#include <cmath>

namespace truck_dump_planner {

// 以 center 为参考点，沿车头方向 d / 右侧方向 r 构造矩形四角（逆时针）。
// center_offset: 矩形中心沿 d 方向相对参考点的偏移（正=偏前）。
// 输出顺序: 前左 -> 后左 -> 后右 -> 前右
static void RectCorners(const Vec2& center, double dx, double dy,
                        double rx, double ry, double length, double width,
                        double center_offset, std::array<Vec2, 4>& out) {
  double hl = length / 2.0;
  double hw = width / 2.0;
  double mx = center.x + dx * center_offset;
  double my = center.y + dy * center_offset;
  out[0] = {mx + dx * hl - rx * hw, my + dy * hl - ry * hw};  // 前左
  out[1] = {mx - dx * hl - rx * hw, my - dy * hl - ry * hw};  // 后左
  out[2] = {mx - dx * hl + rx * hw, my - dy * hl + ry * hw};  // 后右
  out[3] = {mx + dx * hl + rx * hw, my + dy * hl + ry * hw};  // 前右
}

TruckEnvelope BuildTruckEnvelope(double truck_x, double truck_y, double truck_z,
                                 double rtk_bearing_deg,
                                 const TruckParams& params,
                                 AxisConvention conv) {
  TruckEnvelope env;
  env.center = {truck_x, truck_y};
  env.box_center = {truck_x, truck_y};  // 车厢中心 = 卡车中心点
  env.z = truck_z;
  env.heading_rtk = rtk_bearing_deg;
  env.heading_beta = RtkBearingToExcavatorHeading(rtk_bearing_deg);

  double dx, dy, rx, ry;
  HeadingToUnitVector(env.heading_beta, conv, dx, dy);
  RightUnitVector(dx, dy, rx, ry);
  env.forward_dir = {dx, dy};
  env.right_dir = {rx, ry};

  // 车厢四角：以卡车中心点为中心，沿航向/横向按车厢尺寸构建（无偏移）
  RectCorners(env.center, dx, dy, rx, ry, params.box_length, params.box_width,
              0.0, env.box_corners);
  env.box_top_z = truck_z + params.box_height;
  return env;
}

bool PointInPolygon(const Vec2& p, const std::vector<Vec2>& poly) {
  int n = static_cast<int>(poly.size());
  if (n < 3) return false;
  bool inside = false;
  int j = n - 1;
  for (int i = 0; i < n; ++i) {
    double xi = poly[i].x, yi = poly[i].y;
    double xj = poly[j].x, yj = poly[j].y;
    if (((yi > p.y) != (yj > p.y)) &&
        (p.x < (xj - xi) * (p.y - yi) / ((yj - yi) + 1e-12) + xi)) {
      inside = !inside;
    }
    j = i;
  }
  return inside;
}

}  // namespace truck_dump_planner
