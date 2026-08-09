#pragma once

/// @file dump_visualizer.h
/// @brief 卸载轨迹规划节点可视化（RViz 实机联调辅助）。
///
/// 持有 5 个可视化话题（frame_id 默认 base，受 visualization/enable 总开关控制）：
///   dump_viz/truck     MarkerArray  卡车车厢包络线框 + 航向箭头 + 门控高度参考线
///                                    + 卸载点（激活时发布一次）
///   dump_viz/excavator MarkerArray  挖掘机臂架线段（回转中心→动臂销轴→动臂·斗杆铰
///                                    →斗杆·铲斗铰→齿尖）+ 铰点球 + 齿尖球（随指令帧高频发布）
///   dump_viz/plan      MarkerArray  WP1-WP6 球体/标签 + 航路点连线 + WP4 回转圆
///                                    + 入厢点（激活时发布一次）
///   dump_viz/segment   MarkerArray  当前段局部轨迹折线（每段在线重规划时更新）
///   dump_viz/status    Marker       状态机调试文本（phase/段进度/门控次数/铲斗高度，高频发布）
///
/// 设计约定：
///   - 臂架线段与 kinematics::KinematicsSolver::swing_center_forward 严格同源
///     （角度累积约定：θ_boom=q.boom, θ_arm=q.boom+q.arm, θ_bkt=q.boom+q.arm+q.bucket；
///      径向按 swing 旋转，y 向叠加 boom_pivot_y）；
///   - 回转圆半径与 waypoint_generator 的 CalcEntryPointCircleStrategy 同式：
///     R = wp4_R_coff·|dig_end_xy| + (1-wp4_R_coff)·|unload_xy|，圆心为回转中心 (0,0)；
///   - 车厢四角与 truck_dump_planner 约定一致：forward=(cosβ,sinβ)，right=(sinβ,-cosβ)。

#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <kinematics/kinematics.hpp>

#include "dump_trajectory_planner/dump_feasibility.h"
#include "dump_trajectory_planner/waypoint_generator.h"

namespace dump_trajectory_planner {

class DumpVisualizer {
 public:
  /// @param nh  全局 NodeHandle（话题为全局命名）
  /// @param pnh 私有 NodeHandle（读取 visualization/enable、visualization/frame_id）
  DumpVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  /// 卡车场景：车厢 3D 线框（z=0 至 truck_top_z）+ 航向箭头 + 中心球
  ///           + 门控高度参考矩形（gate_z，较车厢外扩 0.3m）+ 卸载点标记
  void PublishTruckScene(const TruckPose& truck,
                         double box_length,
                         double box_width,
                         double truck_top_z,
                         double gate_z,
                         const Point3& unload_point);

  /// 挖掘机臂架线段（随执行指令帧发布，展示当前指令姿态）
  void PublishExcavator(const kinematics::LinkGeometry& geom,
                        const kinematics::JointState& q);

  /// 规划结果：WP 球体+标签、航路点连线、回转圆、入厢点
  /// @param start_joint  挖掘终止点关节角（用于回转圆半径的 dig_end 项）
  /// @param unload_joint 卸载点关节角（用于回转圆半径的 unload 项）
  void PublishPlan(const std::vector<kinematics::JointState>& waypoints,
                   const kinematics::JointState& start_joint,
                   const kinematics::JointState& unload_joint,
                   const Point3& unload_point,
                   const TruckPose& truck,
                   const WaypointParams& params,
                   const kinematics::KinematicsSolver& solver);

  /// 当前段局部轨迹（关节角逐帧 FK 为笛卡尔折线）；traj 为空时清空该层
  void PublishSegmentTrajectory(const std::vector<kinematics::JointState>& traj,
                                const kinematics::KinematicsSolver& solver);

  /// 状态机调试文本（固定悬浮于场景上方）
  void PublishStatus(const std::string& text);

  /// 清空段轨迹层（执行结束/撤销时调用）
  void ClearSegmentTrajectory();

 private:
  /// 构建通用 Marker 骨架（frame/stamp/ns/id/type/ADD/单位四元数）
  visualization_msgs::Marker MakeBase(const std::string& ns, int id,
                                      int type) const;

  bool enabled_;
  std::string frame_id_;

  ros::Publisher pub_truck_;
  ros::Publisher pub_excavator_;
  ros::Publisher pub_plan_;
  ros::Publisher pub_segment_;
  ros::Publisher pub_status_;
};

}  // namespace dump_trajectory_planner
