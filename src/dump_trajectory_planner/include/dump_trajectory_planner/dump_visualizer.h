#pragma once

/// @file dump_visualizer.h
/// @brief 卸载轨迹规划节点可视化（RViz 实机联调辅助）。
///
/// 持有 7 个可视化话题（frame_id 默认 base，受 visualization/enable 总开关控制）：
///   dump_viz/truck     MarkerArray  卡车车厢包络线框 +（可选）mesh 模型 + 航向箭头
///                                    + 门控高度参考线 + 卸载点（激活时发布一次）
///   dump_viz/excavator MarkerArray  挖掘机臂架线段（回转中心→动臂销轴→动臂·斗杆铰
///                                    →斗杆·铲斗铰→齿尖）+ 铰点球 + 齿尖球（随指令帧高频发布）
///   dump_viz/plan      MarkerArray  WP1-WP6 球体/标签 + 航路点连线 + WP4 回转圆
///                                    + 入厢点（激活时发布一次）
///   dump_viz/segment   MarkerArray  当前段局部轨迹折线（每段在线重规划时更新）
///   dump_viz/status    Marker       状态机调试文本（phase/段进度/门控次数/铲斗高度，高频发布）
///   dump_viz/candidates MarkerArray 卸载候选点分布（可达亮蓝/不可达暗灰）+ far/mid/near
///                                    代表点 + 选中卸载点（预校验/激活时发布一次）
///   dump_viz/debug     MarkerArray  回转圆策略调试层（受 visualization/debug 开关控制）：
///                                    R1/R2/R 三圆 + dig_end/unload 射线 + 全部候选交点
///                                    + 选中交点 + rot_dir 方向弧 + fallback 投影线
///                                    + 触发源/WP1 快照/R/flag 文字（预校验/激活时发布一次）
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

/// 卸载候选点可视化数据（由 DumpTrajectoryNode::SelectUnloadPoint 填充）
/// - points / reachable 一一对应：全部候选点（index 0=最远, n-1=最近）及其 IK 可达标志
/// - far/mid/near_idx：可达域三分代表点索引（“装满点”分层），-1 表示无效
/// - selected_idx：按斗数三段式选中的卸载点索引，-1 表示无效
/// - valid：有候选数据时置真；false 时清空图层
struct UnloadCandidateViz {
  std::vector<Point3> points;
  std::vector<char> reachable;
  int far_idx = -1;
  int mid_idx = -1;
  int near_idx = -1;
  int selected_idx = -1;
  bool valid = false;
};

/// 规划调试信息（由 DumpTrajectoryNode::PlanWaypoints 填充，用于 dump_viz/debug）
/// - trigger    : 触发源 "PRECHECK"(/Sys_SUn_FlagUnloadPlan=1) / "ACTIVATE"(/Sys_SeqAction=4)
/// - plan_time  : 本次规划时刻
/// - wp1_joint  : 触发瞬间的实时关节角（=WP1 来源快照，用于暴露两次计算的分叉）
/// - wp4_joint  : 入厢点航路点关节角（内部 WP4）；FK 得齿尖，与 entry(铰接点) 对照
/// - has_wp4    : wp4_joint 是否有效（航路点数组含下标 1）
/// - entry      : 回转圆入厢点策略中间量（候选交点 / rot_dir / R1R2R / flag）
struct PlanDebugInfo {
  std::string trigger;
  ros::Time plan_time;
  kinematics::JointState wp1_joint;
  kinematics::JointState wp4_joint;
  bool has_wp4 = false;
  EntryPointDebug entry;
};

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

  /// 卸载候选点分布：全部候选点（可达亮蓝 / 不可达暗灰）+ 候选连线
  /// + far/mid/near 三个代表点（“装满点”分层）+ 选中卸载点（金色大球+标签）；
  /// cands.valid=false 或候选为空时清空该图层
  void PublishUnloadCandidates(const UnloadCandidateViz& cands);

  /// 回转圆策略调试层（受 visualization/debug 开关控制）：
  /// R1/R2/R 三条回转圆 + dig_end/unload 射线 + 全部候选交点（灰）+ 选中交点（洋红）
  /// + rot_dir 方向弧箭头 + fallback 最近边投影线 + 触发源/WP1 快照/R/flag 文字标注。
  /// 入厢点区分【铰接点 PIVOT】(洋红，=entry 圆∩厢边，IK override=0 的目标) 与
  /// 【齿尖 TOOTH】(蓝，=plan 层 WP2，swing_center_forward 结果)，并画铲斗跨度线，
  /// 说明二者是同一铲斗两端（相距 ~bucket_tooth_length）而非矛盾点。
  /// dbg.entry.valid=false 时清空该图层。
  void PublishDebug(const PlanDebugInfo& dbg,
                    const Point3& unload_point,
                    const TruckPose& truck,
                    const WaypointParams& params,
                    const kinematics::KinematicsSolver& solver);

  /// 清空段轨迹层（执行结束/撤销时调用）
  void ClearSegmentTrajectory();

 private:
  /// 构建通用 Marker 骨架（frame/stamp/ns/id/type/ADD/单位四元数）
  visualization_msgs::Marker MakeBase(const std::string& ns, int id,
                                      int type) const;

  bool enabled_;
  std::string frame_id_;

  // ---- 卡车 mesh 模型叠加（方式A，随卡车实时位姿摆放）----
  bool truck_mesh_enable_ = false;         // 总开关：false 时行为与纯线框完全一致
  bool truck_mesh_keep_wire_ = true;       // 叠加 mesh 时是否保留原车厢线框（调试对照）
  std::string truck_mesh_resource_;        // package://... 或 file://... mesh 路径
  double truck_mesh_scale_ = 1.0;          // 均匀缩放系数
  double truck_mesh_z_offset_ = 0.0;       // mesh 底面相对 base 原点的高度偏置 (m)
  double truck_mesh_yaw_offset_ = 0.0;     // mesh 车头方向相对车厢 forward(β) 的偏置 (rad)
  // mesh 自身原点常不在车厢中心/地面：以下三项把模型局部"对齐点"摆到卡车位姿、
  // 并把局部地面抬到 base z=0（SKT90E.dae 实测：车厢中心≈(0,-5.6)、胎底 z≈-1.477）
  double truck_mesh_align_cx_ = 0.0;       // 对齐点在 mesh 局部系的 x (m)
  double truck_mesh_align_cy_ = 0.0;       // 对齐点在 mesh 局部系的 y (m)
  double truck_mesh_ground_z_ = 0.0;       // mesh 局部地面(最低点) z (m)

  ros::Publisher pub_truck_;
  ros::Publisher pub_excavator_;
  ros::Publisher pub_plan_;
  ros::Publisher pub_segment_;
  ros::Publisher pub_status_;
  ros::Publisher pub_candidates_;
  ros::Publisher pub_debug_;

  bool debug_enabled_ = true;  // visualization/debug：回转圆策略调试层开关
};

}  // namespace dump_trajectory_planner
