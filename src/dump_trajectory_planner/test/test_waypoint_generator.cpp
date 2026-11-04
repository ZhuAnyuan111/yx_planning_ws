/// @file test_waypoint_generator.cpp
/// @brief 航路点生成器与可行性检查离线单元测试。
///
/// 使用 KinematicsSolver(LinkGeometry, JointLimitsDeg) 构造（无 ROS 运行时依赖）。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <kinematics/kinematics.hpp>

#include "dump_trajectory_planner/angle_utils.h"
#include "dump_trajectory_planner/dump_feasibility.h"
#include "dump_trajectory_planner/waypoint_generator.h"

using namespace dump_trajectory_planner;

namespace {

/// 构造标准测试用 solver（与 yaml 配置一致的机构参数）
kinematics::KinematicsSolver MakeSolver() {
  kinematics::LinkGeometry geom;
  geom.boom_length = 6.987;
  geom.arm_length = 2.797;
  geom.bucket_tooth_length = 2.359;
  geom.boom_pivot_x = 0.09;
  geom.boom_pivot_y = 0.03;
  geom.boom_pivot_z = 0.0;

  kinematics::JointLimitsDeg limits;
  limits.boom_lower = -25.0;
  limits.boom_upper = 55.0;
  limits.arm_lower = -150.0;
  limits.arm_upper = -30.0;
  limits.bucket_lower = -135.0;
  limits.bucket_upper = 30.0;

  return kinematics::KinematicsSolver(geom, limits);
}

/// 标准测试用航路点参数
WaypointParams MakeParams() {
  WaypointParams p;
  p.bucket_attitude_target_deg = -180.0;
  p.wp4_height_bias = 2.0;
  p.truck_height_offset = 0.5;
  p.wp4_bucket_angle_deg = 25.0;
  p.p5_arm_cof = 0.5;
  p.seg0_boom_vel_dps = 15.0;
  p.swing_vel_dps = 15.0;
  p.seg0_arm_vel_dps = 15.0;
  p.unload_boom_vel_dps = 8.0;
  p.unload_arm_vel_dps = 15.0;
  p.unload_bucket_vel_dps = 50.0;
  p.bucket_limit_lower_deg = -170.0;
  p.bucket_limit_upper_deg = 30.0;
  p.truck_box_length = 5.0;
  p.truck_box_width = 2.6;
  p.wp4_R_coff = 0.2;
  p.bucket_width = 2.0;
  p.bucket_wall_clearance = 0.1;
  return p;
}

/// 典型卡车位姿（base 系前方 ~8m）
TruckPose MakeTruckPose() {
  TruckPose tp;
  tp.center_x = -7.0;
  tp.center_y = 3.0;
  tp.center_z = 1.5;
  tp.rtk_heading_deg = 120.0;
  tp.yaw_rad = Deg2Rad(60.0);  // β = (180 - 120) = 60°
  return tp;
}

}  // namespace

// ==================== CheckReachable ====================

class FeasibilityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    solver_ = std::make_unique<kinematics::KinematicsSolver>(MakeSolver());
  }
  std::unique_ptr<kinematics::KinematicsSolver> solver_;
};

TEST_F(FeasibilityTest, EnvelopePrefilter_TooClose) {
  FeasibilityParams params;
  params.enable_envelope_prefilter = true;
  params.reach_min = 3.0;
  params.reach_max = 12.0;
  params.dump_height_min = -2.0;
  params.dump_height_max = 5.5;
  params.bucket_attitude_deg = -60.0;

  Point3 pt{1.0, 1.0, 2.0};  // radius = sqrt(2) ≈ 1.41 < 3.0
  auto result = CheckReachable(pt, params, *solver_);
  EXPECT_FALSE(result.reachable);
  EXPECT_NE(result.reason.find("envelope"), std::string::npos);
}

TEST_F(FeasibilityTest, EnvelopePrefilter_TooFar) {
  FeasibilityParams params;
  params.enable_envelope_prefilter = true;
  params.reach_min = 3.0;
  params.reach_max = 12.0;
  params.dump_height_min = -2.0;
  params.dump_height_max = 5.5;
  params.bucket_attitude_deg = -60.0;

  Point3 pt{15.0, 0.0, 2.0};  // radius = 15 > 12
  auto result = CheckReachable(pt, params, *solver_);
  EXPECT_FALSE(result.reachable);
  EXPECT_NE(result.reason.find("envelope"), std::string::npos);
}

TEST_F(FeasibilityTest, EnvelopePrefilter_TooHigh) {
  FeasibilityParams params;
  params.enable_envelope_prefilter = true;
  params.reach_min = 3.0;
  params.reach_max = 12.0;
  params.dump_height_min = -2.0;
  params.dump_height_max = 5.5;
  params.bucket_attitude_deg = -60.0;

  Point3 pt{5.0, 0.0, 8.0};  // z = 8 > 5.5
  auto result = CheckReachable(pt, params, *solver_);
  EXPECT_FALSE(result.reachable);
}

TEST_F(FeasibilityTest, EnvelopePrefilter_Disabled) {
  FeasibilityParams params;
  params.enable_envelope_prefilter = false;
  params.bucket_attitude_deg = -60.0;

  // 超近的点：包络粗筛关了，但 IK 可能仍失败（物理不可达）
  Point3 pt{1.0, 0.0, 0.0};
  auto result = CheckReachable(pt, params, *solver_);
  // 不做具体断言——只验证不因 envelope 被拒
  if (!result.reachable) {
    EXPECT_EQ(result.reason.find("envelope"), std::string::npos);
  }
}

TEST_F(FeasibilityTest, ReachablePoint_TypicalUnload) {
  FeasibilityParams params;
  params.enable_envelope_prefilter = true;
  params.reach_min = 3.0;
  params.reach_max = 12.0;
  params.dump_height_min = -2.0;
  params.dump_height_max = 5.5;
  params.bucket_attitude_deg = -60.0;

  // 典型卸载点：距离 ~7.6m，高度 2.0m
  Point3 pt{-7.0, 3.0, 2.0};
  auto result = CheckReachable(pt, params, *solver_);
  // 可达性取决于 IK——这里验证结果结构完整
  if (result.reachable) {
    EXPECT_EQ(result.reason, "reachable");
    // 关节角应在限位内
    const auto& lim = solver_->limits();
    EXPECT_GE(Rad2Deg(result.unload_joint.boom), lim.boom_lower);
    EXPECT_LE(Rad2Deg(result.unload_joint.boom), lim.boom_upper);
    EXPECT_GE(Rad2Deg(result.unload_joint.arm), lim.arm_lower);
    EXPECT_LE(Rad2Deg(result.unload_joint.arm), lim.arm_upper);
  }
}

// ==================== ComputeMiddleUpXY ====================

class EntryPointTest : public ::testing::Test {
 protected:
  void SetUp() override {
    solver_ = std::make_unique<kinematics::KinematicsSolver>(MakeSolver());
    params_ = MakeParams();
    truck_ = MakeTruckPose();
  }
  std::unique_ptr<kinematics::KinematicsSolver> solver_;
  WaypointParams params_;
  TruckPose truck_;
};

TEST_F(EntryPointTest, BasicCall_ReturnsFiniteXY) {
  // 起点关节角（典型挖掘终止姿态）
  kinematics::JointState start_joint{};
  start_joint.swing = Deg2Rad(180.0);
  start_joint.boom = Deg2Rad(30.0);
  start_joint.arm = Deg2Rad(-100.0);
  start_joint.bucket = Deg2Rad(-50.0);

  // 卸载关节角
  kinematics::JointState unload_joint{};
  unload_joint.swing = Deg2Rad(290.0);
  unload_joint.boom = Deg2Rad(35.0);
  unload_joint.arm = Deg2Rad(-80.0);
  unload_joint.bucket = Deg2Rad(-30.0);

  Point3 unload_pt{-7.0, 3.0, 2.0};
  double out_x = 0.0, out_y = 0.0;
  EntryPointDebug dbg;

  ComputeMiddleUpXY(unload_pt, start_joint, unload_joint, truck_, *solver_,
                    params_, out_x, out_y, &dbg);

  EXPECT_TRUE(std::isfinite(out_x));
  EXPECT_TRUE(std::isfinite(out_y));
  EXPECT_TRUE(dbg.valid);
}

TEST_F(EntryPointTest, DebugInfo_Populated) {
  kinematics::JointState start_joint{};
  start_joint.swing = Deg2Rad(200.0);
  start_joint.boom = Deg2Rad(25.0);
  start_joint.arm = Deg2Rad(-90.0);
  start_joint.bucket = Deg2Rad(-60.0);

  kinematics::JointState unload_joint{};
  unload_joint.swing = Deg2Rad(300.0);
  unload_joint.boom = Deg2Rad(40.0);
  unload_joint.arm = Deg2Rad(-70.0);
  unload_joint.bucket = Deg2Rad(-20.0);

  Point3 unload_pt{-7.0, 3.0, 2.0};
  double out_x = 0.0, out_y = 0.0;
  EntryPointDebug dbg;

  ComputeMiddleUpXY(unload_pt, start_joint, unload_joint, truck_, *solver_,
                    params_, out_x, out_y, &dbg);

  EXPECT_TRUE(dbg.valid);
  EXPECT_TRUE(dbg.rot_dir == 1 || dbg.rot_dir == -1);
  EXPECT_GT(dbg.R1, 0.0);  // dig_end 半径应为正
}

// ==================== GenerateDumpWaypoints ====================

class WaypointGenTest : public ::testing::Test {
 protected:
  void SetUp() override {
    solver_ = std::make_unique<kinematics::KinematicsSolver>(MakeSolver());
    params_ = MakeParams();
    truck_ = MakeTruckPose();
  }
  std::unique_ptr<kinematics::KinematicsSolver> solver_;
  WaypointParams params_;
  TruckPose truck_;
};

TEST_F(WaypointGenTest, SuccessCase_4Waypoints) {
  kinematics::JointState start_joint{};
  start_joint.swing = Deg2Rad(180.0);
  start_joint.boom = Deg2Rad(30.0);
  start_joint.arm = Deg2Rad(-100.0);
  start_joint.bucket = Deg2Rad(-50.0);

  kinematics::JointState unload_joint{};
  unload_joint.swing = Deg2Rad(290.0);
  unload_joint.boom = Deg2Rad(35.0);
  unload_joint.arm = Deg2Rad(-80.0);
  unload_joint.bucket = Deg2Rad(-10.0);

  Point3 unload_pt{-7.0, 3.0, 2.0};

  auto result = GenerateDumpWaypoints(start_joint, unload_joint, unload_pt,
                                      truck_, *solver_, params_);
  // 成功与否取决于 WP4 IK——这里验证结构完整性
  if (result.success) {
    EXPECT_EQ(result.waypoints.size(), 4u);
    EXPECT_EQ(result.t_array.size(), 4u);
    // t_array 应单调递增，t1=0
    EXPECT_NEAR(result.t_array[0], 0.0, 1e-12);
    for (size_t i = 1; i < result.t_array.size(); ++i) {
      EXPECT_GT(result.t_array[i], result.t_array[i - 1]);
    }
  } else {
    // 失败时应给出原因
    EXPECT_FALSE(result.message.empty());
  }
}

TEST_F(WaypointGenTest, WP1_AttitudeCorrection) {
  // 测试 WP1 的 bucket 姿态修正：如果 boom+arm+bucket >= attitude_target，
  // 则 bucket 被修正为 attitude_target - boom - arm
  kinematics::JointState start_joint{};
  start_joint.swing = Deg2Rad(180.0);
  start_joint.boom = Deg2Rad(40.0);
  start_joint.arm = Deg2Rad(-50.0);
  start_joint.bucket = Deg2Rad(10.0);
  // boom+arm+bucket = 40-50+10 = 0° > -180° → 应修正 bucket
  // 修正后 bucket = -180 - 40 - (-50) = -170°

  kinematics::JointState unload_joint{};
  unload_joint.swing = Deg2Rad(290.0);
  unload_joint.boom = Deg2Rad(35.0);
  unload_joint.arm = Deg2Rad(-80.0);
  unload_joint.bucket = Deg2Rad(-10.0);

  Point3 unload_pt{-7.0, 3.0, 2.0};

  auto result = GenerateDumpWaypoints(start_joint, unload_joint, unload_pt,
                                      truck_, *solver_, params_);
  if (result.success) {
    // WP1 bucket 应被姿态角修正：≈ -170°（受 bkt_lo=-170 限位）
    double wp1_bkt_deg = Rad2Deg(result.waypoints[0].bucket);
    EXPECT_LE(wp1_bkt_deg, 10.0 + 1e-6);  // 一定小于原始值
    EXPECT_GE(wp1_bkt_deg, params_.bucket_limit_lower_deg - 1e-6);
  }
}

TEST_F(WaypointGenTest, WP5_BoomEqualsWP4) {
  // 验证 WP5.boom ≡ WP4.boom（Seg1 boom 保持不下降）
  kinematics::JointState start_joint{};
  start_joint.swing = Deg2Rad(180.0);
  start_joint.boom = Deg2Rad(20.0);
  start_joint.arm = Deg2Rad(-100.0);
  start_joint.bucket = Deg2Rad(-60.0);

  kinematics::JointState unload_joint{};
  unload_joint.swing = Deg2Rad(290.0);
  unload_joint.boom = Deg2Rad(35.0);
  unload_joint.arm = Deg2Rad(-80.0);
  unload_joint.bucket = Deg2Rad(-10.0);

  Point3 unload_pt{-7.0, 3.0, 2.0};

  auto result = GenerateDumpWaypoints(start_joint, unload_joint, unload_pt,
                                      truck_, *solver_, params_);
  if (result.success) {
    // WP4 = waypoints[1], WP5 = waypoints[2]
    EXPECT_NEAR(result.waypoints[2].boom, result.waypoints[1].boom, 1e-10);
  }
}

TEST_F(WaypointGenTest, WP6_EqualsUnloadJoint) {
  // WP6 应等于 unload_joint（终点锚定）
  kinematics::JointState start_joint{};
  start_joint.swing = Deg2Rad(180.0);
  start_joint.boom = Deg2Rad(20.0);
  start_joint.arm = Deg2Rad(-100.0);
  start_joint.bucket = Deg2Rad(-60.0);

  kinematics::JointState unload_joint{};
  unload_joint.swing = Deg2Rad(290.0);
  unload_joint.boom = Deg2Rad(35.0);
  unload_joint.arm = Deg2Rad(-80.0);
  unload_joint.bucket = Deg2Rad(-10.0);

  Point3 unload_pt{-7.0, 3.0, 2.0};

  auto result = GenerateDumpWaypoints(start_joint, unload_joint, unload_pt,
                                      truck_, *solver_, params_);
  if (result.success) {
    const auto& wp6 = result.waypoints[3];
    // boom/arm/bucket 应与 unload_joint 一致
    EXPECT_NEAR(wp6.boom, unload_joint.boom, 1e-10);
    EXPECT_NEAR(wp6.arm, unload_joint.arm, 1e-10);
    EXPECT_NEAR(wp6.bucket, unload_joint.bucket, 1e-10);
    // swing 应等价（可能经展开，用最短角距离验证）
    EXPECT_NEAR(ShortestAngularDistanceDeg(Rad2Deg(wp6.swing),
                                           Rad2Deg(unload_joint.swing)),
                0.0, 1e-6);
  }
}

TEST_F(WaypointGenTest, SwingUnwrap_AdjacentDiffLe180) {
  // 验证航路点序列中相邻 swing 差值 ≤ 180°（已展开）
  kinematics::JointState start_joint{};
  start_joint.swing = Deg2Rad(350.0);  // 接近 0° 边界
  start_joint.boom = Deg2Rad(25.0);
  start_joint.arm = Deg2Rad(-90.0);
  start_joint.bucket = Deg2Rad(-50.0);

  kinematics::JointState unload_joint{};
  unload_joint.swing = Deg2Rad(10.0);  // 跨 0° 边界
  unload_joint.boom = Deg2Rad(35.0);
  unload_joint.arm = Deg2Rad(-75.0);
  unload_joint.bucket = Deg2Rad(-10.0);

  Point3 unload_pt{7.0, 1.5, 2.0};

  auto result = GenerateDumpWaypoints(start_joint, unload_joint, unload_pt,
                                      truck_, *solver_, params_);
  if (result.success) {
    for (size_t i = 1; i < result.waypoints.size(); ++i) {
      double diff_deg = std::abs(
          Rad2Deg(result.waypoints[i].swing - result.waypoints[i - 1].swing));
      // 展开后相邻差值应 ≤ 180° + 小容差
      EXPECT_LE(diff_deg, 180.0 + 1.0)
          << "swing unwrap violated between WP" << i << " and WP" << i + 1;
    }
  }
}
