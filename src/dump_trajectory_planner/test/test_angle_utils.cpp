/// @file test_angle_utils.cpp
/// @brief angle_utils 工具函数离线单元测试（纯 C++，无 ROS 运行时依赖）。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "dump_trajectory_planner/angle_utils.h"

using namespace dump_trajectory_planner;

// ==================== Deg2Rad / Rad2Deg ====================

TEST(AngleUtils, DegRadRoundTrip) {
  for (double d : {0.0, 45.0, 90.0, 180.0, -90.0, 360.0, 720.0}) {
    EXPECT_NEAR(d, Rad2Deg(Deg2Rad(d)), 1e-12);
  }
}

TEST(AngleUtils, Deg2RadKnown) {
  EXPECT_NEAR(Deg2Rad(180.0), kPi, 1e-12);
  EXPECT_NEAR(Deg2Rad(90.0), kPi / 2.0, 1e-12);
  EXPECT_NEAR(Deg2Rad(-180.0), -kPi, 1e-12);
}

// ==================== NormalizeRadToPi ====================

TEST(AngleUtils, NormalizeRadToPi_Zero) {
  EXPECT_NEAR(NormalizeRadToPi(0.0), 0.0, 1e-12);
}

TEST(AngleUtils, NormalizeRadToPi_Pi) {
  // π 应归一化到 π（区间为 (-π, π]）
  EXPECT_NEAR(NormalizeRadToPi(kPi), kPi, 1e-12);
}

TEST(AngleUtils, NormalizeRadToPi_NegPi) {
  // -π 应归一化到 π（-π 不在开区间内，+2π 后为 π）
  EXPECT_NEAR(NormalizeRadToPi(-kPi), kPi, 1e-12);
}

TEST(AngleUtils, NormalizeRadToPi_TwoPi) {
  EXPECT_NEAR(NormalizeRadToPi(2.0 * kPi), 0.0, 1e-12);
}

TEST(AngleUtils, NormalizeRadToPi_LargeValues) {
  // 10π → 归一化到 (-π, π]
  double r = NormalizeRadToPi(10.0 * kPi);
  EXPECT_TRUE(r > -kPi - 1e-12 && r <= kPi + 1e-12);
  EXPECT_NEAR(r, 0.0, 1e-12);  // 10π = 5×2π → 0
}

TEST(AngleUtils, NormalizeRadToPi_NegativeLarge) {
  double r = NormalizeRadToPi(-7.0 * kPi);
  EXPECT_TRUE(r > -kPi - 1e-12 && r <= kPi + 1e-12);
  // -7π = -3×2π - π → -π → 归一化到 π
  EXPECT_NEAR(std::abs(r), kPi, 1e-12);
}

// ==================== ShortestAngularDistanceDeg ====================

TEST(AngleUtils, ShortestDist_SameAngle) {
  EXPECT_NEAR(ShortestAngularDistanceDeg(45.0, 45.0), 0.0, 1e-12);
}

TEST(AngleUtils, ShortestDist_Opposite) {
  EXPECT_NEAR(ShortestAngularDistanceDeg(0.0, 180.0), 180.0, 1e-12);
}

TEST(AngleUtils, ShortestDist_AcrossZero) {
  // 350° 与 10° 的最短距离应为 20°（跨 0° 边界）
  EXPECT_NEAR(ShortestAngularDistanceDeg(350.0, 10.0), 20.0, 1e-12);
}

TEST(AngleUtils, ShortestDist_AcrossZero2) {
  // 10° 与 350° 同理
  EXPECT_NEAR(ShortestAngularDistanceDeg(10.0, 350.0), 20.0, 1e-12);
}

TEST(AngleUtils, ShortestDist_AlwaysNonNeg) {
  EXPECT_GE(ShortestAngularDistanceDeg(100.0, 200.0), 0.0);
  EXPECT_GE(ShortestAngularDistanceDeg(-30.0, 30.0), 0.0);
}

TEST(AngleUtils, ShortestDist_NegativeAngles) {
  // -10° 与 10°：距离 20°
  EXPECT_NEAR(ShortestAngularDistanceDeg(-10.0, 10.0), 20.0, 1e-12);
}

// ==================== SignedShortestDiffDeg ====================

TEST(AngleUtils, SignedDiff_CW) {
  // 从 10° 到 350°：最短路径是 -20°（顺时针）
  EXPECT_NEAR(SignedShortestDiffDeg(10.0, 350.0), -20.0, 1e-12);
}

TEST(AngleUtils, SignedDiff_CCW) {
  // 从 350° 到 10°：最短路径是 +20°（逆时针）
  EXPECT_NEAR(SignedShortestDiffDeg(350.0, 10.0), 20.0, 1e-12);
}

TEST(AngleUtils, SignedDiff_SameAngle) {
  EXPECT_NEAR(SignedShortestDiffDeg(90.0, 90.0), 0.0, 1e-12);
}

TEST(AngleUtils, SignedDiff_180) {
  // 恰好 180°：约定返回 +180（或 -180，取决于实现，这里检查绝对值）
  EXPECT_NEAR(std::abs(SignedShortestDiffDeg(0.0, 180.0)), 180.0, 1e-12);
}

// ==================== WrapTo360Deg ====================

TEST(AngleUtils, WrapTo360_Normal) {
  EXPECT_NEAR(WrapTo360Deg(45.0), 45.0, 1e-12);
}

TEST(AngleUtils, WrapTo360_Negative) {
  EXPECT_NEAR(WrapTo360Deg(-10.0), 350.0, 1e-12);
}

TEST(AngleUtils, WrapTo360_Over360) {
  EXPECT_NEAR(WrapTo360Deg(370.0), 10.0, 1e-12);
}

TEST(AngleUtils, WrapTo360_Exactly360) {
  EXPECT_NEAR(WrapTo360Deg(360.0), 0.0, 1e-12);
}

TEST(AngleUtils, WrapTo360_Zero) {
  EXPECT_NEAR(WrapTo360Deg(0.0), 0.0, 1e-12);
}

TEST(AngleUtils, WrapTo360_LargeNeg) {
  EXPECT_NEAR(WrapTo360Deg(-730.0), 350.0, 1e-12);  // -730 + 2*360 = -10 → 350
}

// ==================== UnwrapSwingSequence ====================

TEST(AngleUtils, UnwrapSwing_Normal) {
  // 不跨边界：展开后不变
  std::vector<kinematics::JointState> wps(3);
  wps[0].swing = Deg2Rad(30.0);
  wps[1].swing = Deg2Rad(60.0);
  wps[2].swing = Deg2Rad(90.0);
  UnwrapSwingSequence(wps);
  EXPECT_NEAR(Rad2Deg(wps[1].swing), 60.0, 1e-10);
  EXPECT_NEAR(Rad2Deg(wps[2].swing), 90.0, 1e-10);
}

TEST(AngleUtils, UnwrapSwing_AcrossZero) {
  // 从 350° → 10°：展开后第二点应为 370°（即 350+20），而非 10°
  std::vector<kinematics::JointState> wps(2);
  wps[0].swing = Deg2Rad(350.0);
  wps[1].swing = Deg2Rad(10.0);
  UnwrapSwingSequence(wps);
  EXPECT_NEAR(Rad2Deg(wps[1].swing), 370.0, 1e-10);
}

TEST(AngleUtils, UnwrapSwing_Multiple) {
  // 连续跨越：300° → 30° → 60°
  std::vector<kinematics::JointState> wps(3);
  wps[0].swing = Deg2Rad(300.0);
  wps[1].swing = Deg2Rad(30.0);   // diff = +90° → 300+90=390
  wps[2].swing = Deg2Rad(60.0);   // diff from 390: 60-390=-330 → normalize=+30 → 420
  UnwrapSwingSequence(wps);
  EXPECT_NEAR(Rad2Deg(wps[1].swing), 390.0, 1e-10);
  EXPECT_NEAR(Rad2Deg(wps[2].swing), 420.0, 1e-10);
}

// ==================== MaxJointErrorDeg ====================

TEST(AngleUtils, MaxJointError_Zero) {
  kinematics::JointState a{Deg2Rad(45.0), Deg2Rad(30.0), Deg2Rad(-60.0), Deg2Rad(-90.0)};
  EXPECT_NEAR(MaxJointErrorDeg(a, a), 0.0, 1e-12);
}

TEST(AngleUtils, MaxJointError_SwingDominates) {
  kinematics::JointState a{Deg2Rad(10.0), 0.0, 0.0, 0.0};
  kinematics::JointState b{Deg2Rad(50.0), 0.0, 0.0, 0.0};
  EXPECT_NEAR(MaxJointErrorDeg(a, b), 40.0, 1e-10);
}

TEST(AngleUtils, MaxJointError_SwingWrap) {
  // swing 跨 0/360 时应用最短角距离：350° vs 10° = 20°
  kinematics::JointState a{Deg2Rad(350.0), 0.0, 0.0, 0.0};
  kinematics::JointState b{Deg2Rad(10.0), 0.0, 0.0, 0.0};
  EXPECT_NEAR(MaxJointErrorDeg(a, b), 20.0, 1e-10);
}

TEST(AngleUtils, MaxJointError_BoomDominates) {
  kinematics::JointState a{0.0, Deg2Rad(10.0), 0.0, 0.0};
  kinematics::JointState b{0.0, Deg2Rad(45.0), 0.0, 0.0};
  EXPECT_NEAR(MaxJointErrorDeg(a, b), 35.0, 1e-10);
}
