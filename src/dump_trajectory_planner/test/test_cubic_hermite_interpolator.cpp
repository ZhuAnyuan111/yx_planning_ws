/// @file test_cubic_hermite_interpolator.cpp
/// @brief PCHIP 插值器离线单元测试：端点精度、帧数、单调性、无超调。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "dump_trajectory_planner/angle_utils.h"
#include "dump_trajectory_planner/cubic_hermite_interpolator.h"

using namespace dump_trajectory_planner;

namespace {

/// 辅助：构造单关节非零的 JointState（boom 分量）
kinematics::JointState MakeBoom(double boom_rad) {
  kinematics::JointState q{};
  q.boom = boom_rad;
  return q;
}

/// 辅助：检查轨迹是否对 boom 分量单调递增
bool IsMonotonicBoom(const std::vector<kinematics::JointState>& traj) {
  for (size_t i = 1; i < traj.size(); ++i) {
    if (traj[i].boom < traj[i - 1].boom - 1e-12) return false;
  }
  return true;
}

/// 辅助：检查轨迹是否对 boom 分量单调递减
bool IsMonotonicBoomDec(const std::vector<kinematics::JointState>& traj) {
  for (size_t i = 1; i < traj.size(); ++i) {
    if (traj[i].boom > traj[i - 1].boom + 1e-12) return false;
  }
  return true;
}

}  // namespace

// ==================== 端点精度 ====================

TEST(HermiteInterpolator, EndpointAccuracy_ZeroTangents) {
  auto q0 = MakeBoom(Deg2Rad(10.0));
  auto q1 = MakeBoom(Deg2Rad(40.0));
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 3.0, 0.1);

  ASSERT_GE(traj.size(), 2u);
  // 首帧 ≈ 起点
  EXPECT_NEAR(traj.front().boom, q0.boom, 1e-10);
  // 末帧 == 终点（精确）
  EXPECT_NEAR(traj.back().boom, q1.boom, 1e-12);
}

TEST(HermiteInterpolator, EndpointAccuracy_NonZeroTangents) {
  auto q0 = MakeBoom(Deg2Rad(20.0));
  auto q1 = MakeBoom(Deg2Rad(50.0));
  kinematics::JointState v0{}, v1{};
  v0.boom = Deg2Rad(10.0);   // 10 deg/s
  v1.boom = Deg2Rad(5.0);    // 5 deg/s

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 2.0, 0.05);

  ASSERT_GE(traj.size(), 2u);
  EXPECT_NEAR(traj.front().boom, q0.boom, 1e-10);
  EXPECT_NEAR(traj.back().boom, q1.boom, 1e-12);
}

// ==================== 帧数验证 ====================

TEST(HermiteInterpolator, FrameCount) {
  auto q0 = MakeBoom(Deg2Rad(0.0));
  auto q1 = MakeBoom(Deg2Rad(30.0));
  kinematics::JointState v0{}, v1{};

  const double duration = 2.0;
  const double dt = 0.1;
  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, duration, dt);

  // 预期帧数 = floor(duration/dt) + 1（末帧精确终点）
  // 2.0/0.1 = 20 步 → 约 20+1=21 帧（while 循环 + 末帧 push）
  EXPECT_GE(traj.size(), 20u);
  EXPECT_LE(traj.size(), 22u);
}

TEST(HermiteInterpolator, FrameCount_SmallDuration) {
  auto q0 = MakeBoom(Deg2Rad(0.0));
  auto q1 = MakeBoom(Deg2Rad(5.0));
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 0.5, 0.1);
  EXPECT_GE(traj.size(), 5u);
  EXPECT_LE(traj.size(), 7u);
}

// ==================== 退化情况 ====================

TEST(HermiteInterpolator, ZeroDuration_ReturnsEndpoint) {
  auto q0 = MakeBoom(Deg2Rad(10.0));
  auto q1 = MakeBoom(Deg2Rad(40.0));
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 0.0, 0.1);

  ASSERT_EQ(traj.size(), 1u);
  EXPECT_NEAR(traj[0].boom, q1.boom, 1e-12);
}

TEST(HermiteInterpolator, NegativeDuration_ReturnsEndpoint) {
  auto q0 = MakeBoom(Deg2Rad(10.0));
  auto q1 = MakeBoom(Deg2Rad(40.0));
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, -1.0, 0.1);

  ASSERT_EQ(traj.size(), 1u);
  EXPECT_NEAR(traj[0].boom, q1.boom, 1e-12);
}

TEST(HermiteInterpolator, SameStartEnd) {
  auto q = MakeBoom(Deg2Rad(25.0));
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q, q, v0, v1, 2.0, 0.1);

  // 全程应保持在同一角度
  for (const auto& frame : traj) {
    EXPECT_NEAR(frame.boom, q.boom, 1e-10);
  }
}

// ==================== 单调性（PCHIP 核心保证）====================

TEST(HermiteInterpolator, Monotonicity_IncreasingZeroTangent) {
  auto q0 = MakeBoom(Deg2Rad(-20.0));
  auto q1 = MakeBoom(Deg2Rad(45.0));
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 4.0, 0.05);
  EXPECT_TRUE(IsMonotonicBoom(traj));
}

TEST(HermiteInterpolator, Monotonicity_DecreasingZeroTangent) {
  auto q0 = MakeBoom(Deg2Rad(50.0));
  auto q1 = MakeBoom(Deg2Rad(10.0));
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 3.0, 0.05);
  EXPECT_TRUE(IsMonotonicBoomDec(traj));
}

TEST(HermiteInterpolator, Monotonicity_IncreasingWithTangent) {
  auto q0 = MakeBoom(Deg2Rad(0.0));
  auto q1 = MakeBoom(Deg2Rad(30.0));
  kinematics::JointState v0{}, v1{};
  v0.boom = Deg2Rad(15.0);  // 与弦斜率同向
  v1.boom = Deg2Rad(15.0);

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 2.0, 0.05);
  EXPECT_TRUE(IsMonotonicBoom(traj));
}

// ==================== PCHIP 无超调（Fritsch-Carlson 修正）====================

TEST(HermiteInterpolator, NoOvershoot_LargeStartTangent) {
  // 起点切线远大于弦斜率：PCHIP 应修正为 ≤3×弦斜率，不超调终点
  auto q0 = MakeBoom(Deg2Rad(0.0));
  auto q1 = MakeBoom(Deg2Rad(10.0));
  kinematics::JointState v0{}, v1{};
  v0.boom = Deg2Rad(100.0);  // 极大起点速度（弦斜率仅 10/2=5 deg/s）
  v1.boom = Deg2Rad(0.0);

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 2.0, 0.01);

  // 所有帧的 boom 应在 [q0.boom, q1.boom] 范围内（不超调）
  for (const auto& frame : traj) {
    EXPECT_GE(frame.boom, q0.boom - 1e-10);
    EXPECT_LE(frame.boom, q1.boom + 1e-10);
  }
}

TEST(HermiteInterpolator, NoOvershoot_LargeEndTangent) {
  auto q0 = MakeBoom(Deg2Rad(0.0));
  auto q1 = MakeBoom(Deg2Rad(20.0));
  kinematics::JointState v0{}, v1{};
  v0.boom = Deg2Rad(0.0);
  v1.boom = Deg2Rad(200.0);  // 极大终点速度

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 2.0, 0.01);

  for (const auto& frame : traj) {
    EXPECT_GE(frame.boom, q0.boom - 1e-10);
    EXPECT_LE(frame.boom, q1.boom + 1e-10);
  }
}

TEST(HermiteInterpolator, NoOvershoot_OpositeTangent) {
  // 起点切线反向（与弦斜率异号）：应被截断为 0，仍单调
  auto q0 = MakeBoom(Deg2Rad(0.0));
  auto q1 = MakeBoom(Deg2Rad(30.0));
  kinematics::JointState v0{}, v1{};
  v0.boom = Deg2Rad(-20.0);  // 反向
  v1.boom = Deg2Rad(10.0);

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 3.0, 0.05);
  EXPECT_TRUE(IsMonotonicBoom(traj));
}

// ==================== 多关节同时插值 ====================

TEST(HermiteInterpolator, MultiJoint_AllEndpointExact) {
  kinematics::JointState q0{Deg2Rad(100.0), Deg2Rad(20.0), Deg2Rad(-80.0), Deg2Rad(-120.0)};
  kinematics::JointState q1{Deg2Rad(200.0), Deg2Rad(45.0), Deg2Rad(-60.0), Deg2Rad(-10.0)};
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 5.0, 0.1);

  ASSERT_GE(traj.size(), 2u);
  // 首帧
  EXPECT_NEAR(traj.front().swing, q0.swing, 1e-10);
  EXPECT_NEAR(traj.front().boom, q0.boom, 1e-10);
  EXPECT_NEAR(traj.front().arm, q0.arm, 1e-10);
  EXPECT_NEAR(traj.front().bucket, q0.bucket, 1e-10);
  // 末帧（精确）
  EXPECT_NEAR(traj.back().swing, q1.swing, 1e-12);
  EXPECT_NEAR(traj.back().boom, q1.boom, 1e-12);
  EXPECT_NEAR(traj.back().arm, q1.arm, 1e-12);
  EXPECT_NEAR(traj.back().bucket, q1.bucket, 1e-12);
}

TEST(HermiteInterpolator, MultiJoint_NoOvershootPerJoint) {
  kinematics::JointState q0{Deg2Rad(10.0), Deg2Rad(20.0), Deg2Rad(-100.0), Deg2Rad(-130.0)};
  kinematics::JointState q1{Deg2Rad(50.0), Deg2Rad(40.0), Deg2Rad(-70.0), Deg2Rad(-20.0)};
  kinematics::JointState v0{}, v1{};

  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, 4.0, 0.05);

  for (const auto& f : traj) {
    // 每个关节都应在 [min(q0,q1), max(q0,q1)] 范围内
    EXPECT_GE(f.swing, std::min(q0.swing, q1.swing) - 1e-10);
    EXPECT_LE(f.swing, std::max(q0.swing, q1.swing) + 1e-10);
    EXPECT_GE(f.boom, std::min(q0.boom, q1.boom) - 1e-10);
    EXPECT_LE(f.boom, std::max(q0.boom, q1.boom) + 1e-10);
    EXPECT_GE(f.arm, std::min(q0.arm, q1.arm) - 1e-10);
    EXPECT_LE(f.arm, std::max(q0.arm, q1.arm) + 1e-10);
    EXPECT_GE(f.bucket, std::min(q0.bucket, q1.bucket) - 1e-10);
    EXPECT_LE(f.bucket, std::max(q0.bucket, q1.bucket) + 1e-10);
  }
}

// ==================== 时间连续性 ====================

TEST(HermiteInterpolator, Continuity_NoLargeJumps) {
  // 帧间跳变不应超过 (行程/帧数) 的 3 倍（PCHIP 修正后切线 ≤ 3δ）
  auto q0 = MakeBoom(Deg2Rad(0.0));
  auto q1 = MakeBoom(Deg2Rad(60.0));
  kinematics::JointState v0{}, v1{};
  v0.boom = Deg2Rad(30.0);

  const double duration = 3.0;
  const double dt = 0.1;
  auto traj = InterpolateSegmentWithTangents(q0, q1, v0, v1, duration, dt);

  const double max_step = Rad2Deg(q1.boom - q0.boom) / (duration / dt) * 3.5;
  for (size_t i = 1; i < traj.size(); ++i) {
    double step = std::abs(Rad2Deg(traj[i].boom - traj[i - 1].boom));
    EXPECT_LE(step, max_step) << "jump at frame " << i;
  }
}
