/// @file test_velocity_estimator.cpp
/// @brief 双路关节速度估计器离线单元测试。

#include <gtest/gtest.h>

#include <cmath>

#include "dump_trajectory_planner/angle_utils.h"
#include "dump_trajectory_planner/velocity_estimator.h"

using namespace dump_trajectory_planner;

namespace {
constexpr double kMaxVelRad = 30.0 * kPi / 180.0;  // 30 deg/s 限幅
}

// ==================== 空/不足历史 ====================

TEST(VelocityEstimator, EmptyHistory_ReturnsZero) {
  VelocityEstimator est;
  auto v = est.Estimate();
  EXPECT_NEAR(v.swing, 0.0, 1e-12);
  EXPECT_NEAR(v.boom, 0.0, 1e-12);
  EXPECT_NEAR(v.arm, 0.0, 1e-12);
  EXPECT_NEAR(v.bucket, 0.0, 1e-12);
}

TEST(VelocityEstimator, InsufficientHistory_Swing) {
  VelocityEstimator est;
  // 只 push 2 帧 swing → 不足 3 帧，返回 0
  est.PushSwing(0.0, Deg2Rad(10.0));
  est.PushSwing(0.1, Deg2Rad(12.0));
  auto v = est.Estimate();
  EXPECT_NEAR(v.swing, 0.0, 1e-12);
}

TEST(VelocityEstimator, InsufficientHistory_Joints) {
  VelocityEstimator est;
  kinematics::JointState q1{}, q2{};
  q1.boom = Deg2Rad(20.0);
  q2.boom = Deg2Rad(22.0);
  est.PushJoints(0.0, q1);
  est.PushJoints(0.1, q2);
  auto v = est.Estimate();
  EXPECT_NEAR(v.boom, 0.0, 1e-12);
}

// ==================== 正常估计 ====================

TEST(VelocityEstimator, ConstantVelocity_Swing) {
  VelocityEstimator est;
  // 匀速 15 deg/s，每 0.1s 一帧
  const double vel_dps = 15.0;
  const double dt = 0.1;
  for (int i = 0; i < 5; ++i) {
    est.PushSwing(i * dt, Deg2Rad(100.0 + vel_dps * i * dt));
  }
  auto v = est.Estimate();
  EXPECT_NEAR(Rad2Deg(v.swing), vel_dps, 0.1);
}

TEST(VelocityEstimator, ConstantVelocity_Boom) {
  VelocityEstimator est;
  const double vel_dps = 10.0;
  const double dt = 0.1;
  for (int i = 0; i < 5; ++i) {
    kinematics::JointState q{};
    q.boom = Deg2Rad(20.0 + vel_dps * i * dt);
    q.arm = Deg2Rad(-80.0);
    q.bucket = Deg2Rad(-100.0);
    est.PushJoints(i * dt, q);
  }
  auto v = est.Estimate();
  EXPECT_NEAR(Rad2Deg(v.boom), vel_dps, 0.1);
  EXPECT_NEAR(Rad2Deg(v.arm), 0.0, 0.1);  // arm 不动
}

TEST(VelocityEstimator, ThreeFrames_Sufficient) {
  VelocityEstimator est;
  // 恰好 3 帧即可估计
  est.PushSwing(0.0, Deg2Rad(10.0));
  est.PushSwing(0.1, Deg2Rad(12.0));
  est.PushSwing(0.2, Deg2Rad(14.0));
  auto v = est.Estimate();
  // 首末差分：(14-10) / 0.2 = 20 deg/s
  EXPECT_NEAR(Rad2Deg(v.swing), 20.0, 0.5);
}

// ==================== Swing 跨 0°/360° 处理 ====================

TEST(VelocityEstimator, SwingWrap_CrossZero) {
  VelocityEstimator est;
  // swing 从 350° 匀速到 10°（跨 0°）：实际角速度 = +20°/0.2s = 100 deg/s
  // 但限幅到 30 deg/s
  est.PushSwing(0.0, Deg2Rad(350.0));
  est.PushSwing(0.1, Deg2Rad(355.0));
  est.PushSwing(0.2, Deg2Rad(0.0));  // 等价 360°
  auto v = est.Estimate();
  // NormalizeRadToPi(0° - 350°) = NormalizeRadToPi(10°) = 10° → /0.2 = 50 deg/s → 限幅 30
  EXPECT_NEAR(Rad2Deg(v.swing), 30.0, 0.5);
}

TEST(VelocityEstimator, SwingWrap_NoPseudoLargeValue) {
  VelocityEstimator est;
  // 从 5° 到 355°（实际逆向 -10°/0.2s），不应得到 +350°/0.2s 的伪大值
  est.PushSwing(0.0, Deg2Rad(5.0));
  est.PushSwing(0.1, Deg2Rad(0.0));
  est.PushSwing(0.2, Deg2Rad(355.0));
  auto v = est.Estimate();
  // NormalizeRadToPi(355° - 5°) = NormalizeRadToPi(-10°) = -10° → /0.2 = -50 deg/s → 限幅 -30
  EXPECT_NEAR(Rad2Deg(v.swing), -30.0, 0.5);
}

// ==================== 限幅 ====================

TEST(VelocityEstimator, Clamping_LargeVelocity) {
  VelocityEstimator est;
  // 极大角速度：100°/0.1s = 1000 deg/s → 应被限幅到 30 deg/s
  est.PushSwing(0.0, Deg2Rad(0.0));
  est.PushSwing(0.05, Deg2Rad(50.0));
  est.PushSwing(0.1, Deg2Rad(100.0));
  auto v = est.Estimate();
  EXPECT_LE(std::abs(v.swing), kMaxVelRad + 1e-10);
}

TEST(VelocityEstimator, Clamping_NegativeVelocity) {
  VelocityEstimator est;
  kinematics::JointState q0{}, q1{}, q2{};
  q0.boom = Deg2Rad(50.0);
  q1.boom = Deg2Rad(0.0);
  q2.boom = Deg2Rad(-50.0);
  est.PushJoints(0.0, q0);
  est.PushJoints(0.05, q1);
  est.PushJoints(0.1, q2);
  auto v = est.Estimate();
  EXPECT_GE(v.boom, -kMaxVelRad - 1e-10);
}

// ==================== Clear ====================

TEST(VelocityEstimator, Clear_ResetsAll) {
  VelocityEstimator est;
  // 先填充足够历史
  for (int i = 0; i < 5; ++i) {
    est.PushSwing(i * 0.1, Deg2Rad(10.0 * i));
    kinematics::JointState q{};
    q.boom = Deg2Rad(20.0 + 5.0 * i);
    est.PushJoints(i * 0.1, q);
  }
  auto v_before = est.Estimate();
  EXPECT_NE(v_before.swing, 0.0);
  EXPECT_NE(v_before.boom, 0.0);

  // Clear 后应全部归零
  est.Clear();
  auto v_after = est.Estimate();
  EXPECT_NEAR(v_after.swing, 0.0, 1e-12);
  EXPECT_NEAR(v_after.boom, 0.0, 1e-12);
  EXPECT_NEAR(v_after.arm, 0.0, 1e-12);
  EXPECT_NEAR(v_after.bucket, 0.0, 1e-12);
}

// ==================== 历史窗口滑动 ====================

TEST(VelocityEstimator, SlidingWindow) {
  VelocityEstimator est;
  // push 超过 kHistorySize=5 帧，验证只使用最新 3 帧
  // 前 3 帧：速度 100 deg/s（被限幅）
  est.PushSwing(0.0, Deg2Rad(0.0));
  est.PushSwing(0.01, Deg2Rad(1.0));
  est.PushSwing(0.02, Deg2Rad(2.0));
  // 后 3 帧：速度 10 deg/s
  est.PushSwing(1.0, Deg2Rad(10.0));
  est.PushSwing(1.1, Deg2Rad(11.0));
  est.PushSwing(1.2, Deg2Rad(12.0));

  auto v = est.Estimate();
  // 最近 3 帧：(12-10)/0.2 = 10 deg/s
  EXPECT_NEAR(Rad2Deg(v.swing), 10.0, 0.5);
}

// ==================== 两路独立性 ====================

TEST(VelocityEstimator, IndependentPaths) {
  VelocityEstimator est;
  // 只 push swing，不 push joints → joints 分量应为 0
  est.PushSwing(0.0, Deg2Rad(10.0));
  est.PushSwing(0.1, Deg2Rad(12.0));
  est.PushSwing(0.2, Deg2Rad(14.0));

  auto v = est.Estimate();
  EXPECT_NE(v.swing, 0.0);
  EXPECT_NEAR(v.boom, 0.0, 1e-12);
  EXPECT_NEAR(v.arm, 0.0, 1e-12);
  EXPECT_NEAR(v.bucket, 0.0, 1e-12);
}

TEST(VelocityEstimator, IndependentPaths_JointsOnly) {
  VelocityEstimator est;
  // 只 push joints，不 push swing → swing 应为 0
  kinematics::JointState q{};
  q.boom = Deg2Rad(20.0);
  est.PushJoints(0.0, q);
  q.boom = Deg2Rad(22.0);
  est.PushJoints(0.1, q);
  q.boom = Deg2Rad(24.0);
  est.PushJoints(0.2, q);

  auto v = est.Estimate();
  EXPECT_NEAR(v.swing, 0.0, 1e-12);
  EXPECT_NE(v.boom, 0.0);
}
