#pragma once

/// @file angle_utils.h
/// @brief 角度工具函数：归一化、方向性角度差、最短旋转方向判断。
///        纯 C++ 实现，不依赖 ROS，可独立单元测试。

namespace reset_trajectory_planner {

/// 将角度归一化到 [0, 360)
/// @param angle 输入角度（度）
/// @return 归一化后的角度 [0, 360)
double NormalizeAngle(double angle);

/// 计算方向性角度差（处理 360° 回绕）
/// @param a 起始角度（度）
/// @param b 目标角度（度）
/// @param direction 1=顺时针（角度减小方向），-1=逆时针（角度增大方向）
/// @return 从 a 到 b 沿指定方向的正值角度差 [0, 360)
double CalculateAngleDiff(double a, double b, int direction);

/// 判断从 start 到 goal 的最短旋转方向
/// @param start 起始角度（度）
/// @param goal  目标角度（度）
/// @return 1=顺时针，-1=逆时针
int DetermineDirection(double start, double goal);

}  // namespace reset_trajectory_planner
