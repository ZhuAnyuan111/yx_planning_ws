#pragma once

/// @file terrain_profile.h
/// @brief 2D 地形剖面表示与查询工具（纯 C++，无 ROS 依赖）。
///
/// 功能：
/// - 接收原始地形点序列，构建等间距查找表
/// - 滑动均值滤波平滑地形
/// - O(1) 地形高度查询（线性插值）
/// - 地形法向量计算
///
/// 坐标系约定：X 为水平方向（远离挖掘机为正），Z 为垂直方向（向上为正）。

#include <vector>

namespace dig_trajectory_planner {

/// 2D 地形剖面类
class TerrainProfile {
public:
    /// 构造函数
    /// @param x_raw 原始地形 X 坐标（必须单调递增）
    /// @param z_raw 原始地形 Z 坐标（对应高度）
    /// @param resolution 等间距网格分辨率 (m)，默认 0.05m
    /// @param smooth_window 滑动均值滤波窗口大小（奇数），默认 7
    TerrainProfile(const std::vector<double>& x_raw,
                   const std::vector<double>& z_raw,
                   double resolution = 0.05,
                   int smooth_window = 7);

    /// 默认构造函数（空地形）
    TerrainProfile() = default;

    /// 查询指定 X 位置的地形高度（线性插值）
    /// @param x 查询 X 坐标
    /// @param[out] z 地形高度
    /// @return 查询是否在有效范围内
    bool getHeight(double x, double& z) const;

    /// 查询指定 X 位置的地形法向角
    /// @param x 查询 X 坐标
    /// @param[out] angle 法向角 (rad)，相对于 X 轴逆时针为正
    /// @return 查询是否在有效范围内
    bool getNormalAngle(double x, double& angle) const;

    /// 获取地形 X 范围
    void getXRange(double& x_min, double& x_max) const;

    /// 获取分辨率
    double getResolution() const { return resolution_; }

    /// 检查地形是否有效
    bool isValid() const { return !x_grid_.empty(); }

private:
    /// 滑动均值滤波
    void smoothTerrain(std::vector<double>& z_data, int window) const;

    /// 从原始数据构建等间距网格
    void buildGrid(const std::vector<double>& x_raw,
                   const std::vector<double>& z_raw,
                   double resolution,
                   int smooth_window);

    std::vector<double> x_grid_;  ///< 等间距 X 坐标
    std::vector<double> z_grid_;  ///< 对应地形高度
    double resolution_ = 0.05;    ///< 网格分辨率 (m)
    double x_min_ = 0.0;          ///< X 最小值
    double x_max_ = 0.0;          ///< X 最大值
};

}  // namespace dig_trajectory_planner
