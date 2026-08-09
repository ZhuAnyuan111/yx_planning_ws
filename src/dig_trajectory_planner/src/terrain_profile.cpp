/// @file terrain_profile.cpp
/// @brief 2D 地形剖面实现

#include "dig_trajectory_planner/terrain_profile.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace dig_trajectory_planner {

TerrainProfile::TerrainProfile(const std::vector<double>& x_raw,
                               const std::vector<double>& z_raw,
                               double resolution,
                               int smooth_window)
    : resolution_(resolution) {
    if (x_raw.size() != z_raw.size() || x_raw.size() < 2) {
        return;  // 无效输入，保持空地形
    }
    buildGrid(x_raw, z_raw, resolution, smooth_window);
}

void TerrainProfile::buildGrid(const std::vector<double>& x_raw,
                               const std::vector<double>& z_raw,
                               double resolution,
                               int smooth_window) {
    // 确定等间距网格范围
    x_min_ = x_raw.front();
    x_max_ = x_raw.back();

    size_t n_grid = static_cast<size_t>((x_max_ - x_min_) / resolution) + 1;
    x_grid_.resize(n_grid);
    z_grid_.resize(n_grid);

    // 构建等间距 X 网格
    for (size_t i = 0; i < n_grid; ++i) {
        x_grid_[i] = x_min_ + i * resolution;
    }

    // 从原始数据插值到等间距网格
    size_t raw_idx = 0;
    for (size_t i = 0; i < n_grid; ++i) {
        double x_query = x_grid_[i];

        // 找到 x_query 所在的原始区间
        while (raw_idx + 1 < x_raw.size() && x_raw[raw_idx + 1] < x_query) {
            ++raw_idx;
        }

        if (raw_idx + 1 >= x_raw.size()) {
            // 超出范围，使用最后一个值
            z_grid_[i] = z_raw.back();
        } else if (x_raw[raw_idx + 1] == x_raw[raw_idx]) {
            // 避免除零
            z_grid_[i] = z_raw[raw_idx];
        } else {
            // 线性插值
            double t = (x_query - x_raw[raw_idx]) / (x_raw[raw_idx + 1] - x_raw[raw_idx]);
            z_grid_[i] = z_raw[raw_idx] + t * (z_raw[raw_idx + 1] - z_raw[raw_idx]);
        }
    }

    // 滑动均值滤波平滑
    if (smooth_window >= 3) {
        smoothTerrain(z_grid_, smooth_window);
    }
}

void TerrainProfile::smoothTerrain(std::vector<double>& z_data, int window) const {
    size_t n = z_data.size();
    std::vector<double> z_smoothed(n);

    // 确保窗口为奇数
    int half_window = window / 2;

    for (size_t i = 0; i < n; ++i) {
        // 计算实际使用的窗口范围（边界处理）
        int start = static_cast<int>(i) - half_window;
        int end = static_cast<int>(i) + half_window;

        // 边界裁剪
        int actual_start = std::max(0, start);
        int actual_end = std::min(static_cast<int>(n - 1), end);

        // 计算均值
        double sum = 0.0;
        int count = 0;
        for (int j = actual_start; j <= actual_end; ++j) {
            sum += z_data[j];
            ++count;
        }
        z_smoothed[i] = sum / count;
    }

    z_data = z_smoothed;
}

bool TerrainProfile::getHeight(double x, double& z) const {
    if (!isValid() || x < x_min_ || x > x_max_) {
        return false;
    }

    // 计算网格索引
    double idx_double = (x - x_min_) / resolution_;
    size_t idx_low = static_cast<size_t>(idx_double);

    // 边界处理
    if (idx_low >= x_grid_.size() - 1) {
        z = z_grid_.back();
        return true;
    }

    // 线性插值
    double t = idx_double - idx_low;
    z = z_grid_[idx_low] + t * (z_grid_[idx_low + 1] - z_grid_[idx_low]);
    return true;
}

bool TerrainProfile::getNormalAngle(double x, double& angle) const {
    if (!isValid() || x < x_min_ || x > x_max_) {
        return false;
    }

    // 使用中心差分计算法向量
    double dx = resolution_;
    double z_left, z_right;

    // 获取左右点高度
    if (!getHeight(x - dx, z_left)) {
        z_left = z_grid_.front();
    }
    if (!getHeight(x + dx, z_right)) {
        z_right = z_grid_.back();
    }

    // 切线方向: (2*dx, z_right - z_left)
    // 法线方向: (-(z_right - z_left), 2*dx) 归一化后取角度
    double dz = z_right - z_left;
    angle = std::atan2(-dz, 2 * dx);  // 法向角，向上为正

    return true;
}

void TerrainProfile::getXRange(double& x_min, double& x_max) const {
    x_min = x_min_;
    x_max = x_max_;
}

}  // namespace dig_trajectory_planner
