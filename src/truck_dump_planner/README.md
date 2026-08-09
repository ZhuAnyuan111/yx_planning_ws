# truck_dump_planner

无人挖掘机装车规划 ROS 节点，支持**挖掘 → 卸载 → 复位**多阶段轨迹规划，含坐标转换、卡车车厢包络建模、RViz 可视化和状态机调度。

---

## 功能概述

| 阶段 | 说明 | 实现状态 |
|------|------|----------|
| **Dig（挖掘）** | 从当前铲斗位置规划到挖掘目标点的 4 段直线轨迹 | 基础实现 |
| **Dump（卸载）** | 回转至卡车上方 + 卸料（需接入 kinematics IK） | 占位 |
| **Reset（复位）** | 从当前铲斗位置回到安全待机位的 3 段直线轨迹 | 基础实现 |

状态机通过 `/planner_command` 话题接收指令进行切换。

---

## 目录结构

```
truck_dump_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── dump_planner.yaml          # 节点参数配置
├── launch/
│   └── dump_planner.launch        # 启动文件
├── include/truck_dump_planner/
│   ├── coords.h                   # 坐标转换（RTK方位角 ↔ 挖掘机航向）
│   ├── truck_model.h              # 卡车车厢包络建模
│   ├── dig_planner.h              # 挖掘轨迹规划
│   ├── reset_planner.h            # 复位轨迹规划
│   ├── visualization.h            # RViz MarkerArray 可视化
│   └── planner_node.h             # ROS 节点类 + 状态机
└── src/
    ├── coords.cpp
    ├── truck_model.cpp
    ├── dig_planner.cpp
    ├── reset_planner.cpp
    ├── visualization.cpp
    ├── planner_node.cpp
    └── dump_planner_node.cpp      # main() 入口（12行）
```

---

## 命名规范

本项目遵循 **Google C++ Style Guide** 的命名约定：

| 类别 | 风格 | 示例 |
|------|------|------|
| 文件名 | `snake_case` | `planner_node.cpp`、`truck_model.h` |
| 命名空间 | `snake_case` | `truck_dump_planner` |
| 类/结构体 | `PascalCase` | `PlannerNode`、`TruckEnvelope`、`Vec3` |
| 函数（成员/全局） | `PascalCase` | `LoadParams()`、`BuildTruckEnvelope()` |
| 成员变量 | `snake_case_`（尾部下划线） | `swing_deg_`、`last_env_` |
| 结构体字段 | `snake_case`（无下划线） | `box_length`、`waypoints` |
| 常量/枚举值 | `kCamelCase` | `kDeg2Rad`、`kIdle`、`kDig` |
| 宏/Include Guard | `ALL_CAPS` | `TRUCK_DUMP_PLANNER_COORDS_H` |
| ROS 话题名 | `snake_case` | `truck_pose`、`planner_command` |
| ROS 节点名 | `snake_case` | `dump_planner_node` |

---

## 坐标系约定

```
挖掘机基座系 (frame: "base")
  x 轴 → 地理正北
  y 轴 → 地理正西
  z 轴 → 天（右手系）

  swing = 180° 时，大臂朝向 -x（即地理正南）= 车头朝北
```

### RTK 方位角 → 挖掘机航向转换

```
β_excavator = (180° + α_rtk) mod 360°
```

其中 `α_rtk` 为 RTK 方位角（北 0°，顺时针增大）。

---

## 架构设计

```
系统决策节点                        本节点 (dump_planner_node)
┌────────────┐   planner_activate    ┌────────────────┐
│  Decision   │ ─── "dig"/"dump"/"reset" ───▶ │  PlannerNode   │
│   Node      │                       │                │
│            │ ◀─── planner_feedback ──── │  (状态机调度)    │
└────────────┘   "dig_done"/"dig_failed"  └────────────────┘
```

**工作流程：**
1. 系统决策节点发布激活信号（如 `"dig"`）到 `planner_activate`
2. 本节点执行对应规划
3. 规划完成后发布反馈（如 `"dig_done"` 或 `"dig_failed"`）到 `planner_feedback`
4. 系统决策节点根据反馈决定下一步

---

## 话题接口

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `planner_activate` | `std_msgs/String` | 系统决策节点发送的阶段激活信号：`dig` / `dump` / `reset` / `stop` |
| `truck_pose` | `geometry_msgs/Quaternion` | x=纬度, y=经度, z=高程, **w=RTK方位角(度,北0顺时针)** |
| `/joints_angle` | `geometry_msgs/Quaternion` | x=boom(度), y=arm(度), z=bucket(度) |
| `/Swing_topic` | `geometry_msgs/Point` | x=swing(度) |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `planner_feedback` | `std_msgs/String` | 规划完成反馈给系统决策节点 |
| `truck_envelope_markers` | `visualization_msgs/MarkerArray` | 卡车车厢包络可视化 |
| `trajectory_markers` | `visualization_msgs/MarkerArray` | 当前规划轨迹 LINE_STRIP |

### 反馈信号定义

| 反馈内容 | 含义 |
|----------|------|
| `dig_done` | 挖掘规划完成 |
| `dig_failed` | 挖掘规划失败 |
| `dump_done` | 卸载规划完成 |
| `dump_failed` | 卸载规划失败 |
| `reset_done` | 复位规划完成 |
| `reset_failed` | 复位规划失败 |
| `idle` | 已停止，回到空闲 |

---

## 状态机

```
  系统决策节点发送激活信号          规划完成后发布反馈
        │                               │
        ▼                               ▼
    ┌──────┐   activate    ┌──────┐   feedback
    │ IDLE │ ─────────▶ │  DIG │ ───────▶ dig_done
    └──────┘              └──────┘
    ┌──────┐   activate    ┌──────┐   feedback
    │ IDLE │ ─────────▶ │ DUMP │ ───────▶ dump_done
    └──────┘              └──────┘
    ┌──────┐   activate    ┌───────┐   feedback
    │ IDLE │ ─────────▶ │ RESET│ ───────▶ reset_done
    └──────┘              └───────┘

  任何状态收到 "stop" → 回到 IDLE 并反馈 "idle"
```

- 每个阶段由系统决策节点独立激活
- 规划完成后自动发布 feedback，决策节点据此触发下一阶段
- 规划失败时发布 `xxx_failed` 并自动回到 IDLE
- 定时器 10Hz 发布轨迹可视化

---

## 参数配置

参数通过 `config/dump_planner.yaml` 加载至私有命名空间 `~`：

```yaml
# 卡车车厢几何（米）
truck:
  box_length: 5.0
  box_width: 2.6
  box_height: 1.8

# 挖掘机回转中心（参考点）
excavator:
  x: 0.0
  y: 0.0

# 坐标系
frame_id: base
axis_convention: north_west

# 挖掘轨迹参数
dig:
  point_x: 3.0
  point_y: 0.0
  point_z: -0.5
  approach_height: 1.0
  dig_depth: 0.5

# 复位轨迹参数
reset:
  home_x: 2.0
  home_y: 0.0
  home_z: 1.5
  safe_height: 3.0
```

---

## 编译 & 运行

```bash
# 编译
cd ~/yx_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make

# 运行
source devel/setup.bash
roslaunch truck_dump_planner dump_planner.launch
```

---

## 模块说明

### coords（坐标转换）

- `Wrap360()`：角度归一化到 [0, 360)
- `RtkBearingToExcavatorHeading()`：RTK 方位角 → 挖掘机系航向
- `HeadingToUnitVector()`：航向角 → 单位方向向量
- `RightUnitVector()`：航向右侧垂直方向

### truck_model（卡车建模）

- `BuildTruckEnvelope()`：根据位置/航向/几何参数构建车厢四角包络
- `PointInPolygon()`：判断点是否在多边形内部

### dig_planner（挖掘轨迹）

4 段直线 waypoints：
1. 当前位置 → 接近点（目标正上方）
2. 接近点 → 入土点
3. 入土点 → 铲入点（沿 -x 方向）
4. 铲入点 → 提升点

### reset_planner（复位轨迹）

3 段直线 waypoints：
1. 当前位置 → 安全高度（垂直提升）
2. 安全高度 → home 正上方（水平平移）
3. home 正上方 → home（垂直下降）

### visualization（可视化）

- `BuildEnvelopeMarkers()`：构建车厢包络 LINE_STRIP + 候选卸载点 SPHERE
- `BuildDumpPointMarkers()`：构建卸载点标记

### planner_node（节点核心）

- 状态机调度（`PlannerState` 枚举 + `TransitionTo()`）
- 话题订阅/发布
- 参数加载
- 定时器可视化发布

---

## TODO

- [ ] 接入 kinematics 运动学求解器（ROS1 改造后）
- [ ] `GetBucketPosition()` 接入 FK 正解替换占位值
- [ ] Dump 状态实现（回转角规划 + IK 逆解可达性判断）
- [ ] 卸载点智能选择（基于包络可达区域）
- [ ] 轨迹平滑（目前为直线段拼接）

---

## 依赖

- ROS Noetic
- roscpp
- geometry_msgs
- visualization_msgs
- std_msgs
