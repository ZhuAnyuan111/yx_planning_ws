# reset_trajectory_planner

无人挖掘机卸载复位轨迹规划与分阶段执行节点。

卸载结束后，采用**回转先行、臂架跟随映射**的两阶段策略，将挖掘机安全复位到下一个挖掘点的目标姿态。臂架的调整进度完全由回转的剩余行程实时映射，不依赖固定步数或时间。

节点采用**清晰的三层分层架构**（工具层 → 节点类层 → main 入口层）。

---

## 目录结构

```
reset_trajectory_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/reset_trajectory_planner/
│   ├── angle_utils.h                  # 纯 C++ 角度工具接口（无 ROS 依赖）
│   └── reset_trajectory_node.h        # ResetTrajectoryNode 类声明
├── src/
│   ├── angle_utils.cpp                # 角度工具实现
│   ├── reset_trajectory_node.cpp      # 节点类实现
│   └── reset_trajectory_node_main.cpp # 极简 main 入口
├── config/
│   └── reset_trajectory.yaml          # 节点参数配置
└── launch/
    └── reset_trajectory.launch        # 启动文件
```

**构建产物：**
- `libreset_trajectory_planner_lib.so`：工具 + 节点类共享库（可复用/单测）
- `reset_trajectory_node`：可执行文件

---

## 执行策略

### 总体流程

```
收到 /Sys_SeqAction (data==2.0)
              │
              ▼
    计算方向性总回转行程（处理 360° 回绕）
              │
     ┌────────┴────────┐
     │ 行程 > 10°?      │
     │                  │
     ▼ 是               ▼ 否
┌─────────────┐   ┌─────────────┐
│ 阶段1:kSwing│   │             │
│ 回转先行    │   │  直接跳到   │
│             │──►│  阶段2      │
│ 已转≥10°    │   │             │
└─────────────┘   └──────┬──────┘
                         │
                  ┌──────▼──────┐
                  │ 阶段2:kArm  │
                  │ 臂架跟随映射│
                  │             │
                  │ 结束条件二选一:│
                  │ ①物理提前结束 │
                  │ ②回转到位/超调│
                  └──────┬──────┘
                         │
                  ┌──────▼──────┐
                  │ Position.x=0 │
                  │ 阶段结束     │
                  └─────────────┘
```

### 阶段1：回转先行（kSwing）

| 项目 | 说明 |
|------|------|
| 发布内容 | swing = 目标终点角，boom/arm/bucket = **锁定值**（进入阶段瞬间的臂架角度，非实时值） |
| 发布频率 | 10Hz |
| 退出条件 | 方向性角度差 `traveled >= swing_threshold_deg`（默认 10°） |
| 执行时长 | **自适应** — 取决于实际回转速度 |
| 超时保护 | 超过 `swing_timeout_sec`（默认 15s）未达阈值则异常中止；同时受全局超时统一保护 |
| 设计意图 | 先让回转将铲斗从卡车上方移走，避免臂架调整时与车厢碰撞 |

**为什么锁定值？** 若发布实时值，等于告诉控制器"你在哪待哪"，控制器误差为零，无法对抗重力下沉；锁定后控制器持续保持在锁定位置。

### 阶段2：臂架跟随映射（kArm）

| 项目 | 说明 |
|------|------|
| 发布内容 | swing = 目标终点角（保持），boom/arm/bucket = 根据回转剩余行程映射 |
| 发布频率 | 10Hz |
| 映射起点 | **阶段2触发瞬间**的实际关节角（消除阶段1漂移） |
| 映射公式 | `t = (已回转 - threshold) / (总行程 - threshold)`，t∈[0, 1] |
| 臂架计算 | `θ(t) = θ_start + t * (θ_goal - θ_start)` |
| 结束条件 | 物理提前结束 或 回转到位/超调 |
| 超时保护 | 全局 `total_timeout_sec`（默认 30s）统一覆盖阶段1+阶段2 |
| 执行时长 | **完全自适应** — 臂架完成时刻 ≈ 回转到位时刻 |

### 阶段结束条件（阶段2 每帧检查）

**条件 A：物理提前结束（优先）**

```
(truck_safe && bucket_height_gd < RSThreshold(5)) && RealBktPosXYZ(3) < 0
```

- `truck_safe`：当前回转角与卡车中心航向的**最短角距** > `truck_safe_margin_deg`（默认 60°），视为铲斗已脱离卡车
- `bucket_height_gd`：来自 `/bucket_terrain_delta_position` 的 `.z`（铲斗相对地面高度）
- `RSThreshold(5)`：阈值参数 `rs_threshold_5`
- `RealBktPosXYZ(3)`：来自 `/RealBktPosXYZ` 的 `.z`（齿尖离地高度） < 0

三个订阅话题任一未收到 → 该条件返回 false，退化为仅按条件 B 判定。

**条件 B：回转到位/超调**

方向性行程 `traveled >= total_swing_travel` → 立即发精确目标点。

两者任一满足即结束，发送最后一帧 `Position.x = 0` 表示阶段结束。

### 卡车中心航向计算

```
truck_center_swingAngle = rad2deg(atan2(y, x)) + 180
```

由 `/truck_center_point` 的 x/y 分量计算，与挖掘机 x北/y西 坐标系航向 β 对齐。

---

## 目标姿态输入

目标姿态由上位机预计算后，通过 `/dig_point`（`geometry_msgs/Quaternion`）直接下发：

| 字段 | 含义 |
|------|------|
| `x` | swing 目标角（度） |
| `y` | boom  目标角（度） |
| `z` | arm   目标角（度） |
| `w` | bucket目标角（度） |

**触发时机：** `/dig_point` 到达即更新目标；执行期间目标由激活瞬间的快照锁定，
中途刷新不影响正在执行的轨迹。

**失败会怎样：** 若从未收到 `/dig_point`，激活时前置校验不通过，节点不启动执行。

---

## 话题接口

### 订阅

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/joints_angle` | `geometry_msgs/Quaternion` | 实时关节角：x=boom, y=arm, z=bucket（度） |
| `/Swing_topic` | `geometry_msgs/Point` | 实时回转角：x=swing（度） |
| `/dig_point` | `geometry_msgs/Quaternion` | 目标姿态：x=swing, y=boom, z=arm, w=bucket（度） |
| `/Sys_SeqAction` | `std_msgs/Float64` | 激活信号：`data==2.0` 触发复位；其他值触发停止 |
| `/truck_center_point` | `geometry_msgs/Quaternion` | 卡车中心位置：x/y/z=坐标（用于计算 truck_safe） |
| `/bucket_terrain_delta_position` | `geometry_msgs/PointStamped` | `.z = bucket_height_gd` |
| `/RealBktPosXYZ` | `geometry_msgs/Point` | `.z =` 齿尖离地高度 |

### 发布

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| `/RefDeviceTraj_Reset` | `geometry_msgs/Pose` | 10Hz | 见下方字段定义 |

**`/RefDeviceTraj_Reset` 字段映射：**

```
Position.x    = 1.0（执行中） / 0.0（阶段结束）
Position.y    = 0（保留）
Position.z    = 0（保留）
Orientation.x = swing   （度）
Orientation.y = boom    （度）
Orientation.z = arm     （度）
Orientation.w = bucket  （度）
```

---

## 参数配置

配置文件：`config/reset_trajectory.yaml`

```yaml
# 阶段切换阈值：回转已转过多少度后开始臂架跟随映射
swing_threshold_deg: 10.0

# 阶段1 回转超时时间（秒）
swing_timeout_sec: 15.0

# 全局执行超时（从触发开始计，覆盖阶段2）
total_timeout_sec: 30.0

# truck_safe 判定阈值：与卡车中心航向的最小角距（度）
truck_safe_margin_deg: 60.0

# RSThreshold(5)：bucket_height_gd 结束阈值
rs_threshold_5: 5.0

# 输入新鲜度超时：执行期 /joints_angle 或 /Swing_topic 静默超过此时长则中止
input_timeout_sec: 1.0

# 调试日志开关：true 开启阶段周期进度、物理条件子项、防抖滤波等 [dbg] 打印
enable_debug_log: false
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `swing_threshold_deg` | double | 10.0 | 阶段1→2 切换阈值 |
| `swing_timeout_sec` | double | 15.0 | 阶段1最大等待时间 |
| `total_timeout_sec` | double | 30.0 | 全局执行超时（统一覆盖两阶段） |
| `truck_safe_margin_deg` | double | 60.0 | truck_safe 判定角距阈值 |
| `rs_threshold_5` | double | 5.0 | 铲斗相对地面高度结束阈值 |
| `input_timeout_sec` | double | 1.0 | 输入话题最大静默时长，≤0 关闭检查 |
| `enable_debug_log` | bool | false | 调试打印开关 |

---

## 编译与运行

### 编译

```bash
cd ~/yx_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg reset_trajectory_planner
```

### 运行

```bash
source devel/setup.bash
roslaunch reset_trajectory_planner reset_trajectory.launch
```

### 手动测试

```bash
# 终端1：启动节点
roslaunch reset_trajectory_planner reset_trajectory.launch

# 终端2：模拟实时关节角
rostopic pub -r 10 /joints_angle geometry_msgs/Quaternion \
  '{x: 30.0, y: -45.0, z: 10.0, w: 0.0}'
rostopic pub -r 10 /Swing_topic geometry_msgs/Point \
  '{x: 60.0, y: 0.0, z: 0.0}'

# 终端3：发布目标姿态（swing/boom/arm/bucket 目标角）
rostopic pub -r 2 /dig_point geometry_msgs/Quaternion \
  '{x: 0.0, y: 30.0, z: -45.0, w: 0.0}'

# 终端4：触发复位（data==2.0）
rostopic pub /Sys_SeqAction std_msgs/Float64 "data: 2.0"

# 观察输出
rostopic echo /RefDeviceTraj_Reset
```

---

## 边界情况处理

| 情况 | 处理方式 |
|------|----------|
| 总回转行程 ≤ threshold | 跳过阶段1，直接进入阶段2 |
| 回转到位/超调 | 立即发精确目标角 + Position.x=0，结束 |
| 物理条件先达成 | 优先于回转到位，立即结束 |
| 三个物理话题任一缺失 | 物理条件禁用，仅靠回转到位判定 |
| 阶段1 回转卡死/过慢 | 超过 `swing_timeout_sec` 异常中止（保持末次指令 + Position.x=0） |
| 阶段2 回转卡死 | 全局超时 `total_timeout_sec` 统一保底中止 |
| 执行中反馈话题断流 | 静默超 `input_timeout_sec` 立即中止，避免基于陈旧反馈下发指令 |
| 执行中挖掘点话题刷新 | 不影响执行——目标在激活瞬间已快照锁定 |
| `/dig_point` 从未到达 | 前置校验不通过，激活时拒绝启动 |
| 执行中 Sys_SeqAction 变为非 2 | 立即停止，保持末次指令并发 Position.x=0 |
| 已在执行中重复收到 data==2 | 忽略，避免定时器状态混乱 |
| 输入数据缺失 | 前置校验不通过，不启动 |

---

## 360° 回绕处理

回转角差值计算使用方向性角度差（`CalculateAngleDiff`），支持跨零边界（如 350°→10° 正确识别为 20°）：

```
CalculateAngleDiff(A, B, direction=1)  // 顺时针从 A 到 B
CalculateAngleDiff(A, B, direction=-1) // 逆时针从 A 到 B
DetermineDirection(start, goal)         // 自动选最短路径方向
```

工具函数位于 `angle_utils.h/cpp`，纯 C++ 实现，不依赖 ROS，可独立单元测试。

---

## 分层架构

```
┌──────────────────────────────────────────────────┐
│ Layer 3: reset_trajectory_node_main.cpp          │  ← 极简入口（15 行）
│   int main(...) { init + create + spin }         │
└──────────────────────────────────────────────────┘
                     ▲ 链接
┌──────────────────────────────────────────────────┐
│ Layer 2: reset_trajectory_node.h/.cpp            │  ← ROS 节点类
│   ResetTrajectoryNode 类：订阅/发布/状态机       │
└──────────────────────────────────────────────────┘
                     ▲ 依赖
┌──────────────────────────────────────────────────┐
│ Layer 1: 纯 C++ 工具层（无 ROS 依赖，可单测）    │
│   angle_utils.h/.cpp                             │
│     NormalizeAngle / CalculateAngleDiff /        │
│     DetermineDirection                           │
└──────────────────────────────────────────────────┘
```

**收益：**
- 单向依赖，无循环
- 工具层可独立单元测试（无 ROS）
- 节点类可作为库被其他 executable 复用
- main 极简，只关心 ROS 生命周期

---

## 命名规范

遵循 Google C++ Style：

| 类别 | 规范 | 示例 |
|------|------|------|
| 文件名 | snake_case | `reset_trajectory_node.cpp` |
| 类名 | PascalCase | `ResetTrajectoryNode` |
| 函数名 | PascalCase | `ExecuteArmPhase()` |
| 成员变量 | snake_case_ | `swing_threshold_deg_` |
| 常量 | kCamelCase | `kPublishRate` |
| 枚举值 | kCamelCase | `ResetPhase::kSwing` |

---

## 依赖

- ROS Noetic
- roscpp
- geometry_msgs
- std_msgs

与 `truck_dump_planner` 包零耦合，可独立编译、部署和升级。

---

## 系统集成

```
系统决策节点 ─── /Sys_SeqAction (data==2.0) ──► reset_trajectory_node
                                                        │
              ◄── /RefDeviceTraj_Reset ─────────────────┘
              (Position.x=1 执行中, Position.x=0 结束)
```

本节点遵循系统决策节点统一调度架构，通过单向信号驱动（激活→执行→输出），不维持内部循环。阶段结束状态编码在输出话题的 `Position.x` 字段（1=执行中，0=结束）。

> **注意**：正常完成与异常中止（超时 / 输入断流）当前均以 `Position.x=0` 表示，内部状态机虽已区分 `kDone` / `kFailed`，但尚未通过独立话题对外上报错误码。如上位机需区分两者，待确认反馈话题协议后补充（可参考卸载节点的 `/UT_PlanningPulse_topic` 安全状态字机制）。
