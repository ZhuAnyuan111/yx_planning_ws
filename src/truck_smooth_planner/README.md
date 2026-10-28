# truck_smooth_planner

无人挖掘机**车厢抹平**轨迹规划与分阶段执行节点。

装满车后，用铲斗沿车厢上表面从**车尾向车头**直线推抹，刮平卡车框内高出的部分。节点根据卡车停靠位姿计算抹平范围的头尾两点，对其做逆运动学（IK）取出 `swing` / `arm` 分量作为关节目标，采用**两阶段限速逼近**策略执行；`boom` / `bucket` 每帧填充当前实测角，不主动命令。

节点采用**清晰的三层分层架构**（工具层 → 节点类层 → main 入口层）。

---

## 目录结构

```
truck_smooth_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/truck_smooth_planner/
│   ├── angle_utils.h                  # 纯 C++ 角度工具接口（无 ROS 依赖）
│   └── truck_smooth_node.h            # TruckSmoothNode 类声明
├── src/
│   ├── angle_utils.cpp                # 角度工具实现
│   ├── truck_smooth_node.cpp          # 节点类实现
│   └── truck_smooth_node_main.cpp     # 极简 main 入口
├── config/
│   └── truck_smooth.yaml              # 节点参数配置
└── launch/
    └── truck_smooth.launch            # 启动文件
```

**构建产物：**
- `libtruck_smooth_planner_lib.so`：工具 + 节点类共享库（可复用/单测）
- `truck_smooth_node`：可执行文件

---

## 执行策略

### 总体流程

```
收到 /Sys_SeqAction (data==6.0)
              │
              ▼
   前置校验（joints/swing/卡车位姿是否已到达）
              │
              ▼
   由卡车位姿算车尾起点/车头终点，做 IK 取 swing/arm
   （IK 失败则降级：端点收缩 → 最大扫掠 → 原地保持，恒不拒绝）
   → (swing_start, arm_start)   (swing_end, arm_end)
              │
              ▼
   命令值初始化为当前实时位置（swing_deg_, arm_deg_）
              │
   ┌──────────▼───────────┐
   │ 阶段1:kPhase1        │
   │ 限速逼近车尾起点     │
   │                      │
   │ 实时反馈进入容差且   │
   │ 连续 confirm 帧 ─────┼──►┌─────────────────────┐
   └──────────────────────┘   │ 阶段2:kPhase2       │
                              │ 限速推抹至车头终点  │
                              │                     │
                              │ arm 到位(可选校验   │
                              │ swing) 连续 confirm │
                              │ 帧 → 抹平结束       │
                              └──────────┬──────────┘
                                         │
                              ┌──────────▼──────────┐
                              │ Position.x=0        │
                              │ FlagSmoothFinish=   │
                              │ true                │
                              └─────────────────────┘
```

### 抹平范围几何计算

由 `/truck_center_unloadpoint` 得到卡车中心与航向，沿车厢长方向内缩取头尾两点：

```
base 系卡车航向 β = (180 − α) mod 360      # α = RTK 方位角 (msg.w)
方向向量      dir = (cos β, sin β)
半长(内缩后)  half = truck_box_length / 2 − box_inset

车尾起点  tail = (center_x − half·dir_x,  center_y − half·dir_y)   # Phase1 目标
车头终点  head = (center_x + half·dir_x,  center_y + half·dir_y)   # Phase2 目标

目标高度  z = (center_z − truck_height_offset) + smooth_height_bias
            └─ 车厢上表面高度 z_top ─┘   └─ 刮削裕量 ─┘
```

> `center_z` 为卡车中心 RTK 高度；`box_inset` 为头尾内缩裕量，防止铲斗碰厢壁；
> `smooth_height_bias` 为刮削高度裕量（略高于框顶只削尖，默认 0）。

### IK 求解（取 swing / arm 分量）

对 `tail` / `head` 两点分别调用运动学库：

```cpp
kinematics::SwingBaseIkRequest req;
req.target_pose = {x, y, z, alpha=0.0};
req.bucket_angle_deg = bucket_angle_deg_;                 // 固定铲斗关节角
if (use_pivot_as_target_)
    req.bucket_tooth_length_override = 0.0;               // 目标点=斗杆-铲斗铰接点
auto ik = solver_->swing_center_inverse_by_bucket_angle(req);
// 取 swing / arm 分量（度）作为关节目标；boom / bucket 不采用
```

- 起终点 `swing` 归一化到 `[0, 360)`，`arm` 直接使用 IK 解。

### IK 失败降级（尽力抹平，不可达则如实上报失败）

抹平区间用「沿 `dir` 相对卡车中心的有符号偏移 `s`」参数化：车尾 `s=-half`、车头 `s=+half`，点 `P(s) = center + s·dir`（高度恒为 `z`）。起终点 IK 失败时，优先在挖机能力范围内裁剪出可达子区间照常抹平（L1/L2）；仅当整个 `z` 平面都不可达、物理上无法作业时，才拒绝执行并如实上报未完成（L3）。降级分三级：

```
Level 0/1  端点距离收缩重试（主降级）
   tail 从 -half 朝中心(0)按 shrink_step_m 收缩，取首个可达 s → s_tail
   head 从 +half 朝中心(0)按 shrink_step_m 收缩，取首个可达 s → s_head
   若 (s_head - s_tail) >= min_smooth_span_m → 采用（区间为原区间子集）
        └ 全部端点可达=满量程(INFO)；发生收缩=降级(WARN)
              │ 失败（某侧端点→中心整体不可达）
              ▼
Level 2    最大能力扫掠（兜底）
   在 [-half, half] 以 shrink_step_m 密采样逐点 IK，
   取可达点中 s 最小/最大者作起终点；跨度 >= min_smooth_span_m → 采用(WARN)
              │ 失败（z 平面几乎无可达点）
              ▼
Level 3    拒绝执行 + 上报未完成（最终保底）
   ROS_ERROR 告知 z 平面整体超出可达域；绝不进入两阶段（否则起终点=当前
   位姿会瞬间到位而谎报成功），置 FlagSmoothFinish=false + kFailed，
   由决策层超时兜底，等待现场调整卡车位置/高度
```

| 级别 | 触发条件 | 动作 | 日志 |
|------|----------|------|------|
| L0 满量程 | 头尾点直接可解 | 用原始 `[-half, +half]` | `INFO` |
| L1 端点收缩 | 头/尾点不可解 | 端点朝中心收缩到首个可达点 | `WARN` |
| L2 最大扫掠 | 某侧端点→中心全不可达 | 密采样取最远可达点对 | `WARN` |
| L3 拒绝执行 | z 平面整体不可达 | 不进两阶段，`finish=false` + `kFailed` | `ERROR` |

> **设计依据**：抹平本质是铲斗在 `z` 平面沿 `dir` 的直线扫掠；IK 失败仅说明原始头尾点超出可达域，应将抹平区间**裁剪到可达域内**而非放弃（L1/L2）。但 L1/L2 均失败时意味着**物理上根本无法作业**，此时严禁将“干不了”谎报为“干完了”（L3 置 `finish=false`）。
>
> **高度参考职责边界**：实际刮削平面高度由**其他节点**提供并由控制层齿尖高度闭环维持；本节点的 `z`（及 `truck_height_offset`/`smooth_height_bias`）**仅作为 IK 解 swing/arm 的名义输入**，不决定实际刮削面高度。boom/bucket 锁定实测值是**有意设计**：boom 由控制层基于齿尖高度误差闭环调节，bucket 保持不动（参见“接触式作业规划与控制解耦策略”）。
>
> **探测期间静默**：`SolveOffsetIk` 重试调用 `SolvePointIk(verbose=false)`，避免逐点 IK 失败刷屏；仅在最终判定处打印一次 WARN/ERROR。

### 阶段1：进入车尾起点（kPhase1）

| 项目 | 说明 |
|------|------|
| 命令内容 | swing/arm = 限速逼近 `(swing_start, arm_start)`；boom/bucket = **当前实测角** |
| 发布频率 | `publish_rate`（默认 10Hz） |
| 限速方式 | 每帧朝目标按 `vel·dt` 推进（swing 走最短角路径，arm 线性） |
| 到位判据 | **实时反馈** swing 与 arm 同时进入容差，且连续 `confirm_frames` 帧成立 |
| 超时保护 | 超过 `phase1_timeout_sec`（默认 8s）异常中止；同时受全局超时保护 |
| 设计意图 | 先把铲斗平稳送入车厢内后方起点，为推抹做好姿态准备 |

### 阶段2：推抹至车头终点（kPhase2）

| 项目 | 说明 |
|------|------|
| 命令内容 | swing/arm = 限速逼近 `(swing_end, arm_end)`；boom/bucket = **当前实测角** |
| 发布频率 | `publish_rate`（默认 10Hz） |
| 到位判据 | **arm 为主判据**进入容差；`phase2_check_swing=true` 时同时校验 swing；连续 `confirm_frames` 帧成立 |
| 完成动作 | 发末帧 `Position.x=0` + `/FlagSmoothFinish=true`，停定时器，状态置 `kDone` |
| 超时保护 | 超过 `phase2_timeout_sec`（默认 10s）异常中止；同时受全局超时保护 |

### 限速逼近（斜坡）

对阶跃目标增加速率限制器，防止底层跟踪冲击过大：

- **swing（wrap-aware）**：用有符号最短角差 `SignedShortestDiffDeg` 判断方向，处理 0°/360° 跨界，命令值始终归一化到 `[0, 360)`。
- **arm（线性）**：`StepTowards` 直接朝目标推进不超过 `arm_vel_dps·dt`。
- `dt` 由相邻定时器时刻差计算，异常（≤0 或 >0.5s）回退为 `1/publish_rate`。

> **注意**：到位判定基于**实时反馈角**（`/joints_angle`、`/heading2swing_topic`），而非命令值，确保物理真正到位后才推进阶段。

---

## 话题接口

### 订阅

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/joints_angle` | `geometry_msgs/Quaternion` | 实时关节角：x=boom, y=arm, z=bucket（度） |
| `/heading2swing_topic` | `std_msgs/Float32` | 实时回转角：data=swing（度） |
| `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | 卡车中心：x/y/z=坐标（z=RTK高），w=RTK方位角α（度） |
| `/Sys_SeqAction` | `std_msgs/Float64` | 激活信号：`data==6.0` 触发抹平，其余值停止/复位 |

### 发布

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| `/RefDeviceTraj_Smooth` | `geometry_msgs/Pose` | 10Hz | 参考轨迹帧，见下方字段定义 |
| `/FlagSmoothFinish` | `std_msgs/Bool` | 事件（latched） | `true`=抹平完成；激活/停止/中止时复位为 `false` |

**`/RefDeviceTraj_Smooth` 字段映射：**

```
Position.x    = 1.0（执行中） / 0.0（结束/空闲）
Position.y    = 0（保留）
Position.z    = 0（保留）
Orientation.x = swing   （度，命令值）
Orientation.y = boom    （度，当前实测角）
Orientation.z = arm     （度，命令值）
Orientation.w = bucket  （度，当前实测角）
```

---

## 参数配置

配置文件：`config/truck_smooth.yaml`

### kinematics（运动学机构参数）

由 `KinematicsSolver` 从 `~/kinematics/...` 加载：连杆长度、动臂销轴偏置、各关节限位（deg）。

### smooth（抹平范围/几何）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `truck_height_offset` | double | 0.5 | 车厢上表面高度偏移：`z_top = center_z − 偏移` |
| `smooth_height_bias` | double | 0.0 | IK 目标高度裕量：`z = z_top + bias` |
| `truck_box_length` | double | 5.0 | 卡车厢长 L (m) |
| `box_inset` | double | 0.3 | 头尾内缩裕量 (m)，防碰厢壁 |
| `bucket_angle_deg` | double | 25.0 | IK 固定铲斗关节角 (deg) |
| `use_pivot_as_target` | bool | true | IK 目标点语义：true=斗杆-铲斗铰接点，false=齿尖 |
| `shrink_step_m` | double | 0.1 | IK 降级：端点收缩/路径密采样步长 (m) |
| `min_smooth_span_m` | double | 0.5 | IK 降级：最小有效抹平跨度 (m)，低于此继续降级 |

### 运动与判定

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `publish_rate` | double | 10.0 | 逐帧发布频率 (Hz) |
| `swing_vel_dps` | double | 15.0 | swing 命令限速 (deg/s) |
| `arm_vel_dps` | double | 15.0 | arm 命令限速 (deg/s) |
| `swing_tolerance_deg` | double | 5.0 | swing 到位容差 (deg) |
| `arm_tolerance_deg` | double | 5.0 | arm 到位容差 (deg) |
| `confirm_frames` | int | 3 | 到位需连续确认帧数（防液压振荡误判） |
| `phase2_check_swing` | bool | true | Phase2 是否同时校验 swing 到位 |

### 超时与调试

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `phase1_timeout_sec` | double | 8.0 | 阶段1（进起点）超时 |
| `phase2_timeout_sec` | double | 10.0 | 阶段2（推抹至终点）超时 |
| `total_timeout_sec` | double | 13.0 | 全局执行超时（应 < 决策层 level_truck 超时 15s） |
| `input_timeout_sec` | double | 1.0 | 输入话题最大静默时长，≤0 关闭检查 |
| `enable_debug_log` | bool | false | 开启 `[dbg]` 详细调试日志：激活几何快照、IK 收缩/扫掠探测统计、P1/P2 心跳（fb/tgt/cmd/err/dt/elapsed，0.5s 节流）与到位计数跳变事件。**上生产置 false**，仅保留状态转换类 INFO；非节流的等待类告警（输入未就绪）统一 2s 节流防刷屏 |

---

## 编译与运行

### 编译

```bash
cd ~/yx_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg truck_smooth_planner
```

### 运行

```bash
source devel/setup.bash
roslaunch truck_smooth_planner truck_smooth.launch
```

### 手动测试

```bash
# 终端1：启动节点
roslaunch truck_smooth_planner truck_smooth.launch

# 终端2：模拟实时关节角反馈
rostopic pub -r 10 /joints_angle geometry_msgs/Quaternion \
  '{x: 30.0, y: -90.0, z: 10.0, w: 0.0}'
rostopic pub -r 10 /heading2swing_topic std_msgs/Float32 "data: 60.0"

# 终端3：模拟卡车位姿（x/y/z=中心, w=RTK方位角α）
rostopic pub -r 2 /truck_center_unloadpoint geometry_msgs/Quaternion \
  '{x: 8.0, y: 2.0, z: 3.0, w: 90.0}'

# 终端4：触发抹平（seq_action==6.0）
rostopic pub /Sys_SeqAction std_msgs/Float64 "data: 6.0"

# 观察输出
rostopic echo /RefDeviceTraj_Smooth
rostopic echo /FlagSmoothFinish
```

---

## 边界情况处理

| 情况 | 处理方式 |
|------|----------|
| joints/swing/卡车位姿 任一未到达 | 前置校验不通过，激活被拒绝，停留空闲 |
| 头/尾点 IK 失败（超出可达域） | **不拒绝**：端点朝中心收缩重试，取可达子区间（WARN） |
| 某侧端点→中心整体不可达 | 降级为最大能力扫掠：密采样取最远可达点对（WARN） |
| z 平面整体不可达（卡车太远/太高） | **拒绝执行**：不进两阶段，置 `FlagSmoothFinish=false` + `kFailed`，`ROS_ERROR` 提示现场复核车位，由决策层超时兜底 |
| 阶段1 进起点卡死/过慢 | 超 `phase1_timeout_sec` 异常中止（保持末指令 + Position.x=0 + Finish=false） |
| 阶段2 推抹卡死 | 超 `phase2_timeout_sec` 异常中止 |
| 任一阶段卡死 | 全局 `total_timeout_sec` 统一保底中止 |
| 执行中反馈话题断流 | 静默超 `input_timeout_sec` 立即中止，避免基于陈旧反馈下发指令 |
| 到位瞬间液压振荡 | 需连续 `confirm_frames` 帧进入容差才判定到位，防误判 |
| 执行中 `Sys_SeqAction` 变为非 6.0 | 立即停止，保持末指令并发 Position.x=0、Finish=false，回空闲 |
| 完成后 `Sys_SeqAction` 变为非 6.0 | `FlagSmoothFinish` 复位为 false，状态回空闲 |
| 已在执行中重复收到 6.0 | 忽略，避免定时器状态混乱 |

---

## 360° 回绕处理

回转命令与到位判定均使用角度工具处理跨零边界（如 350°→10° 识别为 20°）：

```
NormalizeAngle(a)                 // 归一化到 [0, 360)
SignedShortestDiffDeg(a, b)       // 有符号最短角差 ∈ (-180, 180]
ShortestAngularDistanceDeg(a, b)  // 无符号最短角距 ∈ [0, 180]
```

工具函数位于 `angle_utils.h/cpp`，纯 C++ 实现，不依赖 ROS，可独立单元测试。

---

## 分层架构

```
┌──────────────────────────────────────────────────┐
│ Layer 3: truck_smooth_node_main.cpp              │  ← 极简入口
│   int main(...) { init + create + spin }         │
└──────────────────────────────────────────────────┘
                     ▲ 链接
┌──────────────────────────────────────────────────┐
│ Layer 2: truck_smooth_node.h/.cpp                │  ← ROS 节点类
│   TruckSmoothNode 类：订阅/发布/几何/IK/状态机   │
└──────────────────────────────────────────────────┘
                     ▲ 依赖
┌──────────────────────────────────────────────────┐
│ Layer 1: 纯 C++ 工具层（无 ROS 依赖，可单测）    │
│   angle_utils.h/.cpp + kinematics 运动学库       │
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
| 文件名 | snake_case | `truck_smooth_node.cpp` |
| 类名 | PascalCase | `TruckSmoothNode` |
| 函数名 | PascalCase | `ExecutePhase1()` |
| 成员变量 | snake_case_ | `swing_start_deg_` |
| 常量 | kCamelCase | `kActivateValue` |
| 枚举值 | kCamelCase | `SmoothPhase::kPhase1` |

---

## 依赖

- ROS Noetic
- roscpp
- geometry_msgs
- std_msgs
- kinematics（运动学求解库）

与其他规划包零耦合，可独立编译、部署和升级。

---

## 系统集成

```
系统决策节点 ─── /Sys_SeqAction (data==6.0) ──► truck_smooth_node
（DiggingCycleFsm::LEVEL_TRUCK）                     │
              ◄── /RefDeviceTraj_Smooth ────────────┤
              (Position.x=1 执行中, =0 结束)          │
              ◄── /FlagSmoothFinish (Bool) ──────────┘
              (true = 抹平完成)
```

本节点遵循系统决策节点统一调度架构，通过单向信号驱动（激活→执行→输出），不维持内部循环：

- 决策层 `DiggingCycleFsm` 的 `LEVEL_TRUCK` 状态发布 `seq_action=6.0` 并等待抹平完成；
- 本节点激活后执行两阶段抹平，完成时回发 `/FlagSmoothFinish=true`，决策层据此推进状态机；
- 全局超时（默认 13s）严格小于决策层 `level_truck` 超时（15s），确保异常时先于上层超时安全退出。
