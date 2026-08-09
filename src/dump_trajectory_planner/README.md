# dump_trajectory_planner

无人挖机 **卸载阶段关节空间轨迹规划节点**（在线规划版）。

## 功能概述

- 由 `/Sys_SeqAction == 4.0` 激活；
- 接收卡车中心及航向 `/truck_center_unloadpoint`（`geometry_msgs/Quaternion`，`x/y`=卡车中心 base 系 m，`w`=RTK 方位角 α deg）→ base 系 β=(180+α) mod 360 → rad；卸载点取卡车中心 x/y，z=0；
- 订阅实时关节角作为轨迹起点（swing 经 `/heading2swing_topic`，boom/arm/bucket 经 `/joints_angle`）；swing 同时实时转发为 `/Swing_topic` 供下游使用；
- 由实时关节角经运动学 FK 计算铲斗齿尖坐标并发布 `/RealBktPosXYZ`，用于卸载过程中的高度安全门控；
- 通过 [kinematics](../kinematics/README.md) 的 `swing_center_inverse_by_bucket_attitude` 做**真 IK 判定**，同时解算卸载点关节角；
- 在**关节空间**生成 6 关键航路点（照 MATLAB 方案），其中 WP4 入厢点由**回转圆与卡车厢矩形交点策略**确定；
- 采用**基于航路点的在线规划**：每段基于实际关节角在线 Hermite 插值，反馈驱动段推进；
- **铲斗高度门控**：回转至中途（WP3）完成后，确认铲斗实际高度超过卡车上表面才允许跨越卡车；等待超时未达标时自动追加一小段 boom 提升重试，次数耗尽则终止卸载；
- **航路点限位校验**：激活时对 WP1-WP6 逐个校验 boom/arm/bucket 是否在关节限位内；小偏差 clamp 容错，修正量超过 `waypoint_clamp_abort_deg` 时判定上游航路点不可信并终止激活；
- 以 10 Hz 流式逐帧发布 `/RefDeviceTraj_Dump`（照搬 reset_trajectory 协议）；
- 发布执行完成标志 `/Sys_RUn_FlagUnloadExcuteFinish`（latched）：执行中为 0，结束或空闲为 1。
- **RViz 联调可视化**：卡车 3D 线框 + 挖掘机臂架线段 + WP1-6 球体/标签 + WP4 回转圆 + 当前段轨迹折线 + 状态机调试文本，受 `visualization/enable` 总开关控制，全部 marker 位于 `base` 系。

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
|:---:|:---|:---|:---|
| Sub | `/Sys_SeqAction` | `std_msgs/Float64` | `data == 4.0` 触发卸载 |
| Sub | `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | `x/y` = 卡车中心（base 系，m）；`w` = RTK 方位角 α（deg，北0顺时针） |
| Sub | `/joints_angle` | `geometry_msgs/Quaternion` | `x/y/z` = boom/arm/bucket 关节角（deg） |
| Sub | `/heading2swing_topic` | `std_msgs/Float32` | `data` = swing 关节角（deg） |
| Pub | `/RefDeviceTraj_Dump` | `geometry_msgs/Pose` | `Orientation.x/y/z/w` = swing/boom/arm/bucket（deg）；`Position.x` 1=执行中 / 0=阶段结束 |
| Pub | `/Swing_topic` | `geometry_msgs/Point` | `x` = swing（deg），由 `/heading2swing_topic` 实时转发（保持下游协议兼容） |
| Pub | `/RealBktPosXYZ` | `geometry_msgs/Point` | 铲斗齿尖坐标（base 系，m），由实时关节角经运动学 FK 计算 |
| Pub | `/Sys_RUn_FlagUnloadExcuteFinish` | `std_msgs/Float64` | 0=执行中 / 1=结束或空闲（latched） |
| Pub | `/UPFeasible` | `std_msgs/Bool` | 卸载点可达性校验结果，每次激活时随 CheckReachable 发布（latched） |
| Pub | `/loadTraj_maxBoom` | `std_msgs/Float32` | 本次航路点动臂最大角度（deg），限位校验定稿后发布（latched） |
| Pub | `/loadTraj_maxZ` | `std_msgs/Float32` | 本次航路点铲斗齿尖最高 z（m），各航路点 FK 取最大（latched） |
| Pub | `/UT_PlanningPulse_topic` | `std_msgs/UInt32` | 功能安全状态字，每次激活校验后发布一次（位段结构见下） |
| Pub | `dump_viz/truck` | `visualization_msgs/MarkerArray` | 卡车车厢 3D 线框 + 航向箭头 + 门控高度矩形 + 卸载点（激活时发布一次） |
| Pub | `dump_viz/excavator` | `visualization_msgs/MarkerArray` | 臂架线段 + 铰点球 + 齿尖球（随指令帧 10Hz 发布） |
| Pub | `dump_viz/plan` | `visualization_msgs/MarkerArray` | WP1-6 球体/标签 + 航路点连线 + WP4 回转圆 + 入厢点（激活时发布一次） |
| Pub | `dump_viz/segment` | `visualization_msgs/MarkerArray` | 当前段局部轨迹折线（每段在线重规划时更新） |
| Pub | `dump_viz/status` | `visualization_msgs/Marker` | 状态机调试文本（10Hz 刷新，显示 phase/seg/boost/bkt_z） |

`/UT_PlanningPulse_topic` 位拼接结构：

| 位段 | 字段 | 说明 |
|:---:|:---|:---|
| `[31:16]` | `heartbeat_cnt` | 心跳计数，每次发布 +1，>255 回绕 0（跨激活持续累加） |
| `[15:8]` | `module_id` | 轨迹规划模块 ID = 2 |
| `[7:0]` | `error_code` | 0=正常；1=卡车位置无法卸载（卸载点不可达）；2=轨迹规划失败（WP4 可达性失败或限位校验超阈值） |

语义映射：`truck_ready` = CheckReachable 结果（与 `/UPFeasible` 同源）；`error_flag` = 航路点生成成功（当前唯一失败源为 WP4 IK 可达性，限位校验失败归入 error_code=2）。输入不完整时激活被拒绝且不发布 pulse（与 `/UPFeasible` 行为一致）。

## 关键航路点方案（关节空间，照 MATLAB）

6 个关节航路点在关节角空间生成，仅 WP4 借助 IK 从笛卡尔求解：

| 点 | 名称 | 主要动作 | 计算 |
|:---:|:---|:---|:---|
| WP1 | 起点 | 挖掘终止时的关节角 | 直接取实时关节角，铲斗按姿态角上限修正 |
| WP2 | 动臂提升 | 仅 `boom` 提升 | `boom2` 由 `\|swing3-swing1\|` 在 `[boom4−max_lift, boom4]` 内插值；`swing/arm/bucket` 保持起点相对姿态 |
| WP3 | 中间过渡 | `swing` 走一半 | `swing3` = sw1→sw4 沿**最短路径**的弧中点（swing 链生成时先展开，处理 0°/360° 跨界；弧长过小时改为 `swing4 − dir·wp3_swing_step`）；`boom3/arm3` 按 `p3_boom_cof/p3_arm_cof` 混合 |
| WP4 | 厢上过渡点 | 到达卸载点上方 | `xy = ComputeMiddleUpXY()`（回转圆与厢矩形交点策略，见下），铰接点高度 `z = truck_top_height + wp4_height_bias`，铲斗长=0 + 铲斗关节角 IK 求解，bucket 由姿态角约束计算 |
| WP5 | 除铲斗到位 | `swing/boom/arm` 到位 | `swing5 = unload_swing`；`boom5/arm5` 按 `p5_boom_cof/p5_arm_cof` 混合到卸载点；铲斗按姿态限制且不倒退 |
| WP6 | 终点 | 铲斗翻转卸料 | `= unload_joint`（IK 解算结果） |

段时间由主导关节速度决定：

```
t1 = 0
t2 = |boom2-boom1| / wp2_vel_boom
t3 = t2 + max(|swing3-swing2|/wp3_vel_swing, |boom3-boom2|/wp2_vel_boom)
     其中 wp3_vel_swing 由 |swing3-swing2| 在 [p3_min_swing, p3_max_swing] 之间线性插值决定
t4 = t3 + max(|swing4-swing3|/wp4_vel_swing, |arm4-arm3|/wp4_vel_arm)
t5 = t4 + max(|swing5-swing4|/wp4_vel_swing, |boom5-boom4|/wp5_vel_boom, |arm5-arm4|/wp5_vel_arm)
t6 = t5 + max(|bkt6-bkt5|/wp5_vel_bkt, |boom6-boom5|/wp5_vel_boom, |arm6-arm5|/wp5_vel_arm)
```

姿态角约束：任何段 `boom+arm+bucket ≥ attitude_angle_deg` 时，用 `bucket = attitude - boom - arm`（并 clamp 到关节限位）修正 bucket，避免铲斗过度上翻导致洒料。

## 插值方法：三次 Hermite（C¹ 连续）

段内公式（`s = (t - t_i) / T ∈ [0,1]`，`T = t_{i+1} - t_i`）：

```
q(s) = h₀₀·q_i  +  h₁₀·T·v_i  +  h₀₁·q_{i+1}  +  h₁₁·T·v_{i+1}
h₀₀(s) =  2s³ - 3s² + 1      h₁₀(s) = s³ - 2s² + s
h₀₁(s) = -2s³ + 3s²           h₁₁(s) = s³ - s²
```

### 切线选择策略

| 航路点 | 切线 v_i | 说明 |
|:---:|:---|:---|
| 端点 (v₀, v₅) | **0** | 起停静止约束 |
| 内部 (v₁…v₄) | `v_i = (q_{i+1} − q_{i-1}) / (t_{i+1} − t_{i-1})` | Catmull-Rom 中心差分 |

> 相邻时刻重合时切线自动降为 0，避免除零。

### 关键性质

- ✅ **位置精确**：每个航路点位置严格通过
- ✅ **C¹ 连续**：位置和速度在航路点处均连续（优于三次多项式仅在位置连续）
- ✅ **端点静止**：起终点速度为零，退出/进入后续阶段无冲击
- ✅ **单调性**：Catmull-Rom 差分方向遵循航路点序列走向，避免 Hermite 过冲

详见 [cubic_hermite_interpolator.h](include/dump_trajectory_planner/cubic_hermite_interpolator.h)。

## WP4 入厢点（回转圆与厢矩形交点策略）

`ComputeMiddleUpXY()` 照 MATLAB `calc_entry_point_circle_strategy` 实现：

1. **回转圆**：以回转中心 O=(0,0) 为圆心，半径 `R = wp4_R_coff · |dig_end| + (1 - wp4_R_coff) · |unload_point|`
   （`dig_end` 由起点关节角的 FK 得到）；
2. **矩形变换**：卡车位姿 `(center, β)` 给定一个矩形厢，将回转圆变换到厢本地坐标系；
3. **圆-矩形交点**：对矩形 4 条边分别求交点（最多 8 个）；
4. **选交点**：`rot_dir` 由 `dig_end swing → unload swing` 的最短路径自动判定；
   取沿 `rot_dir` 方向从 `dig_end swing` 出发角度差最小的交点作为 `p_entry`；
5. **fallback**：无交点时，以矩形 4 条边上距离 O 最近的点为方向，在回转圆上取对应圆上投影作为 `p_entry`；
6. **退化**：`R < 1e-6` 或方向量归零时，回退到卸载点 xy。

> RTK 方位角 α 与挖机 base 系卡车航向 β 关系：`β = (180° + α) mod 360°`（与 `truck_dump_planner` 一致）。
>
> 卡车尺寸 `truck_box_length` / `truck_box_width` 经 yaml 静态提供。

## 目录结构

```
dump_trajectory_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── dump_trajectory.yaml       # 参数（含 kinematics 段）
├── launch/
│   └── dump_trajectory.launch     # 启动
├── include/dump_trajectory_planner/
│   ├── angle_utils.h              # 角度换算/归一化/swing 展开（纯 C++）
│   ├── velocity_estimator.h       # 双路关节速度估计器（纯 C++）
│   ├── cubic_hermite_interpolator.h  # 关节空间三次 Hermite 插值（纯 C++）
│   ├── dump_feasibility.h         # 卸载点 IK 可行性检查
│   ├── waypoint_generator.h       # 6 关节航路点 + 段时间
│   ├── status_reporter.h          # 状态/统计/适配类话题统一发布出口
│   ├── dump_visualizer.h          # RViz 联调可视化（5 话题）
│   └── dump_trajectory_node.h     # ROS 节点类（状态机 + 在线规划）
└── src/
    ├── angle_utils.cpp
    ├── velocity_estimator.cpp
    ├── status_reporter.cpp
    ├── dump_visualizer.cpp
    ├── cubic_hermite_interpolator.cpp
    ├── dump_feasibility.cpp
    ├── waypoint_generator.cpp
    ├── dump_trajectory_node.cpp
    └── dump_trajectory_node_main.cpp
```

### 分层职责

| 层 | 组件 | 职责 |
|---|---|---|
| 纯 C++ 工具层（无 ROS 依赖，可单测） | `angle_utils` | 角度换算、归一化、最短角距、swing 展开/回包、关节误差 |
| | `velocity_estimator` | swing 与臂架双路历史独立差分估计关节速度（±30°/s 限幅） |
| | `cubic_hermite_interpolator` / `dump_feasibility` / `waypoint_generator` | 插值、可达性、航路点生成 |
| ROS 适配层 | `status_reporter` | 除执行指令流外的全部话题出口（含 latched 属性与功能安全状态字心跳计数） |
| | `dump_visualizer` | RViz 联调可视化：卡车包络/臂架线段/WP+回转圆/段轨迹/状态文本，受 `visualization/enable` 总开关控制 |
| 节点层 | `dump_trajectory_node` | 参数加载、订阅回调、状态机、在线段规划、门控、执行指令发布 |
| 入口层 | `dump_trajectory_node_main` | ROS 初始化 + spin |

## 依赖

- `roscpp` / `geometry_msgs` / `std_msgs` / `visualization_msgs`
- **[kinematics](../kinematics/README.md)**：运动学 IK 求解库
- C++17（因 `kinematics` 使用 `std::optional`）

## 编译

```bash
cd ~/yx_planning_ws
source devel/setup.bash            # 若首次编译可跳过
catkin_make --pkg kinematics        # 依赖库（首次）
catkin_make --pkg dump_trajectory_planner
source devel/setup.bash
```

## 启动

```bash
roslaunch dump_trajectory_planner dump_trajectory.launch
```

## 参数

见 [config/dump_trajectory.yaml](config/dump_trajectory.yaml)，主要段：

| 段 | 关键字段 | 说明 |
|:---|:---|:---|
| `kinematics/*` | `boom_length/arm_length/bucket_tooth_length` + 销轴偏置 + 6 项关节限位 | 由 `KinematicsSolver` 加载 |
| `feasibility/*` | `enable_envelope_prefilter`、`reach_min/max`、`dump_height_min/max`、`bucket_attitude_deg` | 包络粗筛 + IK 判定 |
| `waypoint/*` | `attitude_angle_deg`、`wp4_height_bias`（铰接点相对卡车表面偏置）、`wp4_bucket_angle_deg`（IK 铲斗关节角）、`p3/p5_*_cof`、各段速度、`truck_box_length/width`、`wp4_R_coff` | 见“关键航路点方案”与“WP4 入厢点”章节 |
| `online_plan/*` | `seg0_boom_deg`、`seg1_swing_deg`、`seg2_swing_deg`、`seg2_height_gate`、`seg3_swing_deg`、`seg4_arm_deg`、`seg4_bucket_deg`、`segment_confirm_frames`、`segment_timeout_factor`、`truck_top_height_m`、`bucket_clear_margin_m`、`bucket_clear_seg_idx`、`gate_timeout_sec`、`gate_boost_boom_deg`、`gate_max_boost_count`、`waypoint_clamp_abort_deg` | 在线规划与门控参数 |
| `visualization/*` | `enable`、`frame_id` | 可视化总开关与 marker 坐标系 |
| —— | `publish_rate`、`total_timeout_sec` | 全局 |

## 状态机（在线规划版）

```
kIdle ──[SeqAction==4 & inputs valid & IK可达 & WP1-6生成成功]──► kPlanNextSegment
kIdle ──[任一条件失败]──► kIdle（保持）

kPlanNextSegment ──[基于实际关节角 Hermite 插值当前段]──► kExecutingSegment
kPlanNextSegment ──[所有段已完成]──► kDone

kExecutingSegment ──[关节角误差<容差 或 段超时]──► kWaitBucketClear (若当前段=bucket_clear_seg_idx)
                  ──[关节角误差<容差 或 段超时]──► kPlanNextSegment (其它段)
                  ──[seg2 段超时且铲斗高度未达标]──► kWaitBucketClear (安全：不强推跨越，转 boost 重试)
kExecutingSegment ──[Sys_SeqAction≠4 或 total_timeout]──► kIdle

kWaitBucketClear ──[bucket_z > truck_top_height + margin]──► kPlanNextSegment
kWaitBucketClear ──[超过 gate_timeout_sec 未达标 且 提升次数未耗尽]──► kGateBoost
kWaitBucketClear ──[超过 gate_max_boost_count 次仍不达标 或 boom 已达上限]──► kIdle（终止）

kGateBoost ──[反馈到达提升目标 或 提升段超时]──► kWaitBucketClear（重新检测高度）

kDone ──[SeqAction==4 再次触发]──► kPlanNextSegment
```

### 在线规划核心逻辑

1. **激活时**仅生成 WP1–WP6 航路点，不做全程插值；
2. **每段开始时**取实际关节角作为起点，在线 Hermite 插值到下一航路点，消除累积偏差；
3. **段完成判定**：各段仅检查主导关节，容差独立配置（见下表），连续 `segment_confirm_frames` 帧确认（主判定）或段超时（兜底）：

   | 段 | 判定关节 | 参数 |
   |:---:|:---|:---|
   | seg0 WP1→WP2 | 仅 boom | `seg0_boom_deg` |
   | seg1 WP2→WP3 | 仅 swing | `seg1_swing_deg` |
   | seg2 WP3→WP4 | swing + 高度门控 | `seg2_swing_deg` + `seg2_height_gate` |
   | seg3 WP4→WP5 | 仅 swing | `seg3_swing_deg` |
   | seg4 WP5→WP6 | arm + bucket（两者均达标） | `seg4_arm_deg`、`seg4_bucket_deg` |

   > **seg2 安全特例**：seg2 是跨越卡车的关键段，其高度门控不允许被超时兜底击穿。若段超时时铲斗仍未高于卡车上表面，**不强制推进**，而是转入 `kWaitBucketClear` 复用 boost 重试机制（次数耗尽则由门控统一终止），避免在高度不足时继续跨越。
4. **铲斗高度门控**：段1（WP2→WP3，回转至中途）完成后，等待 FK 齿尖高度 `RealBktPosXYZ.z > truck_top_height_m + margin` 才允许进入跨越段（高度由实时关节角 FK 计算，非外部传感器实测）；seg2 跨越段内再次内联检查高度（见上）。
   - 到达 WP3 时若高度已达标则**立即通过**（不停等）；
   - 等待超过 `gate_timeout_sec` 未达标时，自动追加一小段 boom 提升（`gate_boost_boom_deg`，arm 反向联动保持铲斗姿态），完成后重新检测；
   - 超过 `gate_max_boost_count` 次仍不达标、或 boom 已达关节上限时，终止卸载交由上层处理；
5. **段时间自适应**：基于实际角度差和主导关节速度重新计算，而非使用预计算值。
6. **段间平滑过渡**：段起点切线取反馈估计速度（swing 与 boom/arm/bucket **两路历史独立维护**，各自用自身话题时间戳做最近 3 帧首末差分并限幅 ±30°/s，适配两话题发布频率不同场景）；段终点切线取预计算的 Catmull-Rom 目标切线（门控点与端点为 0）。

### 对液压特性的适应

| 液压特性 | 应对机制 |
|:---|:---|
| 启动死区 | 起点取实际值，死区期间误差不累积 |
| 响应延迟 50–200ms | 基于位置到达推进，自然等待 |
| 负载相关速度 | 段时间由实际角度差决定，重载自动延长 |
| 温度漂移 | 每段重新规划，不依赖全局预估 |
| 过冲/振荡 | 容差带内才推进，振荡不会误判为“到达” |

## 回转角度跨越 0°/360° 边界问题

### 问题描述

挖掘机回转角度 swing 范围为 [0°, 360°)，当轨迹跨越 0°/360° 边界时（如 350° → 10°，最短路径应为 +20° 而非 -340°），以下环节会出错：

| 受影响环节 | 具体问题 |
|:---|:---|
| WP3 中点计算（航路点生成） | `median(swing1, swing4)` 算术平均：350° 与 10° 的中点被算成 180° 而非 0°，回转方向完全错误 |
| 段完成判定 (`ComputeSegError`) | swing 差值算出 350° 而非 10°，永远不触发“到达”条件（已用 `ShortestAngularDistanceDeg` 修复） |
| 段时间计算 (`PlanCurrentSegment`) | `swing_diff` 过大导致段时间不合理 |
| Hermite 插值 | 在 0°/360° 断裂处走“长路”，回转方向反转 |
| Catmull-Rom 切线 | 中心差分符号错误，切线方向反转 |

### 解决方案：角度展开/最短距离 + 回包

采用 **“输入展开 → 计算/插值 → 输出回包”** 三步策略。展开在两个层面进行：

- **航路点生成时**（`GenerateDumpWaypoints` 内部）：swing 链 sw1→sw4→sw_unload 沿最短路径展开（`SignedShortestDiffDeg`），WP3 弧中点、WP3 最小步长回退、以及各段时间的 swing 差值全部在展开链上计算，从源头避免方向错误；
- **节点接收航路点后**：`UnwrapSwingSequence` 再做一次相邻点展开（幂等），保证序列连续。

#### 步骤 1：角度展开（Unwrap）

在生成航路点序列后，将 swing 角度进行展开，使相邻点之间角度差绝对值 ≤ 180°：

```cpp
/// 角度展开：保证相邻航路点 swing 差值 ≤ 180°
void UnwrapSwingSequence(std::vector<JointState>& waypoints) {
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double diff = waypoints[i].swing - waypoints[i-1].swing;
        // 归一化到 (-π, π]
        while (diff > M_PI)  diff -= 2.0 * M_PI;
        while (diff <= -M_PI) diff += 2.0 * M_PI;
        waypoints[i].swing = waypoints[i-1].swing + diff;
    }
}
```

展开后 swing 可能超出 [0°, 360°)，但相邻点间插值方向正确。

#### 步骤 2：最短角距离（用于段完成判定）

段完成判定和段时间计算中，swing 差值必须用最短角距离：

```cpp
/// 最短角距离（deg），返回值 ∈ [0, 180]
inline double ShortestAngularDistanceDeg(double a_deg, double b_deg) {
    double diff = std::fmod(b_deg - a_deg, 360.0);
    if (diff > 180.0)  diff -= 360.0;
    if (diff < -180.0) diff += 360.0;
    return std::abs(diff);
}
```

影响的代码位置：

| 函数 | 说明 |
|:---|:---|
| `ComputeSegError()` | 各段已内建 `ShortestAngularDistanceDeg` 计算 swing 误差 |
| `PlanCurrentSegment()` | 段时间计算已使用 `ShortestAngularDistanceDeg` |

#### 步骤 3：输出回包（Wrap）

发布前将 swing 回包到 [0°, 360°)：

```cpp
/// 角度回包到 [0, 360) deg
inline double WrapTo360(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0) deg += 360.0;
    return deg;
}
```

在 `PublishTrajectoryFrame()` 中发布 swing 时应用：
```cpp
msg.orientation.x = WrapTo360(Rad2Deg(q.swing));  // 而非直接 Rad2Deg
```

### 实现注意事项

1. **展开时机**：应在 `GenerateDumpWaypoints()` 返回后、存入 `waypoints_` 前调用 `UnwrapSwingSequence`；在线规划每段时，`seg_start`（取自实际关节角）也需相对目标航路点展开；
2. **Hermite 插值无需修改**：展开后的 swing 值在数值上连续，插值器无感知角度边界；
3. **Catmull-Rom 切线**：展开后差分方向自然正确，无需额外处理；
4. **`current_joint_.swing` 取自话题**：原始值在 [0°, 360°)，在线规划时需展开到与目标航路点同一周期：
   ```cpp
   // PlanCurrentSegment() 中：
   double target_swing = waypoints_[current_seg_idx_ + 1].swing;  // 已展开
   double actual_swing = current_joint_.swing;  // 原始 [0, 2π)
   // 将 actual 展开到与 target 最近的周期
   double diff = actual_swing - target_swing;
   while (diff > M_PI)  diff -= 2.0 * M_PI;
   while (diff <= -M_PI) diff += 2.0 * M_PI;
   seg_start.swing = target_swing + diff;
   ```
5. **边界场景举例**：
   - 当前 swing=350°, 卸载点 swing=10° → 展开后目标=370°，插值走 +20° ✔
   - 当前 swing=10°, 卸载点 swing=350° → 展开后目标=-10°，插值走 -20° ✔
   - 当前 swing=180°, 卸载点 swing=0° → 展开后目标=0°（或 360°），取最近方向 ✔

### 与在线规划方案的協同

在线规划的“每段取实际关节角重新规划”设计天然缓解了 0°/360° 问题：即使某段因未处理包裹而产生微小偏差，下一段会以实际值为起点重新规划，偏差不会累积。但仍需在单段内保证插值方向正确。

## TODO

- [x] ~~实现回转角 0°/360° 包裹处理~~（已完成，见上节方案）
- [x] ~~WP3 swing 中点 0°/360° 跨界方向错误~~（生成阶段 swing 链沿最短路径展开）
- [x] ~~航路点关节限位校验~~（`ValidateWaypointsJointLimits`，超限 clamp + 告警）
- [x] ~~段完成持续确认防抖~~（`segment_confirm_frames` 参数化）
- [x] ~~门控高度不达标死等~~（超时自动 boom 提升重试，次数耗尽终止）
- [x] ~~速度估计双话题混写相位偏差~~（swing/joints 两路历史分离，各自时间戳差分）
- [ ] 入厢点策略按卡车进入方向预筛选交点（当前仅取角度差最小）；
- [ ] 端段 Hermite 切线优化：考虑用单侧差分替代端点零切线，获得更平滑的起停过渡；
- [ ] 增加执行中的反向抖动保护与超范围检测；
- [ ] 单元测试（`gtest`）：
  - 三次 Hermite 插值端点精度与 C¹ 连续性验证；
  - 6 航路点姿态角约束边界；
  - 入厢点不同 rot_dir / 卡车位姿组合；
  - **回转角跨越 0°/360° 的插值方向验证**；
  - Catmull-Rom 切线段时长退化场景；
- [ ] 与系统决策节点、执行节点做联调。
