# dump_trajectory_planner

无人挖机 **卸载阶段关节空间轨迹规划节点**（在线规划版）。

## 功能概述

- 由 `/Sys_SeqAction == 4.0` 激活；
- 接收卡车近点及航向 `/truck_center_unloadpoint` → base 系 β=(180+α) mod 360 → rad；可选接收卡车远点 `/truck_center_unloadpoint_2`；
- **动态卸载点选取**：收到远点时，在近点↔远点间生成 21 个候选点，由远及近做 IK 可达性验证，在可达区域取最远/中间/最近三点，按当前装载斗数 `/Sys_SUn_BucketNumber` 依次选取（前 3 斗最远、中 3 斗中间、后 2 斗最近、超出默认中间）；未收到远点或未启用时回退单点模式（近点即卸载点，z=0）；
- 订阅实时关节角作为轨迹起点（swing 经 `/heading2swing_topic`，boom/arm/bucket 经 `/joints_angle`）；
- 由实时关节角经运动学 FK 计算铲斗齿尖坐标并发布 `/RealBktPosXYZ`，用于卸载过程中的高度安全门控；
- 通过 [kinematics](../kinematics/README.md) 的 `swing_center_inverse_by_bucket_attitude` 做**真 IK 判定**，同时解算卸载点关节角；
- 在**关节空间**生成 **4 关键航路点**（WP1/WP4/WP5/WP6），其中 WP4 入厢点由**回转圆与卡车厢矩形交点策略**确定；
- 采用**基于航路点的在线规划**：每段基于实际关节角在线 PCHIP 插值，反馈驱动段推进；
- **Seg0 两阶段策略**：boom + swing 双阶跃（boom 饱和举升 + swing 满功率回转到"卡车附近"）+ 高度门控 + PCHIP 平滑收尾，兼顾效率与液压友好；
- **铲斗高度门控**：Seg0 完成后，确认铲斗实际高度超过卡车上表面才允许跨越卡车；等待超时未达标时自动追加 boom 提升重试，次数耗尽则终止卸载；
- 以 10 Hz 流式逐帧发布 `/RefDeviceTraj_Dump`（纯位置指令，无速度前馈）；
- **RViz 联调可视化**：卡车 3D 线框 + 挖掘机臂架线段 + WP 球体/标签 + WP4 回转圆 + 当前段轨迹折线 + 状态机调试文本。

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
|:---:|:---|:---|:---|
| Sub | `/Sys_SeqAction` | `std_msgs/Float64` | `data == 4.0` 触发卸载 |
| Sub | `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | `x/y` = 卡车近点（base 系，m）；`w` = RTK 方位角 α（deg，北0顺时针） |
| Sub | `/truck_center_unloadpoint_2` | `geometry_msgs/Quaternion` | `x/y` = 卡车远点（base 系，m），用于动态卸载点选取（可选，未收到则回退单点模式） |
| Sub | `/Sys_SUn_BucketNumber` | `std_msgs/Float64` | `data` = 当前装载斗数，决定选取最远/中间/最近卸载点 |
| Sub | `/joints_angle` | `geometry_msgs/Quaternion` | `x/y/z` = boom/arm/bucket 关节角（deg） |
| Sub | `/heading2swing_topic` | `std_msgs/Float32` | `data` = swing 关节角（deg） |
| **Pub** | **`/RefDeviceTraj_Dump`** | **`geometry_msgs/Pose`** | **`Orientation.x/y/z/w` = swing/boom/arm/bucket（deg）；`Position.x` 1=执行中 / 0=阶段结束** |
| Pub | `/Swing_topic` | `geometry_msgs/Point` | `x` = swing（deg），由 `/heading2swing_topic` 实时转发 |
| Pub | `/RealBktPosXYZ` | `geometry_msgs/Point` | 铲斗齿尖坐标（base 系，m），由实时关节角经运动学 FK 计算 |
| Pub | `/Sys_RUn_FlagUnloadExcuteFinish` | `std_msgs/Float64` | 0=执行中 / 1=结束或空闲（latched） |
| Pub | `/UPFeasible` | `std_msgs/Bool` | 卸载点可达性校验结果（latched） |
| Pub | `/loadTraj_maxBoom` / `/loadTraj_maxZ` | `std_msgs/Float32` | 航路点动臂最大角度 / 齿尖最高 z（latched） |
| Pub | `/UT_PlanningPulse_topic` | `std_msgs/UInt32` | 功能安全状态字（位段结构见下） |
| Pub | `dump_viz/*` | `visualization_msgs/MarkerArray` | 卡车/臂架/航路点/轨迹/状态文本（受 `visualization/enable` 总开关控制） |

`/UT_PlanningPulse_topic` 位拼接结构：

| 位段 | 字段 | 说明 |
|:---:|:---|:---|
| `[31:16]` | `heartbeat_cnt` | 心跳计数，每次发布 +1，>255 回绕 0 |
| `[15:8]` | `module_id` | 轨迹规划模块 ID = 2 |
| `[7:0]` | `error_code` | 0=正常；1=卡车位置无法卸载；2=轨迹规划失败（WP4 可达性失败或限位校验超阈值） |

## 动态卸载点选取（可选，按装载斗数三档映射）

卸载点在航路点生成前确定。收到卡车远点 `/truck_center_unloadpoint_2` 且 `unload_selector_enable` 为真时，在近点↔远点连线上按斗数动态选取真正的卸载点；否则回退单点模式（近点即卸载点）。

### 流程

1. **退化保护**：近点或远点无效，或两点距离 `< 0.5m` → 跳过选取，回退单点模式；
2. **生成候选**：在远点→近点连线上等距生成 `unload_candidates_count(21)` 个候选点（index 0 = 远点，index n-1 = 近点），`z = 0`；
3. **逐候选 IK 验证**：由远及近对每个候选点调用 `CheckReachable`（包络粗筛 + 真 IK），统计可达数量；全不可达 → 回退单点模式；
4. **可达区域三点**：
   - `far_idx` = 首个可达（最远）；
   - `near_idx` = 末个可达（最近）；
   - `mid_idx` = `(far_idx + near_idx)/2`，若该点不可达则在 `[far_idx, near_idx]` 内取距 `mid_idx` 最近的可达点作 `mid_feasible`；
5. **按斗数选取**（斗数取 `/Sys_SUn_BucketNumber`，无效则视为 0）：

   | 斗数区间 | 选取 |
   |:---|:---|
   | `1 ~ unload_bucket_far_count(3)` | 最远 `far_idx` |
   | `4 ~ far+mid(6)` | 中间 `mid_feasible` |
   | `7 ~ far+mid+near(8)` | 最近 `near_idx` |
   | 其它（含 0 / 超出） | 默认中间 `mid_feasible` |

6. **输出**：选中候选点的坐标写入 `unload_point_`、可达结果写入 `FeasibilityResult`，作为后续航路点（WP4/WP5/WP6）生成的依据。

### 降级策略（三层）

- 远点无效 / 未启用 / 距离过近 / 全不可达 / 选取失败 → 回退单点模式（近点 `CheckReachable`）；
- 单点也不可达 → `/UPFeasible=false` + `error_code=1`，不激活；
- 每次激活做一次选取（21 次 IK 调用，一次性开销，可接受）。

## 关键航路点方案（关节空间，4 航路点）

4 个关节航路点在关节角空间生成，仅 WP4 借助 IK 从笛卡尔求解：

| 点 | 名称 | 主要动作 | 计算 |
|:---:|:---|:---|:---|
| WP1 | 起点 | 挖掘终止时的关节角 | 直接取实时关节角，铲斗按姿态角上限修正 |
| WP4 | 厢上过渡点 | 到达卸载点上方 | `xy = ComputeMiddleUpXY()`（回转圆与厢矩形交点策略），铰接点高度 `z = truck_top_height + wp4_height_bias`，IK 求解 |
| WP5 | 卸载过渡点 | `swing/boom/arm` 到位 | `swing5 = unload_swing`；`boom5/arm5` 按 `p5_boom_cof/p5_arm_cof` 混合到卸载点 |
| WP6 | 终点 | 铲斗翻转卸料 | `= unload_joint`（IK 解算结果） |

**3 段规划**：

| 段 | 区间 | 主导关节 | 完成容差 |
|---|---|---|---|
| **Seg0** | WP1→WP4 | 两阶段策略（见下） | swing 5° / boom 5° + 高度门控 |
| **Seg1** | WP4→WP5 | swing 回转 | swing 5° |
| **Seg2** | WP5→WP6 | arm+bucket 卸载 | arm 10° / bucket 20° |

段时间由主导关节速度决定（`max(|Δjoint|/vel_dps)`）。

## 插值方法：PCHIP（保形分段三次 Hermite，无超调）

段内公式（`s = (t - t_i) / T ∈ [0,1]`）：

```
q(s) = h₀₀·q_i  +  h₁₀·T·v_i  +  h₀₁·q_{i+1}  +  h₁₁·T·v_{i+1}
h₀₀(s) =  2s³ - 3s² + 1      h₁₀(s) = s³ - 2s² + s
h₀₁(s) = -2s³ + 3s²           h₁₁(s) = s³ - s²
```

起终切线经 **Fritsch-Carlson 单调性修正**，保证插值不产生超调。详见 [cubic_hermite_interpolator.h](include/dump_trajectory_planner/cubic_hermite_interpolator.h)。

## WP4 入厢点（回转圆与厢矩形交点策略）

`ComputeMiddleUpXY()` 照 MATLAB `calc_entry_point_circle_strategy` 实现：

1. **回转圆**：以回转中心 O=(0,0) 为圆心，半径 `R = wp4_R_coff · |dig_end| + (1 - wp4_R_coff) · |unload_point|`；
2. **矩形变换**：卡车位姿 `(center, β)` 给定一个矩形厢，将回转圆变换到厢本地坐标系；
3. **圆-矩形交点**：对矩形 4 条边分别求交点（最多 8 个）；
4. **选交点**：`rot_dir` 由 `dig_end swing → unload swing` 的最短路径自动判定；取沿 `rot_dir` 方向从 `dig_end swing` 出发角度差最小的交点作为 `p_entry`；
5. **fallback**：无交点时，以矩形 4 条边上距离 O 最近的点为方向，在回转圆上取对应圆上投影作为 `p_entry`；
6. **退化**：`R < 1e-6` 或方向量归零时，回退到卸载点 xy。

> RTK 方位角 α 与挖机 base 系卡车航向 β 关系：`β = (180° + α) mod 360°`。

## Seg0 两阶段策略（核心）

Seg0（WP1→WP4）采用两阶段策略，兼顾效率与液压友好：

### 启用判据

仅当 `step_deg = seg0_midpoint_ratio × boom抬升量 ≥ seg0_min_step_factor × seg0_switch_threshold_deg` **且 boom 抬升**时启用两阶段；否则退化为单段 PCHIP。

### 运动分配

| 关节 | **Phase1**（双阶跃段） | **Phase2**（swing 到位 + 高度 OK 后） |
|---|---|---|
| **boom** | 阶跃到中点 `WP1.boom+ratio×抬升`，阀口**饱和举升** | PCHIP 收尾到 WP4（低流量） |
| **arm** | **保持起点不动**（零流量，避免与 boom 抢泵） | **阶跃外伸到 WP4.arm** |
| **swing** | 阶跃到"卡车附近" `WP4.swing − sign×seg0_swing_offset_deg`，阀口**饱和回转** | PCHIP 续接到 WP4.swing，起点用 **Phase1 指令值**（保证指令连续），速度取**实测值**（保证斜率匹配） |
| **bucket** | **每帧按实测 boom/arm 反馈实时算参考角**维持姿态（单向收斗） | PCHIP 到 WP4.bucket |

### Phase1 bucket 实时姿态保持

```
bucket_hold = bucket_attitude_target_deg − boom_实测 − arm_实测
姿态角(用参考bucket评估) ≥ 目标 → 输出 bucket_hold(裁剪到限位)；否则保持上一帧
```

### 切换条件（swing 到位 + 高度门控）

- `ShortestAngularDistanceDeg(swing实测, swing_Phase1目标) ≤ seg0_switch_threshold_deg(3°)` → **swing 到位**；
- swing 到位后**立即检查高度门控**（不管 boom 是否到位）：
  - 高度 OK → 切 Phase2；
  - 高度不足 → 转 `kWaitBucketClear`（gate boost 抬 boom 直到齿尖高于卡车）；
- **Phase1 独立超时**（`max(boom行程/wp2_vel_boom_dps, swing行程/seg0_phase1_swing_dps) × factor`）→ 转高度门控 boost 重试。

### Phase2 起点斜率衔接（防阀反向冲击）

`v_start = VelocityEstimator::Estimate()`（arm 分量置 0，因 arm 已阶跃到位），并对时长施加 `p2_time ≤ 3Δ/|v_start|` 的 Fritsch-Carlson 可衔接约束。

## 状态机

```
kIdle ──[SeqAction==4 & inputs valid & IK可达 & WP生成成功]──► kPlanNextSegment
kIdle ──[任一条件失败]──► kIdle（保持）

kPlanNextSegment ──[基于实际关节角 PCHIP 插值当前段]──► kExecutingSegment
kPlanNextSegment ──[所有段已完成]──► kDone

kExecutingSegment ──[归一化误差≤1.0 连续N帧 或 段超时]──► kWaitBucketClear (若当前段=bucket_clear_seg_idx)
                  ──[归一化误差≤1.0 连续N帧 或 段超时]──► kPlanNextSegment (其它段)
                  ──[Seg0 段超时且铲斗高度未达标]──► kWaitBucketClear (安全：不强推跨越，转 boost 重试)
kExecutingSegment ──[Sys_SeqAction≠4 或 total_timeout]──► kIdle

kWaitBucketClear ──[bucket_z > truck_top_height + margin]──► kPlanNextSegment
kWaitBucketClear ──[超过 gate_timeout_sec 未达标 且 提升次数未耗尽]──► kGateBoost
kWaitBucketClear ──[超过 gate_max_boost_count 次仍不达标 或 boom 已达上限]──► kIdle（终止）

kGateBoost ──[反馈到达提升目标 或 提升段超时]──► kWaitBucketClear（重新检测高度）

kDone ──[SeqAction==4 再次触发]──► kPlanNextSegment
```

## 段完成判定

- **归一化逐关节判据**：`max_j(err_j / tol_j) ≤ 1.0`（等价于所有主导关节各自达标 AND，修正了旧 `max(err)≤max(tol)` 会让松容差放过紧关节的漏洞）。
- **N 帧确认防抖**：Seg0 用 `seg0_confirm_frames=8`（覆盖 boom 阶跃后 0.5~2Hz 液压振荡模态），其余 `segment_confirm_frames=3`。
- **超时兜底**：`base_time = max(标称段时长, 实际规划时长)`，超时 = base × `segment_timeout_factor(2.0)`。Seg0 若门控未达标 → `kEnterGate`（不强推，安全第一）；否则强制推进。
- Seg0 高度门控未达标时 `ComputeSegError` 直接返回 `1e6` 阻止完成。

## 高度门控与 boost 重试

- **触发点**：Seg0 Phase1 期间 swing 到达"卡车附近"时（**早于 Seg0 完成**，提前检测高度不足）。swing 到位后不管 boom 是否到位，立即检查高度。
- **判据**：齿尖 FK 高度 `bucket_height_ > truck_top_height_m(2.0) + bucket_clear_margin_m(1.8) = 3.8m`。
- **等待期**：恒定发布**锁存姿态** `gate_hold_cmd_`（不跟随反馈，避免液压保压沉降被逐帧追认成下沉指令）。
- **boost 重试**：`gate_timeout_sec(4s)` 超时 → boom+`gate_boost_boom_deg(3°)`、arm 反向联动(Δarm=-Δboom)保姿态；最多 `gate_max_boost_count(6)` 次，耗尽或 boom 到上限 → abort。

## 对液压特性的适应

| 液压特性 | 应对机制 |
|:---|:---|
| 启动死区 | 起点取实际值，死区期间误差不累积 |
| 响应延迟 50–200ms | 基于位置到达推进，自然等待 |
| 负载相关速度 | 段时间由实际角度差决定，重载自动延长 |
| 温度漂移 | 每段重新规划，不依赖全局预估 |
| 过冲/振荡 | 容差带内才推进 + N 帧确认，振荡不会误判为"到达" |
| **阀口饱和与流量竞争** | Seg0 两阶段时序分离 boom 举升与 arm 外伸，避免同帧饱和抢泵 |
| **阀反向冲击** | Phase2 起点切线取实测速度 + 斜率可衔接约束 |
| **保压沉降** | 门控等待期恒定发布锁存指令，不追认沉降 |

## 目录结构与分层职责

```
dump_trajectory_planner/
├── CMakeLists.txt / package.xml / README.md
├── config/dump_trajectory.yaml       # 参数（含 kinematics 段）
├── launch/dump_trajectory.launch     # 启动
├── include/dump_trajectory_planner/
│   ├── angle_utils.h                 # 角度换算/归一化/swing 展开（纯 C++）
│   ├── velocity_estimator.h          # 双路关节速度估计器（纯 C++）
│   ├── cubic_hermite_interpolator.h  # 关节空间 PCHIP 插值（纯 C++）
│   ├── dump_feasibility.h            # 卸载点 IK 可行性检查
│   ├── waypoint_generator.h          # 4 关节航路点 + 段时间
│   ├── status_reporter.h             # 状态/统计/适配类话题统一发布出口
│   ├── dump_visualizer.h             # RViz 联调可视化
│   └── dump_trajectory_node.h        # ROS 节点类（状态机 + 在线规划）
└── src/
    ├── angle_utils.cpp / velocity_estimator.cpp / status_reporter.cpp
    ├── dump_visualizer.cpp / cubic_hermite_interpolator.cpp
    ├── dump_feasibility.cpp / waypoint_generator.cpp
    ├── dump_trajectory_node.cpp / dump_trajectory_node_main.cpp
```

| 层 | 组件 | 职责 |
|---|---|---|
| 纯 C++ 工具层（无 ROS 依赖，可单测） | `angle_utils` / `velocity_estimator` / `cubic_hermite_interpolator` / `dump_feasibility` / `waypoint_generator` | 角度换算、速度估计、插值、可达性、航路点生成 |
| ROS 适配层 | `status_reporter` / `dump_visualizer` | 状态话题统一出口 / RViz 可视化 |
| 节点层 | `dump_trajectory_node` | 参数加载、订阅回调、状态机、在线段规划、门控、执行指令发布 |
| 入口层 | `dump_trajectory_node_main` | ROS 初始化 + spin |

## 依赖 / 编译 / 启动

- **依赖**：`roscpp` / `geometry_msgs` / `std_msgs` / `visualization_msgs` / **[kinematics](../kinematics/README.md)** / C++17
- **编译**：
  ```bash
  cd ~/yx_planning_ws && source devel/setup.bash
  catkin_make --pkg kinematics dump_trajectory_planner
  ```
- **启动**：`roslaunch dump_trajectory_planner dump_trajectory.launch`

## 参数

见 [config/dump_trajectory.yaml](config/dump_trajectory.yaml)，主要段：

| 段 | 关键字段 | 说明 |
|:---|:---|:---|
| `kinematics/*` | `boom_length/arm_length/bucket_tooth_length` + 销轴偏置 + 6 项关节限位 | 由 `KinematicsSolver` 加载 |
| `feasibility/*` | `enable_envelope_prefilter`、`reach_min/max`、`dump_height_min/max`、`bucket_attitude_deg` | 包络粗筛 + IK 判定 |
| `waypoint/*` | `attitude_angle_deg`、`wp4_height_bias`、`wp4_bucket_angle_deg`、`p3/p5_*_cof`、各段速度、`truck_box_length/width`、`wp4_R_coff` | 航路点生成 |
| `online_plan/*` | `seg0_swing_deg/boom_deg`、`seg1_swing_deg`、`seg2_arm_deg/bucket_deg`、`seg0_confirm_frames`、`segment_confirm_frames`、`segment_timeout_factor`、`truck_top_height_m`、`bucket_clear_margin_m`、`bucket_clear_seg_idx`、`gate_timeout_sec`、`gate_boost_boom_deg`、`gate_max_boost_count`、`waypoint_clamp_abort_deg`、**`seg0_midpoint_ratio`、`seg0_switch_threshold_deg`、`seg0_phase1_swing_dps`、`seg0_min_step_factor`、`seg0_swing_offset_deg`、`bucket_attitude_target_deg`**、**`unload_selector_enable`、`unload_candidates_count`、`unload_bucket_far_count/mid_count/near_count`** | 在线规划、Seg0 两阶段与动态卸载点选取 |
| `visualization/*` | `enable`、`frame_id` | 可视化总开关与 marker 坐标系 |
| 顶层 | `publish_rate`、`total_timeout_sec` | 全局 |

## 回转角跨越 0°/360° 边界问题

### 问题描述

挖掘机回转角度 swing 范围为 [0°, 360°)，当轨迹跨越 0°/360° 边界时（如 350° → 10°，最短路径应为 +20° 而非 -340°），以下环节会出错：

| 受影响环节 | 具体问题 |
|:---|:---|
| WP3 中点计算（航路点生成） | `median(swing1, swing4)` 算术平均：350° 与 10° 的中点被算成 180° 而非 0°，回转方向完全错误 |
| 段完成判定 (`ComputeSegError`) | swing 差值算出 350° 而非 10°，永远不触发"到达"条件 |
| 段时间计算 (`PlanCurrentSegment`) | `swing_diff` 过大导致段时间不合理 |
| Hermite 插值 | 在 0°/360° 断裂处走"长路"，回转方向反转 |
| Catmull-Rom 切线 | 中心差分符号错误，切线方向反转 |

### 解决方案：角度展开/最短距离 + 回包

采用 **"输入展开 → 计算/插值 → 输出回包"** 三步策略：

- **航路点生成时**：swing 链沿最短路径展开；
- **节点接收航路点后**：`UnwrapSwingSequence` 再做一次相邻点展开（幂等）；
- **段完成判定和段时间计算**：swing 差值用 `ShortestAngularDistanceDeg`；
- **发布前**：swing 回包到 [0°, 360°)。

详见 [angle_utils.h](include/dump_trajectory_planner/angle_utils.h)。

## 当前状态与验证

| 维度 | 状态 | 说明 |
|---|---|---|
| 编译 | ✅ 100% 通过 | `kinematics` + `dump_trajectory_planner` 全部 Built |
| 静态检查 | ✅ 零错误 | GetProblems 无 error/warning |
| **实机/仿真运行** | ⚠️ **未验证** | 所有设计均为理论，缺乏运行数据支撑 |
| 单元测试 | ⚠️ 缺失 | TODO 里列了但没做 |
| 下游速率限制器 | ⚠️ **未确认** | 决定阶跃策略是否真有效（上实机前最需确认） |
| 多执行器流量竞争 | ⚠️ 架构级未解 | 未做泵流量感知调度（路线 B） |

**核心结论**：设计完整、理论自洽，但缺乏实机/仿真数据验证。上实机前最需确认下游是否有速率限制器。

## TODO

- [x] ~~实现回转角 0°/360° 包裹处理~~（已完成，见上节方案）
- [x] ~~WP3 swing 中点 0°/360° 跨界方向错误~~（生成阶段 swing 链沿最短路径展开）
- [x] ~~航路点关节限位校验~~（`ValidateWaypointsJointLimits`，超限 clamp + 告警）
- [x] ~~段完成持续确认防抖~~（`segment_confirm_frames` 参数化）
- [x] ~~门控高度不达标死等~~（超时自动 boom 提升重试，次数耗尽终止）
- [x] ~~速度估计双话题混写相位偏差~~（swing/joints 两路历史分离，各自时间戳差分）
- [x] ~~Seg0 两阶段策略~~（boom + swing 双阶跃 + 高度门控 + PCHIP 平滑收尾）
- [x] ~~归一化段误差判据~~（`max(err/tol)≤1.0`，逐关节 AND）
- [x] ~~Phase2 起点切线衔接~~（`v_start=Estimate()` + 斜率可衔接约束）
- [x] ~~bucket 实时姿态保持~~（每帧按实测 boom/arm 反馈计算参考 bucket）
- [x] ~~动态卸载点选取~~（近点↔远点 21 候选 IK 验证 + 按斗数三档映射 + 三层降级）
- [ ] **实机/仿真运行验证**（最优先）；
- [ ] 入厢点策略按卡车进入方向预筛选交点；
- [ ] 端段 Hermite 切线优化：考虑用单侧差分替代端点零切线；
- [ ] 增加执行中的反向抖动保护与超范围检测；
- [ ] 单元测试（`gtest`）：
  - PCHIP 插值端点精度与 C¹ 连续性验证；
  - Seg0 两阶段切换连续性验证；
  - bucket 实时姿态保持边界；
  - 回转角跨越 0°/360° 的插值方向验证；
- [ ] 与系统决策节点、执行节点做联调。
