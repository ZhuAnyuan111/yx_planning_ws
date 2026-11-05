# dump_trajectory_planner

无人挖机 **卸载阶段关节空间轨迹规划节点**（在线规划版）。

## 功能概述

- 由 `/Sys_SeqAction == 4.0` 激活在线轨迹执行；另支持 `/Sys_SUn_FlagUnloadPlan == 1.0` 触发**预校验**（卸载点分布+航路点计算，只算不发轨迹），成功后发布 `/Sys_unloadPlanFinish = 1`；
- 接收卡车近点及航向 `/truck_center_unloadpoint`；卡车航向**优先由「近点→远点」连线方向**决定（两点均为 base/下车体系坐标，β=atan2(far.y−near.y, far.x−near.x)），远点缺失时回退 RTK 换算 β=(180−α) mod 360 → rad；可选接收卡车远点 `/truck_center_unloadpoint_2`；
- **动态卸载点选取**：收到远点时，在近点↔远点间生成 21 个候选点，由远及近做 IK 可达性验证，在可达区域取最远/中间/最近三点，按当前装载斗数 `/Sys_SUn_BucketNumber` 依次选取（前 3 斗最远、中 3 斗中间、后 2 斗最近、超出默认中间）；未收到远点或未启用时回退单点模式（近点即卸载点，z=0）；
- 订阅实时关节角作为轨迹起点（swing 经 `/heading2swing_topic`，boom/arm/bucket 经 `/joints_angle`）；
- 由实时关节角经运动学 FK 计算铲斗齿尖坐标并发布 `/RealBktPosXYZ`，用于卸载过程中的高度安全门控；
- 通过 [kinematics](../kinematics/README.md) 的 `swing_center_inverse_by_bucket_attitude` 做**真 IK 判定**，同时解算卸载点关节角；
- 在**关节空间**生成 **4 关键航路点**（WP1/WP4/WP5/WP6），其中 WP4 入厢点由**回转圆与卡车厢矩形交点策略**确定；
- 采用**基于航路点的在线规划**：每段基于实际关节角在线 PCHIP 插值，反馈驱动段推进；
- **Seg0 两阶段策略**：swing 满功率阶跃回转到"卡车附近" + boom 阶跃到可配"中间角"（`seg0_boom_pchip_start_cof`）后立即并行 PCHIP 抬到 WP4 + **arm 条件打出（deploy）** + Phase2 时间 PCHIP 从实测值平滑收尾到 WP4，兼顾效率与液压友好；
- **铲斗高度门控**：以"并行中断监视器 + 段边界兜底"形式生效（非串行闸门）：Seg0 Phase2 宽限期后逐帧监视齿尖高度，不足则转 boost 重试；段边界（`bucket_clear_seg_idx`）必判一次，未达标不推进；超时未达标自动追加 boom 提升，次数耗尽则终止卸载；
- 以 10 Hz 流式逐帧发布 `/RefDeviceTraj_Unload`（纯位置指令，无速度前馈）；
- 航路点定稿后发布卸载终点关节角 `/Sys_RUn_UnloadEndAngle`（latched，WP6/卸载点信息）；
- **RViz 联调可视化**：卡车 3D 线框 + 挖掘机臂架线段 + WP 球体/标签 + WP4 回转圆 + 当前段轨迹折线 + 状态机调试文本。

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
|:---:|:---|:---|:---|
| Sub | `/Sys_SeqAction` | `std_msgs/Float64` | `data == 4.0` 触发卸载（在线轨迹执行） |
| Sub | `/Sys_SUn_FlagUnloadPlan` | `std_msgs/Float64` | `data == 1.0` 触发预校验（卸载点分布+航路点计算，只算不发轨迹） |
| Sub | `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | `x/y` = 卡车近点（base 系，m）；`w` = RTK 方位角 α（deg，北0顺时针） |
| Sub | `/truck_center_unloadpoint_2` | `geometry_msgs/Quaternion` | `x/y` = 卡车远点（base 系，m），用于动态卸载点选取（可选，未收到则回退单点模式） |
| Sub | `/Sys_SUn_BucketNumber` | `std_msgs/Float64` | `data` = 当前装载斗数，决定选取最远/中间/最近卸载点 |
| Sub | `/joints_angle` | `geometry_msgs/Quaternion` | `x/y/z` = boom/arm/bucket 关节角（deg） |
| Sub | `/heading2swing_topic` | `std_msgs/Float32` | `data` = swing 关节角（deg） |
| **Pub** | **`/RefDeviceTraj_Unload`** | **`geometry_msgs/Pose`** | **`Orientation.x/y/z/w` = swing/boom/arm/bucket（deg）；`Position.x` 1=执行中 / 0=阶段结束** |
| Pub | `/Swing_topic` | `geometry_msgs/Point` | `x` = swing（deg），由 `/heading2swing_topic` 实时转发 |
| Pub | `/RealBktPosXYZ` | `geometry_msgs/Point` | 铲斗齿尖坐标（base 系，m），由实时关节角经运动学 FK 计算 |
| Pub | `/Sys_RUn_UnloadEndAngle` | `geometry_msgs/Pose` | 卸载轨迹最终点（WP6/卸载点）关节角：`Orientation.x/y/z/w` = swing/boom/arm/bucket（deg）（latched，航路点定稿后发布一次） |
| Pub | `/Sys_unloadPlanFinish` | `std_msgs/Float32` | 预校验完成标志：**1=卸载点分布+航路点计算完成且判定可达**；**0=不可达/计算失败/初始**（latched，由 `/Sys_SUn_FlagUnloadPlan` 触发） |
| Pub | `/Sys_RUn_FlagUnloadExcuteFinish` | `std_msgs/Float64` | 卸载成功完成标志：**1=卸载轨迹成功执行完成**；**0=执行期间 / 跳出卸载阶段 / 异常中止 / 外部撤销**（latched） |
| Pub | `/UPFeasible` | `std_msgs/Bool` | 卸载点可达性校验结果（latched） |
| Pub | `/loadTraj_maxBoom` / `/loadTraj_maxZ` | `std_msgs/Float32` | 航路点动臂最大角度 / 齿尖最高 z（latched） |
| Pub | `/Sys_RUn_UnloadPointSequence` | `std_msgs/Float32MultiArray` | **离线轨迹序列（适配原程序节点，后续可能移除）**：预校验计算成功后发布一次（2000×5 列优先 reshape 一维；row0 五列全 1=成功标志；row1..1999 前 4 列 swing/boom/arm/bucket（deg）、第 5 列段标志；航路点 PCHIP 插值到实际时长，不足补末点）（latched，由 `/Sys_SUn_FlagUnloadPlan` 触发） |
| Pub | `/UT_PlanningPulse_topic` | `std_msgs/UInt32` | 功能安全状态字（位段结构见下） |
| Pub | `dump_viz/*` | `visualization_msgs/MarkerArray` | 卡车/臂架/航路点/轨迹/状态文本（受 `visualization/enable` 总开关控制）。节点运行期间**持续发布**：空闲态臂架跟随实时反馈，执行态臂架跟随指令帧 |

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
| WP5 | 卸载过渡点 | `swing/boom/arm` 到位 | `swing5 = unload_swing`；`boom5 = boom4`（保持不下降）；`arm5` 按 `p5_arm_cof` 混合到卸载点 |
| WP6 | 终点 | 铲斗翻转卸料 | `= unload_joint`（IK 解算结果） |

**3 段规划**：

| 段 | 区间 | 主导关节 | 完成容差 |
|---|---|---|---|
| **Seg0** | WP1→WP4 | 两阶段策略（见下） | swing 5° / boom 5° + 高度门控 |
| **Seg1** | WP4→WP5 | swing 回转 | swing 5° |
| **Seg2** | WP5→WP6 | arm+bucket 卸载 | bucket 单向判据（实际≥规划即完成） |

段时间由主导关节速度决定（`max(|Δjoint|/vel_dps)`）。

### 命名对应

生成器内部命名 `WP1/WP4/WP5/WP6`（历史遗留），节点日志按索引打印为 `WP1/WP2/WP3/WP4`。对应关系：日志 WP2=厢上过渡点（内部 WP4）、日志 WP3=卸载过渡点（内部 WP5）、日志 WP4=终点（内部 WP6）。下文按**日志命名** WP1~WP4 描述。

### 逐点设计详解

**WP1 起点（挖掘终止点）**
- swing/boom/arm 直接取实时反馈 `current_joint_`（终点锚定思想：起点取实时值）；
- bucket 姿态修正：若 `boom+arm+bucket ≥ bucket_attitude_target_deg(-180°)` → `bucket = attitude − boom − arm`，并 clamp 到 `bucket_limit[-170, 30]`；否则保持原值；
- 切线=0（端点起停静止），`t=0`。

**WP2 厢上过渡点 / 入厢点**
- xy：`ComputeMiddleUpXY()` 回转圆策略（见下节）；
- z（IK 目标高）：`truck_top_height + wp4_height_bias(1.5)`，其中 `truck_top_height = center_z − truck_height_offset(0.5)`（动态 RTK 框高）；
- IK：`swing_center_inverse_by_bucket_angle`，`bucket_angle = wp4_bucket_angle_deg(25°)`，`bucket_tooth_length_override = 0`（目标点=斗杆-铲斗**铰接点**而非齿尖）→ 解出 swing/boom/arm；
- bucket：`AdjustBucketByAttitude(bo, ar, bk1(WP1 的 bucket), attitude, limits)`，基于 WP1 bucket 保持卷收连续；
- swing 展开：`sw2 = sw1 + SignedShortestDiff(sw1, sw_ik)`（与 WP1 差 ≤180°）；
- **切线=0**（门控停止点 `bucket_clear_seg_idx_+1`）→ 到 WP2 速度归零，等待铲斗高度门控；
- Seg0 时长 = `max(|Δboom|/seg0_boom_vel_dps, |Δswing|/swing_vel_dps)`；执行走 Seg0 两阶段策略（非纯插值）。

**WP3 卸载过渡点 / swing 到位点**
- swing = 卸载方位（相对 WP2 展开）→ 回转已到位；
- boom = WP2.boom（保持不下降）；arm = `p5_arm_cof(0.5)·WP2.arm + (1−cof)·unload.arm` 中点混合；
- bucket = 姿态修正（基于 WP2 bucket），且 `bk3 = max(bk3, bk2)` **防铲斗倒退/张斗**；
- 切线 = Catmull-Rom 内部点（非 0，平滑通过）；
- Seg1 时长 = `max(|Δswing|/swing_vel_dps, |Δboom|/unload_boom_vel_dps, |Δarm|/unload_arm_vel_dps)`。

**WP4 终点（卸载点）**
- 完整取 `unload_joint`（`CheckReachable` 的 IK 解），swing 用展开值；
- 切线=0（端点停止）；
- Seg2 时长 = `max(|Δbucket|/unload_bucket_vel_dps, |Δboom|/unload_boom_vel_dps, |Δarm|/unload_arm_vel_dps)`。

### 公共后处理

- 限位校验 `ValidateWaypointsJointLimits`：逐点 clamp，修正量 ≤5° 继续(WARN)、>5° abort(ERROR)；
- `UnwrapSwingSequence`：swing 序列展开防 0°/360° 跳变；
- `segment_times_` = `t_array` 差分（≥最小段时长）；
- `ComputeWaypointTangents`：Catmull-Rom 内部切线，**门控点(WP2)与两端点切线强制 0**（速度整形）；
- 各段 PCHIP 插值（保形无超调）。

### 遗留参数（已清理）

旧 6 航路点版的中间过渡点参数（`wp3_swing_step_deg`、`p3_*` 系列、`max_boom_lift24_deg`）与被动态框高取代的 `online_plan/truck_top_height_m` 已从 yaml、`LoadParams` 与 `WaypointParams` 中全部移除，不再有"调了不生效"的隐患。

## 插值方法：PCHIP（保形分段三次 Hermite，无超调）

段内公式（`s = (t - t_i) / T ∈ [0,1]`）：

```
q(s) = h₀₀·q_i  +  h₁₀·T·v_i  +  h₀₁·q_{i+1}  +  h₁₁·T·v_{i+1}
h₀₀(s) =  2s³ - 3s² + 1      h₁₀(s) = s³ - 2s² + s
h₀₁(s) = -2s³ + 3s²           h₁₁(s) = s³ - s²
```

起终切线经 **Fritsch-Carlson 单调性修正**，保证插值不产生超调。详见 [cubic_hermite_interpolator.h](include/dump_trajectory_planner/cubic_hermite_interpolator.h)。

## 段间衔接：飞越 vs 停稳（在线执行）

整段卸载要求**连续平滑、中途不停**，故在线执行按航路点性质区分两类衔接（`IsFlythroughWaypoint`）：

| 航路点 | 类型 | 起点位置 | 起点速度 `v_start` | 完成判据 |
|:---:|:---|:---|:---|:---|
| WP1 | 停止点（首点） | 实时关节角 | 实际速度估计（≈0） | — |
| WP2 | 停止点（门控） | 实时关节角 | 实际速度估计（≈0） | 实时到位+确认保持+铲斗高度门控 |
| **WP3** | **飞越点**（内部非门控） | **标称航路点** | **`wp_tangents_[2]`**（与上段 `v_end` 同源） | **指令播完即推进**（不停稳） |
| WP4 | 停止点（末点） | — | — | 实时到位+确认保持 |

**飞越点（WP3）三处保证 C¹ 连续、指令不跌零**：
1. **位置连续**：下一段起点取**标称航路点**而非滞后实测值，避免指令回跳；
2. **速度连续**：下一段 `v_start = wp_tangents_[2]`，与上一段 `v_end` 同源（Catmull-Rom 切线），整段指令速度不在 WP3 处跌零；
3. **无切换间隙**：段完成时**同一帧内**立即规划并发布下一段首帧（不经 `kPlanNextSegment` 中转停发），消除 1~2 帧的指令停顿。

> 停止点（WP1/WP2/WP4）仍用实时反馈起点 + 到位确认，门控点 WP2 允许停稳等待铲斗跨越卡车框（物理安全）。飞越逻辑仅作用于内部非门控点，与离线序列 `PublishOfflineSequence` 的连续 PCHIP 拼接行为一致。

## WP4 入厢点（回转圆与厢矩形交点策略）

`ComputeMiddleUpXY()` 照 MATLAB `calc_entry_point_circle_strategy` 实现：

1. **回转圆**：以回转中心 O=(0,0) 为圆心，半径 `R = wp4_R_coff · |dig_end| + (1 - wp4_R_coff) · |unload_point|`；
2. **矩形变换**：卡车位姿 `(center, β)` 给定一个矩形厢，将回转圆变换到厢本地坐标系；
3. **铲斗宽度外扩（配置空间 / Minkowski 膨胀）**：把卡车框每边**向外膨胀** `outset = 0.5·bucket_width + bucket_wall_clearance`，等价于将有宽度的铲斗退化为其宽度中心线（回转圆上的铰接点）、障碍物膨胀铲斗半宽；于是入厢点 = **铲斗边缘刚好接触真实卡车框**时的中心位置（而非把铲斗当点、边缘会超框刮帮）；`bucket_width=0` 即关闭外扩退回点模型；
4. **圆-矩形交点**：对**外扩后**矩形 4 条边分别求交点（最多 8 个）；
5. **选交点**：`rot_dir` 由 `dig_end swing → unload swing` 的最短路径自动判定；取沿 `rot_dir` 方向从 `dig_end swing` 出发角度差最小的交点作为 `p_entry`；
6. **fallback**：无交点时，以外扩矩形 4 条边上距离 O 最近的点为方向，在回转圆上取对应圆上投影作为 `p_entry`；
7. **退化**：`R < 1e-6` 或方向量归零时，回退到卸载点 xy。

> RViz debug 层同时画出原始卡车框（白）与外扩后的**膨胀禁区**（橙，标注 `outset`），入厢点落在橙框上（= 铲斗边缘贴真实框）。


> 卡车航向 β 优先由「近点→远点」连线方向决定：两点均为 base/下车体系坐标，`β = atan2(far.y−near.y, far.x−near.x)`（归一到 [0,360)，forward=(cosβ,sinβ) 由 near 指向 far）；远点 `/truck_center_unloadpoint_2` 未收到或两点重合时回退 RTK 方位角换算 `β = (180° − α) mod 360°`（北180/东90/南0/西270 逆时针增大；与 truck_dump_planner 6 方向闭环验证一致。曾误用 `+α` 导致卡车朝向镜像、厢体画偏）。

## Seg0 两阶段策略（核心）

Seg0（WP1→WP4）采用两阶段策略，兼顾效率与液压友好：

### 启用判据

仅当 `boom 抬升量 ≥ seg0_min_step_factor × seg0_switch_threshold_deg` 时启用两阶段；否则（含 boom 持平/下降，防重力助势下落）退化为单段 PCHIP。

### 运动分配

| 关节 | **Phase1**（回转 + boom 抬升段） | **Phase2**（boom 到位后） |
|---|---|---|
| **boom** | **直接阶跃到"中间角" `mid = WP1.boom + cof·(WP4.boom − WP1.boom)`（`cof=seg0_boom_pchip_start_cof∈[0,1]`）**，随即从 mid **立即并行 PCHIP** 抬到 WP4.boom（不等 swing 到位）；播完保持 | **时间 PCHIP 从实测 boom 收尾到 WP4**（此时差≈0） |
| **arm** | **条件打出（deploy）**：boom 到位（连续帧确认）后经 `IsArmDeploySafe()` 路径级预检放行，则以 `seg0_arm_deploy_dps` **限速追赶** WP4.arm（姿态保持下此动作**抬升齿尖 ~1.8m**）；预检不通过则冻结起点角 | PCHIP 起点 arm **直取 WP4.arm**，故 arm 在 Phase2 内恒平（deploy 已走完时无指令跳变；未放行时本帧一次打到 WP4） |
| **swing** | 阶跃到"卡车附近" `WP4.swing − sign×seg0_swing_offset_deg`，阀口**饱和回转** | PCHIP 续接到 WP4.swing，起点用**实测值**（避免指令跑在实测前面播完、swing 带大滞后进入卸载），速度取实测值保斜率匹配 |
| **bucket** | **每帧按实测 boom/arm 反馈实时算参考角**维持姿态（单向收斗） | PCHIP 到 WP4.bucket |

> ⚠️ **cof 与厢帮安全**：Phase1 全程 + Phase2 宽限期内不判高度门控，回转途中 boom 只到 `mid`。若 `cof` 太小、`mid` 高度不足以越过厢帮，回转扫过时齿尖可能刮到车厢侧板；卸载点越近/起始位姿越低，`cof` 应调越大（`cof=1` 即一步阶跃到 WP4 高度）。

### Phase1 bucket 实时姿态保持

```
bucket_hold = bucket_attitude_target_deg − boom_实测 − arm_实测
姿态角(用参考bucket评估) ≥ 目标 → 输出 bucket_hold(裁剪到限位)；否则保持上一帧
```

> 注：deploy 期间 arm 在变，该式每帧重算 → bucket 随 arm 联动保姿态，铲斗不会因 arm 打出而张开。

### Seg0 斗杆条件打出（arm deploy）

目的是消除 Phase1 的 arm 空窗期（旧设计 arm 全程冻结、整段行程被甩到 Phase1→Phase2 切换瞬间硬阶跃）。三重保护：

1. **触发时机**：`|实测boom − WP4.boom| ≤ seg0_boom_deg` 且连续 `seg0_confirm_frames` 帧确认（防单帧反馈噪声与臂架液压振荡误触发）。可选 `seg0_arm_deploy_on_boom_arrived=false` 改为"即将到位"（`≤ seg0_boom_soon_deg`）即打出、与 boom 举升并行，但 arm 外伸也是高压动作，同帧抢泵流量风险高（默认关闭）。
2. **放行判据 `IsArmDeploySafe()`**：对 arm∈[当前指令角, WP4.arm] 采样 FK（boom 取实测，bucket **复刻 Phase1 的逐帧单向姿态保持**），取全程**最低**齿尖高 `z_min_path` 与**最大**半径 `r_max_path`，满足其一即放行：
   - `z_min_path > TruckTopHeight() + bucket_clear_margin_m`（全程不破门控）；
   - **半径豁免**：`r_max_path < 外扩厢帮近边`（齿尖未进入厢帮方位，高度无关）。

   > ✅ **打出 arm 是抬升齿尖的（姿态保持前提下）**：Phase1 每帧把 bucket 锁在 `A = bucket_attitude_target_deg(−180°)`，而 `sin(−180°)=0` → **斗齿对高度零贡献**，于是
   > ```
   > z = Lb·sin(boom) + La·sin(boom+arm)，  ∂z/∂arm = La·cos(boom+arm)
   > 变号点：arm = −90° − boom   （boom=40.5° → −130.5°；boom=20° → −110°）
   > ```
   > 常规工况（arm > −90°−boom）打出 arm **单调抬高齿尖**：boom=40.5°、arm −117.6°→−60° 时 z 从 1.813 m 升到 **3.605 m（+1.79 m）**，而半径反而从 3.67 m 降到 5.68 m 以内（斗齿回卷）。故 deploy 不只是省时间，**还能主动弥补跨越厢帮的高度余量**（比 gate boost 的 +3° boom 约 +0.31 m 有效得多）。
   >
   > ⚠️ **建模陷阱（已踩）**：若把 bucket **固定为实测值**采样，会得到完全相反的结论——z 先降到谷底 1.309 m（arm≈−85°）再升到 1.614 m，终点差达 **2.0 m**，从而把有效的抬升动作误拦死。两种模型的分岔根源就是 `Lt·sin(A)` 是否被抵消。
   >
   > ⚠️ **仍保留「路径最低值」而非终态值**：低 boom（<25°）或过度卷收未锁姿态（`attitude < A`，bucket 冻结）时确有小幅下探（boom=20° 时 −0.145→−0.407 m）。若只校终态，途中跌破门控会触发 `StartGateBoost` 的 arm 反向内收（`Δarm = −Δboom` 保姿态），与打出动作**互相对抗**（反复振荡 / boost 耗尽 abort）。
3. **执行方式**：限速追赶而非硬阶跃 —— 指令连续、无阀口冲击；因阀口本就饱和，时间收益与硬阶跃几乎相同。

配套联动：
- **Phase1→Phase2 跳转**改为 `(boom_arrived && arm_ready) || p1_timeout`，其中 `arm_ready` = deploy 未启用 / 未触发 / 已走完（预检拦下时**不阻塞**跳转，交给 Phase2 收尾）；
- **Phase1 超时预算**：boom 与 swing 并行取 `max`，arm deploy 属 boom 到位后的**串行追加**用 `+`（用 max 会预算不足、在 arm 追赶途中误判超时）；
- **Phase2 门控宽限期自适应** `seg0_phase2_grace_eff_sec_`：deploy 已走完 → 用配置值 `seg0_phase2_gate_grace_sec`（arm 恒平且打出本身已抬升齿尖）；deploy 未放行 → 拉长到 `max(配置值, Phase2 时长)`，因为此时预检已判定路径存在低于门控的下探、Phase2 首帧阶跃会重现它（高度安全由段边界 `AdvanceToNextSegment` 必判兜底）；
- **状态复位**：`PlanCurrentSegment` 中每轮卸载都清零 `deploy_active/done/boom_arrived_count/grace_eff`，否则第二轮会带着上一轮的 `done=true` 跳过打出。

### 切换条件（boom 到位，不再用 swing 串行闸门）

- `|实测boom − WP4.boom| ≤ seg0_boom_deg(5°)` **且** `arm_ready` → 切 Phase2；
- swing **不作串行闸门**（swing 是真机最慢关节，实测 ~5°/s，串行会让 arm 全程冻住、整流程易超时），在 Phase2 内靠"实测起点 + 时长含剩余行程"收敛；
- 高度门控**不阻塞 Phase2 入口**（深卷 arm 下该位姿齿尖必然低于阈值，阻塞会死锁），仅 WARN，实际判定交给 Phase2 宽限期后的逐帧监视与段边界兜底；
- **Phase1 独立超时**（`(max(boom行程/seg0_boom_vel_dps, swing行程/seg0_phase1_swing_dps) + arm行程/seg0_arm_deploy_dps) × factor`）→ 强制进 Phase2。

### Phase2 起点斜率衔接（防阀反向冲击）

`v_start = VelocityEstimator::Estimate()`（arm 分量置 0，因 arm 已到位、Phase2 内 Δarm=0），并对时长施加 `p2_time ≤ 3Δ/|v|` 的 Fritsch-Carlson 可衔接约束（起/终点切线均校）。

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
- **N 帧确认防抖**：Seg0 用 `seg0_confirm_frames`，其余 `segment_confirm_frames=3`（两者当前均为 3；旧值 8 用于覆盖臂架 0.5~2Hz 液压振荡模态，实测过于保守、拖慢推进，已下调）。
- **超时兜底**：`base_time = max(标称段时长, 实际规划时长)`，超时 = base × `segment_timeout_factor(2.0)`。Seg0 若门控未达标 → `kEnterGate`（不强推，安全第一）；否则强制推进。
- Seg0 高度门控未达标时 `ComputeSegError` 直接返回 `1e6` 阻止完成。

## 高度门控与 boost 重试

- **触发点**（已从串行闸门降级为并行监视器 + 段边界兜底）：
  - ① **Seg0 Phase2 逐帧监视**（仅 `current_seg_idx_ == 0` 生效）：宽限期 `seg0_phase2_grace_eff_sec_` 过后，每帧判齿尖高度，不足即 `kEnterGate`；
  - ② **段边界兜底**：`AdvanceToNextSegment` 中 `current_seg_idx_ == bucket_clear_seg_idx_(0)` 时**必判一次**，通过则标记 "gate pre-cleared"（不清空速度历史、不停顿）；
  - ③ **Phase1 入口不阻塞**：Phase1→Phase2 切换时高度不足仅 WARN（深卷位姿下齿尖必然低于阈值，阻塞会死锁）。
  - ⚠️ **Seg1/Seg2 全程无高度复检**（条件①硬编码 `seg_idx == 0`），齿尖跨越近侧厢帮的安全完全押在 WP4/WP5 的规划高度上。
- **判据**：齿尖 FK 高度 `bucket_height_ > TruckTopHeight() + bucket_clear_margin_m(0.5)`，其中 `TruckTopHeight() = truck_center_z − truck_height_offset(0.5)` 为**动态框高**（随 RTK 高度变化）。⚠️ **margin 必须低于 WP2 齿尖可达裕量**（本工况约 1.4m）：曾设 1.8 使阈值 2.21m 高于全部航路点齿尖（最高 1.81m），高度门控与 Seg0 Phase1→Phase2 切换双双死锁，boost 耗尽必然中止。
- **等待期**：恒定发布**锁存姿态** `gate_hold_cmd_`（不跟随反馈，避免液压保压沉降被逐帧追认成下沉指令）。
- **boost 重试**：`gate_timeout_sec(2s)` 超时 → boom+`gate_boost_boom_deg(3°)`、arm 反向联动(Δarm=-Δboom)保姿态；最多 `gate_max_boost_count(4)` 次，耗尽或 boom 到上限 → abort。
  - ⚠️ **boost 与 arm deploy 互斥**：boost 的 arm 反向内收与 deploy 的 arm 外伸方向相反，若同时生效会形成双向对抗/振荡。已由 `IsArmDeploySafe()` 的**路径最低值**判据（而非终态值）+ Phase2 宽限期自适应两道拦住。

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
| `waypoint/*` | `wp4_height_bias`、`truck_height_offset`、`wp4_bucket_angle_deg`、`p5_arm_cof`、`bucket_limit_*_deg`、段速度（`seg0_boom_vel_dps`/`swing_vel_dps`/`seg0_arm_vel_dps`/`unload_boom_vel_dps`/`unload_arm_vel_dps`/`unload_bucket_vel_dps`）、`truck_box_length/width`、`wp4_R_coff`、**`bucket_width`、`bucket_wall_clearance`**（铲斗宽碰撞外扩） | 航路点生成（姿态角目标已并入 `online_plan/bucket_attitude_target_deg`）|
| `online_plan/*` | `seg0_swing_deg/boom_deg`、`seg1_swing_deg`、`seg2_bucket_deg`、`seg0_confirm_frames`、`segment_confirm_frames`、`segment_timeout_factor`、`bucket_clear_margin_m`、`bucket_clear_seg_idx`、`gate_timeout_sec`、`gate_boost_boom_deg`、`gate_boost_tolerance_deg`、`gate_max_boost_count`、`waypoint_clamp_abort_deg`、**`seg0_switch_threshold_deg`、`seg0_phase1_swing_dps`、`seg0_min_step_factor`、`seg0_swing_offset_deg`、`seg0_boom_pchip_start_cof`、`bucket_attitude_target_deg`**、**`seg0_phase2_gate_grace_sec`、`seg0_arm_deploy_enable`、`seg0_arm_deploy_dps`、`seg0_arm_deploy_path_samples`、`seg0_arm_deploy_on_boom_arrived`、`seg0_boom_soon_deg`、`seg0_arm_deploy_radius_exempt`**（斗杆条件打出）、**`unload_selector_enable`、`unload_candidates_count`、`unload_bucket_far_count/mid_count/near_count`** | 在线规划、Seg0 两阶段与动态卸载点选取 |
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
| 下游速率限制器 | ⚠️ **未确认** | 决定 swing 阶跃/boom 限速指令是否被下游平滑（上实机前最需确认） |
| 多执行器流量竞争 | ⚠️ 架构级未解 | 未做泵流量感知调度（路线 B） |

**核心结论**：设计完整、理论自洽，但缺乏实机/仿真数据验证。上实机前最需确认下游是否有速率限制器。

## TODO

- [x] ~~实现回转角 0°/360° 包裹处理~~（已完成，见上节方案）
- [x] ~~WP3 swing 中点 0°/360° 跨界方向错误~~（生成阶段 swing 链沿最短路径展开）
- [x] ~~航路点关节限位校验~~（`ValidateWaypointsJointLimits`，超限 clamp + 告警）
- [x] ~~段完成持续确认防抖~~（`segment_confirm_frames` 参数化）
- [x] ~~门控高度不达标死等~~（超时自动 boom 提升重试，次数耗尽终止）
- [x] ~~速度估计双话题混写相位偏差~~（swing/joints 两路历史分离，各自时间戳差分）
- [x] ~~Seg0 两阶段策略~~（swing 阶跃回转 + boom 阶跃到可配中间角 `seg0_boom_pchip_start_cof`（与 swing 解耦）+ 高度门控 + Phase2 时间 PCHIP 收尾；旧"boom 随 swing 进度插值抬升"与更早的"boom 阶跃中点保持"方案均已废弃）
- [x] ~~RViz 航路点可视化按硬编码下标错位~~（4 航路点版入厢点误取 `waypoints[3]`=终点、`else if(i==3)` 配色死分支；已修：入厢点取下标 1、配色按语义角色、标签沿用日志索引命名 WP1..WP4）
- [x] ~~归一化段误差判据~~（`max(err/tol)≤1.0`，逐关节 AND）
- [x] ~~Phase2 起点切线衔接~~（`v_start=Estimate()` + 斜率可衔接约束）
- [x] ~~bucket 实时姿态保持~~（每帧按实测 boom/arm 反馈计算参考 bucket）
- [x] ~~动态卸载点选取~~（近点↔远点 21 候选 IK 验证 + 按斗数三档映射 + 三层降级）
- [x] ~~Seg0 斗杆空窗期~~（新增 **arm 条件打出 deploy**：boom 到位带连续帧确认 → `IsArmDeploySafe()` 路径级预检（全程最低齿尖高过门控 **或** 最大半径未进厢帮方位则放行）→ `seg0_arm_deploy_dps` 限速追赶；配套跳转判据 `arm_ready`、超时预算串行追加、Phase2 宽限期自适应、每轮状态复位）
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
