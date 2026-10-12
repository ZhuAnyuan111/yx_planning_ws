# 挖掘机规划仿真体系架构契约

> 本文档是仿真体系各层之间的**唯一契约**。任何 plant / scenario / planner / adapter 的
> 话题、单位、frame 变更都必须先落到这里，再改代码。

## 1. 系统分层

```
+------------------------------------------------------------------+
| L6 入口层    launch/*.launch      run_batch.py                    |
+------------------------------------------------------------------+
| L5 度量层    kpi_collector.py     record_cycle.sh                 |
+------------------------------------------------------------------+
| L4 编排层    excavator_system_decision_node (C++)   environment_stub.py   phase_trigger.py |
+------------------------------------------------------------------+
| L3 桥接层    traj_to_urdf_adapter.py   sy200_urdf_control         |
+------------------------------------------------------------------+
| L2 场景层    scenario_manager.py   scenarios/*.yaml               |
+------------------------------------------------------------------+
| L1 被控对象  exca_plant_sim.py     config/plant.yaml              |
+------------------------------------------------------------------+
| L0 资产      urdf/exca870_single.urdf   meshes/   kinematics      |
+------------------------------------------------------------------+
```

## 2. 坐标 / 单位 / 角度约定

| 项目 | 约定 |
|---|---|
| Frame | `base_link` (URDF) 与 `base` (planner marker) 通过静态 TF 对齐，见 `tf_base_link_to_base` |
| 长度 | 米 (m) |
| 关节角 | **planner ↔ plant ↔ adapter**：deg；**URDF/joint_states**：rad；转换只在 `traj_to_urdf_adapter` 与 `sy200_urdf_control` 中发生 |
| Swing 方向 | 内部连续（不 wrap），发布时 `wrap360` 到 `[0, 360)` deg |
| 关节顺序 | `orientation.x=swing, y=boom, z=arm, w=bucket`（`RefDeviceTraj_*`）<br/>`joints_angle.x=boom, y=arm, z=bucket, w=0`（`Quaternion`） |

## 3. 话题契约表

### 3.1 反馈（plant → planners / adapter）

| 话题 | 类型 | 单位 | 发布者 | 订阅者 | 说明 |
|---|---|---|---|---|---|
| `/joints_angle` | `geometry_msgs/Quaternion` | deg | plant | dump / reset / adapter | `x=boom, y=arm, z=bucket` |
| `/heading2swing_topic` | `std_msgs/Float32` | deg | plant | dump / reset / adapter | `[0, 360)` |
| `/Swing_topic` | `std_msgs/Float32` | deg | plant | truck_dump | 与 `heading2swing_topic` 同值，兼容 truck_dump 命名 |
| `/RealBktPosXYZ` | `geometry_msgs/Point` | m | dump planner | reset | 铲斗齿尖 base 系坐标，由 `dump_trajectory_planner/status_reporter` 用其精确运动学发布；**plant 不发布**（避免双发布者冲突）。standalone reset/dig 无此话题时，reset 自动禁用"物理提前结束"判定 |

### 3.2 场景静态输入（scenario → planners）

| 话题 | 类型 | 单位 | 订阅者 | 语义 |
|---|---|---|---|---|
| `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | `x,y,z` m / `w` deg | dump | 卡车近点 + RTK 方位 |
| `/truck_center_unloadpoint_2` | `geometry_msgs/Quaternion` | m | dump | 卡车远点（动态卸载点选取） |
| `/truck_center_point` | `geometry_msgs/Quaternion` | m | reset | 卡车中心 x/y（reset 内部 `atan2(y,x)+180` 得到 swing 角） |
| `/Sys_SUn_BucketNumber` | `std_msgs/Float64` | — | dump / decision | 当前装载斗数（全循环模式由 decision 发布） |
| `/GP_dig_joints` | `geometry_msgs/Quaternion` | deg | reset | 下一挖掘点关节角目标（dig-cycle 复位模式），`x=swing, y=boom, z=arm, w=bucket` |
| `/clean_digPoint` | `geometry_msgs/Quaternion` | deg | reset | 搭台模式目标关节角（同上格式） |
| `/bucket_terrain_delta_position` | `geometry_msgs/PointStamped` | m | reset | 地形高度桩 `.point.z` |
| `/terrain_profile` | `nav_msgs/Path` | m | dig (stub) | 地形剖面（stub 桩可为空） |
| `truck_pose` | `geometry_msgs/Pose` | m/rad | truck_dump | 卡车 pose（私有命名，truck_dump 独有） |

### 3.3 命令（planners → plant）

| 话题 | 类型 | 单位 | 发布者 | 语义 |
|---|---|---|---|---|
| `/RefDeviceTraj_Unload` | `geometry_msgs/Pose` | deg | dump | `position.x=1` 执行中，`0` 结束 |
| `/RefDeviceTraj_Reset` | `geometry_msgs/Pose` | deg | reset | 同上 |
| `/RefDeviceTraj_Dig` | `geometry_msgs/Pose` | deg | dig (stub) | 同上 |
| `/RefDeviceTraj_Dump` | `geometry_msgs/Pose` | deg | (兼容旧命名) | plant 仍订阅，仲裁优先级最低 |

plant 内部按 `cmd_priority` 与 phase 门控仲裁；同帧多源时按优先级取一，避免抖动。

### 3.4 激活 / 完成

| 话题 | 类型 | 值 | 发布者 | 订阅者 | 语义 |
|---|---|---|---|---|---|
| `/Sys_SeqAction` | `std_msgs/Float64` | `4.0` | decision / scenario / phase_trigger | dump | 触发卸载 |
| `/Sys_SRe_FlagResetExcute` | `std_msgs/Float64` | `1.0` = dig-cycle / `2.0` = bench / `0.0` = exit | decision / scenario / phase_trigger | reset | 触发复位 |
| `/Sys_SDi_FlagDigExcute` | `std_msgs/Float64` | `1.0` | decision / phase_trigger | dig (stub) | 触发挖掘 |
| `/Sys_SUn_FlagUnloadPlan` | `std_msgs/Float64` | `1.0` | decision / scenario | dump | 预校验（离线） |
| `/Sys_SGp_FlagGpPlan` | `std_msgs/Float32` | `1.0` | decision | environment_stub | 整地规划请求 |
| `/Sys_SLe_FlagGroundLevelExcute` | `std_msgs/Float64` | `1.0` | decision | (预留) | 整平执行 |
| `/Sys_RUn_FlagUnloadExcuteFinish` | `std_msgs/Float64` | `1.0` (latched) | dump (status_reporter) | phase_trigger / kpi | 卸载完成 |
| `/Sys_RUn_FlagDigFinish` | `std_msgs/Float64` | `1.0` (latched) | dig (stub) | phase_trigger / kpi | 挖掘完成 |
| `/RefDeviceTraj_Reset.position.x` | (Pose 字段) | `1→0` 下降沿 | reset | decision / phase_trigger | reset 完成 |
| `/RefDeviceTraj_Dig.position.x` | (Pose 字段) | `1→0` 下降沿 | dig (stub) | decision / phase_trigger | dig 完成 |
| `/RefDeviceTraj_Unload.position.x` | (Pose 字段) | `1→0` 下降沿 | dump | decision | dump 完成 |

### 3.5 Decision 节点输入（environment_stub / plant / scenario 提供）

| 话题 | 类型 | 发布者 | 语义 |
|---|---|---|---|
| `/truck_pose_topic` | `geometry_msgs/PoseStamped` | environment_stub | `orientation.w` = 事件码 (1=entry, 2=entering, 3=loading, 4=leave) |
| `/Sys_RGp_FlagGpPlanFinish` | `std_msgs/Float64` | environment_stub | 整地规划完成 (非零) |
| `/Sys_RUn_UnloadPointSequence` | `std_msgs/Float32MultiArray` | dump planner | 卸载点序列 (≥10000 元素, data[0]=finish) |
| `/FlagSmoothFinish` | `std_msgs/Bool` | environment_stub | 整平完成 |
| `/FlagCleanFinish` | `std_msgs/Float32` | environment_stub | 清场完成 (0.0=未完成) |
| `/FlagWalkFinish` | `std_msgs/Bool` | environment_stub | 行走完成 |
| `/FlagTruckFull` | `std_msgs/Bool` | environment_stub | 卡车满载 |
| `/FlagUpperLowerAlignmentFinish` | `std_msgs/Bool` | environment_stub | 上下对齐完成 |
| `/unman_params_set` | `geometry_msgs/Pose` | environment_stub | 无人参数设置 (w=1.0 触发 bucket_number 重置) |
| `/Mpc_Switch` | `std_msgs/Bool` | environment_stub | 系统启动 (true=开始卡车交互) |
| `/Mpc_Switch2` | `std_msgs/Bool` | environment_stub | 搭台请求 (false=不触发) |
| `/Sys_SCo_Traj` | `geometry_msgs/Pose` | environment_stub | 参考回转角 (`orientation.x` = deg) |
| `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | scenario_manager | 卡车卸载点 |
| `/joints_angle` | `geometry_msgs/Quaternion` | plant | 关节角反馈 |
| `/heading2swing_topic` | `std_msgs/Float32` | plant | 回转角反馈 |

### 3.6 Decision 节点输出

| 话题 | 类型 | 语义 |
|---|---|---|
| `/Sys_SeqAction` | `std_msgs/Float64` | 卸载执行命令 (4.0) |
| `/Sys_SDi_FlagDigExcute` | `std_msgs/Float64` | 挖掘执行命令 (1.0) |
| `/Sys_SRe_FlagResetExcute` | `std_msgs/Float64` | 复位执行命令 (1.0) |
| `/Sys_SUn_FlagUnloadPlan` | `std_msgs/Float64` | 卸载规划命令 (1.0) |
| `/Sys_SGp_FlagGpPlan` | `std_msgs/Float32` | 整地规划命令 (1.0) |
| `/Sys_SLe_FlagGroundLevelExcute` | `std_msgs/Float64` | 整平执行命令 |
| `/Sys_SeqPlan` | `std_msgs/Float64` | 规划序号 |
| `/Sys_SeqTruck` | `std_msgs/Float64` | 卡车序号 |
| `/Sys_SUn_BucketNumber` | `std_msgs/Float64` | 当前装载斗数 |
| `/Sys_TruckLoadNumber` | `std_msgs/Float32` | 卡车装载数 |
| `/digger_loading_topic` | `geometry_msgs/PoseStamped` | 装载状态 |
| `/fault_require_takeover` | `std_msgs/Bool` | 故障接管请求 |
| `/system_decision/state_text` | `std_msgs/String` | 状态机文本 (TruckStage:OpState:...) |

### 3.7 可视化 / 度量（旁路）

| 话题 | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/Sys_RefDeviceTraj` | `geometry_msgs/Pose` | adapter | rad，robot1 = 实际反馈 |
| `/Sys_RefDeviceTraj2` | `geometry_msgs/Pose` | rad, robot2 = 规划指令（对比） |
| `/joint_states` | `sensor_msgs/JointState` | `sy200_urdf_control` | rad，`robot_state_publisher` 消费 |
| `/cycle_report` | `std_msgs/String` (JSON) | cycle_sequencer | 每循环耗时 / 阶段耗时 / 失败原因 |
| `/kpi/*` | 多种 | kpi_collector | 循环指标 |
| `/plant/fault` | `std_srvs/SetBool` + 参数 | 测试注入 | 故障开关 |

## 4. 各 planner 反馈依赖清单

| Planner | 必需反馈 | 必需场景 | 激活值 | 完成信号 |
|---|---|---|---|---|
| dump | `/joints_angle`, `/heading2swing_topic` | `/truck_center_unloadpoint{,_2}`, `/Sys_SUn_BucketNumber` | `/Sys_SeqAction=4.0` | `/Sys_RUn_FlagUnloadExcuteFinish=1.0` |
| reset | `/joints_angle`, `/heading2swing_topic`, `/RealBktPosXYZ` | `/GP_dig_joints` 或 `/clean_digPoint`, `/truck_center_point`, `/bucket_terrain_delta_position` | `/Sys_SRe_FlagResetExcute=1.0 或 2.0` | `/RefDeviceTraj_Reset.position.x` 1→0 |
| dig (stub) | `/joints_angle`, `/heading2swing_topic` | `/terrain_profile` (可选) | `/Sys_SDi_FlagDigExcute=1.0` | `/Sys_RUn_FlagDigFinish=1.0` 或 `/RefDeviceTraj_Dig.position.x` 1→0 |
| truck_dump | `/joints_angle`, `/Swing_topic`, `truck_pose` | `planner_activate` | 只发 marker，**不进入闭环控制**，仅作可视化叠加 |

### 4.5 Decision 节点状态机概览

```
TruckInteractionScenario (顶层)
├── TruckStage:
│   IDLE → INITIAL_WAIT_TRUCK_ENTRY → TRUCK_POSITION_CHECK
│       → DIGGING_CYCLE → DISPATCH_TRUCK_LEAVE → WAIT_TRUCK_ENTRY → (循环)
│       → STAGING_PRE_MOVE → STAGING_CYCLE (搭台路径，仿真暂不覆盖)
│
├── DiggingCycleFsm (OperationState):
│   IDLE → UNLOAD_PLAN → UNLOAD_EXECUTE → RESET_EXECUTE
│       → DIG_EXECUTE → UNLOAD_EXECUTE → ... → LEVEL_TRUCK → SUCCESS
│
├── DigWaitFsm:     初始回转对准（真实角 → 卸载点角误差 < 30°）
├── ResetDigWaitFsm: 复位后回转对准
└── TakeoverMonitor: 超时监控 → /fault_require_takeover
```

state_text 格式：`"TruckStage:OperationState:StagingPreMoveState:StagingCycleState"`

environment_stub 通过解析 state_text 自动推进卡车事件序列和整平反馈。

### 4.6 决策 ↔ 轨迹规划：完整循环时序

> 决策节点**不直接下发关节指令**，只发「触发标志」；规划器据触发标志产出
> `/RefDeviceTraj_*` 轨迹喂给 plant，plant 回灌实际状态，规划器再回灌完成标志。
> 以下为一个循环内各阶段实际流动的话题（GP 未接入，其角色暂由 environment_stub +
> scenario_manager 伪造）。

```
① DIG_POINT_SELECTION   (ResetDigWaitFsm / DigWaitFsm)
   决策 -/Sys_SGp_FlagGpPlan=1.0-> env_stub --延迟--> /Sys_RGp_FlagGpPlanFinish=1.0-> 决策
   scenario_manager 早已 latched /GP_dig_joints（下一挖掘点目标）
② RESET_EXECUTE
   决策 -/Sys_SRe_FlagResetExcute=1.0-> reset
   reset 读 /GP_dig_joints + /truck_center_point + /bucket_terrain_delta_position
   reset -/RefDeviceTraj_Reset(x:1→轨迹→0)-> plant & 决策(判完成)
③ DIG_EXECUTE
   决策 -/Sys_SDi_FlagDigExcute=1.0-> dig_stub
   dig_stub 下探→收斗→抬臂 -/RefDeviceTraj_Dig(x:1→0)-> plant & 决策
   同时 /Sys_RUn_FlagDigFinish=1.0 (latched)
④ BOOM_UP_EXECUTE
   决策发 seq_action=5.0；仿真无 MPC 抬臂 → launch 覆盖 boom_up_angle_threshold=3.0 放行
   回转到位判定：/joints_angle + /Sys_SCo_Traj(参考角) vs /heading2swing_topic(实际角)
--- 卡车到位后进入 DIGGING_CYCLE ---
⑤ UNLOAD_PLAN
   决策 -/Sys_SUn_FlagUnloadPlan-> dump（校验卸载点）
   dump -/Sys_RUn_UnloadPointSequence(data[0]!=0, ≥10000 元素)-> 决策
⑥ UNLOAD_EXECUTE
   决策 -/Sys_SeqAction=4.0-> dump（dump 主激活口）
   dump 读 /truck_center_unloadpoint{,_2} + /Sys_SUn_BucketNumber + 实际角
   dump -/RefDeviceTraj_Unload(x:1→0)-> plant & 决策
⑦ LEVEL_TRUCK
   决策 -/Sys_SLe_FlagGroundLevelExcute->（整平）；env_stub 检测 state_text=LEVEL_TRUCK
   --延迟--> /FlagSmoothFinish=1.0-> 决策
⑧ SUCCESS → DISPATCH_TRUCK_LEAVE → 再挖一斗(reset+dig) → WAIT_TRUCK_ENTRY → 下一车
```

### 4.7 决策 ↔ 轨迹规划：按节点接口归属矩阵

| 节点 | 订阅（输入） | 发布（输出） |
|---|---|---|
| **decision** | 见 §3.5 | 见 §3.6 |
| **dump_planner** | `/Sys_SeqAction`(=4.0), `/Sys_SUn_FlagUnloadPlan`, `/truck_center_unloadpoint{,_2}`, `/Sys_SUn_BucketNumber`, `/joints_angle`, `/heading2swing_topic` | `/RefDeviceTraj_Unload`, `/Sys_RUn_UnloadPointSequence`, `/Sys_RUn_FlagUnloadExcuteFinish`, `dump_viz/*`, `/RealBktPosXYZ` |
| **reset_planner** | `/Sys_SRe_FlagResetExcute`(1/2), `/GP_dig_joints`\|`/clean_digPoint`, `/truck_center_point`, `/bucket_terrain_delta_position`, `/RealBktPosXYZ`, `/joints_angle`, `/heading2swing_topic` | `/RefDeviceTraj_Reset` |
| **dig_stub** | `/Sys_SDi_FlagDigExcute`(=1.0), `/joints_angle`, `/heading2swing_topic`, `/terrain_profile`(未消费) | `/RefDeviceTraj_Dig`, `/Sys_RUn_FlagDigFinish` |
| **plant** | `/RefDeviceTraj_{Unload,Reset,Dig}`（优先级仲裁） | `/joints_angle`, `/heading2swing_topic`, `/Swing_topic`, `/plant/settled`, `/plant/active_cmd` |
| **scenario_manager** | — | `/truck_center_unloadpoint{,_2}`, `/GP_dig_joints`, `/clean_digPoint`, `/truck_center_point`, `/bucket_terrain_delta_position`（`/Sys_SUn_BucketNumber` 全循环已关） |
| **environment_stub** | `/system_decision/state_text`, `/Sys_SGp_FlagGpPlan`, `/Sys_SUn_BucketNumber`, `/truck_center_unloadpoint` | `/truck_pose_topic`, `/Sys_RGp_FlagGpPlanFinish`, `/FlagSmoothFinish`, `/FlagCleanFinish`, `/FlagWalkFinish`, `/FlagUpperLowerAlignmentFinish`, `/FlagTruckFull`, `/Sys_SCo_Traj`, `/unman_params_set`, `/Mpc_Switch`, `/Mpc_Switch2` |

**易踩契约要点**

1. **统一完成信号**：三规划器都用 `/RefDeviceTraj_*.position.x` 表达状态（`1.0=执行中 / 0.0=结束`）；决策仅在 `x==0.0` 判完成（非严格下降沿）。
2. **dump 双重触发**：决策既发 `/Sys_SUn_FlagUnloadExcute` 又发 `/Sys_SeqAction`，但 **dump planner 实际只认 `/Sys_SeqAction=4.0`**。
3. **reset 目标源双通道**：`1.0` 走 `/GP_dig_joints`（挖循环），`2.0` 走 `/clean_digPoint`（搭台）；均为 `x=swing, y=boom, z=arm, w=bucket`（deg）。
4. **卸载序列尺寸门槛**：`/Sys_RUn_UnloadPointSequence` 必须 ≥`2000×5=10000` 元素且 `data[0]!=0`，否则决策忽略。
5. **选点所有权（GP 接入前）**：`/Sys_RGp_FlagGpPlanFinish` 与 `/GP_dig_joints` 现分别由 env_stub / scenario_manager 提供；GP 节点接入后这两个话题所有权须移交（当前决定不接入闭环）。
6. **state_text 可见性**：只暴露 TruckStage+DiggingCycleFsm+Staging，DigWaitFsm 子态不可见，故 env_stub 卡车事件用**时间驱动**而非状态驱动。

## 5. 完成度约定（Position.x 语义）

所有 `RefDeviceTraj_*` 消息统一：

```
msg.position.x = 1.0  # 执行中
msg.position.x = 0.0  # 阶段结束（正常或异常中止）
msg.position.y = 0.0  # 保留
msg.position.z = 0.0  # 保留
msg.orientation = (swing, boom, arm, bucket) in deg
```

sequencer / phase_trigger / decision 通过订阅命令话题检测 `1→0` 下降沿判定阶段结束，无需 planner 单独发布 finish flag（dump 的 `FlagUnloadExcuteFinish` 是历史遗留的冗余信号，decision 优先使用 `position.x` 下降沿，phase_trigger 优先使用独立 finish flag）。

## 6. 保真度模型（L1 plant）

`exca_plant_sim.py` 对每个关节独立建模：

```
cmd_deg (输入)  ──[deadband]──[first-order lag τ]──[rate limit]──▶  actual_deg
                                                                      │
                                                    [feedback delay]──[noise σ]
                                                                      ▼
                                                                  publish
```

参数（`config/plant.yaml`）：

| 参数 | 类型 | 说明 |
|---|---|---|
| `rate_<joint>_dps` | float | 速率上限，模拟阀口饱和 |
| `tau_<joint>_s` | float | 一阶惯性时间常数（0 = 纯速率限制，等价旧 exec_sim） |
| `deadband_<joint>_deg` | float | 死区，|cmd - actual| < deadband 时不动作 |
| `sigma_<joint>_deg` | float | 反馈高斯噪声标准差 |
| `feedback_delay_ms` | float | 反馈延迟（环形缓冲实现） |
| `settle_tol_deg` | float | 完成判据：跟踪误差阈值 |
| `settle_dur_sec` | float | 完成判据：稳定持续时间 |
| `publish_rate_hz` | float | 主循环频率 |

**故障注入**（Phase 6）：
- `joint_stuck`：某关节速率临时置 0
- `sensor_dropout`：反馈按概率丢弃
- `latency_spike`：`feedback_delay_ms` 阶跃
- `cmd_freeze`：忽略命令 M 秒

## 7. 目录约定

```
src/exca200_simulink/
├── docs/
│   └── sim_arch.md               # 本文档
├── config/
│   ├── plant.yaml                # L1 plant 参数
│   ├── scenario_default.yaml     # L2 场景默认值
│   ├── rviz.rviz                 # 双机器人对比布局
│   └── rviz_single.rviz          # 单机器人 + marker 叠加
├── scenarios/                    # L2 场景库（Phase 3 填充）
│   ├── baseline.yaml
│   ├── tight_truck.yaml
│   ├── high_reach.yaml
│   ├── noisy_rtk.yaml
│   └── saturated_valve.yaml
├── launch/
│   ├── dump_viz_sim_single_trigger.launch   # 现有，dump 单阶段
│   ├── dig_viz_sim.launch                    # Phase 2
│   ├── reset_viz_sim.launch                  # Phase 2
│   ├── truck_dump_viz_sim.launch             # Phase 2
│   └── full_cycle_sim.launch                 # Phase 4
├── scripts/
│   ├── exec_sim.py                # 旧壳，import exca_plant_sim
│   ├── exca_plant_sim.py          # L1（Phase 1）
│   ├── scenario_pub.py            # 旧壳，import scenario_manager
│   ├── scenario_manager.py        # L2（Phase 3）
│   ├── traj_to_urdf_adapter.py    # L3
│   ├── dump_trigger.py            # 旧壳，import phase_trigger
│   ├── phase_trigger.py           # L4（Phase 2 泛化）
│   ├── cycle_sequencer.py         # L4（Phase 4，保留用于单阶段/向后兼容）
│   ├── environment_stub.py        # L4（decision 外部世界桩）
│   ├── kpi_collector.py           # L5（Phase 5）
│   ├── record_cycle.sh            # L5（Phase 5）
│   ├── run_batch.py               # L6（Phase 5）
│   ├── sim_clock.py               # L6（Phase 5，可选）
│   └── dig_planner_stub.py        # 挖掘阶段桩节点（Phase 2）
├── metrics/                       # 由 kpi_collector / run_batch 写入
├── tests/                         # Phase 6 回归测试
│   └── golden/
└── urdf/
    ├── exca870.urdf               # 双机器人对比布局
    └── exca870_single.urdf        # 单机器人 + marker
```

## 8. 版本基线

| 版本 | Git tag | 说明 |
|---|---|---|
| v0 | `sim-baseline-v0` (待打) | 现有 dump 单阶段闭环（`dump_viz_sim_single_trigger.launch`），tag 前需先 commit 当前未跟踪文件 |
| v1 | `sim-plant-v1` | Phase 1 完成，plant 泛化但向后兼容 |
| v2 | `sim-multi-phase-v2` | Phase 2/3 完成，dig/reset 独立可跑 |
| v3 | `sim-full-cycle-v3` | Phase 4 完成，一键跑完整循环 |
| v4 | `sim-batch-v4` | Phase 5/6 完成，支持批量回归 |
| v5 | `sim-decision-v5` | Decision 节点替换 cycle_sequencer，生产级编排 |

## 9. 待决点

1. **dig planner**：`dig_trajectory_planner` 是纯库，Phase 2 建立 stub 节点（canned 挖掘动作）以满足循环闭合；接入真实 DP 输出（`dig_planner_dp::Plan`）需要额外定义地形输入契约与航路点→时间序列的映射，暂不在本路径内。
2. **truck_dump_planner**：只发 marker，不进入闭环控制，仅作可视化叠加；如需将其纳入循环，需先在该 planner 中新增 `RefDeviceTraj_*` 发布。
3. **use_sim_time**：默认关闭（走 wall clock），Phase 5 引入 `sim_clock.py` 后可切换。
