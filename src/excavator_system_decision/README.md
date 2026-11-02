# excavator_system_decision

无人挖掘机 **系统决策层（L4）** 节点。作为整车作业的**总指挥**，它不生成任何轨迹，只做**阶段编排**：接收矿卡事件流与执行反馈，驱动双层状态机推进，通过 `/Sys_SeqAction`、`/Sys_SeqPlan`、`/Sys_S_Re/Di/Un/Le_Flag*Excute` 等**阶段激活信号**指挥下游 5 大轨迹规划节点、traj_fusion 融合层与矿卡协同，最终完成"卡车进场→挖装→离场→（可选）搭台"的完整作业闭环。

- **命名空间**：`excavator_system_decision`
- **主循环节奏**：10 Hz（可 `update_rate_hz` 配置）
- **架构模式**：DecisionNode → ScenarioManager → TruckInteractionScenario → 5 个内嵌 FSM + TakeoverMonitor + TopicInterface
- **场景扩展**：`ScenarioBase` 抽象接口预留，当前仅实现 `TRUCK_INTERACTION`

---

## 目录结构

```
excavator_system_decision/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── truck_interaction.yaml                        # 参数配置
├── launch/
│   ├── excavator_system_decision_start.launch        # 生产启动（含 robot_logger）
│   └── decision_test_gui.launch                      # 测试 GUI 启动
├── scripts/
│   ├── decision_test_gui.py                          # PyQt 决策测试 GUI
│   └── test_truck_interaction.sh                     # 矿卡交互自动测试脚本
├── include/excavator_system_decision/
│   ├── decision_node.hpp                             # 顶层节点封装（38 行）
│   ├── decision_context.hpp                          # 全局上下文与输出结构（114 行）
│   ├── topic_interface.hpp                           # 话题装配接口（145 行）
│   ├── state_types.hpp                               # 3 大状态枚举（97 行）
│   ├── scenario_base.hpp                             # 场景抽象基类（18 行）
│   ├── scenario_manager.hpp                          # 场景注册与调度（23 行）
│   ├── takeover/takeover_monitor.hpp                 # 卡阶段接管监控
│   └── scenarios/truck_interaction_scenario/
│       ├── truck_interaction_scenario.hpp            # 外层场景（TruckStage 状态机）
│       ├── dig_wait_fsm/                             # 子 FSM 1: 挖装前等车
│       ├── reset_dig_wait_fsm/                       # 子 FSM 2: 复位+挖装等车
│       ├── digging_cycle_fsm/                        # 子 FSM 3: 装车挖装循环
│       ├── staging_pre_move_fsm/                     # 子 FSM 4: 搭台预备移动（当前跳过）
│       └── staging_cycle_fsm/                        # 子 FSM 5: 搭台大循环
└── src/  (对应 .cpp，含 main 位于 decision_node.cpp)
```

**构建产物**：可执行 `excavator_system_decision_node` 与 `decision_test_gui.py`。

---

## 一、系统上下文

```
┌────────────────────────────────────────────────────────────────┐
│              上层：外部指令源                                    │
│  /Mpc_Switch (Bool)     决策总开关（场景启动/停止）              │
│  /Mpc_Switch2 (Bool)    搭台请求（电平信号，true 保持有效）      │
└────────────────┬───────────────────────────────────────────────┘
                 ▼
┌────────────────────────────────────────────────────────────────┐
│              ★ excavator_system_decision（本包）★                │
│                                                                 │
│  DecisionNode (10Hz timer)                                      │
│    ├── ScenarioManager       按类型注册/启动/停止/切换           │
│    ├── TruckInteractionScenario  外层 TruckStage 状态机          │
│    │     ├── DigWaitFsm              阶段 3→5                   │
│    │     ├── ResetDigWaitFsm         阶段 1→2→3→5               │
│    │     ├── DiggingCycleFsm         UNLOAD_PLAN→...→SUCCESS    │
│    │     ├── StagingPreMoveFsm       SWING_RESET→WALK→SUCCESS   │
│    │     └── StagingCycleFsm         INITIAL_DELAY→...→SUCCESS  │
│    ├── TakeoverMonitor        规则表监控 seq_action 持续时长     │
│    └── TopicInterface         15 订阅 + 15 发布                  │
└────────┬──────────────────────────────────────────┬─────────────┘
         │ 输出激活                                 ▲ 反馈输入
         ▼                                        │
┌────────────────────────────────────────────────────────────────┐
│  下游执行链                                                     │
│  traj_fusion_pkg   ← /Sys_SeqAction 阶段号                      │
│  reset_trajectory_planner ← /Sys_SRe_FlagResetExcute            │
│  dig (GP2mid)      ← /Sys_SDi_FlagDigExcute                     │
│  dump_trajectory_planner ← /Sys_SUn_FlagUnloadExcute / Plan     │
│  clean_trajectory_planner ← /Sys_SLe_FlagGroundLevelExcute      │
│  truck_dump_planner / smooth_trajectory_planner 等               │
└────────────────────────────────────────────────────────────────┘
         │                                    ▲
         ▼                                    │
   电液/CAN 执行                       完成反馈 / FlagCleanFinish 等
```

**关键定位**：本包是**唯一的状态编排者**。所有轨迹节点只被动响应 `/Sys_SeqAction` 与自己的 `Flag*Excute` 电平；本包不生成轨迹、不判断几何、不做 IK。

---

## 二、核心数据结构

### 2.1 `DecisionContext`（[decision_context.hpp#L51](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/include/excavator_system_decision/decision_context.hpp)）

全局共享上下文，所有 FSM 通过指针读写：

| 分组 | 字段 | 语义 |
|------|------|------|
| **卡车事件** | `truck_event_code` / `truck_event_time` | `/truck_pose_topic.orientation.w` 承载的事件码 |
|  | `truck_full` | `/FlagTruckFull` 装满标志 |
|  | `staging_requested` | `/Mpc_Switch2` 电平值 |
| **执行反馈** | `reset_execute_finished` | /RefDeviceTraj_Reset position.x=0 触发 |
|  | `dig_execute_finished` | /RefDeviceTraj_Dig position.x=0 触发 |
|  | `unload_execute_finished` | /RefDeviceTraj_Unload position.x=0 触发 |
|  | `smooth_finished` | /FlagSmoothFinish Bool |
|  | `clean_finish_flag` | /FlagCleanFinish Float32（0/1/2）|
|  | `walk_finished` | /FlagWalkFinish Bool（阶段 8） |
|  | `upper_lower_alignment_finished` | /FlagUpperLowerAlignmentFinish Bool（上下盘对齐）|
| **感知** | `real_boom/arm/bucket_angle` | /joints_angle (Quaternion xyz) |
|  | `real_swing_angle` | /heading2swing_topic |
|  | `ref_swing_angle` | /Sys_SCo_Traj orientation.x |
|  | `truck_center_unloadpoint_angle` | atan2(-y,-x) → 0~360° |
| **计数** | `truck_number` | 累计服务卡车数（进程生命周期不清零） |
|  | `bucket_number` | 本车已装铲数（多时机清零，见 §6.2） |
| **输出** | `output` | DecisionOutputs 结构（见下） |

### 2.2 `DecisionOutputs`（[decision_context.hpp#L14](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/include/excavator_system_decision/decision_context.hpp)）

本节点唯一对外可见的输出容器，主循环末尾由 `TopicInterface::publishOutputs()` 统一发布：

```cpp
struct DecisionOutputs {
  double seq_plan;                              // 规划阶段激活
  double seq_action{1.0};                       // 执行阶段激活（默认 1.0）
  double seq_truck;                             // 卡车协同阶段
  bool   fault_require_takeover;                // 接管请求

  double flag_ground_plan;                      // 挖掘点规划激活
  double flag_unload_plan;                      // 卸载点规划激活
  double flag_reset_execute;                    // 复位执行激活
  double flag_dig_execute;                      // 挖掘执行激活
  double flag_ground_level_execute;             // 清台执行激活
  double flag_unload_execute;                   // 卸载执行激活

  geometry_msgs::PoseStamped digger_loading;    // 装车指令（orientation.w 承载事件码）
  std::string state_text;                       // "TruckStage:DigCycle:PreMove:StagingCycle"
};
```

**两套复位语义**：
- `resetCommands()`：只清 6 个 `flag_*` + `digger_loading.w`，保留 seq_* 与 state_text（阶段切换用）
- `resetAll()`：全清到 `seq_action=1.0` 兜底态（进程启动/场景停止用）

---

## 三、话题接口

### 3.1 订阅（15 个，来自 [topic_interface.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/topic_interface.cpp)）

| 类别 | 话题 | 类型 | 回调 | 语义 |
|------|------|------|------|------|
| **规划反馈** | `/Sys_RGp_FlagGpPlanFinish` | `std_msgs/Float64` | `groundPlanCallback` | 挖掘点规划完成（!=0 时刷新时间戳）|
|  | `/Sys_RUn_UnloadPointSequence` | `std_msgs/Float32MultiArray` | `unloadPlanCallback` | 卸载点序列（data[0]!=0 视为完成）|
| **执行反馈** | `/RefDeviceTraj_Reset` | `geometry_msgs/Pose` | `resetExecuteCallback` | **position.x=0 → 复位完成** |
|  | `/RefDeviceTraj_Dig` | `geometry_msgs/Pose` | `digExecuteCallback` | position.x=0 → 挖掘完成 |
|  | `/RefDeviceTraj_Unload` | `geometry_msgs/Pose` | `unloadExecuteCallback` | position.x=0 → 卸载完成 |
|  | `/FlagSmoothFinish` | `std_msgs/Bool` | `smoothCallback` | 抹平完成 |
|  | `/FlagCleanFinish` | `std_msgs/Float32` | `cleanCallback` | **1.0=单轮完成 / 2.0=最终清台** |
|  | `/FlagWalkFinish` | `std_msgs/Bool` | `walkFinishCallback` | 底盘行走完成 |
|  | `/FlagUpperLowerAlignmentFinish` | `std_msgs/Bool` | `upperLowerAlignmentFinishedCallback` | 上下盘回转对齐 |
| **状态** | `/truck_pose_topic` | `geometry_msgs/PoseStamped` | `truckPoseCallback` | **orientation.w = 卡车事件码** |
|  | `/FlagTruckFull` | `std_msgs/Bool` | `fullCallback` | 卡车装满 |
|  | `/joints_angle` | `geometry_msgs/Quaternion` | `jointAnglesCallback` | x/y/z = boom/arm/bucket (deg) |
|  | `/heading2swing_topic` | `std_msgs/Float32` | `realSwingAngleCallback` | 回转实角 |
|  | `/Sys_SCo_Traj` | `geometry_msgs/Pose` | `refSwingAngleCallback` | 回转参考角（orientation.x）|
|  | `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | `truckCenterUnloadpointCallback` | 目标回转角 atan2(-y,-x) |
|  | `/unman_params_set` | `geometry_msgs/Pose` | `unmanParamsSetCallback` | **orientation.w 上升沿→清零 bucket_number** |
|  | `/Mpc_Switch2` | `std_msgs/Bool` | `mpcSwitch2Callback` | 搭台请求（电平）|

### 3.2 发布（15 个）

| 类别 | 话题 | 类型 | 语义 | 消费者 |
|------|------|------|------|--------|
| **规划命令** | `/Sys_SGp_FlagGpPlan` | `std_msgs/Float32` | 挖掘点规划激活 | GP2mid |
|  | `/Sys_SUn_FlagUnloadPlan` | `std_msgs/Float64` | 卸载点规划激活 | GP2mid |
| **执行命令** | `/Sys_SRe_FlagResetExcute` | `std_msgs/Float64` | 复位执行 1/0 | reset_trajectory_planner |
|  | `/Sys_SDi_FlagDigExcute` | `std_msgs/Float64` | 挖掘执行 1/0 | dig 执行 |
|  | `/Sys_SLe_FlagGroundLevelExcute` | `std_msgs/Float64` | 清台执行 1/0 | clean_trajectory_planner |
|  | `/Sys_SUn_FlagUnloadExcute` | `std_msgs/Float64` | 卸载执行 1/0 | dump_trajectory_planner |
| **阶段字** | `/Sys_SeqPlan` | `std_msgs/Float64` | **规划阶段号**（0/1.0/1.5/4.0/4.5）| 数采、可视化 |
|  | `/Sys_SeqAction` | `std_msgs/Float64` | **执行阶段号**（1~9 及 x.5 结束态）| traj_fusion / 各规划节点 |
|  | `/Sys_SeqTruck` | `std_msgs/Float64` | 卡车协同阶段 | 矿卡调度 |
| **装车指令** | `/digger_loading_topic` | `geometry_msgs/PoseStamped` | **orientation.w**：0/1=working/2=permit_entry/3=request_loading/4=request_leave | 矿卡 |
| **状态** | `/fault_require_takeover` | `std_msgs/Bool` | 请求接管 | HMI / 数采 |
|  | `/Sys_SUn_BucketNumber` | `std_msgs/Float64` | 本车已装铲数 | 数采 / HMI |
|  | `/Sys_TruckLoadNumber` | `std_msgs/Float32` | 累计服务车数 | 数采 / HMI |
|  | `/system_decision/state_text` | `std_msgs::String` | "TruckStage:Op:PreStg:Stg" 四段状态字符串 | RViz HUD |

### 3.3 SeqAction / SeqPlan 语义表

**`/Sys_SeqAction`（Float64）** —— 由 `sysSeqActionCallback`（[traj_fusion_pkg/act_traj_fusion.cpp](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/act_traj_fusion.cpp)）消费：

| 值 | 语义 | 触发 FSM 状态 |
|----|------|--------------|
| 1.0 | 初始化 / TRUCK_POSITION_CHECK 过渡 | `initStatusProcess` |
| 2.0 / 2.5 | 复位执行 / 复位结束 | `resetStatusProcess` |
| 3.0 / 3.5 | 挖掘执行 / 挖掘结束 | `digStatusProcess` |
| 4.0 / 4.5 | 提升+卸载 / 卸载结束 | `raiseAndUnloadStausProcess` |
| 5.0 / 5.5 | Boom 抬起 / 抬起结束 | `waitStatusProcess` |
| 6.0 / 6.5 | 抹平执行 / 抹平结束 | `smoothStatusProcess` |
| 7.0 / 7.5 | 清台执行 / 清台结束 | `cleaupStatusProcess` |
| **8.0** | 底盘行走（StagingPreMoveFsm） | — |
| **9.0** | 上下盘对齐（StagingPreMoveFsm） | — |

**`/Sys_SeqPlan`（Float64）** —— 规划阶段：0=空闲 / 1.0=挖掘点规划中 / 1.5=完成 / 4.0=卸载点规划中 / 4.5=完成。

---

## 四、外层状态机：TruckStage

**位置**：[truck_interaction_scenario.cpp#L129](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/truck_interaction_scenario.cpp)

```
              /Mpc_Switch=true
                    │
                    ▼
       ┌─────── INITIAL_WAIT_TRUCK_ENTRY ───────┐
       │  首次等车（选择 DigWait/ResetDigWait） │
       │  挖机挖装当前斗直至卡车 loading_code   │
       └────────────────┬───────────────────────┘
                        ▼
                TRUCK_POSITION_CHECK
                (truck_number++, seq_action=1.0)
                        │
                        ▼
                  DIGGING_CYCLE ──────┐
                  (DiggingCycleFsm     │ 完成
                   多轮挖装直到装满)   │
                        ▼              │
                 DISPATCH_TRUCK_LEAVE  │
                 (并行运行 ResetDigWait│
                  + 等 leave_confirmed)│
                        │              │
                        ▼              │
             staging_requested?        │
                ├── false → WAIT_TRUCK_ENTRY ──┐
                └── true  → STAGING_PRE_MOVE   │
                             │                  │
                             ▼                  │
                       STAGING_CYCLE            │
                             │                  │
                             └──→ WAIT_TRUCK_ENTRY ←┘
                                    （循环下一车）
```

**8 个 TruckStage 状态**（[state_types.hpp#L19](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/include/excavator_system_decision/state_types.hpp)）：

| 值 | 状态 | 职责 |
|----|------|------|
| 0 | `IDLE` | 场景停止 |
| 8 | `INITIAL_WAIT_TRUCK_ENTRY` | 冷启动首次等车 + 等待 FSM 二选一 |
| 1 | `WAIT_TRUCK_ENTRY` | 每车结束后回到此处等下一车 |
| 2 | `TRUCK_POSITION_CHECK` | 卡车到位检测（当前直接自增计数转 DIGGING_CYCLE）|
| 3 | `DIGGING_CYCLE` | 装车挖装循环（内嵌 DiggingCycleFsm）|
| 4 | `DISPATCH_TRUCK_LEAVE` | 请求驶离 + 并行挖一斗等待 |
| 6 | `STAGING_PRE_MOVE` | 搭台预备移动（内嵌 StagingPreMoveFsm）|
| 7 | `STAGING_CYCLE` | 搭台大循环（内嵌 StagingCycleFsm）|

**FSM 二选一策略**（INITIAL_WAIT_TRUCK_ENTRY）：
- 等到 `/heading2swing_topic` 与 `/truck_center_unloadpoint` 都收到首帧
- 计算 `angle_error = |remainder(real_swing - truck_center_angle, 360)|`
- ≤ `truck_center_angle_error_threshold`(默认 30°) → 走 **ResetDigWaitFsm**（先复位再挖装等车，路径：ground_plan→reset→dig→boom_up）
- 否则 → 走 **DigWaitFsm**（直接挖装等车，路径：dig→boom_up）

---

## 五、五个子 FSM 详解

### 5.1 `DigWaitFsm`（2 状态，[dig_wait_fsm.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/dig_wait_fsm/dig_wait_fsm.cpp)）

**用途**：装车前只挖当前斗并抬臂等车。

```
DIG_EXECUTE (seq_action=3.0, flag_dig_execute=1.0)
   │  ── dig_execute_finished (2s 后) ──▶  seq_action=3.5
   ▼
BOOM_UP_PLAN (seq_action=5.0)
   │  ── real_boom_angle ≥ boom_up_angle_threshold
   │      AND |ref_swing - real_swing| ≤ swing_angle_error_threshold
   ▼                                             (seq_action=5.5)
DONE
```

### 5.2 `ResetDigWaitFsm`（4 状态，[reset_dig_wait_fsm.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/reset_dig_wait_fsm/reset_dig_wait_fsm.cpp)）

**用途**：等车前多一段"选点+复位"，用于挖机停在卸载位/远离挖掘区时的自然过渡。

```
DIG_POINT_SELECTION → RESET_EXECUTE → DIG_EXECUTE → BOOM_UP_EXECUTE → DONE
   seq_plan=1.0        seq_action=2.0   seq_action=3.0   seq_action=5.0
   flag_ground_plan    flag_reset_exec  flag_dig_exec
```

### 5.3 `DiggingCycleFsm`（5 状态，[digging_cycle_fsm.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/digging_cycle_fsm/digging_cycle_fsm.cpp)）

**核心装车循环**，多斗挖装直至卡车装满或是达上限。

```
       start() (context.truck_full/smooth_finished=false)
                    │
                    ▼
              UNLOAD_PLAN  (seq_plan=4.0, flag_unload_plan=1.0)
                    │  ── unload_plan_feedback_time > 进入时刻 ──
                    │     seq_plan=4.5
                    ▼
       ┌──▶  UNLOAD_EXECUTE  (双并行)
       │      左线：seq_action=4.0, flag_unload_execute=1.0
       │            └─ 2s 后 unload_execute_finished → bucket_number++, seq_action=4.5
       │      右线：seq_plan=1.0, flag_ground_plan=1.0（挖掘点选择）
       │            └─ ground_plan_feedback_time → seq_plan=1.5, plan_phase=DONE
       │                    │
       │       判据（两条并行都完成 unload_done & 完成 plan）：
       │       ├─ 装满 (truck_full & bucket≥min) 或 bucket≥max → LEVEL_TRUCK
       │       └─ 未装满 → RESET_EXECUTE
       │                    │
       │                    ▼
       │              RESET_EXECUTE (seq_action=2.0, flag_reset_execute)
       │                    │  └─ reset_execute_finished → seq_action=2.5
       │                    ▼
       │              DIG_EXECUTE (双并行)
       │                ├─ seq_action=3.0, flag_dig_execute（左线）
       │                └─ seq_plan=4.0, flag_unload_plan（右线，规划下次卸料点）
       │                    两者都完成 ──┘
       │                                   │
       └───────────────────────────────────┘
                                        (循环回 UNLOAD_EXECUTE)

       LEVEL_TRUCK (seq_action=6.0) —— 抹平整平车厢
              │  ┌─ smooth_finished=true → seq_action=6.5 → SUCCESS
              │  └─ 15s 超时（level_truck_timeout_sec）→ 强制跳过 → SUCCESS
              ▼
            SUCCESS
```

**关键判据**：
- 每斗 +1 `bucket_number`；`bucket_number >= min_bucket_count` 且 `truck_full`，或 `bucket_number >= max_bucket_count` 触发整平
- 所有执行反馈**必须等 `kExecutionFeedbackDelaySec=2.0s` 才生效**（避免规划节点上一轮残留 position.x=0 误触发）

### 5.4 `StagingPreMoveFsm`（3 状态，当前**跳过**，[staging_pre_move_fsm.cpp#L21](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/staging_pre_move_fsm/staging_pre_move_fsm.cpp)）

**当前实现**：`start()` 直接 `transitionTo(SUCCESS)`，跳过 SWING_RESET/WALK_EXECUTE 两步。

**设计意图**（源码保留，等硬件到位）：
```
SWING_RESET (seq_action=9.0)
    │  等待 /FlagUpperLowerAlignmentFinish（上下盘对齐完成）
    ▼
WALK_EXECUTE (seq_action=8.0)
    │  等待 /FlagWalkFinish（底盘后退行走完成）
    ▼
SUCCESS → 触发 StagingCycleFsm
```

启用需修改第 20~21 行注释切换。

### 5.5 `StagingCycleFsm`（11 状态，[staging_cycle_fsm.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/staging_cycle_fsm/staging_cycle_fsm.cpp)）

**搭台（地面平整）大循环**，卡车驶离后由 `/Mpc_Switch2=true` 触发。核心逻辑：**清台→挖装→卸载→复位→再清台** 的外层循环，直到清台报"进入最终挖掘"。

```
INITIAL_DELAY (2s 缓冲)
   ▼
INITIAL_UNLOAD_PLAN (seq_plan=4.0)   ─┐
   ▼                                  │ 首次卸载（清空铲斗）
INITIAL_UNLOAD_EXECUTE (seq_action=4.0)│ bucket_number++
   ▼                                  ─┘
RESET_EXECUTE (seq_action=2.0) ─────┐
   ▼                                │
GROUND_LEVEL_EXECUTE ───────────────┼───  清台循环入口 ★
   seq_action=7.0                   │
   flag_ground_level_execute=1.0    │
   等待 /FlagCleanFinish:           │
     ├─ 1.0 → DIG_EXECUTE（继续循环）
     └─ 2.0 → FINAL_DIG_EXECUTE（进入收尾）
   ▼                                │
DIG_EXECUTE (双并行: seq_action=3.0 + seq_plan=4.0)
   ▼
UNLOAD_EXECUTE (seq_action=4.0) ────┘  bucket_number++
   │ 完成
   └─→ 回到 RESET_EXECUTE（外层循环，重复清台→挖→卸）

（清台报 2.0 后）
   ▼
FINAL_DIG_EXECUTE (seq_action=3.0)
   ▼
FINAL_BOOM_UP_WAIT (seq_action=5.0)
   │ real_boom_angle ≥ boom_up_angle_threshold
   │ AND |ref_swing - real_swing| ≤ swing_angle_error_threshold
   ▼  seq_action=5.5
SUCCESS → 场景回到 WAIT_TRUCK_ENTRY 等下一车
```

**关键设计**：
- 进入 `GROUND_LEVEL_EXECUTE` 时会**主动清 `clean_finish_flag=0.0`**，只接收本阶段内新到达的反馈（[staging_cycle_fsm.cpp#L52](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/staging_cycle_fsm/staging_cycle_fsm.cpp)）
- 清台 1.0/2.0 语义由 `clean_trajectory_planner` 定义，本 FSM 只分发

---

## 六、辅助机制

### 6.1 执行反馈门控（防误触发）

所有执行完成判据必须满足：
```cpp
(ros::Time::now() - state_enter_time_) >= ros::Duration(kExecutionFeedbackDelaySec)  // 2.0s
```

原因：轨迹节点在阶段结束时会发出最后一帧 `position.x=0` 的**保持信号**，可能残留到下一状态进入瞬间；2s 门控保证只接收真正的"新一轮结束"。

规划反馈类似用 `feedbackAfter(feedback_time) = !isZero && > state_enter_time_` 判定。

### 6.2 `bucket_number` 清零时机

| 触发 | 位置 | 场景 |
|------|------|------|
| 卡车入场 | TruckInteractionScenario `entry_requested → truck_entered` 时 | 新车到位 |
| 抹平完成 | DiggingCycleFsm `LEVEL_TRUCK → SUCCESS` | 装完一车 |
| 搭台循环结束 | TruckInteractionScenario `STAGING_CYCLE → WAIT_TRUCK_ENTRY` | 搭台收尾 |
| `/unman_params_set.orientation.w` 上升沿 | `unmanParamsSetCallback` | 无人参数复位 |

`truck_number` **仅进程启动时清零**（`initializeForProgramStart`），场景 stop 不清。

### 6.3 TakeoverMonitor（[takeover_monitor.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/takeover/takeover_monitor.cpp)）

**独立于 FSM 的看门狗**：按规则表监控 `seq_action` 持续时长，超时置 `/fault_require_takeover=true` 并保持 `hold_sec` 秒。

**当前规则**（`initRules`）：
- 唯一规则：`seq_action ≈ 2.0`（复位执行）持续超过 `reset_execute_timeout_sec` → 请求接管
- 生产配置中该 timeout 被设为 **1200000.0s（≈14 天）**，实质禁用；调试时可临时改小

**工作机制**：
```
每帧 update():
  1. 若已激活：hold_sec 内保持 true；到期自动清零
  2. 遍历规则：seq_action 匹配 → 记录 enter_time
  3. 匹配且 (now - enter_time) ≥ timeout_sec → 触发 takeover_active_=true
```

`seq_action_tolerance=1e-6`：Float64 精确匹配。

### 6.4 卡车事件协议（`/truck_pose_topic.orientation.w`）

矿卡 RTK 位姿的话题借用 `orientation.w` 传递**事件码**（PoseStamped 未使用的第 4 个四元数分量）：

| 事件码 | 语义 | 决策响应 |
|--------|------|---------|
| 1.0 `request_entry_code` | 请求进场 | 回 `permit_entry_command=2.0` |
| 2.0 `entering_code` | 进场中（当前未使用）| — |
| 3.0 `loading_code` | 已到位允许装车 | 触发进入 TRUCK_POSITION_CHECK |
| 4.0 `leave_confirmed_code` | 已确认驶离 | 触发 STAGING 或 WAIT_TRUCK_ENTRY 转换 |

**新鲜度**：`truck_event_stale_sec=2.0`，超时视为无事件。

---

## 七、参数配置（[config/truck_interaction.yaml](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/config/truck_interaction.yaml)）

```yaml
update_rate_hz: 10.0                     # 主循环频率

thresholds:
  bucket_numbers_threshold_low: 5        # 最少铲数（配合 truck_full 判定）
  bucket_numbers_threshold_high: 10      # 最多铲数（防呆）
  # 注：DiggingCycleFsm 也读取同名两项作 min/max_bucket_count

timing:
  truck_event_stale_sec: 2.0             # 卡车事件新鲜度

truck_events:
  request_entry: 1.0                     # 事件码
  entering: 2.0
  loading: 3.0
  leave_confirmed: 4.0
  working_command: 1.0                   # 装车指令 orientation.w 值
  permit_entry_command: 2.0
  request_loading_command: 3.0
  request_leave_command: 4.0

operation:
  boom_up_angle_threshold: 33.0          # 抬臂到位判定角（度）
  level_truck_timeout_sec: 15.0          # 抹平整平最长等待时间
  swing_angle_error_threshold: 20.0      # 参考回转 vs 实际回转容差
  truck_center_angle_error_threshold: 30.0  # 决定 DigWait vs ResetDigWait 的容差

takeover:
  seq_action_tolerance: 0.000001         # seq_action 匹配精度
  reset_execute_timeout_sec: 1200000.0   # ≈14 天，实际禁用
  hold_sec: 10.0                         # 接管请求保持时长
```

**加载路径**：launch 用 `<rosparam command="load" file="$(arg config_file)"/>` 挂到**私有命名空间**（`~`），FSM 用 `nh_.param("thresholds/...")` 读取。

---

## 八、编译与运行

### 8.1 编译

```bash
cd ~/yx_planning_ws
catkin_make --pkg excavator_system_decision
source devel/setup.bash
```

依赖：`roscpp` / `std_msgs` / `geometry_msgs` 全为 ROS Noetic 自带。

### 8.2 运行

```bash
# 生产启动（含 robot_logger）
roslaunch excavator_system_decision excavator_system_decision_start.launch

# 测试 GUI（PyQt，手动模拟卡车事件与状态查询）
roslaunch excavator_system_decision decision_test_gui.launch

# 无 robot_logger 环境时
rosrun excavator_system_decision excavator_system_decision_node \
     _update_rate_hz:=10
rosparam load src/excavator_system_decision/config/truck_interaction.yaml \
     excavator_system_decision
```

### 8.3 手动模拟矿卡交互（办公室/仿真）

```bash
# 启动场景
rostopic pub -r 10 /Mpc_Switch std_msgs/Bool "data: true"

# 允许进场（卡车发出 request_entry=1.0）
rostopic pub -1 /truck_pose_topic geometry_msgs/PoseStamped \
  "{pose: {orientation: {w: 1.0}}}"

# 到位装车（loading=3.0）
rostopic pub -1 /truck_pose_topic geometry_msgs/PoseStamped \
  "{pose: {orientation: {w: 3.0}}}"

# 反馈复位完成（模拟 /RefDeviceTraj_Reset position.x=0）
rostopic pub -r 10 /RefDeviceTraj_Reset geometry_msgs/Pose \
  "{position: {x: 0.0}}"

# 请求搭台
rostopic pub -1 /Mpc_Switch2 std_msgs/Bool "data: true"

# 清台反馈
rostopic pub -1 /FlagCleanFinish std_msgs/Float32 "data: 1.0"   # 单轮完成
rostopic pub -1 /FlagCleanFinish std_msgs/Float32 "data: 2.0"   # 进入最终收尾
```

### 8.4 自动化测试

```bash
bash src/excavator_system_decision/scripts/test_truck_interaction.sh
```

---

## 九、调试与观测

### 9.1 关键话题监听

```bash
# 综合状态字符串（4 段：TruckStage:Op:PreStg:Stg）
rostopic echo /system_decision/state_text

# 阶段字（决策对下游发的核心指令）
watch -n0.1 'rostopic echo -n1 /Sys_SeqAction; rostopic echo -n1 /Sys_SeqPlan'

# 执行激活位
rostopic echo /Sys_SRe_FlagResetExcute
rostopic echo /Sys_SDi_FlagDigExcute
rostopic echo /Sys_SUn_FlagUnloadExcute
rostopic echo /Sys_SLe_FlagGroundLevelExcute

# 铲数/车数
rostopic echo /Sys_SUn_BucketNumber
rostopic echo /Sys_TruckLoadNumber

# 接管请求
rostopic echo /fault_require_takeover
```

### 9.2 日志锚点

所有 FSM 状态切换都有 `ROS_INFO("[XXXFsm] -> YYY")`，可 grep：

```bash
rosrun ... 2>&1 | grep -E "\->|takeover|Bucket number reset"
```

### 9.3 常见卡点定位

| 现象 | 定位方向 |
|------|---------|
| 决策不启动 | `/Mpc_Switch` 是否为 true；日志是否有 `[DecisionNode] Ready` |
| 卡在 `INITIAL_WAIT_TRUCK_ENTRY` | `/heading2swing_topic` 与 `/truck_center_unloadpoint` 是否都收到首帧（二选一 FSM 必须两者齐）|
| 卡在 `UNLOAD_EXECUTE` | `unload_execute_finished` 是否变 true；2s 门控是否满足；轨迹节点 `position.x=0` 是否发出 |
| 装车斗数不涨 | `bucket_number++` 只在 `unload_done_` 分支；`/RefDeviceTraj_Unload position.x=0` 反馈是必要 |
| 卡在 `LEVEL_TRUCK` 不结束 | 未发 `/FlagSmoothFinish=true`；等 15s 会自动跳过 |
| 搭台不进入 | `/Mpc_Switch2` 是**电平**信号，必须在 `DISPATCH_TRUCK_LEAVE` 时保持 true |
| 反复清台不结束 | `/FlagCleanFinish` 从未升到 2.0；确认 `clean_trajectory_planner` 条带遍历上限判定 |
| `/fault_require_takeover` 常亮 | TakeoverMonitor `reset_execute` 规则触发；生产 yaml 已放宽到 14 天，检查是否被本地覆盖 |

---

## 十、与其他包的耦合关系

| 相邻包 | 方向 | 关键话题 | 语义 |
|--------|------|---------|------|
| traj_fusion_pkg | → | `/Sys_SeqAction` | 阶段号，驱动 5 大 process |
| traj_fusion_pkg | ← | `/Sys_SCo_Traj` (orientation.x) | 参考回转角（BOOM_UP 判据用）|
| GP2mid | → | `/Sys_SGp_FlagGpPlan`、`/Sys_SUn_FlagUnloadPlan` | 规划激活 |
| GP2mid | ← | `/Sys_RGp_FlagGpPlanFinish`、`/Sys_RUn_UnloadPointSequence` | 规划完成 |
| reset_trajectory_planner | → | `/Sys_SRe_FlagResetExcute` | 复位执行位 |
| reset_trajectory_planner | ← | `/RefDeviceTraj_Reset.position.x` | 复位完成反馈 |
| clean_trajectory_planner | → | `/Sys_SLe_FlagGroundLevelExcute` | 清台执行位 |
| clean_trajectory_planner | ← | `/FlagCleanFinish` (Float32 1/2) | 单轮完成/最终收尾 |
| dump_trajectory_planner | → | `/Sys_SUn_FlagUnloadExcute` | 卸载执行位 |
| dump_trajectory_planner | ← | `/RefDeviceTraj_Unload.position.x` | 卸载完成 |
| smooth_trajectory_planner | ← | `/FlagSmoothFinish` (Bool) | 抹平完成 |
| truck_dump_planner / 卡车 | → | `/digger_loading_topic.orientation.w` | 装车指令 |
| truck_dump_planner / 卡车 | ← | `/truck_pose_topic.orientation.w` | 事件码 |
| 数采 data_to_nas | ← | 所有 `/Sys_*` 与反馈话题被旁听存档 | — |

**唯一入口开关**：`/Mpc_Switch` 是场景启停的**唯一切换信号**，其他话题全部为业务反馈。

---

## 十一、已知限制与接手建议

1. **`StagingPreMoveFsm` 被硬编码跳过**（源码 L20~21）：实际部署时若底盘行走硬件到位，需切换回 `transitionTo(SWING_RESET)`。
2. **TakeoverMonitor 规则极少**：只覆盖 `reset_execute` 超时一条，且生产 yaml 阈值≈禁用；建议按阶段增加 `dig/unload/clean` 各自的超时监控。
3. **未使用 `/excavator_state`（traj_fusion 里的）**：本包完全靠轨迹节点自己的 `position.x=0` 反馈判定完成，若某节点忘记发会导致死等；可考虑加入超时兜底。
4. **`Seq_Plan=1.5 / 4.5` 未清理**：一次设值后直到下一次规划激活前会持续发布；下游消费者需注意电平语义。
5. **`truck_number` 只在进程启动清零**：调试时想重新计数需重启节点。
6. **`bucket_numbers_takeover_threshold=5`** 在 `TruckInteractionConfig` 声明但代码未使用（预留字段）。
7. **launch 依赖 `robot_logger` 包**：工作区内不存在，办公室复现需注释掉或补齐。
8. **FSM 无持久化**：节点重启会丢失所有状态（`bucket_number/truck_number` 除外），重启后需从 `INITIAL_WAIT_TRUCK_ENTRY` 重新开始。

---

## 十二、上手学习顺序（针对本包）

1. **半天**：读 [state_types.hpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/include/excavator_system_decision/state_types.hpp) 建立 3 大枚举心智模型；读 [topic_interface.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/topic_interface.cpp) 掌握全部话题
2. **半天**：读 [decision_node.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/decision_node.cpp)（60 行）→ [scenario_manager.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenario_manager.cpp)（39 行）→ [truck_interaction_scenario.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/truck_interaction_scenario.cpp)（265 行）主流程
3. **1 天**：吃透 [digging_cycle_fsm.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/digging_cycle_fsm/digging_cycle_fsm.cpp)（装车循环）与 [staging_cycle_fsm.cpp](file:///home/zhuay/yx_planning_ws/src/excavator_system_decision/src/scenarios/truck_interaction_scenario/staging_cycle_fsm/staging_cycle_fsm.cpp)（搭台循环）
4. **半天**：`dig_wait_fsm/reset_dig_wait_fsm/staging_pre_move_fsm` 三个短 FSM 快速过
5. **半天**：跑 `scripts/test_truck_interaction.sh`，观察日志与 `state_text` 变化
6. **实战**：把本包话题对照 [traj_fusion_pkg/README.md](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/README.md) 中的阶段映射表双向印证——决策发 3.0，看 traj_fusion 是否走 `digStatusProcess`；这是理解整个系统语义最快的路径

---

## 十三、扩展到新场景

`ScenarioBase` + `ScenarioManager` 是**已抽象但未复用**的扩展点：

1. 新建 `include/excavator_system_decision/scenarios/<your_scenario>/` 与对应 cpp
2. 继承 `ScenarioBase`，实现 `type()/start()/stop()/process()/running()` 5 个纯虚函数
3. 在 `state_types.hpp` 的 `ScenarioType` 枚举增加 `YOUR_TYPE`
4. 在 `DecisionNode` 构造函数中 `std::make_shared<YourScenario>() → registerScenario`
5. 增加新的话题入口触发 `scenario_manager_.start(ScenarioType::YOUR_TYPE)`

当前 `ScenarioManager` 用 `active_scenario_` 只允许单场景运行，多场景并存需要改造。
