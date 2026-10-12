# 挖掘机规划仿真体系 · 使用说明书

> 本文只讲**怎么用**。话题/单位/frame/完成信号的**契约**见 [`sim_arch.md`](./sim_arch.md)，
> 改契约前先改那份文档。二者配套阅读。

---

## 1. 这套仿真能做什么

| 能力 | 入口 | 说明 |
|---|---|---|
| 单阶段闭环回放 | `launch/{dig,dump,reset,truck_dump}_viz_sim*.launch` | 单独跑一个规划器，看命令 vs 反馈 |
| 全循环一键跑 | `launch/full_cycle_sim.launch` | decision 节点编排 dig → dump → reset → level → next truck |
| 场景驱动 | `scenarios/*.yaml` | 卡车位置、挖掘目标、RTK 噪声、阀饱和等 |
| 被控对象保真 | `config/plant.yaml` | 一阶惯性 τ、速率限制、死区、反馈噪声/延迟、settle 完成判据 |
| 故障注入 | `/plant/fault` 服务 | `joint_stuck` / `sensor_dropout` / `latency_spike` / `cmd_freeze` |
| 度量 | `/cycle_report`、`kpi_collector.py` | 每循环耗时、阶段耗时、状态、跟踪误差 |
| 批量回归 | `scripts/run_batch.py` | 遍历 场景 × 重复次数，汇总 `metrics/` |
| 回归测试 | `tests/test_*.py` | 黄金基线比对，测试间 ROS master 隔离 |

**保真度定位**：纯运动学 + 执行机构延迟，不含 Gazebo/物理引擎。单阶段约 15s、全循环约 25s（wall-clock）。

---

## 2. 目录速览

```
src/exca200_simulink/
├── docs/            sim_arch.md（契约） · user_manual.md（本文）
├── config/          plant.yaml · scenario_default.yaml · rviz*.rviz
├── scenarios/       baseline / tight_truck / high_reach / noisy_rtk / saturated_valve
├── launch/          单阶段 + full_cycle_sim 入口
├── scripts/         plant / scenario / trigger / decision stub / kpi / batch / adapter / dig 桩
├── metrics/         run_batch 与 kpi_collector 产物（运行后生成）
├── tests/           test_*.py · _isolation.py · golden/
└── urdf/            exca870.urdf（双机器人对比） · exca870_single.urdf（单机 + marker）
```

---

## 3. 环境准备

```bash
# 1) ROS + 工作空间
source /opt/ros/noetic/setup.bash
cd ~/yx_planning_ws
catkin_make
source devel/setup.bash

# 2) 校验包可见
rospack find exca200_simulink     # 应打印 .../src/exca200_simulink
```

- **roslaunch 会自动拉起 roscore**，手动跑 launch 无需先起 master。
- **无显示器 / 远程**：给所有 launch 加 `rviz:=false`（headless）。
- **回归测试**自带独立 master（见 §12），**不需要**预先 `roscore`。

---

## 4. 快速开始（3 条命令）

```bash
cd ~/yx_planning_ws && source devel/setup.bash

# A) dump 单阶段（headless，自动跑一轮，约 15s）
roslaunch exca200_simulink dump_viz_sim_single_trigger.launch rviz:=false auto_loop_sec:=6.0

# B) 全循环 1 圈（headless，decision 编排，约 40-60s）
roslaunch exca200_simulink full_cycle_sim.launch rviz:=false truck_full_threshold:=1

# C) 回归测试（黄金基线比对）
cd src/exca200_simulink
python3 tests/test_dump_regression.py
python3 tests/test_full_cycle_regression.py
python3 tests/test_plant_step_response.py
```

想边跑边看：去掉 `rviz:=false`（默认 `true`）即弹出 RViz，单机器人 + 规划 marker 叠加。

> 每步的**真实终端输出与逐行解读**见 §15；**如何读输出 / RViz / 话题**见 §16。

---

## 5. 单阶段运行

所有单阶段 launch 都自带 `phase_trigger`：**启动后自动触发一轮**；设 `auto_loop_sec:=N`（N>0）可每 N 秒重复触发，便于连续观察。

### 5.1 dump —— 真实 `dump_trajectory_planner` 闭环

```bash
roslaunch exca200_simulink dump_viz_sim_single_trigger.launch \
    rviz:=false auto_loop_sec:=6.0 \
    truck_near_x:=6.0 truck_near_y:=1.5 truck_far_x:=8.0 truck_far_y:=2.0 truck_heading:=20.0
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `rviz` | `true` | 是否开 RViz |
| `bucket_number` | `2.0` | 当前装载斗数（`/Sys_SUn_BucketNumber`） |
| `truck_near_x/y` | `6.0/1.5` | 卡车近点（m） |
| `truck_far_x/y` | `8.0/2.0` | 卡车远点（m，动态卸载点选取用） |
| `truck_heading` | `20.0` | RTK 方位（deg） |
| `tau_{swing,boom,arm,bucket}_s` | `0.0` | 各关节一阶惯性时间常数 |
| `rate_{swing,boom,arm,bucket}_dps` | `30/15/20/40` | 各关节速率上限 |
| `dig_{swing,boom,arm,bucket}_deg` | `180/5/-110/-110` | **起始(carry)姿态** |
| `auto_loop_sec` | `0.0` | >0 时周期重复触发 |

**观察点**：终端出现 `[phase_trigger:dump] cycle finished`；命令话题 `/RefDeviceTraj_Unload.position.x` 由 `1→0`（下降沿=阶段结束）；完成标志 `/Sys_RUn_FlagUnloadExcuteFinish=1.0`。

### 5.2 dig —— 桩节点（canned 动作，非真实 DP）

```bash
roslaunch exca200_simulink dig_viz_sim.launch rviz:=false \
    dig_swing_deg:=180 dig_boom_deg:=5 dig_arm_deg:=-110 dig_bucket_deg:=-110 auto_loop_sec:=8.0
```

- `dig_*_deg`：挖掘起始(carry)姿态。
- 桩脚本执行 KF1→KF4 的固定时序动作，**末尾回到 carry 姿态**并等 plant 稳定后再报完成（`/Sys_RUn_FlagDigFinish=1.0`），以便无缝衔接 dump。
- ⚠️ **这是桩**：真实 `dig_trajectory_planner`（纯库 `dig_planner_dp`）尚未接入闭环，见 §14 与 `sim_arch.md §9`。

### 5.3 reset —— 真实 `reset_trajectory_planner` 闭环

```bash
roslaunch exca200_simulink reset_viz_sim.launch rviz:=false auto_loop_sec:=8.0
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `dig_{swing,boom,arm,bucket}_deg` | `20/20/-80/-60` | **复位起点**（当前姿态，一般是卸料后的伸展姿） |
| `gp_{swing,boom,arm,bucket}_deg` | `180/5/-110/-110` | **复位目标**（下一挖掘点，`/GP_dig_joints`） |
| `auto_loop_sec` | `0.0` | >0 时周期重复触发 |

- reset 无独立 finish 话题，靠 `/RefDeviceTraj_Reset.position.x` 的 `1→0` 下降沿判完成。
- 若 swing 行程小于阈值会跳过回转子阶段直接收臂（正常行为）。

### 5.4 truck_dump —— 仅可视化叠加（不进闭环）

```bash
roslaunch exca200_simulink truck_dump_viz_sim.launch rviz:=true \
    test_truck:=true init_swing_deg:=180 init_boom_deg:=5 init_arm_deg:=-110 init_bucket_deg:=-110
```

- `truck_dump_planner` **只发 marker**，不发布 `RefDeviceTraj_*`，因此不参与 plant 闭环控制，仅用于可视化对比。
- `init_*_deg`：plant 初始姿态；`test_truck:=true` 用内置测试卡车 pose。

---

## 6. 全循环运行（`full_cycle_sim.launch`）

```bash
roslaunch exca200_simulink full_cycle_sim.launch \
    rviz:=false truck_full_threshold:=1 \
    scenario_file:=$(rospack find exca200_simulink)/scenarios/baseline.yaml
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `scenario_file` | `scenarios/baseline.yaml` | 场景 YAML（绝对路径） |
| `auto_start_delay` | `2.0` | 启动后多少秒发 `/Mpc_Switch=true`（触发 decision） |
| `truck_entry_delay` | `3.0` | 启动后多少秒发卡车进场事件 |
| `truck_loading_delay` | `2.0` | 进场后多少秒切换为 loading 事件 |
| `truck_leave_delay` | `5.0` | DISPATCH_TRUCK_LEAVE 后多少秒发 leave_confirmed |
| `truck_full_threshold` | `99` | 累计斗数达此值发 `/FlagTruckFull`（默认 99 = 不主动满车） |
| `ground_plan_delay` | `1.0` | 收到整地规划请求后多少秒回复完成 |
| `smooth_finish_delay` | `2.0` | LEVEL_TRUCK 后多少秒回复整平完成 |
| `include_truck_dump_viz` | `false` | 叠加 truck_dump marker（可选） |
| `rate_{boom,arm,swing,bucket}_dps` | `15/20/30/40` | 各关节速率上限 |

**组成节点**：`exca_plant_sim` + `dig_planner_stub` + `scenario_manager` + `excavator_system_decision`（C++）+ `environment_stub` + `traj_to_urdf_adapter`，并 `include` 真实 `dump_trajectory_planner` 与 `reset_trajectory_planner`。

**阶段触发值**：dig=`/Sys_SDi_FlagDigExcute=1.0`，dump=`/Sys_SeqAction=4.0`，reset=`/Sys_SRe_FlagResetExcute=1.0`。

**完成/状态语义**：decision 节点通过 `/system_decision/state_text`（latched String）发布当前 FSM 状态。一个完整卡车循环的标志是 state_text 经历 `DIGGING_CYCLE → DISPATCH_TRUCK_LEAVE → WAIT_TRUCK_ENTRY`。

### 6.1 Decision 节点状态机简图

```
Mpc_Switch=true
    ↓
INITIAL_WAIT_TRUCK_ENTRY  ← DigWaitFsm 或 ResetDigWaitFsm（回转对准）
    ↓ (truck event=3.0 且 dig_wait 完成)
TRUCK_POSITION_CHECK
    ↓
DIGGING_CYCLE  ← DiggingCycleFsm:
    │   UNLOAD_PLAN → UNLOAD_EXECUTE → RESET_EXECUTE → DIG_EXECUTE → ...
    │   → LEVEL_TRUCK → SUCCESS
    ↓
DISPATCH_TRUCK_LEAVE  (一个完整循环结束)
    ↓ (truck event=4.0)
WAIT_TRUCK_ENTRY  → (下一辆卡车 event=1.0) → TRUCK_POSITION_CHECK → ...
```

### 6.2 Environment Stub 参数

`environment_stub.py` 模拟 decision 所需的全部外部世界信号：

| 私有参数 | 默认 | 说明 |
|---|---|---|
| `~auto_truck_events` | `true` | 自动推进卡车事件序列 |
| `~truck_entry_delay_sec` | `3.0` | 启动→request_entry 延迟 |
| `~truck_loading_delay_sec` | `2.0` | 进入 DIGGING_CYCLE 后→loading 延迟 |
| `~truck_leave_delay_sec` | `5.0` | DISPATCH_TRUCK_LEAVE→leave_confirmed 延迟 |
| `~ground_plan_delay_sec` | `1.0` | 整地规划完成延迟 |
| `~smooth_finish_delay_sec` | `2.0` | 整平完成延迟 |
| `~auto_start_delay_sec` | `2.0` | 自动发 Mpc_Switch=true 延迟 |
| `~truck_full_threshold` | `99` | 满车斗数阈值 |
| `~truck_event_rate_hz` | `2.0` | 卡车事件持续发布频率 |
| `~sco_traj_rate_hz` | `10.0` | 参考回转角发布频率 |

---

## 7. 场景文件（`scenarios/*.yaml`）

场景只描述**静态输入 + 激活策略 + 扰动**；plant 动力学参数（τ/rate）不在场景里，走 launch 的 `rate_*/tau_*` 或 `config/plant.yaml`。

```yaml
truck:                       # 卡车（dump/reset 的静态输入）
  near_x: 6.0
  near_y: 1.5
  near_z: 0.0
  far_x: 8.0
  far_y: 2.0
  heading_deg: 20.0          # RTK 方位
bucket_number: 2.0           # 当前装载斗数
gp_dig_joints:               # reset 目标（下一挖掘点），deg
  {swing_deg: 180.0, boom_deg: 5.0, arm_deg: -110.0, bucket_deg: -110.0}
clean_dig_joints:            # 搭台模式目标，deg（同上格式）
  {swing_deg: 180.0, boom_deg: 5.0, arm_deg: -110.0, bucket_deg: -110.0}
bucket_terrain_z: -1.0       # 地形高度桩 .point.z（m）
terrain_profile: {points: []}  # dig 地形剖面（桩可为空）
activations:                 # 激活策略
  publish_rate_hz: 1.0
  auto_activate: true
  activate_delay_sec: 2.0
  loop_activate: false
  dump_activate_value: 4.0
  reset_activate_value: 1.0
  dig_activate_value: 3.0
perturbations:               # 扰动（0 = 关闭）
  truck_pos_noise_sigma_m: 0.0
  truck_heading_noise_sigma_deg: 0.0
  rtk_bias_deg: 0.0
  bucket_number_jitter: 0
```

**内置场景**：

| 文件 | 用途 |
|---|---|
| `baseline.yaml` | 标准卡车 + 标准挖掘目标 + 无扰动（所有回归的参考基线） |
| `tight_truck.yaml` | 卡车更近/更窄，考验可达性与卸载点选取 |
| `high_reach.yaml` | 远/高卸载点，考验伸展极限 |
| `noisy_rtk.yaml` | 开 RTK 噪声/偏置（`perturbations`），考验鲁棒性 |
| `saturated_valve.yaml` | 阀饱和（低 rate / 高 τ），考验执行机构受限下的规划 |

**自定义**：复制 `baseline.yaml` 改字段即可；`scenario_manager` 对缺省字段有内置默认值（见脚本头 `DEFAULT_SCENARIO`）。

**运行中热切换**（详见 §15.3）：`rosparam set /scenario_manager/select_name <name>` 后 `rosservice call /scenario/select "{}"`；改了当前 YAML 用 `rosservice call /scenario/reload "{}"` 原地重载；或用私有参数覆盖单字段（`_truck_near_x:=5.0` 等）。

---

## 8. 被控对象参数（`config/plant.yaml`）

每关节独立建模：`cmd ─[deadband]─[一阶惯性 τ]─[rate limit]─▶ actual ─[feedback delay]─[noise σ]─▶ publish`。

| 参数（`<j>` ∈ swing/boom/arm/bucket） | 默认 | 含义 |
|---|---|---|
| `publish_rate_hz` | `50.0` | 主循环频率 |
| `rate_<j>_dps` | `30/15/20/40` | 速率上限（阀口饱和） |
| `tau_<j>_s` | `0.0` | 一阶惯性时间常数（`0`=纯速率限制，等价旧 exec_sim） |
| `deadband_<j>_deg` | `0.0` | 死区，`|cmd-actual|<deadband` 不动作 |
| `sigma_<j>_deg` | `0.0` | 反馈高斯噪声标准差 |
| `feedback_delay_ms` | `0.0` | 反馈延迟（环形缓冲） |
| `init_<j>_deg` | `180/5/-110/-110` | 初始(carry)姿态 |
| `settle_tol_deg` / `settle_dur_sec` | `3.0 / 0.5` | 完成判据：误差阈值 / 稳定持续时间 |
| `cmd_priority` | Unload=10,Reset=20,Dig=30,Dump=90 | 命令源仲裁（数字小=优先级高） |

**覆盖方式（优先级由低到高）**：`config/plant.yaml` < launch 的 `rate_*/tau_*/init_*` 参数 < 节点私有 ROS 参数。
> ⚠️ 私有 ROS 参数会**覆盖** YAML。手动 `rosrun ... _tau_boom_s:=0.3` 后若不清理，参数会残留到共享 master（见 §13 排查）。

---

## 9. 故障注入（`/plant/fault`）

两步：**先用参数选择故障，再用服务触发**。

| 故障 | 关键参数 | 效果 |
|---|---|---|
| `joint_stuck` | `joint`(swing/boom/arm/bucket), `duration_sec`(0=直到手动关) | 指定关节速率临时置 0 |
| `sensor_dropout` | `drop_prob` ∈ [0,1] | 反馈按概率丢弃 |
| `latency_spike` | `extra_delay_ms` | 反馈延迟阶跃增大 |
| `cmd_freeze` | `duration_sec` | 忽略命令若干秒 |

```bash
# 例：让 boom 卡死 3 秒
rosparam set /exca_plant_sim/faults/joint_stuck/enabled true
rosparam set /exca_plant_sim/faults/joint_stuck/joint boom
rosparam set /exca_plant_sim/faults/joint_stuck/duration_sec 3.0
rosservice call /plant/fault "data: true"     # 触发
rosservice call /plant/fault "data: false"    # 关闭
```

也可在 launch 里用私有参数预选（再由 `/plant/fault(true)` 触发）：
```
rosrun exca200_simulink exca_plant_sim.py \
    _faults/joint_stuck/enabled:=true _faults/joint_stuck/joint:=boom
```
> 注：`/exca_plant_sim` 是默认节点名；在 launch 里节点名可能不同，用 `rosnode list` 确认后替换前缀。

---

## 10. 度量与产物

**`/cycle_report`（`std_msgs/String`，JSON，由 `cycle_sequencer` 每圈发布）** 含：`cycle_id`、`status`、整圈耗时、各阶段耗时、失败原因。

**`kpi_collector.py`（可选，写盘）** CSV 列：
```
cycle_id, status, cycle_duration_sec,
dur_dig, dur_dump, dur_reset,
max_err_dig, max_err_dump, max_err_reset,
final_err_boom, final_err_arm, final_err_bucket, final_err_swing,
cmd_count_dig, cmd_count_dump, cmd_count_reset, timestamp
```
参数：`~output_dir`（默认 `metrics/`）、`~max_cycles`（0=不限）。产物 `run_<ts>.csv` + `run_<ts>.json`。手动挂载：
```bash
rosrun exca200_simulink kpi_collector.py _output_dir:=$(rospack find exca200_simulink)/metrics
```

**`record_cycle.sh`**：录制一轮 rosbag（用法见脚本头注释）。

> ⚠️ **现状**：`full_cycle_sim.launch` **默认不启动** `kpi_collector`。若需要 `metrics/*.csv`，请另开终端按上面命令 `rosrun`，或在 launch 中自行加节点。`/cycle_report`（终端可见）不受影响。

---

## 11. 批量回归（`run_batch.py`）

```bash
cd src/exca200_simulink
python3 scripts/run_batch.py --list                                  # 仅列出可用场景
python3 scripts/run_batch.py --scenarios scenarios/ --repeats 3 --cycles 2
python3 scripts/run_batch.py --scenarios scenarios/baseline.yaml scenarios/tight_truck.yaml --repeats 1 --cycles 1
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `--scenarios` | — | 场景目录或若干 YAML（`nargs+`） |
| `--repeats` | `1` | 每场景重复次数 |
| `--cycles` | `1` | 每次运行的循环数 |
| `--timeout` | `300.0` | 单 case 超时（s） |
| `--rviz` | 关 | 加此 flag 才开 RViz（默认 headless） |
| `--list` | — | 仅列出场景后退出 |

**机制**：每个 case 启一次 `full_cycle_sim.launch`（headless）→ 监控 `/cycle_report` 计数直到目标圈数或超时 → 收集 KPI → 汇总。
**产物**：
```
metrics/batch_<ts>/
  ├── <scenario>_<rep>/run_*.csv
  ├── <scenario>_<rep>/run_*.json
  └── summary.md
```
> 因 §10 的 `kpi_collector` 未默认接入 `full_cycle_sim`，`summary.md` 的丰富 KPI 依赖 `kpi_collector` 输出；当前稳定可得的是 success / 循环计数。需要完整 KPI 时，先手动 `rosrun kpi_collector`（见 §10）。

---

## 12. 回归测试（`tests/`）

| 文件 | 覆盖 | 容差 | 黄金基线 |
|---|---|---|---|
| `test_plant_step_response.py` | plant 阶跃：速率限制 / 一阶惯性 / 死区 / settle / joint_stuck 故障 | — | 无（断言式，5 项） |
| `test_dump_regression.py` | baseline 场景 dump 单阶段 | **5%** | `golden/baseline_dump.json` |
| `test_full_cycle_regression.py` | baseline 场景全循环（要求 `status=ok`） | **10%** | `golden/full_cycle.json` |

**运行**（需先 `source devel/setup.bash`；**不需要**预先 roscore）：
```bash
cd src/exca200_simulink
python3 tests/test_dump_regression.py            # 或 pytest tests/test_dump_regression.py -v
python3 tests/test_full_cycle_regression.py
python3 tests/test_plant_step_response.py
```

**黄金基线**：首次运行若基线不存在则**自动生成**（首跑=建基线）；改了预期行为后用 `--update-golden` 重建：
```bash
python3 tests/test_dump_regression.py --update-golden
```
当前基线：`baseline_dump.json = {success:true, dump_duration_sec≈15.4}`；`full_cycle.json = {success:true, cycles_completed:1, cycle_statuses:["ok"], mean_cycle_duration_sec≈24.4}`。

**测试隔离**：`tests/_isolation.py` 给每个测试起一个**独立 ROS master**（随机空闲端口，全新 param server），因此：测试顺序无关、可并行、不依赖外部 roscore、**不会跨测试串参**。

**验收自检**（对应 Phase 6 验收标准）：把 `dump_viz_sim_single_trigger.launch` 的 `tau_boom_s` 默认临时改大 3 倍（如 `1.5`）→ `test_dump_regression.py` 应 **FAIL** 并打印超差（`dump_duration_sec: golden=... actual=... diff=..%`）；改回 `0.0` 后复跑应 PASS。

---

## 13. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `rospack find` 找不到包 | 未 source | `source ~/yx_planning_ws/devel/setup.bash` |
| 跨测试/跨运行参数怪异（如 init 姿态变 0） | 上一次 `rosrun`/`roslaunch` 被 SIGKILL，私有参数残留在**共享** roscore | 用隔离 master 的测试不受影响；手动污染时：`rosparam delete /exca_plant_sim` `rosparam delete /exec_sim` |
| dump 中止、终端见 `bucket below truck (height=.. <= 0.00m)` | seg0 高度门：铲斗齿尖未过卡车顶；会 boost boom 最多 6 次 / 4s，超时 30s abort | 确认起始姿态为 carry `180/5/-110/-110`；卡车过远/过高时调 `truck_*` 或场景 |
| 全循环 `status=timeout:reset` | 多为**上一阶段异常中止**的 `position.x` 下降沿被误判为"完成"→ 进 reset 卡住 | 查 dig 桩是否回到 carry、dump 是否 abort；先让单阶段各自 PASS 再跑全循环 |
| dump 航路点越界 abort（`arm/bucket out of [...]`） | 起始姿态不对，IK 解越限（arm∈[-150,-30]、bucket∈[-135,30]） | 复位到 carry 姿态；检查场景 `truck_*` 是否可达 |
| roslaunch 输出到文件时"卡住不动" | Python 块缓冲 | 排查时直接输出到终端；脚本里用 `PYTHONUNBUFFERED=1` |
| 远程/无显示报错 | 开了 RViz | 全部加 `rviz:=false` |
| 测试很慢（>100s） | 未走 headless 或轮询未及时早退 | 确认 `rviz:=false`；测试已内置 `PYTHONUNBUFFERED` 早退 |

---

## 14. 已知限制与后续

1. **dig 是桩**：`dig_planner_stub.py` 跑固定 KF1→KF4 动作（末尾回 carry）。真实 `dig_trajectory_planner`（纯库 `dig_planner_dp::Plan`）尚未接入闭环——需先定义地形输入契约与"航路点→时间序列"映射（`sim_arch.md §9`）。
2. **truck_dump 仅可视化**：只发 marker，不发 `RefDeviceTraj_*`，不进闭环；如需纳入循环，须先在该 planner 内新增命令发布。
3. **kpi_collector 未默认挂到 full_cycle**：`/cycle_report` 正常，但 `metrics/*.csv` 需手动 `rosrun`（§10）。
4. **use_sim_time 关闭**：走 wall-clock。加速需引入 `sim_clock.py` 并切 `use_sim_time:=true`（尚未落地）。
5. **尚未纳入 git**：当前 harness 多为未跟踪文件；打版本 tag（`sim-baseline-v0` 等，见 `sim_arch.md §8`）前需先 commit。
6. **CI 工作流可选、未创建**：`.github/workflows/sim-regression.yml` 在计划中标为"可选"，尚未添加。

---

## 15. 端到端实操示例（真实终端输出）

> `[…]` 时间戳前缀是 roslaunch `output="screen"` 的固定格式 `[INFO] [<ros秒>]:`（异常为 `[WARN]`/`[ERROR]`）。**示例数值随机器负载浮动**，此处取值与当前黄金基线自洽（全循环 ≈24.4s、standalone dump ≈15.4s），仅作解读参考。

### 15.1 dump 单阶段：完整跑一轮

```bash
roslaunch exca200_simulink dump_viz_sim_single_trigger.launch rviz:=false auto_loop_sec:=0.0
```

典型输出（一轮）：
```
[INFO] [...]: [scenario] loaded 'baseline' from .../scenarios/baseline.yaml; truck=(6.00,1.50) heading=20.0 ...
[INFO] [...]: [plant] loaded config: .../config/plant.yaml
[INFO] [...]: [phase_trigger:dump] firing /Sys_SeqAction=4.0
[INFO] [...]: [phase_trigger:dump] waiting for phase done ...
[INFO] [...]: [phase_trigger:dump] done via finish flag
[INFO] [...]: [phase_trigger:dump] cycle finished, ready for next
```
**解读**：`firing` = 已向 `/Sys_SeqAction` 发 `4.0` 激活 dump；dump planner 随即发布 `/RefDeviceTraj_Unload`（`position.x=1` 执行中）；`done via finish flag` = 收到冗余完成标志 `/Sys_RUn_FlagUnloadExcuteFinish=1.0`；`cycle finished` 即一轮成功。想连续跑设 `auto_loop_sec:=6.0`。

### 15.2 全循环：跑一圈并读 KPI

```bash
roslaunch exca200_simulink full_cycle_sim.launch rviz:=false truck_full_threshold:=1
```

典型 decision + environment_stub 输出：
```
[INFO] [...]: [DecisionNode] Ready; TruckInteractionScenario trigger: /Mpc_Switch
[INFO] [...]: [env_stub] ready; auto_truck=True entry_delay=3.0 ...
[INFO] [...]: [env_stub] auto-start: publishing /Mpc_Switch=true
[INFO] [...]: [TruckInteractionScenario] Started by /Mpc_Switch
[INFO] [...]: [TruckInteractionScenario] -> INITIAL_WAIT_TRUCK_ENTRY
[INFO] [...]: [env_stub] truck event → request_entry (w=1.0)
[INFO] [...]: [DigWaitFsm] -> FINISHED
[INFO] [...]: [TruckInteractionScenario] -> TRUCK_POSITION_CHECK
[INFO] [...]: [env_stub] truck event → loading (w=3.0)
[INFO] [...]: [TruckInteractionScenario] -> DIGGING_CYCLE
[INFO] [...]: [DiggingCycleFsm] -> UNLOAD_PLAN
[INFO] [...]: [DiggingCycleFsm] -> UNLOAD_EXECUTE
[INFO] [...]: [DiggingCycleFsm] -> RESET_EXECUTE
[INFO] [...]: [DiggingCycleFsm] -> DIG_EXECUTE
[INFO] [...]: [dig_stub] triggered; start pose sw/bm/arm/bkt=180.0/5.0/-110.0/-110.0
[INFO] [...]: [DiggingCycleFsm] -> UNLOAD_EXECUTE
[INFO] [...]: [DiggingCycleFsm] -> LEVEL_TRUCK
[INFO] [...]: [env_stub] FlagSmoothFinish=true (LEVEL_TRUCK responded)
[INFO] [...]: [DiggingCycleFsm] -> SUCCESS
[INFO] [...]: [TruckInteractionScenario] -> DISPATCH_TRUCK_LEAVE
[INFO] [...]: [env_stub] truck leave pending (waiting 5.0s)
[INFO] [...]: [env_stub] truck event → leave_confirmed (w=4.0)
[INFO] [...]: [TruckInteractionScenario] -> WAIT_TRUCK_ENTRY
```

实时观察 decision 状态：
```bash
rostopic echo /system_decision/state_text
# data: "WAIT_TRUCK_ENTRY:IDLE:IDLE:IDLE"
```
**看到 `DISPATCH_TRUCK_LEAVE` 即表示一个完整卡车装载循环完成。**

### 15.3 运行中热切换场景（不重启）

```bash
# 切到 noisy_rtk：先设 ~select_name，再调 /scenario/select
rosparam set /scenario_manager/select_name noisy_rtk
rosservice call /scenario/select "{}"

# 或：改了当前 YAML 后原地重载
rosservice call /scenario/reload "{}"
```
`scenario_manager` 也支持私有参数覆盖单个字段（兼容旧 `scenario_pub`）：
`rosrun exca200_simulink scenario_manager.py _truck_near_x:=5.0 _rtk_bias_deg:=3.0`。

### 15.4 故障注入：让 dump 变慢 / 触发回归 FAIL

```bash
# 加大 boom 一阶惯性（执行变慢）
roslaunch exca200_simulink dump_viz_sim_single_trigger.launch rviz:=false tau_boom_s:=0.3 auto_loop_sec:=6.0
```
boom 响应变慢，`[phase_trigger:dump] cycle finished` 仍会出现但**耗时明显拉长**。把该 launch 的 `tau_boom_s` 默认改大 3× 后跑回归，即得到超差：
```
[regression] FAIL: dump_duration_sec: golden=15.438 actual=23.657 (diff=53.2%)
```
（这正是 §12 的验收自检；验证后记得改回 `0.0`。）命令冻结 / 关节卡死类故障走 `/plant/fault`（见 §9）。

### 15.5 跑回归测试（本机实测输出）

```bash
cd src/exca200_simulink
python3 tests/test_plant_step_response.py
python3 tests/test_dump_regression.py
python3 tests/test_full_cycle_regression.py
```
末行示例：
```
Ran 5 tests in 17.4s
OK
...
Ran 1 test in 27.4s
OK
[regression] PASS: {'success': True, 'dump_duration_sec': 15.443}
...
Ran 1 test in 45.0s
OK
[regression] PASS: {'success': True, 'cycles_completed': 1, 'mean_cycle_duration_sec': 30.2}
```
每个测试自起独立 master（§12），彼此顺序无关、可与日常 roscore 并存。`test_plant_step_response.py` 里偶见 `ResourceWarning` 属无害（service/socket 的 GC 提示）。

---

## 16. 读懂输出

### 16.1 终端前缀与关键字
roslaunch `output="screen"` 每行形如 `[INFO] [<ros秒>]: <消息>`。排错先 `grep` 关键字：`cycle finished`、`status=ok`、`TIMEOUT`、`below truck`、`out of`。

### 16.2 Decision 状态机标记（`excavator_system_decision`）

| 日志标记 | 含义 |
|---|---|
| `[TruckInteractionScenario] Started by /Mpc_Switch` | decision 收到启动信号 |
| `[TruckInteractionScenario] -> INITIAL_WAIT_TRUCK_ENTRY` | 等待卡车进场（回转对准中） |
| `[TruckInteractionScenario] -> TRUCK_POSITION_CHECK` | 卡车到位确认 |
| `[TruckInteractionScenario] -> DIGGING_CYCLE` | 进入装载循环 |
| `[DiggingCycleFsm] -> UNLOAD_PLAN` | 请求卸载规划 |
| `[DiggingCycleFsm] -> UNLOAD_EXECUTE` | 执行卸载（dump） |
| `[DiggingCycleFsm] -> RESET_EXECUTE` | 执行复位（reset） |
| `[DiggingCycleFsm] -> DIG_EXECUTE` | 执行挖掘（dig） |
| `[DiggingCycleFsm] -> LEVEL_TRUCK` | 整平阶段 |
| `[DiggingCycleFsm] -> SUCCESS` | 装载循环完成 |
| `[TruckInteractionScenario] -> DISPATCH_TRUCK_LEAVE` | **一圈完成标志** |
| `[TruckInteractionScenario] -> WAIT_TRUCK_ENTRY` | 等待下一辆卡车 |
| `[env_stub] truck event → ...` | environment_stub 推进卡车事件 |
| `[env_stub] FlagSmoothFinish=true` | 整平完成反馈 |

实时话题观察：`rostopic echo /system_decision/state_text`（latched，格式 `TruckStage:OpState:StagingPreMove:StagingCycle`）。

### 16.3 `/cycle_report`（JSON，仅 cycle_sequencer 旧路径）

> 注意：全循环 launch 已切换到 decision 节点，不再发布 `/cycle_report`。
> 以下仅适用于单独运行 `cycle_sequencer.py` 的场景（向后兼容/单阶段调试）。

```json
{"cycle_id": 1,
 "phase_durations_sec": {"dig": 8.2, "dump": 14.1, "reset": 0.6},
 "cycle_duration_sec": 24.4,
 "status": "ok",
 "timestamp": 1729000000.0}
```
`status ∈ {ok, timeout:<phase>, fault:<phase>}`。

### 16.4 `/system_decision/state_text`（Decision 路径）

格式：`"TruckStage:OperationState:StagingPreMoveState:StagingCycleState"`

示例值：
- `"INITIAL_WAIT_TRUCK_ENTRY:IDLE:IDLE:IDLE"` — 等待卡车进场
- `"DIGGING_CYCLE:UNLOAD_EXECUTE:IDLE:IDLE"` — 正在执行卸载
- `"DISPATCH_TRUCK_LEAVE:SUCCESS:IDLE:IDLE"` — 一圈完成，调度卡车离场
- `"WAIT_TRUCK_ENTRY:IDLE:IDLE:IDLE"` — 等待下一辆

实时观察：`rostopic echo /system_decision/state_text`。

### 16.5 完成信号判读
所有 `RefDeviceTraj_*`：`position.x=1` 执行中、`0` 结束（正常或异常中止）。dump 另有冗余 finish flag `/Sys_RUn_FlagUnloadExcuteFinish`；reset/dig 仅靠 `position.x` 的 `1→0` 下降沿判完成（契约见 `sim_arch.md §5`）。⚠️ 异常中止同样产生下降沿，故全循环要靠"当前阶段是否匹配"过滤——这正是 §13 里 `timeout:reset` 的根因。

### 16.6 RViz 观察
- `rviz:=true` 弹 RViz，布局见 `config/rviz_single.rviz`（单机 + marker）。
- **单机器人**：`robot1` = plant **实际反馈**（走 `/joint_states`，rad）。
- **指令 vs 反馈对比**：`traj_to_urdf_adapter` 发 `/Sys_RefDeviceTraj`（反馈）与 `/Sys_RefDeviceTraj2`（指令），二者应逐步收敛——差距大即跟踪滞后 / 故障显现。
- **truck_dump**：`truck_dump_viz_sim.launch` 叠加卡车 / 卸载点 marker（仅可视化）。

### 16.7 快速话题自检
```bash
rostopic echo -n1 /joints_angle          # boom/arm/bucket (deg) 在 x/y/z
rostopic echo -n1 /heading2swing_topic   # swing (deg)
rostopic echo /RefDeviceTraj_Unload      # dump 指令 + position.x 完成沿
rostopic hz /joints_angle                # plant 发布频率(默认 50Hz)
rosnode list                             # 确认节点名(故障参数前缀用)
```

---

## 附录 A · 命令速查

```bash
# 准备
source /opt/ros/noetic/setup.bash && cd ~/yx_planning_ws && catkin_make && source devel/setup.bash

# 单阶段（headless）
roslaunch exca200_simulink dump_viz_sim_single_trigger.launch rviz:=false auto_loop_sec:=6.0
roslaunch exca200_simulink dig_viz_sim.launch                 rviz:=false auto_loop_sec:=8.0
roslaunch exca200_simulink reset_viz_sim.launch               rviz:=false auto_loop_sec:=8.0
roslaunch exca200_simulink truck_dump_viz_sim.launch          rviz:=true

# 全循环（decision 编排）
roslaunch exca200_simulink full_cycle_sim.launch rviz:=false truck_full_threshold:=1
roslaunch exca200_simulink full_cycle_sim.launch rviz:=false   # 持续循环，Ctrl-C 停

# 批量
cd src/exca200_simulink && python3 scripts/run_batch.py --scenarios scenarios/ --repeats 3 --cycles 2

# 测试
cd src/exca200_simulink
python3 tests/test_plant_step_response.py
python3 tests/test_dump_regression.py
python3 tests/test_full_cycle_regression.py
python3 tests/test_dump_regression.py --update-golden      # 重建黄金基线

# 故障注入
rosparam set /exca_plant_sim/faults/joint_stuck/enabled true
rosservice call /plant/fault "data: true"

# 清理残留私有参数（手动污染时）
rosparam delete /exca_plant_sim ; rosparam delete /exec_sim
```

## 附录 B · 话题 / 单位 / 完成信号

见 [`sim_arch.md`](./sim_arch.md) §2–§5（坐标单位约定、话题契约表、各 planner 反馈依赖、`position.x` 完成语义）。本说明书不重复契约内容。
