# exca200_simulink

液压挖掘机**规划仿真 harness**：以纯运动学 + 执行机构延迟保真（无 Gazebo / 无物理引擎）
驱动 `dig / dump / reset / truck_dump` 四个规划器做**独立闭环回归**与**全循环集成**，
并提供 YAML 场景、故障注入、KPI 度量、批量回归与回归测试。

> 📚 文档三件套：
> - **本 README** —— 入口 / 概览 / 速查
> - [`docs/user_manual.md`](docs/user_manual.md) —— 详细操作手册（怎么跑、参数、排错、实操示例）
> - [`docs/sim_arch.md`](docs/sim_arch.md) —— 分层架构与话题/单位/完成信号**契约**

---

## 功能概述

| 能力 | 入口 | 状态 |
|------|------|------|
| 单阶段闭环回放 | `launch/{dig,dump,reset,truck_dump}_viz_sim*.launch` | ✅ |
| 全循环 dig→dump→reset→level | `launch/full_cycle_sim.launch` | ✅ |
| YAML 场景驱动 | `scenarios/*.yaml` + `scenario_manager.py` | ✅ |
| 被控对象保真（τ/速率/死区/噪声/延迟/settle） | `config/plant.yaml` + `exca_plant_sim.py` | ✅ |
| 故障注入（卡死/丢包/延迟尖峰/命令冻结） | `/plant/fault` 服务 | ✅ |
| 度量（每圈耗时/阶段耗时/状态/跟踪误差） | `/cycle_report`、`kpi_collector.py` | ✅* |
| 批量回归（场景 × 重复） | `scripts/run_batch.py` | ✅ |
| 回归测试（黄金基线 + master 隔离） | `tests/test_*.py` | ✅ |

\* `kpi_collector` 未默认挂到 `full_cycle_sim`，需 `metrics/*.csv` 时手动 `rosrun`（见手册 §10）。
dig 为**桩节点**（canned 动作），truck_dump **仅可视化叠加**，均不进真实闭环控制。

---

## 分层架构

```
+------------------------------------------------------------------+
| L6 入口层    launch/*.launch      run_batch.py                    |
| L5 度量层    kpi_collector.py     record_cycle.sh                 |
| L4 编排层    excavator_system_decision (C++)   environment_stub.py   phase_trigger.py |
| L3 桥接层    traj_to_urdf_adapter.py   sy200_urdf_control         |
| L2 场景层    scenario_manager.py   scenarios/*.yaml               |
| L1 被控对象  exca_plant_sim.py     config/plant.yaml              |
| L0 资产      urdf/exca870_single.urdf   meshes/   kinematics      |
+------------------------------------------------------------------+
```

数据主环：`decision → 激活话题 → planner → RefDeviceTraj_*(deg) → plant(一阶惯性/限速/死区/噪声/延迟) → joints_angle/heading2swing → decision + planner`；
旁路可视化：`adapter → Sys_RefDeviceTraj(rad) → RViz`；`environment_stub → 卡车事件/整地反馈 → decision`。
完整话题表、单位、frame、完成信号语义见 [`docs/sim_arch.md`](docs/sim_arch.md)。

---

## 目录结构

```
exca200_simulink/
├── README.md                    # 本文件（入口）
├── docs/
│   ├── sim_arch.md              # 架构 + 话题契约
│   └── user_manual.md           # 详细操作手册
├── config/
│   ├── plant.yaml               # L1 被控对象参数
│   ├── scenario_default.yaml    # L2 场景默认值
│   └── rviz.rviz / rviz_single.rviz
├── scenarios/                   # baseline / tight_truck / high_reach / noisy_rtk / saturated_valve
├── launch/
│   ├── dig_viz_sim.launch       # 单阶段（桩）
│   ├── dump_viz_sim*.launch     # 单阶段（真实 dump planner）
│   ├── reset_viz_sim.launch     # 单阶段（真实 reset planner）
│   ├── truck_dump_viz_sim.launch# 单阶段（仅 marker）
│   ├── full_cycle_sim.launch    # 全循环
│   └── display_rviz.launch
├── scripts/
│   ├── exca_plant_sim.py        # L1 被控对象
│   ├── exec_sim.py              # 旧壳 → import plant（向后兼容）
│   ├── scenario_manager.py      # L2 场景发布
│   ├── scenario_pub.py          # 旧壳 → import scenario_manager
│   ├── traj_to_urdf_adapter.py  # L3 deg→rad 可视化桥
│   ├── phase_trigger.py         # L4 单阶段触发助手
│   ├── dump_trigger.py          # 旧壳 → import phase_trigger
│   ├── cycle_sequencer.py       # L4 旧全循环状态机（保留用于向后兼容）
│   ├── environment_stub.py      # L4 decision 外部世界桩
│   ├── dig_planner_stub.py      # 挖掘阶段桩节点
│   ├── kpi_collector.py         # L5 度量落盘
│   ├── record_cycle.sh          # L5 rosbag 录制
│   ├── run_batch.py             # L6 批量回归
│   └── sim_clock.py             # L6 时钟加速（可选，未启用）
├── metrics/                     # 运行产物（kpi_collector / run_batch 写入）
├── tests/
│   ├── test_plant_step_response.py
│   ├── test_dump_regression.py
│   ├── test_full_cycle_regression.py
│   ├── _isolation.py            # 每个测试独立 ROS master
│   └── golden/                  # baseline_dump.json / full_cycle.json
└── urdf/                        # exca870.urdf（双机对比）/ exca870_single.urdf（单机+marker）
```

---

## 依赖与构建

- **ROS Noetic**（`rospy` / `roslaunch` / `rosrun` / `rosparam`），Python 3。
- 依赖同工作空间的规划包：`dump_trajectory_planner`、`reset_trajectory_planner`、
  `truck_dump_planner`、`kinematics`、`excavator_system_decision`（launch 通过 `<include>` 调用其 launch 文件）。
- 可视化：`robot_state_publisher`、`rviz`、`tf`。

```bash
source /opt/ros/noetic/setup.bash
cd ~/yx_planning_ws
catkin_make
source devel/setup.bash
rospack find exca200_simulink    # 校验包可见
```

---

## 快速开始

```bash
cd ~/yx_planning_ws && source devel/setup.bash

# dump 单阶段闭环（headless，约 15s；去掉 rviz:=false 可看 RViz）
roslaunch exca200_simulink dump_viz_sim_single_trigger.launch rviz:=false auto_loop_sec:=6.0

# 全循环（headless，decision 编排，truck_full_threshold:=1 → 1 斗即满）
roslaunch exca200_simulink full_cycle_sim.launch rviz:=false truck_full_threshold:=1

# 回归测试（自带 master 隔离，无需先起 roscore）
cd src/exca200_simulink && python3 tests/test_dump_regression.py
```

---

## 常见任务速查

| 想做的事 | 命令 |
|----------|------|
| 跑某单阶段 | `roslaunch exca200_simulink <dig\|dump_viz_sim_single_trigger\|reset\|truck_dump>_viz_sim.launch rviz:=false` |
| 周期重复触发 | 追加 `auto_loop_sec:=6.0` |
| 跑全循环（持续） | `roslaunch exca200_simulink full_cycle_sim.launch rviz:=false`（Ctrl-C 停） |
| 跑全循环（1斗满车） | `roslaunch exca200_simulink full_cycle_sim.launch rviz:=false truck_full_threshold:=1` |
| 指定场景 | `scenario_file:=$(rospack find exca200_simulink)/scenarios/noisy_rtk.yaml` |
| 加大执行延迟 | `... dump_viz_sim_single_trigger.launch tau_boom_s:=0.3` |
| 注入卡死故障 | `rosparam set /exca_plant_sim/faults/joint_stuck/enabled true; rosservice call /plant/fault "data: true"` |
| 批量回归 | `python3 scripts/run_batch.py --scenarios scenarios/ --repeats 3 --cycles 2` |
| 跑全部回归测试 | `python3 tests/test_plant_step_response.py && python3 tests/test_dump_regression.py && python3 tests/test_full_cycle_regression.py` |
| 重建黄金基线 | `python3 tests/test_dump_regression.py --update-golden` |

> 逐项参数表、场景字段、故障键、排错与**端到端实操示例（含真实终端输出）**见
> [`docs/user_manual.md`](docs/user_manual.md)。

---

## 输出与 KPI

- **终端**（`excavator_system_decision`）：`[TruckInteractionScenario] -> DISPATCH_TRUCK_LEAVE` 标志一个完整循环完成。
- **`/system_decision/state_text`**（latched String）：格式 `TruckStage:OpState:StagingPreMove:StagingCycle`。
- **旧路径**（`cycle_sequencer`，保留兼容）：`=== cycle N end: status=X duration=Ys ===`、`/cycle_report`（JSON）。
- **`kpi_collector`**（可选落盘）：`metrics/run_<ts>.csv|json`，列含阶段耗时、最大跟踪误差、命令计数等。
- **批量产物**：`metrics/batch_<ts>/<scenario>_<rep>/run_*.{csv,json}` + `summary.md`。

---

## 回归测试与黄金基线

| 测试文件 | 覆盖 | 容差 | 基线 |
|----------|------|------|------|
| `test_plant_step_response.py` | 限速/一阶惯性/死区/settle/joint_stuck（5 断言） | — | 无 |
| `test_dump_regression.py` | baseline dump 单阶段 | 5% | `golden/baseline_dump.json` |
| `test_full_cycle_regression.py` | baseline 全循环（decision 编排，检测 DISPATCH_TRUCK_LEAVE） | 10% | `golden/full_cycle.json` |

- 每个测试经 `tests/_isolation.py` 独占一个 **随机端口的独立 roscore**：顺序无关、可并行、不串参、无需外部 master。
- 首次运行自动生成基线；改预期行为后用 `--update-golden` 重建。
- 验收自检：把 `tau_boom_s` 临时调大 3× → `test_dump_regression.py` 应 **FAIL** 并打印超差指标。

当前基线：`dump_duration_sec≈15.4`、`mean_cycle_duration_sec` 待首次运行生成。

---

## 文档索引

| 文档 | 用途 |
|------|------|
| [`README.md`](README.md) | 入口 / 概览 / 速查（本文件） |
| [`docs/user_manual.md`](docs/user_manual.md) | 详细操作手册：运行、参数、场景、故障、度量、批量、测试、排错、实操示例 |
| [`docs/sim_arch.md`](docs/sim_arch.md) | 分层架构与话题/单位/完成信号契约（改动前必读） |

---

## 状态与已知限制

1. **dig 为桩**：以触发时刻实时姿态为 KF0，按 delta 叠加跑"远→近单向冲程"四段关键帧（起点=斗杆完全打开伸手够料、终点=斗杆垂直 θa=−90° + 铲斗端平 θk=−180°，**末尾不回 carry**）；关键帧绝对角按关节限位裁剪兜底，保证终点（=dump 的 WP1）不越限。dig→dump 由 dump planner 现读 dig 实时终点解耦交接。真实 `dig_trajectory_planner`（纯库 `dig_planner_dp`）尚未接入闭环。起点位姿由 `gp_dig_joints`（scenario）与 `plant.yaml` 初始位姿共同决定，需同步。
2. **truck_dump 仅可视化**：只发 marker，不发 `RefDeviceTraj_*`，不进 plant 闭环。
3. **`kpi_collector` 未默认挂 `full_cycle_sim`**：decision 路径通过终端日志观测，`metrics/*.csv` 需手动 `rosrun`。
4. **`use_sim_time` 关闭**：走 wall-clock；加速需引入 `sim_clock.py`。
5. **未纳入 git / CI**：harness 多为未跟踪文件；打 tag、`.github/workflows/sim-regression.yml` 均未做。
6. **搭台路径未覆盖**：`environment_stub` 默认 `/Mpc_Switch2=false`，staging 流程暂不仿真。

版本基线（tag 计划）见 [`docs/sim_arch.md §8`](docs/sim_arch.md)。
