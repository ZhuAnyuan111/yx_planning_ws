# clean_trajectory_planner

无人挖掘机**清台（地面平整 / 搭台）**轨迹规划与执行节点。

挖掘装车结束、卡车驶离后，决策层 `StagingCycleFsm` 进入搭台/清台循环。本节点以**中心线**（`/GP2mid/front_lane_deg`）± 左右偏移展开多条**径向条带**，逐条带遍历清台区域：每条带先由复位节点把挖机摆到**起点姿态**（`/clean_digPoint`），激活后命令**斗杆内收**到固定设定 `arm` 角，把挖掘区物料向机体方向刮拢规整。

起点姿态固定 `arm = 最大外伸极限`、`bucket = 0°`，并由**清台目标高度** `z`（`/Sys_SCo_Traj` 的 `position.y`）经 kinematics 正运动学（FK）**一维二分反解 `boom`**，使齿尖落到 `z`。内收过程 `boom`/`bucket` 每帧填实测角，由控制层齿尖高度闭环维持贴地。

节点采用**清晰的三层分层架构**（工具层 → 节点类层 → main 入口层），并以**单步执行 + 跨激活记忆**方式工作：每次激活只完成"当前条带"的内收，靠决策层 staging 外层循环驱动逐条带推进。

---

## 目录结构

```
clean_trajectory_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/clean_trajectory_planner/
│   ├── angle_utils.h                  # 纯 C++ 角度工具接口（无 ROS 依赖）
│   └── clean_trajectory_node.h        # CleanTrajectoryNode 类声明
├── src/
│   ├── angle_utils.cpp                # 角度工具实现
│   ├── clean_trajectory_node.cpp      # 节点类实现
│   └── clean_trajectory_node_main.cpp # 极简 main 入口
├── config/
│   └── clean_trajectory.yaml          # 节点参数配置
└── launch/
    └── clean_trajectory.launch        # 启动文件
```

**构建产物：**
- `libclean_trajectory_planner_lib.so`：工具 + 节点类共享库（可复用/单测）
- `clean_trajectory_node`：可执行文件

---

## 执行策略

### 总体流程

```
节点启动 → 读中心线 front_lane_deg → 生成径向条带序列 θ_0..θ_{N-1}
        → armed(kIdle)：反解 strip0 起点 boom，发布 /clean_digPoint(latched)
              │
              ▼（决策层 staging 外层循环逐条带驱动）
   复位节点(搭台模式 2.0)摆到 /clean_digPoint 起点姿态
              │
              ▼
   收到 /Sys_SLe_FlagGroundLevelExcute (data==1.0)
              │
              ▼
   前置校验（joints/swing 新鲜 + 当前条带起点姿态已就绪）
              │
   ┌──────────▼───────────┐
   │ kRetract             │  限速内收斗杆姿态角 → end_arm_attitude_deg
   │ 流式发 /RefDeviceTraj │  swing 保持 θ_i；boom/bucket 填实测角
   │ _Clean (Position.x=1)│
   │                      │
   │ 实测 arm 进入容差且  │
   │ 连续 confirm 帧 ─────┼──►┌──────────────────────────┐
   └──────────────────────┘   │ CompleteStrip            │
                              │ 末帧 Position.x=0        │
                              │ 还有剩余: strip_index++  │
                              │   反解+发下一条带         │
                              │   /clean_digPoint        │
                              │   /FlagCleanFinish=1.0   │
                              │ 无剩余: /FlagCleanFinish  │
                              │   =2.0 + 复位到 strip0   │
                              └──────────┬───────────────┘
                                         │
                              ┌──────────▼──────────┐
                              │ kStripDone          │  等待决策层撤销(flag=0)
                              │ 忽略重发的 1.0      │  → 回 kIdle(armed 下一条带)
                              └─────────────────────┘
```

### 清台区域几何（径向条带）

清台区域 = 中心线 swing 角 ± 左右偏移，按条带间隔展开为多条**径向条带**（每条带是一个固定 swing 角）：

```
中心线   θc = front_lane_deg            # 每次生成时 nh_.getParam 重新读取
条带序列 θ_i = θc − left_offset_deg + i·strip_step_deg,  i = 0..N−1
条带数   N = floor((left_offset + right_offset) / strip_step) + 1
遍历顺序 由当前实时 swing 自动决定：从最近区域边界开始（lo=θc−left / hi=θc+right，哪端近就从哪端起）
```

> `front_lane_deg` **直接是 swing 角（度）**，无需坐标转换（由 GP2mid 节点维护的全局 ROS 参数）。

### 起点姿态与 boom 高度反解

每条带起点姿态（发 `/clean_digPoint`，供复位节点搭台模式摆位）：

```
swing  = θ_i                            # 条带角，归一化 [0,360)
arm    = start_arm_deg                  # 斗杆最大外伸极限角（默认 kinematics arm_upper）
bucket = start_bucket_deg               # 固定 0°
boom   = SolveBoomForHeight(z, arm, bucket)   # 由清台目标高度反解
```

`boom` 反解：给定齿尖目标高度 `z`，`arm`/`bucket` 固定，求 `boom` 使 FK 高度等于 `z`：

```
f(boom) = pivot_z + Lb·sin(boom) + La·sin(boom+arm) + Lbt·sin(boom+arm+bucket) − z = 0
```

- FK 的 `z` 与 `swing` 无关，故 `swing` 不影响解；
- `arm`/`bucket` 固定 → **单变量方程**，在 `[boom_lower, boom_upper]` 内用**二分法**求解（FK z 对 boom 单调，端点夹逼即有根）；
- 端点同号（`z` 超出该 `arm`/`bucket` 下 boom 限位可达高度）→ `ROS_ERROR` 无解，该条带起点姿态不就绪，**不发布错误姿态、拒绝激活**；
- `z = /Sys_SCo_Traj position.y + height_offset_m`；`height_target_at_pivot=true` 时目标点取斗杆-铲斗铰接点（齿长项置 0）。

> **高度参考职责边界**：起点 `boom` 仅由本节点按名义高度 `z` 反解，供复位摆位；**内收过程的实际贴地高度由控制层基于齿尖高度误差闭环维持**（boom/bucket 锁实测值是有意设计，参见"接触式作业规划与控制解耦策略"）。

### 内收执行（kRetract）

| 项目 | 说明 |
|------|------|
| 命令内容 | arm = 限速逼近「斗杆姿态角」`end_arm_attitude_deg`（关节角目标 = 该姿态角 − 实测 boom）；swing = 保持条带角 `θ_i`；boom/bucket = **当前实测角** |
| 发布频率 | `publish_rate`（默认 10Hz），流式发 `/RefDeviceTraj_CleanUp`（Position.x=1） |
| 限速方式 | 每帧朝目标按 `vel·dt` 推进（arm 线性 `StepTowards`，swing 走最短角路径 `StepSwingTowards`） |
| 到位判据 | **实测斗杆姿态角**(boom+arm) 进入 `arm_tolerance_deg` 容差，连续 `confirm_frames` 帧（`retract_check_swing=true` 时同时校验 swing） |
| 完成动作 | 发末帧 Position.x=0 → 推进/收尾 → 上报 `/FlagCleanFinish` → 转 `kStripDone` |
| 超时保护 | 单条带超 `strip_timeout_sec`、全局超 `total_timeout_sec` 异常中止 |

### 单步执行 + 跨激活记忆

- 节点**每次激活只完成一条带**的内收，不维持内部循环；
- `strip_index_` 跨激活记忆遍历进度，由决策层 staging 外层循环（复位→清台→挖掘→卸载→复位…）驱动逐条带推进；
- `kStripDone` 门控：完成后**忽略决策层持续重发的 `flag=1.0`**，直到决策层撤销（`flag=0.0`）才回 `kIdle`，防止在决策层消费 `/FlagCleanFinish` 前抢跑下一条带；
- armed 空闲期（`kIdle`）按 `idle_refresh_sec` 节流重读中心线、按最新高度刷新当前条带 `/clean_digPoint`（带高度死区 `height_eps_m` 防抖）。

### 跨激活时序（与决策层 staging 外层循环衔接）

```
节点 armed → 发布 strip0 /clean_digPoint(latched)
决策: INITIAL_UNLOAD → RESET_EXECUTE(搭台2.0, 摆到 strip0 起点) → GROUND_LEVEL(flag=1.0)
节点: 内收 arm → strip0 完成 → 发布 strip1 /clean_digPoint + FlagCleanFinish=1.0 → kStripDone
决策: 消费 1.0 → flag=0.0(节点回 kIdle) → DIG_EXECUTE → UNLOAD_EXECUTE
      → RESET_EXECUTE(搭台2.0, 摆到 strip1 起点) → GROUND_LEVEL(flag=1.0)
节点: 内收 arm → strip1 完成 → 发布 strip2 /clean_digPoint + 1.0 → kStripDone
... 直到最后一条带 → FlagCleanFinish=2.0（并复位到 strip0 备用）
决策: 消费 2.0 → FINAL_DIG_EXECUTE → FINAL_BOOM_UP_WAIT → SUCCESS
```

> 关键：`/clean_digPoint` 必须 **latched** 且在每次条带完成后立即更新，保证下一次复位摆到正确条带起点。

---

## 话题接口

### 订阅

| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/joints_angle` | `geometry_msgs/Quaternion` | 实时关节角：x=boom, y=arm, z=bucket（度） |
| `/heading2swing_topic` | `std_msgs/Float32` | 实时回转角：data=swing（度） |
| `/Sys_SCo_Traj` | `geometry_msgs/Pose` | **position.y = 清台目标地面相对高度 z(m)**（既有语义 orientation.x=参考回转角，本节点不用） |
| `/Sys_SLe_FlagGroundLevelExcute` | `std_msgs/Float64` | 激活信号：`data==1.0` 执行清台 / `==0.0` 停止（主触发） |
| `/Sys_SeqAction` | `std_msgs/Float64` | `data==7.0` 清台阶段（并行指示，仅日志） |
| `/GP2mid/front_lane_deg` | ROS 参数(double) | 清台中心线 swing 角（度），`nh_.getParam` 读取 |

### 发布

| 话题名 | 消息类型 | 频率 | 说明 |
|--------|----------|------|------|
| `/clean_digPoint` | `geometry_msgs/Quaternion` | 事件（latched） | 当前条带起点姿态：x=swing, y=boom, z=arm, w=bucket（度） |
| `/RefDeviceTraj_CleanUp` | `geometry_msgs/Pose` | 10Hz | 参考轨迹帧，见下方字段定义 |
| `/FlagCleanFinish` | `std_msgs/Float32` | 事件（latched） | 1.0=本条带完成且还有剩余；2.0=全部条带完成（最终收尾）；0.0=未完成 |

**`/RefDeviceTraj_CleanUp` 字段映射：**

```
Position.x    = 1.0（执行中） / 0.0（结束/空闲）
Position.y    = 0（保留）
Position.z    = 0（保留）
Orientation.x = swing   （度，命令值=条带角）
Orientation.y = boom    （度，当前实测角）
Orientation.z = arm     （度，命令值，限速内收）
Orientation.w = bucket  （度，当前实测角）
```

> **注意**：`/FlagCleanFinish` 是 **Float32 三态**，区别于抹平的 `/FlagSmoothFinish`(Bool)。复位目标话题名是 `/clean_digPoint`（非 `/clean_dig_point`）。

---

## 参数配置

配置文件：`config/clean_trajectory.yaml`

### kinematics（运动学机构参数）

由 `KinematicsSolver` 从 `~/kinematics/...` 加载：连杆长度、动臂销轴偏置、各关节限位（deg）。本节点仅用其 **FK + 几何/限位**反解 boom，不做三级降级 IK。

### clean（清台区域与起点姿态）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `front_lane_param` | string | `/GP2mid/front_lane_deg` | 中心线 swing 角的全局 ROS 参数路径 |
| `ground_height_topic` | string | `/Sys_SCo_Traj` | 清台目标高度 z 来源话题（取 position.y） |
| `left_offset_deg` | double | 20.0 | 中心线左侧偏移（swing 度） |
| `right_offset_deg` | double | 20.0 | 中心线右侧偏移（swing 度） |
| `strip_step_deg` | double | 5.0 | 条带间隔（≈铲斗有效刮宽对应的回转角） |
| （遍历方向） | — | — | 由当前实时 swing 自动决定，从最近边界开始，无参数 |
| `start_arm_deg` | double | arm_upper | 起点 arm = 斗杆最大外伸极限角 |
| `start_bucket_deg` | double | 0.0 | 起点铲斗关节角（固定 0°） |
| `end_arm_attitude_deg` | double | -60.0 | 内收终点「斗杆姿态角」(boom+arm, deg)，非关节角（需标定到比起点更"收"） |
| `boom_solve_tol_m` | double | 0.01 | boom 二分反解的高度收敛容差 (m) |
| `boom_solve_max_iter` | int | 60 | 二分最大迭代次数 |
| `height_offset_m` | double | 0.0 | 高度基准偏置：`z = position.y + 偏置`（基准不一致时标定） |
| `height_target_at_pivot` | bool | false | 高度目标点：false=齿尖，true=斗杆-铲斗铰接点 |
| `height_eps_m` | double | 0.02 | 空闲期重发 /clean_digPoint 的高度死区 (m)，防抖 |

### 运动与判定

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `publish_rate` | double | 10.0 | 逐帧发布频率 (Hz) |
| `idle_refresh_sec` | double | 1.0 | armed 空闲态刷新 /clean_digPoint 的节流周期 (s) |
| `arm_vel_dps` | double | 15.0 | arm 命令限速 (deg/s) |
| `swing_vel_dps` | double | 15.0 | swing 命令限速 (deg/s) |
| `arm_tolerance_deg` | double | 5.0 | arm 到位容差 (deg) |
| `swing_tolerance_deg` | double | 5.0 | swing 到位容差 (deg) |
| `confirm_frames` | int | 3 | 到位需连续确认帧数（防液压振荡误判） |
| `retract_check_swing` | bool | false | 内收到位是否同时校验 swing（默认仅 arm 为主判据） |

### 超时与调试

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `strip_timeout_sec` | double | 8.0 | 单条带内收超时 |
| `total_timeout_sec` | double | 12.0 | 全局执行超时（单次激活，应 < 决策层 ground_level 超时） |
| `input_timeout_sec` | double | 1.0 | 输入话题最大静默时长，≤0 关闭检查 |
| `enable_debug_log` | bool | true | 开启 `[dbg]` 详细调试日志：条带生成、boom 反解收敛快照、内收心跳（fb/tgt/cmd/err/dt/elapsed，0.5s 节流）与到位计数跳变事件。**上生产置 false**，仅保留状态转换类 INFO；等待类告警统一 2s 节流防刷屏 |

---

## 编译与运行

### 编译

```bash
cd ~/yx_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg clean_trajectory_planner
```

### 运行

```bash
source devel/setup.bash
roslaunch clean_trajectory_planner clean_trajectory.launch
```

### 手动测试

```bash
# 终端1：启动节点
roslaunch clean_trajectory_planner clean_trajectory.launch

# 终端2：设置中心线参数 + 模拟实时反馈
rosparam set /GP2mid/front_lane_deg 90
rostopic pub -r 10 /joints_angle geometry_msgs/Quaternion '{x: 0.0, y: -30.0, z: 0.0, w: 0.0}'
rostopic pub -r 10 /heading2swing_topic std_msgs/Float32 "data: 70.0"

# 终端3：模拟清台目标高度（position.y = 地面相对高度 z）
rostopic pub -r 2 /Sys_SCo_Traj geometry_msgs/Pose \
  '{position: {x: 0.0, y: -1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}'

# 观察 armed 期发布的起点姿态（boom 由 z 反解）
rostopic echo /clean_digPoint

# 终端4：激活清台（flag=1.0），观察 arm 内收
rostopic pub /Sys_SLe_FlagGroundLevelExcute std_msgs/Float64 "data: 1.0"
rostopic echo /RefDeviceTraj_CleanUp
rostopic echo /FlagCleanFinish

# 撤销（模拟决策层消费完成后 flag=0）→ 节点回 kIdle，armed 于下一条带
rostopic pub /Sys_SLe_FlagGroundLevelExcute std_msgs/Float64 "data: 0.0"
```

> 配合 `excavator_system_decision/scripts/decision_test_gui.py`（已有 Clean Finish 0/1/2 按钮）可验证与决策层 staging 的衔接。

---

## 边界情况处理

| 情况 | 处理方式 |
|------|----------|
| 中心线参数 `front_lane_deg` 未就绪 | 不生成条带，节流 WARN；armed 期持续重试 |
| 清台高度 `/Sys_SCo_Traj` 未就绪 | 不发布 /clean_digPoint，节流 WARN；拒绝激活 |
| `z` 超出 boom 限位可达高度 | `SolveBoomForHeight` 返回失败，`ROS_ERROR`；该条带起点姿态不就绪，拒绝激活（不摆错误姿态） |
| joints/swing 反馈未到达或断流 | 前置校验不通过 / 执行期新鲜度校验失败 → 拒绝激活或异常中止 |
| 当前条带起点姿态未就绪却收到激活 | 即时补发 /clean_digPoint，仍失败则节流 WARN 拒绝激活 |
| 内收卡死/过慢 | 超 `strip_timeout_sec` 异常中止（保持末指令 + Position.x=0 + Finish=0.0 + kFailed） |
| 单次激活卡死 | 全局 `total_timeout_sec` 统一保底中止 |
| 到位瞬间液压振荡 | 需连续 `confirm_frames` 帧进入容差才判定到位，防误判 |
| 内收中 `flag` 变 0.0 | 立即停止，保持末指令并发 Position.x=0、Finish=0.0，回 kIdle（**同一条带不推进**，可重试） |
| 完成后决策层持续重发 `flag=1.0` | `kStripDone` 门控忽略，直到 `flag=0.0` 才回 kIdle，防抢跑下一条带 |
| 异常态(kFailed) 收到 `flag=0.0` | 恢复 kIdle，重试当前条带 |
| 中心线在 armed 期被 GP2mid 更新 | 节流重读，变化则重新生成条带并刷新 /clean_digPoint（clamp 保留进度） |

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
│ Layer 3: clean_trajectory_node_main.cpp          │  ← 极简入口
│   int main(...) { init + create + spin }         │
└──────────────────────────────────────────────────┘
                     ▲ 链接
┌──────────────────────────────────────────────────┐
│ Layer 2: clean_trajectory_node.h/.cpp            │  ← ROS 节点类
│   CleanTrajectoryNode：订阅/发布/条带/boom反解/  │
│   状态机                                          │
└──────────────────────────────────────────────────┘
                     ▲ 依赖
┌──────────────────────────────────────────────────┐
│ Layer 1: 纯 C++ 工具层（无 ROS 依赖，可单测）    │
│   angle_utils.h/.cpp + kinematics 运动学库(FK)   │
└──────────────────────────────────────────────────┘
```

**收益：** 单向依赖无循环；工具层可独立单测；节点类可作为库复用；main 极简，只关心 ROS 生命周期。

---

## 命名规范

遵循 Google C++ Style：

| 类别 | 规范 | 示例 |
|------|------|------|
| 文件名 | snake_case | `clean_trajectory_node.cpp` |
| 类名 | PascalCase | `CleanTrajectoryNode` |
| 函数名 | PascalCase | `ExecuteRetract()` |
| 成员变量 | snake_case_ | `strip_index_` |
| 常量 | kCamelCase | `kExecuteValue` |
| 枚举值 | kCamelCase | `CleanPhase::kRetract` |

---

## 依赖

- ROS Noetic
- roscpp
- geometry_msgs
- std_msgs
- kinematics（运动学求解库，仅用 FK + 几何/限位）

与其他规划包零耦合，可独立编译、部署和升级。

---

## 系统集成

```
系统决策节点 ── /Sys_SLe_FlagGroundLevelExcute(1.0/0.0) ──► clean_trajectory_node
（StagingCycleFsm::                /Sys_SeqAction(7.0)            │
  GROUND_LEVEL_EXECUTE）                                          │
              ◄── /RefDeviceTraj_CleanUp (Position.x=1/0) ───────┤
              ◄── /FlagCleanFinish (Float32: 1.0/2.0/0.0) ────────┤
              ◄── /clean_digPoint (latched) ─────────────────────┘
                          │
                          ▼
              reset_trajectory_planner（搭台模式 2.0，摆到条带起点）
```

本节点遵循系统决策节点统一调度架构，**单步执行**（激活→内收一条带→上报），由决策层 staging 外层循环驱动逐条带推进：

- 决策层 `StagingCycleFsm` 的 `GROUND_LEVEL_EXECUTE` 发布 `seq_action=7.0` + `flag_ground_level_execute=1.0`，等待 `/FlagCleanFinish`；
- 本节点完成一条带后回发 `/FlagCleanFinish=1.0`（还有剩余）或 `2.0`（全部完成），决策层据此推进/收尾；
- 全局超时（默认 12s）应严格小于决策层 ground_level 超时，确保异常时先于上层安全退出。

> **决策层依赖（需用户自行修改，本包不实施）**：`staging_cycle_fsm.cpp` 的 `RESET_EXECUTE` 在清台前需发 `flag_reset_execute=2.0`（搭台模式），reset 节点才会以 `/clean_digPoint` 为目标摆位（当前发 1.0 走挖掘循环模式 `/GP_dig_joints`）。
