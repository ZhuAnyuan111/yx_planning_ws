# traj_fusion_pkg

无人挖掘机 **轨迹融合层（L2）** 节点，是整个系统的"总装车间"。它同时汇聚 **传统规则规划** 与 **SmolVLA 大脑学习策略** 两条通道，按决策层下发的阶段号做仲裁、限幅、安全兜底，输出唯一一条下发给下游执行层的融合轨迹 `/Sys_SCo_Traj`。

- **节点版本**：`2026.09.08`（硬编码在 [traj_fusion_main.cpp](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/traj_fusion_main.cpp)）
- **主要维护**：wangxing（ACT 融合框架）、wuql7（大脑推理/reward）、杏姐（车道-圆交点、发布几何）
- **主循环频率**：10 Hz，`AsyncSpinner(2)` 多线程回调

---

## 目录结构

```
traj_fusion_pkg/
├── include/
│   ├── traj_fusion_node.hpp             # 类定义（606 行）：30+ 订阅 / 15+ 发布 / 500+ 成员
│   ├── stb_image.h                       # 第三方：图像解码
│   └── stb_image_resize2.h               # 第三方：图像缩放
├── src/
│   ├── traj_fusion_main.cpp              # main 入口（18 行）
│   ├── traj_fusion_node.cpp              # 构造函数：参数加载 + 话题装配（256 行）
│   ├── act_traj_fusion.cpp               # 核心：状态机 / 融合 / 安全（2804 行）
│   └── truck_generator_node.cpp          # 矿卡几何：顶点/卸载点/车道-圆交点（754 行）
```

> ⚠️ **注意**：当前工作区内该目录**缺少** `CMakeLists.txt` / `package.xml` / `launch/` / `config/`，属于源码快照状态。编译前需补建 catkin 包骨架，或参考 `~/yx_planning_ws/src/action_fused_pyws/` 里的部署配置。

---

## 一、系统上下文：双通道融合架构

```
                    ┌────────────────────────────────────┐
                    │   L4 决策 (GP2mid + decision)      │
                    │   /Sys_SeqAction (阶段号 1~7.5)   │
                    │   /Sys_RGp_SwWkAgl /Sys_RGp_Unload│
                    │   /Mpc_Switch /Mpc_Switch2         │
                    └───────────┬────────────────────────┘
                                │ 阶段选择
                                ▼
     ┌───────────────────────────────────────────────────────┐
     │              ★ traj_fusion_pkg (本包) ★                │
     │                                                        │
     │  传统通道 (5路)              学习通道 (1路)             │
     │  /RefDeviceTraj_Dig         /smolVLA_output_angle      │
     │  /RefDeviceTraj_Unload      /smolVLA_status_check      │
     │  /RefDeviceTraj_Reset              ↑                   │
     │  /RefDeviceTraj_Smooth        来自数采节点 data_to_nas │
     │  /RefDeviceTraj_CleanUp              (大脑 HTTP 返回)  │
     │                                                        │
     │  本体反馈: /joints_angle /heading2swing_topic          │
     │           /rtk_can_msg /RealBktPosXYZ /ef_tip_enu_1    │
     │  矿卡感知: /truck_pose_topic /truck_info_topic         │
     │           /truck_location_xyz_topic                    │
     │                                                        │
     │  主循环 10Hz: 阶段分派 → clamp 限位 → 变化率限制        │
     │              → 心跳拼装 → 发布                          │
     └────────────────┬───────────────────────────────────────┘
                      │
                      ▼  /Sys_SCo_Traj (统一融合轨迹)
              ┌───────────────────────────┐
              │ L1 action_fused_pyws      │
              │  → CAN/电液 下位机         │
              └───────────────────────────┘
```

**核心定位**：本包是**唯一有权生成 `/Sys_SCo_Traj` 的节点**。所有轨迹规划节点的输出都必须经它仲裁才能下发。它同时反哺部分几何信息（卡车顶点、卸载点、回转方向标志、车道-圆交点）给上游规划节点和下游可视化。

---

## 二、状态机与主循环

### 2.1 traj_status 映射表

`/Sys_SeqAction` (std_msgs/Float64) 的取值经 `sysSeqActionCallback`（[act_traj_fusion.cpp#L756](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/act_traj_fusion.cpp)）映射到内部 `traj_status` 整数：

| SeqAction 值 | traj_status | 语义 | 处理函数 |
|--------------|-------------|------|---------|
| 1.0 | 1 | 初始化 | `initStatusProcess` |
| 2.0 | 2 | 复位执行 | `resetStatusProcess` |
| 2.5 | 13 | 复位结束保持 | `current_traj = last_traj`，z=2.5 |
| 3.0 | 3 | 挖掘执行 | `digStatusProcess` 或 `digStatusProcess_model` |
| 3.5 | 9 | 挖掘结束保持 | z=3.5 |
| 4.0 | 4 | 提升+卸载 | `raiseAndUnloadStausProcess` |
| 4.5 | 8 | 卸载结束保持 | z=4.5 |
| 5.0 | 5 | 等待 | `waitStatusProcess` |
| 5.5 | 10 | 等待结束保持 | z=5.5 |
| 6.0 | 6 | 抹平执行 | `smoothStatusProcess` |
| 6.5 | 11 | 抹平结束保持 | z=6.5 |
| 7.0 | 7 | 清台执行 | `cleaupStatusProcess` |
| 7.5 | 12 | 清台结束保持 | z=7.5 |
| 其他 | 14 | 兜底 | `current_traj = last_traj`，z=0.5 |

**关键点**：
- `x.5` 结束态不重规划，直接沿上一帧下发；仅通过 `position.z` 通知下游当前处于哪个"结束保持"
- `position.z` 语义 = "刚完成的阶段" 或"兜底"，供 `action_fused` 与数采判断阶段边界

### 2.2 主循环 10Hz 流程

`run()` 位于 [traj_fusion_node.cpp#L216](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/traj_fusion_node.cpp)：

```
每帧：
  1) heart_value++ (0~255 循环)
  2) assembleFaultCode(heart_value, faultMask) → /trajfusionSafety_topic
  3) processACTOutput()  ← 状态分派 + clamp + 变化率限制 + publish /Sys_SCo_Traj
  4) reward_.store(0)    ← 大脑推理评分清零（供离线分析）
```

`processACTOutput()` 详细流程（[act_traj_fusion.cpp#L2488](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/act_traj_fusion.cpp)）：

1. **状态计时**：`traj_status` 变化时刷新 `state_start_time_`；`==3` 额外刷 `dig_stage_start_time_`；清除 FAULT_5 超时位
2. **switch(traj_status) 分派到 7 个阶段处理函数**
3. **关节角 clamp**：调用 `clampTrajAngles(traj, stage)`（stage=2/3/4 分别用 `limits_reset_/limits_dig_/limits_unload_` 限 boom/arm/bucket；swing 目前不 clamp）
4. **回转方向发布**：根据阶段选 `swing_clockwise_2 / 4 / 5` → `/swing_clockwise_flag`
5. **齿尖-矿卡距离计算**：`bucket_to_truck` / `bucket_to_truck_ground` 写入 `position.x / position.y` 供下游门控使用
6. **铲斗变化率限幅**：`w` 增量 ≤ `bucket_change_rate`（防抖）
7. **发布** `/Sys_SCo_Traj` + `/2stage_traj_topic`
8. **离开复位/卸载阶段时的状态清理**：`reset_swing_*` / `unload_swing_*` 一堆标志复位
9. **FAULT_6 线缆避让同步**：仅在离开 status 2/4 后清零，避免心跳里持续挂着

---

## 三、输入接口清单（订阅）

### 3.1 阶段激活（决策层）

| 话题 | 类型 | 语义 | 回调 |
|------|------|------|------|
| `/Sys_SeqAction` | `std_msgs/Float64` | 阶段激活（见 2.1 表） | `sysSeqActionCallback` |
| `/Mpc_Switch` | `std_msgs/Bool` | 大脑/传统通道总开关 | `mpcSwitchCallback` |
| `/Mpc_Switch2` | `std_msgs/Bool` | 搭台/清台模式请求（决策层 StagingCycleFsm 消费） | `mpcSwitch2Callback` |

### 3.2 传统通道：5 路轨迹规划输入（都是 `geometry_msgs/Pose`，Orientation=swing/boom/arm/bucket deg）

| 话题 | 上游节点 | 用途 |
|------|---------|------|
| `/RefDeviceTraj_Dig` | dig / GP2mid | 挖掘轨迹 |
| `/RefDeviceTraj_Unload` | dump_trajectory_planner | 卸载轨迹 |
| `/RefDeviceTraj_Reset` | reset_trajectory_planner | 复位轨迹 |
| `/RefDeviceTraj_Smooth` | smooth_trajectory_planner | 抹平轨迹 |
| `/RefDeviceTraj_CleanUp` | clean_trajectory_planner | 清台轨迹 |
| `/Sys_RUn_UnloadEndAngle` | dump | 卸载结束姿态（额外约束） |

### 3.3 学习通道：SmolVLA 大脑（由数采节点 `data_to_nas` 发布）

| 话题 | 类型 | 语义 |
|------|------|------|
| `/smolVLA_output_angle` | `geometry_msgs/Pose` | 大脑推理关节角（**rad**，需转 deg） |
| `/smolVLA_status_check` | `std_msgs/Int8` | 大脑状态码 0~6，非 0 触发对应 FAULT_0~5 |

### 3.4 决策点（GP2mid 发布）

| 话题 | 类型 | 语义 |
|------|------|------|
| `/Sys_RGp_SwWkAgl` | `geometry_msgs/Point` | 挖掘点回转+挖深+姿态角 |
| `/Sys_RGp_Unload` | `geometry_msgs/Quaternion` | 卸载点（清台模式） |
| `/clean_digPoint` | `geometry_msgs/Quaternion` | 清台起点位姿（reset 双模式） |
| `/dig_region_reference_angles` | `geometry_msgs/Point` | 挖掘区域参考回转角区间（双阈值 [lb1,ub1]/[lb2,ub2]） |

### 3.5 本体感知

| 话题 | 类型 | 语义 |
|------|------|------|
| `/joints_angle` | `geometry_msgs/Quaternion` | boom/arm/bucket 当前角（**deg**，xyz；w 保留） |
| `/heading2swing_topic` | `std_msgs/Float32` | RTK 航向转 base 系回转角（0~360°） |
| `/rtk_can_msg` | `interfaces/rtk_cgi610` | 挖机 RTK 经纬高+航向 |
| `/RealBktPosXYZ` | `geometry_msgs/Point` | 铲斗齿尖实时 base 系 xyz（下位机 FK） |
| `/ef_tip_enu_1` | `interfaces/SinglePoint` | 齿尖 ENU 坐标 |
| `/bucket_terrain_delta_position` | `geometry_msgs/PointStamped` | 铲斗-地面高差 |

### 3.6 矿卡感知（truck_generator 消费）

| 话题 | 类型 | 语义 |
|------|------|------|
| `/truck_pose_topic` | `geometry_msgs/PoseStamped` | 卡车主天线 RTK 位姿 |
| `/truck_info_topic` | `geometry_msgs/PoseStamped` | 卡车完整位姿（含 pitch/roll） |
| `/truck_location_xyz_topic` | `std_msgs/Int8` | 卡车位置状态（1=到位、2=驶离 等） |

---

## 四、输出接口清单（发布）

| 话题 | 类型 | 频率 | 语义 | 下游消费者 |
|------|------|------|------|-----------|
| **`/Sys_SCo_Traj`** | `geometry_msgs/Pose` | 10 Hz | **融合后的唯一下发轨迹**（Orientation=swing/boom/arm/bucket deg；Position.x=齿尖到厢面距离、y=到厢底距离、z=阶段标识） | action_fused, 数采, 可视化 |
| `/swing_clockwise_flag` | `std_msgs/Bool` | 10 Hz | 当前阶段回转方向（true=顺时针） | reset / dump 轨迹节点 |
| `/truck_center_unloadpoint` | `geometry_msgs/Quaternion` | 触发式 | 车厢中心卸载点（RTK 高度+航向 α） | dump 轨迹节点 WP4 计算 |
| `/truck_center_unloadpoint_2` | `geometry_msgs/Quaternion` | 触发式 | 后轴中心沿航向前推 `unload_forward_offset` 的远点 | dump 轨迹节点 |
| `/truck_center_point` | `geometry_msgs/Quaternion` | 触发式 | 车厢几何中心（不含高度偏置） | 可视化/规划 |
| `/truck_vertices_topic` | `interfaces/truckVertices` | 触发式 | 车厢 4 个顶点（LLA 或 base 系） | 碰撞检测 / RViz |
| `/lane_circle_intersections` | `geometry_msgs/PoseArray` | 触发式 | 中/左/右车道线 与 回转半径圆 的 3 个交点 | GP2mid 挖掘点选择 |
| `/truck_swive_angle` | `std_msgs/Float32` | 阶段 4 | 车尾目标回转角（含 `truck_tail_offset_from_unload_deg_` 偏置） | reset 双模式 |
| `/trajfusionSafety_topic` | `std_msgs/UInt32` | 10 Hz | 高 16=心跳计数 / 低 16=故障位掩码 | 数采 / 监控 / 决策 |
| `/fault_require_takeover` | `std_msgs/Bool` | 触发式 | 人工接管请求 | 决策 / HMI |
| `/2stage_traj_topic` | `geometry_msgs/Pose` | 10 Hz | 二段轨迹调试（x=unload_angle, y=front_lane, z=rear_cable） | 可视化 |

---

## 五、各阶段处理函数详解

### 5.1 `initStatusProcess` — 阶段 1

- 首次进入锁定当前 swing/boom/arm/bucket（`init_stage1_angles_captured_`）
- 保持四关节为进入时的角度整段下发
- **用途**：等待决策层激活下一阶段；作为上电或异常后的"零速保持"

### 5.2 `resetStatusProcess` — 阶段 2

**主要职责**：把挖机从当前位置（多在卸载/挖掘结束姿态）平滑回到**复位待机位**。

- **回转单调展开**：`reset_swing_unwrap_offset_deg_` + `reset_swing_phase_` 保证跨 ±180° 边界时不折返
- **回转方向**：仅在进入本阶段首帧计算一次 `swing_clockwise_2`（最短弧）
- **齿尖强制垂直门控**：`bucket_terrain < reset_bucket_ground_dist_thresh(默认0.1m)` 且回转距目标 ≤ `RESET_SWING_ANGLE_THRESH(15°)` 时，`arm` 强制拉到 `ARM_VERTICAL_ANGLE(-120°)`
- **矿卡避让**：`reset_swing_cable_first_flag_` 判定回转路径先经过线缆危险区则触发 `FAULT_6` + 请求接管
- **卡车驶离门控**：`reset_truck_exit_thresh(默认 30°)` 判定回转是否已越过卡车中心

### 5.3 `digStatusProcess` / `digStatusProcess_model` — 阶段 3

两条支路由 `/use_dig_status_process_model` 参数切换：

- **传统支路 `digStatusProcess`**：直接使用 `traj_dig`（GP2mid 输出的挖掘轨迹）
- **模型支路 `digStatusProcess_model`**：使用 `model_traj`（ACT/SmolVLA），带 `act_boom_angle / act_arm_angle / act_bucket_angle` 三个弧度→角度转换（[act_traj_fusion.cpp#L1457](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/act_traj_fusion.cpp)）

**共用逻辑**：
- **憋压检测** `dig_OP_detection`：三关节平滑角速度 EMA 均低于 `dig_joint_vel_thresh` → `dig_OP_level_` 升到 1（等级），配合 `dig_traj_adjust` 微调 boom
- **动臂自由期** `dig_boom_free_sec`（默认 10s）：进入挖掘后前 10s 不做积分干预，之后按 `dig_boom_integrate_rate_deg_per_sec` 缓慢调整
- **每阶段一次性 `stop_swing_main`**：计算回转停止主判据（避免挖掘中反复算），`stop_swing_main_done_in_dig_stage_` 标志位保证只算一次

### 5.4 `raiseAndUnloadStausProcess` — 阶段 4

**主要职责**：提升+回转+卸料一体。此阶段包含大量**矿卡几何仲裁**：

- **两段式**：Seg0 提升到 `unload_boom_angle`（由 dump 上游 `middle_UP` 提供）→ Seg1 回转→ Seg2 卸料
- **回转越过卡车目标锁存**：`unload_swing_passed_truck_target_latch_` 高速跳 epsilon 带兜底
- **铲斗角双策略**：接近目标角时（差 ≤ `unload_bucket_planner_enter_deg_(8°)`）切规划器给的 `w`；否则用端平角 `bucket_flat_angle`（`calculateBucketFlatAngle` 每帧算）
- **动臂保持**：`Seg0 → Seg1` 过渡要求 boom 到位带容差，防止大幅甩动
- **卡车顶点重算门控**：`truck_dock_success_flag_` 下 w==2 每帧刷新；w==3 距上次位置 > 0.5m 或航向 > 2° 才重算（`kUnloadRecalcDistM_` / `kUnloadRecalcHeadingDeg_`）

### 5.5 `waitStatusProcess` — 阶段 5

- 挖一斗后的等待（等待卡车调整位置）
- `bucket_ahead_unload_deg(默认20°)` 提前把铲斗摆到卸载方位角附近预备

### 5.6 `smoothStatusProcess` — 阶段 6

- 直接使用 `traj_smooth`（`smooth_trajectory_planner` 输出）
- 数采/大脑通道关闭时仍工作

### 5.7 `cleaupStatusProcess` — 阶段 7

- 使用 `traj_cleanup`（`clean_trajectory_planner` 输出）
- 需要 `/clean_digPoint` 有效（`clean_dig_point_angle_valid_`）

---

## 六、参数配置（40+ 项，全部通过 `ros::param` 全局命名空间读取）

> ⚠️ 目前无独立 yaml，全部通过 `<param name="/xxx" value="..."/>` 塞进 launch；或修改默认值后重编译。

### 6.1 通道切换 / 主循环

| 参数 | 默认 | 说明 |
|------|------|------|
| `/use_dig_status_process_model` | false | 阶段 3 是否走模型支路 |
| `/dig_boom_free_sec` | 10.0 | 挖掘阶段动臂积分前的自由时间（秒） |
| `/dig_boom_integrate_rate_deg_per_sec` | 1.0 | 超时后动臂积分速度（°/s） |

### 6.2 挖掘憋压检测

| 参数 | 默认 | 说明 |
|------|------|------|
| `/arm_target_threshold` | 70.0 | 斗杆到位阈值 |
| `/bucket_offset` | 5.0 | 铲斗补偿角 |
| `/boom_adjust_range` | 5.0 | 动臂微调范围 |
| `/angle_arm_thresh` | 5.0 | 斗杆角度阈值 |
| `/dig_v_arm_ub` | 20.0 | 斗杆角速度上界（deg/s） |
| `/dig_v_arm_lb` | 10.0 | 斗杆角速度下界 |

### 6.3 本体几何（**毫米**）

| 参数 | 默认 | 说明 |
|------|------|------|
| `/QV` (bucket_length) | 1464.4056 | 铲斗长度 |
| `/CF` (boom_length) | 5703.5758 | 动臂长度 |
| `/FQ` (arm_length) | 2929.9387 | 斗杆长度 |
| `/RO_L` (exc_center_W_L) | 2.007 | 回转中心到铲斗纵向偏置 |
| `/W_H` (exc_center_W_H) | 1.204 | 横向偏置 |
| `/W_W` (exc_center_W_W) | 0.778 | 高度偏置 |
| `/exc_bucket_H` | 0.5 | 铲斗参考高度 |

### 6.4 卸载阶段

| 参数 | 默认 | 说明 |
|------|------|------|
| `/unload_height_offset` | 1.0 | 卸载点高度偏置（m） |
| `/unload_forward_offset` | 0.5 | 卸载点 2 相对后轴中心沿航向前推距离（m） |
| `/unload_bucket_planner_enter_deg_` | 8.0 | 距卸载回转角 ≤ 该值切规划铲斗角 |
| `/truck_tail_offset_from_unload_deg_` | 30.0 | 车尾目标相对卸载方位角偏置 |
| `/bucket_ahead_unload_deg` | 20.0 | 铲斗提前卸载角 |
| `/truck_edge_offset` | 0.1 | 卡车边偏置 |

### 6.5 复位阶段

| 参数 | 默认 | 说明 |
|------|------|------|
| `/reset_bucket_ground_dist_thresh` | 0.1 | 齿尖-地面高差阈值（m） |
| `/reset_truck_exit_thresh` | 30.0 | 复位阶段卡车驶离判定阈值（°） |

### 6.6 各阶段关节限位（度，仅 boom/arm/bucket 生效，swing 目前不 clamp）

`/reset_swing_min/max`、`/reset_boom_min/max`、`/reset_arm_min/max`、`/reset_bucket_min/max`；`/dig_*`、`/unload_*` 三套独立。默认 boom [-30, 90]、arm [-180, 0]、bucket [-180, 90]。

### 6.7 车道-圆交点（无矿卡 RTK 时的兜底）

| 参数 | 默认 | 说明 |
|------|------|------|
| `/GP2mid/front_lane_deg` | 0.0 | 中车道航向（同时用作挖一斗等待角） |
| `/GP2mid/rear_cable_deg` | 0.0 | 线缆危险区回转角 |
| `/lane_offset_x_m` | 3.0 | 侧向偏移 |
| `/circle_radius_y_m` | 7.0 | 回转半径 |

### 6.8 其他

`/bucket_to_angle` 0.0 / `/digover_boom` 20.0 / `/relative_z` 0.8

---

## 七、安全机制

### 7.1 心跳与故障位掩码

`/trajfusionSafety_topic` (`std_msgs/UInt32`) = `[高 16 位: heart_value 0~255][低 16 位: faultMask]`

`assembleFaultCode(high16, faultMask)` 拼装；`addFaultCode/clearFaultCode/hasFault` 逐位操作。

### 7.2 故障码定义（FaultCode 枚举，16 位 bitmask）

| 位 | 触发条件（当前源码实际使用） | 关联阶段 |
|----|----------------------------|----------|
| FAULT_0 | `smolVLA_status == 2` 大脑模型异常 | 全阶段 |
| FAULT_1 | `smolVLA_status == 3` gridmap 数据丢失 | 全阶段 |
| FAULT_2 | `smolVLA_status == 4` range 前向灰度图丢失 | 全阶段 |
| FAULT_3 | `smolVLA_status == 5` camera0 丢失 | 全阶段 |
| FAULT_4 | `smolVLA_status == 6` camera1 丢失 | 全阶段 |
| FAULT_5 | 大脑状态码保留位（超时逻辑当前注释） | 保留 |
| FAULT_6 | 复位/卸载阶段回转路径**穿过线缆危险区** | 阶段 2 / 4 |
| FAULT_7~15 | 预留 | — |

### 7.3 状态超时

`STATE_TIMEOUT_SEC = 20.0`：单一 `traj_status` 停留 > 20s 会（当前**注释掉了** FAULT_5 触发，仅作计时与日志）。挖掘阶段另有 `dig_stage_start_time_` 独立计时。

### 7.4 人工接管

`/fault_require_takeover` (Bool)：`require_takeover=true` 时发布；接管计数 `takeover_pub_count_` 避免刷屏；`takeover_pub_count_cable_reset/unload` 分别跟踪线缆避让场景。

### 7.5 铲斗变化率限制

`bucket_change_rate` 限制每帧 `orientation.w` 上升量（防突跳）。

---

## 八、矿卡几何与车道圆交点（truck_generator_node.cpp）

**输入**：`/truck_pose_topic`（主天线 LLA）、`/truck_info_topic`（含 pitch/roll/heading）、车厢尺寸 `truck_length=9.2 / width=3.9 / height=4.1`。

**核心能力**：

1. **CGCS2000 坐标转换**：内建 `Merdian / xy_to_wgs84` 高斯反算，把卡车 LLA 转 base 局部坐标（[truck_generator_node.cpp#L14~L35](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/truck_generator_node.cpp)）
2. **卡车 4 顶点**：`calculateTruckVertices` 生成 `interfaces/truckVertices` → `/truck_vertices_topic`
3. **多个卸载点**：`local_unLoadPoints` 一次性生成未来多卸料点（每点被卸 `unload_perpoint_count=3` 次）
4. **车道-圆交点**：`computeLaneCircleLocalIntersections` / `computeLaneCircleIntersectionsLLA` 求解中/左/右车道线与以挖机为中心的半径 `circle_radius_y_m` 圆的 3 个交点，供无 RTK 时的兜底挖掘点选择
5. **几何缓存策略**：`truck_geom_cache_valid_` + 位移/航向变化阈值决定是否重算，减少开销

**输出话题**：`/truck_center_unloadpoint`、`/truck_center_unloadpoint_2`、`/truck_center_point`、`/truck_vertices_topic`、`/lane_circle_intersections`

**注意**：这些输出**同时被 dump_trajectory_planner 反向订阅**，构成 traj_fusion 与规划节点间的双向耦合。

---

## 九、大脑（SmolVLA）通道对接

数采节点 `data_to_nas_and_ubuntu2204` 通过 HTTP POST `/state_data` 向 192.168.1.120:8000 请求推理结果，返回的 4 关节动作（**弧度**）通过 `/smolVLA_output_angle` 发布，本节点订阅并：

1. 弧度 → 角度：`radiansToDegrees()`（[act_traj_fusion.cpp#L1457~L1459](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/act_traj_fusion.cpp)）
2. 只在 `traj_status==3 && use_dig_status_process_model_==true` 时作为 `act_boom/arm/bucket_angle` 生效
3. 否则置为 10000（哨兵值，供日志辨识模型未介入）
4. `smolVLA_status_check` 非 0 触发 FAULT_0~5，通知运维

**reward 机制**：`std::atomic<int> reward_` 每主循环清零，供离线评估模型决策质量（当前 `ROS_INFO_THROTTLE` 输出被注释，可按需打开）。

---

## 十、编译与运行

### 10.1 依赖

**ROS 系统包**：
```bash
sudo apt install ros-noetic-grid-map-msgs
```

**自定义 `interfaces` 包**（在 `~/yx_planning_ws/src/action_fused_pyws/src/action_fused_pynode/interfaces/`）：
- `rtk_cgi610.h` / `JointAngles.h` / `SinglePoint.h` / `truckVertices.h` / `local_unLoadPoints.h`

**第三方**：`stb_image.h` / `stb_image_resize2.h`（已随源码附带）

### 10.2 编译（当前缺 CMakeLists.txt）

**先补建 catkin 骨架**（参考 `action_fused_pyws/src/action_fused_pynode/CMakeLists.txt`），核心内容：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(traj_fusion_pkg)
set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs interfaces)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(traj_fusion_node
  src/traj_fusion_main.cpp
  src/traj_fusion_node.cpp
  src/act_traj_fusion.cpp
  src/truck_generator_node.cpp)
target_link_libraries(traj_fusion_node ${catkin_LIBRARIES})
```

**构建**：
```bash
cd ~/yx_planning_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="traj_fusion_pkg"
```

### 10.3 运行

```bash
source ~/yx_planning_ws/devel/setup.bash
# 需先启动 roscore、上游话题（RTK/关节/卡车/决策/5 个规划节点）
rosrun traj_fusion_pkg traj_fusion_node
```

---

## 十一、调试技巧

### 11.1 关键话题监听

```bash
# 决策阶段推进
rostopic echo /Sys_SeqAction

# 融合输出（本节点核心）
rostopic echo /Sys_SCo_Traj

# 心跳+故障位（低 16 位为故障）
rostopic echo /trajfusionSafety_topic

# 大脑通道状态
rostopic echo /smolVLA_status_check
rostopic echo /smolVLA_output_angle

# 回转方向（供 reset/dump 用）
rostopic echo /swing_clockwise_flag
```

### 11.2 手动模拟阶段推进（办公室无大脑/无卡车时）

```bash
# 依次喂 SeqAction，观察 current_traj 变化
rostopic pub /Sys_SeqAction std_msgs/Float64 "data: 1.0" -1  # 初始化
rostopic pub /Sys_SeqAction std_msgs/Float64 "data: 3.0" -1  # 挖掘
rostopic pub /Sys_SeqAction std_msgs/Float64 "data: 4.0" -1  # 卸载
```

配合 `exca200_simulink` 全循环仿真可自动推进 SeqAction，无需手发。

### 11.3 常见现象与定位

| 现象 | 定位方向 |
|------|---------|
| `/Sys_SCo_Traj` 一直是 0 | SeqAction 从未有效→`traj_status=14` 兜底；查 `/Sys_SeqAction` 是否有值 |
| 阶段 3 卡住不动 | `stop_swing_main_done_in_dig_stage_` 是否已置；`dig_OP_level_` 是否触发降级 |
| 阶段 4 卸料铲斗角乱跳 | `unload_bucket_use_planner_w_` 阈值切换点前后；`bucket_change_rate` 是否过小 |
| 心跳里 FAULT_6 常亮 | 复位/卸载回转路径经过线缆区；检查 `/GP2mid/rear_cable_deg` 配置 |
| 大脑通道未生效 | `use_dig_status_process_model` 必须显式置 true 且 `Mpc_Switch=true` |
| 卡车可视化偏 180° | RTK 航向 α → base 系 yaw 应为 `180°−α`（非 `180°+α`），已在 truck_generator 修正 |

---

## 十二、与其他包的耦合关系

| 相邻包 | 方向 | 关键话题 |
|--------|------|---------|
| excavator_system_decision | ← | `/Sys_SeqAction` |
| GP2mid | ←→ | `/Sys_RGp_SwWkAgl`、`/GP2mid/front_lane_deg`、`/lane_circle_intersections` |
| dump_trajectory_planner | ←→ | `/RefDeviceTraj_Unload`、`/Sys_RUn_UnloadEndAngle` → / `/truck_center_unloadpoint(_2)` |
| reset_trajectory_planner | ←→ | `/RefDeviceTraj_Reset` → / `/truck_swive_angle`、`/swing_clockwise_flag` |
| clean_trajectory_planner | ← | `/RefDeviceTraj_CleanUp`、`/clean_digPoint`、`/Sys_RGp_Unload` |
| smooth_trajectory_planner | ← | `/RefDeviceTraj_Smooth` |
| data_to_nas | ← | `/smolVLA_output_angle`、`/smolVLA_status_check` |
| action_fused_pyws | → | `/Sys_SCo_Traj`（唯一驱动源） |
| exca200_simulink | ← | 仿真时通过 `traj_to_urdf_adapter.py` 转 `/RefDeviceTraj_Unload` 到 URDF |

---

## 十三、已知限制与接手后建议

1. **缺 catkin 骨架**：当前工作区无 `CMakeLists.txt` / `package.xml` / `launch/` / `config/`，无法直接 `catkin_make`。建议向同事索要车上使用的完整版本。
2. **参数无 yaml**：40+ 参数全部散在代码/launch 中，无集中配置文件。建议新建 `config/traj_fusion.yaml` 集中管理，方便现场调参。
3. **单文件过大**：`act_traj_fusion.cpp` 2804 行，`hpp` 606 行，成员变量数百。建议按阶段拆分为 `status_init.cpp / status_reset.cpp / status_dig.cpp / ...`。
4. **心跳 FAULT_5 状态超时被注释**：`processACTOutput` L2507 `addFaultCode(faultMask, FAULT_5)` 处于注释状态，超时故障不会真的触发。接手后按需决定要不要恢复。
5. **`clampTrajAngles` 不 clamp swing**：L2479 那行被注释，若回转角本身有奇异风险需要评估。
6. **`AsyncSpinner(2)`**：只有 2 线程，回调多（30+），大流量时可能拥堵；建议压测时改 4 或分优先级。
7. **大脑通道与传统通道的仲裁逻辑**目前只在**阶段 3（挖掘）**通过 `use_dig_status_process_model_` 简单切换，卸载/复位阶段仍走传统。若要扩展到其他阶段需要改 `processACTOutput` 的 switch 分支。

---

## 十四、上手学习顺序（针对 traj_fusion 单包）

按由浅入深：

1. 读 [traj_fusion_main.cpp](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/traj_fusion_main.cpp)（18 行）建立入口印象
2. 读 [traj_fusion_node.cpp](file:///home/zhuay/yx_planning_ws/src/traj_fusion_pkg/src/traj_fusion_node.cpp) 构造函数（42~206 行）——所有参数与话题一目了然
3. 读 `run()` + `processACTOutput()` 状态分派框架（act_traj_fusion.cpp 2488~2760）——理解 10Hz 骨架
4. 挑一个你熟悉的阶段处理函数深挖（推荐 `raiseAndUnloadStausProcess` 或 `resetStatusProcess`）
5. 最后看 `truck_generator_node.cpp` 理解几何生成
6. 对照 `Sys_SeqAction` 阶段号（2/3/4/5/6/7 + x.5 语义）跟 `excavator_system_decision` 决策层状态机双向印证
