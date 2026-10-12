# Excavator System Decision

```bash
# 进入 ROS 工作空间
cd /home/wx/yx870_system_decision_ws
```

```bash
# 编译无人挖机系统决策包
catkin_make --pkg excavator_system_decision
```

```bash
# 加载当前工作空间环境
source devel/setup.bash
```

```bash
# 启动系统决策节点
roslaunch excavator_system_decision excavator_system_decision_start.launch
```

```bash
# 启动测试 GUI
roslaunch excavator_system_decision decision_test_gui.launch
```

```bash
# 启动矿卡交互流程
rostopic pub -r 10 /Mpc_Switch std_msgs/Bool "data: true"
```

`INITIAL_WAIT_TRUCK_ENTRY` 会等待 `/heading2swing_topic` 和
`/truck_center_unloadpoint` 均收到数据后选择等待 FSM。后者类型为
`geometry_msgs/Quaternion`，仅使用 `x`、`y`，目标回转角按
`atan2(-y, -x)` 转换并归一化至 0～360 度。与当前回转角的环绕误差不大于
`operation.truck_center_angle_error_threshold`（默认 30 度）时执行
`ResetDigWaitFsm`，否则执行 `DigWaitFsm`；未配置超时。

```bash
# x=-1, y=0 时目标回转角为 0 度
rostopic pub -1 /truck_center_unloadpoint geometry_msgs/Quaternion \
  "{x: -1.0, y: 0.0, z: 0.0, w: 0.0}"
```

```bash
# 请求在本车离场、挖一铲等待完成后执行搭台
rostopic pub -1 /Mpc_Switch2 std_msgs/Bool "data: true"
```

保持 `data: true` 表示持续请求搭台；发送 `data: false` 后立即取消请求。

```bash
# 运行矿卡交互流程自动测试脚本
./src/excavator_system_decision/scripts/test_truck_interaction.sh
```

`/FlagCleanFinish` 使用 `std_msgs/Float32`，仅用于搭台第 7 阶段：输入 `1.0` 继续下一轮搭台；输入 `2.0` 进入最终挖掘和抬臂，完成搭台后回到矿卡等待。

```bash
rostopic pub -1 /FlagCleanFinish std_msgs/Float32 "data: 2.0"
```

```bash
# 后退行走完成反馈（仅在 Sys_SeqAction=8.0 时发送）
rostopic pub -1 /FlagWalkFinish std_msgs/Bool "data: true"
```
