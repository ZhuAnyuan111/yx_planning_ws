# kinematics

无人挖机 **运动学求解库**（ROS1 Noetic 版）。提供 4 自由度关节链（`swing → boom → arm → bucket`）的正向 / 逆向运动学接口，作为独立 catkin package，供上层规划节点（`dump_trajectory_planner`、`reset_trajectory_planner`、`truck_dump_planner` 等）以库依赖方式复用。

> 本包由早期 ROS2 (`rclcpp`) 版本改造而来，改造要点：
> - `rclcpp::Node::SharedPtr` → `ros::NodeHandle&`
> - 参数命名从 `kinematics.xxx` 改为 ROS1 风格 `kinematics/xxx`
> - 追加**纯 C++ 构造函数**，支持脱离 ROS 单元测试与跨包结构体注入

## 功能概述

| 能力 | 接口 | 说明 |
|:---|:---|:---|
| 正向运动学（回转中心系） | `swing_center_forward(JointState)` | 输入关节角（rad），输出齿尖 3D 位姿 |
| 正向运动学（动臂销轴系） | `boom_pivot_forward(JointState)` | 输入关节角，输出 2D 位姿（臂平面内） |
| 逆向运动学（给定铲斗**转角**） | `swing_center_inverse_by_bucket_angle(...)` | 3D 目标 + 铲斗转角 deg → 关节角 |
| 逆向运动学（给定铲斗**姿态角**） | `swing_center_inverse_by_bucket_attitude(...)` | 3D 目标 + 齿尖姿态 deg → 关节角（常用于卸载） |
| 逆向运动学（2D 臂平面） | `boom_pivot_inverse_by_bucket_attitude(...)` | 忽略回转，仅在臂平面内求解 |

所有 IK 求解均包含**关节限位检查**，失败时通过 `IkResult::message` 输出诊断原因。

## 目录结构

```
kinematics/
├── CMakeLists.txt                  # catkin 构建：输出 libkinematics.so
├── package.xml                     # 依赖 roscpp
├── README.md
├── include/kinematics/
│   └── kinematics.hpp              # 公开头文件
└── src/
    └── kinematics.cpp              # 实现
```

## 数据结构

| 类型 | 字段 | 说明 |
|:---|:---|:---|
| `JointState` | `swing/boom/arm/bucket` | 关节角，单位 **rad** |
| `Pose2D` | `x, z, alpha` | 臂平面 2D 位姿 |
| `Pose3D` | `x, y, z, alpha` | 3D 位姿（回转中心系） |
| `LinkGeometry` | `boom_length` / `arm_length` / `bucket_tooth_length` / `boom_pivot_x/y/z` | 连杆长度与销轴偏置（m） |
| `JointLimitsDeg` | `boom_lower/upper` / `arm_lower/upper` / `bucket_lower/upper` | 关节限位，单位 **deg** |
| `IkResult` | `success, q, message` | 求解结果与诊断信息 |
| `SwingBaseIkRequest` | `target_pose` + `bucket_angle_deg` / `bucket_attitude_deg` (optional) | 3D IK 输入 |
| `BoomPivotIkRequest` | `target_pose` + `bucket_attitude_deg` (optional) | 2D IK 输入 |

## 两种构造方式

### 方式 1：从 ROS 参数服务器加载（推荐）

```cpp
#include <ros/ros.h>
#include <kinematics/kinematics.hpp>

ros::NodeHandle pnh("~");
// 从 <ns>/kinematics/... 命名空间读取参数
kinematics::KinematicsSolver solver(pnh, "kinematics");
```

### 方式 2：纯 C++ 结构体注入（脱离 ROS，便于测试）

```cpp
kinematics::LinkGeometry g;
g.boom_length = 5.7;
g.arm_length = 2.9;
g.bucket_tooth_length = 1.6;
// ...

kinematics::JointLimitsDeg lim;
lim.boom_lower = -60.0;  lim.boom_upper = 90.0;
lim.arm_lower  = -170.0; lim.arm_upper  = 0.0;
lim.bucket_lower = -170.0; lim.bucket_upper = 30.0;

kinematics::KinematicsSolver solver(g, lim);
```

## ROS 参数（方式 1 使用）

命名空间前缀由构造函数传入（默认 `kinematics`）。yaml 示例：

```yaml
kinematics:
  # 连杆长度 (m)
  boom_length: 5.7
  arm_length: 2.9
  bucket_tooth_length: 1.6

  # 动臂销轴相对回转中心的偏置 (m)
  boom_pivot_x: 0.5
  boom_pivot_y: 0.0
  boom_pivot_z: 2.1

  # 关节限位 (deg)
  boom_joint_lower: -60.0
  boom_joint_upper:  90.0
  arm_joint_lower:  -170.0
  arm_joint_upper:     0.0
  bucket_joint_lower: -170.0
  bucket_joint_upper:   30.0
```

## 典型使用（卸载点可达性判断）

```cpp
#include <kinematics/kinematics.hpp>

kinematics::KinematicsSolver solver(pnh, "kinematics");

kinematics::SwingBaseIkRequest req;
req.target_pose = {unload_x, unload_y, unload_z, 0.0};
req.bucket_attitude_deg = -90.0;  // 卸载时齿尖朝下

const auto ik = solver.swing_center_inverse_by_bucket_attitude(req);
if (!ik.success) {
  ROS_WARN("unload point unreachable: %s", ik.message.c_str());
} else {
  ROS_INFO("q: swing=%.3f boom=%.3f arm=%.3f bucket=%.3f (rad)",
           ik.q.swing, ik.q.boom, ik.q.arm, ik.q.bucket);
}
```

## 编译

```bash
cd ~/yx_planning_ws
catkin_make --pkg kinematics
source devel/setup.bash
```

产物：`devel/lib/libkinematics.so` + `devel/include/kinematics/kinematics.hpp`

## 下游集成（三步）

在下游 package 中：

**1) `package.xml`**

```xml
<depend>kinematics</depend>
```

**2) `CMakeLists.txt`**

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kinematics    # ← 添加
  ...
)

catkin_package(
  CATKIN_DEPENDS roscpp kinematics ...
)

target_link_libraries(your_target ${catkin_LIBRARIES})
```

**3) 源码**

```cpp
#include <kinematics/kinematics.hpp>
```

## 坐标 / 角度约定

- **回转中心系（swing center frame）**：`x` 沿正前方，`z` 沿铅垂向上，`y` 补齐右手系。
- **动臂销轴系（boom pivot frame）**：以动臂销轴为原点的 2D 臂平面（`x-z`）。
- **正向运动学**：给定 `JointState`，累加各关节角度得到齿尖 3D 位置及姿态。
- **逆向运动学**：使用余弦定理求解三角形（`boom-arm-target`），失败原因分为三类：
  - `degenerate geometry`：目标点过于接近关节销轴，几何退化；
  - `triangle cosine out of range`：目标点超出连杆可达包络；
  - `joint limits violated`：解在几何上存在但超出关节限位。
- **单位**：关节状态存 rad；接口输入 / 输出的角度参数（如 `bucket_attitude_deg`、`JointLimitsDeg`）为 deg。

## TODO

- [ ] 单元测试（`gtest`）：正向 / 逆向 IK 数值精度与限位边界；
- [ ] 支持 elbow-up / elbow-down 双分支解切换（当前 `elbow_up` 字段预留未启用）；
- [ ] 参数校验：构造时检查 `boom_length / arm_length / bucket_tooth_length > 0`，否则打印警告；
- [ ] 提供 `plugin` / `nodelet` 形式的服务化封装（可选）。
