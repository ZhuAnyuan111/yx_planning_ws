# 数据采集与 NAS 上传节点 (data_to_nas_and_ubuntu2204)

> **版本**：20261029 | **维护人**：wuql17 | **文档生成日期**：2026-10-29  
> **源码规模**：主节点 C++ 文件 4835 行 + NAS 端 Flask 服务 69KB + 4 个辅助脚本

---

## 一、功能概述

本节点是**无人挖掘机数据采集系统的核心组件**，运行在车载 NVIDIA Jetson (Ubuntu 22.04) 上，负责将挖掘机作业过程中的全量传感器数据实时采集、本地持久化、远程上传至 NAS，并与"大脑"（SmolVLA 视觉语言动作模型）保持通信。

### 1.1 六大核心功能

| # | 功能 | 说明 |
|---|------|------|
| 1 | **实时数据采集** | 订阅 20+ 路传感器话题（关节角、IMU、RTK、栅格地图、相机图像等），按会话（session）持久化到 G500 移动硬盘 |
| 2 | **大脑通信** | 将挖掘机状态数据、栅格地图、灰度距离图等通过 HTTP POST 实时推送给大脑控制器（SmolVLA），并接收大脑预测动作 |
| 3 | **NAS 分块上传** | 会话结束后，对数据进行完整性质检 → ZIP 打包 → 分块断点续传上传到群晖 DS923+ NAS |
| 4 | **数据质检** | 上传前对会话数据进行 8 项完整性校验（目录结构、JSON 有效性、文件零字节、时间戳对齐等），损坏数据标记为 `_Damaged.zip` |
| 5 | **双摄像头监控** | 通过 GStreamer 拉流两路相机（前视+后视），JPEG 编码保存，含看门狗超时检测、流错误确认、自动重连、安全告警发布 |
| 6 | **自动启停** | 根据挖掘机运动状态（关节角/回转角变化）自动开启/关闭数据采集，含去抖动、冷却时间、最大采集时长限制 |

### 1.2 设计特点

- **多线程架构**：使用 `ros::AsyncSpinner`（默认 4 线程）+ 独立相机工作线程 + 独立质检/上传线程
- **双写策略**：RTK 数据同时写入本地 JSON 文件 + POST 到大脑，保证数据不丢失
- **会话级管理**：每次采集创建 `YYYYMMDDhhmmss` 时间戳会话目录，支持断点续传
- **存储安全保护**：G500 剩余空间 < 20GB 或已用 > 100GB 时自动拒绝新采集
- **损坏恢复**：节点启动时自动恢复 `.json.tmp` 孤立文件，重命名为 `.json`
- **多机型支持**：NAS 端根据客户端 IP 自动分配到对应机型存储目录

---

## 二、系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                     UnifiedDataNode (主节点)                     │
├─────────────────────────────────────────────────────────────────┤
│  订阅层 (20+ 话题)                                              │
│  ├─ /joints_angles, /swivel_angle, /angular_velocity           │
│  ├─ /imu_topic_arm/boom/bucket/vehicle                         │
│  ├─ /rtk_can_msg, /excavation_gridmap, /gray_range_image       │
│  ├─ /data_capture_switch_manual (采集开关信号)                  │
│  └─ /fault_code, /Sys_SeqAction, /Sys_SCo_Traj 等              │
├─────────────────────────────────────────────────────────────────┤
│  发布层                                                         │
│  ├─ /smolVLA_output_angle (大脑预测动作)                        │
│  ├─ /smolVLA_status_check (大脑通信状态 0~6)                    │
│  └─ /CameraSafety_topic (摄像头安全状态)                        │
├─────────────────────────────────────────────────────────────────┤
│  核心模块                                                       │
│  ├─ StateJsonWriter: 状态数据 JSON 序列化                       │
│  ├─ GstCameraCapture: GStreamer 相机拉流 + JPEG 编码            │
│  ├─ CameraWorker: 相机线程（含看门狗、重连、安全告警）          │
│  ├─ SessionZipUploader: 数据质检 + 打包 + NAS 分块上传          │
│  ├─ NasHttpSession: NAS HTTP 会话管理                           │
│  └─ LatestPostWorker: 大脑 HTTP POST 异步发送                   │
└─────────────────────────────────────────────────────────────────┘
         │                    │                    │
         ▼                    ▼                    ▼
   G500 移动硬盘         大脑控制器 (192.168.1.120)   NAS DS923+ (192.168.1.245)
   /media/nvidia/G500    http://192.168.1.120:8082   http://192.168.1.245:8082
```

---

## 三、目录结构

```
最新的数采20261029/
├── data_to_nas_and_ubuntu2204/          # ROS 功能包
│   ├── CMakeLists.txt                   # 构建配置（依赖 CURL/OpenSSL/GStreamer/jsoncpp/yaml-cpp/jpeg/png）
│   ├── package.xml                      # 包描述
│   ├── launch/
│   │   ├── data_to_NAS_and_ubuntu2204.launch      # 主启动文件
│   │   └── data_to_NAS_and_ubuntu2204_20260712.launch  # 历史版本
│   └── src/
│       ├── data_to_NAS_and_ubuntu2204_node.cpp    # 主节点源码 (4835 行)
│       └── data_to_NAS_and_ubuntu2204_node_*.cpp  # 历史版本 (20260703~20260902)
├── DS923_app.py                         # NAS 端 Flask 服务 (69KB)
├── data_capture_signal_pub.py           # 采集开关信号发布器 (Python)
├── data_upload_to_check.py              # 手动上传脚本 (用于质检)
├── nas_ds923_watchdog_1014.sh           # NAS 看门狗脚本 (SSH 远程重启 DS923_app.py)
└── YX_auto_start_smolvla_20260813_省略检测_ok.sh  # 整机自动启动脚本
```

---

## 四、核心类与职责

| 类名 | 职责 |
|------|------|
| **UnifiedDataNode** | 主节点类，负责话题订阅/发布、数据采集开关、状态 JSON 写入、大脑通信、相机管理。包含 20+ 个回调函数、5 个定时器、3 个发布器、20+ 个订阅器 |
| **StateJsonWriter** | 将各话题数据序列化为 JSON 行，按会话写入 `bag/state.json.tmp`，支持 beginSession/append/snapshot/finishSession 四个生命周期 |
| **GstCameraCapture** | GStreamer 拉流相机视频，使用 `v4l2src ! videoconvert ! videoscale ! appsink` 管道，缩放到 640×360，JPEG 编码后保存 |
| **CameraWorker** | 相机工作线程，含看门狗超时检测（默认 0.6s）、流错误确认（默认 2.5s）、自动重连（冷却 2.0s）、启动宽限期（5.0s）、安全告警发布 |
| **SessionZipUploader** | 数据质检 + ZIP 打包 + NAS 分块上传 + 断点续传 + 损坏归档标记。含独立质检线程 + 上传线程，支持并发分块（默认 4 路） |
| **NasHttpSession** | NAS HTTP 会话封装，管理 upload_init/chunk/complete 流程，支持会话重建（连续 3 次失败后重建） |
| **LatestPostWorker** | 异步发送 HTTP POST 到大脑（状态/栅格/灰度图/相机图），含超时重试、回调通知 |

### 4.1 UnifiedDataNode 内部结构详解

```
UnifiedDataNode
├── 数据缓存层
│   ├── DataCache data_               // 所有话题的最新数据快照（互斥保护）
│   ├── std::mutex data_mutex_         // 保护 data_ 的读写锁
│   └── markTopicReceived()            // 话题到达标记（用于诊断）
│
├── 会话管理层
│   ├── recording_                     // 当前是否正在采集
│   ├── state_session_id_              // 会话 ID（递增）
│   ├── dataset_name_                  // 当前数据集名 (excavator_data_YYYYMMDD)
│   ├── session_root_                  // 当前会话根目录
│   ├── bag_dir_ / gray_dir_ / range_dir_  // 子目录路径
│   ├── json_tmp_path_ / json_final_path_  // JSON 临时/最终文件路径
│   └── buildSessionPaths()            // 根据时间戳构建会话目录结构
│
├── 采集开关控制层
│   ├── raw_save_signal_               // 原始信号值（去抖动前）
│   ├── last_save_signal_              // 已应用信号值
│   ├── capture_switch_debounce_s_     // 去抖动时间（默认 1.0s）
│   ├── captureSwitchDebounceTimerCb() // 去抖动定时器回调
│   ├── applySaveSignalTransition()    // 信号跳变处理（0→1=开始, 1→0=结束）
│   └── captureCapacityAllowsStart()   // 存储容量检查
│
├── 大脑通信层
│   ├── brain_state_url_               // POST /state_data
│   ├── brain_gridmap_url_             // POST /gridmap_data
│   ├── brain_range_url_               // POST /range_data
│   ├── brain_cam0_url_ / brain_cam1_url_  // POST /camera0_data, /camera1_data
│   ├── brain_post_worker_             // 异步 POST 工作器
│   └── handleBrainStatus()            // 大脑响应处理 + 状态发布
│
├── 相机监控层
│   ├── camera_workers_[2]             // 两路相机工作线程
│   ├── cam0/cam1_http_status_         // HTTP 状态码
│   ├── cam0/cam1_stream_status_       // 流状态 (0=正常, 非 0=异常)
│   ├── heartbeatTimerCb()             // 心跳定时器 (10Hz)，发布 CameraSafety
│   └── packSafety()                   // 打包安全状态为 uint32
│
├── 诊断层
│   ├── topic_monitors_                // 话题到达监控表
│   ├── diagnosticTimerCb()            // 诊断定时器 (1Hz)，检查话题缺失
│   ├── stateJsonStatusTimerCb()       // JSON 状态定时器 (30s)，报告写入行数
│   └── recoverOrphanStateJsonTmpFiles()  // 启动时恢复孤立 .tmp 文件
│
└── 定时器汇总
    ├── status_timer_      (2Hz)       // 状态计算与发布
    ├── heartbeat_timer_   (10Hz)      // 相机心跳
    ├── debounce_timer_    (10Hz)      // 采集开关去抖动
    ├── diagnostic_timer_  (1Hz)       // 话题诊断
    └── state_status_timer_(30s)       // JSON 状态报告
```

---

## 五、数据保存结构详解

### 5.1 目录层级

```
/media/nvidia/G500/DataCapture/
└── excavator_data_20261029/             # 日期数据集
    └── 20261029143025/                  # 会话时间戳 (YYYYMMDDhhmmss)
        ├── bag/                         # 状态 JSON 文件
        │   ├── state.json               # 最终合并的完整 JSON
        │   └── state.json.tmp           # 临时写入文件
        ├── camera_data/                 # 相机图像
        │   ├── camera0/                 # 前视相机
        │   │   └── 20261029143025_001.jpg
        │   └── camera1/                 # 后视相机
        │       └── 20261029143025_001.jpg
        ├── grayPicture/                 # 灰度图（来自 /gray_range_image）
        │   └── 20261029143025_001.png
        └── rangePicture/                # 距离图（来自 /gray_range_image）
            └── range_20261029143025_001.png
```

### 5.2 文件命名规则

| 目录 | 文件命名规则 | 示例 |
|------|------------|------|
| `bag/` | `state.json` / `state.json.tmp` | `state.json.tmp` |
| `camera_data/camera0/` | `YYYYMMDDhhmmss_NNN.jpg` | `20261029143025_001.jpg` |
| `camera_data/camera1/` | `YYYYMMDDhhmmss_NNN.jpg` | `20261029143025_001.jpg` |
| `grayPicture/` | `YYYYMMDDhhmmss.png` | `220261029143025.png` |
| `rangePicture/` | `range_YYYYMMDDhhmmss.png` | `range_20261029143025.png` |

### 5.3 数据同步机制

所有数据源的时间戳必须对齐：

```
RTK 时间戳 (rtk_stamp_ms) ← 基准
    ↓
相机图像时间戳 (文件名中的 YYYYMMDDhhmmss)
    ↓
灰度图时间戳 (文件名中的 YYYYMMDDhhmmss)
    ↓
距离图时间戳 (文件名中的 YYYYMMDDhhmmss)
```

质检时检查各数据源时间戳与 RTK 时间戳的对齐情况，容差为 500ms。

---

## 六、关键参数配置

### 6.1 启动参数 (launch 文件)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `scan_root` | `/media/nvidia/G500/DataCapture` | 数据保存根目录 |
| `brain_base_url` | `http://192.168.1.120:8082` | 大脑控制器 HTTP 地址 |
| `nas_webhook_url` | `http://192.168.1.245:8082` | NAS Flask 服务地址 |
| `camera0_device` | `0` | 前视相机设备号 |
| `camera1_device` | `1` | 后视相机设备号 |
| `resize` | `[640, 360]` | 图像缩放尺寸 |
| `jpeg_quality` | `90` | JPEG 压缩质量 |
| `frame_interval_s` | `0.066` | 相机帧间隔 (~15Hz) |
| `upload_interval_s` | `90.0` | NAS 上传检查间隔 |
| `chunk_size_bytes` | `8388608` (8MB) | NAS 上传分块大小 |
| `min_free_bytes` | `21474836480` (20GB) | G500 最小剩余空间 |
| `max_scan_root_bytes` | `107374182400` (100GB) | G500 最大使用空间 |

### 6.2 运行时参数 (yaml 配置)

| 参数 | 说明 |
|------|------|
| `yaml_config_path` | 外部 yaml 配置文件路径（如 `param_sy870.yaml`） |
| `default_machine_model` | 机型标识（如 `SY750-1`） |
| `default_operation_mode` | 操作模式（如 `人工`、`自动`） |

---

## 七、话题接口

### 7.1 订阅话题 (输入)

| 话题名 | 消息类型 | 用途 |
|--------|----------|------|
| `/joints_angles` | `interfaces/JointAngles` | 关节角（boom/arm/bucket/swing） |
| `/swivel_angle` | `std_msgs/Float32` | 回转角 |
| `/angular_velocity` | `std_msgs/Float32` | 角速度 |
| `/imu_topic_arm/boom/bucket/vehicle` | `interfaces/imu_syd` | 各连杆 IMU 数据 |
| `/rtk_can_msg` | `interfaces/rtk_cgi610` | RTK 定位数据 |
| `/excavation_gridmap` | `grid_map_msgs/GridMap` | 栅格地图 |
| `/gray_range_image` | `sensor_msgs/Image` | 灰度距离图 |
| `/data_capture_switch_manual` | `std_msgs/Int8` | 采集开关信号 (0=停止, 1=开始) |
| `/fault_code` | `std_msgs/String` | 故障码 |
| `/Sys_SeqAction` | `std_msgs/Float64` | 系统序列动作 |
| `/Sys_SCo_Traj` | `geometry_msgs/Pose` | 系统轨迹坐标 |
| `/bucket_local_position` | `interfaces/SinglePoint` | 铲斗本地坐标 |
| `/bucket_rotation_local_position` | `interfaces/SinglePoint` | 铲斗旋转坐标 |
| `/bucket_terrain_delta_position` | `geometry_msgs/PointStamped` | 铲斗地形 delta |
| `/evcavator_joystick_topic` | `geometry_msgs/Quaternion` | 手柄指令 |
| `/fused_controller_joystick_topic` | `geometry_msgs/Quaternion` | 融合控制器手柄 |
| `/joints_velocity` | `geometry_msgs/Quaternion` | 关节速度 |

### 7.2 发布话题 (输出)

| 话题名 | 消息类型 | 用途 |
|--------|----------|------|
| `/smolVLA_output_angle` | `geometry_msgs/Pose` | 大脑预测的动作指令 |
| `/smolVLA_status_check` | `std_msgs/Int8` | 大脑通信状态 (0~6，表示不同故障类型) |
| `/CameraSafety_topic` | `std_msgs/UInt32` | 摄像头安全状态 |

---

## 八、执行流程

### 8.1 数据采集流程

```
1. 节点启动 → 订阅所有话题 → 等待 /data_capture_switch_manual 信号
2. 收到信号=1 → 创建会话目录 (YYYYMMDDhhmmss) → 开始写入 JSON/图像
3. 每收到一帧数据 → StateJsonWriter 追加到 state.json.tmp
4. 每 20 帧或 0.5 秒 → 刷新到 state.json (可配置)
5. 收到信号=0 → 关闭会话 → 触发质检上传
```

### 8.2 大脑通信流程

```
1. 心跳定时器 (每 2 秒) → POST /state_data 到大脑
2. 栅格地图回调 → POST /gridmap_data
3. 灰度图回调 → POST /range_picture
4. 相机帧 → POST /camera0_data + /camera1_data
5. 接收大脑响应 → 发布 /smolVLA_output_angle + /smolVLA_status_check
```

### 8.3 NAS 上传流程

```
1. 会话关闭 → SessionZipUploader 启动质检
2. 校验文件完整性 → 打包为 ZIP (压缩级别 1)
3. 计算 SHA-256 → 与 NAS 协商 upload_id
4. 分块上传 (每块 8MB) → POST /upload_chunk
5. 全部上传完成 → POST /upload_complete
6. NAS 返回 ok=true → 删除本地 ZIP，保留源数据
7. 如果校验失败 → 重命名为 *_Damaged.zip，等待人工处理
```

---

## 九、核心功能详解

### 9.1 数据采集机制详解

#### 9.1.1 采集开关信号处理

采集开关信号 `/data_capture_switch_manual` 经过**三级处理**：

```
原始信号 (raw_save_signal_)
    ↓ 去抖动 (capture_switch_debounce_s_ = 1.0s)
去抖后信号 (last_save_signal_)
    ↓ 跳变检测
应用跳变 (applySaveSignalTransition)
    ↓ 容量检查
实际采集状态 (recording_)
```

**去抖动逻辑**：
- 原始信号变化后，等待 1.0 秒稳定期
- 稳定期内如果信号再次变化，重置计时器
- 稳定期结束后才触发实际跳变

**跳变处理**：
- `0 → 1`（开始采集）：
  1. 检查 G500 剩余空间 ≥ 20GB
  2. 检查 G500 已用空间 ≤ 100GB
  3. 创建会话目录 `YYYYMMDDhhmmss`
  4. 初始化 StateJsonWriter
  5. 设置 `recording_ = true`
  6. 通知相机线程开始录制
- `1 → 0`（停止采集）：
  1. 设置 `recording_ = false`
  2. 通知 StateJsonWriter 完成会话
  3. 通知相机线程停止录制
  4. 触发 SessionZipUploader 质检上传

#### 9.1.2 状态数据写入机制

StateJsonWriter 采用**追加写入 + 定期刷新**策略：

```
RTK 回调触发
    ↓
构建 JSON 对象 (包含所有话题最新数据)
    ↓
追加到 state.json.tmp (每行一个 JSON)
    ↓
每 20 帧或 0.5 秒 → 刷新到磁盘 (fsync)
    ↓
会话结束时 → 重命名为 state.json
```

**JSON 数据结构**（每行一个 JSON 对象）：
```json
{
  "frame123, 1698567890.123": {
    "rtk_stamp_ms": 1698567890123,
    "rtk": {
      "lat": 30.123456,
      "lon": 120.123456,
      "alt": 15.5,
      "satellites": 24,
      "status": 4
    },
    "joints_angles": {
      "swing": 45.5,
      "boom": -15.2,
      "arm": -90.3,
      "bucket": 30.1
    },
    "swivel_angle": 45.5,
    "angular_velocity": 0.12,
    "bucket_local_position": {"x": 1.5, "y": 2.3, "z": -0.8},
    "bucket_rotation_local_position": {"x": 1.5, "y": 2.3, "z": -0.8},
    "bucket_terrain_delta_position": {"x": 0.1, "y": 0.2, "z": -0.05},
    "joints_velocity": {"x": 0.1, "y": 0.2, "z": 0.0, "w": 0.0},
    "imu_arm": {...},
    "imu_boom": {...},
    "imu_bucket": {...},
    "imu_vehicle": {...},
    "joystick": {"x": 0.5, "y": -0.3, "z": 0.0, "w": 1.0},
    "fused_controller_joystick": {...},
    "sys_seqAction": 3.0,
    "sys_sco_traj": {...},
    "fault_code": "",
    "delta_vla_height": 0.15
  }
}
```

#### 9.1.3 相机数据采集机制

两路相机（前视 camera0、后视 camera1）独立工作：

```
GStreamer 管道拉流
    ↓
videoconvert + videoscale (缩放到 640×360)
    ↓
appsink 接收帧
    ↓
JPEG 编码 (质量 90)
    ↓
保存到 camera_data/camera0/ 或 camera1/
    ↓
同时 POST 到大脑 (/camera0_data 或 /camera1_data)
```

**相机看门狗机制**：
- 每帧到达时重置看门狗计时器
- 如果超过 0.6 秒无新帧 → 标记为流异常
- 流异常持续 2.5 秒 → 确认故障，发布安全告警
- 自动重连冷却时间 2.0 秒
- 启动宽限期 5.0 秒（避免启动时误报）

**CameraSafety 话题编码**：
```cpp
uint32 packSafety(uint8 error_code, uint8 heartbeat_counter) {
  return (error_code << 24) | (heartbeat_counter << 16) | (heartbeat_counter << 8) | error_code;
}
```
- 位 31-24: error_code（bit0=camera0 异常, bit1=camera1 异常）
- 位 23-16: heartbeat_counter（0-255 循环计数）
- 位 15-8: heartbeat_counter（冗余）
- 位 7-0: error_code（冗余）

### 9.2 大脑通信协议详解

#### 9.2.1 状态数据推送

**触发时机**：每次 RTK 回调（约 10Hz）

**HTTP POST 请求**：
```
POST http://192.168.1.120:8082/state_data
Content-Type: application/json
Timeout: 0.8s

{
  "frame123, 1698567890.123": {
    "joints_angles": {...},
    "swivel_angle": 45.5,
    "bucket_terrain_delta_position": {...},
    "joints_velocity": {...},
    "rtk_stamp_ms": 1698567890123,
    "rtk": {...}
  }
}
```

**响应处理**：
- 如果响应包含 `pred_action` 字段 → 解析为动作指令 → 发布到 `/smolVLA_output_angle`
- 如果响应包含 `status` 字段 → 解析为状态码 → 发布到 `/smolVLA_status_check`

#### 9.2.2 栅格地图推送

**触发时机**：每次 `/excavation_gridmap` 回调

**图像处理流程**：
```
grid_map_msgs/GridMap
    ↓
提取 "grayscale" 层
    ↓
转换为 GrayImage (uint8)
    ↓
缩放到 360×360
    ↓
居中粘贴到 640×360 画布（两侧留黑边）
    ↓
JPEG 编码 (质量 90)
    ↓
Base64 编码
    ↓
POST /gridmap_data
```

**HTTP POST 请求**：
```json
{
  "ts": 1698567890123,
  "w": 640,
  "h": 360,
  "format": "jpeg_base64",
  "data": "/9j/4AAQSkZJRgABAQAAAQABAAD..."
}
```

#### 9.2.3 灰度距离图推送

**触发时机**：每次 `/gray_range_image` 回调

**图像处理流程**：
```
sensor_msgs/Image (mono8)
    ↓
缩放到 512×N (保持宽高比)
    ↓
居中粘贴到 640×360 画布
    ↓
JPEG 编码 + Base64
    ↓
POST /range_data
```

#### 9.2.4 大脑通信状态码

`/smolVLA_status_check` 发布值含义：

| 值 | 含义 |
|----|------|
| 0 | 正常 |
| 1 | HTTP 请求超时 |
| 2 | HTTP 响应状态码异常 |
| 3 | JSON 解析失败 |
| 4 | 连续多次失败 |
| 5 | 大脑返回格式错误 |
| 6 | 未知错误 |

### 9.3 数据质检机制详解

#### 9.3.1 质检触发时机

- 会话关闭时（采集信号 1→0）
- 节点启动时（扫描未处理的会话）
- 定时扫描（每 90 秒检查一次）

#### 9.3.2 八项质检规则

SessionZipUploader::validateSession() 执行以下检查：

| # | 检查项 | 说明 | 失败后果 |
|---|--------|------|----------|
| 1 | 目录结构完整性 | 必须包含 bag/, camera_data/, grayPicture/, rangePicture/ 四个子目录 | 标记为不合格 |
| 2 | 符号链接检查 | 会话目录内不允许符号链接 | 标记为不合格 |
| 3 | 零字节文件检查 | 所有文件大小必须 > 0 | 记录错误详情 |
| 4 | JSON 有效性检查 | bag/ 目录下的 .json 文件必须每行都是合法 JSON | 记录错误详情 |
| 5 | 时间戳存在性检查 | 每个 JSON 记录必须包含 rtk_stamp_ms 或可从键名解析时间戳 | 记录错误详情 |
| 6 | 相机文件命名检查 | camera_data/camera0/ 和 camera1/ 下的文件必须符合 `YYYYMMDDhhmmss_*.jpg` 格式 | 记录错误详情 |
| 7 | 灰度/距离图命名检查 | grayPicture/ 和 rangePicture/ 下的文件必须符合 `YYYYMMDDhhmmss.png` 和 `range_YYYYMMDDhhmmss.png` 格式 | 记录错误详情 |
| 8 | 时间戳对齐检查 | 各数据源的时间戳必须与 JSON 记录的时间戳对齐（容差 500ms） | 记录错误详情 |

#### 9.3.3 质检结果处理

```
质检完成
    ↓
qualified = (total_error_count == 0)
    ↓
├─ qualified = true
│   ↓
│  创建 ZIP 包 (压缩级别 1)
│   ↓
│  计算 SHA-256
│   ↓
│  上传到 NAS
│   ↓
│  上传成功 → 删除本地 ZIP，保留源数据
│
└─ qualified = false
    ↓
   重命名为 *_Damaged.zip
    ↓
   等待人工处理
```

#### 9.3.4 时间戳对齐检查详解

```
对于每个 JSON 记录的 rtk_stamp_ms:
    ↓
在 camera0_stamps 中查找最近的时间戳
    ↓
计算 delta = abs(nearest_camera0_stamp - rtk_stamp_ms)
    ↓
如果 delta > 500ms → 记录错误 "camera_0 alignment exceeds tolerance"
    ↓
对 camera1、grayPicture、rangePicture 重复上述检查
```

### 9.4 NAS 分块上传机制详解

#### 9.4.1 上传流程状态机

```
IDLE
    ↓ 会话关闭
QUALITY_CHECK
    ↓ 质检通过
ZIP_CREATION
    ↓ ZIP 创建成功
UPLOAD_INIT
    ↓ 获取 upload_id
UPLOAD_CHUNKS (并发 4 路)
    ↓ 所有分块上传完成
UPLOAD_COMPLETE
    ↓ NAS 确认完成
CLEANUP → IDLE
```

#### 9.4.2 分块上传协议

**1. 初始化上传**：
```
POST /upload_init
{
  "upload_id": "state_20261029_143025_abc123",
  "archive_kind": "state",
  "dataset_root": "excavator_data_20261029",
  "archive_name": "state_20261029_143025_abc123.tar",
  "total_size": 12345678,
  "chunk_size": 8388608,
  "total_chunks": 2,
  "source_files": ["bag/state.json", ...]
}

响应:
{
  "ok": true,
  "state": "ready_to_upload",
  "uploaded_chunks": []
}
```

**2. 上传分块**：
```
POST /upload_chunk
Content-Type: multipart/form-data

upload_id: state_20261029_143025_abc123
chunk_index: 0
file: state_20261029_143025_abc123.tar.000000.part (二进制)

响应:
{
  "ok": true,
  "bytes_received": 8388608,
  "uploaded_chunk_count": 1
}
```

**3. 完成上传**：
```
POST /upload_complete
{
  "upload_id": "state_20261029_143025_abc123"
}

响应:
{
  "ok": true,
  "state": "completed",
  "files_saved": 15
}
```

#### 9.4.3 断点续传机制

```
上传中断
    ↓
下次启动时扫描 .session_upload_status/ 目录
    ↓
发现未完成的 upload_id
    ↓
POST /upload_init (重新初始化)
    ↓
NAS 返回已接收的 uploaded_chunks 列表
    ↓
跳过已上传的分块，继续上传剩余分块
    ↓
全部完成后 POST /upload_complete
```

#### 9.4.4 损坏归档处理

```
质检失败
    ↓
创建 ZIP 包 (包含所有源文件)
    ↓
重命名为 {session_id}_Damaged.zip
    ↓
移动到 scan_root/.session_zip_tmp/
    ↓
等待人工检查和处理
```

### 9.5 存储空间管理机制

#### 9.5.1 容量保护策略

```
采集信号 0→1
    ↓
captureCapacityAllowsStart()
    ↓
检查 G500 剩余空间 ≥ 20GB
    ↓
检查 G500 已用空间 ≤ 100GB
    ↓
├─ 通过 → 开始采集
└─ 不通过 → 拒绝采集，通知 SessionZipUploader 加速上传
```

#### 9.5.2 自动清理策略

```
定时任务 (每小时执行一次)
    ↓
扫描 scan_root/ 下的所有数据集
    ↓
对于每个数据集:
    ├─ 检查是否所有会话都已上传完成
    ├─ 检查数据集目录是否为空
    └─ 如果都满足 → 删除数据集目录
```

---

## 十、辅助脚本说明

### 10.1 data_capture_signal_pub.py

**功能**：自动根据关节角/回转角变化发布采集开关信号

**逻辑**：
- 监听 `/joints_angles` 和 `/swivel_angle`
- 30 秒窗口内角度变化 > 10° → 发布信号=1（开始采集）
- 超过 10 分钟无变化 → 强制发布信号=0（自动截断）
- 冷却时间 600 秒（防止频繁启停）

**参数**：
- `publish_rate_hz`: 1.0 Hz
- `window_seconds`: 30.0 s
- `angle_threshold_deg`: 10.0°
- `rise_cooldown_seconds`: 600.0 s
- `max_on_duration_seconds`: 480.0 s

### 10.2 data_upload_to_check.py

**功能**：手动上传单个会话到 NAS 进行质检

**用法**：
```bash
python3 data_upload_to_check.py /media/nvidia/G500/excavator_data_20260707/20260707082204
```

**参数**：
- `--nas-url`: NAS 地址（默认 `http://192.168.1.240:8082`）
- `--chunk-size`: 分块大小（默认 8MB）
- `--retries`: 重试次数（默认 3）

### 10.3 nas_ds923_watchdog_1014.sh

**功能**：监控 NAS DS923+ 健康状态，自动 SSH 远程重启 DS923_app.py

**逻辑**：
- 每 30 秒检查 `http://192.168.1.245:8082/` 是否返回 "Sany Hello"
- 连续 3 次失败 → SSH 到 NAS 重启 DS923_app.py
- 使用 flock 防止多实例运行
- 日志按天轮动，保留 72 小时

### 10.4 YX_auto_start_smolvla_20260813_省略检测_ok.sh

**功能**：整机自动启动脚本，启动所有节点（roscore、DPC、传感器、决策、数据采集等）

**启动顺序**：
1. roscore
2. DPC 节点 (smooth/decision/gp/dp/rs/ut 等)
3. ACT (action_fused)
4. 传感器 (rslidar/imu/rtk/localization/excavation_to_gridmap)
5. 软件 (gui/excavator/truck/intel_panel)
6. 轨迹融合 (traj_fusion)
7. GP2mid (决策层)
8. 故障管理 (fault_manager)
9. 数据采集 (data_to_NAS_and_ubuntu2204 + data_capture_signal_pub)
10. NAS 看门狗 (nas_ds923_watchdog)

---

## 十一、NAS 端服务 (DS923_app.py)

**功能**：Flask Web 服务，接收挖掘机上传的数据

**接口**：
| 路径 | 方法 | 说明 |
|------|------|------|
| `/` | GET | 健康检查（返回 "Sany Hello"） |
| `/upload_init` | POST | 初始化上传任务 |
| `/upload_chunk` | POST | 接收分块数据 |
| `/upload_complete` | POST | 完成上传，合并分块 |
| `/upload_status/<upload_id>` | GET | 查询上传状态 |

**存储结构**：
```
/volume1/sy200_data/          # 默认根目录
├── excavator_data_20261029/  # 日期数据集
│   └── 20261029143025/       # 会话目录
│       ├── bag/
│       ├── camera_data/
│       ├── grayPicture/
│       └── rangePicture/
└── .incoming/                # 临时上传目录
```

**多机型支持**：
```python
CLIENT_ROOT_DIR_MAP = {
    "192.168.1.102": "/volume1/sy750_data",
    "192.168.1.103": "/volume1/sy870_data",
    "192.168.1.104": "/volume1/sy1350_data",
}
```

---

## 十二、依赖项

### 12.1 ROS 包
- `roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`
- `grid_map_msgs` (栅格地图)
- `interfaces` (自定义消息：JointAngles, imu_syd, rtk_cgi610, SinglePoint)

### 12.2 系统库
- `libcurl4-openssl-dev` (HTTP 客户端)
- `libssl-dev` (SHA-256 计算)
- `libjsoncpp-dev` (JSON 解析)
- `libyaml-cpp-dev` (yaml 配置)
- `libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev` (相机拉流)
- `libjpeg-dev`, `libpng-dev` (图像编解码)

### 12.3 Python 3 包
- `flask` (NAS 端服务)
- `requests` (上传脚本)

---

## 十三、编译与运行

### 13.1 编译

```bash
cd /home/yx_planning_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=data_to_nas_and_ubuntu2204
```

### 13.2 运行

```bash
source devel/setup.bash
roslaunch data_to_nas_and_ubuntu2204 data_to_NAS_and_ubuntu2204.launch
```

### 13.3 手动测试采集开关

```bash
# 开始采集
rostopic pub /data_capture_switch_manual std_msgs/Int8 "data: 1" -1

# 停止采集
rostopic pub /data_capture_switch_manual std_msgs/Int8 "data: 0" -1
```

---

## 十四、常见问题与排查

### 14.1 G500 未挂载

**现象**：节点启动失败，日志显示 "DataCapture root is not a directory"

**解决**：
```bash
# 检查挂载
mountpoint /media/nvidia/G500

# 手动挂载
sudo mount /dev/sda1 /media/nvidia/G500
```

### 14.2 相机无数据

**现象**：`/CameraSafety_topic` 持续发布非零值

**排查**：
```bash
# 检查 GStreamer 管道
gst-launch-1.0 v4l2src device=/dev/video0 ! appsink

# 检查相机设备
ls -l /dev/video*
```

### 14.3 NAS 上传失败

**现象**：日志显示 "NAS upload failed"

**排查**：
```bash
# 检查 NAS 连通性
curl http://192.168.1.245:8082/

# 检查 DS923_app.py 状态
ssh sanyNAS@192.168.1.245 "ps aux | grep DS923_app.py"

# 手动重启 NAS 服务
bash nas_ds923_watchdog_1014.sh
```

### 14.4 大脑通信超时

**现象**：`/smolVLA_status_check` 发布非零值

**排查**：
```bash
# 检查大脑服务
curl http://192.168.1.120:8082/state_data -X POST -d '{}'

# 检查网络延迟
ping 192.168.1.120
```

---

## 十五、版本历史

| 日期 | 版本 | 改动 |
|------|------|------|
| 2026-07-03 | v1 | 基于 Python 程序改 C++，新增控制/轨迹融合/VLA 轨迹订阅 |
| 2026-07-06 | v2 | 移除 OpenCV，换 GStreamer 拉流，libpng 处理图片 |
| 2026-07-07 | v3 | 修复摄像头数据保存频率低的问题 |
| 2026-07-09 | v4 | 修复摄像头功能安全误报问题 |
| 2026-07-12 | v5 | 解决 NAS 数据上传问题 |
| 2026-08-10 | v6 | 修改数据保存及上传逻辑：先质检、再打包上传 NAS、存 log、删除本地文件 |
| 2026-08-12 | v7 | 将数据质检压缩和上传放到两个不同的线程 |
| 2026-09-02 | v8 | smolvla_status_check 话题的值从 0/1 变为 0~6，表示大脑通信故障的具体问题 |
| 2026-09-09 | v9 | 删除已完成上传的历史日期空目录及 capture_upload.log |

---

## 十六、学习建议

1. **先理解数据流**：从话题订阅开始，理清每个话题的数据来源和用途
2. **关注会话管理**：理解 session 目录的创建、写入、关闭、上传流程
3. **研究质检逻辑**：SessionZipUploader 的质检、打包、分块上传是核心
4. **对比 Python 原版**：如果有条件，对比早期的 Python 实现理解 C++ 改造思路
5. **调试技巧**：使用 `rostopic echo` 观察发布话题，使用日志文件排查问题

---

## 十七、相关文档

- [ROS 官方文档](http://wiki.ros.org/)
- [GStreamer 文档](https://gstreamer.freedesktop.org/documentation/)
- [Flask 文档](https://flask.palletsprojects.com/)
- [群晖 DS923+ 开发者文档](https://www.synology.com/developer)

---

**文档维护**：本文档基于源码 `data_to_NAS_and_ubuntu2204_node.cpp` (4835 行) 及辅助脚本整理生成。
