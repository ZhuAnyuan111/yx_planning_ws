
###变更纪律
# 1、2026.8.21 DZH
# 1.增加清台阶段第二挖掘点卸载点读取 2.清台阶段 一阶段记录 用于2阶段变更 
# 3.清台阶段二阶段、四阶段挖掘机点卸载点变更4.7阶段融合至清台5.清台二阶段慢速曲线，无超调
# 
# 
import rospy
import numpy as np
import math
from typing import Optional, List, Dict, Any, Deque
from dataclasses import dataclass
from collections import deque
import time


from enum import IntEnum
import threading
import random
from .utils.read_yaml import read_yaml

# ROS消息导入
from std_msgs.msg import Float64, Float32, Int32, UInt32, Header, Bool
from geometry_msgs.msg import Quaternion, Point, Pose, PointStamped, PoseStamped
from sensor_msgs.msg import PointCloud2, JointState
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from grid_map_msgs.msg import GridMap
import actionlib
import actionlib_msgs.msg as actionlib_msgs
from interfaces.msg import JointAngles, JointCoordinates, IncenterData, SinglePoint, imu_syd, rtk_cgi610
from .utils.filter import filter as rate_limit_filter
from .utils.rate_dataclass import RateLimitConfig
from .utils.flat_PIDboom_controller import FlatBoomPidController
from .utils.angle_unwrapping import angle_diff2normal_area




def interpolate(x, x_points, y_points):
    # 边界条件检查
    if len(x_points) == 0 or len(y_points) == 0 or len(x_points) != len(y_points):
        return 0.0

    # 处理x小于最小点的情况
    if x <= x_points[0]:
        return y_points[0]

    # 处理x大于最大点的情况
    if x >= x_points[-1]:
        return y_points[-1]

    # 找到包含x的区间
    i = 0
    while i < len(x_points) - 1 and x_points[i + 1] <= x:
        i += 1

    # 线性插值计算
    x0 = x_points[i]
    x1 = x_points[i + 1]
    y0 = y_points[i]
    y1 = y_points[i + 1]

    # 线性插值公式: y = y0 + (y1-y0) * (x-x0) / (x1-x0)
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def randomnumber():
    random_num = random.uniform(-1.5, 1.5)
    return random_num


class ExcavatorState(IntEnum):
    """挖掘机状态枚举"""
    INITIALIZING = 0
    RESETTING = 1
    DIGGING = 2
    UPPING = 3
    UNLOADING = 4
    FLATLAND = 5


class ActionFusedNode:
    def __init__(self, node_name: str='action_data_fusion_node'):

        self.excavator_state = ExcavatorState.INITIALIZING

        self._init_variables()
        self._init_ros_interfaces()
        # self._init_yaml_params()


    def _init_variables(self):
        #  dengzhihong = #
        self.randomnumberoutput: float = 0.0  # <random>
        self.real_boom_angle_velocity: float = 0.0
        self.real_arm_angle_velocity: float = 0.0
        self.real_bucket_angle_velocity: float = 0.0
        self.real_arm_angle_radian0: float = 0.0  # 转换为弧度
        # self.ref_boom_angle: float = 0.0
        # self.ref_arm_angle: float = 0.0
        # self.ref_bucket_angle: float = 0.0
        self.ref_swing_angle: float = 0.0
        # self.ref_swing_angle_limit: float = 0.0
        # self.ref_swing_angleMax: float = 190.0
        # self.ref_swing_angle_Min: float = 140.0
        # 挖掘参数角度
        self.ref_bucket_cutting_angle: float = 0.0
        # 抹平铲斗角度
        self.ref_bucket_cutting_angle6: float = 0.0
        # 清台铲斗角度
        # self.ref_bucket_cutting_angle7: float = 0.0
        self.ref_bucket_cutting_angle_stage7 : float = 0.0
        self.current_arm_angle_radian: float = 0.0
        self.real_arm_angle_radian: float = 0.0
        self.wxref_boom_angle: float = 0.0
        self.wxref_arm_angle: float = 0.0
        self.wxref_bucket_angle: float = 0.0
        self.wxref_swing_angle: float = 0.0
        self.record_4_wxref_swing_angle: float = 0.0
        self.record_4_4_wxref_swing_angle: float = 0.0
        self.record_5_wxref_swing_angle: float = 0.0
        
        self.record_2_wxref_swing_angle: float = 0.0
        self.record_2_current_angle_swing_end: float = 0.0
        self.record_4_current_angle_swing_end: float = 0.0
        self.amendment_current_angle_swing: float = 0.0
        self.record_4_wxref_swing_angle_end: float = 0.0

        self.record_3_wxref_swing_angle: float = 0.0
        self.record_3_current_angle_swing_end: float = 0.0
        
        self.record_2_wxref_swing_angle_end: float = 0.0

        self.record_7_wxref_swing_angle: float = 0.0
        self.record_7_current_angle_swing_end: float = 0.0
        self.record_7_wxref_swing_angle_end: float = 0.0
        # self.record_1_wxref_swing_angle: float = 0.0
        self.record_4_swing_clockwise_switch=False
        self.current_2_swing_clockwise_switch=False
        self.phase5to4_start_active=False
        self.phase6to2_start_active=False
        self.dif_swing_angle_phase6to2: float = 0.0
        
        self.record_1_state: float = 0.0
        self.record_4_state: float = 0.0
        self.record_5_state: float = 0.0

        self.wxref_boom_angle_orig: float = 0.0
        self.wxref_arm_angle_orig: float = 0.0
        self.wxref_bucket_angle_orig: float = 0.0
        self.wxref_swing_angle_orig: float = 0.0
        self.wxref_limit_swing_angle: float = 0.0
        self.wxref_limit_ad_swing_angle: float = 0.0
        self.wxref_swing_angle_stage2: float = 0.0
        self.wxref_swing_angle_stage2_limit: float = 0.0
        self.wxref_swing_angle_stage2_limit_ad: float = 0.0
        
        
        
    



        self.last_ori_traj_time_: Optional[rospy.Time] = None
        self.wxref_traj_initialized_ = False
        # self.max_rate_up: float = 25.0
        # self.max_rate_down: float = 25.0
        self.dif_bucket_anglepid: float = 0.0
        

        self.real_swing_angle_velocity: float = 0.0
        self.real_swing_angle: float = 0.0
        self.dif_swing_angle: float = 0.0  # 回转角度差值
        self.dif_swing_angleadd: float = 0.0
        self.dif_swing_angle_end: float = 0.0
        self.ref_swing_angle_temp: float = 0.0
        self.dif_bucket_angle: float = 0.0  # 铲斗角度差值
        self.dif_bucket_anglepid3: float = 0.0  # 铲斗角度差值
        self.dif_bucket_anglepid4: float = 0.0  # 铲斗角度差值
        self.dif_bucket_anglepid6: float = 0.0  # 铲斗角度差值
        self.dif_bucket_anglepid7: float = 0.0  # 铲斗角度差值
        self.dif_bucket_anglepid2: float = 0.0  # 铲斗角度差值
        self.dif_arm_anglepid3: float = 0.0  # arm角度差值
        self.dif_arm_anglepid4: float = 0.0  # 铲斗角度差值

        self.dif_arm_anglepid6: float = 0.0  # arm角度差值
        self.dif_boom_anglepid6: float = 0.0  # boom角度差值
        self.dif_arm_anglepid7: float = 0.0  # arm角度差值
    
  
        self.lim_dif_bucket_anglepid3: float = 0.0  # 铲斗角度差值
        self.lim_dif_bucket_anglepid4: float = 0.0  # 铲斗角度差值
        self.lim_dif_arm_anglepid3: float = 0.0  # 斗杆角度差值
        self.lim_dif_arm_anglepid4: float = 0.0  # 斗杆角度差值
        


        self.dif_boom_anglepid3: float = 0.0  # 动臂角度差值
        self.dif_boom_anglepid4: float = 0.0  # 动臂角度差值

        self.seqactionmode: float = 0.0
        self.last_seqactionmode: float | None = None

        # self.feedforward_swing_pilot: float = 0.0
        # self.feedforward_boom_pilot: float = 0.0
        # self.feedforward_arm_pilot: float = 0.0
        # self.feedforward_bucket_pilot: float = 0.0
        self.boom_pilot_fused_data: float = 0.0
        self.arm_pilot_fused_data: float = 0.0
        self.bucket_pilot_fused_data: float = 0.0
        self.swing_pilot_fused_data: float = 0.0

        self.swing_pilot_fused_data_temp: float = 0.0
        self.swing_pilot_fused_data_temp2: float = 0.0
        # self.swing_pilot_fused_data_temp_end: float = 0.0
        self.arm_pilot_fused_data_temp: float = 0.0
        self.arm_pilot_fused_data_temp3: float = 0.0
        self.arm_pilot_fused_data_temp4: float = 0.0
        self.bucket_pilot_fused_data_temp2: float = 0.0
        self.bucket_pilot_fused_data_temp_end: float = 0.0
        self.bucket_pilot_fused_data_temp3: float = 0.0
        self.bucket_pilot_fused_data_temp4: float = 0.0
        self.bucket_pilot_fused_data_temp6: float = 0.0
        self.boom_pilot_fused_data_temp4: float = 0.0

        self.fusion_controller_joystickx: float = 0.0  # 手柄回转信号
        self.fusion_controller_joysticky: float = 0.0  # 手柄动臂信号
        self.fusion_controller_joystickz: float = 0.0  # 手柄斗杆信号
        self.fusion_controller_joystickw: float = 0.0  # 手柄铲斗信号
        # 新增6阶段参数
        self.bucket_truck_distance: float = 0.0
        self.bucket_ground_distance: float = 0.0

        self.calculate_boom3_pilot: float = 0.0


        self.calculate_arm_pilot: float = 0.0
        self.calculate_bucket_pilot: float = 0.0
        self.calculate_arm4_pilot: float = 0.0
        self.wxref_arm_angle_after_filter: float = 0.0
        self.digpoint_angle: float = 0.0
        self.clean_digpoint_angle: float = 0.0      # /clean_digPoint 的 x：清台阶段挖掘点
        self.clean_digpoint_received: bool = False  # /clean_digPoint 是否已收到有效数据
        # self.wxref_arm_angle_after_filter= None
        self.last_sys_sco_traj_time_: rospy.Time | None = None
        # 新增7阶段参数

        self.rate_limit = RateLimitConfig()
        

        self.k_p: float = 0.0  # PD比例增益
        self.k_d: float = 0.0  # PD微分增益
        self.error: float = 0.0  # 上上一次的误差（用于微分计算）
        self.prev_error: float = 0.0  # 上一次误差

        # self.integral_reset_threshold: float = 50.0                    # PD比例增益
        self.integral_bucket_error: float = 0.0                    # PD微分增益
        # self.max_integral: float = 0.0
        # self.min_integral: float = -400.0      # 上上一次的误差（用于微分计算）
        # self.k_i_bucket: float = 2             # 上一次误差
        # self.k_i_swing: float = 2  
        # self.combined_output: float = 0.0
        # self.integral_reset_threshold_swing: float = 50.0 
        # self.k_i_swing: float = 2  
        self.swing_integral_control: float = 0.0
        self.swing_integral_control_4: float = 0.0
        self.swing_integral_control_7: float = 0.0
        # self.integral_bucket_error: float = 0.0
        self.integral_swing_error: float = 0.0
        # self.integral_swing_error_limt: float = 0.0
        # self.integral_control_limt

        self.output: float = 0.0
        self.maxOutput: float = 1000.0
        self.PIDOutPut: float = 0.0
        self.swing_input_points: list[float] = []  # 回转输入点
        self.swing_output_points: list[float] = []  # 回转输出点

        self.prev_diff_current_between_ref: float = 0.0  # 上一循环的-挖机航向角与车尾角差值
        self.prev_angle: float = 0.0       # 上一循环的-挖机航向角
        self.p_prev_angle: float = 0.0       # 上-上一循环的-挖机航向角
        self.EPSILON: float = 1e-3     #容差
        # = dengzhihong #
        # = hukaiting 
        self.ef_tip_rotation_x: float = 0.0
        self.ef_tip_rotation_y: float = 0.0
        self.ef_tip_rotation_z: float = 0.0

        self.excavator_state_msg = Int32()      #std_msgs::Int32()
        self.excavator_state_flag = Bool()      #std_msgs::Bool()
        self.excavator_state.data = 0
        self.excavator_state_flag.data = False

        self.bucket_attitude_angle_threshold: float | None = None
        self.max_bucket_joint_angle: float | None  = None
        self.min_bucket_joint_angle: float | None  = None
        self.rtk_heading: float = 0.0
        self.dig_rtk_heading_angle: float = 0.0
        self.vehicle_pitch: float = 0.0
        self.dig_vehicle_pitch: float = 0.0
        self.data_mutex= threading.Lock()

        self.dig_flag: bool = True
        self.delta_dig_pitch: float | None = None
        self.angular_velocity = Float32()
        self.angular_velocity.data = 0.0
        self.joint_angles: Optional[Quaternion] = None
        self.max_upping_z: float = 0.0
        self.swivel_angle: Float32 | None = Float32()
        self.swivel_angle.data = 0.0
        self.dig_xyz: Point | None = Point()
        self.delta_dig_distance: float | None = None    #Threshold for significant change in distance

        self.working_alt_down: float = 0.0              # 铲斗工作的绝对高程下限
        self.INITIALIZING_FLAG: bool = False
        self.bucket_point_stamped: PointStamped | None = PointStamped()    #geometry_msgs::PointStamped
        self.grid_map_: GridMap | None = None


        self.swivel_angle_flag: bool = True
        self.cutting_angle: float = 0.0             # 铲斗角度阈值
        self.delta_bucket_angles: float = 0.0
        self.pre_bucket_angles: float = 0.0
        # = hukaiting #
        
        # === xiongyan
        # 前馈关闭角度限制
        # self.ff_limit_swing_: float = 10.0
        # self.ff_limit_boom_: float = 10.0
        # self.ff_limit_arm_: float = 10.0
        # self.ff_limit_bucket_: float = 10.0
        # 手柄前馈
        # self.u_swing_feedforward_: float = 0.0
        # self.u_boom_feedforward_: float = 0.0
        # self.u_arm_feedforward_: float = 0.0
        # self.u_bucket_feedforward_: float = 0.0
        # 手柄前馈
        # self.u_swing_feedforward_sub_: float = 0.0
        # self.u_boom_feedforward_sub_: float = 0.0
        # self.u_arm_feedforward_sub_: float = 0.0
        # self.u_bucket_feedforward_sub_: float = 0.0
        # 反馈角度
        self.current_angle_boom_: float = 0.0
        self.current_angle_arm_: float = 0.0
        self.current_angle_bucket_: float = 0.0
        self.current_angle_MNQ_: float = 0.0
        # 反馈角速度
        self.current_vpitch_boom_: float = 0.0
        self.current_vpitch_arm_: float = 0.0
        self.current_vpitch_bucket_: float = 0.0
        # 反馈回转
        self.current_angle_swing_: float = 0.0
        self.current_velovity_swing_: float = 0.0
        # 目标角度
        # self.target_angle_swing_: float = 0.0
        # self.target_angle_boom_: float = 0.0
        # self.target_angle_arm_: float = 0.0
        # self.target_angle_bucket_: float = 0.0
        # 卸载点角度
        self.unload_angle_: float = 0.0
        # MPC开关
        self.mpc_switch_: bool = False
        self.mpc_switch2_: bool = False
        # 航向360-0跳变翻转 开关

        self.swing_clockwise_switch: bool = False

        # 提升阶段动臂最大角度
        self.load_traj_max_boom_: float = 0.0

        # 中值滤波缓存
        # self.window_: Deque[float] = deque()

        # 提升阶段末端角度  ### 上游规划发送
        # self.target_angle_swing_3_: float = 0.0
        self.target_angle_boom_3_: float = 8.0
        # self.target_angle_arm_3_: float = -120
        # self.target_angle_bucket_3_: float = 0.0

        ## u_fused输出  ##############################
        self.u_swing_fused_: float = 0.0
        self.u_boom_fused_: float = 0.0
        self.u_arm_fused_: float = 0.0
        self.u_bucket_fused_: float = 0.0

        # bool target_received_; // 目标关节角度是否接收
        # bool current_received_; // 当前关节角度是否接收

        self.loop_rate_: int | None = None  # 循环频率
        self.last_time_: rospy.rostime.Time | None = None  # 上一次时间
        # self.u_arm_smooth_: float = 0.0
        self.k_: float = 4.0  # 积分增益，越大响应越快
        self.dt_: float = 0.02  # 控制周期（100Hz）
 
        self.stop_swing_angle_flag_: bool = False
        self.phase4_start_active: bool = False
        self.current_swing_angle_: float = 0.0
        self.bucket_is_upper_truck: bool = False
        self.sub_boom_enu: rospy.Subscriber | None = None
        self.sub_bucket_enu: rospy.Subscriber | None = None
        self.sub_truck_enu: rospy.Subscriber | None = None
        self.sub_truck_swive_angle: rospy.Subscriber | None = None
        self.delta_target_sub_current_angle_boom: float | None = None
        self.bucket_azimuth_angle: float = -180.0  # 铲斗方位角
        self.rtk_uu: float = 0.0
        # self.stop_dig_up_boom_flag: bool = False
        self.BUCKET_ANGLE_MIN: float = -116.0  # -116 * M_PI / 180
        self.BUCKET_ANGLE_MAX: float = 47.0  # 47 * M_PI / 180
        self.truck_swive_data_angle: float = 0.0
        self.raw_angle_bkt: float = 0.0
        self.bucket_attact_terrain_flag: bool = False
        self.ref_angle_swing_: float = 0.0
        self.diff_angle_swing_: float = 0.0
        self.unload_bucket_angle_limit_: float = 25.0
        self.unload_bucket_reach_target_angle_flag_: bool = False
        self.min_dig_swivel_angle_: float = 145.0  # 挖掘区域的最小航向角
        self.boom_enu_point: SinglePoint = SinglePoint()  # 铲斗端点位置
        self.bucket_enu_point: SinglePoint = SinglePoint()  # bucket端点位置
        self.truck_enu_point: SinglePoint = SinglePoint()  # 矿卡全局位置

        # self.max_delta_u_boom_: float = 40.0  # 新增变量：定义动臂手柄变化率的最大允许值
        # self.previous_u_boom_fused_ = 0.0  # 新增变量：记录前一个控制变量的值
        self.max_delta_u_arm_ = 15.0  # 新增变量：定义斗杆手柄变化率的最大允许值
        self.previous_u_arm_fused_ = 0.0  # 新增变量：记录前一个控制变量的值
        # self.max_delta_u_bucket_: float = 50.0  # 新增变量：定义铲斗手柄变化率的最大允许值
        # self.previous_u_bucket_fused_: float = 0.0  # 新增变量：记录前一个控制变量的值
        self.distance_bucket_point_and_ground=0.0
        # self.High_Slow_Switch_Values_to_bucket=0.0  # 回转限制铲斗
    

        # 新增PID部分变量
        self.ref_swing_angle_temp: float = 0.0
        self.dif_swing_angleadd: float = 0.0
        self.dif_swing_angle: float = 0.0
        self.dif_swing_angle_end: float = 0.0
        self.swing_pilot_fused_data_temp: float = 0.0
        self.swing_pilot_fused_data_temp2: float = 0.0
        # self.swing_pilot_fused_data_temp_end: float = 0.0
        self.calculate_swing_speed_fast_slow_: float = 0.0
        self.record_current_angle_swing_: float = 0.0
        self.High_Slow_Switch_Values: float = 0.0
        self.calculate_swing_speed_fast_slow_4: float = 0.0
        self.record_current_angle_swing_4: float = 0.0
        self.High_Slow_Switch_Values_4: float = 0.0
        self.High_Slow_Switch_Values_4_5: float = 0.0
        # 新增arm_picth变量26.0125        
        self.arm_pitch: float = 0.0
        self.safety_counter = 0
        # 功能安全
        self.heartbeat = 0      # 0-255 心跳
        self.error_code = 0     # 0-7   错误代码
        self.fault_lock= threading.Lock()

        # 平地阶段(6阶段)PID控制
        self.boomPID_6 = FlatBoomPidController(kp=1000.0, ki=500.0, kd=0.0)

        # 提升卸载阶段计时器，用于区分是否是提升阶段还是卸载阶段
        # self.prev_seqactionmode = None      # 记录上一帧的模式
        self.prev_seqactionmode: float = 0.0 
        self.seqactionmodewx: float = 0.0   #王杏转发阶段
        
        # self.phase4_start_time = None       # 进入阶段4 的时间戳
        self.phase4_start_time: float = 0.0
        self.phase4_timer_active = False    # 是否正在阶段4 计时
        
        # 新增变量
        self.unload_local_point: PointStamped | None = PointStamped()  # geometry_msgs::PointStamped
        self.unload_local_pointdzh: PointStamped | None = PointStamped()  # geometry_msgs::PointStamped
        self.unload_local_point_2: PointStamped | None = PointStamped()  # geometry_msgs::PointStamped
        self.unload_point_received: bool = False    # /truck_center_unloadpoint 是否已收到有效数据
        self.unload_point_2_received: bool = False  # /truck_center_unloadpoint_2 是否已收到有效数据

        # /Sys_RGp_Unload 卸载点（只使用 X、Y 坐标）
        self.sys_unload_point_x: float = 0.0
        self.sys_unload_point_y: float = 0.0
        self.sys_unload_local_point: PointStamped | None = PointStamped()  # 转换到本体坐标系后的卸载点
        self.sys_unload_angle_: float = 0.0         # 由 X、Y 计算出的卸载点回转角度
        self.sys_unload_point_received: bool = False  # /Sys_RGp_Unload 是否已收到有效数据

        self.delta_unload_distance: float = 0.0
        self.unload_truck_full_flag_: bool = False
        self.truck_unload_pose_info_z:  float = 0.0

        # YAML 加载参数
        # 抹平高度参数
        self.stage6_bridge_the_gap: float = 0.0
        #齿尖距离矿卡修正参数     gridmap_avg_z
        self.gridmap_height_err: float = 0.0
        # 动臂铰点距离履带底部位置
        self.base_height: float = 0.0
        self.model_config: float = 0.0
        self.delta_local_unload_ground = PointStamped()
        self.delta_local_unload_ground.header.frame_id = 'base'
        # delta_local_unload_ground.point.z 的滑动均值滤波缓冲区, 窗口大小10
        self.unload_ground_z_buffer: Deque[float] = deque(maxlen=5)


    def _init_ros_interfaces(self):
        self.nh = rospy.get_param('~', {})

        self.bucket_attitude_angle_threshold = rospy.get_param('/bucket_attitude_angle_threshold', 2.4) 
        self.max_bucket_joint_angle = rospy.get_param('/max_bucket_joint_angle', 0.8)
        self.min_bucket_joint_angle = rospy.get_param('/min_bucket_pitch_angle', -0.8)
        self.max_bucket_joint_angle = 0.0
        self.delta_dig_pitch = rospy.get_param('/delta_dig_pitch', 3.0)
        self.delta_dig_distance = rospy.get_param('/delta_dig_distance', 1.0)
        self.working_alt_down = rospy.get_param('/working_alt_down', 0.0)
        self.cutting_angle = rospy.get_param('/dig_cutting_angle', 0.0)
        self.delta_unload_distance = rospy.get_param('/delta_unload_distance', 0)
        self.unload_height_offset = rospy.get_param('/unload_height_offset')
        while not rospy.has_param('/GP2mid/rear_cable_deg'):
            rospy.loginfo("Waiting for params")
            rospy.sleep(0.5)
        self.rear_cable_deg = rospy.get_param('/GP2mid/rear_cable_deg', 120.0)
        self.record_7_wxref_swing_angle = rospy.get_param('/GP2mid/front_lane_deg', 120.0)
        print("rear_cable_deg: {}".format(self.rear_cable_deg))
        self.angle_cable_lb_deg  = (self.rear_cable_deg - 50)%360
        self.angle_cable_ub_deg = (self.rear_cable_deg + 50)%360
        # print(f"angle_cable_ub_deg----angle_cable_ub_deg:{self.angle_cable_ub_deg}")
        # 抹平高度参数
        self.stage6_bridge_the_gap= rospy.get_param('/stage6_bridge_the_gap', 0.2)
        # print(f"抹平高度参数----stage6_bridge_the_gap:{self.stage6_bridge_the_gap}")
        #齿尖距离矿卡修正参数     gridmap_avg_z
        self.gridmap_height_err =  rospy.get_param('/gridmap_height_err', 0.0)
        self.base_height=  rospy.get_param('/base_height', 2.7)
        self.ref_bucket_cutting_angle = rospy.get_param('/ref_bucket_cutting_angle', 5)
        self.ref_bucket_cutting_angle_stage7 =rospy.get_param('/ref_bucket_cutting_angle_stage7', 0)
        self.model_config =rospy.get_param('/model_config', 0)
        print(f"0000000000   model_config:{self.model_config}")
        

        # rospy.loginfo("\033[1;31mInitialized action_data_fusion_node Node, parameters as follows:\033[0m")
        # rospy.loginfo("\033[1;33mbucket_attitude_angle_threshold: %f\033[0m", self.bucket_attitude_angle_threshold)
        # rospy.loginfo("\033[1;33mbucket_joint_angle: %f\033[0m", self.max_bucket_joint_angle)
        # rospy.loginfo("\033[1;33mmin_bucket_joint_angle: %f\033[0m", self.min_bucket_joint_angle)
        # rospy.loginfo("\033[1;33mdelta_dig_pitch: %f\033[0m", self.delta_dig_pitch)
        # rospy.loginfo("\033[1;33mdelta_dig_distance: %f\033[0m", self.delta_dig_distance)
        # rospy.loginfo("\033[1;33mworking_alt_down: %f\033[0m", self.working_alt_down)
        # rospy.loginfo("\033[1;33mcutting_angle: %f\033[0m", self.cutting_angle)

        self.pub_excavator_state = rospy.Publisher('/excavator_state', Int32, queue_size=1)
        self.pub_excavator_state_flag = rospy.Publisher('/excavator_state_flag', Bool, queue_size=1)
        self.pub_angular_velocity = rospy.Publisher('/angular_velocity', Float32, queue_size=1)
        self.pub_swivel_angle = rospy.Publisher('/swivel_angle', Float32, queue_size=1)
        self.pub_bucket_point = rospy.Publisher('/bucket_terrain_delta_position', PointStamped, queue_size=1)
        self.pub_bucket_real_point = rospy.Publisher('/bucket_real_position', PointStamped, queue_size=1)
        self.pub_unload_truck_full_flag = rospy.Publisher('/FlagTruckFull', Bool, queue_size=1)
        self.sub_ori_traj = rospy.Subscriber('/Sys_SCo_Traj', Pose, self.oriDeviceTrajCallback, queue_size=1)
        self.sub_ori_traj_stage2 = rospy.Subscriber('/2stage_traj_topic', Pose, self.stage2_TrajCallback, queue_size=1)
        self.sub_digpoint_angle = rospy.Subscriber('/Sys_RGp_SwWkAgl', Point, self.DigPoint_angleCallback, queue_size=1)
        self.sub_clean_digpoint = rospy.Subscriber('/clean_digPoint', Quaternion, self.clean_digPointCallback, queue_size=1)  # x：清台阶段挖掘点
        # self.action_data_pid_pub = rospy.Publisher('/pid_controller_joystick_topic', Quaternion, queue_size=10)
        self.u_fused_pub_ = rospy.Publisher('/fused_controller_joystick_topic', Quaternion, queue_size=1000)
        self.test_u_fused_pub_ = rospy.Publisher('/test_fused_controller_joystick_topic', Quaternion, queue_size=1000)
        self.sub_joint_angles = rospy.Subscriber('/joints_angle', Quaternion, self.joint_angles_callback, queue_size=1)
        self.sub_rtk_data = rospy.Subscriber('/rtk_can_msg', rtk_cgi610, self.ENUDataTopicCallback, queue_size=1)
        self.sub_max_upping_z = rospy.Subscriber('/loadTraj_maxZ', Float32, self.max_upping_z_callback, queue_size=1)
        self.sub_max_boom_angle = rospy.Subscriber('/loadTraj_maxBoom', Float32, self.max_boom_angle_callback, queue_size=1)
        self.sub_ef_tip_rotation = rospy.Subscriber('/RealBktPosXYZ', Point, self.EFTipRotationTopicCallback, queue_size=1)
        self.sub_grid_map = rospy.Subscriber('/excavation_gridmap1', GridMap, self.grid_map_callback, queue_size=1)

        self.joint_velocity_sub = rospy.Subscriber('/joint_velocity', Quaternion, self.joint_velocityTopicCallback, queue_size=10)
        self.Swing_clockwise_switch_sub = rospy.Subscriber('/swing_clockwise_flag', Bool, self.swing_clockwise_flag_TopicCallback, queue_size=1)
        self.Mpc_Switch_sub = rospy.Subscriber('/Mpc_Switch', Bool, self.Mpc_SwitchTopicCallback, queue_size=1)
        self.Mpc_Switch2_sub = rospy.Subscriber('/Mpc_Switch2', Bool, self.Mpc_Switch2TopicCallback, queue_size=1) #清台功能
        # self.Ref_angle_sub = rospy.Subscriber('/Sys_SCo_RefDeviceTraj', Pose, self.Ref_angleTopicCallback, queue_size=1)
        # self.controller_joystick_topic_lstm_sub = rospy.Subscriber('/controller_joystick_topic_lstm', Quaternion, self.controller_joystick_topic_lstmTopicCallback, queue_size=1)
        self.Sys_SeqAction_sub = rospy.Subscriber('/Sys_SeqAction', Float64, self.Sys_SeqActionTopicCallback, queue_size=1)
        self.SwingTopic_sub = rospy.Subscriber('/Swing_topic', Point, self.SwingTopicCallback, queue_size=1)
        self.heading2swing_topic_sub = rospy.Subscriber('/heading2swing_topic', Float32, self.heading2swingCallback, queue_size=1)
        # self.controller_joystick_sub = rospy.Subscriber('/controller_joystick_topic', Quaternion, self.controller_joystick_topicCallback, queue_size=1)
        # self.controller_PID_U0_sub = rospy.Subscriber('/U0', Quaternion, self.controller_PID_U0_topicCallback, queue_size=1)


        #  新增的卸载点订阅
        self.sub_unload_point = rospy.Subscriber('/truck_center_unloadpoint', Quaternion, self.unload_point_callback, queue_size=1) # X轴：正北 Y轴：正西
        self.sub_unload_point_2 = rospy.Subscriber('/truck_center_unloadpoint_2', Quaternion, self.unload_point_2_callback, queue_size=1) # X轴：正北 Y轴：正西
        self.sub_sys_rgp_unload = rospy.Subscriber('/Sys_RGp_Unload', Quaternion, self.Sys_RGp_UnloadCallback, queue_size=1)  # 读取 X、Y 用于计算卸载点
        self.delta_local_unload_ground_pub = rospy.Publisher('/delta_local_unload_ground', PointStamped, queue_size=1)
        self._rear_angle = rospy.Subscriber('/dig_region_reference_angles', Point, self.rear_cableCallback, queue_size=1)
        self.truck_unload_pose_info_sub = rospy.Subscriber('/truck_pose_topic',PoseStamped, self.truck_unload_pose_info_callback, queue_size=100)



        # self.u_feedforward_sub_ = rospy.Subscriber('/u_feedforward', Quaternion, self.u_feedforwardCallback, queue_size=1000)
        # self.ff_limit_sub_ = rospy.Subscriber('/ff_limit', Quaternion, self.ff_limitCallback, queue_size=1000)
        self.current_vpitch_sub_ = rospy.Subscriber('/joints_velocity', Quaternion, self.current_vpitch_msgCallback, queue_size=1000)
        # self.target_angle_sub_ = rospy.Subscriber('/target_angle', Pose, self.target_angleCallback, queue_size=1000)
        # self.dig_end_angle_sub_ = rospy.Subscriber('/UT_debugStartJoint', Pose, self.target_angle_3Callback, queue_size=1000)
        # rospy.loginfo("\033[1;33m excav control node init success:\033[0m")

        self.sub_boom_enu = rospy.Subscriber('/boom_anchor_enu', SinglePoint, self.boom_enu_callback, queue_size=1000)
        self.sub_bucket_enu = rospy.Subscriber('/ef_tip_enu_1', SinglePoint, self.bucket_enu_callback, queue_size=1000)
        self.sub_truck_enu = rospy.Subscriber('/truck_enu', SinglePoint, self.truck_enu_callback, queue_size=1000)
        self.sub_truck_swive_angle = rospy.Subscriber('/truck_swive_angle', Float32, self.truck_swive_callback, queue_size=1000)
        self.sub_arm_imu_topic = rospy.Subscriber('/imu_topic_arm', imu_syd, self.arm_imu_topic_callback, queue_size=1000)

        self.sub_vehicle_imu_topic = rospy.Subscriber('/imu_topic_vehicle', imu_syd, self.imuvehicleTopicCallback, queue_size=1000)

        self.loop_rate_ = 50
        self.last_time_ = rospy.Time.now()


    def DigPoint_angleCallback(self, msg: Point): 
        # self.current_angle_swing_ = msg.x
        self.digpoint_angle = msg.x

        # print(f"挖掘点角度digpoint_angle:{self.digpoint_angle}")

    def clean_digPointCallback(self, msg: Quaternion):
        """/clean_digPoint：x 为清台阶段挖掘点"""
        self.clean_digpoint_angle = msg.x
        self.clean_digpoint_received = True
        # print(f"清台阶段挖掘点clean_digpoint_angle:{self.clean_digpoint_angle}")

    def rear_cableCallback(self, msg: Point):
        self.record_7_wxref_swing_angle = msg.x
        self.rear_cable_deg = msg.y
        print("self.rear_cable_deg = msg.y {}".format(self.rear_cable_deg))
        self.angle_cable_lb_deg  = (self.rear_cable_deg - 5)%360
        self.angle_cable_ub_deg = (self.rear_cable_deg + 5)%360

    def joint_angles_callback(self, msg: Quaternion):
        with self.data_mutex:
            if self.joint_angles is not None:
                self.pre_bucket_angles = self.joint_angles.z
            self.joint_angles = msg
            self.delta_bucket_angles = self.pre_bucket_angles - msg.z

            self.current_angle_boom_ = msg.x
            self.current_angle_arm_ = msg.y
            self.current_angle_bucket_ = msg.z
            self.current_angle_MNQ_ = msg.w
            self.current_arm_angle_radian = self.current_angle_arm_ * math.pi / 180.0

            # calculateBucketReference(current_arm_angle_radian)*180/M_PI;; 
            self.calculateBucketRef()

            self.pub_excavator_state.publish(self.excavator_state)
            self.pub_excavator_state_flag.publish(self.excavator_state_flag)
            self.pub_angular_velocity.publish(self.angular_velocity)

    def ENUDataTopicCallback(self, msg: rtk_cgi610):
        with self.data_mutex:
            self.rtk_heading = msg.heading
            self.rtk_uu = msg.uu
            # self.angular_velocity.data = msg.ang_z
            if self.swivel_angle_flag:
                self.dig_rtk_heading_angle = msg.heading

            self.swivel_angle.data = -msg.heading + 180.0
            if self.swivel_angle.data <= 0.0:
                self.swivel_angle.data += 360.0

    def max_upping_z_callback(self, msg: Float32):
        self.max_upping_z = msg.data

    def max_boom_angle_callback(self, msg: Float32):
        self.load_traj_max_boom_ = msg.data

    def truck_unload_pose_info_callback(self, msg: PoseStamped):
        self.truck_unload_pose_info_z = msg.pose.position.z
    
    def unload_point_callback(self, msg: Quaternion):
        self.delta_unload_distance = rospy.get_param('/delta_unload_distance', 0)
        self.unload_local_point.header.stamp = rospy.Time.now()
        self.unload_local_pointdzh.point.x = msg.x  #   原始
        self.unload_local_pointdzh.point.y = msg.y
        self.unload_local_pointdzh.point.z = msg.z
        self.unload_local_point.point.x = -msg.y  #  8.21
        self.unload_local_point.point.y = msg.x
        self.unload_local_point.point.z = msg.z
        self.unload_point_received = True

    def unload_point_2_callback(self, msg: Quaternion):
        self.unload_local_point_2.header.stamp = rospy.Time.now()
        self.unload_local_point_2.point.x = -msg.y  #   原始   3.027 -2.445
        self.unload_local_point_2.point.y = msg.x
        self.unload_local_point_2.point.z = msg.z
        self.unload_point_2_received = True

    def Sys_RGp_UnloadCallback(self, msg: Quaternion):
        """/Sys_RGp_Unload：读取 X、Y 坐标，用于计算卸载点"""
        self.sys_unload_point_x = msg.x
        self.sys_unload_point_y = msg.y
        # 与 /truck_center_unloadpoint 保持一致的坐标转换
        self.sys_unload_local_point.header.stamp = rospy.Time.now()
        self.sys_unload_local_point.point.x = -msg.y
        self.sys_unload_local_point.point.y = msg.x
        # 由原始 X、Y 计算卸载点回转角度
        self.sys_unload_angle_ = self.normalize_angle(math.atan2(-msg.y, -msg.x) * 180.0 / math.pi)
        self.sys_unload_point_received = True
        print(f"卸载点角度sys_unload_angle_:{self.sys_unload_angle_}")


    def EFTipRotationTopicCallback(self, msg):
        if hasattr(msg, 'point'):
            self.ef_tip_rotation_x = msg.point.x
            self.ef_tip_rotation_y = msg.point.y
            self.ef_tip_rotation_z = msg.point.z
        else:
            self.ef_tip_rotation_x = msg.x
            self.ef_tip_rotation_y = msg.y
            # self.ef_tip_rotation_z = msg.z - 0.9
            self.ef_tip_rotation_z = msg.z 

        # bucket_position_0 = -self.ef_tip_rotation_y
        # bucket_position_1 = self.ef_tip_rotation_x
        # bucket_position_2 = self.ef_tip_rotation_z
       
        # 原转换方式  将朱安源的转换到栅格地图坐标系下  0722修改
        bucket_position_0 = -self.ef_tip_rotation_y
        bucket_position_1 = self.ef_tip_rotation_x
        bucket_position_2 = self.ef_tip_rotation_z

        self.bucket_point_stamped.header.frame_id = 'base'
        self.bucket_point_stamped.header.stamp = rospy.Time.now()
        self.bucket_point_stamped.point.x = bucket_position_0
        self.bucket_point_stamped.point.y = bucket_position_1
        self.bucket_point_stamped.point.z = bucket_position_2

        if not hasattr(msg, 'point'):
            self.pub_bucket_real_point.publish(self.bucket_point_stamped)

    def grid_map_callback(self, msg: GridMap):
        with self.data_mutex:
            self.grid_map_ = msg
        self.excavator_state_process_data()

    def joint_velocityTopicCallback(self, msg: Quaternion):
        self.real_boom_angle_velocity = msg.y
        self.real_arm_angle_velocity = msg.z
        self.real_bucket_angle_velocity = msg.w

    def Mpc_SwitchTopicCallback(self, msg: Bool):
        self.mpc_switch_ = msg.data
    def Mpc_Switch2TopicCallback(self, msg: Bool):
        self.mpc_switch2_ = msg.data
    def swing_clockwise_flag_TopicCallback(self, msg: Bool):
        self.swing_clockwise_switch = msg.data
        # printf("self.swing_clockwise_switch:\n",self.swing_clockwise_switch)

    # def Ref_angleTopicCallback(self, msg: Pose):
    #     self.ref_swing_angle_limit = msg.orientation.x
    #     self.ref_boom_angle = msg.orientation.y
    #     self.ref_arm_angle = msg.orientation.z
    #     self.ref_bucket_angle = msg.orientation.w

    # def controller_joystick_topic_lstmTopicCallback(self, msg: Quaternion):
    #     self.feedforward_swing_pilot = msg.x
    #     self.feedforward_boom_pilot = msg.y
    #     self.feedforward_arm_pilot = msg.z
    #     self.feedforward_bucket_pilot = msg.w

    def Sys_SeqActionTopicCallback(self, msg: Float64):
        self.seqactionmode = msg.data

    def SwingTopicCallback(self, msg: Point):
        # self.current_angle_swing_ = msg.x
        self.current_velovity_swing_ = msg.y
        self.ref_angle_swing_ = msg.z
    def heading2swingCallback(self, msg: Float32):
        self.current_angle_swing_ = msg.data
        # print(f"current_angle_swing_:{self.current_angle_swing_}")


        

    # def controller_joystick_topicCallback(self, msg: Quaternion):
    #     self.fusion_controller_joystickx = msg.x
    #     self.fusion_controller_joysticky = msg.y
    #     self.fusion_controller_joystickz = msg.z
    #     self.fusion_controller_joystickw = msg.w
    
    def stage2_TrajCallback(self, msg: Pose):
        self.wxref_swing_angle_stage2_limit= msg.orientation.x
        if self.wxref_swing_angle_stage2_limit > 540:
            self.wxref_swing_angle_stage2_limit_ad= self.wxref_swing_angle_stage2_limit-360
        else:
            if self.wxref_swing_angle_stage2_limit <-180:
                self.wxref_swing_angle_stage2_limit_ad= self.wxref_swing_angle_stage2_limit+360
            else:
                self.wxref_swing_angle_stage2_limit_ad= self.wxref_swing_angle_stage2_limit
        self.wxref_swing_angle_stage2 =self.wxref_swing_angle_stage2_limit_ad
        # print(f"回调输出2阶段wxref_swing_angle_stage2:{self.wxref_swing_angle_stage2}")

    def oriDeviceTrajCallback(self, msg: Pose):
        self.bucket_truck_distance= msg.position.x    #铲斗与矿卡上表面距离
        self.bucket_ground_distance= msg.position.y    #铲斗
        self.seqactionmodewx = msg.position.z 
        # print(f"王杏转发阶段seqactionmodewx:{self.seqactionmodewx}")
        # self.wxref_swing_angle = msg.orientation.x
        # self.wxref_boom_angle = msg.orientation.y
        # self.wxref_arm_angle = msg.orientation.z
        # self.wxref_bucket_angle = msg.orientation.w
        now = rospy.Time.now()
        dt = self.dt_
        if self.last_ori_traj_time_ is not None:
            dt_meas = (now - self.last_ori_traj_time_).to_sec()
            if dt_meas > 0.0 and dt_meas < 1.0:
                dt = dt_meas
        self.last_ori_traj_time_ = now

        if not self.wxref_traj_initialized_:
            self.wxref_limit_swing_angle = msg.orientation.x
            self.wxref_boom_angle = msg.orientation.y
            self.wxref_arm_angle = msg.orientation.z
            self.wxref_bucket_angle = msg.orientation.w
            self.wxref_traj_initialized_ = True
        self.wxref_limit_swing_angle = msg.orientation.x

        if self.wxref_limit_swing_angle > 540:
            self.wxref_limit_ad_swing_angle= self.wxref_limit_swing_angle-360
        else:
            if self.wxref_limit_swing_angle <-180:
                self.wxref_limit_ad_swing_angle= self.wxref_limit_swing_angle+360
            else:
                self.wxref_limit_ad_swing_angle= self.wxref_limit_swing_angle
        self.wxref_swing_angle =self.wxref_limit_ad_swing_angle


        # self.wxref_swing_angle_orig= msg.orientation.x
        self.wxref_boom_angle_orig= msg.orientation.y
        self.wxref_arm_angle_orig= msg.orientation.z
        self.wxref_bucket_angle_orig= msg.orientation.w
  
        # self.wxref_swing_angle = rate_limit_filter(self.wxref_limit_ad_swing_angle, self.wxref_swing_angle, dt, self.rate_limit.swing.up, self.rate_limit.swing.down)
        self.wxref_boom_angle = rate_limit_filter(msg.orientation.y, self.wxref_boom_angle, dt, self.rate_limit.boom.up, self.rate_limit.boom.down)
        self.wxref_arm_angle = rate_limit_filter(msg.orientation.z, self.wxref_arm_angle, dt, self.rate_limit.arm.up, self.rate_limit.arm.down)
        self.wxref_bucket_angle = rate_limit_filter(msg.orientation.w, self.wxref_bucket_angle, dt, self.rate_limit.bucket.up, self.rate_limit.bucket.down)
        # print(f"回调输出wxref_swing_angle:{self.wxref_swing_angle}")
        # print(f"wxref_limit_swing_angle:{self.wxref_limit_swing_angle}")


    # def controller_PID_U0_topicCallback(self, msg: Quaternion):
    #     self.k_p = msg.x
    #     self.k_d = msg.y
    #     self.maxOutput = msg.z

    # def u_feedforwardCallback(self, msg: Quaternion):
    #     self.u_swing_feedforward_sub_ = msg.x
    #     self.u_boom_feedforward_sub_ = msg.y
    #     self.u_arm_feedforward_sub_ = msg.z
    #     self.u_bucket_feedforward_sub_ = msg.w

    # def ff_limitCallback(self, msg: Quaternion):
    #     self.ff_limit_swing_ = msg.x
    #     self.ff_limit_boom_ = msg.y
    #     self.ff_limit_arm_ = msg.z
    #     self.ff_limit_bucket_ = msg.w

    def current_vpitch_msgCallback(self, msg: Quaternion):
        self.current_vpitch_boom_ = msg.y
        self.current_vpitch_arm_ = msg.z
        self.current_vpitch_bucket_ = msg.w

    # def target_angleCallback(self, msg: Pose):
    #     self.target_angle_swing_ = msg.orientation.x
    #     self.target_angle_boom_ = msg.orientation.y
    #     self.target_angle_arm_ = msg.orientation.z
    #     self.target_angle_bucket_ = msg.orientation.w

    # def target_angle_3Callback(self, msg: Pose):
    #     self.target_angle_swing_3_ = msg.orientation.x
    #     self.target_angle_boom_3_ = msg.orientation.y
    #     self.target_angle_arm_3_ = msg.orientation.z
    #     self.target_angle_bucket_3_ = msg.orientation.w

    def boom_enu_callback(self, msg: SinglePoint):
        self.boom_enu_point = msg

    def bucket_enu_callback(self, msg: SinglePoint):
        self.bucket_enu_point = msg

    def truck_enu_callback(self, msg: SinglePoint):
        self.truck_enu_point = msg

    def truck_swive_callback(self, msg: Float32):
        self.truck_swive_data_angle = msg.data

    def imuvehicleTopicCallback(self, msg: imu_syd):
        self.vehicle_pitch = msg.pitch
        self.angular_velocity.data = msg.ang_y
    
    def arm_imu_topic_callback(self, msg: imu_syd):
        self.arm_pitch = msg.pitch
        # self.angular_velocity.data = msg.ang_y

    def normalize_angle(self, angle):
        """将角度归一化到 [0, 360) 的范围内"""
        return angle % 360.0
    def excavator_state_process_data(self):
        if self.mpc_switch_ == False:
            self.excavator_state.data = int(ExcavatorState.INITIALIZING)

        if not self.joint_angles:
            return

        delata_dig_position_working_alt_down = self.working_alt_down - self.dig_xyz.z
        bucket_attitude_angle = self.joint_angles.x + self.joint_angles.y + self.joint_angles.z
        # ================================= 栅格地图提取挖掘点附近的XYZ平均值 =========================================================
        gridmap_avg_z = 0.0
        mean_x = 0.0
        mean_y = 0.0

        if self.grid_map_ is not None and hasattr(self.grid_map_, 'layers') and ('elevation' in list(self.grid_map_.layers)):
            # ========= 对XY范围内的进行滤波提取范围内的点云 ===========
            position_x = self.bucket_point_stamped.point.x
            position_y = self.bucket_point_stamped.point.y
            x_min = position_x - 0.2
            x_max = position_x + 0.2
            y_min = position_y - 0.2
            y_max = position_y + 0.2

            # 计算平均高度
            sum_x = 0.0
            sum_y = 0.0
            sum_z = 0.0
            count = 0

            try:
                layer_index = list(self.grid_map_.layers).index('elevation')
                elevation_data = self.grid_map_.data[layer_index]
                rows = elevation_data.layout.dim[0].size
                cols = elevation_data.layout.dim[1].size
                resolution = self.grid_map_.info.resolution
                origin_x = self.grid_map_.info.pose.position.x
                origin_y = self.grid_map_.info.pose.position.y

                # grid_map_msgs 约定: 数据列优先(Fortran)序列化, dim[0]=x方向栅格数、
                # dim[1]=y方向; 且索引增大对应坐标减小。
                # 旧代码按图像式约定(idx=r*cols+c、索引增大坐标增大)换算，两个轴都反号,
                # 等效于把采样点绕地图原点旋转180度, 导致取到对称位置的高程
                elev = np.array(elevation_data.data, dtype=np.float32).reshape((rows, cols), order='F')
                ci = int(round((origin_x - position_x) / resolution)) + (rows - 1) // 2
                cj = int(round((origin_y - position_y) / resolution)) + (cols - 1) // 2

                # 取挖掘点周围±0.2m邻域的有效栅格求均值
                half_win = max(1, int(round(0.2 / resolution)))
                i0, i1 = max(0, ci - half_win), min(rows, ci + half_win + 1)
                j0, j1 = max(0, cj - half_win), min(cols, cj + half_win + 1)
                window = elev[i0:i1, j0:j1]
                valid = window[np.isfinite(window)]
                count = int(valid.size)
                if count > 0:
                    sum_z = float(valid.sum())
            except Exception:
                count = 0
                print(count)
            if count > 0:
                gridmap_avg_z = sum_z / count
                bucket_point_stamped_1 = PointStamped()
                bucket_point_stamped_1.header.frame_id = 'base'
                bucket_point_stamped_1.header.stamp = rospy.Time.now()

                bucket_point_stamped_1.point.x = self.ef_tip_rotation_z 
                bucket_point_stamped_1.point.y = self.ef_tip_rotation_z + self.base_height  #2.7
                
                # print(f"--------------地面高度base_height:{self.base_height}")  
                bucket_point_stamped_1.point.z = self.ef_tip_rotation_z - gridmap_avg_z+self.gridmap_height_err
                self.distance_bucket_point_and_ground= self.ef_tip_rotation_z - gridmap_avg_z        #增加一个固定偏差1.2m

                self.pub_bucket_point.publish(bucket_point_stamped_1)

                if self.ef_tip_rotation_z <= gridmap_avg_z:
                    self.bucket_attact_terrain_flag = True
                else:
                    self.bucket_attact_terrain_flag = False
            else:
                print('没有有效的高度数据。')
        else:
            print("Grid map 中不存在 'elevation' 图层。")
        
        # ================================= 栅格地图提取卸载点点附近的XYZ平均值 =========================================================
        gridmap_avg_z = 3.0

        cx = self.unload_local_pointdzh.point.x
        cy = self.unload_local_pointdzh.point.y
        
        cxdzh = self.unload_local_pointdzh.point.x
        cydzh = self.unload_local_pointdzh.point.y
            # self.unload_angle = normalizeAngle(std::atan2(-cy, -cx) * 180.0 / M_PI);
        self.unload_angle_ = self.normalize_angle(math.atan2(-cydzh, -cxdzh) * 180.0 / math.pi)
        # print(f"卸载点角度unload_angle_:{self.unload_angle_}")


         # ========= 以两个卸载点中心为圆心、两点间距为直径构建圆, 提取圆内的栅格高程 ===========
        unload_p1 = self.unload_local_point.point
        unload_p2 = self.unload_local_point_2.point
        # 圆心: 两个卸载.点的中心 (栅格地图坐标系下)
        position_x = (unload_p1.x + unload_p2.x) / 2.0
        position_y = (unload_p1.y + unload_p2.y) / 2.0

        # 铲斗(/RealBktPosXYZ)平面位置距卸载点中心2m范围内才提取栅格高程
        bucket_near_unload = math.hypot(self.bucket_point_stamped.point.x - position_x,
                                        self.bucket_point_stamped.point.y - position_y) <= 2.5

        if  self.unload_point_received and self.unload_point_2_received \
            and self.grid_map_ is not None and hasattr(self.grid_map_, 'layers') and ('elevation' in list(self.grid_map_.layers)):
            # and bucket_near_unload 
            
            # 半径: 两个卸载点距离的一半 (直径 = 两点间距)
            circle_radius = math.hypot(unload_p1.x - unload_p2.x, unload_p1.y - unload_p2.y) / 2.0

            # 计算平均高度
            sum_z = 0.0
            count = 0

            try:
                layer_index = list(self.grid_map_.layers).index('elevation')
                elevation_data = self.grid_map_.data[layer_index]
                rows = elevation_data.layout.dim[0].size
                cols = elevation_data.layout.dim[1].size
                resolution = self.grid_map_.info.resolution
                origin_x = self.grid_map_.info.pose.position.x
                origin_y = self.grid_map_.info.pose.position.y

                # grid_map_msgs 约定: 数据列优先(Fortran)序列化, dim[0]=x方向栅格数、
                # dim[1]=y方向; 且索引增大对应坐标减小。
                # 旧代码按图像式约定(idx=r*cols+c、索引增大坐标增大)换算，两个轴都反号,
                # 等效于把采样点绕地图原点旋转180度, 导致取到对称位置的高程
                elev = np.array(elevation_data.data, dtype=np.float32).reshape((rows, cols), order='F')
                ci = int(round((origin_x - position_x) / resolution)) + (rows - 1) // 2
                cj = int(round((origin_y - position_y) / resolution)) + (cols - 1) // 2

                # 两点重合时圆退化为点, 半径至少取一个栅格分辨率, 保证能覆盖中心栅格
                radius = max(circle_radius, 2*resolution)
                # 取圆外接正方形窗口, 再在其中筛选圆内栅格
                half_win = max(1, int(math.ceil(radius / resolution)) + 1)
                i0, i1 = max(0, ci - half_win), min(rows, ci + half_win + 1)
                j0, j1 = max(0, cj - half_win), min(cols, cj + half_win + 1)
                window = elev[i0:i1, j0:j1]

                # 窗口内各栅格的中心坐标 (索引增大对应坐标减小, 与ci/cj换算保持一致)
                x_coords = origin_x - (np.arange(i0, i1) - (rows - 1) // 2) * resolution
                y_coords = origin_y - (np.arange(j0, j1) - (cols - 1) // 2) * resolution
                gx, gy = np.meshgrid(x_coords, y_coords, indexing='ij')

                # 只保留落在圆内、且高程在卸载点下方2m以内 (unload_z - gridmap_avg_z - 2.0, unload_z - gridmap_avg_z) 的栅格求均值
                inside_circle = (gx - position_x) ** 2 + (gy - position_y) ** 2 <= radius ** 2
                z_upper = self.unload_local_point.point.z - self.unload_height_offset
                z_lower = z_upper - 2.5
                height_in_range = (window > z_lower) & (window < z_upper)
                valid = window[np.isfinite(window) & inside_circle & height_in_range]
                count = int(valid.size)
                # valid = window[np.isfinite(window) & inside_circle]
                # count = int(valid.size)
                if count > 0:
                    sum_z = float(valid.sum())
            except Exception:
                count = 0

            # print("count:", count)

            if count > 0:
                gridmap_avg_z = sum_z / count
                self.delta_local_unload_ground.header.stamp = rospy.Time.now()
                self.delta_local_unload_ground.point.x = self.unload_local_point.point.z - self.unload_height_offset
                self.delta_local_unload_ground.point.y = gridmap_avg_z
                raw_z_diff = self.unload_local_point.point.z - gridmap_avg_z - self.unload_height_offset
                self.unload_ground_z_buffer.append(raw_z_diff)
                filtered_z_diff = sum(self.unload_ground_z_buffer) / len(self.unload_ground_z_buffer)
                if(filtered_z_diff > -0.1):
                    self.delta_local_unload_ground.point.z = filtered_z_diff
                if(self.delta_local_unload_ground.point.z < self.delta_unload_distance):
                    self.unload_truck_full_flag_ = True
                else:
                    self.unload_truck_full_flag_ = False
                    
                self.delta_local_unload_ground_pub.publish(self.delta_local_unload_ground)
                self.pub_unload_truck_full_flag.publish(self.unload_truck_full_flag_)

        # 初始化阶段,挖机静止，等待各传感器、节点程序完成启机与初始化操作
        if self.INITIALIZING_FLAG == False:
            self.INITIALIZING_FLAG = True
            self.excavator_state.data = int(ExcavatorState.INITIALIZING)

        if gridmap_avg_z != 0.0:
            state = ExcavatorState(self.excavator_state.data)
            if state == ExcavatorState.INITIALIZING or state == ExcavatorState.FLATLAND:
                if (self.ef_tip_rotation_z < gridmap_avg_z - self.delta_dig_distance
                        and self.joint_angles.z < self.cutting_angle):
                    if self.dig_flag:
                        self.dig_flag = False
                        self.swivel_angle_flag = False
                        self.dig_rtk_heading_angle = self.rtk_heading
                        self.dig_vehicle_pitch = self.vehicle_pitch
                    self.excavator_state_flag.data = (self.vehicle_pitch - self.dig_vehicle_pitch >= self.delta_dig_pitch)
                    self.excavator_state.data = int(ExcavatorState.DIGGING)
            elif state == ExcavatorState.DIGGING:
                if (self.joint_angles.z < self.min_bucket_joint_angle
                        and self.ef_tip_rotation_z > gridmap_avg_z + self.delta_dig_distance):
                    self.dig_flag = True
                    self.excavator_state.data = int(ExcavatorState.UPPING)
            elif state == ExcavatorState.UPPING:
                if (self.joint_angles.x >= (self.load_traj_max_boom_ - 3.0)
                        and self.delta_bucket_angles <= -0.4):
                    self.excavator_state.data = int(ExcavatorState.UNLOADING)
            elif state == ExcavatorState.UNLOADING:
                # 复位阶段
                if (self.joint_angles.x >= self.cutting_angle
                        and self.joint_angles.x < (self.load_traj_max_boom_ - 3.0)):
                    self.excavator_state.data = int(ExcavatorState.RESETTING)
            elif state == ExcavatorState.RESETTING:
                if (self.joint_angles.z >= self.cutting_angle
                        and self.ef_tip_rotation_z < gridmap_avg_z):
                    self.excavator_state.data = int(ExcavatorState.FLATLAND)
            else:
                pass

    # 中值滤波函数
    def medianFilter(self, newData: float, window):
        # 添加新数据到窗口
        window.append(newData)
        # 保持窗口大小为3
        if len(window) > 3:
            window.popleft()
        # 如果窗口数据不足3个，返回当前数据
        if len(window) < 3:
            return newData
        # 创建临时向量进行排序
        temp = list(window)
        temp.sort()
        # 返回中值
        return temp[1]


    def SWingfeedforwardpdcontroller(self):
        

 
        # ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

                        # record_2_wxref_swing_angle_end=0
        if self.record_4_wxref_swing_angle > 360:
            self.record_4_wxref_swing_angle_end= self.record_4_wxref_swing_angle-360
        
        else:
            if self.record_4_wxref_swing_angle <0:
                self.record_4_wxref_swing_angle_end= self.record_4_wxref_swing_angle+360
            else:
                self.record_4_wxref_swing_angle_end= self.record_4_wxref_swing_angle
                
        print(f"22222222222record_4_wxref_swing_angle:{self.record_4_wxref_swing_angle}")  
        print(f"22222222222record_4_wxref_swing_angle_end:{self.record_4_wxref_swing_angle_end}")   
        print(f"22222222222record_4_current_angle_swing_end:{self.record_4_current_angle_swing_end}")
        self.calculate_arm_pilot += 1
        self.calculate_boom3_pilot += 1
        # if self.calculate_arm_pilot <1: #30
        #     self.u_swing_fused_=0
        
        # else:
            # if self.record_4_state==4 and self.seqactionmode == 2:

            #     if self.record_4_swing_clockwise_switch==self.swing_clockwise_switch:

            #         self.current_2_swing_clockwise_switch= not self.swing_clockwise_switch
            #     else:
            #         self.current_2_swing_clockwise_switch=self.swing_clockwise_switch
            # else:
            #     self.current_2_swing_clockwise_switch=self.swing_clockwise_switch
            # print(f"当前的方向:{self.current_2_swing_clockwise_switch}") 
        if self.swing_clockwise_switch == True:                      #回转方向为顺时针
            if self.record_4_wxref_swing_angle_end < 135:
                # if self.record_4_wxref_swing_angle_end< 180:
                if self.current_angle_swing_ > 180 and self.current_angle_swing_ < 360:
                    self.amendment_current_angle_swing=self.current_angle_swing_ - 360
                    # self.amendment_current_angle_swing=self.current_angle_swing_
                    # # printf("222222222222222222222222d:%f\n",self.amendment_current_angle_swing)
                    print(f"222222222222222222222222amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                else: 
                    self.amendment_current_angle_swing=self.current_angle_swing_
                    # self.amendment_current_angle_swing=self.current_angle_swing_-360
                    print(f"3333333333amendment_current_angle_swing:{self.amendment_current_angle_swing}")
            else:
                if self.record_4_wxref_swing_angle_end > 225:
                    if self.record_4_current_angle_swing_end < 180 and self.record_4_current_angle_swing_end>0:
                    # if self.record_4_current_angle_swing_end < 360 and self.record_4_current_angle_swing_end>180:
                        if self.current_angle_swing_ > 0 and self.current_angle_swing_ < 180:
                            self.amendment_current_angle_swing=self.current_angle_swing_+ 360
                            print(f"44444amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                        if self.current_angle_swing_ > 180 and self.current_angle_swing_ <=360:
                            self.amendment_current_angle_swing=self.current_angle_swing_
                            print(f"5555555amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else: 
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"6665amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                else:
                    if self.current_angle_swing_ > 270 and self.current_angle_swing_ < 360:
                        self.amendment_current_angle_swing=self.current_angle_swing_-360
                        print(f"7777777amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else: 
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        # print(f"888888amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                        # print("True: shunshizhen xiangwei 0")

        else: 
            if self.record_4_wxref_swing_angle_end < 135:
                # self.record_2_current_angle_swing__end=

                if self.record_4_current_angle_swing_end > 180 and self.record_4_current_angle_swing_end < 360:
                    
                    if self.current_angle_swing_ > 180 and self.current_angle_swing_ < 360:
                            self.amendment_current_angle_swing=self.current_angle_swing_-360
                            print(f"-------------00000:{self.amendment_current_angle_swing}")
                            # print(f"555555:{self.record_2_current_angle_swing_end}")
                            # print(f"66666:{self.current_angle_swing_}"                   
                    if self.current_angle_swing_ >=0 and self.current_angle_swing_ < 180:
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"-----------11111111111111amendment_current_angle_swing:{self.amendment_current_angle_swing}")

                else:

                    self.amendment_current_angle_swing=self.current_angle_swing_
                    print(f"----------2222222222amendment_current_angle_swing:{self.amendment_current_angle_swing}")
            else:
                if self.record_4_wxref_swing_angle_end > 225:

                    if self.current_angle_swing_ > 0 and self.current_angle_swing_ < 180:
                        self.amendment_current_angle_swing=self.current_angle_swing_ + 360
                        print(f"----333333333333amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else: 
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"----444444444444444amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                else:
                    if self.current_angle_swing_ > 0 and self.current_angle_swing_ < 90:
                        self.amendment_current_angle_swing=self.current_angle_swing_+360
                        print(f"----55555555555amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else: 
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"----666666666666666amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                        # print("false: nishizhen xiangwei 0")


    
        

        self.randomnumberoutput = randomnumber()
        # printf("randomnumberoutput:%f\n",randomnumberoutput);
        self.ref_swing_angle = self.wxref_swing_angle + self.randomnumberoutput
        # wxref_swing_angle_stage2 替换 wxref_swing_angle

        # 新增PID部分
        # printf("seqactionmode:%d\n",seqactionmode);

        real_swing_angle_velocity_input = [-40, -40, -32, -18, -9, -2, 0, 2, 9, 18, 32, 40, 40]  # -左 +右dps = °/s 范围  -327dps 至 +327dps
        real_swing_angle_velocity_output = [10, 10, 10, 8, 7, 5, 0, 5, 7, 8, 10, 10, 10]  # jisuancuowu 
        self.ref_swing_angle_temp = abs(interpolate(self.current_velovity_swing_, real_swing_angle_velocity_input, real_swing_angle_velocity_output))
        # print(f"00000000000000000ref_swing_angle_temp_swing:{self.ref_swing_angle_temp}")
        # std::cout<<"interpolate(current_velovity_swing_, real_swing_angle_velocity_input, real_swing_angle_velocity_output):"<<interpolate(current_velovity_swing_, real_swing_angle_velocity_input, real_swing_angle_velocity_output)<<std::endl;

        # gentle_swing_input = [0, 1, 3, 5, 25, 30, 45, 90, 180, 360]
        # gentle_swing_output = [0, 0, 150, 400, 600, 800, 850, 1000, 1000, 1000]
        gentle_swing_input = [0, 1, 3, 5, 25, 30, 45, 90, 180, 360]
        gentle_swing_output = [0, 0, 150, 400, 600, 800, 850, 1000, 1000, 1000]

        gentle_swing_input_slow = [0, 1, 3, 5, 25, 30, 45, 90, 180, 360]
        gentle_swing_output_slow = [0, 0, 100, 400, 500, 500, 600, 600, 600, 600]

        # printf("ref_swing_angle_temp:%f\n",ref_swing_angle_temp);

        self.dif_swing_angleadd = self.ref_swing_angle - self.amendment_current_angle_swing
        print(f"00000000000000000dif_swing_angleadd:{self.dif_swing_angleadd}")
        print(f"00000000000000000ref_swing_angle:{self.ref_swing_angle}")
        print(f"00000000000000000amendment_current_angle_swing:{self.amendment_current_angle_swing}")
        

        self.dif_swing_angle = abs(self.ref_swing_angle - self.amendment_current_angle_swing)
        # cankaojiaodu xianfu
        if self.dif_swing_angle < 20:
            self.dif_swing_angle_end = abs(self.dif_swing_angle - self.ref_swing_angle_temp)
            

            # std::cout<<"dif_swing_angle_end"<<dif_swing_angle_end<<std::endl;
            # if (dif_swing_angle_end<0) 
            # {      
            #     dif_swing_angle_end=0;
            # }
        else:
            self.dif_swing_angle_end = self.dif_swing_angle
        print(f"00000000000000000dif_swing_angle_end:{self.dif_swing_angle_end}")

        # 对异常的差值划归到[-180,+180)之间，正常的差值不作更改
        # print("Before  self.dif_swing_angle_end_: {}".format(self.dif_swing_angle_end))
        self.dif_swing_angle_end = abs(angle_diff2normal_area(self.dif_swing_angle_end))
        # print("self.dif_swing_angle_end: {}".format(self.dif_swing_angle_end))

        swing_pilot_fused_data_temp_high = interpolate(self.dif_swing_angle_end, gentle_swing_input, gentle_swing_output)
        swing_pilot_fused_data_temp_slow = interpolate(self.dif_swing_angle_end, gentle_swing_input_slow, gentle_swing_output_slow)
        # print(f"0000000000000000swing_pilot_fused_data_temp_high:{swing_pilot_fused_data_temp_high}")
        # print(f"0000000000000000swing_pilot_fused_data_temp_slow:{swing_pilot_fused_data_temp_slow}")

        # 放200处  calculate_swing_speed_fast_slow_= 0.0; 
        self.calculate_swing_speed_fast_slow_ += 1
        if self.calculate_swing_speed_fast_slow_ < 2:
            self.record_current_angle_swing_ = self.amendment_current_angle_swing

        if self.ref_swing_angle < self.amendment_current_angle_swing:
        # if self.swing_clockwise_switch == True:
            self.High_Slow_Switch_Values = self.record_current_angle_swing_ - (self.record_current_angle_swing_ - self.ref_swing_angle) * 0.65

            if self.amendment_current_angle_swing > self.High_Slow_Switch_Values:
                self.swing_pilot_fused_data_temp = swing_pilot_fused_data_temp_high  # 用快
                self.swing_pilot_fused_data_temp2 = 0 - self.swing_pilot_fused_data_temp
                # print(f"fast---------swing_pilot_fused_data_temp2:{self.swing_pilot_fused_data_temp2}")
                # if self.swing_pilot_fused_data_temp2>0:
                #     self.swing_pilot_fused_data_temp2=0
                # else:
                #     self.swing_pilot_fused_data_temp2 =  0 - self.swing_pilot_fused_data_temp  # 用快


            else:
                self.swing_pilot_fused_data_temp = swing_pilot_fused_data_temp_slow  # 慢
                self.swing_pilot_fused_data_temp2 =  0 - self.swing_pilot_fused_data_temp
                # print(f"slow---------swing_pilot_fused_data_temp2:{self.swing_pilot_fused_data_temp2}")
        else:
            self.High_Slow_Switch_Values = self.record_current_angle_swing_ + (self.ref_swing_angle - self.record_current_angle_swing_) * 0.65
            if self.amendment_current_angle_swing< self.High_Slow_Switch_Values:
                self.swing_pilot_fused_data_temp = swing_pilot_fused_data_temp_high  # 用快
                self.swing_pilot_fused_data_temp2 = self.swing_pilot_fused_data_temp
                # print(f"fast---------swing_pilot_fused_data_temp2:{self.swing_pilot_fused_data_temp2}")
                # if self.swing_pilot_fused_data_temp2<0:
                #     self.swing_pilot_fused_data_temp2=0
                # else:
                #     self.swing_pilot_fused_data_temp2 = self.swing_pilot_fused_data_temp  # 用快
            else:
                self.swing_pilot_fused_data_temp = swing_pilot_fused_data_temp_slow  # 慢
                self.swing_pilot_fused_data_temp2 = self.swing_pilot_fused_data_temp
                # print(f"slow---------swing_pilot_fused_data_temp2:{self.swing_pilot_fused_data_temp2}")
    

        self.swing_integral_control = self.update_integral_swing_control(-250.0, 250, 15.0, self.dif_swing_angleadd, 6.0)
        self.u_swing_fused_ = self.swing_pilot_fused_data_temp2+ self.swing_integral_control
        # self.u_swing_fused_ = self.swing_pilot_fused_data_temp2
        print(f"u_swing_fused_:{self.u_swing_fused_}")
        

        #回转积分增加

    def add_2_arm_state_contorll_function(self):
        dif_arm_anglepid2 = self.wxref_arm_angle - self.current_angle_arm_
        gentle_arm2_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm2_output = [200, 400, 1100, 1300, 1600, 1800, 2000]
        gentle_arm2_start_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm2_start_output = [200, 400, 1600, 1700, 1800, 1800, 2000]

        arm_pilot_fused_data_temp2 = interpolate(dif_arm_anglepid2, gentle_arm2_input, gentle_arm2_output)
        arm_pilot_fused_data_start2 = interpolate(dif_arm_anglepid2, gentle_arm2_start_input, gentle_arm2_start_output)
        # u_arm_fused_ = arm_pilot_fused_data_temp2 -1000.0;
        self.arm_pilot_fused_data_temp = arm_pilot_fused_data_temp2 - 1000.0
        self.u_arm_fused_ =self.arm_pilot_fused_data_temp
        if self.calculate_arm_pilot < 30:
            self.u_arm_fused_ = 0
        elif self.calculate_arm_pilot > 30 and self.calculate_arm_pilot < 300:
            self.u_arm_fused_ = arm_pilot_fused_data_start2 -1000
            if self.u_arm_fused_ < 600:
                self.u_arm_fused_ = 600
        else:
            self.u_arm_fused_ =self.arm_pilot_fused_data_temp
        # print(f"------arm_pilot_fused_data_temp:{self.arm_pilot_fused_data_temp}")

    def add_2_boom_state_contorll_function(self):
        dif_boom_anglepid2 = self.wxref_boom_angle - self.current_angle_boom_
        # gentle_boom2_input = [-15, -10, -5, 0, 5, 10, 15]
        # gentle_boom2_output = [600, 600, 600, 1000, 1400, 1600, 2000]
        #主要是动臂下降

        gentle_boom2_input = [-25, -25, -15,-5, 0, 5, 10, 15]
        gentle_boom2_output = [300, 300,500 ,700, 1000, 1400, 1600, 1800]

        boom_pilot_fused_data_temp2 = interpolate(dif_boom_anglepid2, gentle_boom2_input, gentle_boom2_output)

        # u_boom_fused_ = boom_pilot_fused_data_temp2 -1000.0;
        self.u_boom_fused_ = boom_pilot_fused_data_temp2 - 1000.0
        swing_dif_during2state = abs(self.ref_swing_angle - self.amendment_current_angle_swing)  
        if swing_dif_during2state < 20 and swing_dif_during2state > 0:
            self.u_boom_fused_ = -800
# 铲斗控制
        self.dif_bucket_angle = self.wxref_bucket_angle - self.current_angle_bucket_
        

        # real_arm_angle_radian=real_arm_angle*(M_PI/180);
        #  ref_bucket_cutting_angle= calculateBucketReference(real_arm_angle_radian)*180/M_PI;



        self.dif_bucket_anglepid2 = self.ref_bucket_cutting_angle - self.current_angle_bucket_
        # print(f" 铲斗差值  self.dif_bucket_anglepid2:{self.dif_bucket_anglepid2}")

        gentle_bucket2_input = [-20, -15,    -10, -5, -3,     0,   3, 5,     10,   15,   20]
        gentle_bucket2_output = [0,    0,   300,  500, 800, 1000,1200,1600, 1700, 2000 , 2000]
        self.bucket_pilot_fused_data_temp2 = interpolate(self.dif_bucket_anglepid2, gentle_bucket2_input, gentle_bucket2_output)
        self.bucket_pilot_fused_data_temp_end = self.bucket_pilot_fused_data_temp2 - 1000.0
        # print(f"铲斗 interpolate ----self.bucket_pilot_fused_data_temp2:{self.bucket_pilot_fused_data_temp2}")
        # self.bucket_pilot_fused_data_temp_end = self.bucket_pilot_fused_data_temp2 - 1000.0
        # print(f"铲斗 计数 ----self.calculate_arm_pilot:{self.calculate_arm_pilot}")
        # if self.calculate_arm_pilot < 10:
        #     self.bucket_pilot_fused_data_temp_end = 0
        #     # print(f"铲斗 ----self.bucket_pilot_fused_data_temp_end:{self.bucket_pilot_fused_data_temp_end}")
        # else:
        #     self.bucket_pilot_fused_data_temp_end = self.bucket_pilot_fused_data_temp2 - 1000.0
        #     # print(f"铲斗 22222----self.bucket_pilot_fused_data_temp_end:{self.bucket_pilot_fused_data_temp_end}")
        # # self.u_bucket_fused_ = self.bucket_pilot_fused_data_temp_end
        #6转2限制铲斗在矿卡上方的输出
        dif_swing_angle_phase6to2_add=self.current_angle_swing_-self.unload_angle_
        print(f"6转2限制铲斗在矿卡上方的输出dif_swing_angle_phase6to2_add:{dif_swing_angle_phase6to2_add}")

        if dif_swing_angle_phase6to2_add > 360:
            self.dif_swing_angle_phase6to2=dif_swing_angle_phase6to2_add-360
        
        else:
            if dif_swing_angle_phase6to2_add <0:
                self.dif_swing_angle_phase6to2= dif_swing_angle_phase6to2_add+360
            else:
                self.dif_swing_angle_phase6to2= dif_swing_angle_phase6to2_add

        print(f"6转2限制铲斗在矿卡上方的输出dif_swing_angle_phase6to2:{self.dif_swing_angle_phase6to2}")
        print(f"6转2限制条件phase6to2_start_active_:{self.phase6to2_start_active}")

        if self.phase6to2_start_active==True and self.dif_swing_angle_phase6to2<20:
            self.u_bucket_fused_ = 0
            print(f"6转2限制铲斗在矿卡上方的输出self.u_bucket_fused_:{self.u_bucket_fused_}")
        else:
            self.u_bucket_fused_ = self.bucket_pilot_fused_data_temp_end
            print(f"6转2正常输出 self.u_bucket_fused_:{self.u_bucket_fused_}")

           
        # if self.dif_swing_angle_phase6to2 < :
        #     if self.u_boom_fused_ > -200:
        #             self.u_boom_fused_ = -200
        #     if self.u_boom_fused_ < -1000:
        #             self.u_boom_fused_ = -1000



        # if self.dif_swing_angle < 25:
        #     if self.u_boom_fused_ > -200:
        #             self.u_boom_fused_ = -200
        #     if self.u_boom_fused_ < -1000:
        #             self.u_boom_fused_ = -1000
            # print("swing_dif_during2state:", swing_dif_during2state)

            # print("￥￥￥￥￥回转接近目标:", self.u_boom_fused_)

    def add_2_2stage_switch_3stage_arm_down_function(self):
        if (self.distance_bucket_point_and_ground<0.6)and(self.dif_swing_angle<10):
            self.u_arm_fused_ = -1000
            # print(f"***************u_arm_fused_:{self.u_arm_fused_}")
            

       



        # print(f"u_boom_fused_:{self.u_boom_fused_}")
        # print(f"boom_pilot_fused_data_temp2:{boom_pilot_fused_data_temp2}")
        # print(f"dif_boom_anglepid2:{dif_boom_anglepid2}")
        # print(f"wxref_boom_angle:{self.wxref_boom_angle}")
        # print(f"current_angle_boom_:{self.current_angle_boom_}")

    def update_integral_control(self, min_integral: float, max_integral: float, integral_reset_threshold: float, dif_anglepid: float, k_i_bucket: float):
        # 初始化积分误差项
        # integral_bucket_error = 0
        
        # 设置积分项的上下限
        # 当误差较大时重置积分项（防止积分饱和）
        if abs(dif_anglepid) > integral_reset_threshold:
            self.integral_bucket_error = 0  # 重置积分项
        else:
            # 累加积分项，并进行限幅
            self.integral_bucket_error += dif_anglepid
        
            # print(f"integral_bucket_error:{self.integral_bucket_error}")
        
        # 计算积分控制项wewqeqqqqqss
        integral_control = k_i_bucket * self.integral_bucket_error
        integral_control_limt = max(min_integral, min(max_integral, integral_control))
        return integral_control_limt

    def update_integral_swing_control(self, min_integral: float, max_integral: float, integral_reset_threshold: float, dif_anglepid: float, k_i_bucket: float):
        # 初始化积分误差项
        # integral_bucket_error = 0
        
        # 设置积分项的上下限
        # 当误差较大时重置积分项（防止积分饱和）
        if abs(dif_anglepid) > integral_reset_threshold:
            self.integral_swing_error = 0  # 重置积分项
            integral_control_limt= 0 
        else:
            # 累加积分项，并进行限幅
            self.integral_swing_error += dif_anglepid
            
            
        
        # 计算积分控制项wewqeqqqqqss
        integral_control = k_i_bucket * self.integral_swing_error
        integral_control_limt = max(min_integral, min(max_integral, integral_control))
        return  integral_control_limt
        # print(f"integral_control:{integral_control}")

        


    # def add_4_boom_state_contorll_function(self):
    #     # 第一种变化率的方式控制数据的变化
    #     new_u_boom_fused = self.u_boom_fused_
    #     # 计算新的变化量
    #     delta_u_boom = new_u_boom_fused - self.previous_u_boom_fused_

    #     # 限制变化量
    #     if delta_u_boom > self.max_delta_u_boom_:
    #         new_u_boom_fused = self.previous_u_boom_fused_ + self.max_delta_u_boom_
    #     # if (delta_u_boom < -max_delta_u_boom_)
    #     # {
    #     #     new_u_boom_fused = previous_u_boom_fused_ - max_delta_u_boom_;
    #     # }
    #     # 更新 u_boom_fused_
    #     self.u_boom_fused_ = new_u_boom_fused
    #     # 更新前一个控制变量的值
    #     self.previous_u_boom_fused_ = self.u_boom_fused_

    #     # 第二种积分的方式控制数据的变化
    #     # double error = u_boom_fused_ - u_boom_smooth_;
    #     # k = (std::abs(error) < 50 ? 0.5 : 1.0);
    #     # u_boom_smooth_ += k * error * dt;
    #     # u_boom_fused_ = u_boom_smooth_;

    def add_4_arm_state_contorll_function(self):
        # 第一种变化率的方式控制数据的变化
        new_u_arm_fused = self.u_arm_fused_
        # 计算新的变化量
        delta_u_arm = new_u_arm_fused - self.previous_u_arm_fused_

        # 限制变化量
        if delta_u_arm > self.max_delta_u_arm_:
            new_u_arm_fused = self.previous_u_arm_fused_ + self.max_delta_u_arm_
        elif delta_u_arm < -self.max_delta_u_arm_:
            new_u_arm_fused = self.previous_u_arm_fused_ - self.max_delta_u_arm_
        # 更新 u_arm_fused_
        self.u_arm_fused_ = new_u_arm_fused
        # 更新前一个控制变量的值
        self.previous_u_arm_fused_ = self.u_arm_fused_

        # 第二种变化率方式控制数据变化
        # double error = u_arm_fused_ - u_arm_smooth_;
        # k = (std::abs(error) < 50 ? 0.5 : 1.0);
        # u_arm_smooth_ += k * error * dt;
        # u_arm_fused_ = u_arm_smooth_;

    # def add_4_bucket_state_contorll_function(self):
    #     # 第一种变化率的方式控制数据的变化
    #     new_u_bucket_fused = self.u_bucket_fused_
    #     # 计算新的变化量
    #     delta_u_bucket = new_u_bucket_fused - self.previous_u_bucket_fused_

    #     # 限制变化量
    #     if delta_u_bucket > self.max_delta_u_bucket_:
    #         new_u_bucket_fused = self.previous_u_bucket_fused_ + self.max_delta_u_bucket_
    #     elif delta_u_bucket < -self.max_delta_u_bucket_:
    #         new_u_bucket_fused = self.previous_u_bucket_fused_ - self.max_delta_u_bucket_
    #     # 更新 u_bucket_fused_
    #     self.u_bucket_fused_ = new_u_bucket_fused
    #     # 更新前一个控制变量的值
    #     self.previous_u_bucket_fused_ = self.u_bucket_fused_

    #     # 第二种积分的方式控制数据的变化
    #     # double error = u_bucket_fused_ - u_bucket_smooth_;
    #     # k = (std::abs(error) < 50 ? 0.5 : 1.0);
    #     # u_bucket_smooth_ += k * error * dt;
    #     # u_bucket_fused_ = u_bucket_smooth_;

    def dzh_add_2_state_contorll_function(self):
        #  u_boom_fused_ = fusion_controller_joysticky;
        return

    def dzh_add_3_boom_state_contorll_function(self):
        self.calculate_boom3_pilot += 1


        if self.calculate_boom3_pilot < 100:
            self.wxref_boom3_angle = self.current_angle_boom_
        else:
            self.wxref_boom3_angle = self.wxref_boom_angle
        # print(f"------arm_pilot_fused_data_temp:{self.arm_pilot_fused_data_temp}")
        self.dif_boom_anglepid3 = self.wxref_boom3_angle - self.current_angle_boom_
        # print(f"boom 差值   dif_boom_anglepid3:{self.dif_boom_anglepid3}")

        print(f"boom 差值   model_config:{self.model_config}")
        if  self.model_config==0:
            print(f"0000000000   model_config:{self.model_config}")
        # 默认控制自调
            # gentle_boom3_input = [-20, -15, -10,  -5,    0,    3,    5,    15,  20 ,  25]
            # gentle_boom3_output = [200, 200, 600, 600, 1100,1500, 1700, 1800, 2000 , 2000]
            gentle_boom3_input = [-15, -10, -5, 0,5,10, 15, 20]
            gentle_boom3_output = [200, 600, 700, 1000,1400, 1500, 1800, 2000]
        elif  self.model_config==1:
        # 尹博士
            gentle_boom3_input = [-15, -10, -5, 0,5,10, 15, 20]
            gentle_boom3_output = [200, 600, 700, 1000,1300, 1500, 1800, 2000]
            print(f"11111111111  model_config:{self.model_config}")
        # # 吴青琳
        elif  self.model_config==2:
            gentle_boom3_input = [-15, -10, -5, 0, 10, 15, 20]
            gentle_boom3_output = [200, 600, 700, 950, 1500, 1800, 2000]
            print(f"222222222222  model_config:{self.model_config}")

        # 王宵
        elif  self.model_config==3:
            gentle_boom3_input = [-20, -15, -10,  -5,    0,    3,    5,    15,  20 ,  25]
            gentle_boom3_output = [200, 200, 600, 600, 1100,1250, 1500, 1800, 2000 , 2000]
            print(f"33333333333333  model_config:{self.model_config}")
        # # 郑万里
        # gentle_boom3_input = [-15, -10, -5, 0, 10, 15, 20]
        # gentle_boom3_output = [200, 600, 700, 800, 1500, 1800, 2000]

        # # 吴青琳
        # gentle_boom3_input = [-15, -10, -5, 0, 10, 15, 20]
        # gentle_boom3_output = [200, 600, 700, 950, 1500, 1800, 2000]


        # 王宵
        # gentle_boom3_input = [-20, -15, -10,  -5,    0,    3,    5,    15,  20 ,  25]
        # gentle_boom3_output = [200, 200, 600, 600, 1100,1250, 1500, 1800, 2000 , 2000]

        # 尹博士
        # gentle_boom3_input = [-15, -10, -5, 0,5,10, 15, 20]
        # gentle_boom3_output = [200, 600, 700, 1000,1300, 1500, 1800, 2000]

        ################################0625实机测试版本
        # gentle_boom3_input = [-5, -2, -1, 0,1,2, 3, 5]
        # gentle_boom3_output = [200, 600, 700, 1000,1300, 1500, 1800, 2000]
        ################################

        ################################0628 模型直接控制测试版本
        # gentle_boom3_input = [-0.1197, -0.0595, 0, 0.0595, 0.1197]
        # gentle_boom3_output = [ 0, 200, 1000, 1800, 2000]
        ################################

        boom_pilot_fused_data_temp3 = interpolate(self.dif_boom_anglepid3, gentle_boom3_input, gentle_boom3_output)
        boom_integral_control = self.update_integral_control(0.0, 200.0, 5.0, self.dif_boom_anglepid3, 2.0)
        # print(f"boom 积分 boom_integral_control:{boom_integral_control}")
        self.u_boom_fused_ = boom_pilot_fused_data_temp3-1000.0+boom_integral_control
        # print(f"boom 输出 u_boom_fused_:{self.u_boom_fused_}")

    def dzh_add_3_arm_state_contorll_function(self):
        self.dif_arm_anglepid3 = self.wxref_arm_angle - self.current_angle_arm_


        ################################0625实机测试版本
        gentle_arm3_input = [-20, -15, -10, -5,-1, 0, 1,5, 10, 15]
        gentle_arm3_output = [0.0, 0.0, 300.0, 400.0,600, 900,1300, 1300, 1400, 1400]

        # gentle_arm3_input = [-20, -15, -10, -5, 0, 1,5, 10, 15]
        # gentle_arm3_output = [0.0, 0.0, 300.0, 400.0, 500,1300, 1300, 1400, 1400]
        ################################

        ################################0628 模型直接控制测试版本
        # gentle_arm3_input = [-0.3099, -0.1203, 0, 0.1203 , 0.3099]
        # gentle_arm3_output = [ 0, 200, 1000, 1800, 2000]
        ################################

        arm_pilot_fused_data_temp3 = interpolate(self.dif_arm_anglepid3, gentle_arm3_input, gentle_arm3_output) 
        self.u_arm_fused_ = arm_pilot_fused_data_temp3- 1000.0
        # self.u_arm_fused_ = arm_pilot_fused_data_temp3-1000

    def dzh_add_3_state_contorll_function(self):
        self.dif_bucket_anglepid3 = self.wxref_bucket_angle - self.current_angle_bucket_


        # ###############################0625实机测试版本
        gentle_bucket_input = [-20, -15, -10, -5, 0, 5, 10, 15]
        gentle_bucket_output = [0, 0, 0, 200, 400, 1300, 1380, 1800]
        # ###############################

        ################################0628 模型直接控制测试版本
        # gentle_bucket_input = [-3.9637, -2.6225, 0, 2.6225, 3.9637]
        # gentle_bucket_output = [ 0, 200, 1000, 1800, 2000]
        ################################

        self.bucket_pilot_fused_data_temp3 = interpolate(self.dif_bucket_anglepid3, gentle_bucket_input, gentle_bucket_output)
        # 积分环节
        bucket_integral_control = self.update_integral_control(-400.0, 0.0, 50.0, self.dif_bucket_anglepid3, 2.0)

        # 合并比例控制和积分控制
        combined_output = self.bucket_pilot_fused_data_temp3 - 1000.0 + bucket_integral_control

        self.u_bucket_fused_ = combined_output


       

    def dzh_add_4_state_contorll_function(self):
        # std::cout<<"dzh_add_4_state_contorll_function"<<std::endl;
        # if(current_angle_boom_ < 40)
        # {
        #
        #     u_boom_fused_ = fusion_controller_joysticky + 1000;
        # }
        # eles
        # {
        #
        #     u_boom_fused_ = fusion_controller_joysticky ;
        # }

        # ///boom pid
        # print(f"---------------------铲斗轨迹 wxref_bucket_angle:{self.wxref_bucket_angle}")
        self.dif_boom_anglepid4 = self.wxref_boom_angle - self.current_angle_boom_
        # print(f"---------------------wxref_boom_angle:{self.wxref_boom_angle}")
        # print(f"---------------------动臂current_angle_boom_:{self.current_angle_boom_}")
        # print(f"---------------------dif_boom_anglepid4:{self.dif_boom_anglepid4}")
      

        gentle_boom4_input = [-20, -15, -5,-3,      0, 10, 15, 20]
        gentle_boom4_output = [400, 500, 600, 800,  1500, 1900, 2000, 2000]
        self.boom_pilot_fused_data_temp4 = interpolate(self.dif_boom_anglepid4, gentle_boom4_input, gentle_boom4_output)
        self.u_boom_fused_ = self.boom_pilot_fused_data_temp4 - 1000.0

        # /////////////////////////////////
        self.unload_bucket_reach_target_angle_flag_ = True if self.current_angle_bucket_ > self.unload_bucket_angle_limit_ else False

        if self.wxref_bucket_angle > 25:
            self.wxref_bucket_angle = 25.0
        if self.wxref_bucket_angle < -120:
            self.wxref_bucket_angle = -120.0
        self.dif_bucket_anglepid4 = self.wxref_bucket_angle - self.current_angle_bucket_

        # print(f"c铲斗差值   ---------dif_bucket_anglepid4:{self.dif_bucket_anglepid4}")
        # print(f"c铲斗dangqian   ---------current_angle_bucket_:{self.current_angle_bucket_}")
        # print(f"c铲斗dangqian   ---------wxref_bucket_angle:{self.wxref_bucket_angle}")
        # 快
        # gentle_bucket4_input = [-15, -10, -5, 0, 5,10, 15, 30,40,90]
        # gentle_bucket4_output = [100, 100, 600, 800, 1100,1500, 1800, 1800, 2000, 2000]

        # gentle_bucket4_input = [-90, -40, -30, -15, -10, -5, 0, 5,10, 15, 70, 90,180]
        # gentle_bucket4_output = [0, 0, 200, 400, 400, 400,   800, 1100,1500, 1800, 1800, 1500, 1500]

        if abs(self.dif_arm_anglepid4 )> 20:
            gentle_bucket4_input = [-90, -40, -30, -15, -10, -5, 0, 5,10, 15, 30, 40,90]
            gentle_bucket4_output = [0, 0, 200, 400, 400, 400,   800, 1100,1500, 1500, 1500, 1500, 1500]
            #    print(f"1111111111111u_bucket_fused_:{self.u_bucket_fused_}")
        else:
            gentle_bucket4_input = [-90, -40, -30, -15, -10, -5, 0, 5,10, 15, 30, 70,90]
            gentle_bucket4_output = [0, 0, 200, 400, 400, 400,   800, 1000,1600, 1800, 1900, 2000, 2000]

        # gentle_bucket4_unload_input = [-15, -10, -5, 0, 5,10, 15, 30,40,90]
        # gentle_bucket4_unload_output = [100, 100, 600, 0, 1200,1600, 1800, 1800, 2000, 2000]

        # gentle_bucket4_input = [-15, -10, -5, 0, 5,10, 15, 20]
        # gentle_bucket4_output = [500, 500, 600, 800, 1100,1500, 1500, 1500]

        self.bucket_pilot_fused_data_temp4 = interpolate(self.dif_bucket_anglepid4, gentle_bucket4_input, gentle_bucket4_output)
        self.u_bucket_fused_ = self.bucket_pilot_fused_data_temp4 - 1000.0

        self.calculate_bucket_pilot += 1
        if self.calculate_bucket_pilot < 250 and self.phase5to4_start_active != True:
            if self.u_bucket_fused_ > 0:
               self.u_bucket_fused_ = -200
            #    print(f"1111111111111u_bucket_fused_:{self.u_bucket_fused_}")
            else:
               self.u_bucket_fused_ = self.bucket_pilot_fused_data_temp4 - 1000.0
            #    print(f"222222222222u_bucket_fused_:{self.u_bucket_fused_}")
        else:
            self.u_bucket_fused_ = self.bucket_pilot_fused_data_temp4 - 1000.0
        print(f"3333333333333u_bucket_fused_:{self.u_bucket_fused_}")



    def dzh_add_5_state_contorll_function(self):
        # if(current_angle_boom_ < 40)
        # {
        #
        #     u_boom_fused_ = fusion_controller_joysticky + 1000;
        # }
        # eles
        # {
        #
        #     u_boom_fused_ = fusion_controller_joysticky ;
        # }

        # ///boom pid
        # print(f"---------------------铲斗轨迹 wxref_bucket_angle:{self.wxref_bucket_angle}")
        dif_boom_anglepid5 = self.wxref_boom_angle - self.current_angle_boom_
        print(f"---------------------wxref_boom_angle:{self.wxref_boom_angle}")
        print(f"---------------------动臂current_angle_boom_:{self.current_angle_boom_}")
        print(f"---------------------dif_boom_anglepid4:{dif_boom_anglepid5}")
      

        gentle_boom5_input = [-20, -15, -5, 0, 2, 15, 20]
        gentle_boom5_output = [0, 0, 800, 1600, 1900, 2000, 2000]
        boom_pilot_fused_data_temp5 = interpolate(dif_boom_anglepid5, gentle_boom5_input, gentle_boom5_output)
        self.u_boom_fused_ = boom_pilot_fused_data_temp5 - 1000.0

        # /////////////////////////////////
        self.unload_bucket_reach_target_angle_flag_ = True if self.current_angle_bucket_ > self.unload_bucket_angle_limit_ else False

        if self.wxref_bucket_angle > 30:
            self.wxref_bucket_angle = 27.0
        dif_bucket_anglepid5 = self.wxref_bucket_angle - self.current_angle_bucket_
        # 快

        gentle_bucket5_input = [-15, -10, -5, 0, 5,10, 15, 30,40,90]
        gentle_bucket5_output = [100, 100, 600, 800, 1100,1500, 1800, 1800, 2000, 2000]

        # gentle_bucket4_input = [-15, -10, -5, 0, 5,10, 15, 20]
        # gentle_bucket4_output = [500, 500, 600, 800, 1100,1500, 1500, 1500]

        bucket_pilot_fused_data_temp5 = interpolate(dif_bucket_anglepid5, gentle_bucket5_input, gentle_bucket5_output)
        self.u_bucket_fused_ = bucket_pilot_fused_data_temp5 - 1000.0

        # self.calculate_bucket_5_pilot += 1
        # if self.calculate_bucket_pilot < 100:
        #     if self.u_bucket_fused_ > 0:
        #        self.u_bucket_fused_ = 0
        #        print(f"1111111111111u_bucket_fused_:{self.u_bucket_fused_}")
        #     else:
        #        self.u_bucket_fused_ = bucket_pilot_fused_data_temp4 - 1000.0
        #        print(f"222222222222u_bucket_fused_:{self.u_bucket_fused_}")
        # else:
        #     self.u_bucket_fused_ = bucket_pilot_fused_data_temp4 - 1000.0
        #     print(f"3333333333333u_bucket_fused_:{self.u_bucket_fused_}")

    def dzh_add_5_arm_state_contorll_function(self):

        # self.calculate_arm4_pilot += 1
      
        # if self.calculate_arm4_pilot < 50:
        #     # now = rospy.Time.now()
        #     # dt = (now - self.last_sys_sco_traj_time_).to_sec() if self.last_sys_sco_traj_time_ is not None else 0.0
        #     # self.last_sys_sco_traj_time_ = now
        #     dt = 0.02
        #     if self.wxref_arm_angle_after_filter is None:
        #         self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        #     self.wxref_arm_angle_after_filter = rate_limit_filter(self.wxref_arm_angle, self.wxref_arm_angle_after_filter, dt, max_rate_up=3)
        # else:
        #     self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        print(f"---------------------5阶段斗杆轨迹 wxref_arm_angle_after_filter:{self.wxref_arm_angle_after_filter}")
        dif_arm_anglepid5 = self.wxref_arm_angle_after_filter - self.current_angle_arm_
        # print(f"---------------------5阶段斗杆差值 dif_arm_anglepid5:{dif_arm_anglepid5}")
        # print(f"---------------------5阶段斗杆 current_angle_arm_:{self.current_angle_arm_}")

        # print(f"4444444444444444444斗杆差值dif_arm_anglepid4:{self.dif_arm_anglepid4}")
        # print(f"wxref_arm_angle_after_filter:{self.wxref_arm_angle_after_filter}")
        # print(f"current_angle_arm_:{self.current_angle_arm_}")
        # print(f"current_angle_bucket_:{self.current_angle_bucket_}")
        # print(f"current_angle_swing_:{self.current_angle_swing_}")

        

        # self.dif_arm_anglepid4 = self.wxref_arm_angle - self.current_angle_arm_ 
        # 原有提升卸载阶段控制插值，现用作卸载阶段插值
        gentle_arm4_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm4_output = [800, 800, 800, 1220, 1600, 1800, 1800]
        # arm_pilot_fused_data_temp4 = interpolate(self.dif_arm_anglepid4, gentle_arm4_input, gentle_arm4_output)
        # 提升阶段斗杆插值
        gentle_arm4_up_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm4_up_output = [400, 400, 800, 1000, 1100, 1100, 1100]


        # if (time.time() - self.phase4_start_time <= 3.0 and
        #     time.time() - self.phase4_start_time > 0 ):
        #     # 用的是[400, 400, 800, 1000, 1100, 1200, 1300]插值

            
        #     arm_pilot_fused_data_temp4 = interpolate(dif_arm_anglepid4, gentle_arm4_up_input, gentle_arm4_up_output)
        #     # print(f"-------------3s 成立 arm_pilot_fused_data_temp4:{arm_pilot_fused_data_temp4}")
        #     # print("-----提升------")
        # else:
        #     # 用的是 [400, 400, 800, 1220, 1800, 2000, 2000]插值
        #     arm_pilot_fused_data_temp4 = interpolate(dif_arm_anglepid4, gentle_arm4_input, gentle_arm4_output)
        #     # print(f"-------------3s else arm_pilot_fused_data_temp4:{arm_pilot_fused_data_temp4}")
        #     # print("-----卸载-----:", arm_pilot_fused_data_temp4)
        # # print(f"---------------------计时phase4_start_time:{time.time()}")
        arm_pilot_fused_data_temp4 = interpolate(dif_arm_anglepid5, gentle_arm4_input, gentle_arm4_output)
            
        self.u_arm_fused_ = arm_pilot_fused_data_temp4 - 1000.0
        # print(f"---------------------5阶段斗杆输出 u_arm_fused_:{self.u_arm_fused_}")

        if self.arm_pitch <-100:
            if self.u_arm_fused_ > 500:
                self.u_arm_fused_ = 500
                # print(f"---------------------4阶段斗杆arm_pitch <-100 u_arm_fused_:{self.u_arm_fused_}")
            if self.u_arm_fused_ < -300:
                self.u_arm_fused_ = -300  
                # print(f"---------------------4阶段斗杆arm_pitch >-100 u_arm_fused_:{self.u_arm_fused_}")          
        else:
            if self.u_arm_fused_ > 1000:
                self.u_arm_fused_ = 1000
            if self.u_arm_fused_ < -1000:
                self.u_arm_fused_ = -1000
        




    def dzh_add_4_arm_state_contorll_function(self):

        # self.calculate_arm4_pilot += 1
      
        # if self.calculate_arm4_pilot < 50:
        #     # now = rospy.Time.now()
        #     # dt = (now - self.last_sys_sco_traj_time_).to_sec() if self.last_sys_sco_traj_time_ is not None else 0.0
        #     # self.last_sys_sco_traj_time_ = now
        #     dt = 0.02
        #     if self.wxref_arm_angle_after_filter is None:
        #         self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        #     self.wxref_arm_angle_after_filter = rate_limit_filter(self.wxref_arm_angle, self.wxref_arm_angle_after_filter, dt, max_rate_up=3)
        # else:
        #     self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        # print(f"---------------------4阶段斗杆轨迹 wxref_arm_angle_after_filter:{self.wxref_arm_angle_after_filter}")
        self.dif_arm_anglepid4 = self.wxref_arm_angle_after_filter - self.current_angle_arm_
        # print(f"---------------------4阶段斗杆差值 dif_arm_anglepid4:{self.dif_arm_anglepid4}")
        # print(f"---------------------4阶段斗杆 current_angle_arm_:{self.current_angle_arm_}")
        # print(f"---------------------4阶段斗杆 wxref_arm_angle_after_filter:{self.wxref_arm_angle_after_filter}")

        # print(f"4444444444444444444斗杆差值dif_arm_anglepid4:{self.dif_arm_anglepid4}")
        # print(f"wxref_arm_angle_after_filter:{self.wxref_arm_angle_after_filter}")
        # print(f"current_angle_arm_:{self.current_angle_arm_}")
        # print(f"current_angle_bucket_:{self.current_angle_bucket_}")
        # print(f"current_angle_swing_:{self.current_angle_swing_}")

        

        # self.dif_arm_anglepid4 = self.wxref_arm_angle - self.current_angle_arm_ 
        # 原有提升卸载阶段控制插值，现用作卸载阶段插值
        # gentle_arm4_input = [-15, -10, -5, 0, 5, 10, 15]
        # gentle_arm4_output = [800, 800, 800, 1220, 1400, 1600, 1600]

        # gentle_arm4_input = [-15, -10, -5, 0, 5, 10, 15]
        # gentle_arm4_output = [800, 800, 800, 1220, 1400, 1500, 1600]


        gentle_arm4_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm4_output = [800, 800, 800, 1220, 1600, 1700, 1800]
        # arm_pilot_fused_data_temp4 = interpolate(self.dif_arm_anglepid4, gentle_arm4_input, gentle_arm4_output)
        # 提升阶段斗杆插值
        gentle_arm4_up_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm4_up_output = [400, 400, 800, 1000, 1200, 1300, 1400] 


        if (time.time() - self.phase4_start_time <= 3.0 and
            time.time() - self.phase4_start_time > 0 ):
            # 用的是[400, 400, 800, 1000, 1100, 1200, 1300]插值

            
            arm_pilot_fused_data_temp4 = interpolate(self.dif_arm_anglepid4, gentle_arm4_up_input, gentle_arm4_up_output)
        else:
            # 用的是 [400, 400, 800, 1220, 1800, 2000, 2000]插值
            arm_pilot_fused_data_temp4 = interpolate(self.dif_arm_anglepid4, gentle_arm4_input, gentle_arm4_output)
            
        self.u_arm_fused_ = arm_pilot_fused_data_temp4 - 1000.0
        # print(f"---------------------4阶段斗杆输出 u_arm_fused_:{self.u_arm_fused_}")

        if self.arm_pitch <-100:
            if self.u_arm_fused_ > 500:
                self.u_arm_fused_ = 500
                # print(f"---------------------4阶段斗杆arm_pitch <-100 u_arm_fused_:{self.u_arm_fused_}")
            if self.u_arm_fused_ < -300:
                self.u_arm_fused_ = -300  
                # print(f"---------------------4阶段斗杆arm_pitch >-100 u_arm_fused_:{self.u_arm_fused_}")          
        else:
            if self.u_arm_fused_ > 1000:
                self.u_arm_fused_ = 1000
            if self.u_arm_fused_ < -1000:
                self.u_arm_fused_ = -1000
                
        # print(f"---------------------4阶段斗杆回转current_angle_swing_:{self.current_angle_swing_}")
        # print(f".------------------------------------u_arm_fused_     {self.u_arm_fused_}")
# 
        # self.add_4_arm_state_contorll_function()

        # real_arm_angle_radian=real_arm_angle*(M_PI/180);
        #  ref_bucket_cutting_angle= calculateBucketReference(real_arm_angle_radian)*180/M_PI;
      
        # if self.calculate_arm4_pilot < 20:
        #      if self.u_arm_fused_ > 0:
        #         self.u_arm_fused_ = 0



    def dzh_add_4_swing_state_contorll_function(self):

        # if self.seqactionmode == 5:
        #         self.record_2_wxref_swing_angle= self.record_3_wxref_swing_angle
        #         self.record_2_current_angle_swing_end=self.record_3_current_angle_swing_end

                # record_2_wxref_swing_angle_end=0
        if self.record_2_wxref_swing_angle > 360:
            self.record_2_wxref_swing_angle_end= self.record_2_wxref_swing_angle-360
        
        else:
            if self.record_2_wxref_swing_angle <0:
                self.record_2_wxref_swing_angle_end= self.record_2_wxref_swing_angle+360
            else:
                self.record_2_wxref_swing_angle_end= self.record_2_wxref_swing_angle
        print(f"4阶段++++++++++record_2_wxref_swing_angle_end:%:{self.record_2_wxref_swing_angle_end}")
        print(f"4阶段++++++++++record_2_wxref_swing_angle:%:{self.record_2_wxref_swing_angle}")
       

        print(f"4阶段++++++++++:%:{self.swing_clockwise_switch}")
        print("current_angle_swing_{}".format(self.current_angle_swing_))
        if self.swing_clockwise_switch == True:                      #回转方向为顺时针
            if self.record_2_wxref_swing_angle_end < 135:
                # if self.current_angle_swing_ > 180 and self.current_angle_swing_ < 360:
                if self.current_angle_swing_ > 180 and self.current_angle_swing_ < 360:
                    self.amendment_current_angle_swing=self.current_angle_swing_ - 360
                    print(f"4阶段1111111111111111111amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                else: 
                    self.amendment_current_angle_swing=self.current_angle_swing_
                    # self.amendment_current_angle_swing=self.current_angle_swing_-360
                    print(f"4阶段222222222222222amendment_current_angle_swing:{self.amendment_current_angle_swing}")
            else:
                if self.record_2_wxref_swing_angle_end > 225:
                    if self.record_2_current_angle_swing_end < 180 and self.record_2_current_angle_swing_end>0:
                        if self.current_angle_swing_ > 0 and self.current_angle_swing_ < 180:
                            self.amendment_current_angle_swing=self.current_angle_swing_+360
                            print(f"4阶段3333333333333333amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                        
                        if self.current_angle_swing_ > 180 and self.current_angle_swing_ <=360:
                            self.amendment_current_angle_swing=self.current_angle_swing_ 
                            print(f"4阶段4444444444444444amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else: 
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"4阶段555555555555555amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                else:
                    # self.amendment_current_angle_swing=self.current_angle_swing_/
                    if self.current_angle_swing_ > 270 and self.current_angle_swing_ < 360:
                        self.amendment_current_angle_swing=self.current_angle_swing_-360
                        print(f"4阶段66666666666amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else:
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"4阶段7777777777777773amendment_current_angle_swing:{self.amendment_current_angle_swing}")

                    # print("True: shunshizhen xiangwei 0")

        else: 
            if self.record_2_wxref_swing_angle_end < 135:
                # self.record_2_current_angle_swing__end=
                if self.record_2_current_angle_swing_end < 360 and self.record_2_current_angle_swing_end>180:

                    if self.current_angle_swing_ > 180 and self.current_angle_swing_ < 360:
                        self.amendment_current_angle_swing=self.current_angle_swing_ - 360
                        print(f"4阶段-------------1111amendment_current_angle_swing:{self.amendment_current_angle_swing}")

                    if self.current_angle_swing_ >=0 and self.current_angle_swing_ < 180:
                            self.amendment_current_angle_swing=self.current_angle_swing_
                            print(f"4阶段-------------2222amendment_current_angle_swing:{self.amendment_current_angle_swing}")

                else:
                    self.amendment_current_angle_swing=self.current_angle_swing_
                    print(f"4阶段-------------333333amendment_current_angle_swing:{self.amendment_current_angle_swing}")
            else:
                if self.record_2_wxref_swing_angle_end > 225:

                    if self.current_angle_swing_ > 0 and self.current_angle_swing_ < 180:
                        self.amendment_current_angle_swing=self.current_angle_swing_ + 360
                        print(f"4阶段------------3333amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else: 
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"4阶段-------------4444amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                else:
                    if self.current_angle_swing_ > 0 and self.current_angle_swing_ < 90:
                        self.amendment_current_angle_swing=self.current_angle_swing_ + 360
                        print(f"4阶段-------------5555amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                    else: 
                        self.amendment_current_angle_swing=self.current_angle_swing_
                        print(f"4阶段-------------6666amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                
                        # print("false: nishizhen xiangwei 0")

        real_swing_angle_velocity_input = [-40, -40, -32, -18, -9, -2, 0, 2, 9, 18, 32, 40, 40]  # -左 +右dps = °/s 范围  -327dps 至 +327dps
        real_swing_angle_velocity_output = [15, 15, 15, 10, 5, 2, 0, 2, 5, 10, 15, 15, 15]  # jisuancuowu 

        ref_swing_angle_temp_4 = abs(interpolate(self.current_velovity_swing_, real_swing_angle_velocity_input, real_swing_angle_velocity_output))
        print(f"4阶段--角速度current_velovity_swing_:{self.current_velovity_swing_}")
        
        dif_swing_angleadd_4 = self.wxref_swing_angle - self.amendment_current_angle_swing

        dif_swing_angle_4 = abs(self.wxref_swing_angle - self.amendment_current_angle_swing)
        print(f"4阶段-------------wxref_swing_angle:{self.wxref_swing_angle}")
        print(f"4阶段-------------amendment_current_angle_swing:{self.amendment_current_angle_swing}")
        print(f"4阶段-------------dif_swing_angleadd_4:{dif_swing_angleadd_4}")
        


        # print(f"777777777777777wxref_swing_angle:{self.wxref_swing_angle}")
        # print(f"77777777777777777777amendment_current_angle_swing:{self.amendment_current_angle_swing}")
        # print(f"current_angle_arm_:{self.current_angle_arm_}")
        # print(f"current_angle_bucket_:{self.current_angle_bucket_}")
        # print(f"current_angle_swing_:{self.current_angle_swing_}")
        # cankaojiaodu xianfu
        if dif_swing_angle_4 < 35:
            dif_swing_angle_end_4 = abs(dif_swing_angle_4 - ref_swing_angle_temp_4)
            # std::cout<<"dif_swing_angle_end"<<dif_swing_angle_end<<std::endl;
            # if (dif_swing_angle_end<0) 
            # {      
            #     dif_swing_angle_end=0;
            # }
        else:
            dif_swing_angle_end_4 = dif_swing_angle_4

        # print(f"---------------------4阶段斗杆轨迹 wxref_arm_angle_after_filter:{self.wxref_arm_angle_after_filter}")
        armdiff_anglepid4 = self.wxref_arm_angle - self.current_angle_arm_
        # gentle_swing_angle_input = [0,       30,    45, 65,    80,   100,  140, 180, 360]
        # gentle_swing_angle_output = [0.55, 0.55, 0.55,  0.55, 0.50, 0.50,0.50,  0.55, 0.55]
                
        gentle_swing_angle_input = [0,       30,    45, 65,    80,   100,  140, 180, 360]
        gentle_swing_angle_output = [0.55, 0.55, 0.55,  0.55, 0.52, 0.52,0.52,  0.55, 0.55]
        

        gentle_swing_input = [0, 1, 3, 5, 25, 30, 45, 90, 180, 360]
        gentle_swing_output = [0, 0, 200, 450, 600, 750, 900, 1000, 1000, 1000]

        gentle_armdiff_input = [0, 10, 30, 50, 70, 90]
        gentle_armdiff_output = [0, 0, 50, 100, 200, 200]

        # 90度参数

        # gentle_swing_input = [0, 1, 3, 5, 25, 30, 45, 90, 180, 360]
        # gentle_swing_output = [0, 0, 200, 450, 600, 850, 900, 1000, 1000, 1000]

        # gentle_swing_input = [0, 1, 3, 10, 25, 30, 45, 90, 180, 360]
        # gentle_swing_output = [0, 0, 200, 350, 500, 700, 700, 700, 700, 700]

        # gentle_swing_input_slow = [0, 1, 3, 10, 25, 30, 45, 90, 180, 360]
        # gentle_swing_output_slow = [0, 0, 100, 150, 200, 300, 400, 700, 700, 700]

        gentle_swing_input_slow = [0, 1, 3, 5, 15, 30, 35, 45,90, 180, 360]
        gentle_swing_output_slow = [0, 0, 100, 300, 400, 400, 500,550, 600, 700, 700]

        gentle_swing_input_slow_5= [-10,  0,  1,  3,    5, 15, 30,  35,   45,  90, 180, 360]
        gentle_swing_output_slow_5 = [0,  0,  0, 100, 300, 400, 400, 450,500, 500, 500, 500]

        # gentle_swing_input_slow_7 = [0, 1, 3, 5, 15, 30, 35, 45,90, 180, 360]
        # gentle_swing_output_slow_7 = [0, 0, 150, 300, 400, 400, 400,400, 600, 700, 700]
                # 对异常的差值划归到[-180,+180)之间，正常的差值不作更改
        print("Before  self.dif_swing_angle_end_: {}".format(dif_swing_angle_end_4))
        dif_swing_angle_end_4 = abs(angle_diff2normal_area(dif_swing_angle_end_4))
        print("dif_swing_angle_end_4: {}".format(dif_swing_angle_end_4))

        swing_pilot_fused_data_temp_high = interpolate(dif_swing_angle_end_4, gentle_swing_input, gentle_swing_output)
        swing_pilot_fused_data_temp_slow = interpolate(dif_swing_angle_end_4, gentle_swing_input_slow, gentle_swing_output_slow)
        swing_pilot_fused_data_temp_slow_5 = interpolate(dif_swing_angle_end_4, gentle_swing_input_slow_5, gentle_swing_input_slow_5)
        armdiff_swing_pilot_fused_data_temp = interpolate(armdiff_anglepid4, gentle_armdiff_input, gentle_swing_output)
        # swing_pilot_fused_data_temp_slow_7 = interpolate(dif_swing_angle_end_4, gentle_swing_input_slow_7, gentle_swing_output_slow_7)

        # 放200处  calculate_swing_speed_fast_slow_= 0.0; 
        self.calculate_swing_speed_fast_slow_4 += 1
        if self.calculate_swing_speed_fast_slow_4 < 2:
                self.record_current_angle_swing_4 = self.amendment_current_angle_swing

                # print(f"self.amendment_current_angle_swing:{self.amendment_current_angle_swing}")

        # print(f"swing_angle_percent:{swing_angle_percent}")
        if self.seqactionmode == 5:
            self.calculate_swing_speed_fast_slow_4 =0
            if self.current_angle_boom_<12:
                swing_pilot_fused_data_temp_4 = 0
                print(f"5阶段 swing_pilot_fused_data_temp_4:{swing_pilot_fused_data_temp_4}")
            else:
                swing_pilot_fused_data_temp_4 = swing_pilot_fused_data_temp_slow
                print(f"5阶段 swing_pilot_fused_data_temp_4:{swing_pilot_fused_data_temp_4}")


        else:
            
            if self.wxref_swing_angle < self.amendment_current_angle_swing:
                # 取小  self.truck_swive_data_angle  self.wxref_swing_angle
                truck_swive_data_angle_comp=min(self.wxref_swing_angle,self.truck_swive_data_angle)
                swing_angle_err=abs(self.record_current_angle_swing_4 - truck_swive_data_angle_comp)
                swing_angle_percent = interpolate(swing_angle_err, gentle_swing_angle_input, gentle_swing_angle_output)
                # print(f"min -truck_swive_data_angle_min:{truck_swive_data_angle_min}")

                self.High_Slow_Switch_Values_4 = self.record_current_angle_swing_4- (self.record_current_angle_swing_4 - truck_swive_data_angle_comp) * swing_angle_percent
                
                
                if self.amendment_current_angle_swing > self.High_Slow_Switch_Values_4:
                # if self.record_current_angle_swing_4 > self.truck_swive_data_angle*0.7:

                    swing_pilot_fused_data_temp_4 = swing_pilot_fused_data_temp_high  # 用快
                
                else:
                    swing_pilot_fused_data_temp_4 = swing_pilot_fused_data_temp_slow  # 慢
                
            else:

                truck_swive_data_angle_comp=max(self.wxref_swing_angle,self.truck_swive_data_angle)
                swing_angle_err=abs(self.record_current_angle_swing_4 - truck_swive_data_angle_comp)
                swing_angle_percent = interpolate(swing_angle_err, gentle_swing_angle_input, gentle_swing_angle_output)
                # print(f"max -truck_swive_data_angle_max:{truck_swive_data_angle_max}")

                # self.High_Slow_Switch_Values_4 = self.record_current_angle_swing_4 + (self.wxref_swing_angle - self.record_current_angle_swing_4) * swing_angle_percent
                # self.High_Slow_Switch_Values_4 = self.record_current_angle_swing_4 + (self.truck_swive_data_angle - self.record_current_angle_swing_4) * swing_angle_percent
                self.High_Slow_Switch_Values_4_5 = self.record_current_angle_swing_4 + (truck_swive_data_angle_comp - self.record_current_angle_swing_4) * swing_angle_percent
                
                if self.amendment_current_angle_swing < self.High_Slow_Switch_Values_4_5:
                    swing_pilot_fused_data_temp_4 = swing_pilot_fused_data_temp_high  # 用快
                else:
                    swing_pilot_fused_data_temp_4 = swing_pilot_fused_data_temp_slow  # 慢
 
        if self.wxref_swing_angle < self.amendment_current_angle_swing:
            swing_pilot_fused_data_temp2_4 = 0 - swing_pilot_fused_data_temp_4-armdiff_swing_pilot_fused_data_temp-100.0
            # print(f"------斗杆增加-----armdiff_swing_pilot_fused_data_temp:{armdiff_swing_pilot_fused_data_temp}") 
            # print(f"+++++++积分self.swing_integral_control_4 :{self.swing_integral_control_4 }")
            # print(f"+++++++++++++++swing_pilot_fused_data_temp2_4:{swing_pilot_fused_data_temp2_4}") 

        else:
            swing_pilot_fused_data_temp2_4 = swing_pilot_fused_data_temp_4 * (1)+armdiff_swing_pilot_fused_data_temp
            # print(f"+++++++++++++++swing_pilot_fused_data_temp2_4:{swing_pilot_fused_data_temp2_4}") 
            # print(f"-----------积分self.swing_integral_control_4 :{self.swing_integral_control_4 }")
            # print(f"------斗杆增加-----armdiff_swing_pilot_fused_data_temp:{armdiff_swing_pilot_fused_data_temp}") 
            

        if dif_swing_angle_4 > 180:
            # swing_pilot_fused_data_temp_end=swing_pilot_fused_data_temp2*(-1);
            swing_pilot_fused_data_temp_end_4 = swing_pilot_fused_data_temp2_4 * (1)
            # print(f"+++++++积分self.swing_integral_control_4 :{self.swing_integral_control_4 }")  
        else:
            swing_pilot_fused_data_temp_end_4 = swing_pilot_fused_data_temp2_4 * (1)
            # print(f"444444441swing_pilot_fused_data_temp_end_4:{swing_pilot_fused_data_temp_end_4}")  

        # self.swing_integral_control_4 = self.update_integral_swing_control(-250, 250, 13.0, dif_swing_angleadd_4, 10.0)
        self.swing_integral_control_4 = self.update_integral_swing_control(-200, 200, 15.0, dif_swing_angleadd_4, 15.0)
        self.u_swing_fused_ = (swing_pilot_fused_data_temp_end_4+ self.swing_integral_control_4)
        # self.u_swing_fused_ = swing_pilot_fused_data_temp_end_4
        print(f"回转输出u_swing_fused_:{self.u_swing_fused_}")  
        self.calculate_arm4_pilot += 1
        # if self.calculate_arm4_pilot<200:
        #     self.record_4_wxref_swing_angle= self.wxref_swing_angle
        #     self.record_4_current_angle_swing_end=self.current_angle_swing_
        # else:
        #     if self.current_angle_bucket_ > -60.0:
        #         if self.record_4_swing_clockwise_switch==self.swing_clockwise_switch:
        #             self.record_4_wxref_swing_angle= self.wxref_swing_angle   
        #             self.record_4_current_angle_swing_end=self.current_angle_swing_
        #             self.record_4_4_wxref_swing_angle=self.wxref_swing_angle
        #         else:
        #             self.record_4_wxref_swing_angle= self.record_4_4_wxref_swing_angle
        #             self.record_4_current_angle_swing_end=self.current_angle_swing_
        #     else:
        #         self.record_4_wxref_swing_angle= self.wxref_swing_angle
        #         self.record_4_current_angle_swing_end=self.current_angle_swing_
        # print(f"4阶段记录record_4_wxref_swing_angle:{self.record_4_wxref_swing_angle}") 
        # print(f"4阶段记录record_4_current_angle_swing_end:{self.record_4_current_angle_swing_end}") 




        # # 提升阶段斗杆插值
        # gentle_arm4_up_input = [-15, -10, -5, 0, 5, 10, 15]
        # gentle_arm4_up_output = [400, 400, 800, 1000, 1100, 1200, 1200]
        # arm_pilot_fused_data_temp4 = interpolate(self.dif_arm_anglepid4, gentle_arm4_up_input, gentle_arm4_up_output)
        # print("-----提升------")
            
        
# 6阶段 动臂、铲斗
    def dzh_add_6_state_contorll_function(self):
        # gentle_boom6_input = [-2.00, -1.00, -0.50,    0.10, 0.15, 0.45,  1.00,2.00]
        # gentle_boom6_output = [2000, 1500, 1500, 1400, 1000, 800,600, 650]
        bucket_truck_distance_add=self.bucket_truck_distance-self.stage6_bridge_the_gap

        gentle_boom6_input = [-2.00, -1.00, -0.50,    0.10, 0.15,0.25, 0.45,  1.00,2.00]
        gentle_boom6_output = [1700, 1500,    1500,   1400,    1300, 700,600,500, 500]
        boom_pilot_fused_data_temp6 = interpolate(bucket_truck_distance_add, gentle_boom6_input, gentle_boom6_output)
        self.u_boom_fused_ = boom_pilot_fused_data_temp6 - 1000.0

        self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        # print(f"666666wxref_arm_angle_after_filter_:{self.wxref_arm_angle_after_filter}") 
        self.dif_arm_anglepid6 = self.wxref_arm_angle_after_filter - self.current_angle_arm_        
        if self.dif_arm_anglepid6<0:
            if self.current_angle_bucket_ <-70 and self.current_angle_arm_>-120:
            # if self.current_angle_arm_ >-50 and self.current_angle_arm_<20:
            # if self.current_angle_bucket_ >-50 and self.current_angle_bucket_<0:
                self.u_boom_fused_ = boom_pilot_fused_data_temp6 - 1000.0   
            else:
                self.u_boom_fused_ = -200
        else:
            self.u_boom_fused_ = boom_pilot_fused_data_temp6 - 1000.0 

        # print(f"self.u_boom_fused_1111:{self.u_boom_fused_}")

        # print(f"self.bucket_truck_distance1111:{self.bucket_truck_distance}")
         
        self.ref_bucket_cutting_angle6 = -125-self.current_angle_arm_-self.current_angle_boom_
        print(f"6666666666666666 self.ref_bucket_cutting_angle6:{self.ref_bucket_cutting_angle6}")
        # self.ref_bucket_cutting_angle6 = 0

        self.dif_bucket_anglepid6 = self.ref_bucket_cutting_angle6 - self.current_angle_bucket_
        # print(f"self.dif_bucket_anglepid6:{self.dif_bucket_anglepid6}")
        # print(f"self.ref_bucket_cutting_angle6:{self.ref_bucket_cutting_angle6}")
# 
        gentle_bucket6_input = [-15, -10, -5,-3, 0,3, 5,10, 15, 20]
        gentle_bucket6_output = [200, 400, 600,1000, 1000, 1000,1400, 1500, 1500, 1500]

        self.bucket_pilot_fused_data_temp6 = interpolate(self.dif_bucket_anglepid6, gentle_bucket6_input, gentle_bucket6_output)
        # print(f"self.bucket_pilot_fused_data_temp6:{self.bucket_pilot_fused_data_temp6}")
        self.u_bucket_fused_ = self.bucket_pilot_fused_data_temp6 - 1000.0
# 6阶段 斗杆
    def dzh_add_6_arm_state_contorll_function(self):
        self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        # print(f"666666wxref_arm_angle_after_filter_:{self.wxref_arm_angle_after_filter}") 
        self.dif_arm_anglepid6 = self.wxref_arm_angle_after_filter - self.current_angle_arm_
        # print(f"7777777-dif_arm_anglepid6:{self.dif_arm_anglepid6}") 

        # self.dif_arm_anglepid4 = self.wxref_arm_angle - self.current_angle_arm_ 

        # 原有提升卸载阶段控制插值，现用作卸载阶段插值
        gentle_arm6_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm6_output = [400, 400, 500, 1200, 1400, 1450, 1500]
        gentle_arm6_distance_input = [-2.00, -1.00, -0.50,    0.10, 0.15,0.20, 0.45,  1.00,2.00]
        gentle_arm6_distance_output = [0, 0,         -100,     -100, -100, -100,-100,0, 0]
        # arm_pilot_fused_data_temp6 = interpolate(self.dif_arm_anglepid6, gentle_arm6_input, gentle_arm6_output)

        # self.dif_boom_anglepid4 = self.wxref_boom_angle - self.current_angle_boom_
        

        if self.dif_arm_anglepid6<0:
            arm_pilot_fused_data_temp6 = interpolate(self.dif_arm_anglepid6, gentle_arm6_input, gentle_arm6_output)
            self.u_arm_fused_ = arm_pilot_fused_data_temp6 - 1000.0
        else:
            if abs(self.dif_arm_anglepid6)<2:
                self.u_arm_fused_=0
            else:
                bucket_truck_distance_add=self.bucket_truck_distance-self.stage6_bridge_the_gap
                arm_pilot_fused_data_temp6 = interpolate(self.dif_arm_anglepid6, gentle_arm6_input, gentle_arm6_output)
                arm_pilot_fused_data_temp7 = interpolate(bucket_truck_distance_add, gentle_arm6_distance_input, gentle_arm6_distance_output)
                

                self.u_arm_fused_ = arm_pilot_fused_data_temp6 -arm_pilot_fused_data_temp7-1000.0

        # self.u_arm_fused_ = arm_pilot_fused_data_temp6 - 1000.0
# 7阶段  
    def dzh_add_7_state_contorll_function(self):
        # gentle_boom6_input = [-2.00, -1.00, -0.50,    0.10, 0.15, 0.45,  1.00,2.00]
        # gentle_boom6_output = [2000, 1500, 1500, 1400, 1000, 800,600, 650]
        bucket_ground_distance_add=self.bucket_ground_distance  #这个高度需要触底去量s 
        print(bucket_ground_distance_add)

        gentle_boom7_input = [-2.00, -1.00, -0.50,    0.10, 0.15,0.20, 0.45,  1.00,2.00]
        gentle_boom7_output = [1500, 1450, 1400, 1000, 900, 700,550,500, 500]
        boom_pilot_fused_data_temp7 = interpolate(bucket_ground_distance_add, gentle_boom7_input, gentle_boom7_output)
        self.u_boom_fused_ = boom_pilot_fused_data_temp7 - 1000.0
        # dif_swing_angle

        # self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        # # print(f"666666wxref_arm_angle_after_filter_:{self.wxref_arm_angle_after_filter}") 
        # self.dif_arm_anglepid7 = self.wxref_arm_angle_after_filter - self.current_angle_arm_        
        # if self.dif_arm_anglepid7<0:
        #     if self.current_angle_bucket_ <-50 and self.current_angle_arm_<-90:
        #         self.u_boom_fused_ = boom_pilot_fused_data_temp7 - 1000.0   
        #     else:
        #         self.u_boom_fused_ = -200
        # else:
        #     self.u_boom_fused_ = boom_pilot_fused_data_temp7 - 1000.0 

        # print(f"self.u_boom_fused_1111:{self.u_boom_fused_}")

        # print(f"self.bucket_truck_distance1111:{self.bucket_truck_distance}")

        self.dif_bucket_anglepid7 = self.ref_bucket_cutting_angle_stage7 - self.current_angle_bucket_
        # print(f"self.ref_bucket_cutting_angle777:{self.dif_bucket_anglepid7}")
# 
        gentle_bucket7_input = [-15, -10, -5,-3, 0,3, 5,10, 15, 20]
        gentle_bucket7_output = [0, 0, 400,1000, 1000, 1000,1400, 1600, 2000, 2000]

        bucket_pilot_fused_data_temp7 = interpolate(self.dif_bucket_anglepid7, gentle_bucket7_input, gentle_bucket7_output)
        self.u_bucket_fused_ = bucket_pilot_fused_data_temp7 - 1000.0
# 7阶段 斗杆
    def dzh_add_7_arm_state_contorll_function(self):
        self.wxref_arm_angle_after_filter = self.wxref_arm_angle
        self.dif_arm_anglepid7 = self.wxref_arm_angle_after_filter - self.current_angle_arm_

        gentle_arm7_input = [-15, -10, -5, 0, 5, 10, 15]
        gentle_arm7_output = [400, 500, 500, 700, 1400, 1450, 1500]
        gentle_arm7_distance_input = [-2.00, -1.00, -0.50,    0.10, 0.15,0.20, 0.45,  1.00,2.00]
        gentle_arm7_distance_output = [0, 0, -100, -100, -100, -100,-100,0, 0]

        print("dif_arm_anglepid7:{}".format(self.dif_arm_anglepid7))
        print("wxref_arm_after:{}".format(self.wxref_arm_angle_after_filter))
        if self.dif_arm_anglepid7<0:
            arm_pilot_fused_data_temp7 = interpolate(self.dif_arm_anglepid7, gentle_arm7_input, gentle_arm7_output)
            self.u_arm_fused_ = arm_pilot_fused_data_temp7 - 1000.0
        else:
            if abs(self.dif_arm_anglepid7)<2:
                # self.u_arm_fused_=0
                pass
            else:
                bucket_ground_distance_add=self.bucket_ground_distance
                arm_pilot_fused_data_temp7 = interpolate(self.dif_arm_anglepid7, gentle_arm7_input, gentle_arm7_output)
                arm_pilot_fused_data_temp8 = interpolate(bucket_ground_distance_add, gentle_arm7_distance_input, gentle_arm7_distance_output)
                

                self.u_arm_fused_ = arm_pilot_fused_data_temp7 -arm_pilot_fused_data_temp8-1000.0
        print("u_arm_fused_: {}".format(self.u_arm_fused_))


 

    def stop_swing_main(self):
        # Point3D bucket_pos = {5.0, -2.2, 2.3};
        truck_pos_x = 601062.4
        truck_pos_y = 3475450.76
        truck_pos_z = 17.5
        # Point3D boom_pos = {0.0, 0.0, 0.0};

        current_angular_velocity = 30.0
        current_angular_acceleration = 50.0
        stop_angle = 0.0

        boom_pos_x = self.boom_enu_point.point.x
        boom_pos_y = self.boom_enu_point.point.y
        boom_pos_z = self.boom_enu_point.point.z
        bucket_pos_x = self.bucket_enu_point.point.x
        bucket_pos_y = self.bucket_enu_point.point.y
        bucket_pos_z = self.bucket_enu_point.point.z - 1.0
        # Point3D truck_pos = {truck_enu_point.point.x, truck_enu_point.point.y, truck_enu_point.point.z};

        stop_swing = False

        stop_angle = current_angular_velocity * current_angular_velocity / (2 * current_angular_acceleration)
        angleDeg = self.compute_Angle(bucket_pos_x - boom_pos_x, bucket_pos_y - boom_pos_y, truck_pos_x - boom_pos_x, truck_pos_y - boom_pos_y)

        if abs(angleDeg) < 15:
            self.delta_target_sub_current_angle_boom = self.wxref_boom_angle - self.current_angle_boom_
            delta_height = bucket_pos_z - truck_pos_z
            # std::cout<<"delta_target_sub_current_angle_boom:    "<<delta_target_sub_current_angle_boom<<std::endl;
            if delta_height > 0.0:
                stop_swing = False
                self.bucket_is_upper_truck = True
            else:
                self.bucket_is_upper_truck = False
                stop_swing = True

        self.stop_swing_angle_flag_ = stop_swing
        if self.seqactionmode == 3:
            self.current_swing_angle_ = angleDeg + self.current_angle_swing_

    # def stop_dig_up_boom(self):
    #     if self.seqactionmode == 3:  # 挖掘阶段
    #         # 当铲斗齿尖挖掘到绝对高程下限时，停止挖掘，提升动臂
    #         if self.working_alt_down >= self.bucket_enu_point.point.z:
    #             self.stop_dig_up_boom_flag = True
    #         else:
    #             self.stop_dig_up_boom_flag = False

    # 实时计算铲斗切削角
    def calculateBucketReference(self, arm_angle):
        boom_length = 5703.58
        arm_length = 2929.94
        bucket_length = 1464.4056
        pi = math.pi
        angle_bucket_toe = 50 * pi / 180

        cos_val = math.cos(pi + arm_angle)
        cos_angle_bucket_toe = math.cos(50 * pi / 180)

        numerator1 = arm_length - boom_length * cos_val
        numerator2 = bucket_length * cos_angle_bucket_toe
        denominator = math.sqrt(boom_length * boom_length + arm_length * arm_length - 2 * boom_length * arm_length * cos_val)
        
        # 数值安全检查 - 非常重要！
        ratio1 = numerator1 / denominator
        if ratio1 > 1.0:
            ratio1 = 1.0
            rospy.logwarn("acos parameter clamped to 1.0 (was: %f)", numerator1/denominator)
        elif ratio1 < -1.0:
            ratio1 = -1.0
            rospy.logwarn("acos parameter clamped to -1.0 (was: %f)", numerator1/denominator)
        
        ratio2 = numerator2 / denominator
        if ratio2 > 1.0:
            ratio2 = 1.0
            rospy.logwarn("acos parameter clamped to 1.0 (was: %f)", numerator2/denominator)
        elif ratio2 < -1.0:
            ratio2 = -1.0
            rospy.logwarn("acos parameter clamped to -1.0 (was: %f)", numerator2/denominator)
        
        bucket_angle = math.acos(ratio1) + math.acos(ratio2) + angle_bucket_toe - 20 * pi / 180 - pi
        bucket_angle_level = math.acos(ratio1) + math.acos(ratio2) + angle_bucket_toe - 20 * pi / 180 - pi   #dai xiugai 

    def calculateBucketRef(self):
        angle_prot = 5.0
        self.raw_angle_bkt = self.bucket_azimuth_angle - self.current_angle_boom_ - self.current_angle_arm_ - angle_prot
        if self.raw_angle_bkt < self.BUCKET_ANGLE_MIN:
            self.raw_angle_bkt = self.BUCKET_ANGLE_MIN
        elif self.raw_angle_bkt > self.BUCKET_ANGLE_MAX:
            self.raw_angle_bkt = self.BUCKET_ANGLE_MAX

    def initializeInterpolationCurves(self):
        # 回转关节插值点 (手柄输入 -> 控制输出)
        self.swing_input_points = [-1.0, -0.5, 0.0, 0.5, 1.0]
        self.swing_output_points = [-0.8, -0.3, 0.0, 0.3, 0.8]

    def truck_pos_compute(self):
        if not self.truck_enu_point:
            return
            
        truck_pos = {
            'x': self.truck_enu_point.point.x,
            'y': self.truck_enu_point.point.y,
            'z': self.truck_enu_point.point.z
        }
        
        # 注意：这里需要根据实际需求实现坐标变换逻辑
        # 原C++代码使用了Eigen库，Python中可以使用numpy或类似实现
        truck_delta_pos_1 = {'x': -6.0, 'y': -1.0, 'z': -0.5}
        
        # 构建平移向量（简化版本，实际需要完整的旋转矩阵变换）
        truck_translation_last = [
            truck_pos['x'] + truck_delta_pos_1['x'],
            truck_pos['y'] + truck_delta_pos_1['y'],
            truck_pos['z'] + truck_delta_pos_1['z'],
            1.0
        ]
        
        # TODO: 实现完整的坐标变换逻辑
        # 需要根据车身IMU数据构建旋转矩阵
        # transform_rotation.prerotate(AngleAxisd(truck_roll, Vector3d::UnitX()))
        # transform_rotation.prerotate(AngleAxisd(-truck_pitch, Vector3d::UnitY()))
        # transform_rotation.prerotate(AngleAxisd(trcuk_heading, Vector3d::UnitZ()))

        
        # TODO: 实现完整的坐标变换逻辑
        # 需要根据车身IMU数据构建旋转矩阵
        trcuk_heading = 0.0
        truck_roll = 0.0
        truck_pitch = 0.0
        cx = math.cos(truck_roll)
        sx = math.sin(truck_roll)
        cy = math.cos(-truck_pitch)
        sy = math.sin(-truck_pitch)
        cz = math.cos(trcuk_heading)
        sz = math.sin(trcuk_heading)

        rx = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, cx, -sx, 0.0],
            [0.0, sx, cx, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        ry = np.array([
            [cy, 0.0, sy, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [-sy, 0.0, cy, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        rz = np.array([
            [cz, -sz, 0.0, 0.0],
            [sz, cz, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        transform_rotation = rz @ ry @ rx
        truck_translation_current = transform_rotation @ np.array(truck_translation_last)

        # 加上矿卡rtk的坐标，得到全局坐标系下的位置坐标

        return

    def compute_Angle(self, x1, y1, x2, y2):
        # 点积与叉积
        dot = x1 * x2 + y1 * y2
        cross = x1 * y2 - y1 * x2

    

        # 使用 atan2 获取带符号的角度（弧度）
        angleRad = math.atan2(cross, dot)

        # 转换为角度制
        angleDeg = angleRad * 180.0 / math.pi

        return angleDeg

    def run(self):
        
        if self.mpc_switch_ or self.mpc_switch2_:
            # mpc_switch2_ 自动循环状态机：内部驱动 seqactionmode（4卸载->2复位->7清台->3挖掘->4卸载->2复位->7清台）
            # if self.mpc_switch2_:
            #     self._mpc_switch2_update() #备用函数  清空一次记录值 
            print(f"222222jieduan:{self.seqactionmode}")
            if self.prev_seqactionmode == 3.5 and self.seqactionmode == 4:
                self.phase4_start_time = time.time()
                # self.phase4_start_active = True
            elif self.prev_seqactionmode == 5.5 and self.seqactionmode == 4:
                self.phase5to4_start_active = True
            elif self.prev_seqactionmode == 6 and self.seqactionmode == 2:
                self.phase6to2_start_active = True
            
            if self.seqactionmode != 4:
                self.phase5to4_start_active = False
            if self.seqactionmode != 2:
                self.phase6to2_start_active = False

            # 线缆功能安全，进入拖电线缆区域，停止动作并向功能安全节点请求接管。
            print("self.angle_cable_lb_deg.{}".format(self.angle_cable_lb_deg))
            print("self.angle_cable_ub_deg.{}".format(self.angle_cable_ub_deg))
            if self.angle_cable_lb_deg < self.angle_cable_ub_deg:
                if (self.current_angle_swing_ > self.angle_cable_lb_deg and
                    self.current_angle_swing_ < self.angle_cable_ub_deg):
                    is_in_sector = True
                    # self.u_swing_fused_ = 0
                    # self.u_boom_fused_ = 0
                    # self.u_arm_fused_ = 0
                    # self.u_bucket_fused_ = 0
                    # self._add_fault(self.FAULT_CABLE_TAKEOVER)
                    pass
                else:
                    is_in_sector = False
        #           self._clear_fault(self.FAULT_CABLE_TAKEOVER)
            else:
                if ((self.current_angle_swing_ > 0 and self.current_angle_swing_ < self.angle_cable_ub_deg) or 
                    (self.current_angle_swing_ > self.angle_cable_lb_deg and self.current_angle_swing_ < 360)):
                    is_in_sector = True
                else:
                    is_in_sector = False
        #           self._clear_fault(self.FAULT_CABLE_TAKEOVER)

            # 阶段跳转检测
            override_4_enable = False
            if self.prev_seqactionmode == 2 and self.seqactionmode == 3:
                if is_in_sector == True:
                    override_4_enable = True
            if self.seqactionmode != 4 and self.seqactionmode != 3:
                override_4_enable = False
            self.prev_seqactionmode = self.seqactionmode

            if self.mpc_switch2_ and self.seqactionmode == 7:

                self.dzh_add_7_arm_state_contorll_function()
                self.dzh_add_7_state_contorll_function()
                self.u_swing_fused_ = 0
                # self.dzh_add_7_swing_state_contorll_function()
                # self.calculate_swing_speed_fast_slow_ = 0.0
                # self.record_4_wxref_swing_angle = self.wxref_swing_angle
                # self.record_4_current_angle_swing_end = self.current_angle_swing_
                # self.swing_integral_control = 0.0
                # self.swing_integral_control_4 = 0.0
                # self.calculate_arm_pilot = 0
                print(f"[mpc_switch2] 7阶段(清台) seqactionmode:{self.seqactionmode}")

            elif self.seqactionmode == 2:

                # print("#222222222", self.record_2_wxref_swing_angle_end)
                
                self.calculate_arm4_pilot = 0
                self.rate_limit.set_joints(boom=(6, 50), arm=(1000, 1000.0)) 
                # self.wxref_arm_angle_after_filter = None

                # 清台循环：2阶段回转解缠的起始角参考
                if self.mpc_switch2_:
                    if self.record_1_state == 1:
                        # 第一个循环存在1阶段：沿用1阶段记录的当前回转角，不覆盖
                        pass
                    else:
                        # 无1阶段的循环：起始角取清台阶段 卸载 角
                        self.record_4_current_angle_swing_end = self.sys_unload_angle_

                
                self.SWingfeedforwardpdcontroller()
                self.add_2_arm_state_contorll_function()
                self.add_2_boom_state_contorll_function()
                
                self.add_2_2stage_switch_3stage_arm_down_function()

                if self.mpc_switch2_:
                    if self.u_boom_fused_ >= 400:
                        self.u_boom_fused_ = 400
                    elif self.u_boom_fused_ <= -400:
                        self.u_boom_fused_ = -400
                    if self.u_arm_fused_ <= -300:
                        self.u_arm_fused_ = -300
                self.calculate_swing_speed_fast_slow_4 = 0.0
                self.calculate_bucket_pilot= 0.0
                self.record_2_wxref_swing_angle= self.digpoint_angle
                self.record_2_current_angle_swing_end=self.current_angle_swing_
                self.swing_integral_control_4= 0.0
                self.record_5_state=0
                print(f"2222wxref_swing_angle:{self.wxref_swing_angle}")
                print(f"2222amendment_current_angle_swing:{self.amendment_current_angle_swing}")
                
            elif self.seqactionmode == 3:

                self.rate_limit.set_joints(boom=(15.0, 10.0), arm=(9, 1000.0), bucket=(1000, 1000.0))  
                # print(f"3jieduan:{self.u_boom_fused_}")
               
                # print("3阶段 #3333333333", self.record_2_wxref_swing_angle_end)
                # print("3阶段 #3333333333", self.record_2_wxref_swing_angle_end)
                
                self.calculate_arm4_pilot = 0
                self.calculate_bucket_pilot= 0.0
                # self.wxref_arm_angle_after_filter = None
                # self.Dig_state_controller()
                self.dzh_add_3_boom_state_contorll_function()
                # self.xy_add_3_state_contorll_function()
                self.dzh_add_3_arm_state_contorll_function()
                self.calculate_arm_pilot = 0
                
                self.dzh_add_3_state_contorll_function()
                self.calculate_swing_speed_fast_slow_ = 0.0
                self.calculate_swing_speed_fast_slow_4 = 0.0
                self.swing_integral_control_4= 0.0
                if self.record_1_state == 1:
                    self.record_2_wxref_swing_angle=self.current_angle_swing_
                    self.record_2_current_angle_swing_end=self.current_angle_swing_ 
                else:
                    self.record_2_wxref_swing_angle= self.digpoint_angle
                    self.record_2_current_angle_swing_end=self.current_angle_swing_
                
                if self.mpc_switch2_:
                    if self.record_1_state == 1:
                        # 第一个循环存在1阶段：沿用1阶段记录的当前回转角，不覆盖
                        pass
                    else:
                        # 无1阶段的循环：起始角取清台阶段 挖掘 角
                        self.record_2_wxref_swing_angle = self.clean_digpoint_angle
                # self.record_2_wxref_swing_angle= self.digpoint_angle
                # self.record_2_current_angle_swing_end=self.current_angle_swing_
                self.record_3_wxref_swing_angle= self.digpoint_angle
                self.record_3_current_angle_swing_end=self.current_angle_swing_   # 5进4 
                print(f"3jieduan3333333record_2_wxref_swing_angle:{self.digpoint_angle}")
                print(f"3jiedua33333333record_2_wxref_swing_angle:{self.current_angle_swing_}")
                self.High_Slow_Switch_Values_4=0
                self.High_Slow_Switch_Values_4_5=0
                self.record_4_state=0
                self.record_5_state=0
                self.u_swing_fused_ = 0


            elif self.seqactionmode == 4:
                if self.record_5_state == 5:
                    self.record_2_wxref_swing_angle=self.record_5_wxref_swing_angle
                    self.record_2_current_angle_swing_end=self.record_5_current_angle_swing_end 

                # 清台循环：4阶段为清台起始阶段
                # 回转解缠的起始角取清台阶段当前角
                if self.mpc_switch2_:
                    self.record_2_current_angle_swing_end = self.clean_digpoint_angle
                    if self.record_1_state == 1:
                        self.record_2_wxref_swing_angle=self.current_angle_swing_
                        self.record_2_current_angle_swing_end=self.current_angle_swing_ 
                self.rate_limit.set_joints(boom=(3000, 60), arm=(600, 1000.0), bucket=(45.0, 60.0)) 
                self.calculate_arm4_pilot += 1
                self.dzh_add_4_swing_state_contorll_function()
                self.dzh_add_4_arm_state_contorll_function()
                # self.dzh_add_4_swing_state_contorll_function()
                self.dzh_add_4_state_contorll_function()
                self.calculate_arm_pilot = 0
                self.calculate_swing_speed_fast_slow_ = 0.0
                 
                self.record_4_wxref_swing_angle= self.unload_angle_

                # 清台阶段记录清台的卸载为2阶段起始角
                if self.mpc_switch2_:
                    if self.record_1_state == 1:
                        # 第一个循环存在1阶段：沿用1阶段记录的当前回转角，不覆盖
                        pass
                    else:
                        # 无1阶段的循环：起始角取清台阶段卸载角
                        self.record_4_wxref_swing_angle = self.sys_unload_angle_
                
                #  self.record_4_wxref_swing_angle= self.wxref_swing_angle
                self.record_4_current_angle_swing_end=self.current_angle_swing_ 
                self.swing_integral_control= 0.0
                self.record_4_swing_clockwise_switch=self.swing_clockwise_switch
                self.record_4_state=4
                self.record_1_state =0
                # print(f"44444444jieduan:{self.u_boom_fused_}")
                # unload_angle_
                print(f"---卸载点---unload_angle_:{self.unload_angle_}")
                print(f"---车尾角---unload_angle_:{self.truck_swive_data_angle}")
                
                print(f"---方向---swing_clockwise_switch:{self.swing_clockwise_switch}")
                print(f"444444444444444record_4_wxref_swing_angle:{self.record_4_wxref_swing_angle}")
                print(f"444444444444record_4_current_angle_swing_end:{self.record_4_current_angle_swing_end}")

                # elif self.u_swing_fused_ and override_4_enable == True:
                #     pass

            elif self.seqactionmode == 5:
                self.rate_limit.set_joints(boom=(3000,8), arm=(600, 1000.0), bucket=(45.0, 20.0))  
                
                self.dzh_add_5_arm_state_contorll_function()
                self.dzh_add_4_swing_state_contorll_function()
                self.dzh_add_5_state_contorll_function()
                self.calculate_arm_pilot = 0
                self.calculate_bucket_pilot= 0.0
                self.calculate_swing_speed_fast_slow_ = 0.0
                
            
                # self.record_5_wxref_swing_angle= self.wxref_swing_angle
                self.record_5_wxref_swing_angle= self.current_angle_swing_
                self.record_5_current_angle_swing_end=self.current_angle_swing_   # 5进4 
                self.swing_integral_control= 0.0
                self.record_5_state=5
                self.record_1_state =0
            
            elif self.seqactionmode == 6:
                self.dzh_add_6_arm_state_contorll_function()
                self.dzh_add_6_state_contorll_function()
                self.u_swing_fused_ = 0.0
                self.record_7_wxref_swing_angle= self.wxref_swing_angle
                self.record_7_current_angle_swing_end=self.current_angle_swing_
                self.calculate_arm_pilot = 0
                self.calculate_bucket_pilot= 0.0
                # self.record_6_state=6


                print(f"6666666666record_7_wxref_swing_angle:{self.record_7_wxref_swing_angle}")
                print(f"66666666666record_7_current_angle_swing_end:{self.record_7_current_angle_swing_end}")
        
                self.swing_integral_control_7= 0.0



            elif self.seqactionmode == 5.5 or self.seqactionmode == 3.5 or self.seqactionmode == 4.5 :
                self.u_swing_fused_ = 0.0
                self.u_boom_fused_ = 0.0
                self.u_arm_fused_ = 0.0
                self.u_bucket_fused_ = 0.0
                print(f"5.55555555u_swing_fused_:{self.u_swing_fused_}")
            elif self.seqactionmode == 2.5:
                pass

            elif self.seqactionmode == 1:

                self.record_4_wxref_swing_angle= self.current_angle_swing_
                self.record_4_current_angle_swing_end=self.current_angle_swing_

                print(f"1111111111111111record_4_wxref_swing_angle:{self.record_4_wxref_swing_angle}")
                print(f"111111111111111record_4_wxref_swing_angle:{self.record_4_wxref_swing_angle}")
                self.u_swing_fused_ = 0.0
                self.u_boom_fused_ = 0.0
                self.u_arm_fused_ = 0.0
                self.u_bucket_fused_ = 0.0

                self.calculate_arm4_pilot = 0
                self.swing_integral_control= 0.0
                self.swing_integral_control_4=0.0
                self.swing_integral_control_7= 0.0
                self.record_4_state=0
                self.calculate_arm_pilot = 0
                self.record_1_state=1

                # self.wxref_arm_angle_after_filter = None

                # self.calculate_arm_pilot = 0
                # self.calculate_bucket_pilot= 0.0
                # self.calculate_swing_speed_fast_slow_ = 0.0
                # self.calculate_swing_speed_fast_slow_4 = 0.0
            else:
                self.rate_limit.set_joints(boom=(3000, 1000.0), arm=(600, 1000.0))  
                self.calculate_arm4_pilot = 0
                # self.wxref_arm_angle_after_filter = None

                self.calculate_arm_pilot = 0
                self.calculate_bucket_pilot= 0.0
                self.calculate_swing_speed_fast_slow_ = 0.0
                self.calculate_swing_speed_fast_slow_4 = 0.0

            # 限幅
            if self.u_swing_fused_ > 1000:
                self.u_swing_fused_ = 1000
            if self.u_swing_fused_ < -1000:
                self.u_swing_fused_ = -1000
            if self.u_boom_fused_ > 1000:
                self.u_boom_fused_ = 1000
            if self.u_boom_fused_ < -1000:
                self.u_boom_fused_ = -1000
            if self.u_arm_fused_ > 1000:
                self.u_arm_fused_ = 1000
            if self.u_arm_fused_ < -1000:
                self.u_arm_fused_ = -1000
            if self.u_bucket_fused_ > 1000:
                self.u_bucket_fused_ = 1000
            if self.u_bucket_fused_ < -1000:
                self.u_bucket_fused_ = -1000
            if is_in_sector and is_in_sector is not None:
                self.u_swing_fused_ = 0
                self.u_boom_fused_ = 0
                self.u_arm_fused_ = 0
                self.u_bucket_fused_ = 0

            # 回转中值滤波
            # if self.seqactionmode != 2:
            #     self.u_swing_fused_ = self.medianFilter(self.u_swing_fused_, self.window_)
        else:
            self.u_swing_fused_ = 0.0
            self.u_boom_fused_ = 0.0
            self.u_arm_fused_ = 0.0
            self.u_bucket_fused_ = 0.0
        self.last_seqactionmode = self.seqactionmode


        u_fused_msg = Quaternion()
        u_fused_msg.x = self.u_swing_fused_
        u_fused_msg.y = self.u_boom_fused_
        u_fused_msg.z = self.u_arm_fused_
        u_fused_msg.w = self.u_bucket_fused_
        self.u_fused_pub_.publish(u_fused_msg)
        # test_topic
        test_u_fused_msg = Quaternion()
        test_u_fused_msg.x = self.amendment_current_angle_swing
        # test_u_fused_msg.x = self.record_4_wxref_swing_angle_end
        test_u_fused_msg.y = self.record_current_angle_swing_4 
        test_u_fused_msg.z = self.record_4_wxref_swing_angle
        test_u_fused_msg.w = self.swing_integral_control_4     
        self.test_u_fused_pub_.publish(test_u_fused_msg)


    #功能安全相关内容
     # ---------- 故障码管理 ----------
    def _add_fault(self, fault_bit):
        with self.fault_lock:
            self.error_code |= fault_bit

    def _clear_fault(self, fault_bit):
        with self.fault_lock:
            self.error_code &= ~fault_bit
    def pack_data(self):
        with self.fault_lock:
            # 低十六位故障码
            error_asL16 = self.error_code & 0xFFFF
            hb = self.heartbeat
            self.heartbeat = (self.heartbeat + 1) % 256
        return ((hb & 0xFF)<< 16) | error_asL16
    def publish_10hz_task(self):
        """10Hz 发布任务（后台线程执行）"""
        # 初始化 10Hz 发布器
        pub = rospy.Publisher("/action_controlSafety_topic", UInt32, queue_size=10)
        msg = UInt32()         
        # 10Hz 专用 Rate（独立于主线程的 50Hz）
        loop_rate_10hz = rospy.Rate(10)  # 10Hz

        # 后台线程循环，直到 ROS 节点退出
        while not rospy.is_shutdown():
            try:
                
                msg.data = self.pack_data()
                pub.publish(msg)

                # rospy.loginfo("heartbeat=%d, error=%d", heartbeat, error_code)
                loop_rate_10hz.sleep()  # 休眠至 10Hz 周期
            except rospy.ROSInterruptException:
                break
