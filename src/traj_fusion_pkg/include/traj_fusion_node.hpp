/*
项目          修改人       修改日期          改动
无人挖掘机     wangxing    2025/11/12    act轨迹融合框架
无人挖掘机     wuql7       2025/11/17    添加act小脑推理需要的变量和函数
无人挖掘机     wuql7       2025/11/27    添加模型评价reward变量
无人挖掘机     wuql7       2025/12/19    融合杏姐1219修改的内容
无人挖掘机     wuql7       2025/12/19    融合杏姐1224修改的内容publishLaneCircleIntersections

z




*/

#ifndef TRAJ_FUSION_NODE_HPP
#define TRAJ_FUSION_NODE_HPP

// ==================== ROS消息头文件 ====================
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


// ==================== 标准库头文件 ====================
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <cmath>
#include <random>
#include <deque>
#include <numeric>
#include <mutex>
#include <string>
#include <cstdint>  // 1117新增
#include <memory>
#include <atomic> // 用于线程安全的布尔标志
#include <sstream> 
#include <unordered_map>


// ==================== 第三方库头文件 ====================
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>  // 1127新增

#include "interfaces/rtk_cgi610.h"
#include "interfaces/JointAngles.h"
#include "interfaces/SinglePoint.h"
#include "interfaces/truckVertices.h"
#include "interfaces/local_unLoadPoints.h"

class traj_fusion_node
{
    /*================================= 变量声明 =============================*/
    private:
        //ros相关
        ros::NodeHandle nh_;
        ros::Subscriber sub_rtk_;
        ros::Subscriber sub_gridmap_;
        ros::Subscriber sub_ori_traj;
        ros::Subscriber sub_ori_traj_dig;
        ros::Subscriber sub_ori_traj_unload;
        ros::Subscriber sub_ori_traj_reset;
        ros::Subscriber sub_sys_SeqAction;
        ros::Subscriber sub_excavator_state;
        ros::Subscriber sub_joints_angles;  
        ros::Subscriber sub_joints_velocity; 
        ros::Subscriber sub_ef_tip_rotation; 
        ros::Subscriber sub_rtk_data;  
        ros::Subscriber SwingTopic_sub;      
        ros::Publisher pub_refDeviceTraj_Dig;
        ros::Publisher pub_test_refDeviceTraj_Dig;
        ros::Publisher pub_swing_direction;
        ros::Subscriber sub_current_vpitch;
        ros::Publisher pub_dig_OP_level;
        ros::Subscriber sub_smolVLA_angle;
        ros::Subscriber sub_smolVLA_status;
        ros::Publisher pub_heart_value;
        ros::Subscriber sub_bucket_terrain;
        // ros::Publisher pub_bucket_target_height;
        ros::Subscriber sub_truck_location_state;
        ros::Subscriber sub_dig_point;
        ros::Publisher pub_takeover;
        ros::Subscriber sub_mpc_switch;
        ros::Subscriber sub_mpc_switch2;
        ros::Subscriber sub_ori_traj_smooth;
        ros::Subscriber sub_ori_traj_cleanup;
        ros::Subscriber sub_dig_region_angles;
        ros::Subscriber sub_clean_unload_point;
        ros::Subscriber sub_clean_dig_point;
        // ros::Publisher pub_bucket_over_truck;
        ros::Publisher pub_2stage_traj;
        ros::Subscriber sub_ori_unload_posture;
        



        //轨迹融合相关
        // bool is_dig_state;
        int excavator_state;
        double arm_angle_current = 0.0;
        double boom_angle_current = 0.0;
        double bucket_angle_current = 0.0;
        double arm_angle_target = 0.0;
        int traj_status = 10;
        int prev_seq_traj_status_ = -1;
        //  int traj_status;
        // double NQF = 5.448;

        geometry_msgs::Pose ori_traj;//原来的轨迹规划内容
        geometry_msgs::Pose traj_dig;
        geometry_msgs::Pose traj_unload;
        geometry_msgs::Pose traj_reset;
        geometry_msgs::Pose traj_smooth;
        geometry_msgs::Pose traj_cleanup;
        geometry_msgs::Pose current_traj;
        geometry_msgs::Pose act_traj;
        geometry_msgs::Pose smolVLA_traj;
        geometry_msgs::Pose model_traj;
        geometry_msgs::Pose last_traj;
        geometry_msgs::Pose stage2_traj;
        geometry_msgs::Pose traj_unload_posture;
        // double MNQ;//原轨迹规划发送的铲斗角度是MNQ角度
        double unload_boom_angle = 0.0;
        bool unload_boom_flag = true;
        int smolVLA_status = 0;

        double boom_cut_angle;
        double arm_cut_angle;
        double bucket_flat_angle;
        double act_bucket_angle;
        double bucket_offset;
  
        double arm_target_threshold;
        double act_arm_angle;
        double act_boom_angle;
        double arm_target_angle;
        bool current_angle_dif_flag = false;
        bool unload_angle_dif_flag = false;
        // 阶段4：回转越过 truck 目标角后锁存为 true（避免高速跳过 epsilon 带漏判）
        bool unload_swing_passed_truck_target_latch_ = false;
        double unload_truck_cross_prev_swing_deg_ = 0.0;
        bool unload_truck_cross_prev_valid_ = false;

        //后续改为配置值
        // double dig_OP_level;
        // double boom_adjust_range = 5;
        std::atomic<double> dig_OP_level_{0.0};  // 憋压等级 0/1，jointsCallback 写、dig_traj_adjust 读
        double boom_adjust_range;
        double angle_arm_thresh;
        // double dig_v_arm_ub = 8;
        // double dig_v_arm_lb = 1;
        double dig_v_arm_ub;
        double dig_v_arm_lb;
        double dig_joint_vel_thresh;  // 憋压判定：三关节平滑角速度均低于此阈值 (deg/s)
        double dig_op_vel_ema_alpha;  // 角速度 EMA 系数，越大响应越快 (0,1]
        double arm_velocity_current;
        double boom_velocity_current;
        double bucket_velocity_current;

        double dig_op_smoothed_boom_vel_ = 0.0;
        double dig_op_smoothed_arm_vel_ = 0.0;
        double dig_op_smoothed_bucket_vel_ = 0.0;
        double dig_op_prev_boom_ = 0.0;
        double dig_op_prev_arm_ = 0.0;
        double dig_op_prev_bucket_ = 0.0;
        bool dig_op_prev_angles_valid_ = false;
        bool dig_op_smoothed_vel_valid_ = false;
        ros::Time dig_op_last_sample_time_;
        std::mutex dig_op_mutex_;           // 保护憋压检测内部状态（回调线程 vs 主循环 reset）
        double dig_op_max_sample_dt_;       // 采样间隔超过此值视为断流，重新 seed（秒）



/*================================= wangxing =============================*/
        double last_state_ = 0; // 记录上一次状态值
        // 奖励/惩罚计分（每主循环重置）
        std::atomic<int> reward_{0};

/*=================================== 新增私有变量 =====================*/
/*================================= 挖卡协同 =============================*/
        ros::Subscriber sub_truck_rtk_data;
        ros::Subscriber sub_truck_info_data;

        ros::Publisher pub_truck_vertices;
        ros::Publisher pub_split_unLoadPoint;//划分的卸载点
        ros::Publisher pub_center_unLoadPoint;//中心卸载点
        ros::Publisher pub_center_unLoadPoint_2;//后轴中心沿航向前推的卸载点
        ros::Publisher pub_center_Point;
        ros::Publisher pub_lane_circle_intersections_;  // 车道–圆交点经纬度 PoseArray
        // int unload_count_total;//针对这台矿卡的总卸载次数
        int unload_count_total = 1;//针对这台矿卡的总卸载次数
        int unload_perpoint_count = 3;//每个点被允许的卸载次数
        std::vector<geometry_msgs::Quaternion> local_unLoadPoints;//所有卸载点

        double exc_center_W_L;
        double exc_center_W_H;
        double exc_center_W_W;

        // double exc_center_W_L = 1.98;
        // double exc_center_W_H = -0.8;
        // double exc_center_W_W = -1.21;

        double exc_bucket_W = 0.0;
        double exc_bucket_H = 0.0;
        Eigen::Affine3d truck_transform_global = Eigen::Affine3d::Identity();
        
        
        Eigen::Vector4d exc_center_enu;//挖掘机中心点

        double truck_length = 9.2;//矿卡车厢长
        double truck_width = 3.9;//矿卡车厢宽
        double truck_height = 4.1;//矿卡车厢宽
        double truck_pitch = 0.0;//矿卡俯仰角
        double truck_roll = 0.0;//矿卡横滚角
        double truck_heading = 0.0;//矿卡航向角
        // double truck_xx = 0.0;
        // double truck_yy = 0.0;
        // double truck_zz = 0.0;
        geometry_msgs::Point truck_rtk_position; //卡车主天线位置
        interfaces::truckVertices truck_vertices; // 卡车四个顶点
        // interfaces::local_unLoadPoints local_unLoadPoints; //所有卸载点
        geometry_msgs::Quaternion selectUnPoint;// 被选中的卸载点
        double unLoadCount = 3.0;//每个中心的卸载次数

        bool isGenerateVertices = false;
        // w==2 每帧刷新；w==3 经纬>0.5m 或航向>2° 才重算；其它 w 清 cache
        bool truck_dock_success_flag_ = false;
        bool truck_geometry_calc_done_for_dock_ = false;
        bool truck_geom_cache_valid_ = false;
        static constexpr double kUnloadRecalcDistM_ = 0.5;
        static constexpr double kUnloadRecalcHeadingDeg_ = 2.0;
        bool last_unload_calc_pos_valid_ = false;
        double last_unload_calc_lat_ = 0.0;
        double last_unload_calc_lon_ = 0.0;
        double last_unload_calc_heading_ = 0.0;
        geometry_msgs::Quaternion truck_unload_point_cache_;
        geometry_msgs::Quaternion truck_unload_point_cache_2;
        geometry_msgs::Quaternion truck_center_point_cache_;
        interfaces::truckVertices truck_vertices_cache_;
        Eigen::Affine3d truck_transform = Eigen::Affine3d::Identity();;
        double delta_dx = 0.0;  //主天线到矿卡中心点的偏移量 以车体坐标系表示
        double delta_dy = 0.0;
        double delta_dz = 0.0;

        double truck_swive_angle;
        std::vector<double> truck_swive_angles_vc;
        std::mutex mutex_;
        std::mutex data_mutex; 
        std_msgs::Bool excavator_rotates_clockwise_flag_;
        // 阶段4回转方向（顺时针=true，逆时针=false）
        bool swing_clockwise_4 = false;
        double delta_target_sub_current_angle_boom;

        // 提升阶段末端角度  ### 上游规划发送
        double init_angle_swing_4_;
        double init_angle_boom_4_ = 8.0;
        double target_angle_arm_3_ = -120;
        double target_angle_bucket_3_ ;
        bool stop_swing_angle_flag_ = false;
        bool bucket_is_upper_truck = false;
        bool swivel_angle_flag = true;
        double rtk_uu = 0.0;
        double rtk_alt = 0.0;
        double rtk_lat = 0.0;  // 挖机 RTK 纬度（供 MerLon / 车道圆交点）
        double rtk_lon = 0.0;  // 挖机 RTK 经度
        bool rtk_lla_valid_ = false;
        double rtk_heading = 0.0;
        double init_rtk_heading = 0.0;
        double dig_rtk_heading_angle = 0.0;
        std_msgs::Float32 swivel_angle;
        std_msgs::Float32 angular_velocity;
        geometry_msgs::PointStamped bucket_point_stamped;
        double current_angle_boom_ = 0.0;
        // 回转角相关成员必须有确定初值：会进入角度归一化，未初始化的栈垃圾值（inf/极大值）会导致归一化死循环
        double current_angle_swing_ = 0.0;
        bool current_angle_swing_valid_ = false;  // 是否收到过 /heading2swing_topic
        double current_velocity_swing_ = 0.0;
        double ref_angle_swing_ = 0.0;

        double ef_tip_rotation_x = 0;
        double ef_tip_rotation_y = 0;
        double ef_tip_rotation_z = 0;
        Eigen::Vector4d  ef_position_enu_;
        interfaces::SinglePoint bucket_enu_point;
        ros::Subscriber sub_ef_position_enu;
        ros::Publisher pub_truck_swive_angle;

        // double init_reset_swing_4_;
        Eigen::Vector4d bucket_point;
        double init_bucket_height;
        double last_percent = 0.0;
        double swing_percent = 0.0;
        double state_value = 0.0;
        double bucket_to_angle = 0.0;
        Eigen::Vector4d center_point;
        double digover_boom = 40.0;
        Eigen::Vector4d init_bucket_point;
        double unload_angle = 0.0;
        bool unload_angle_valid_ = false;  // 卸载方位角是否已由矿卡几何/搭台卸载点算出
        // double wait_angle;
        double unload_height_offset;
        double unload_forward_offset;  // 卸载点2相对后轴中心沿航向前推距离(m)
        std_msgs::Float32 truck_swive_angle_msg;
        std_msgs::Float32 truck_swive_angle_pub_msg;

        enum FaultCode {
                FAULT_0 = 0x0001,  // bit 0
                FAULT_1 = 0x0002,  // bit 1
                FAULT_2 = 0x0004,  // bit 2
                FAULT_3 = 0x0008,  // bit 3
                FAULT_4 = 0x0010,  // bit 4
                FAULT_5 = 0x0020,  // bit 5
                FAULT_6 = 0x0040,  // bit 6
                FAULT_7 = 0x0080,  // bit 7
                FAULT_8 = 0x0100,  // bit 8
                FAULT_9 = 0x0200,  // bit 9
                FAULT_10 = 0x0400, // bit 10
                FAULT_11 = 0x0800, // bit 11
                FAULT_12 = 0x1000, // bit 12
                FAULT_13 = 0x2000, // bit 13
                FAULT_14 = 0x4000, // bit 14
                FAULT_15 = 0x8000  // bit 15
        };

        uint16_t heart_value = 0;  // 高16位代表0-255的数字
        uint16_t faultMask = 0;  // 低16位故障码
        std_msgs::UInt32 heart_and_fault;
        bool model_traj_valid_ = false;
        bool act_traj_valid_ = false;  // 模型/融合轨迹是否有效（原 ACT 推理标志，现作通用有效位）

        // 状态计时：超过20s发布故障码
        ros::Time state_start_time_;
        // 挖掘阶段专用计时：仅在进入 traj_status==3 时刷新，供动臂自由期/积分使用
        ros::Time dig_stage_start_time_;
        int last_traj_status_for_timer_ = -1;
        int prev_traj_status_for_timer_ = -1; // 用于识别阶段切换来源（如1->2或4->2）
        static constexpr double STATE_TIMEOUT_SEC = 20.0;
        double dig_boom_free_sec_ = 10.0;  // 挖掘阶段动臂前多少秒不做积分干预
        double dig_boom_integrate_rate_deg_per_sec_ = 1.0;  // 超时后动臂积分速度 (deg/s)

        double bucket_terrain;//铲斗齿尖在地面之上为正值

        // 复位阶段：齿尖距地面<50cm且回转距规划目标15度内时斗杆强制垂直
        double reset_bucket_ground_dist_thresh;  // 50cm
        static constexpr double RESET_SWING_ANGLE_THRESH = 15.0;        // 15度
        static constexpr double TRUCK_TIRE_HALF_HEIGHT = 0.72;        // 半个矿卡轮胎高度
        static constexpr double ARM_VERTICAL_ANGLE = -120.0;             // 斗杆垂直角度(度)

        double boom_length;
        double arm_length;
        double bucket_length;

        // 各阶段关节角度限制（度）：reset 复位, dig 挖掘, unload 卸载
        struct StageAngleLimits {
            double swing_min, swing_max;
            double boom_min, boom_max;
            double arm_min, arm_max;
            double bucket_min, bucket_max;
        };
        StageAngleLimits limits_reset_;
        StageAngleLimits limits_dig_;
        StageAngleLimits limits_unload_;

        double truck_edge_offset;

        // 卸载阶段：回转相位 unload_swing_phase_ ∈{-1,0,+1}；发布角 = 车尾基准 + phase×360°
        double last_unload_swing_raw_deg_ = 0.0;
        bool last_unload_swing_raw_valid_ = false;
        int unload_swing_phase_ = 0;

        // 复位阶段(2)：回转单调展开；基准仅首帧写入，reset_swing_phase_ ∈{-1,0,+1}（逻辑同阶段4）
        double last_reset_swing_raw_deg_ = 0.0;
        bool last_reset_swing_raw_valid_ = false;
        double reset_swing_unwrap_offset_deg_ = 0.0;
        int reset_swing_phase_ = 0;
        // 阶段2：回转方向 swing_clockwise_2 仅在进入本阶段首周期计算一次
        bool reset_swing_direction_computed_this_entry_ = false;
        // 阶段2：起始回转角/动臂进度基准仅在进入本阶段首帧采一次（与 RGp 是否有效解耦）
        bool init_reset_phase2_captured_ = false;

        double init_reset_boom_4_;// 进入复位阶段时的动臂角度（上一阶段角度）
        // 阶段2：按回转进度重算动臂时使用
        double init_reset_swing_2_ = 0.0;
        double reset_boom_last_percent_ = 0.0;
        double reset_truck_exit_thresh;

        double init_dig_bucket_3_;
        bool last_traj_dig_valid_ = false;
        double init_dig_rtk_heading = 0.0;
        double init_dig_bucket_height = 0.0;
        double init_dig_swing_3_ = 0.0;

        double reset_swing_clear_truck_center_deg = 35;

        // 阶段3 内 stop_swing_main 只算一次；离开阶段3 后清零以便下次进入再算
        bool stop_swing_main_done_in_dig_stage_ = false;

        // 卸载阶段：距卸载回转角近时用 traj_unload 铲斗角，远时用端平角；
        bool unload_bucket_use_planner_w_ = false;
        double unload_bucket_planner_enter_deg_ = 8.0;  // 距卸载回转角 ≤ 该值(度)后切规划铲斗角，ROS 可配
        // 已切规划铲斗角后：在阈值内记录当前铲斗角，超调时发记录角
        bool unload_planner_bucket_record_valid_ = false;
        double unload_planner_bucket_recorded_deg_ = 0.0;
        // 阶段4：上一控制周期限幅后的动臂下发角；与其它阶段无关（非 last_traj）
        bool last_unload_boom_cmd_valid_ = false;
        double last_unload_boom_cmd_deg_ = 0.0;

        bool digover_boom_applied_ = false;

        // 阶段1：进入时锁定当前回转/动臂/斗杆/铲斗角，整段保持
        bool init_stage1_angles_captured_ = false;
        double init_stage1_swing_ = 0.0;
        double init_stage1_boom_ = 0.0;
        double init_stage1_arm_ = 0.0;
        double init_stage1_bucket_ = 0.0;
        std_msgs::Int8 swing_wrap_jump_msg;

        double truck_tail_offset_from_unload_deg_ = 30.0;  // 车尾目标相对卸载方位角偏置（度），ROS 可配
        double dig_point_angle_switch1 = 0.0;
        bool dig_point_angle_switch1_valid_ = false;  // 是否收到过 /Sys_RGp_SwWkAgl
        double RGp_dig_point_angle = 0.0;
        bool RGp_dig_point_angle_valid_ = false;      // 挖掘点回转角是否已就绪
        // 阶段2回转方向（顺时针=true，逆时针=false）
        bool swing_clockwise_2 = false;
        double init_reset_arm_4_ = 0.0;// 进入复位阶段时的arm角度（上一阶段角度）'
        double truck_to_center = 0.0;
        bool require_takeover = false;
        std_msgs::Bool require_takeover_flag_;
        int takeover_pub_count_ = 0;

        double  angle_dig_lb1_deg = 0.0;
        double  angle_dig_ub1_deg = 0.0;
        double  angle_dig_lb2_deg = 0.0;
        double  angle_dig_ub2_deg = 0.0;

        bool swing_clockwise_5 = false;
        int wait_swing_phase_ = 0;
        double bucket_ahead_unload_deg = 20.0;
        double relative_z;
        /** 线缆危险区回转角边界（度），与安全短弧语义一致：默认 315°→45° 经正南为小弧约 90° */
        // double angle_cable_lb_deg = 45.0;
        // double angle_cable_ub_deg = 315.0;
        // double angle_cable_deg = 0.0; 
        bool swing_clockwise_cable = false;
        /** 卸料回转与线缆回转方向一致，且沿线缆方向上 cur→线缆 早于 cur→卸载（先经线缆防碰）。由 calculateTruckVertices 刷新 */
        bool unload_swing_cable_first_flag_ = false;
        bool reset_swing_cable_first_flag_ = false;
        int takeover_pub_count_cable_reset = 0;
        int takeover_pub_count_cable_unload = 0;
        double unload_angle_mono = 0.0;
        double bucket_to_truck = 0.0;
        double truck_tail_angle = 0.0;
        // std_msgs::Int8 bucket_over_truck_msg;
        double bucket_change_rate = 0.0;
        bool boom_fall_flag = false;
        double truck_real_height = 0.0;
        double bucket_to_truck_ground = 0.0;
        bool reset_swing_direction_precomputed_in_unload_ = false;
        bool start5_angle_flag = false;//判断是3->4还是5->4
        bool start1_angle_flag = false;//判断是1->3还是2->3
        double start3_angle = 0.0;
        double boom_increment = 0.0;
        bool use_dig_status_process_model_ = false;
        double start_dig_angle = 0.0;
        double front_lane_deg = 0.0;//挖一斗等待角；亦作无矿卡RTK时车道线航向角(°)
        double rear_cable_deg = 0.0;//线缆危险区回转角
        // 无矿卡RTK：车道–圆交点（局部侧向偏移 x、回转中心圆半径 y，单位 m）
        double lane_offset_x_m_;
        double circle_radius_y_m_;
        double m_working_height = 0.0;
        double clean_unload_angle = 0.0;
        bool clean_unload_angle_valid_ = false;     // 是否收到过 /Sys_RGp_Unload
        bool mpc_switch2_flag = false;
        double clean_dig_point_angle = 0.0;
        bool clean_dig_point_angle_valid_ = false;  // 是否收到过 /clean_digPoint



        
        

        

/*================================= 函数声明 =============================*/
    public:    
        traj_fusion_node();
        ~traj_fusion_node();

        double middleAngleSmallerArcDeg(double angle_a_deg, double angle_b_deg);

        //======================= 轨迹融合主循环 ==============================
        //处理轨迹融合结果并发布（函数名沿用历史命名）
        void processACTOutput();


        // void rtkCallback(const interfaces::rtk_cgi610::ConstPtr& msg);
        // void gridmapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);

        // void oriDeviceTrajCallback(const geometry_msgs::Pose::ConstPtr& msg_oriTraj);
        void mpcSwitchCallback(const std_msgs::Bool::ConstPtr& msg);
        void mpcSwitch2Callback(const std_msgs::Bool::ConstPtr& msg);
        void oriTrajDigCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void oriTrajUnloadCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void oriTrajResetCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void oriTrajSmoothCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void oriTrajCleanupCallback(const geometry_msgs::Pose::ConstPtr& msg);

        void excavatorStateCallback(const std_msgs::Int32::ConstPtr& msg);
        void sysSeqActionCallback(const std_msgs::Float64::ConstPtr& msg); 
        void jointsCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
        void jointsVelocityCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
        void truckRTKCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void EFTipRotationTopicCallback(const geometry_msgs::Point::ConstPtr& msg);
        void ENUDataTopicCallback(const interfaces::rtk_cgi610::ConstPtr& msg);
        // void SwingTopicCallback(const geometry_msgs::Point::ConstPtr& msg); // 回转角度话题订阅
        void SwingTopicCallback(const std_msgs::Float32::ConstPtr& msg); // 回转角度话题订阅
        void smolVLAOuputCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void smolVLAStatusCallback(const std_msgs::Int8::ConstPtr& msg);
        void truckLocationCallback(const std_msgs::Int8::ConstPtr& msg);
        void digPointCallback(const geometry_msgs::Point::ConstPtr& msg);
        void digRegionAnglesCallback(const geometry_msgs::Point::ConstPtr& msg);
        void cleanUnloadPointCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
        void cleanDigPointCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
        void oriUnloadPostureCallback(const geometry_msgs::Pose::ConstPtr& msg);



        void initStatusProcess();
        // void levelStatusProcess();
        void digStatusProcess();
        void digStatusProcess_model();
        void raiseAndUnloadStausProcess();
        // void unloadStatusProcess();
        void resetStatusProcess();
        void waitStatusProcess();
        void smoothStatusProcess();
        void cleaupStatusProcess();
        

        void calculateBoomCutAngle(double arm_angle);
        void calculateArmCutAngle();
        void calculateBucketFlatAngle();
        void calculateTruckVertices();
        void publishTruckGeometryCache();
        // 无矿卡RTK：按 front_lane_deg / lane_offset_x_m_ / circle_radius_y_m_
        // 求中/左/右车道线与圆的前向交点，输出局部 (N, W)；|x|>y 时返回 false
        bool computeLaneCircleLocalIntersections(Eigen::Vector2d& p0_nw,
                                                 Eigen::Vector2d& pl_nw,
                                                 Eigen::Vector2d& pr_nw);
        // 局部 (N,W) → lat/lon/alt（经 exc_center_enu 合成 XX/YY 后 xy_to_wgs84）
        bool localNWToLLA(double N, double W, double& lat, double& lon, double& alt);
        // 三交点转经纬度；Point: x=lon, y=lat, z=alt；true=至少 P0 有效
        bool computeLaneCircleIntersectionsLLA(geometry_msgs::Point& p0_lla,
                                               geometry_msgs::Point& pl_lla,
                                               geometry_msgs::Point& pr_lla);
        void publishLaneCircleIntersections();
        // void calculateUnloadPoints();
        bool stop_swing_main();

        void dig_OP_detection();
        void dig_traj_adjust();
        void resetDigOpDetection();


        // /*=================================== 后续新增函数与变量 ===================================*/


        void run();
        void bucket_enu_callback(const interfaces::SinglePoint::ConstPtr& msg) ;
        void ef_position_enu_callback(const interfaces::SinglePoint::ConstPtr& msg);
        void truckInfoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        Eigen::Vector4d globalToLocal(const Eigen::Vector4d& global_point, const Eigen::Vector4d& local_origin);
        double calculateTargetSwing(double currentBH, double targetBH, double targetSwingAngle);
        double calculateTargetBoomBySwingPercent(double currentSwingAngle, double targetSwingAngle, double targetBoomAngle);
        void clampTrajAngles(geometry_msgs::Pose& traj, int stage);
        void addFaultCode(uint16_t& faultMask, FaultCode fault);
        void clearFaultCode(uint16_t& faultMask, FaultCode fault);
        uint32_t assembleFaultCode(uint16_t high16, uint16_t faultMask);
        bool hasFault(uint16_t faultMask);
        void bucketTerrainCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
        double normalizeAngle(double angle);
        // 轨迹发布用：在 [0,360) 的 raw 上选 360 整数倍，使相对 prev 逆时针单调增、顺时针单调减
        double unwrapTrajectorySwing(double prev_unwrapped, double raw_deg, bool clockwise);
        
        // double compute_Angle(double x1, double y1, double x2, double y2);
        // void reloadDigSwingAngleBoundsFromRosParam(double def_lb1, double def_ub1, double def_lb2, double def_ub2);
};

#endif // TRAJ_FUSION_NODE_HPP
