/*
项目          修改人       修改日期          改动
无人挖掘机     wangxing    2025/11/12    act轨迹融合框架
无人挖掘机     wuql7       2025/11/17    添加act小脑推理流程相关代码
无人挖掘机     wuql7       2025/11/27    添加模型评价reward统计并输出
无人挖掘机     wuql7       2025/11/28    在输出结果前进行反归一化
无人挖掘机     wuql7       2025/12/19    融合wangx1219的结果
无人挖掘机     wuql7       2025/12/19    融合wangx1219的结果
无人挖掘机     wuql7       2025/12/24    融合wangx1224的结果






*/

#include "traj_fusion_node.hpp"

// static void waitForDoubleParam(const ros::NodeHandle& nh,
//                                const std::string& global_name,
//                                const std::string& private_name,
//                                double& value,
//                                double default_value)
// {
//     const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(100.0);
//     while (ros::ok() && ros::WallTime::now() < deadline)
//     {
//         if (ros::param::get(global_name, value) || nh.getParam(private_name, value))
//         {
//             ROS_INFO("Loaded param %s = %.3f", global_name.c_str(), value);
//             return;
//         }
//         ros::WallDuration(0.05).sleep();
//     }

//     value = default_value;
//     ROS_WARN("Param %s not found, use default %.3f", global_name.c_str(), default_value);
// }


traj_fusion_node::traj_fusion_node():nh_("~")
{
    // 参数（可在 launch 中覆盖）
    nh_.param("/use_dig_status_process_model", use_dig_status_process_model_, false);
    nh_.param("/dig_boom_free_sec", dig_boom_free_sec_, 10.0);
    nh_.param("/dig_boom_integrate_rate_deg_per_sec", dig_boom_integrate_rate_deg_per_sec_, 1.0);
    nh_.param("/arm_target_threshold",arm_target_threshold,70.0);
    nh_.param("/bucket_offset",bucket_offset,5.0);
    nh_.param("/boom_adjust_range",boom_adjust_range,5.0);
    nh_.param("/angle_arm_thresh",angle_arm_thresh,5.0);
    nh_.param("/dig_v_arm_ub",dig_v_arm_ub,20.0);
    nh_.param("/dig_v_arm_lb",dig_v_arm_lb,10.0);
    nh_.param("/RO_L",exc_center_W_L,2.007);//qianhou
    nh_.param("/W_H",exc_center_W_H,1.204);//zuoyou
    nh_.param("/W_W",exc_center_W_W,0.778);//gaodu
    nh_.param("/exc_bucket_H",exc_bucket_H,0.5);
    nh_.param("/bucket_to_angle",bucket_to_angle,0.0);
    nh_.param("/digover_boom",digover_boom,20.0);
    nh_.param("/unload_height_offset",unload_height_offset,1.0);
    nh_.param("/unload_forward_offset", unload_forward_offset, 0.5);
    nh_.param("/QV",bucket_length,1464.4056);
    nh_.param("/CF",boom_length,5703.5758);
    nh_.param("/FQ",arm_length,2929.9387);
    nh_.param("/truck_edge_offset",truck_edge_offset,0.1);
    nh_.param("/reset_bucket_ground_dist_thresh",reset_bucket_ground_dist_thresh,0.1);
    nh_.param("/reset_truck_exit_thresh",reset_truck_exit_thresh,30.0);
    nh_.param("/unload_bucket_planner_enter_deg_", unload_bucket_planner_enter_deg_, 8.0);
    nh_.param("/truck_tail_offset_from_unload_deg_", truck_tail_offset_from_unload_deg_, 30.0);
    nh_.param("/bucket_ahead_unload_deg", bucket_ahead_unload_deg, 20.0);
    nh_.param("/relative_z", relative_z, 0.8);
    

    
    // nh_.param("/GP2mid/angle_dig_lb1_deg", angle_dig_lb1_deg, 238.0);
    // nh_.param("/GP2mid/angle_dig_ub1_deg", angle_dig_ub1_deg, 263.0);
    // nh_.param("/GP2mid/angle_dig_lb2_deg", angle_dig_lb2_deg, 373.0);
    // nh_.param("/GP2mid/angle_dig_ub2_deg", angle_dig_ub2_deg, 398.0);

    // nh_.param("/angle_cable_lb_deg", angle_cable_lb_deg, 45.0);
    // nh_.param("/angle_cable_ub_deg", angle_cable_ub_deg, 315.0);
    // nh_.param("/angle_cable_deg", angle_cable_deg, 127.0);

    nh_.param("/GP2mid/front_lane_deg", front_lane_deg, 0.0);
    nh_.param("/GP2mid/rear_cable_deg", rear_cable_deg, 0.0);
    // 无矿卡RTK车道–圆交点：航向用 front_lane_deg；侧向偏移 x、圆半径 y
    nh_.param("/lane_offset_x_m", lane_offset_x_m_, 3.0);
    nh_.param("/circle_radius_y_m", circle_radius_y_m_, 7.0);

    // waitForDoubleParam(nh_, "/GP2mid/front_lane_deg", "GP2mid/front_lane_deg", front_lane_deg, 0.0);
    // waitForDoubleParam(nh_, "/GP2mid/rear_cable_deg", "GP2mid/rear_cable_deg", rear_cable_deg, 0.0);





    // 复位阶段关节角度限制（度）
    nh_.param("/reset_swing_min", limits_reset_.swing_min, 0.0);
    nh_.param("/reset_swing_max", limits_reset_.swing_max, 360.0);
    nh_.param("/reset_boom_min", limits_reset_.boom_min, -30.0);
    nh_.param("/reset_boom_max", limits_reset_.boom_max, 90.0);
    nh_.param("/reset_arm_min", limits_reset_.arm_min, -180.0);
    nh_.param("/reset_arm_max", limits_reset_.arm_max, 0.0);
    nh_.param("/reset_bucket_min", limits_reset_.bucket_min, -180.0);
    nh_.param("/reset_bucket_max", limits_reset_.bucket_max, 90.0);
    // 挖掘阶段关节角度限制（度）
    nh_.param("/dig_swing_min", limits_dig_.swing_min, 0.0);
    nh_.param("/dig_swing_max", limits_dig_.swing_max, 360.0);
    nh_.param("/dig_boom_min", limits_dig_.boom_min, -30.0);
    nh_.param("/dig_boom_max", limits_dig_.boom_max, 90.0);
    nh_.param("/dig_arm_min", limits_dig_.arm_min, -180.0);
    nh_.param("/dig_arm_max", limits_dig_.arm_max, 0.0);
    nh_.param("/dig_bucket_min", limits_dig_.bucket_min, -180.0);
    nh_.param("/dig_bucket_max", limits_dig_.bucket_max, 90.0);
    // 卸载阶段关节角度限制（度）
    nh_.param("/unload_swing_min", limits_unload_.swing_min, 0.0);
    nh_.param("/unload_swing_max", limits_unload_.swing_max, 360.0);
    nh_.param("/unload_boom_min", limits_unload_.boom_min, -30.0);
    nh_.param("/unload_boom_max", limits_unload_.boom_max, 90.0);
    nh_.param("/unload_arm_min", limits_unload_.arm_min, -180.0);
    nh_.param("/unload_arm_max", limits_unload_.arm_max, 0.0);
    nh_.param("/unload_bucket_min", limits_unload_.bucket_min, -180.0);
    nh_.param("/unload_bucket_max", limits_unload_.bucket_max, 90.0);

    sub_dig_point = nh_.subscribe<geometry_msgs::Point>("/Sys_RGp_SwWkAgl", 1, &traj_fusion_node::digPointCallback, this);
    sub_clean_dig_point = nh_.subscribe<geometry_msgs::Quaternion>("/clean_digPoint", 1, &traj_fusion_node::cleanDigPointCallback, this);
    sub_dig_region_angles = nh_.subscribe<geometry_msgs::Point>("/dig_region_reference_angles", 1, &traj_fusion_node::digRegionAnglesCallback, this);

    sub_clean_unload_point = nh_.subscribe<geometry_msgs::Quaternion>("/Sys_RGp_Unload", 1, &traj_fusion_node::cleanUnloadPointCallback, this);
    // sub_rtk_ = nh_.subscribe("/rtk_can_msg", 10, &traj_fusion_node::rtkCallback, this);
    // sub_gridmap_ = nh_.subscribe("/excavation_gridmap", 10, &traj_fusion_node::gridmapCallback, this);
    // sub_ori_traj_dig = nh_.subscribe<geometry_msgs::Pose>("/OriDeviceTraj_Dig", 1, &traj_fusion_node::oriDeviceTrajCallback, this);
    
    // sub_ori_traj = nh_.subscribe<geometry_msgs::Pose>("/Sys_SCo_RefDeviceTraj", 1, &traj_fusion_node::oriDeviceTrajCallback, this);


    
    // SwingTopic_sub = nh_.subscribe("/Swing_topic", 1, &traj_fusion_node::SwingTopicCallback, this);
    SwingTopic_sub = nh_.subscribe<std_msgs::Float32>("/heading2swing_topic", 1, &traj_fusion_node::SwingTopicCallback, this);
    sub_ori_traj_dig = nh_.subscribe<geometry_msgs::Pose>("/RefDeviceTraj_Dig", 1, &traj_fusion_node::oriTrajDigCallback, this);
    sub_ori_traj_unload = nh_.subscribe<geometry_msgs::Pose>("/RefDeviceTraj_Unload", 1, &traj_fusion_node::oriTrajUnloadCallback, this);
    sub_ori_unload_posture = nh_.subscribe<geometry_msgs::Pose>("/Sys_RUn_UnloadEndAngle", 1, &traj_fusion_node::oriUnloadPostureCallback, this);
    
    sub_ori_traj_reset = nh_.subscribe<geometry_msgs::Pose>("/RefDeviceTraj_Reset", 1, &traj_fusion_node::oriTrajResetCallback, this);
    sub_ori_traj_smooth = nh_.subscribe<geometry_msgs::Pose>("/RefDeviceTraj_Smooth", 1, &traj_fusion_node::oriTrajSmoothCallback, this);
    sub_ori_traj_cleanup = nh_.subscribe<geometry_msgs::Pose>("/RefDeviceTraj_CleanUp", 1, &traj_fusion_node::oriTrajCleanupCallback, this);
    sub_sys_SeqAction = nh_.subscribe<std_msgs::Float64>("/Sys_SeqAction", 1, &traj_fusion_node::sysSeqActionCallback, this);
        // sub_joints_angles = nh_.subscribe<interfaces::JointAngles>("/joints_angles", 1, &traj_fusion_node::jointsCallback, this); //弧度
    sub_joints_angles = nh_.subscribe<geometry_msgs::Quaternion>("/joints_angle", 1, &traj_fusion_node::jointsCallback, this); //角度
    // sub_joints_velocity = nh_.subscribe<geometry_msgs::Quaternion>("/joints_velocity", 1, &traj_fusion_node::jointsVelocityCallback, this);
    sub_truck_rtk_data = nh_.subscribe<geometry_msgs::PoseStamped>("/truck_pose_topic", 1, &traj_fusion_node::truckRTKCallback, this);
    sub_truck_location_state = nh_.subscribe<std_msgs::Int8>("/truck_location_xyz_topic", 1, &traj_fusion_node::truckLocationCallback, this);
    sub_rtk_data = nh_.subscribe<interfaces::rtk_cgi610>("/rtk_can_msg", 1, &traj_fusion_node::ENUDataTopicCallback, this);
    sub_ef_position_enu = nh_.subscribe<interfaces::SinglePoint>("/ef_tip_enu_1",1,&traj_fusion_node::ef_position_enu_callback,this);

    
    // sub_excavator_state_flag = nh_.subscribe<std_msgs::Bool>("/excavator_state_flag", 1, &traj_fusion_node::excavatorStateFlagCallback, this);//是否撑车
    sub_ef_tip_rotation = nh_.subscribe<geometry_msgs::Point>("/RealBktPosXYZ",1,&traj_fusion_node::EFTipRotationTopicCallback,this);


    sub_truck_info_data = nh_.subscribe("/truck_info_topic", 1, &traj_fusion_node::truckInfoCallback, this);
    // sub_current_vpitch = nh_.subscribe("/joints_velocity", 1000, &traj_fusion_node::jointsVelocityCallback, this); 

    sub_bucket_terrain = nh_.subscribe<geometry_msgs::PointStamped>("/bucket_terrain_delta_position", 1, &traj_fusion_node::bucketTerrainCallback, this);

    sub_smolVLA_angle = nh_.subscribe<geometry_msgs::Pose >("/smolVLA_output_angle",1,&traj_fusion_node::smolVLAOuputCallback,this);
    sub_smolVLA_status = nh_.subscribe<std_msgs::Int8>("/smolVLA_status_check",1,&traj_fusion_node::smolVLAStatusCallback,this);
    sub_mpc_switch = nh_.subscribe<std_msgs::Bool>("/Mpc_Switch", 1, &traj_fusion_node::mpcSwitchCallback, this);
    sub_mpc_switch2 = nh_.subscribe<std_msgs::Bool>("/Mpc_Switch2", 1, &traj_fusion_node::mpcSwitch2Callback, this);
    // sub_excavator_state = nh_.subscribe<std_msgs::Int32>("/excavator_state", 1, &traj_fusion_node::excavatorStateCallback, this);
    
    // pub_refDeviceTraj_Dig = nh_.advertise<geometry_msgs::Pose>("/RefDeviceTraj_Dig", 1);
    pub_refDeviceTraj_Dig = nh_.advertise<geometry_msgs::Pose>("/Sys_SCo_Traj", 1);
    // pub_test_refDeviceTraj_Dig = nh_.advertise<geometry_msgs::Pose>("/Test_Sys_SCo_Traj", 1);
    pub_swing_direction = nh_.advertise<std_msgs::Bool>("/swing_clockwise_flag", 1);
    // sub_bucket_enu = nh_.subscribe("/ef_tip_enu_1", 1000, &traj_fusion_node::bucket_enu_callback,this);

    pub_truck_vertices = nh_.advertise<interfaces::truckVertices>("/truck_vertices_topic", 1);
    pub_center_unLoadPoint = nh_.advertise<geometry_msgs::Quaternion>("/truck_center_unloadpoint", 1);
    pub_center_unLoadPoint_2 = nh_.advertise<geometry_msgs::Quaternion>("/truck_center_unloadpoint_2", 1);
    pub_center_Point = nh_.advertise<geometry_msgs::Quaternion>("/truck_center_point", 1);
    pub_lane_circle_intersections_ = nh_.advertise<geometry_msgs::PoseArray>("/lane_circle_intersections", 1);
        // pub_unLoadPoints = nh_.advertise<interfaces::local_unLoadPoints>("/truck_unloading_points", 10);
    // pub_split_unLoadPoint = nh_.advertise<geometry_msgs::Quaternion>("/truck_split_unloadpoint", 1);

    pub_truck_swive_angle = nh_.advertise<std_msgs::Float32>("/truck_swive_angle",1);

    // pub_dig_OP_level = nh_.advertise<std_msgs::Float64>("/dig_OP_level", 1);
    // pub_bucket_over_truck = nh_.advertise<std_msgs::Int8>("/bucket_overstep_truck", 1);



    pub_heart_value = nh_.advertise<std_msgs::UInt32>("/trajfusionSafety_topic", 1);


    // 发布人工接管标志话题
    pub_takeover = nh_.advertise<std_msgs::Bool>("/fault_require_takeover", 1);

    // 发布2阶段轨迹发布话题
    pub_2stage_traj = nh_.advertise<geometry_msgs::Pose>("/2stage_traj_topic", 1);


    // pub_bucket_target_height = nh_.advertise<std_msgs::Float32>("/bucket_target_height", 1);

    
}

traj_fusion_node::~traj_fusion_node()
{
}





void traj_fusion_node::run() 
{

    // std::cout<<"***********ACT begin:    "<<std::endl;

    ros::Rate rate(10); // 设置循环频率为 10Hz
    while (ros::ok())
    {
        //heart
        heart_value++;
        if(heart_value >= 256)
        {
            heart_value = 0;
        }

        // addFaultCode(faultMask, FAULT_5);

        heart_and_fault.data = assembleFaultCode(heart_value, faultMask);
        // std::cout<<"[[traj_fusion_node]]heart_and_fault:    "<<heart_and_fault.data<<std::endl;

        pub_heart_value.publish(heart_and_fault);

        // 轨迹融合主循环：输出控制所需角度
        processACTOutput();
        // 本轮惩罚结果输出并重置
        int reward_snapshot = reward_.load();
        // ROS_INFO_THROTTLE(1.0, "[REWARD LOOP] penalty_sum=%d (next epoch to 0)", reward_snapshot);
        reward_.store(0);

        rate.sleep();
    }
}

// void traj_fusion_node::reloadDigSwingAngleBoundsFromRosParam(
//     double def_lb1, double def_ub1, double def_lb2, double def_ub2)
// {
//     nh_.getParam("/GP2mid/angle_dig_lb1_deg", angle_dig_lb1_deg);
//     nh_.getParam("/GP2mid/angle_dig_ub1_deg", angle_dig_ub1_deg);
//     nh_.getParam("/GP2mid/angle_dig_lb2_deg", angle_dig_lb2_deg);
//     nh_.getParam("/GP2mid/angle_dig_ub2_deg", angle_dig_ub2_deg);
// }