#include <ros/ros.h>
#include <string>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <limits>
#include <fstream>
#include <sstream>
#include <map>
#include <cstdio>
#include <GP2/GP.h>

uint32_t line_strip = visualization_msgs::Marker::LINE_STRIP;

namespace
{
bool writeAngleParamsToYaml(const std::string& path, const std::map<std::string, double>& values)
{
    if (path.empty())
        return false;

    std::ifstream input(path);
    if (!input.is_open())
        return false;

    std::vector<std::string> lines;
    std::map<std::string, bool> found;
    for (const auto& value : values)
        found[value.first] = false;

    std::string line;
    while (std::getline(input, line))
    {
        const std::size_t first = line.find_first_not_of(" \t");
        if (first != std::string::npos)
        {
            for (const auto& value : values)
            {
                const std::string token = value.first + ":";
                if (line.compare(first, token.size(), token) != 0)
                    continue;

                const std::size_t comment = line.find('#', first + token.size());
                std::ostringstream number;
                number << std::fixed << std::setprecision(6) << value.second;
                std::string replacement = line.substr(0, first) + token + " " + number.str();
                if (comment != std::string::npos)
                    replacement += " " + line.substr(comment);
                line = replacement;
                found[value.first] = true;
                break;
            }
        }
        lines.push_back(line);
    }
    input.close();

    // Append any keys not found in the file so that new params can be added
    for (const auto& item : found)
    {
        if (!item.second)
        {
            const std::string token = item.first + ":";
            std::ostringstream number;
            number << std::fixed << std::setprecision(6) << values.at(item.first);
            std::string line_to_append = token + " " + number.str();
            lines.push_back(line_to_append);
        }
    }

    const std::string temporary_path = path + ".tmp";
    std::ofstream output(temporary_path, std::ios::trunc);
    if (!output.is_open())
        return false;

    for (const auto& output_line : lines)
        output << output_line << '\n';
    output.close();
    if (!output)
    {
        std::remove(temporary_path.c_str());
        return false;
    }

    if (std::rename(temporary_path.c_str(), path.c_str()) != 0)
    {
        std::remove(temporary_path.c_str());
        return false;
    }
    return true;
}
}

#pragma region plan flag
void GP::planFlag_callback(const std_msgs::Float32::ConstPtr& msg)          
// two ways of activation: 1. detect a signal increase, wait for scan time, then calculate
//                         2. detect a signal decrease, immediately calculate
{                                                                           
    plan_flag = msg->data;
    msg_update();
    
    #ifndef DEBUG_MODE
        signal = (plan_flag == 1 || (boom_pitch >= activate_agl && plan_flag == 2)) ? 1 : 0;
    
        if (signal > signal_)       // detect a increase of signal value 触发延迟计算，等待scan_time后计算
        {   
            flag_calfin = false;
            double scan_time = (plan_flag == 1) ? 1 : max_scan_time;
            FlagCal1_tmer = nh.createTimer(ros::Duration(scan_time), &GP::FlagCal1_callback, this, true);//定时器结束后planFlag_callbacak()触发时，传入flag_cal1=true
        }
        
        if (signal_ > signal && !flag_calfin)        // detect a decrease of signal value 信号关闭时，信号激活触发的延迟计算还没有计算完的话，立即触发计算；若计算完则不触发立即计算
        {
            flag_cal2 = true;
        }

        if (flag_cal1 || flag_cal2) // 延迟计算或立即计算
        {
            flag_cal1 = false;
            flag_cal2 = false;
            FlagCal1_tmer.stop();
            ros::Time start = ros::Time::now();
            calculate();
            calculate_counter_++;
            msg_write();
            ros::Time end = ros::Time::now();
            ros::Duration dur = end - start;
            time_elapsed = dur.toSec();
            msg_print();
            ResetMsg_tmer = nh.createTimer(ros::Duration(reset_delay),&GP::ResetMsg_callback,this,true);
            flag_calfin = true;
        }
    #endif
    msg_publish();
    signal_ = signal;
}
#pragma endregion

#pragma region callback func
void GP::sysSeq_callback(const std_msgs::Float32::ConstPtr& msg)
{
    sys_seq = msg->data;
}


void GP::mpcSwitch_callback(const std_msgs::Bool::ConstPtr& msg)
{
    // One message controls both switches:
    // - mpc_switch_interp (default false)
    // - mpc_switch_dig (default true)
    mpc_switch_interp = msg->data;
    mpc_switch_dig = msg->data;
}

void GP::mpc2Switch_callback(const std_msgs::Bool::ConstPtr& msg)
{
    const bool in = msg && msg->data;
    ROS_INFO("mpc2_switch -> %s", in ? "true" : "false");
    // Rising edge: compute once
    if (in && !mpc2_switch_prev)
    {
        mpc2_switch = true;
        unload_solution_valid = false;
        unload_compute_pending = true;
    }
    else if (!in)
    {
        mpc2_switch = false;
        unload_solution_valid = false;
        unload_compute_pending = false;
    }
    mpc2_switch_prev = in;
}

void GP::deviceRef_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    swing_joint_ref[0] = deg2rad(msg->orientation.x);
    swing_joint_ref[1] = deg2rad(msg->orientation.y);
    swing_joint_ref[2] = deg2rad(msg->orientation.z);
    swing_joint_ref[3] = deg2rad(msg->orientation.w);
}

void GP::groundDectection_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (use_rtk_ground_def)
        return;

    const auto& ori=msg->pose.orientation;
    if (ori.z==0 && sys_seq!=0)
    {
        ground_coef.assign({0,0,1,ground_def});
        ROS_WARN("Ground Dectection is error! Use default value. \n");
    }            
    else 
        ground_coef.assign({ori.x,ori.y,ori.z,ori.w-ori.z*ground_setting}); 
        
}

void GP::rtkAlt_callback(const interfaces::rtk_cgi610::ConstPtr& msg)
{
    if (!use_rtk_ground_def)
        return;

    rtk_alt = msg->alt;
    swing_center_alt = rtk_alt - rtk_to_swing_center_alt_offset;
    swing_center_to_ground_rel_height = swing_center_alt - ground_protect_alt;

    const double ground_def_min = base2ground + boom_pos[2];
    const double ground_def_raw = swing_center_to_ground_rel_height + boom_pos[2];
    ground_def = ground_def_raw;

    const double band = std::abs(rtk_ground_def_band);
    if (std::abs(ground_def_raw - rtk_ground_def_ref) > band)
    {
        ROS_WARN_THROTTLE(1.0, "rtkAlt_callback(): ground_def_raw %.3f out of [ref %.3f +/- %.3f], use ref.", ground_def_raw, rtk_ground_def_ref, band);
        ground_def = rtk_ground_def_ref;
    }

    if (ground_def < ground_def_min)
    {
        ROS_WARN_THROTTLE(1.0, "rtkAlt_callback(): ground_def %.3f < ground_def_min (base2ground+boom_pos[2]) %.3f, clamped.", ground_def, ground_def_min);
        ground_def = ground_def_min;
    }
    ground_coef.assign({0, 0, 1, ground_def});
}

void GP::autoSetDigRegion_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    //std::cout << "mpc_swintch = " << mpc_switch_dig << std::endl;// 在任何处理前先检查 Mpc_Switch 必须为 false
    if (mpc_switch_dig)
    {
        // 若 MPC 打开则不触发，直接复位等待下一次 1->2->0
        auto_dig_region_trigger_state = 0;
        return;
    }
    // State machine on position.y: expect 1 -> 2 -> 0 in order
    // States: 0 = idle; 1 = saw 1; 2 = saw 1 then 2; 3 = ready (saw 1->2->0)
    const double y = msg->position.y;
    if (auto_dig_region_trigger_state == 0)
    {
        if (std::abs(y - 1.0) < 1e-6) auto_dig_region_trigger_state = 1;
        else auto_dig_region_trigger_state = 0;
        return;
    }
    else if (auto_dig_region_trigger_state == 1)
    {
        if (std::abs(y - 2.0) < 1e-6) auto_dig_region_trigger_state = 2;
        else if (std::abs(y - 1.0) < 1e-6) {/* stay */}
        else auto_dig_region_trigger_state = 0; // reset on unexpected value
        return;
    }
    else if (auto_dig_region_trigger_state == 2)
    {
        if (std::abs(y - 0.0) < 1e-6) auto_dig_region_trigger_state = 3; // ready to trigger
        else if (std::abs(y - 2.0) < 1e-6) {/* stay */}
        else auto_dig_region_trigger_state = 0; // reset on unexpected value
    }
    
    if (auto_dig_region_trigger_state != 3)
        return;
    // reset state and continue to perform setting
    auto_dig_region_trigger_state = 0;

    const auto publish_result = [&](bool success)
    {
        std_msgs::Bool result;
        result.data = success;
        autoSetDigRegionResult_publisher.publish(result);
    };

    if (!std::isfinite(real_swing))
    {
        ROS_ERROR("autoSetDigRegion_callback(): no valid swing heading received.");
        publish_result(false);
        return;
    }

    const double half_width = std::abs(auto_dig_region_half_width_deg);
    const double near_offset = std::abs(auto_dig_region_near_offset_deg);
    const double far_offset = std::abs(auto_dig_region_far_offset_deg);
    if (!std::isfinite(half_width) || !std::isfinite(near_offset) || !std::isfinite(far_offset) ||
        half_width <= 0.0 || half_width > 180.0 || far_offset <= near_offset || far_offset > half_width)
    {
        ROS_ERROR("autoSetDigRegion_callback(): invalid offsets: half_width=%.3f, near=%.3f, far=%.3f.",
                  half_width, near_offset, far_offset);
        publish_result(false);
        return;
    }

    const auto wrap_deg = [](double angle)
    {
        angle = std::fmod(angle, 360.0);
        return angle < 0.0 ? angle + 360.0 : angle;
    };
    const double heading = wrap_deg(rad2deg(real_swing));
    const double angle_lb_deg = wrap_deg(heading - half_width);
    double angle_ub_deg = wrap_deg(heading + half_width);
    if (angle_ub_deg < angle_lb_deg)
        angle_ub_deg += 360.0;

    const double angle_dig_lb1_deg = angle_lb_deg + near_offset;
    const double angle_dig_ub1_deg = angle_lb_deg + far_offset;
    const double angle_dig_lb2_deg = angle_ub_deg - far_offset;
    const double angle_dig_ub2_deg = angle_ub_deg - near_offset;

    const double front_lane_deg = heading;
    const double rear_cable_deg = wrap_deg(heading + 180.0);

    const std::map<std::string, double> yaml_values = {
        {"angle_lb_deg", angle_lb_deg},
        {"angle_ub_deg", angle_ub_deg},
        {"angle_dig_lb1_deg", angle_dig_lb1_deg},
        {"angle_dig_ub1_deg", angle_dig_ub1_deg},
        {"angle_dig_lb2_deg", angle_dig_lb2_deg},
        {"angle_dig_ub2_deg", angle_dig_ub2_deg},
        {"front_lane_deg", front_lane_deg},
        {"rear_cable_deg", rear_cable_deg}
    };

    if (!writeAngleParamsToYaml(auto_dig_region_yaml_path, yaml_values))
    {
        ROS_ERROR("autoSetDigRegion_callback(): failed to update YAML file '%s'.", auto_dig_region_yaml_path.c_str());
        publish_result(false);
        return;
    }

    angle_lb_setting = deg2rad(angle_lb_deg);
    angle_ub_setting = deg2rad(angle_ub_deg);
    angle_dig_lb1_setting = deg2rad(angle_dig_lb1_deg);
    angle_dig_ub1_setting = deg2rad(angle_dig_ub1_deg);
    angle_dig_lb2_setting = deg2rad(angle_dig_lb2_deg);
    angle_dig_ub2_setting = deg2rad(angle_dig_ub2_deg);

    for (const auto& value : yaml_values)
        nh.setParam("GP2mid/" + value.first, value.second);

    dig_region_ref_angles_msg.x = front_lane_deg;
    dig_region_ref_angles_msg.y = rear_cable_deg;
    dig_region_ref_angles_msg.z = 0.0;
    dig_region_ref_angles_publish_count = 100;

    ROS_INFO("autoSetDigRegion_callback(): heading=%.3f deg, front_lane=%.3f, rear_cable=%.3f, global=[%.3f, %.3f], region1=[%.3f, %.3f], region2=[%.3f, %.3f].",
             heading,
             yaml_values.at("front_lane_deg"), yaml_values.at("rear_cable_deg"),
             yaml_values.at("angle_lb_deg"), yaml_values.at("angle_ub_deg"),
             yaml_values.at("angle_dig_lb1_deg"), yaml_values.at("angle_dig_ub1_deg"),
             yaml_values.at("angle_dig_lb2_deg"), yaml_values.at("angle_dig_ub2_deg"));
    publish_result(true);
}

void GP::gridMap_callback(const grid_map_msgs::GridMap::ConstPtr& gridmsg)
{        
    if (gridconverter.fromMessage(*gridmsg,map)){
        elevmat = std::move(map.get("elevation").transpose().colwise().reverse().rowwise().reverse());
        labelmat = std::move(map.get("label").transpose().colwise().reverse().rowwise().reverse());
        resolution = map.getResolution();    
        ori_coord_x = gridmsg->info.pose.position.x;
        ori_coord_y = gridmsg->info.pose.position.y;
        lengthX = gridmsg->info.length_x;
        lengthY = gridmsg->info.length_y;
    }           
    else
    {
        ROS_ERROR("Gridmap: Reading Error!");
        gridexist_errorflag = 1;
    }
    #ifdef DEBUG_MODE
        if (flag_cal1)
        {
            flag_cal1 = false;
            FlagCal1_tmer = nh.createTimer(ros::Duration(3.0), &GP::FlagCal1_callback, this, true);
            ros::Time start = ros::Time::now();
            calculate();
            msg_write();
            ros::Time end = ros::Time::now();
            ros::Duration dur = end - start;
            time_elapsed = dur.toSec();
            msg_print();
        }
        msg_publish();
    #endif
}

void GP::jointsAngle_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    boom_pitch = deg2rad(msg->x);
    arm_pitch = deg2rad(msg->y);
    bkt_pitch = deg2rad(msg->z);
}

// void GP::armPitch_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
// {
//     arm_pitch = deg2rad(msg->x);
// }

// void GP::bktPitch_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
// {
//     bkt_pitch = deg2rad(msg->x);
// } 

void GP::truckVertices_callback(const interfaces::truckVertices::ConstPtr& msg)
{
    const auto& p1 = msg->points[0];  
    const auto& p2 = msg->points[1];  
    const auto& p3 = msg->points[2];  
    const auto& p4 = msg->points[3]; 

    double angle1 = std::atan2(p1.y, p1.x);
    angle1 = (angle1 < 0) ? angle1 + 2 * M_PI : angle1;
    //angle11角度为angle1增加M_PI，超出2*M_PI范围则减2*M_PI，angle11为挖掘坐标系角度，与传进来的angle1有偏差
    double angle11 = angle1 + M_PI;
    angle11 = (angle11 >= 2*M_PI) ? angle11 - 2 * M_PI : angle11;

    double angle2 = std::atan2(p2.y, p2.x);
    angle2 = (angle2 < 0) ? angle2 + 2 * M_PI : angle2;
    double angle22 = angle2 + M_PI;
    angle22 = (angle22 >= 2*M_PI) ? angle22 - 2 * M_PI : angle22;

    double angle3 = std::atan2(p3.y, p3.x);
    angle3 = (angle3 < 0) ? angle3 + 2 * M_PI : angle3;
    double angle33 = angle3 + M_PI;
    angle33 = (angle33 >= 2*M_PI) ? angle33 - 2 * M_PI : angle33;

    double angle4 = std::atan2(p4.y, p4.x);
    angle4 = (angle4 < 0) ? angle4 + 2 * M_PI : angle4;
    double angle44 = angle4 + M_PI;
    angle44 = (angle44 >= 2*M_PI) ? angle44 - 2 * M_PI : angle44;

    double max_angle = std::max({angle11, angle22, angle33, angle44});
    double min_angle = std::min({angle11, angle22, angle33, angle44});

    std::vector<double> angles = {angle11, angle22, angle33, angle44};
    if (max_angle - min_angle >= M_PI)
    {
        // 发生了横跨，需要特殊处理
        // 找出在 [π, 2π) 范围内的最小角度（最接近 π 的角度）
        double min_in_upper = 2 * M_PI;  // 初始化为最大值
        for (double angle : angles) {
            if (angle >= M_PI && angle < min_in_upper) {
                min_in_upper = angle;
            }
        }
        // 如果没找到，说明所有角度都在 [0, π) 范围内，这不应该发生
        if (min_in_upper == 2 * M_PI) {
            min_in_upper = max_angle;  // 回退到原逻辑
        }
        truck_angle_back = min_in_upper;
        
        // 找出在 [0, π) 范围内的最大角度（最接近 π 的角度）
        double max_in_lower = 0.0;  // 初始化为最小值
        for (double angle : angles) {
            if (angle < M_PI && angle > max_in_lower) {
                max_in_lower = angle;
            }
        }
        // 如果没找到，说明所有角度都在 [π, 2π) 范围内，这不应该发生
        if (max_in_lower == 0.0) {
            max_in_lower = min_angle;  // 回退到原逻辑
        }
        truck_angle_front = max_in_lower;
    }
    else
    {
        // 没有横跨，使用原来的逻辑
        
        truck_angle_front = min_angle;
        truck_angle_back = max_angle;
    }
}

void GP::truckCenter_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    double x = msg->x;
    double y = msg->y;
    double angle = std::atan2(y,x);
    angle = (angle < 0) ? angle + 2 * M_PI : angle;
    double angle1 = angle + M_PI;
    angle1 = (angle1 >= 2*M_PI) ? angle1 - 2 * M_PI : angle1;
    truck_angle_mid = angle1;
}

void GP::platformPitch_callback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    platform_pitch = deg2rad(msg->x);
    platform_roll = 0;
}

void GP::swing_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    real_swing = deg2rad(msg->x);
}

void GP::truckPose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    const double alt = msg->pose.position.z;
    if (!std::isfinite(alt))
        return;

    ground_protect_alt = alt - truck_pose_ground_protect_alt_offset;
    ground_protect_alt_overridden_by_truck_pose = true;

    if (std::isfinite(swing_center_alt))
    {
        swing_center_to_ground_rel_height = swing_center_alt - ground_protect_alt;
    }
}

void GP::ResetMsg_callback(const ros::TimerEvent&)
{
    msg_reset();
}

void GP::FlagCal1_callback(const ros::TimerEvent&)
{
    flag_cal1 = true;
}
#pragma endregion

#pragma region GUI callback
void GP::D0_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    track_prot=msg->y;
    N_fans=std::floor(msg->z);
    if (N_fans<=0)
        ROS_ERROR("Incorrect N_fans input!");
}

void GP::E0_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    ground_crash_prot=msg->x;
    max_depth=msg->y;
    ground_setting=msg->z;
}

void GP::F0_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    angle_ub_setting=std::max(deg2rad(msg->x),deg2rad(msg->y));
    angle_lb_setting=std::min(deg2rad(msg->x),deg2rad(msg->y));
    angle_offset=deg2rad(msg->z);
}

void GP::F6_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    angle_dig_ub1_setting=std::max(deg2rad(msg->x),deg2rad(msg->y));
    angle_dig_lb1_setting=std::min(deg2rad(msg->x),deg2rad(msg->y));
    
    angle_dig_ub1_setting=std::min(angle_dig_ub1_setting , angle_ub_setting);
    angle_dig_lb1_setting=std::max(angle_dig_lb1_setting , angle_lb_setting);

}

void GP::F7_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    angle_dig_ub2_setting=std::max(deg2rad(msg->x),deg2rad(msg->y));
    angle_dig_lb2_setting=std::min(deg2rad(msg->x),deg2rad(msg->y));
    
    angle_dig_ub2_setting=std::min(angle_dig_ub2_setting , angle_ub_setting);
    angle_dig_lb2_setting=std::max(angle_dig_lb2_setting , angle_lb_setting);

}

void GP::D1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    target_ub=msg->x;
    target_lb=msg->y;
    if (target_ub<target_lb)
        ROS_ERROR("Incorrect target volume input!");
    edge_extend=msg->z;
}

void GP::E1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    extend_angle=deg2rad(msg->x);
    expnt=msg->y;
    diggable_threshold=msg->z;
}

void GP::F1_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    digx_min = std::min(std::max(r_limit[0],msg->x),r_limit[1]);
    turn_point = msg->z;
}

void GP::O4_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    l_QV=msg->z;
}

#pragma endregion

#pragma region msg init
void GP::initMsg()
{
    // visualization msg initialize
    // dig point
    GPmarker_msg.header.frame_id = "base";
    GPmarker_msg.header.stamp = ros::Time::now();
    GPmarker_msg.ns = "GPpoint";
    GPmarker_msg.id = 0;
    GPmarker_msg.type = visualization_msgs::Marker::SPHERE;
    GPmarker_msg.action = visualization_msgs::Marker::ADD;
    GPmarker_msg.pose.position.x = 0;
    GPmarker_msg.pose.position.y = 0;
    GPmarker_msg.pose.position.z = 0;
    GPmarker_msg.pose.orientation.w = 1.0;
    GPmarker_msg.scale.x = 0.3;
    GPmarker_msg.scale.y = 0.3;
    GPmarker_msg.scale.z = 0.3;
    GPmarker_msg.color.r = 1.0;
    GPmarker_msg.color.g = 1.0;
    GPmarker_msg.color.b = 1.0;
    GPmarker_msg.color.a = 1.0;

    // device pos
    GPdevice_msg.header.frame_id = "base";
    GPdevice_msg.header.stamp = ros::Time::now();
    GPdevice_msg.ns = "GPdevice";
    GPdevice_msg.id = 1;
    GPdevice_msg.type = line_strip;
    GPdevice_msg.action = visualization_msgs::Marker::ADD;
    GPdevice_msg.pose.orientation.w = 1.0;
    GPdevice_msg.scale.x = 0.1;
    GPdevice_msg.scale.y = 0.1;
    GPdevice_msg.scale.z = 0.1;
    GPdevice_msg.color.r = 1.0;
    GPdevice_msg.color.g = 1.0;
    GPdevice_msg.color.b = 1.0;
    GPdevice_msg.color.a = 1.0;

    // digging range
    GPrange_msg.header.frame_id = "base";
    GPrange_msg.header.stamp = ros::Time::now();
    GPrange_msg.ns = "GP_range";
    GPrange_msg.id = 2;
    GPrange_msg.type = line_strip;
    GPrange_msg.action = visualization_msgs::Marker::ADD;
    GPrange_msg.pose.orientation.w = 1.0;
    GPrange_msg.scale.x = 0.3;
    GPrange_msg.scale.y = 0.3;
    GPrange_msg.scale.z = 0.3;
    GPrange_msg.color.r = 0.0;
    GPrange_msg.color.g = 0.0;
    GPrange_msg.color.b = 1.0;
    GPrange_msg.color.a = 1.0;

    // digging region 1 (configured)
    GPdigRange1_msg.header.frame_id = "base";
    GPdigRange1_msg.header.stamp = ros::Time::now();
    GPdigRange1_msg.ns = "GP_dig_region1";
    GPdigRange1_msg.id = 8;
    GPdigRange1_msg.type = line_strip;
    GPdigRange1_msg.action = visualization_msgs::Marker::ADD;
    GPdigRange1_msg.pose.orientation.w = 1.0;
    GPdigRange1_msg.scale.x = 0.3;
    GPdigRange1_msg.scale.y = 0.3;
    GPdigRange1_msg.scale.z = 0.3;
    GPdigRange1_msg.color.r = 0.0;
    GPdigRange1_msg.color.g = 1.0;
    GPdigRange1_msg.color.b = 0.0;
    GPdigRange1_msg.color.a = 1.0;

    // digging region 2 (configured)
    GPdigRange2_msg.header.frame_id = "base";
    GPdigRange2_msg.header.stamp = ros::Time::now();
    GPdigRange2_msg.ns = "GP_dig_region2";
    GPdigRange2_msg.id = 9;
    GPdigRange2_msg.type = line_strip;
    GPdigRange2_msg.action = visualization_msgs::Marker::ADD;
    GPdigRange2_msg.pose.orientation.w = 1.0;
    GPdigRange2_msg.scale.x = 0.3;
    GPdigRange2_msg.scale.y = 0.3;
    GPdigRange2_msg.scale.z = 0.3;
    GPdigRange2_msg.color.r = 1.0;
    GPdigRange2_msg.color.g = 1.0;
    GPdigRange2_msg.color.b = 0.0;
    GPdigRange2_msg.color.a = 1.0;

    // digging region 1 expanded (configured ±10deg)
    GPdigRange1Ext_msg.header.frame_id = "base";
    GPdigRange1Ext_msg.header.stamp = ros::Time::now();
    GPdigRange1Ext_msg.ns = "GP_dig_region1_ext";
    GPdigRange1Ext_msg.id = 10;
    GPdigRange1Ext_msg.type = line_strip;
    GPdigRange1Ext_msg.action = visualization_msgs::Marker::ADD;
    GPdigRange1Ext_msg.pose.orientation.w = 1.0;
    GPdigRange1Ext_msg.scale.x = 0.2;
    GPdigRange1Ext_msg.scale.y = 0.2;
    GPdigRange1Ext_msg.scale.z = 0.2;
    GPdigRange1Ext_msg.color.r = 0.0;
    GPdigRange1Ext_msg.color.g = 1.0;
    GPdigRange1Ext_msg.color.b = 1.0;
    GPdigRange1Ext_msg.color.a = 0.8;

    // digging region 2 expanded (configured ±10deg)
    GPdigRange2Ext_msg.header.frame_id = "base";
    GPdigRange2Ext_msg.header.stamp = ros::Time::now();
    GPdigRange2Ext_msg.ns = "GP_dig_region2_ext";
    GPdigRange2Ext_msg.id = 11;
    GPdigRange2Ext_msg.type = line_strip;
    GPdigRange2Ext_msg.action = visualization_msgs::Marker::ADD;
    GPdigRange2Ext_msg.pose.orientation.w = 1.0;
    GPdigRange2Ext_msg.scale.x = 0.2;
    GPdigRange2Ext_msg.scale.y = 0.2;
    GPdigRange2Ext_msg.scale.z = 0.2;
    GPdigRange2Ext_msg.color.r = 1.0;
    GPdigRange2Ext_msg.color.g = 0.0;
    GPdigRange2Ext_msg.color.b = 1.0;
    GPdigRange2Ext_msg.color.a = 0.8;

    // 新增的GPtruckRange_msg初始化（格式完全一样，只改颜色和ID）
    GPtruckRange_msg.header.frame_id = "base";
    GPtruckRange_msg.header.stamp = ros::Time::now();
    GPtruckRange_msg.ns = "GP_truck_range";  // namespace不同
    GPtruckRange_msg.id = 7;  // ID必须不同！
    GPtruckRange_msg.type = line_strip; // 一样的类型
    GPtruckRange_msg.action = visualization_msgs::Marker::ADD;
    GPtruckRange_msg.pose.orientation.w = 1.0;
    GPtruckRange_msg.scale.x = 0.3;  // 一样的线宽
    GPtruckRange_msg.scale.y = 0.3;
    GPtruckRange_msg.scale.z = 0.3;
    // 颜色改成红色以区分
    GPtruckRange_msg.color.r = 1.0;
    GPtruckRange_msg.color.g = 0.27;
    GPtruckRange_msg.color.b = 0.0;
    GPtruckRange_msg.color.a = 1.0;  // 不透明

    GPtruckRawRange_msg.header.frame_id = "base";
    GPtruckRawRange_msg.header.stamp = ros::Time::now();
    GPtruckRawRange_msg.ns = "GP_truck_raw_range";
    GPtruckRawRange_msg.id = 12;
    GPtruckRawRange_msg.type = line_strip;
    GPtruckRawRange_msg.action = visualization_msgs::Marker::ADD;
    GPtruckRawRange_msg.pose.orientation.w = 1.0;
    GPtruckRawRange_msg.scale.x = 0.2;
    GPtruckRawRange_msg.scale.y = 0.2;
    GPtruckRawRange_msg.scale.z = 0.2;
    GPtruckRawRange_msg.color.r = 1.0;
    GPtruckRawRange_msg.color.g = 0.0;
    GPtruckRawRange_msg.color.b = 0.0;
    GPtruckRawRange_msg.color.a = 1.0;

    // digging terrain
    GPterrain_msg.header.frame_id = "base";
    GPterrain_msg.header.stamp = ros::Time::now();
    GPterrain_msg.ns = "GP_terrain";
    GPterrain_msg.id = 3;
    GPterrain_msg.type = line_strip;
    GPterrain_msg.action = visualization_msgs::Marker::ADD;
    GPterrain_msg.pose.orientation.w = 1.0;
    GPterrain_msg.scale.x = 0.1;
    GPterrain_msg.scale.y = 0.1;
    GPterrain_msg.scale.z = 0.1;
    // GPterrain_msg.color.r = 0.9294;
    // GPterrain_msg.color.g = 0.7412;
    // GPterrain_msg.color.b = 0.3961;
    // GPterrain_msg.color.a = 1.0;
    GPterrain_msg.color.r = 1;
    GPterrain_msg.color.g = 0;
    GPterrain_msg.color.b = 0;
    GPterrain_msg.color.a = 1.0;

    // predicted trajectory
    GPtraj_msg.header.frame_id = "base";
    GPtraj_msg.header.stamp = ros::Time::now();
    GPtraj_msg.ns = "GP_traj";
    GPtraj_msg.id = 4;
    GPtraj_msg.type = line_strip;
    GPtraj_msg.action = visualization_msgs::Marker::ADD;
    GPtraj_msg.pose.orientation.w = 1.0;
    GPtraj_msg.scale.x = 0.1;
    GPtraj_msg.scale.y = 0.1;
    GPtraj_msg.scale.z = 0.1;
    GPtraj_msg.color.r = 1.0;
    GPtraj_msg.color.g = 1.0;
    GPtraj_msg.color.b = 1.0;
    GPtraj_msg.color.a = 1.0;

    // real bucket pos
    RealBkt_msg.header.frame_id = "base";
    RealBkt_msg.header.stamp = ros::Time::now();
    RealBkt_msg.ns = "GP_realbkt";
    RealBkt_msg.id = 5;
    RealBkt_msg.type = line_strip;
    RealBkt_msg.action = visualization_msgs::Marker::ADD;
    RealBkt_msg.pose.orientation.w = 1.0;
    RealBkt_msg.scale.x = 0.1;
    RealBkt_msg.scale.y = 0.1;
    RealBkt_msg.scale.z = 0.1;
    RealBkt_msg.color.a = 0.8;

    // reference bucket pos
    RefBkt_msg.header.frame_id = "base";
    RefBkt_msg.header.stamp = ros::Time::now();
    RefBkt_msg.ns = "GP_refbkt";
    RefBkt_msg.id = 6;
    RefBkt_msg.type = line_strip;
    RefBkt_msg.action = visualization_msgs::Marker::ADD;
    RefBkt_msg.pose.orientation.w = 1.0;
    RefBkt_msg.scale.x = 0.1;
    RefBkt_msg.scale.y = 0.1;
    RefBkt_msg.scale.z = 0.1;
    RefBkt_msg.color.a = 0.8;

    // data
    SwWkAgl_msg.x = 180.0f;
    SwWkAgl_msg.y = 0.0f;

    DigStartDepth_msg.x = 9.0f;
    DigStartDepth_msg.y = 0.0f;

    AreaUbLb_msg.x = 200.0f;
    AreaUbLb_msg.y = 100.0f;
    
    VolTrack_msg.x = 0.0f;
    VolTrack_msg.y = 4.0f;
    
    BktAgl_msg.x = 0.0f;

    // terrain and ground
    std::vector<double> zeros(array_len-r_len,0);

    auto& terrain_data = terrainZ_msg.data;
    terrain_data.clear();
    terrain_data.assign(r_len,-5);
    terrain_data.insert(terrain_data.end(),zeros.begin(),zeros.end());

    auto& ground_data = groundZ_msg.data;
    ground_data.clear();
    ground_data.assign(r_len,-5);
    ground_data.insert(ground_data.end(),zeros.begin(),zeros.end());

    msg_reset();
    msg_publish();
}

#pragma endregion

#pragma region msg reset
void GP::msg_reset()
{
    // flag
    flagGPfinish_msg.data = 0.0f;
    SwWkAgl_msg.z = 0.0f;
    DigStartDepth_msg.z = 0.0f;
    AreaUbLb_msg.z = 0.0f;
    VolTrack_msg.z = 0.0f;
    GPemtpy_msg.x = 0.0f;
    GPemtpy_msg.y = 0.0f;
    BktAgl_msg.z = 0.0f;
    ErrorFlag_msg.x = 0;
    ErrorFlag_msg.y = 0;
    ErrorFlag_msg.z = 0;

    // terrain
    std::vector<double> zeros(array_len-r_len,0);
    auto&  terrain_data = terrainZ_msg.data;
    auto&  ground_data = groundZ_msg.data;
    std::copy(zeros.begin(),zeros.end(),terrain_data.begin()+r_len);
    std::copy(zeros.begin(),zeros.end(),ground_data.begin()+r_len);

    // save GUI parameters
    GUIparam_vec.assign({track_prot,static_cast<double>(N_fans),ground_crash_prot,max_depth,ground_setting,
                    rad2deg(angle_ub_setting),rad2deg(angle_lb_setting),rad2deg(angle_dig_ub1_setting),rad2deg(angle_dig_lb1_setting),
                    rad2deg(angle_dig_ub2_setting),rad2deg(angle_dig_lb2_setting),target_ub,target_lb,
                    edge_extend,rad2deg(extend_angle),expnt,diggable_threshold,l_QV, turn_point});

    GUIparam_msg.data.clear();
    for (int i=0;i<GUIparam_vec.size();i++)
    {
        GUIparam_msg.data.push_back(GUIparam_vec[i]);
    }
}
#pragma endregion

#pragma region msg write    
// write msg after calculation    
void GP::msg_write()        
{
    if (mpc2_switch)
    {
        flagGPfinish_msg.data = dig_flag;
        GPtraj_msg.points.clear();
        GPtraj_msg.colors.clear();
        GPterrain_msg.points.clear();
        ErrorFlag_msg.x = gridheight_errorflag;
        ErrorFlag_msg.y = gridlabel_errorflag;
        ErrorFlag_msg.z = gridexist_errorflag;
        // do not update dig joints in unload mode
        const double qnan_j = std::numeric_limits<double>::quiet_NaN();
        DigJoints_msg.x = qnan_j;
        DigJoints_msg.y = qnan_j;
        DigJoints_msg.z = qnan_j;
        DigJoints_msg.w = qnan_j;
        return;
    }

    double ground_height = -3.3;
    // GP point and device position 
    Eigen::Matrix3d devicepos = getDeviceRealPos().array();
    geometry_msgs::Point ori;
    ori.x=0;
    ori.y=0;
    ori.z=0 + boom_pos[2];
    
    GPmarker_msg.pose.position.x = devicepos(0,2);
    GPmarker_msg.pose.position.y = devicepos(1,2);
    GPmarker_msg.pose.position.z = devicepos(2,2);
    GPmarker_msg.header.stamp = ros::Time::now();
    
    auto& device_pts = GPdevice_msg.points;
    device_pts.clear();
    device_pts.push_back(ori);
    for (int i=0; i<3; i++)
    {
        geometry_msgs::Point p;
        p.x = devicepos(0,i);
        p.y = devicepos(1,i);
        p.z = devicepos(2,i);
        device_pts.push_back(p);
    }

    GPmarker_msg.color.r = 1.0;         // gp low, show red; normal, show white
    GPmarker_msg.color.g = (dig_flag<0) ? 0.0 : 1.0;
    GPmarker_msg.color.b = (dig_flag<0) ? 0.0 : 1.0;
    GPdevice_msg.color.r = 1.0;
    GPdevice_msg.color.g = (dig_flag<0) ? 0.0 : 1.0;
    GPdevice_msg.color.b = (dig_flag<0) ? 0.0 : 1.0;

    // GP digging range
    auto& range_pts = GPrange_msg.points;
    range_pts.clear();
    sort(track_dist_vec.begin(),track_dist_vec.end());
    sort(theta_vec.begin(),theta_vec.end());

    for (int i=0; i<N_fans; i++)
    {
        geometry_msgs::Point p;
        p.x = track_dist_vec[i] * cos(theta_vec[i]-angle_offset);
        p.y = track_dist_vec[i] * sin(theta_vec[i]-angle_offset);
        p.z = -1;
        range_pts.push_back(p);
    }
    for (int j=N_fans-1; j>=0; j--)
    {
        geometry_msgs::Point p;
        p.x = 11 * cos(theta_vec[j]-angle_offset);
        p.y = 11 * sin(theta_vec[j]-angle_offset);
        p.z = ground_height;
        range_pts.push_back(p);
    }
    geometry_msgs::Point p;
    p.x = track_dist_vec[0] * cos(theta_vec[0]-angle_offset);
    p.y = track_dist_vec[0] * sin(theta_vec[0]-angle_offset);
    p.z = -1;
    range_pts.push_back(p);

    // Configured digging regions
    auto fill_region = [&](visualization_msgs::Marker& msg, double ang_lb, double ang_ub)
    {
        auto& pts = msg.points;
        pts.clear();

        double inner_r = (track_dist_vec.empty()) ? 3.0 : track_dist_vec.front();
        double outer_r = 11.0;
        int n_arc = std::max(2, N_fans);

        for (int i = 0; i < n_arc; i++)
        {
            double a = ang_lb + i * (ang_ub - ang_lb) / static_cast<double>(n_arc - 1);
            geometry_msgs::Point p;
            p.x = inner_r * cos(a - angle_offset);
            p.y = inner_r * sin(a - angle_offset);
            p.z = -1;
            pts.push_back(p);
        }
        for (int i = n_arc - 1; i >= 0; i--)
        {
            double a = ang_lb + i * (ang_ub - ang_lb) / static_cast<double>(n_arc - 1);
            geometry_msgs::Point p;
            p.x = outer_r * cos(a - angle_offset);
            p.y = outer_r * sin(a - angle_offset);
            p.z = ground_height;
            pts.push_back(p);
        }
        if (!pts.empty())
            pts.push_back(pts.front());
    };

    fill_region(GPdigRange1_msg, angle_dig_lb1_setting, angle_dig_ub1_setting);
    fill_region(GPdigRange2_msg, angle_dig_lb2_setting, angle_dig_ub2_setting);

    fill_region(GPdigRange1Ext_msg, angle_dig_lb1_setting - dig_region_margin, angle_dig_ub1_setting + dig_region_margin);
    fill_region(GPdigRange2Ext_msg, angle_dig_lb2_setting - dig_region_margin, angle_dig_ub2_setting + dig_region_margin);

    // GP truck range
    auto& range_truck_pts = GPtruckRange_msg.points;
    range_truck_pts.clear();
    auto& range_truck_raw_pts = GPtruckRawRange_msg.points;
    range_truck_raw_pts.clear();

    const int expected = 2 * N_fans;
    auto fill_truck_range = [&](const std::vector<double>& theta_truck_vec, visualization_msgs::Marker& msg, const char* name)
    {
        auto& pts = msg.points;
        pts.clear();

        if (static_cast<int>(theta_truck_vec.size()) >= expected && expected > 0)
        {
            for (int i = 0; i < expected; i++)
            {
                geometry_msgs::Point p;
                p.x = 3 * cos(theta_truck_vec[i]);
                p.y = 3 * sin(theta_truck_vec[i]);
                p.z = -1;
                pts.push_back(p);
            }
            for (int j = expected - 1; j >= 0; j--)
            {
                geometry_msgs::Point p;
                p.x = 11 * cos(theta_truck_vec[j]);
                p.y = 11 * sin(theta_truck_vec[j]);
                p.z = -1;
                pts.push_back(p);
            }

            if (!pts.empty())
                pts.push_back(pts.front());
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "msg_write(): %s undersized (%ld), expected %d; truck range visualization will use fallback point", name, static_cast<long>(theta_truck_vec.size()), expected);
            geometry_msgs::Point pp;
            pp.x = 0;
            pp.y = 0;
            pp.z = -1;
            pts.push_back(pp);
        }
    };

    fill_truck_range(thetaTruck_vec, GPtruckRange_msg, "thetaTruck_vec");
    fill_truck_range(thetaTruckRaw_vec, GPtruckRawRange_msg, "thetaTruckRaw_vec");

    // If there is no safe sector, do not publish a latched dig solution visualization.
    if (dig_flag == -3)
    {
        GPtraj_msg.points.clear();
        GPtraj_msg.colors.clear();
        GPterrain_msg.points.clear();
        return;
    }

    // GP predicted trajectory
    auto& traj_pts = GPtraj_msg.points;
    traj_pts.clear();
    GPtraj_msg.colors.clear();
    Eigen::AngleAxisd rotation(dig_swing - angle_offset,Eigen::Vector3d::UnitZ());
    Eigen::Translation3d trans(boom_pos[0],boom_pos[1],boom_pos[2]);
    Eigen::Matrix3Xd pts1 = rotation * trans * traj_pred.transpose();
    for (int i=0; i<pts1.cols(); i++)
    {
        geometry_msgs::Point p;
        p.x = pts1(0,i);
        p.y = pts1(1,i);
        p.z = pts1(2,i);
        traj_pts.push_back(p);

        std_msgs::ColorRGBA color2;
        color2.a = 0.7;
        const int terrain_idx = 45 + i;
        const double terrain_v = (dig_terrainZ.size() > terrain_idx) ? dig_terrainZ[terrain_idx] : std::numeric_limits<double>::quiet_NaN();
        if (!std::isnan(terrain_v) && (p.z > terrain_v + boom_pos[2] - diggable_threshold))    // undiggable: purple, else: ivory
        {
            color2.r = 0.3333;
            color2.g = 0.1020;
            color2.b = 0.5451;
        }
        else
        {
            color2.r = 0.8039;
            color2.g = 0.8039;
            color2.b = 0.7569;
        }
        GPtraj_msg.colors.push_back(color2);
    }

    // GP terrain
    auto& terrain_pts = GPterrain_msg.points;
    terrain_pts.clear();
    Eigen::Matrix3Xd pts2(3,r_len);
    
    if (r_range.size() == r_len)
        pts2.row(0) = r_range;
    else
        pts2.row(0) = Eigen::VectorXd::Constant(r_len, std::numeric_limits<double>::quiet_NaN());
    pts2.row(1) = Eigen::VectorXd::Zero(r_len);
    Eigen::VectorXd terrain_safe;
    if (dig_terrainZ.size() == r_len)
    {
        terrain_safe = dig_terrainZ;
    }
    else
    {
        ROS_WARN_THROTTLE(1.0, "msg_write(): dig_terrainZ size mismatch (%ld) vs r_len=%d, terrain visualization will use NaNs", static_cast<long>(dig_terrainZ.size()), r_len);
        terrain_safe = Eigen::VectorXd::Constant(r_len, std::numeric_limits<double>::quiet_NaN());
    }
    pts2.row(2) = terrain_safe.array() + boom_pos[2];
    pts2 = rotation.toRotationMatrix() * pts2;
    
    for (int i=0; i<r_len; i++)
    {
        geometry_msgs::Point p;
        p.x = pts2(0,i);
        p.y = pts2(1,i);
        p.z = pts2(2,i);
        terrain_pts.push_back(p);
    }
        
    
    // flag
    flagGPfinish_msg.data = dig_flag;

    GPemtpy_msg.x = emtpy_flag;
    GPemtpy_msg.y = vol_modifier;

    // data
    SwWkAgl_msg.x = rad2deg(dig_swing);
    SwWkAgl_msg.y = rad2deg(working_angle);
    SwWkAgl_msg.z = dig_flag;

    DigStartDepth_msg.x = dig_start;
    DigStartDepth_msg.y = dig_depth;
    DigStartDepth_msg.z = dig_flag;

    AreaUbLb_msg.x = rad2deg(area_ub);
    AreaUbLb_msg.y = rad2deg(area_lb);
    AreaUbLb_msg.z = dig_flag;

    VolTrack_msg.x = vol_pred;
    VolTrack_msg.y = dig_trackdist;
    VolTrack_msg.z = dig_flag;

    BktAgl_msg.x = rad2deg(dig_bktangle);
    BktAgl_msg.z = dig_flag;

    // terrain and ground
    std::vector<double> fills(array_len-r_len,dig_flag);
    auto&  terrain_data = terrainZ_msg.data;
    auto&  ground_data = groundZ_msg.data;
    terrain_data.clear();
    ground_data.clear();
    
    for (int i=0; i<r_len; i++)
    {
        const double terrain_v = (dig_terrainZ.size() > i) ? dig_terrainZ[i] : std::numeric_limits<double>::quiet_NaN();
        const double ground_v = (dig_groundZ.size() > i) ? dig_groundZ[i] : std::numeric_limits<double>::quiet_NaN();
        terrain_data.push_back(terrain_v);
        ground_data.push_back(ground_v);
    }
    terrain_data.insert(terrain_data.end(),fills.begin(),fills.end());
    ground_data.insert(ground_data.end(),fills.begin(),fills.end());

    // error flag
    ErrorFlag_msg.x = gridheight_errorflag;
    ErrorFlag_msg.y = gridlabel_errorflag;
    ErrorFlag_msg.z = gridexist_errorflag;

    // prepare dig joints Quaternion (in degrees): x=swing, y=boom, z=arm, w=bucket
    DigJoints_msg.x = rad2deg(static_cast<double>(dig_swing));
    DigJoints_msg.y = (dig_joints.size() > 0) ? rad2deg(dig_joints[0]) : std::numeric_limits<double>::quiet_NaN();
    DigJoints_msg.z = (dig_joints.size() > 1) ? rad2deg(dig_joints[1]) : std::numeric_limits<double>::quiet_NaN();
    DigJoints_msg.w = (dig_joints.size() > 2) ? rad2deg(dig_joints[2]) : std::numeric_limits<double>::quiet_NaN();
}
#pragma endregion

#pragma region msg update
// realtime msg update
void GP::msg_update(){
    // real bkt position msg
    auto& pts = RealBkt_msg.points;
    auto& cls = RealBkt_msg.colors;
    int maxnum = 100;
    forward_kinematic();  // get realtime bkt teeth pos, based on IMU data
    int n = pts.size();
    
    if (n>maxnum)
    {
        pts.erase(pts.begin());
        cls.erase(cls.begin());
    }   
    static double colorid = 0;
    std_msgs::ColorRGBA color;
    color.r = 1 - abs(colorid-maxnum)/maxnum;
    color.g = abs(colorid-maxnum)/maxnum;
    color.b = 0.8;
    color.a = 0.7;
    colorid = (colorid >= 2 * maxnum) ? 1 : (colorid+1) ;
    cls.push_back(color);

    geometry_msgs::Point p;
    p.x = realbkt_pos[0];
    p.y = realbkt_pos[1];
    p.z = realbkt_pos[2];
    pts.push_back(p);

    // ref bkt postition msg
    auto& pts2 = RefBkt_msg.points;
    auto& cls2 = RefBkt_msg.colors;
    Eigen::Vector3d refpos = getDeviveRefPos();
    int n2 = pts2.size();

    if (n2>maxnum)
    {
        pts2.erase(pts2.begin());
        cls2.erase(cls2.begin());
    }   
    std_msgs::ColorRGBA color2;
    color2.r = 1;
    color2.g = 0.9255;
    color2.b = 0.5451;
    color2.a = 0.7;
    cls2.push_back(color2);

    geometry_msgs::Point p2;
    p2.x = refpos[0];
    p2.y = refpos[1];
    p2.z = refpos[2];
    pts2.push_back(p2);

}
#pragma endregion

#pragma region msg pub
void GP::msg_publish()
{
    GPvisual_publisher.publish(GPmarker_msg);
    GPvisual_publisher.publish(GPdevice_msg);
    GPvisual_publisher.publish(GPrange_msg);
    GPvisual_publisher.publish(GPdigRange1_msg);
    GPvisual_publisher.publish(GPdigRange2_msg);
    GPvisual_publisher.publish(GPdigRange1Ext_msg);
    GPvisual_publisher.publish(GPdigRange2Ext_msg);
    GPvisual_publisher.publish(GPtruckRange_msg);
    GPvisual_publisher.publish(GPtruckRawRange_msg);
    GPvisual_publisher.publish(GPtraj_msg);
    GPvisual_publisher.publish(GPterrain_msg);
    RealBkt_publisher.publish(RealBkt_msg);
    RefBkt_publisher.publish(RefBkt_msg);
    flagGPfinish_publisher.publish(flagGPfinish_msg);
    if (!mpc2_switch)
    {
        SwWkAgl_publisher.publish(SwWkAgl_msg);
        DigStartDepth_publisher.publish(DigStartDepth_msg);
        BktAgl_publisher.publish(BktAgl_msg);
        DigJoints_publisher.publish(DigJoints_msg);
    }
    terrainZ_publisher.publish(terrainZ_msg);
    groundZ_publisher.publish(groundZ_msg);
    AreaUbLb_publisher.publish(AreaUbLb_msg);
    VolTrack_publisher.publish(VolTrack_msg);
    GUIparam_publisher.publish(GUIparam_msg);
    GPempty_publisher.publish(GPemtpy_msg);
    ErrorFlag_publisher.publish(ErrorFlag_msg);
    
}

#pragma region msg print
void GP::msg_print()
{
     // print time
    std::time_t now = std::time(nullptr);
    std::tm *ltm = std::localtime(&now);
    std::cout << "Time: "
              << std::setw(4) << 1900 + ltm->tm_year << "-"
              << std::setw(2) << std::setfill('0') << 1 + ltm->tm_mon << "-"
              << std::setw(2) << std::setfill('0') << ltm->tm_mday << "  "
              << std::setw(2) << std::setfill('0') << ltm->tm_hour << ":"
              << std::setw(2) << std::setfill('0') << ltm->tm_min << ":"
              << std::setw(2) << std::setfill('0') << ltm->tm_sec 
              << std::endl ;

    ROS_INFO("Time elapsed: %f s", time_elapsed);

    if (!ground_protect_alt_overridden_by_truck_pose)
        ROS_INFO("ground_protect_alt source: YAML default (no /truck_pose_topic yet): %f", ground_protect_alt);
    else
        ROS_INFO("ground_protect_alt source: /truck_pose_topic override: %f", ground_protect_alt);

    if (std::isfinite(swing_center_alt))
        ROS_INFO("swing_center_alt: %f", swing_center_alt);
    else
        ROS_INFO("swing_center_alt: NaN");

    ROS_INFO("ground_def: %f m", ground_def);

    if (!mpc2_switch)
    {
        ROS_INFO("Dig range: [%f, %f]",rad2deg(area_lb),rad2deg(area_ub));
        ROS_INFO("Swing angle: %f deg ", rad2deg(dig_swing));
        ROS_INFO("dig start: %f m, dig depth: %f m,\n bkt angle: %f deg, volume: %f m3",dig_start,dig_depth,rad2deg(dig_bktangle),vol_pred);
        ROS_INFO("dig height: %f m ", dig_height); 
        ROS_INFO("dig fan start id: %f . ", dig_fan_start_id);
        ROS_INFO("dig x start id: %f . ", dig_start_id);
        ROS_INFO("dig height terrain: %f m ", dig_height_terrain);
        if (dig_groundZ.size() > 0)
            ROS_INFO("dig groundZ[0]: %f m", dig_groundZ[0]);
    }
    if (mpc2_switch && unload_solution_valid)
        ROS_INFO("Unload: angle=%.3f deg, dist=%.3f, height=%.3f", Unload_msg.x, Unload_msg.y, Unload_msg.z);
    // ROS_INFO_STREAM("dig terrainZ: " << dig_terrainZ.transpose() << " m");
    // ROS_INFO_STREAM("dig groundZ: " << dig_groundZ.transpose() << " m");
    #ifdef DEBUG_MODE
        std::cout << "\n terrain:\n";
        printeig(dig_terrainZ.transpose());
        std::cout << "\n ground: \n" ;
        printeig(dig_groundZ.transpose());
    #endif
    std::cout << "by " << GP_VERSION << std::endl;
    std::cout <<"----------------------------------------------------------------------------- \n";
}

#pragma endregion

