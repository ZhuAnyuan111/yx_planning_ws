#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <string>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <limits>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <GP2/GP.h>
#include <GP2/utils.h>
#include <fstream>
#include <random>

static bool getDoubleArrayParam(ros::NodeHandle& nh,
                                const std::string& key,
                                std::vector<double>& out,
                                size_t expected_size)
{
    XmlRpc::XmlRpcValue v;
    if (!nh.getParam(key, v))
        return false;
    if (v.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Param %s is not an array.", key.c_str());
        return false;
    }
    if (expected_size > 0 && static_cast<size_t>(v.size()) != expected_size)
    {
        ROS_ERROR("Param %s size mismatch (%d != %zu).", key.c_str(), v.size(), expected_size);
        return false;
    }

    std::vector<double> tmp;
    tmp.reserve(v.size());
    for (int i = 0; i < v.size(); ++i)
    {
        if (v[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
            tmp.push_back(static_cast<int>(v[i]));
        else if (v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
            tmp.push_back(static_cast<double>(v[i]));
        else
        {
            ROS_ERROR("Param %s[%d] is not numeric.", key.c_str(), i);
            return false;
        }
    }
    out.swap(tmp);
    return true;
}

 static void unwrapAngleInterval(double& lb, double& ub)
 {
     if (ub < lb)
         ub += 2 * M_PI;
 }

 static void normalizeAndClampDigRegion(double& lb, double& ub,
                                       double global_lb, double global_ub,
                                       const char* name)
 {
     auto overlap_len = [&](double a_lb, double a_ub) -> double
     {
         const double o_lb = std::max(a_lb, global_lb);
         const double o_ub = std::min(a_ub, global_ub);
         return std::max(0.0, o_ub - o_lb);
     };

     const double shifts[3] = {0.0, 2 * M_PI, -2 * M_PI};
     double best_lb = lb;
     double best_ub = ub;
     double best_ol = overlap_len(lb, ub);
     for (double s : shifts)
     {
         const double cand_lb = lb + s;
         const double cand_ub = ub + s;
         const double ol = overlap_len(cand_lb, cand_ub);
         if (ol > best_ol)
         {
             best_ol = ol;
             best_lb = cand_lb;
             best_ub = cand_ub;
         }
     }

     lb = std::max(best_lb, global_lb);
     ub = std::min(best_ub, global_ub);

     if (lb > ub)
     {
         ROS_WARN("%s does not overlap global range [%.3f, %.3f] rad. Region disabled.",
                  name, global_lb, global_ub);
         lb = 1.0;
         ub = 0.0;
     }
 }

 static double projectAngleIntoDigRegions(double angle_wrapped,
                                         double lb1, double ub1,
                                         double lb2, double ub2,
                                         double margin_rad)
 {
     angle_wrapped = std::fmod(angle_wrapped, 2 * M_PI);
     if (angle_wrapped < 0) angle_wrapped += 2 * M_PI;

     struct Interval
     {
         double lb;
         double ub;
     };
     std::vector<Interval> intervals;
     intervals.reserve(2);
     if (lb1 <= ub1) intervals.push_back({lb1 - margin_rad, ub1 + margin_rad});
     if (lb2 <= ub2) intervals.push_back({lb2 - margin_rad, ub2 + margin_rad});
     if (intervals.empty())
         return angle_wrapped;

     const double shifts[3] = {-2 * M_PI, 0.0, 2 * M_PI};
     for (const auto& itv : intervals)
     {
         for (double s : shifts)
         {
             const double lb = itv.lb + s;
             const double ub = itv.ub + s;
             if (angle_wrapped >= lb && angle_wrapped <= ub)
                 return angle_wrapped;
         }
     }

     double best = angle_wrapped;
     double best_dist = std::numeric_limits<double>::infinity();
     for (const auto& itv : intervals)
     {
         for (double s : shifts)
         {
             const double lb = itv.lb + s;
             const double ub = itv.ub + s;
             const double clamped = std::min(std::max(angle_wrapped, lb), ub);
             const double dist = std::abs(angle_wrapped - clamped);
             if (dist < best_dist)
             {
                 best_dist = dist;
                 best = clamped;
             }
         }
     }

     best = std::fmod(best, 2 * M_PI);
     if (best < 0) best += 2 * M_PI;
     return best;
 }

 static void selectLargestContiguousSafeBlock(std::vector<int>& safe_fan_ids,
                                             const std::vector<std::vector<int>>& blocks)
 {
     if (blocks.empty())
     {
         safe_fan_ids.clear();
         return;
     }

     size_t best_i = 0;
     for (size_t i = 1; i < blocks.size(); ++i)
     {
         if (blocks[i].size() > blocks[best_i].size())
             best_i = i;
     }
     safe_fan_ids = blocks[best_i];
 }

#pragma region construct func
GP::GP(): rt(10.0)
{
    #ifdef DEBUG_MODE
        flag_cal1=true;
    #endif

    // Load GUI-like parameters from ROS parameter server (typically loaded from a YAML via rosparam in launch).
    {
        double tmp_l_CF = l_CF;
        double tmp_l_FQ = l_FQ;
        double tmp_l_QV = l_QV;
        double tmp_l_MN = l_MN;
        double tmp_l_NQ = l_NQ;
        double tmp_l_KQ = l_KQ;
        double tmp_l_KM = l_KM;
        double phi_FQN_deg = rad2deg(phi_FQN);
        double phi_KQV_deg = rad2deg(phi_KQV);
        double phi_BCF_deg = rad2deg(phi_BCF);
        double tmp_bkt_width = bkt_width;
        double tmp_base2ground = base2ground;
        std::vector<double> tmp_IMU_bias_deg = {rad2deg(IMU_bias[0]), rad2deg(IMU_bias[1]), rad2deg(IMU_bias[2])};
        std::vector<double> tmp_boom_angle_limit_deg = {rad2deg(boom_angle_limit[0]), rad2deg(boom_angle_limit[1])};
        std::vector<double> tmp_arm_angle_limit_deg = {rad2deg(arm_angle_limit[0]), rad2deg(arm_angle_limit[1])};
        std::vector<double> tmp_bkt_angle_limit_deg = {rad2deg(bkt_angle_limit[0]), rad2deg(bkt_angle_limit[1])};
        std::vector<double> tmp_boom_pos = boom_pos;

        double tmp_track_prot = track_prot;
        int tmp_n_fans = N_fans;
        double tmp_ground_crash_prot = ground_crash_prot;
        double tmp_max_depth = max_depth;
        double tmp_ground_setting = ground_setting;
        double tmp_ground_def = ground_def;
        double tmp_max_pt_height = max_pt_height;
        double tmp_min_pt_height = min_pt_height;

        double angle_ub_deg = rad2deg(angle_ub_setting);
        double angle_lb_deg = rad2deg(angle_lb_setting);
        double angle_dig_ub1_deg = rad2deg(angle_dig_ub1_setting);
        double angle_dig_lb1_deg = rad2deg(angle_dig_lb1_setting);
        double angle_dig_ub2_deg = rad2deg(angle_dig_ub2_setting);
        double angle_dig_lb2_deg = rad2deg(angle_dig_lb2_setting);
        double angle_offset_deg = rad2deg(angle_offset);
        double dig_region_margin_deg = rad2deg(dig_region_margin);
        double danger_zone_margin_deg = rad2deg(danger_zone_margin);

        double tmp_target_ub = target_ub;
        double tmp_target_lb = target_lb;
        double tmp_edge_extend = edge_extend;
        double extend_angle_deg = rad2deg(extend_angle);
        double truck_angle_prot_deg = rad2deg(truck_angle_prot);
        double tmp_expnt = expnt;
        double tmp_diggable_threshold = diggable_threshold;
        double tmp_digx_min = digx_min;
        double tmp_turn_point = turn_point;

        double sensor_l_CF_mm = tmp_l_CF * 1000.0;
        if (nh.getParam("sensor/CF", sensor_l_CF_mm))
            tmp_l_CF = sensor_l_CF_mm / 1000.0;
        else
            nh.param("GP2mid/l_CF", tmp_l_CF, tmp_l_CF);
        double sensor_l_FQ_mm = tmp_l_FQ * 1000.0;
        if (nh.getParam("sensor/FQ", sensor_l_FQ_mm))
            tmp_l_FQ = sensor_l_FQ_mm / 1000.0;
        else
            nh.param("GP2mid/l_FQ", tmp_l_FQ, tmp_l_FQ);
        double sensor_l_QV_mm = tmp_l_QV * 1000.0;
        if (nh.getParam("sensor/QV", sensor_l_QV_mm))
            tmp_l_QV = sensor_l_QV_mm / 1000.0;
        else
            nh.param("GP2mid/l_QV", tmp_l_QV, tmp_l_QV);
        double sensor_l_MN_mm = tmp_l_MN * 1000.0;
        if (nh.getParam("sensor/MN", sensor_l_MN_mm))
            tmp_l_MN = sensor_l_MN_mm / 1000.0;
        else
            nh.param("GP2mid/l_MN", tmp_l_MN, tmp_l_MN);
        double sensor_l_NQ_mm = tmp_l_NQ * 1000.0;
        if (nh.getParam("sensor/QN", sensor_l_NQ_mm))
            tmp_l_NQ = sensor_l_NQ_mm / 1000.0;
        else
            nh.param("GP2mid/l_NQ", tmp_l_NQ, tmp_l_NQ);
        double sensor_l_KQ_mm = tmp_l_KQ * 1000.0;
        if (nh.getParam("sensor/QK", sensor_l_KQ_mm))
            tmp_l_KQ = sensor_l_KQ_mm / 1000.0;
        else
            nh.param("GP2mid/l_KQ", tmp_l_KQ, tmp_l_KQ);
        double sensor_l_KM_mm = tmp_l_KM * 1000.0;
        if (nh.getParam("sensor/MK", sensor_l_KM_mm))
            tmp_l_KM = sensor_l_KM_mm / 1000.0;
        else
            nh.param("GP2mid/l_KM", tmp_l_KM, tmp_l_KM);
        if (!nh.getParam("sensor/NQF", phi_FQN_deg))
            nh.param("GP2mid/phi_FQN_deg", phi_FQN_deg, phi_FQN_deg);
        if (!nh.getParam("sensor/KQV", phi_KQV_deg))
            nh.param("GP2mid/phi_KQV_deg", phi_KQV_deg, phi_KQV_deg);
        nh.param("GP2mid/phi_BCF_deg", phi_BCF_deg, phi_BCF_deg);

        nh.param("GP2mid/bkt_width", tmp_bkt_width, tmp_bkt_width);
        double sensor_PC_mm = tmp_base2ground * 1000.0;
        if (nh.getParam("sensor/PC", sensor_PC_mm))
            tmp_base2ground = sensor_PC_mm / 1000.0;
        else
            nh.param("GP2mid/base2ground", tmp_base2ground, tmp_base2ground);
        if (!getDoubleArrayParam(nh, "sensor/IMU_bias", tmp_IMU_bias_deg, 3))
            (void)getDoubleArrayParam(nh, "GP2mid/IMU_bias_deg", tmp_IMU_bias_deg, 3);
        double boom_limit_upper = tmp_boom_angle_limit_deg[1];
        double boom_limit_lower = tmp_boom_angle_limit_deg[0];
        const bool has_sensor_boom_upper = nh.getParam("sensor/Boom_Limit_Upper", boom_limit_upper);
        const bool has_sensor_boom_lower = nh.getParam("sensor/Boom_Limit_Lower", boom_limit_lower);
        if (has_sensor_boom_upper && has_sensor_boom_lower)
            tmp_boom_angle_limit_deg = {boom_limit_lower, boom_limit_upper};
        else
            (void)getDoubleArrayParam(nh, "GP2mid/boom_angle_limit_deg", tmp_boom_angle_limit_deg, 2);
        double arm_limit_upper = tmp_arm_angle_limit_deg[1];
        double arm_limit_lower = tmp_arm_angle_limit_deg[0];
        const bool has_sensor_arm_upper = nh.getParam("sensor/Arm_Limit_Upper", arm_limit_upper);
        const bool has_sensor_arm_lower = nh.getParam("sensor/Arm_Limit_Lower", arm_limit_lower);
        if (has_sensor_arm_upper && has_sensor_arm_lower)
            tmp_arm_angle_limit_deg = {arm_limit_lower, arm_limit_upper};
        else
            (void)getDoubleArrayParam(nh, "GP2mid/arm_angle_limit_deg", tmp_arm_angle_limit_deg, 2);
        double bkt_limit_upper = tmp_bkt_angle_limit_deg[1];
        double bkt_limit_lower = tmp_bkt_angle_limit_deg[0];
        const bool has_sensor_bkt_upper = nh.getParam("sensor/Bkt_Limit_Upper", bkt_limit_upper);
        const bool has_sensor_bkt_lower = nh.getParam("sensor/Bkt_Limit_Lower", bkt_limit_lower);
        if (has_sensor_bkt_upper && has_sensor_bkt_lower)
            tmp_bkt_angle_limit_deg = {bkt_limit_lower, bkt_limit_upper};
        else
            (void)getDoubleArrayParam(nh, "GP2mid/bkt_angle_limit_deg", tmp_bkt_angle_limit_deg, 2);
        (void)getDoubleArrayParam(nh, "GP2mid/boom_pos", tmp_boom_pos, 3);

        nh.param("GP2mid/track_prot", tmp_track_prot, tmp_track_prot);
        nh.param("GP2mid/N_fans", tmp_n_fans, tmp_n_fans);
        nh.param("GP2mid/ground_crash_prot", tmp_ground_crash_prot, tmp_ground_crash_prot);
        nh.param("GP2mid/max_depth", tmp_max_depth, tmp_max_depth);
        nh.param("GP2mid/ground_setting", tmp_ground_setting, tmp_ground_setting);
        nh.param("GP2mid/ground_def", tmp_ground_def, tmp_ground_def);
        nh.param("GP2mid/max_pt_height", tmp_max_pt_height, tmp_max_pt_height);
        nh.param("GP2mid/min_pt_height", tmp_min_pt_height, tmp_min_pt_height);

        std::vector<double> tmp_r_limit = r_limit;
        (void)getDoubleArrayParam(nh, "GP2mid/r_limit", tmp_r_limit, 2);

        nh.param("GP2mid/use_rtk_ground_def", use_rtk_ground_def, use_rtk_ground_def);
        double sensor_rtk_to_swing_center_alt_offset_mm = rtk_to_swing_center_alt_offset * 1000.0;
        if (nh.getParam("sensor/W_W", sensor_rtk_to_swing_center_alt_offset_mm))
            rtk_to_swing_center_alt_offset = sensor_rtk_to_swing_center_alt_offset_mm / 1000.0;
        else
            nh.param("GP2mid/rtk_to_swing_center_alt_offset", rtk_to_swing_center_alt_offset, rtk_to_swing_center_alt_offset);
        nh.param("GP2mid/ground_protect_alt", ground_protect_alt, ground_protect_alt);
        nh.param("GP2mid/truck_pose_ground_protect_alt_offset", truck_pose_ground_protect_alt_offset, truck_pose_ground_protect_alt_offset);
        nh.param("GP2mid/rtk_ground_def_ref", rtk_ground_def_ref, rtk_ground_def_ref);
        nh.param("GP2mid/rtk_ground_def_band", rtk_ground_def_band, rtk_ground_def_band);
        nh.param("GP2mid/auto_dig_region_yaml_path", auto_dig_region_yaml_path, auto_dig_region_yaml_path);
        nh.param("GP2mid/auto_dig_region_half_width_deg", auto_dig_region_half_width_deg, auto_dig_region_half_width_deg);
        nh.param("GP2mid/auto_dig_region_near_offset_deg", auto_dig_region_near_offset_deg, auto_dig_region_near_offset_deg);
        nh.param("GP2mid/auto_dig_region_far_offset_deg", auto_dig_region_far_offset_deg, auto_dig_region_far_offset_deg);

        nh.param("GP2mid/angle_ub_deg", angle_ub_deg, angle_ub_deg);
        nh.param("GP2mid/angle_lb_deg", angle_lb_deg, angle_lb_deg);
        nh.param("GP2mid/angle_dig_ub1_deg", angle_dig_ub1_deg, angle_dig_ub1_deg);
        nh.param("GP2mid/angle_dig_lb1_deg", angle_dig_lb1_deg, angle_dig_lb1_deg);
        nh.param("GP2mid/angle_dig_ub2_deg", angle_dig_ub2_deg, angle_dig_ub2_deg);
        nh.param("GP2mid/angle_dig_lb2_deg", angle_dig_lb2_deg, angle_dig_lb2_deg);
        nh.param("GP2mid/angle_offset_deg", angle_offset_deg, angle_offset_deg);
        nh.param("GP2mid/dig_region_margin_deg", dig_region_margin_deg, dig_region_margin_deg);
        nh.param("GP2mid/danger_zone_margin_deg", danger_zone_margin_deg, danger_zone_margin_deg);

        nh.param("GP2mid/target_ub", tmp_target_ub, tmp_target_ub);
        nh.param("GP2mid/target_lb", tmp_target_lb, tmp_target_lb);
        nh.param("GP2mid/edge_extend", tmp_edge_extend, tmp_edge_extend);

        nh.param("GP2mid/extend_angle_deg", extend_angle_deg, extend_angle_deg);
        nh.param("GP2mid/truck_angle_prot_deg", truck_angle_prot_deg, truck_angle_prot_deg);
        nh.param("GP2mid/expnt", tmp_expnt, tmp_expnt);
        nh.param("GP2mid/diggable_threshold", tmp_diggable_threshold, tmp_diggable_threshold);
        nh.param("GP2mid/digx_min", tmp_digx_min, tmp_digx_min);
        nh.param("GP2mid/turn_point", tmp_turn_point, tmp_turn_point);

        // Apply and validate
        l_CF = tmp_l_CF;
        l_FQ = tmp_l_FQ;
        l_QV = tmp_l_QV;
        l_MN = tmp_l_MN;
        l_NQ = tmp_l_NQ;
        l_KQ = tmp_l_KQ;
        l_KM = tmp_l_KM;
        phi_FQN = deg2rad(phi_FQN_deg);
        phi_KQV = deg2rad(phi_KQV_deg);
        phi_BCF = deg2rad(phi_BCF_deg);
        bkt_width = tmp_bkt_width;
        base2ground = tmp_base2ground;
        IMU_bias = {deg2rad(tmp_IMU_bias_deg[0]), deg2rad(tmp_IMU_bias_deg[1]), deg2rad(tmp_IMU_bias_deg[2])};
        boom_angle_limit = {deg2rad(tmp_boom_angle_limit_deg[0]), deg2rad(tmp_boom_angle_limit_deg[1])};
        arm_angle_limit = {deg2rad(tmp_arm_angle_limit_deg[0]), deg2rad(tmp_arm_angle_limit_deg[1])};
        bkt_angle_limit = {deg2rad(tmp_bkt_angle_limit_deg[0]), deg2rad(tmp_bkt_angle_limit_deg[1])};
        boom_pos = tmp_boom_pos;
        boom2ground = base2ground + boom_pos[2];
        ROS_INFO("effective boom_pos: [%.6f, %.6f, %.6f], boom2ground: %.6f", boom_pos[0], boom_pos[1], boom_pos[2], boom2ground);

        track_prot = tmp_track_prot;
        N_fans = tmp_n_fans;
        if (N_fans < 2)
        {
            ROS_ERROR("Param GP2mid/N_fans invalid (%d). Fallback to 30.", N_fans);
            N_fans = 30;
        }
        ground_crash_prot = tmp_ground_crash_prot;
        max_depth = tmp_max_depth;
        ground_setting = tmp_ground_setting;
        ground_def = tmp_ground_def;
        ground_coef.assign({0,0,1,ground_def});
        max_pt_height = tmp_max_pt_height;
        min_pt_height = tmp_min_pt_height;

        if (tmp_r_limit.size() == 2)
        {
            r_limit = tmp_r_limit;
        }
        if (r_limit.size() != 2)
        {
            ROS_ERROR("Param GP2mid/r_limit invalid size %ld. Fallback to [2,12].", static_cast<long>(r_limit.size()));
            r_limit = {2.0, 12.0};
        }
        if (r_limit[1] < r_limit[0])
        {
            ROS_ERROR("Param GP2mid/r_limit max < min (%.3f < %.3f). Swap them.", r_limit[1], r_limit[0]);
            std::swap(r_limit[0], r_limit[1]);
        }
        if (r_limit[1] - r_limit[0] < 1e-6)
        {
            ROS_ERROR("Param GP2mid/r_limit too small: [%.6f, %.6f]. Fallback to [2,12].", r_limit[0], r_limit[1]);
            r_limit = {2.0, 12.0};
        }
        r_len = static_cast<int>((r_limit[1] - r_limit[0]) / r_grain_size) + 1;
        r_len = std::max(r_len, 2);
        r_range = Eigen::VectorXd::LinSpaced(r_len, r_limit[0], r_limit[1]);

        // Normalize all configured angles into [0, 2*pi) first for robustness (e.g. negative or >360deg inputs),
        // then handle wrap-around by unwrapping upper bound if needed.
        angle_ub_setting = wrapTo2Pi(deg2rad(angle_ub_deg));
        angle_lb_setting = wrapTo2Pi(deg2rad(angle_lb_deg));
        unwrapAngleInterval(angle_lb_setting, angle_ub_setting);

        angle_dig_ub1_setting = wrapTo2Pi(deg2rad(angle_dig_ub1_deg));
        angle_dig_lb1_setting = wrapTo2Pi(deg2rad(angle_dig_lb1_deg));
        angle_dig_ub2_setting = wrapTo2Pi(deg2rad(angle_dig_ub2_deg));
        angle_dig_lb2_setting = wrapTo2Pi(deg2rad(angle_dig_lb2_deg));

        unwrapAngleInterval(angle_dig_lb1_setting, angle_dig_ub1_setting);
        unwrapAngleInterval(angle_dig_lb2_setting, angle_dig_ub2_setting);

        normalizeAndClampDigRegion(angle_dig_lb1_setting, angle_dig_ub1_setting,
                                  angle_lb_setting, angle_ub_setting,
                                  "GP2mid/dig_region1");
        normalizeAndClampDigRegion(angle_dig_lb2_setting, angle_dig_ub2_setting,
                                  angle_lb_setting, angle_ub_setting,
                                  "GP2mid/dig_region2");
        angle_offset = deg2rad(angle_offset_deg);
        dig_region_margin = deg2rad(dig_region_margin_deg);
        danger_zone_margin = deg2rad(danger_zone_margin_deg);

        target_ub = tmp_target_ub;
        target_lb = tmp_target_lb;
        if (target_ub < target_lb)
        {
            ROS_ERROR("Param GP2mid/target_ub < target_lb (%.3f < %.3f). Swap them.", target_ub, target_lb);
            std::swap(target_ub, target_lb);
        }

        edge_extend = tmp_edge_extend;
        extend_angle = deg2rad(extend_angle_deg);
        truck_angle_prot = deg2rad(truck_angle_prot_deg);
        expnt = tmp_expnt;
        diggable_threshold = tmp_diggable_threshold;
        digx_min = tmp_digx_min;
        turn_point = tmp_turn_point;
    }
    
    // publish msg
    GPvisual_publisher = nh.advertise<visualization_msgs::Marker>("GP_visual",10);
    RealBkt_publisher = nh.advertise<visualization_msgs::Marker>("GP_RealBkt",1);
    RefBkt_publisher = nh.advertise<visualization_msgs::Marker>("GP_RefBkt",1);
    flagGPfinish_publisher = nh.advertise<std_msgs::Float64>("Sys_RGp_FlagGpPlanFinish",1);
    SwWkAgl_publisher = nh.advertise<geometry_msgs::Point>("Sys_RGp_SwWkAgl",1);
    DigStartDepth_publisher = nh.advertise<geometry_msgs::Point>("Sys_RGp_DigStartDepth",1);
    terrainZ_publisher = nh.advertise<std_msgs::Float64MultiArray>("Sys_RGp_InterpZ",1);
    groundZ_publisher = nh.advertise<std_msgs::Float64MultiArray>("Sys_RGp_Ground",1);
    AreaUbLb_publisher = nh.advertise<geometry_msgs::Point>("Sys_RGp_AreaUbLb",1);
    VolTrack_publisher = nh.advertise<geometry_msgs::Point>("Sys_RGp_Vol",1);
    GUIparam_publisher = nh.advertise<std_msgs::Float64MultiArray>("GPdbg_GUIparam",1);
    GPempty_publisher = nh.advertise<geometry_msgs::Point>("GPdbg_EmptyVolm",1);
    BktAgl_publisher = nh.advertise<geometry_msgs::Point>("Sys_RGp_BktAgl",1);
    ErrorFlag_publisher = nh.advertise<geometry_msgs::Point>("Safety_GP",1);
    HeartFault_publisher = nh.advertise<std_msgs::UInt32>("Safety_GP_HeartFault", 1);
    autoSetDigRegionResult_publisher = nh.advertise<std_msgs::Bool>("/dig_region_update_done", 1);
    digRegionReferenceAngles_publisher = nh.advertise<geometry_msgs::Point>("/dig_region_reference_angles", 1);
    Unload_publisher = nh.advertise<geometry_msgs::Point>("Sys_RGp_Unload", 1);
    DigJoints_publisher = nh.advertise<geometry_msgs::Quaternion>("/GP_dig_joints", 1);
    
    #ifndef DEBUG_MODE
    // subscribe from GUIs
    // D0_subscriber=nh.subscribe("D0",1,&GP::D0_callback,this);
    // E0_subscriber=nh.subscribe("E0",1,&GP::E0_callback,this);
    // F0_subscriber=nh.subscribe("F0",1,&GP::F0_callback,this);
    // F6_subscriber=nh.subscribe("F6",1,&GP::F6_callback,this); // 订阅挖掘区1
    // F7_subscriber=nh.subscribe("F7",1,&GP::F7_callback,this); // 订阅挖掘区2
    // D1_subscriber=nh.subscribe("D1",1,&GP::D1_callback,this);
    // E1_subscriber=nh.subscribe("E1",1,&GP::E1_callback,this);
    // F1_subscriber=nh.subscribe("F1",1,&GP::F1_callback,this);
    // O4_subscriber=nh.subscribe("O4",1,&GP::O4_callback,this);
#endif

    // subscribe from IMU
    jointsAngle_subscriber = nh.subscribe("joints_angle",1,&GP::jointsAngle_callback,this);
    // armPitch_subscriber = nh.subscribe("imu_arm_pitch_topic",1,&GP::armPitch_callback,this);
    // bktPitch_subscriber = nh.subscribe("imu_bucket_pitch_topic",1,&GP::bktPitch_callback,this);
    truckVertices_subscriber = nh.subscribe("truck_vertices_topic",1,&GP::truckVertices_callback,this);
    truckCenter_subscriber = nh.subscribe("truck_center_unloadpoint",1,&GP::truckCenter_callback,this);
    platformPitch_subscriber = nh.subscribe("imu_platform_pitch_topic",1,&GP::platformPitch_callback,this);
    swing_subscriber = nh.subscribe("Swing_topic",1,&GP::swing_callback,this);
    deviceRef_subscriber = nh.subscribe("Sys_RefDeviceTraj",1,&GP::deviceRef_callback,this);

    // subcribe gridmap
    gridMap_subscriber = nh.subscribe("excavation_gridmap1",1,&GP::gridMap_callback,this);

    // subscribe ground dectection
    groundDectection_subscriber = nh.subscribe("Sys_RSe_GroundDepth",1,&GP::groundDectection_callback,this);

    rtkAlt_subscriber = nh.subscribe("/rtk_can_msg", 1, &GP::rtkAlt_callback, this);
    mpcSwitch_subscriber = nh.subscribe("/Mpc_Switch", 1, &GP::mpcSwitch_callback, this);
    mpc2Switch_subscriber = nh.subscribe("/Mpc_Switch2", 1, &GP::mpc2Switch_callback, this);

    nh.param("GP2mid/unload_distance", unload_distance, unload_distance);
    nh.param("GP2mid/unload_height", unload_height, unload_height);
    autoSetDigRegion_subscriber = nh.subscribe(auto_dig_region_trigger_topic, 1, &GP::autoSetDigRegion_callback, this);

    truckPose_subscriber = nh.subscribe("/truck_pose_topic", 1, &GP::truckPose_callback, this);

    // subscribe from system decision
    sysSeq_subscriber = nh.subscribe("Sys_Seq",1,&GP::sysSeq_callback,this);        
    planFlag_subscriber = nh.subscribe("Sys_SGp_FlagGpPlan",1,&GP::planFlag_callback,this);

    // boot info
    initMsg();
    std::cout << "##############################" << std::endl << std::endl;
    std::cout << GP_VERSION << " launched! " ;
    #ifdef DEBUG_MODE
        std::cout << "DEBUG MODE is on!" ;
    #endif
    std::cout << std::endl << std::endl;
    std::cout << "##############################" << std::endl << std::endl;
}
#pragma endregion


#pragma region cal& run
void GP::calculate()
{
    anormaly_check();
    map_construct();
    gridmap_correct();
    area_split();
    angle_split();
    edge_calculation();
    trackdist_calculation();
    terrain_section();
    ground_calculation();

    if (mpc2_switch)
    {
        dig_flag = -4;
        vol_pred = 0;
        return;
    }
    onestep_choose();

    if (dig_flag == -3 )
        addFaultCode(FAULT_3);
    else
        clearFaultCode(FAULT_3);
}

void GP::run()
{
    while (ros::ok())
    {
        ros::spinOnce();
        updateAndPublishHeartAndFault();
        // Ensure unload computation runs even if calculate() isn't triggered
        if (mpc2_switch && unload_compute_pending)
        {
            // Prepare data if not ready
            const bool terrain_ok = terrain_mat.rows() == r_len && terrain_mat.cols() == N_fans;
            const bool ground_ok = ground_mat.rows() == r_len && ground_mat.cols() == N_fans;
            const bool theta_ok = static_cast<int>(theta_vec.size()) == N_fans;
            if (!terrain_ok || !ground_ok || !theta_ok)
            {
                anormaly_check();
                map_construct();
                gridmap_correct();
                area_split();
                angle_split();
                edge_calculation();
                trackdist_calculation();
                terrain_section();
                ground_calculation();
            }
            computeUnloadPlan();
        }
        if (mpc2_switch && unload_solution_valid)
        {
            Unload_publisher.publish(Unload_msg);
        }
        if (dig_region_ref_angles_publish_count > 0)
        {
            digRegionReferenceAngles_publisher.publish(dig_region_ref_angles_msg);
            dig_region_ref_angles_publish_count--;
        }
        rt.sleep();
    }
}

void GP::addFaultCode(FaultCode fault)
{
    fault_mask_ |= static_cast<uint16_t>(fault);
}

void GP::clearFaultCode(FaultCode fault)
{
    fault_mask_ &= ~static_cast<uint16_t>(fault);
}

void GP::computeUnloadPlan()
{
    unload_solution_valid = false;
    const bool terrain_ok = terrain_mat.rows() == r_len && terrain_mat.cols() == N_fans;
    const bool ground_ok = ground_mat.rows() == r_len && ground_mat.cols() == N_fans;
    const bool theta_ok = static_cast<int>(theta_vec.size()) == N_fans;
    if (!terrain_ok || !ground_ok || !theta_ok)
    {
        ROS_WARN("computeUnloadPlan(): terrain data is not ready, keep pending. terrain=(%ld,%ld), ground=(%ld,%ld), theta=%ld, expected=(%d,%d)",
                 static_cast<long>(terrain_mat.rows()), static_cast<long>(terrain_mat.cols()),
                 static_cast<long>(ground_mat.rows()), static_cast<long>(ground_mat.cols()),
                 static_cast<long>(theta_vec.size()), r_len, N_fans);
        return;
    }

    const double lb1 = angle_dig_lb1_setting;
    const double ub1 = angle_dig_ub1_setting;
    const double lb2 = angle_dig_lb2_setting;
    const double ub2 = angle_dig_ub2_setting;

    auto in_interval = [](double a, double lb, double ub) -> bool {
        return a >= lb && a <= ub;
    };

    auto accum_region_volume = [&](double lb, double ub) -> double {
        double sum = 0.0;
        for (int dir_id = 0; dir_id < static_cast<int>(theta_vec.size()); ++dir_id)
        {
            const double th = theta_vec[dir_id];
            if (!in_interval(th, lb, ub)) continue;
            // Sum positive terrain above ground with protection
            Eigen::VectorXd groundZ = ground_mat.col(dir_id);
            Eigen::VectorXd terrainZ = terrain_mat.col(dir_id);
            for (int j = 0; j < terrainZ.size(); ++j)
            {
                const double upper = terrainZ[j];
                const double lower = std::max(groundZ[j] + ground_crash_prot, -std::numeric_limits<double>::infinity());
                const double incr = std::max(0.0, upper - lower) * r_grain_size * bkt_width;
                sum += incr;
            }
        }
        return sum;
    };

    const double vol1 = (lb1 <= ub1) ? accum_region_volume(lb1, ub1) : std::numeric_limits<double>::infinity();
    const double vol2 = (lb2 <= ub2) ? accum_region_volume(lb2, ub2) : std::numeric_limits<double>::infinity();

    // Choose region with smaller volume
    const bool choose1 = (vol1 <= vol2);
    const double sel_lb = choose1 ? lb1 : lb2;
    const double sel_ub = choose1 ? ub1 : ub2;
    const double center = 0.5 * (sel_lb + sel_ub);

    unload_angle_deg = rad2deg(center);
    Unload_msg.x = unload_angle_deg;
    Unload_msg.y = unload_distance;
    Unload_msg.z = unload_height;
    unload_solution_valid = std::isfinite(unload_angle_deg);
    if (unload_solution_valid)
        unload_compute_pending = false;

    const int chosen_region = choose1 ? 1 : 2;
    ROS_INFO("computeUnloadPlan(): vol1=%.6f, vol2=%.6f, chosen=region%d, angle=%.3f deg, dist=%.3f, height=%.3f",
             vol1, vol2, chosen_region, unload_angle_deg, unload_distance, unload_height);
}

uint32_t GP::assembleHeartAndFault(uint16_t heart_value, uint16_t fault_mask) const
{
    const uint32_t high_part = static_cast<uint32_t>(heart_value) << 16;
    return high_part | static_cast<uint32_t>(fault_mask);
}

void GP::updateAndPublishHeartAndFault()
{
    // Heartbeat: 0..255
    heart_value_ = static_cast<uint16_t>((heart_value_ + 1) & 0x00FF);

    HeartFault_msg.data = assembleHeartAndFault(heart_value_, fault_mask_);
    HeartFault_publisher.publish(HeartFault_msg);
}

void GP::anormaly_check()
{
    gridexist_errorflag = 0;
    gridheight_errorflag = 0;
    gridlabel_errorflag = 0;
    
    if (elevmat.array().isNaN().count()>=elevmat.size()*0.9)
    {
        ROS_ERROR("Gridmap: Too Many NaNs!");
        gridexist_errorflag = 1;
    }

    for (int i=0; i< elevmat.size();i++)
    {
        if (!std::isnan(elevmat(i)) && !std::isnan(labelmat(i)))
        {
            if (elevmat(i)>10 || elevmat(i)<-10)
            {
                gridheight_errorflag = 1;
            }
            if (labelmat(i)>10 || labelmat(i)<-10 )
            {
                gridlabel_errorflag = 1;
            }
        }
    } 

    if (gridheight_errorflag)   ROS_ERROR("Gridmap: Height out of range!");
    if (gridlabel_errorflag)   ROS_ERROR("Gridmap: Label out of range!");

    if (gridexist_errorflag)
        addFaultCode(FAULT_0);
    else
        clearFaultCode(FAULT_0);

    if (gridheight_errorflag)
        addFaultCode(FAULT_1);
    else
        clearFaultCode(FAULT_1);

    if (gridlabel_errorflag)
        addFaultCode(FAULT_2);
    else
        clearFaultCode(FAULT_2);
}
#pragma endregion

void GP::map_construct()
{
    // Use elevmat's actual shape to avoid size mismatch / indexing issues when lengthX/lengthY are not symmetric.
    const int m = static_cast<int>(elevmat.rows());
    const int n = static_cast<int>(elevmat.cols());

    xgridmat = Eigen::MatrixXd::Zero(m, n);
    ygridmat = Eigen::MatrixXd::Zero(m, n);

    if (m <= 0 || n <= 0)
    {
        return;
    }

    // meshgrid: x varies across columns, y varies across rows
    const Eigen::VectorXd x_coords = Eigen::VectorXd::LinSpaced(
        n,
        ori_coord_x - lengthX / 2.0 + resolution / 2.0,
        ori_coord_x + lengthX / 2.0 - resolution / 2.0);
    const Eigen::VectorXd y_coords = Eigen::VectorXd::LinSpaced(
        m,
        ori_coord_y - lengthY / 2.0 + resolution / 2.0,
        ori_coord_y + lengthY / 2.0 - resolution / 2.0);

    for (int i = 0; i < m; ++i)
    {
        xgridmat.row(i) = x_coords.transpose();
    }
    for (int j = 0; j < n; ++j)
    {
        ygridmat.col(j) = y_coords;
    }
}

#pragma region area split
void GP::area_split()   // % determine digging area
                    // angle_lb, angle_ub: maximum digging area angle
                    // area_ub, area_lb: chosen digging area angle
{
    area_lb = angle_lb_setting;
    area_ub = angle_ub_setting;
        
}
#pragma endregion

#pragma region angle_split
void GP::angle_split()   // split angles within areas into N_fans parts 
{
    // we split each area into N_fans equal parts, each part corresponding to one fan
    flag1_vec.assign(N_fans,true);
    theta_vec.assign(N_fans,0);

    // we first enumerate all possible angles
    for (int j=0; j<N_fans; j++)
    {
        theta_vec[j] = area_lb + (area_ub - area_lb)*(2*j + 1)/2/N_fans;
    }
    // then, we sort the angles within each fan, with the largest angle comes first
    sort(theta_vec.begin(),theta_vec.end(),[this](auto a,auto b){return abs(angle_ub_setting-a)<abs(angle_ub_setting-b);});

}
#pragma endregion

#pragma region edge calc
void GP::edge_calculation()     // calculate the edge distance
{
    edge_dist_vec.assign(N_fans,12);         // edge distance for each fan
}
#pragma endregion

#pragma region trackdist_calc
void GP::trackdist_calculation()    // calculate track protection distance of each direction
{
    track_dist_vec.assign(N_fans,trackdist_def);
}
#pragma endregion

#pragma region terrain_sect
void GP::terrain_section()          // calculate the terrain height in one direction
{
    terrain_mat.resize(r_len,N_fans);  // each col stores terrain height of one direction
    terrain_mat.setZero();
    Eigen::MatrixXd x_mat(r_len,N_fans);
    Eigen::MatrixXd y_mat(r_len,N_fans);
    for (int i=0; i<N_fans; i++)
    {
        x_mat.col(i) = r_range * cos(theta_vec[i]-angle_offset);
        y_mat.col(i) = r_range * sin(theta_vec[i]-angle_offset);
    }

    terrain_mat = interp2(elevmat.cast<double>(), xgridmat, ygridmat, x_mat, y_mat) - Eigen::ArrayXXd::Ones(r_len,N_fans)*boom_pos[2];     // 2d interpolation
    fillnan(terrain_mat, expnt);            // blind zone fill missing
    
    // smoothing
    int windowSize = std::ceil(2*atan2(bkt_width/2,9)/(area_ub-area_lb)*N_fans);
    for (int j=0; j<r_len; j++)
    {
        terrain_mat.row(j) = (r_range[j] < 4) ? Eigen::VectorXd::Ones(N_fans,1) * (-base2ground-boom_pos[2]):  movmean(terrain_mat.row(j).array(),windowSize);
    }

    Eigen::ArrayXi noiseCounts = terrain_mat.array().unaryExpr([this](double x){return x>GP::height_threshold ? 1 : 0;}).colwise().sum();      // count pts with abnormal height
    Eigen::ArrayXi nanCounts = terrain_mat.array().unaryExpr([](double x){return std::isnan(x) ? 1 : 0;}).colwise().sum();          // count pts with nan value
    for (int i=0; i<N_fans; i++)            
    {
        if (noiseCounts[i]>0 || nanCounts[i]>0)
            flag1_vec[i] = false;       // if point with z>threshold exists, the direction is abandoned
        
    }
}
#pragma endregion

#pragma region ground_cal
void GP::ground_calculation()           // calculate the ground height in one direction
{
    double b = ground_coef[2];
    double c = ground_coef[3];
    ground_mat.resize(r_len,N_fans);     // each col stores ground height of one direction
    for (int j=0; j<N_fans; j++)
    {
        double angle = theta_vec[j] - angle_offset;
        double a = ground_coef[0]*cos(angle) + ground_coef[1]*sin(angle);
        ground_mat.col(j)=(-c-a*r_range.array())/b - boom_pos[2];
    }
}
#pragma endregion

#pragma region onestep choose
static std::pair<double, double> calcSwingOffsetBounds(
    double dig_swing,
    bool danger_ok,
    double danger_min,
    double danger_max,
    double default_opp_max,
    double danger_margin)
{
    double offset_lb = -default_opp_max;
    double offset_ub = default_opp_max;

    if (!danger_ok)
    {
        return {offset_lb, offset_ub};
    }

    const double dmin = angleDiffSigned(danger_min, dig_swing);
    const double dmax = angleDiffSigned(danger_max, dig_swing);
    const double abs_dmin = std::abs(dmin);
    const double abs_dmax = std::abs(dmax);

    // Choose nearest boundary and set asymmetric bounds.
    const bool nearest_is_min = (abs_dmin <= abs_dmax);
    const double toward_signed = nearest_is_min ? dmin : dmax;
    const double toward_max = std::min(std::abs(toward_signed) + danger_margin, default_opp_max);

    // Opposite-direction max offset is limited by the directed distance to the other boundary.
    const double other_boundary = nearest_is_min ? danger_max : danger_min;
    // wrapTo2Pi(x - y) gives CCW distance from y to x in [0, 2pi).
    const double opp_dist = (toward_signed >= 0)
        ? wrapTo2Pi(dig_swing - other_boundary)   // opposite dir is CW
        : wrapTo2Pi(other_boundary - dig_swing);  // opposite dir is CCW
    const double opp_max = std::min(opp_dist + danger_margin, default_opp_max);

    if (toward_signed >= 0)
    {
        offset_lb = -opp_max;
        offset_ub = toward_max;
    }
    else
    {
        offset_lb = -toward_max;
        offset_ub = opp_max;
    }

    return {offset_lb, offset_ub};
}

bool GP::validateOnestepChooseInputs(std::string& err) const
{
    err.clear();

    // Hard guards to prevent Eigen out-of-range aborts when upstream data is invalid.
    if (r_len <= 1 || N_fans <= 0)
    {
        err = "invalid grid size r_len=" + std::to_string(r_len) + ", N_fans=" + std::to_string(N_fans);
        return false;
    }

    const bool terrain_ok = (terrain_mat.rows() == r_len && terrain_mat.cols() == N_fans);
    const bool ground_ok = (ground_mat.rows() == r_len && ground_mat.cols() == N_fans);
    if (!terrain_ok || !ground_ok)
    {
        err = "terrain/ground size mismatch. r_len=" + std::to_string(r_len) +
              " N_fans=" + std::to_string(N_fans) +
              " terrain=(" + std::to_string(static_cast<long>(terrain_mat.rows())) + "," + std::to_string(static_cast<long>(terrain_mat.cols())) + ")" +
              " ground=(" + std::to_string(static_cast<long>(ground_mat.rows())) + "," + std::to_string(static_cast<long>(ground_mat.cols())) + ")";
        return false;
    }

    if (r_range.size() != r_len)
    {
        err = "r_range size mismatch. r_len=" + std::to_string(r_len) + " r_range.size()=" + std::to_string(static_cast<long>(r_range.size()));
        return false;
    }

    if (static_cast<int>(theta_vec.size()) != N_fans)
    {
        err = "theta_vec size mismatch. N_fans=" + std::to_string(N_fans) + " theta_vec.size()=" + std::to_string(static_cast<long>(theta_vec.size()));
        return false;
    }

    if (static_cast<int>(flag1_vec.size()) != N_fans || static_cast<int>(edge_dist_vec.size()) != N_fans || static_cast<int>(track_dist_vec.size()) != N_fans)
    {
        err = "per-fan vector size mismatch. N_fans=" + std::to_string(N_fans) +
              " flag1=" + std::to_string(static_cast<long>(flag1_vec.size())) +
              " edge=" + std::to_string(static_cast<long>(edge_dist_vec.size())) +
              " track=" + std::to_string(static_cast<long>(track_dist_vec.size()));
        return false;
    }

    return true;
}

std::vector<std::vector<int>> GP::computeContiguousSafeBlocksByAngle(const std::vector<int>& safe_fan_ids,
                                                                     double area_lb,
                                                                     double area_ub) const
{
    std::vector<std::vector<int>> blocks;
    if (safe_fan_ids.empty())
        return blocks;

    std::vector<std::pair<double, int>> safe_by_angle;
    safe_by_angle.reserve(safe_fan_ids.size());
    for (int id : safe_fan_ids)
        safe_by_angle.emplace_back(theta_vec[id], id);

    std::sort(safe_by_angle.begin(), safe_by_angle.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    const double expected_step = (area_ub - area_lb) / static_cast<double>(std::max(1, N_fans));
    const double gap_thresh = expected_step * 1.5;

    blocks.reserve(safe_by_angle.size());
    blocks.emplace_back();
    blocks.back().push_back(safe_by_angle.front().second);
    for (size_t i = 1; i < safe_by_angle.size(); ++i)
    {
        const double prev_a = safe_by_angle[i - 1].first;
        const double curr_a = safe_by_angle[i].first;
        if ((curr_a - prev_a) > gap_thresh)
            blocks.emplace_back();
        blocks.back().push_back(safe_by_angle[i].second);
    }

    return blocks;
}

void GP::updateAndLogSafeSectorAngleRangesDeg(const std::vector<std::vector<int>>& blocks,
                                              double half_step_rad)
{
    safe_sector_angle_ranges_deg.clear();
    if (blocks.empty())
        return;

    if (!(half_step_rad > 0.0) || !std::isfinite(half_step_rad))
        half_step_rad = 0.0;

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << "safe blocks=" << blocks.size();
    for (size_t bi = 0; bi < blocks.size(); ++bi)
    {
        double min_ang = std::numeric_limits<double>::infinity();
        double max_ang = -std::numeric_limits<double>::infinity();
        for (int id : blocks[bi])
        {
            const double ang = theta_vec[id];
            const double left = rad2deg(ang - half_step_rad);
            const double right = rad2deg(ang + half_step_rad);
            min_ang = std::min(min_ang, left);
            max_ang = std::max(max_ang, right);
        }
        safe_sector_angle_ranges_deg.emplace_back(min_ang, max_ang);
        oss << "; [" << bi << "] ang[" << min_ang << "," << max_ang << "]deg";
    }
    ROS_INFO_THROTTLE(1.0, "%s", oss.str().c_str());
}

void GP::onestep_choose()
{
    if (mpc2_switch)
    {
        // unloading mode active: disable digging computation
        dig_flag = -4;
        vol_pred = 0;
        ROS_INFO_THROTTLE(1.0, "unload mode active, skip onestep_choose");
        return;
    }
    dig_flag = 1;

    std::string guard_err;
    if (!validateOnestepChooseInputs(guard_err))
    {
        dig_flag = -1;
        vol_pred = 0;
        ROS_ERROR("onestep_choose(): %s", guard_err.c_str());
        return;
    }

    int end_id = 30;
    end_id = std::min(std::max(end_id, 0), r_len - 1);
    double x_end = r_range[end_id];
    if (std::isnan(turn_point))
        turn_point = 0.5 * (r_limit[0] + r_limit[1]);
    turn_point = Saturate(turn_point, r_limit[1], r_limit[0]);
    diggable_threshold = Saturate(diggable_threshold, 1.0, 0.0);
    int turn_point_id = std::floor((turn_point-r_limit[0])/r_grain_size);
    turn_point_id = std::min(std::max(turn_point_id, 0), r_len - 1);
    double x_turnpoint = r_range[turn_point_id];

    dig_start = 9;
    dig_start_id = 140;
    dig_height = -5;
    dig_height_terrain = -4;
    dig_depth = 0;
    vol_pred = 0;
    dig_fan_start_id = 0;
    int bestdir_id = 0;
    bool isEmpty=true;

    double dig_start_alt = 9;
    double dig_start_id_alt = 140;
    double dig_height_alt = -5;
    double dig_height_terrain_alt = -4;
    double dig_depth_alt = 0;
    double vol_pred_alt = 0;
    double area_pred_alt = 0;
    Eigen::MatrixX3d traj_pred_alt;
    int bestdir_id_alt = N_fans - 1;
    bool isEmpty_alt=true;

    dig_bktangle = 0;
    dig_joints=inverse_kinematic(dig_start,dig_height,0).second;

    std::vector<int> safe_fan_ids;

    for (int raw_id = 0; raw_id < N_fans; ++raw_id)
    {
        const double sector_center = theta_vec[raw_id];
        const bool in_dig_area =
            (sector_center >= angle_dig_lb1_setting && sector_center <= angle_dig_ub1_setting) ||
            (sector_center >= angle_dig_lb2_setting && sector_center <= angle_dig_ub2_setting);

        if (!in_dig_area)
            continue;

        if (isInDangerZone(sector_center))
            continue;

        safe_fan_ids.push_back(raw_id);
    }

    const auto blocks = computeContiguousSafeBlocksByAngle(safe_fan_ids, area_lb, area_ub);
    const double expected_step = (area_ub - area_lb) / static_cast<double>(std::max(1, N_fans));
    updateAndLogSafeSectorAngleRangesDeg(blocks, 0.5 * expected_step);

    // If safe regions are split by the truck danger zone into multiple blocks, select the largest block.
    selectLargestContiguousSafeBlock(safe_fan_ids, blocks);

    if (safe_fan_ids.empty())
    {
        // All sectors are filtered out (e.g., fully occupied by danger zone). This is a valid runtime state;
        // avoid spamming ROS_ERROR every cycle.
        dig_flag = -3;
        vol_pred = 0;

        // Explicitly invalidate any previously latched solution so downstream publishing doesn't look like we still have a plan.
        const double qnan = std::numeric_limits<double>::quiet_NaN();
        dig_start = qnan;
        dig_start_id = -1;
        dig_height = qnan;
        dig_height_terrain = qnan;
        dig_depth = qnan;
        dig_swing = qnan;
        dig_bktangle = qnan;
        dig_trackdist = qnan;
        dig_edgedist = qnan;
        working_angle = qnan;
        dig_fan_start_id = -1;
        traj_pred.resize(0, 3);
        dig_terrainZ.resize(0);
        dig_groundZ.resize(0);

        ROS_WARN_THROTTLE(1.0, "onestep_choose(): no safe fan sectors (all filtered by dig area / danger zone). N_fans=%d", N_fans);
        return;
    }
    
    for (int dir_id : safe_fan_ids)
    {
        
        if (!flag1_vec[dir_id]) continue;
        Eigen::VectorXd groundZ = ground_mat.col(dir_id);
        Eigen::VectorXd terrainZ = terrain_mat.col(dir_id);
        double edgedist = edge_dist_vec[dir_id];
        double z_end = terrainZ[end_id];
        double swing_angle = theta_vec[dir_id];
        
        for (int start_id = floor(digx_min/r_grain_size); start_id < r_len; start_id++) 
        {
            double x_start = r_range[start_id];
            double z_start_terrain = terrainZ[start_id];
            double ground_level = groundZ[start_id];

            const double depth_min_allowed = (ground_level + ground_crash_prot) - z_start_terrain;
            const double depth_low = std::max(min_pt_height, depth_min_allowed);
            const double depth_high = std::max(max_pt_height, depth_low);

            for (double depth = depth_high; depth >= depth_low; depth = depth - 0.1)          // search for best dig_depth
            {
                double v_temp = 0;
                double z_start = z_start_terrain + depth;
                double z_turnpoint = std::max(groundZ[turn_point_id] + ground_crash_prot, z_start);

                if (x_start > edgedist + edge_extend || x_start <= x_end ||
                std::isnan(z_start))    // out of digging range
                {
                    continue;
                }
                std::pair<bool, std::vector<double>> result = inverse_kinematic(x_start,z_start,0);
                if (!result.first)            // inverse kinematic infeasible when bkt angle= 0
                {
                    continue;
                }
                if (arm_collision(result.second,terrainZ))      // arm will collide with terrain
                {
                    continue;
                }
                std::vector<double> trajZ;
                if (turn_point_id >= start_id)
                {
                    trajZ = ExpInterp(x_end,x_start,z_end,z_start,start_id-end_id+1,expnt);          // generate trajectory with exp function
                }
                else if (turn_point_id <= end_id)
                {
                    trajZ = LinSegment(x_end,x_start,groundZ[end_id],groundZ[start_id], start_id-end_id+1).second;   // generate trajectory equal to ground
                }
                else
                {
                    trajZ = ExpInterp(x_end,x_turnpoint,z_end,z_turnpoint,turn_point_id-end_id+1,expnt);    // generate first-half trajectory with exp function
                    std::vector<double> traj2 = LinSegment(x_turnpoint,x_start,z_turnpoint,z_start, start_id-turn_point_id+1).second;  // generate second-half trajectory with linear function
                    trajZ.insert(trajZ.end(),traj2.begin()+1,traj2.end());
                }
                
                Eigen::MatrixX3d traj_temp;
                traj_temp=Eigen::MatrixXd::Zero(trajZ.size(),3);
                for (int j=end_id; j<=start_id; j++)
                {
                    double upper = terrainZ[j];
                    double lower = std::max(trajZ[j-end_id],groundZ[j] + ground_crash_prot);
                    double incr = (upper - lower >= diggable_threshold) * (upper - lower);
                    v_temp = v_temp + incr * r_grain_size * bkt_width;
                    traj_temp(j-end_id,0)=r_range[j];
                    traj_temp(j-end_id,2)=lower;
                }

                // 检查是否超过target_ub
                if (v_temp > target_ub) 
                {
                    // 找到一个超过上限的解决方案，直接使用它并结束搜索
                    dig_start = x_start;
                    dig_start_id = start_id;
                    dig_height = z_start;
                    dig_height_terrain = z_start_terrain;
                    dig_depth = depth;
                    traj_pred = traj_temp;
                    vol_pred = v_temp;
                    bestdir_id = dir_id;
                    dig_fan_start_id = dir_id;
                    
                    // 直接跳转到最终赋值部分
                    goto FINAL_ASSIGNMENT;
                }

                if (v_temp >= vol_pred && v_temp > target_lb)           // find the pareto optimum which satisfies vol > target lb 
                {
                    dig_start = x_start;
                    dig_start_id = start_id;
                    dig_height = z_start;
                    dig_height_terrain = z_start_terrain;
                    dig_depth = depth;
                    traj_pred = traj_temp;
                    vol_pred = v_temp;
                    bestdir_id = dir_id;
                    dig_fan_start_id = dir_id; //
                    isEmpty = false;
                }
                if (v_temp > vol_pred_alt)                   // find max: vol
                {
                    dig_start_alt = x_start;
                    dig_start_id_alt = start_id;
                    dig_height_alt = z_start;
                    dig_height_terrain_alt = z_start_terrain;
                    dig_depth_alt = depth;
                    traj_pred_alt = traj_temp;
                    vol_pred_alt = v_temp;
                    bestdir_id_alt = dir_id;
                    isEmpty_alt = false;
                }
            }
        }
    }

    if (isEmpty_alt)          // no feasible soln: default soln
    {
        dig_start = 7;
        dig_height = -4;
        dig_height_terrain = -1.5;
        dig_depth = -2.5;
        traj_pred = Eigen::MatrixXd::Zero(1,3);
        vol_pred = 0;
        bestdir_id = safe_fan_ids.empty() ? 0 : safe_fan_ids.front();
        dig_fan_start_id =bestdir_id;
        dig_joints=inverse_kinematic(dig_start,dig_height,0).second;
        dig_flag = -1;
        ROS_ERROR("No feasible dig point! \n");
    }
    else if (isEmpty )       // gp low or clean area is small: max vol
    {
        dig_start = dig_start_alt;
        dig_start_id = dig_start_id_alt;
        dig_height = dig_height_alt;
        dig_height_terrain = dig_height_terrain_alt;
        dig_depth = dig_depth_alt;
        traj_pred = traj_pred_alt;
        vol_pred = vol_pred_alt;
        bestdir_id = bestdir_id_alt;
        dig_fan_start_id = bestdir_id_alt;
        if (vol_pred < target_lb)
        {
            dig_flag = -2;                    
            ROS_WARN("GP low! \n");
        }
    }
    if (vol_pred < target_ub&&vol_pred > target_lb)
    {                   
        ROS_WARN("No ub but More lb! \n");
    }

FINAL_ASSIGNMENT:
    dig_swing = theta_vec[bestdir_id];
    {
        static thread_local std::mt19937 gen(std::random_device{}());

        const double default_opp_max = deg2rad(20.0);
        const bool danger_ok = last_danger_valid && std::isfinite(last_danger_min) && std::isfinite(last_danger_max);
        const double danger_margin = std::max(0.0, danger_zone_margin);
        const auto bounds = calcSwingOffsetBounds(dig_swing, danger_ok, last_danger_min, last_danger_max, default_opp_max, danger_margin);
        const double offset_lb = bounds.first;
        const double offset_ub = bounds.second;

        if (danger_ok)
        {
            const double dmin = angleDiffSigned(last_danger_min, dig_swing);
            const double dmax = angleDiffSigned(last_danger_max, dig_swing);
            const double toward_signed = (std::abs(dmin) <= std::abs(dmax)) ? dmin : dmax;
            const char* dir_str = (toward_signed >= 0) ? "+ (CCW)" : "- (CW)";
            ROS_INFO_THROTTLE(1.0, "swing offset bounds: [%.3f, %.3f] deg, nearest boundary dir: %s, toward_dist=%.3f deg",
                              rad2deg(offset_lb), rad2deg(offset_ub), dir_str, rad2deg(std::abs(toward_signed)));
        }
        else
        {
            ROS_INFO_THROTTLE(1.0, "swing offset bounds: [%.3f, %.3f] deg (danger boundary invalid)", rad2deg(offset_lb), rad2deg(offset_ub));
        }

        std::uniform_real_distribution<double> dist(offset_lb, offset_ub);
        const double swing_offset = dist(gen);
        const double dig_swing_before_offset = dig_swing;
        dig_swing += swing_offset;
        ROS_INFO("dig_swing random offset: %.3f deg (%.6f rad)", swing_offset * 180.0 / M_PI, swing_offset);
        ROS_INFO("dig_swing: %.3f deg (%.6f rad) -> %.3f deg (%.6f rad)",
                 rad2deg(dig_swing_before_offset), dig_swing_before_offset,
                 rad2deg(dig_swing), dig_swing);
    }
    dig_swing = std::fmod(dig_swing, 2 * M_PI);
    if (dig_swing < 0) dig_swing += 2 * M_PI;

    {
        const double before = dig_swing;
        dig_swing = projectAngleIntoDigRegions(dig_swing,
                                               angle_dig_lb1_setting, angle_dig_ub1_setting,
                                               angle_dig_lb2_setting, angle_dig_ub2_setting,
                                               dig_region_margin);
        if (std::abs(angleDiffSigned(dig_swing, before)) > 1e-9)
        {
            ROS_WARN_THROTTLE(1.0, "dig_swing adjusted into dig region: %.3f deg -> %.3f deg", rad2deg(before), rad2deg(dig_swing));
        }
    }

    dig_terrainZ = terrain_mat.col(bestdir_id);
    dig_groundZ = ground_mat.col(bestdir_id);
    dig_trackdist = track_dist_vec[bestdir_id];
    dig_edgedist = edge_dist_vec[bestdir_id];
    working_angle = -atan((dig_groundZ[r_len-1]-dig_groundZ[0])/(r_limit[1]-r_limit[0]));
    bool joints_flag=false;

    float target_angle = atan2(dig_height,dig_start);           // bkt teeth, bkt joint, boom joint as close to one line as possible 
    float dif = 999;
    for (float angle = bkt_angle_limit[0];angle < bkt_angle_limit[1];angle = angle+deg2rad(1))
    {
        std::pair<bool, std::vector<double>> result = inverse_kinematic(dig_start,dig_height,angle); 
        if (abs(result.second[0] + result.second[1] + result.second[2]- target_angle) < dif && result.first && !arm_collision(result.second,dig_terrainZ))
        {
            dif = abs(result.second[0] + result.second[1] + result.second[2]- target_angle);
            dig_bktangle = angle;   
            dig_joints = result.second;
            joints_flag=true;
        }
    }
    if (!joints_flag)
    {
        dig_joints = inverse_kinematic(dig_start,dig_height,0).second;
    }

    
}
#pragma endregion

bool GP::isInDangerZone(double angle){
    angle = std::fmod(angle, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;

    auto normAngle = [](double a)
    {
        a = std::fmod(a, 2 * M_PI);
        if (a < 0) a += 2 * M_PI;
        return a;
    };
    auto angleDiffSignedPi = [](double a, double b)
    {
        // minimal signed difference from b to a, in [-pi, pi]
        double d = std::fmod(a - b + M_PI, 2 * M_PI);
        if (d < 0) d += 2 * M_PI;
        return d - M_PI;
    };

    const double front = normAngle(truck_angle_front);
    const double back = normAngle(truck_angle_back);
    const double mid = normAngle(truck_angle_mid);

    // truck occupied angular width (raw)
    const double half_raw = 0.5 * std::abs(angleDiffSignedPi(front, back));
    const double half_danger = half_raw + std::max(0.0, truck_angle_prot);

    double raw_min = 0;
    double raw_max = 0;
    bool raw_is_continuous = true;
    if (half_raw >= M_PI)
    {
        raw_min = 0;
        raw_max = 2 * M_PI;
        raw_is_continuous = true;
    }
    else
    {
        const double raw_start = normAngle(mid - half_raw);
        const double raw_end = normAngle(mid + half_raw);
        raw_is_continuous = (raw_start <= raw_end);
        raw_min = std::min(raw_start, raw_end);
        raw_max = std::max(raw_start, raw_end);
    }

    double danger_min = 0;
    double danger_max = 0;
    bool danger_is_continuous = true;
    if (half_danger >= M_PI)
    {
        // Protection is so large that the danger zone covers the full circle.
        danger_min = 0;
        danger_max = 2 * M_PI;
        danger_is_continuous = true;
    }
    else
    {
        const double avoid_start = normAngle(mid - half_danger);
        const double avoid_end = normAngle(mid + half_danger);
        danger_is_continuous = (avoid_start <= avoid_end);
        danger_min = std::min(avoid_start, avoid_end);
        danger_max = std::max(avoid_start, avoid_end);
    }

    last_danger_min = danger_min;
    last_danger_max = danger_max;
    last_danger_valid = std::isfinite(danger_min) && std::isfinite(danger_max);

    (void)mid;

    updateThetaTruckVec(raw_min, raw_max, raw_is_continuous, thetaTruckRaw_vec);
    updateThetaTruckVec(danger_min, danger_max, danger_is_continuous, thetaTruck_vec);

    if(danger_is_continuous){
        // 返回卡车占据的危险角度
        return (angle >= danger_min && angle <= danger_max);
    }else{
        return (angle <= danger_min) || (angle >= danger_max);
    }
}

void GP::updateThetaTruckVec(double danger_min, double danger_max, bool danger_is_continuous, std::vector<double>& theta_truck_vec){
    // 将卡车占据角度分解成点集，方便之后显示
    theta_truck_vec.assign(2*N_fans,0);

    if(danger_is_continuous){
        for (int j=0; j<2*N_fans; j++){
            theta_truck_vec[j] = danger_min + j*(danger_max - danger_min)/static_cast<double>(2*N_fans - 1);
        }
    }else{
        // 将thetaTruck_vec拆分成两个向量表示
        std::vector<double> thetaTruck_vec1(N_fans);
        std::vector<double> thetaTruck_vec2(N_fans);
        for (int j=0; j<N_fans; j++){
            thetaTruck_vec1[j] = danger_max + j*(2*M_PI - danger_max)/static_cast<double>(N_fans - 1);
        }

        for (int j=0; j<N_fans; j++){
            thetaTruck_vec2[j] = 0 + j*(danger_min - 0)/static_cast<double>(N_fans - 1);
        }
        // 填充thetaTruck_vec
        for (int i = 0; i < N_fans; i++) {
            theta_truck_vec[i] = thetaTruck_vec1[i];      // 前半部分
            theta_truck_vec[i + N_fans] = thetaTruck_vec2[i];  // 后半部分
        }
    }

    // 对thetaTruck_vec减去坐标系偏移angle_offset
    for (int j=0; j<2*N_fans; j++){
        theta_truck_vec[j] -= angle_offset;
        if(theta_truck_vec[j] < 0 ){
            theta_truck_vec[j] += 2*M_PI;
        }
    }
}

#pragma region interp2
Eigen::ArrayXXd GP::interp2(const Eigen::MatrixXd& z_mat, const Eigen::MatrixXd& x_mat, const Eigen::MatrixXd& y_mat, const Eigen::MatrixXd& xq_mat, const Eigen::MatrixXd& yq_mat)
{
    /// \brief Interpolate a 2D function from a grid to query points.
    /// \param[in] z_mat M*N matrix of known function values.
    /// \param[in] x_mat M*N matrix of known x values (first column should be the same as xq_mat).
    /// \param[in] y_mat M*N matrix of known y values (first row should be the same as yq_mat).
    /// \param[in] xq_mat P*Q matrix of query x values.
    /// \param[in] yq_mat P*Q matrix of query y values.
    /// \return Interpolated values at the query points.
    
    int nrow = xq_mat.rows();
    int ncol = xq_mat.cols();

    // Check if the input matrices have the correct dimensions
    if (xq_mat.size() != yq_mat.size() || z_mat.size() != x_mat.size() || z_mat.size() != y_mat.size() || x_mat.size() != y_mat.size() && mpc_switch_interp)
    {
        ROS_WARN("Interp error!");
        flag1_vec.assign(false,N_fans);
        return Eigen::ArrayXXd::Constant(nrow,ncol,std::numeric_limits<double>::quiet_NaN());
    }

    Eigen::ArrayXXd out(nrow,ncol);
 
    double x0 = x_mat(0,0);
    double y0 = y_mat(0,0);
    
    for (int i=0; i<nrow; i++)
    {
        for (int j=0; j<ncol; j++)
        {
            double x = xq_mat(i,j);
            double y = yq_mat(i,j);
            int col1 = std::floor((x-x0)/resolution);
            int col2 = std::ceil((x-x0)/resolution);
            int row1 = std::floor((y-y0)/resolution);
            int row2 = std::ceil((y-y0)/resolution);

            if (col1<0 || row1<0 || col2>=z_mat.cols() || row2>= z_mat.rows())
            {
                out(i,j) = std::numeric_limits<double>::quiet_NaN();
                continue;
            }

            double x1 = x_mat(row1,col1);
            double x2 = x_mat(row1,col2);
            double y1 = y_mat(row1,col1);
            double y2 = y_mat(row2,col1);

            double q11 = z_mat(row1,col1);
            double q12 = z_mat(row1,col2);
            double q21 = z_mat(row2,col1);
            double q22 = z_mat(row2,col2);
            
            // Linear interpolation
            double R1 = q11 * (x2 - x)/resolution + q21 * (x - x1)/resolution;
            double R2 = q12 * (x2 - x)/resolution + q22 * (x - x1)/resolution;

            out(i,j) = R1 * (y2 - y)/resolution + R2 * (y - y1)/resolution;
        }
    }
    return out;
}
#pragma endregion

#pragma region inverse_kinematic
std::pair<bool, std::vector<double>> GP::inverse_kinematic(double x_start,double z_start, double bucket_angle) 
{
    bool feasibility = true;
    std::vector<double> joint_angle = {boom_angle_limit[1],arm_angle_limit[1],bkt_angle_limit[1]};

    // Calculating distances and angles for inverse kinematics
    double theta_FQV = (bucket_angle <= 0) ? M_PI + bucket_angle : M_PI - bucket_angle;
    double l_FV = sqrt(pow(l_FQ, 2) + pow(l_QV, 2) - 2 * cos(theta_FQV) * l_FQ * l_QV);
    double l_CV = sqrt(pow(x_start, 2) + pow(z_start, 2));
    double cos_FCV = (pow(l_CF, 2) + pow(l_CV, 2) - pow(l_FV, 2)) / (2 * l_CF * l_CV);
    double cos_CFV = (pow(l_CF, 2) + pow(l_FV, 2) - pow(l_CV, 2)) / (2 * l_CF * l_FV);

    if (abs(cos_FCV) > 1  || abs(cos_CFV) > 1) 
        return {false, joint_angle};

    double theta_FCV = acos(cos_FCV);
    double theta_CFV = acos(cos_CFV);

    // Calculate final joint angles
    double boom_angle = theta_FCV + atan2(z_start, x_start);
    double theta_VFQ = (abs(bucket_angle) < 1e-5) ? 0 : acos((pow(l_FV, 2) + pow(l_FQ, 2) - pow(l_QV, 2)) / (2 * l_FV * l_FQ));
    double arm_angle = (bucket_angle <= 0) ? theta_CFV + theta_VFQ - M_PI : theta_CFV - theta_VFQ - M_PI;

    // Validate angles against joint limits
    if (!(boom_angle >= boom_angle_limit[0] && boom_angle <= boom_angle_limit[1] &&
        arm_angle >= arm_angle_limit[0] && arm_angle <= arm_angle_limit[1] &&
        bucket_angle >= bkt_angle_limit[0] && bucket_angle <= bkt_angle_limit[1])) {
        return {false, joint_angle};
    }

    joint_angle = {boom_angle, arm_angle, bucket_angle};
    return {true, joint_angle};
}
#pragma endregion

#pragma region fillnan
void GP::fillnan(Eigen::MatrixXd& mat,double expnt)         // fill nan values in gridmap
{
    int ncol = mat.cols();
    int nrow = mat.rows();

    for (int col = 0; col < ncol; col++)
    {
        int begin_idx = 0;
        int end_idx = 0;
        for (int row = 0; row < nrow - 1; row++)
        {
            if (!std::isnan(mat(row,col)) && std::isnan(mat(row+1,col)) )       // find begin pos for nan intervals
                begin_idx = row;
            if (std::isnan(mat(row,col)) && !std::isnan(mat(row+1,col)) )       // find end pos for nan intervals
            {
                end_idx = row + 1;
                if (begin_idx == 0)                             // for the first nan inverval [0 , xn], use xn as value
                    mat.block(begin_idx,col,end_idx-begin_idx+1,1) = Eigen::MatrixXd::Ones(end_idx-begin_idx+1,1) * mat(end_idx,col);
                else                                            // for [xn,  xn+k] nan interval, use exponential interpolation
                {
                    std::vector<double> vec = ExpInterp(static_cast<double>(begin_idx),static_cast<double>(end_idx),
                                                                    mat(begin_idx,col),mat(end_idx,col),end_idx-begin_idx+1 ,expnt);
                    mat.block(begin_idx,col,end_idx-begin_idx+1,1) = Eigen::Map<Eigen::ArrayXd>(vec.data(),vec.size());
                }
            }
            if (row == nrow - 2 && end_idx < begin_idx)        // for [xn , xend] nan interval, use xn as value
                mat.block(begin_idx,col,nrow-begin_idx,1) = Eigen::MatrixXd::Ones(nrow-begin_idx,1) * mat(begin_idx,col);
        }
    }
}

#pragma endregion

#pragma region gridmap_correct
void GP::gridmap_correct()
{
    // std::cout << tan(pitch_error) << std::endl;
    elevmat = elevmat - xgridmat.cast<float>()*tan(pitch_error);
}
#pragma endregion

#pragma region arm collide
// Check if the arm collides with the earth
bool GP::arm_collision(const std::vector<double>& joint_angles, const Eigen::VectorXd& terrainZ)   // whether arm collides with earth
{
    double angle0 = joint_angles[0];
    double angle1 = angle0 + joint_angles[1];

    // Calculate the positions of the arm joints
    double x_F = l_CF * cos(angle0);            // arm joint position
    double z_F = l_CF * sin(angle0);
    double x_Q = x_F + l_FQ * cos(angle1);      // bkt joint position
    double z_Q = z_F + l_FQ * sin(angle1);

    // If the arm is not vertical, check if any point in the line segment is below the terrain
    if (x_F != x_Q)
    {
        double k = (z_Q - z_F)/(x_Q - x_F);     // fitted arm straight line (not vertical)
        double b = z_Q - k * x_Q;

        for (int i = 0; i < r_len; i++)
        {
            if ((x_F - r_range[i]) * (x_Q - r_range[i]) <= 0 && terrainZ[i] + ground_crash_prot > k * r_range[i] + b)   // any point in line segment is lower than terrain
                return true;
        }
    }
    else            // vertical line
    {
        for (int i = 0; i< r_len - 1; i++)
        {
            if ((r_range[i] - x_F) * (r_range[i+1] - x_F) <= 0 && std::max(terrainZ[i],terrainZ[i+1]) + ground_crash_prot > std::min(z_F,z_Q))
                return true;
        }
    }
    return false;
}
#pragma endregion

#pragma region kinematics
Eigen::Matrix3d GP::getDeviceRealPos()          // get device joint position in base coordinate from joint angles
{
    double angle0 = dig_joints[0];          // joint angle 0
    double angle1 = angle0 + dig_joints[1]; // joint angle 1
    double angle2 = angle1 + dig_joints[2]; // joint angle 2

    Eigen::AngleAxisd rotation(dig_swing - angle_offset,Eigen::Vector3d::UnitZ());
    Eigen::Translation3d trans(boom_pos[0],boom_pos[1],boom_pos[2]);
    Eigen::Matrix3d result=Eigen::Matrix3d::Zero();     // create a zero matrix of 3x3

    result(0,0) = l_CF * cos(angle0);        // result matrix element 00
    result(0,2) = l_CF * sin(angle0);        // result matrix element 02
    result(1,0) = result(0,0) + l_FQ * cos(angle1); // result matrix element 10
    result(1,2) = result(0,2) + l_FQ * sin(angle1); // result matrix element 12
    result(2,0) = result(1,0) + l_QV * cos(angle2); // result matrix element 20
    result(2,2) = result(1,2) + l_QV * sin(angle2); // result matrix element 22
    // transpose the result matrix
    result.transposeInPlace();

    // multiply the result matrix by the rotation and translation matrices
    result = rotation * trans * result;
    return result;
}

Eigen::Vector3d GP::getDeviveRefPos()
{
    double swing_ref = swing_joint_ref[0];
    double angle0_ref = swing_joint_ref[1];
    double angle1_ref = angle0_ref + swing_joint_ref[2];
    double angle2_ref = angle1_ref + swing_joint_ref[3];

    Eigen::AngleAxisd rotation(swing_ref - angle_offset,Eigen::Vector3d::UnitZ());
    Eigen::Translation3d trans(boom_pos[0],boom_pos[1],boom_pos[2]);
    Eigen::Vector3d result=Eigen::Vector3d::Zero();     // create a zero matrix of 3x3

    result[0] = l_CF * cos(angle0_ref) + l_FQ * cos(angle1_ref) + l_QV * cos(angle2_ref);        // result matrix element 0
    result[1] = 0;        // result matrix element 1
    result[2] = l_CF * sin(angle0_ref) + l_FQ * sin(angle1_ref) + l_QV * sin(angle2_ref);        // result matrix element 2

    // multiply the result matrix by the rotation and translation matrices
    result = rotation * trans * result;
    return result;
}

double f(double a, double b, double t) {
    return std::sqrt(a*a + b*b - 2*a*b*std::cos(t));
}

double g(double a, double b, double c) {
    return (a*a + b*b - c*c) / (2 * a * b);
}
void GP::forward_kinematic()        // get bkt teeth real postiion based on IMU data
{
    // based on IMU data, calculate the real positions of bkt teeth
    std::vector<double> IMUreal = {
        boom_pitch - IMU_bias[0],
        arm_pitch - IMU_bias[1] ,
        bkt_pitch - IMU_bias[2]
    };

    // calculate theta_boom
    double theta_boom = IMUreal[0];

    // calculate Joint_1
    double Joint_1 = IMUreal[1] - IMUreal[0];

    // calculate phi_QNM
    double phi_QNM = -IMUreal[1] + phi_FQN + IMUreal[2];

    // calculate l_QM
    double l_QM = f(l_MN, l_NQ, phi_QNM);

    // calculate phi_MQK
    double phi_MQK = std::acos(g(l_QM, l_KQ, l_KM));

    // calculate phi_MQN
    double phi_MQN = std::acos(g(l_QM, l_NQ, l_MN));

    // calculate Joint_2
    double Joint_2 = -phi_MQN - phi_MQK - phi_FQN - phi_KQV + M_PI;

    // calculate X and Z
    double X = l_CF * std::cos(theta_boom) + l_FQ * std::cos(theta_boom + Joint_1) + l_QV * std::cos(theta_boom + Joint_1 + Joint_2);
    double Z = l_CF * std::sin(theta_boom) + l_FQ * std::sin(theta_boom + Joint_1) + l_QV * std::sin(theta_boom + Joint_1 + Joint_2);

    // calculate the real bkt postion
    Eigen::Vector3d pt(X+boom_pos[0],boom_pos[1],Z+boom_pos[2]);
    Eigen::AngleAxisd rotation(real_swing - angle_offset,Eigen::Vector3d::UnitZ());

    realbkt_pos = rotation * pt;
}
#pragma endregion
