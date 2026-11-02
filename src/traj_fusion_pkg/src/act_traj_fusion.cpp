/*
项目          修改人       修改日期          改动
无人挖掘机     wangxing    2025/11/18    兜底策略添加
无人挖掘机     wuql7       2025/11/20    基于dengzh20251119的测试结果进行兜底策略修改
无人挖掘机     wuql7       2025/11/27    基于wangx20251127的测试结果进行兜底策略修改,添加模型评价reward
无人挖掘机     wuql7       2025/12/19    融合wangx20251219的代码
无人挖掘机     wuql7       2025/12/24    融合wangx20251224的代码



*/

#include "traj_fusion_node.hpp"




// 函数用于将弧度转换为角度
double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}


// 辅助函数：角度转弧度
double degreesToRadians(double degrees) 
{
    return degrees * M_PI / 180.0;
}

// 角度归一化到 [0,360)。必须用 fmod 而非 while 累加：
// while 形式在输入为 ±inf 时永不退出，输入为极大值（如 1e300）时需迭代 ~1e297 次，
// 等效于把整个 10Hz 主循环卡死、下游轨迹话题停更。非有限值统一兜底为 0。
static double wrapTo360Deg(double angle_deg)
{
    if (!std::isfinite(angle_deg))
        return 0.0;
    double a = std::fmod(angle_deg, 360.0);
    if (a < 0.0)
        a += 360.0;
    return a;
}

// 最短有向转角归一化到 (-180,180]，同样避免 while 归一化在非有限/极大值下死循环
static double wrapTo180Deg(double delta_deg)
{
    if (!std::isfinite(delta_deg))
        return 0.0;
    double d = std::fmod(delta_deg, 360.0);
    if (d > 180.0)
        d -= 360.0;
    else if (d <= -180.0)
        d += 360.0;
    return d;
}

// 两段折线：高度进度 p∈[0,1] → 回转插值系数 q∈[0,1]，过 (0,0)、(p1,q1)、(1,1)
// q1 < p1 时先慢后快；q1 > p1 时先快后慢；q1 = p1 时退化为线性
static double piecewiseLinearSwingInterp(double p, double p1, double q1)
{
    if (p1 <= 1e-9 || p1 >= 1.0 - 1e-9)
        return p;
    if (p <= p1)
        return (q1 / p1) * p;
    return q1 + ((1.0 - q1) / (1.0 - p1)) * (p - p1);
}

// 辅助函数：线性下降的函数
// double linearDecay(double threadDegree, double duration)
// {
//     if (x < 0) return 0;
//     if (x > 3) return 3;
//     return 3 - (x / duration) * 3;

// }

// 判断角度 angle 是否在 a(init) 与 b(unload) 之间，按旋转方向定义的弧上
// clockwise: true=顺时针(读数减小)，false=逆时针(读数增大)
// 坐标系：0°=正南, 90°=正东, 180°=正北, 270°=正西
static bool isAngleBetween(double angle, double a, double b, bool clockwise)
{
    angle = wrapTo360Deg(angle);
    a = wrapTo360Deg(a);
    b = wrapTo360Deg(b);

    if (clockwise)  // 顺时针：从 a 到 b 沿角度减小方向（经270°西侧）
    {
        if (a >= b)
            return angle <= a && angle >= b;
        else  // 跨 0°(南)
            return angle <= a || angle >= b;
    }
    else  // 逆时针：从 a 到 b 沿角度增大方向（经90°东侧）
    {
        if (a <= b)
            return angle >= a && angle <= b;
        else  // 跨 0°(南)
            return angle >= a || angle <= b;
    }
    //  std::cout<<"*angle:    "<<angle<<std::endl;
    
}

// theta 是否在 lb 与 ub 之间的较短圆弧上（两边界配置成跨约 90° 时即为「线缆 90° 扇区」）
static bool isAngleOnShorterArcBetween(double theta_deg, double lb_deg, double ub_deg)
{
    const double theta = wrapTo360Deg(theta_deg);
    const double lb = wrapTo360Deg(lb_deg);
    const double ub = wrapTo360Deg(ub_deg);
    const double d_ccw = wrapTo360Deg(ub - lb);  // lb→ub 逆时针弧长 ∈ [0,360)
    const bool clockwise_shorter = (d_ccw > 180.0);
    return isAngleBetween(theta, lb, ub, clockwise_shorter);
}

static double angleDiff(double a_deg, double b_deg)
{
    return wrapTo180Deg(b_deg - a_deg);
}

static bool isClockwiseShortestTurn(double from_deg, double to_deg)
{
    const double diff = angleDiff(from_deg, to_deg);
    if (std::fabs(diff) < 1e-9)
        return false;
    return diff < 0.0;
}



// 沿指定回转方向从 from 到 to 的转角 [0,360)：顺时针=角度减小，逆时针=增大（与 isAngleBetween 一致）
static double swingDistanceAlongTurnDirectionDeg(double from_deg, double to_deg, bool clockwise)
{
    const double from = wrapTo360Deg(from_deg);
    const double to = wrapTo360Deg(to_deg);
    if (clockwise)
        return wrapTo360Deg(from - to);
    return wrapTo360Deg(to - from);
}

// 沿指定方向从 from 转到 target 的过程中，是否会先经过 avoid_deg。
static bool swingPathPassesAngleBeforeTarget(double from_deg, double target_deg, double avoid_deg, bool clockwise)
{
    const double dist_to_avoid = swingDistanceAlongTurnDirectionDeg(from_deg, avoid_deg, clockwise);
    const double dist_to_target = swingDistanceAlongTurnDirectionDeg(from_deg, target_deg, clockwise);
    return dist_to_avoid > 1e-9 && dist_to_avoid < dist_to_target - 1e-9;
}


void traj_fusion_node::SwingTopicCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // current_angle_swing_ = msg->x;
    // current_velocity_swing_ = msg->y;
    // ref_angle_swing_ = msg->z;
    if (!std::isfinite(msg->data))
    {
        ROS_WARN_THROTTLE(2.0, "[SwingTopicCallback] non-finite swing angle, keep last value");
        return;
    }
    current_angle_swing_ = msg->data;
    current_angle_swing_valid_ = true;

}

void traj_fusion_node::digPointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
//    RGp_dig_point_angle = msg->x;
   if (!std::isfinite(msg->x))
   {
       ROS_WARN_THROTTLE(2.0, "[digPointCallback] non-finite dig point angle, ignored");
       return;
   }
   dig_point_angle_switch1 = msg->x;
   dig_point_angle_switch1_valid_ = true;
    // std::cout<<"===ccccccccccc===RGp_dig_point_angle:    "<<RGp_dig_point_angle<<std::endl;
}

void traj_fusion_node::cleanDigPointCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    if (!std::isfinite(msg->x))
    {
        ROS_WARN_THROTTLE(2.0, "[cleanDigPointCallback] non-finite dig point angle, ignored");
        return;
    }
    clean_dig_point_angle = msg->x;
    clean_dig_point_angle_valid_ = true;
}


//启动无人：/Mpc_Switch 消息为 true 时为无人状态
void traj_fusion_node::mpcSwitchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg || !msg->data)
        return;

    // 源角未收到过时不得赋值：/Mpc_Switch 完全可能先于 /Sys_RGp_SwWkAgl 到达，
    // 否则会把未初始化值写入回转角，后续阶段3 每周期的角度归一化将被脏值污染
    if (!dig_point_angle_switch1_valid_ || !std::isfinite(dig_point_angle_switch1))
    {
        ROS_WARN_THROTTLE(2.0,
            "[mpcSwitchCallback] dig point angle not ready (/Sys_RGp_SwWkAgl), skip update");
        return;
    }

    RGp_dig_point_angle = dig_point_angle_switch1;
    RGp_dig_point_angle_valid_ = true;

}


//搭台指令回调
void traj_fusion_node::mpcSwitch2Callback(const std_msgs::Bool::ConstPtr& msg)
{
    mpc_switch2_flag = msg->data;
    //搭台卸载点
    if(mpc_switch2_flag)
    {
        // 同上：搭台卸载点/挖掘点未就绪时保留原值，不用未初始化值覆盖
        if (clean_unload_angle_valid_ && std::isfinite(clean_unload_angle))
        {
            unload_angle = clean_unload_angle;
            unload_angle_valid_ = true;
        }
        else
        {
            ROS_WARN_THROTTLE(2.0,
                "[mpcSwitch2Callback] clean unload angle not ready (/Sys_RGp_Unload), keep last");
        }

        if (clean_dig_point_angle_valid_ && std::isfinite(clean_dig_point_angle))
        {
            RGp_dig_point_angle = clean_dig_point_angle;
            RGp_dig_point_angle_valid_ = true;
        }
        else
        {
            ROS_WARN_THROTTLE(2.0,
                "[mpcSwitch2Callback] clean dig point angle not ready (/clean_digPoint), keep last");
        }
    }
}

void traj_fusion_node::cleanUnloadPointCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    // /Sys_RGp_Unload：挖机回转中心北西局部坐标（与 truck 卸载点一致）
    //   x = N（北），y = W（西）；回转角约定 0°南 / 90°东 / 180°北 / 270°西
    // 与 calculateTruckVertices 相同：unload = atan2(-W, -N)
    const double cx = msg->x;  // N
    const double cy = msg->y;  // W
    if (!(std::fabs(cx) > 1e-6) && !(std::fabs(cy) > 1e-6))
    {
        ROS_WARN_THROTTLE(2.0, "[cleanUnload] local (N,W)~(0,0), skip angle update");
        return;
    }
    clean_unload_angle = normalizeAngle(std::atan2(-cy, -cx) * 180.0 / M_PI);
    clean_unload_angle_valid_ = true;

}

void traj_fusion_node::digRegionAnglesCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    front_lane_deg = msg->x;
    rear_cable_deg = msg->y;
}



// 两角先压到 [0,360)，angleDiff(a,b) 为从 a 到 b 的最短有向转角 ∈ (-180,180]，
// 短弧中点 = a + 0.5*angleDiff（即较小夹角的平分方向）。对径 180° 时两弧等长，结果与 a、b 先后次序有关。
double traj_fusion_node::middleAngleSmallerArcDeg(double angle_a_deg, double angle_b_deg)
{
    const double a = wrapTo360Deg(angle_a_deg);
    const double b = wrapTo360Deg(angle_b_deg);
    return wrapTo360Deg(a + 0.5 * angleDiff(a, b));
}

//实时计算动臂切削角（弧度）
void traj_fusion_node::calculateBoomCutAngle(double arm_angle) 
{

    // const double boom_length = 5703.58;
    // const double arm_length = 2929.94;
    // const double bucket_length = 1464.4056;
    const double pi = M_PI;
    const double angle_bucket_toe = 50*pi/180;

    double cos_val = cos(pi + arm_angle);
    double cos_angle_bucket_toe = cos(50*pi/180);

    // double boom2bucket_length = sqrt(boom_length * boom_length + arm_length * arm_length - 2 * boom_length * arm_length * cos_val);
    
    double numerator1 = arm_length - boom_length * cos_val;
    double numerator2 = bucket_length*cos_angle_bucket_toe;
    double denominator = sqrt(boom_length * boom_length + arm_length * arm_length - 2 * boom_length * arm_length * cos_val);
    
    // 数值安全检查 - 非常重要！
    double ratio1 = numerator1 / denominator;
    if (ratio1 > 1.0) {
        ratio1 = 1.0;
        ROS_WARN("acos parameter clamped to 1.0 (was: %f)", numerator1/denominator);
    } 
    else if (ratio1 < -1.0) {
        ratio1 = -1.0;
        ROS_WARN("acos parameter clamped to -1.0 (was: %f)", numerator1/denominator);
    }
    double ratio2 = numerator2 / denominator;
    if (ratio2 > 1.0) {
        ratio2 = 1.0;
        ROS_WARN("acos parameter clamped to 1.0 (was: %f)", numerator2/denominator);
    } 
    else if (ratio2 < -1.0) {
        ratio2 = -1.0;
        ROS_WARN("acos parameter clamped to -1.0 (was: %f)", numerator2/denominator);
    }

    boom_cut_angle = acos(ratio1) + acos(ratio2) + angle_bucket_toe -20*pi/180 - pi;
}


//实时计算斗杆切削角(角度)
void traj_fusion_node::calculateArmCutAngle() 
{ 
    arm_cut_angle = -45;
}

//实时计算铲斗端平角（角度）
void traj_fusion_node::calculateBucketFlatAngle() 
{
 
    bucket_flat_angle = -180 - arm_angle_current - boom_angle_current;
}



void traj_fusion_node::resetDigOpDetection()
{
    std::lock_guard<std::mutex> lock(dig_op_mutex_);
    dig_op_prev_angles_valid_ = false;
    dig_op_smoothed_vel_valid_ = false;
    dig_op_smoothed_boom_vel_ = 0.0;
    dig_op_smoothed_arm_vel_ = 0.0;
    dig_op_smoothed_bucket_vel_ = 0.0;
    dig_op_last_sample_time_ = ros::Time();
    dig_OP_level_.store(0.0);
}

// 挖掘阶段憋压检测（由 jointsCallback 在 traj_status==3 时调用）：
// 三关节角差分求瞬时角速度，EMA 低延时平滑；均低于阈值则判为憋压
void traj_fusion_node::dig_OP_detection()
{
    // 快照当前关节角，避免与 jointsCallback 后续写入交错
    const double boom = boom_angle_current;
    const double arm = arm_angle_current;
    const double bucket = bucket_angle_current;

    std::lock_guard<std::mutex> lock(dig_op_mutex_);
    const ros::Time now = ros::Time::now();

    if (!dig_op_prev_angles_valid_)
    {
        dig_op_prev_boom_ = boom;
        dig_op_prev_arm_ = arm;
        dig_op_prev_bucket_ = bucket;
        dig_op_prev_angles_valid_ = true;
        dig_op_last_sample_time_ = now;
        dig_OP_level_.store(0.0);
        return;
    }

    const double dt = (now - dig_op_last_sample_time_).toSec();
    if (dt < 1e-6)
        return;

    // 采样断流：间隔过大时重新 seed，避免误算极小平均速度
    if (dt > dig_op_max_sample_dt_)
    {
        dig_op_prev_boom_ = boom;
        dig_op_prev_arm_ = arm;
        dig_op_prev_bucket_ = bucket;
        dig_op_last_sample_time_ = now;
        dig_op_smoothed_vel_valid_ = false;
        dig_OP_level_.store(0.0);
        return;
    }

    dig_op_last_sample_time_ = now;

    const double abs_boom_vel = std::abs(angleDiff(dig_op_prev_boom_, boom) / dt);
    const double abs_arm_vel = std::abs(angleDiff(dig_op_prev_arm_, arm) / dt);
    const double abs_bucket_vel = std::abs(angleDiff(dig_op_prev_bucket_, bucket) / dt);
    dig_op_prev_boom_ = boom;
    dig_op_prev_arm_ = arm;
    dig_op_prev_bucket_ = bucket;

    const double alpha = std::clamp(dig_op_vel_ema_alpha, 0.05, 1.0);
    if (!dig_op_smoothed_vel_valid_)
        {
        dig_op_smoothed_boom_vel_ = abs_boom_vel;
        dig_op_smoothed_arm_vel_ = abs_arm_vel;
        dig_op_smoothed_bucket_vel_ = abs_bucket_vel;
        dig_op_smoothed_vel_valid_ = true;
        }
        else
        {
        dig_op_smoothed_boom_vel_ = alpha * abs_boom_vel + (1.0 - alpha) * dig_op_smoothed_boom_vel_;
        dig_op_smoothed_arm_vel_ = alpha * abs_arm_vel + (1.0 - alpha) * dig_op_smoothed_arm_vel_;
        dig_op_smoothed_bucket_vel_ = alpha * abs_bucket_vel + (1.0 - alpha) * dig_op_smoothed_bucket_vel_;
    }

    if (dig_op_smoothed_boom_vel_ < dig_joint_vel_thresh
        && dig_op_smoothed_arm_vel_ < dig_joint_vel_thresh
        && dig_op_smoothed_bucket_vel_ < dig_joint_vel_thresh)
    {
        dig_OP_level_.store(1.0);
    }
    else
    {
        dig_OP_level_.store(0.0);
    } 
}

//挖掘阶段动臂提升适应性改进
// void traj_fusion_node::dig_traj_adjust()
// {
//     traj_fusion_node::dig_OP_detection();
//     traj_dig.orientation.y += dig_OP_level * boom_adjust_range;
// }
void traj_fusion_node::dig_traj_adjust()
{
    // dig_OP_level 已在 jointsCallback 中更新，此处仅叠加动臂偏置
    boom_increment = dig_OP_level_.load() * boom_adjust_range;
    stage2_traj.orientation.x = dig_OP_level_.load();
    stage2_traj.orientation.y = boom_increment;
    stage2_traj.position.x = dig_op_smoothed_boom_vel_;
    stage2_traj.position.y = dig_op_smoothed_arm_vel_;
    stage2_traj.position.z = dig_op_smoothed_bucket_vel_;
    // 单行聚合输出：原先每周期 5 行 std::endl 强制 flush，10Hz 下持续刷屏；
    // stdout 一旦是未被排空的管道/卡顿终端，write() 会阻塞在主循环里导致轨迹话题停更。
    ROS_INFO_STREAM("===3===dig_traj_adjust"
                    << " dig_OP_level=" << dig_OP_level_.load()
                    << " boom_increment=" << boom_increment
                    << " vel_boom=" << dig_op_smoothed_boom_vel_
                    << " vel_arm=" << dig_op_smoothed_arm_vel_
                    << " vel_bucket=" << dig_op_smoothed_bucket_vel_);

    current_traj.orientation.y += boom_increment;

    // pub_dig_OP_level.publish(dig_OP_level);
}




//计算回转方向及角度
bool traj_fusion_node::stop_swing_main() 
{
    // 回转方向：按 current_angle_swing_ → unload_angle 的最小行程（有向角 ∈ (-180,180]）
    // d<0 为顺时针，d>0 为逆时针（与 act_traj_fusion 复位阶段约定一致）

    // 本函数在阶段3 每个控制周期都会被调用，且内部连续做多次角度归一化。
    // 输入角未就绪或非有限时直接放弃本轮计算，保持上一周期的
    // swing_clockwise_4/5、truck_swive_angle_msg 结果，避免脏值进入归一化。
    if (!RGp_dig_point_angle_valid_ || !unload_angle_valid_ ||
        !std::isfinite(RGp_dig_point_angle) || !std::isfinite(unload_angle) ||
        !std::isfinite(rear_cable_deg) || !std::isfinite(front_lane_deg) ||
        !std::isfinite(current_angle_swing_) || !std::isfinite(start3_angle))
    {
        ROS_WARN_THROTTLE(2.0,
            "[stop_swing_main] swing inputs not ready, skip this cycle "
            "(dig_valid=%d unload_valid=%d dig=%.3f unload=%.3f cable=%.3f lane=%.3f swing=%.3f start3=%.3f)",
            static_cast<int>(RGp_dig_point_angle_valid_), static_cast<int>(unload_angle_valid_),
            RGp_dig_point_angle, unload_angle, rear_cable_deg, front_lane_deg,
            current_angle_swing_, start3_angle);
        return false;
    }

    start_dig_angle = normalizeAngle(RGp_dig_point_angle);
    
    double cur_sw = normalizeAngle(RGp_dig_point_angle);
    // double cur_sw = std::isfinite(RGp_dig_point_angle) ? normalizeAngle(RGp_dig_point_angle)
    //                                                             : current_angle_swing_;

    // if (prev_seq_traj_status_ == 1)  // 1 -> 3 
    // {
    //     cur_sw = current_angle_swing_;
    //     std::cout << "===3333====stop_swing_main:cur_sw=======:    " << cur_sw << std::endl;
    // }

    if(start1_angle_flag)
    {
        cur_sw = current_angle_swing_;
        

    }


    // if (traj_status == 5)  // 5 -> 4
    // {
    //     cur_sw = normalizeAngle(front_lane_deg);
    //     std::cout << "===5555====stop_swing_main:cur_sw=======:    " << cur_sw << std::endl;

    // }

    const double unload_sw = normalizeAngle(unload_angle);
    const double cable_sw = normalizeAngle(rear_cable_deg);
    const bool shortest_clockwise = isClockwiseShortestTurn(cur_sw, unload_sw);
    const bool shortest_hits_cable =
        swingPathPassesAngleBeforeTarget(cur_sw, unload_sw, cable_sw, shortest_clockwise);
    const bool reverse_hits_cable =
        swingPathPassesAngleBeforeTarget(cur_sw, unload_sw, cable_sw, !shortest_clockwise);

    swing_clockwise_4 = shortest_clockwise;
    if (shortest_hits_cable && !reverse_hits_cable)
        swing_clockwise_4 = !shortest_clockwise;

    // std::cout << "===3====RGp_dig_point_angle=======:    " << RGp_dig_point_angle << std::endl;
    // std::cout << "===3====current_swing_angle=======:    " << cur_sw << std::endl;
    // std::cout << "===3====unload_sw=======:    " << unload_sw << std::endl;
    // std::cout << "===3====current_swing_angle=======:    " << current_angle_swing_ << std::endl;

    swing_clockwise_cable = isClockwiseShortestTurn(cur_sw, cable_sw);
    // std::cout<<"===3===swing_clockwise_cable:    "<<swing_clockwise_cable<<std::endl;

    // 线缆防碰：最小角方向若会先经过 rear_cable_deg，则优先切到反方向；若最终方向仍经过线缆角，触发接管。
    unload_swing_cable_first_flag_ = false;
    if (swingPathPassesAngleBeforeTarget(cur_sw, unload_sw, cable_sw, swing_clockwise_4))
    {
        unload_swing_cable_first_flag_ = true;
    }
    


    // 车尾目标回转角：在卸载方位 unload_angle 上，沿去往卸载点的回转方向偏置 truck_tail_offset_from_unload_deg_（默认30°）
    // 顺时针趋近卸载：车尾在卸载角“增大”一侧；逆时针趋近：在“减小”一侧
    if (swing_clockwise_4)
        truck_swive_angle_msg.data = normalizeAngle(unload_angle + truck_tail_offset_from_unload_deg_);
    else
        truck_swive_angle_msg.data = normalizeAngle(unload_angle - truck_tail_offset_from_unload_deg_);


    //3->5的回转方向计算
    const double new_target = front_lane_deg;
    // const double shortest_delta = angleDiff(cur_sw, new_target);
    const double shortest_delta = angleDiff(start3_angle, new_target);

    if (std::fabs(shortest_delta) < 1e-9)
        swing_clockwise_5 = true;
    else
        swing_clockwise_5 = (shortest_delta < 0.0);

    // 单行聚合输出：本函数在阶段3 每周期调用，原先 4 行 std::endl 会放大 stdout 阻塞风险
    ROS_INFO_STREAM("===3===stop_swing_main"
                    << " cur_sw=" << cur_sw
                    << " start3_angle=" << start3_angle
                    << " unload_sw=" << unload_sw
                    << " cable_sw=" << cable_sw
                    << " cw4=" << swing_clockwise_4
                    << " cw5=" << swing_clockwise_5
                    << " cable_first=" << unload_swing_cable_first_flag_
                    << " truck_swive=" << truck_swive_angle_msg.data);


    return true;

}







//辅助函数：根据铲斗高度百分比重新计算回转角度
double traj_fusion_node::calculateTargetSwing(double currentBH, double targetBH, double targetSwingAngle)
{
    if(!(std::fabs(state_value - 4.0) < 1e-9 || std::fabs(state_value - 4.5) < 1e-9))
        return 0.0;

    // 1) 由铲斗高度得到线性进度 p（并保持单调不回退）
    const double heightDenom = targetBH - init_bucket_height;
    if (std::fabs(heightDenom) < 1e-6)
        return 0.0;
    swing_percent = (currentBH - init_bucket_height) / heightDenom;
    if (swing_percent < 0.0)
        swing_percent = last_percent;
    if (swing_percent < last_percent)
        swing_percent = last_percent;
    if (swing_percent > 1.0)
        swing_percent = 1.0;
    if (targetBH <= currentBH)
        swing_percent = 1.0;

    // 2) 两段折线非线性：先慢后快（q1 < p1）
    const double swing_interp_p1 = 0.7;
    const double swing_interp_q1 = 0.4;
    const double swing_interp =
        piecewiseLinearSwingInterp(swing_percent, swing_interp_p1, swing_interp_q1);

    // 3) 在坐标系 0南/90东/180北/270西 下，按回转方向做有向插值
    // clockwise=true 表示顺时针：角度减小方向
    auto directedArcDeg = [](double from_deg, double to_deg, bool clockwise) {
        const double from = wrapTo360Deg(from_deg);
        const double to = wrapTo360Deg(to_deg);
        if (clockwise)
            return wrapTo360Deg(from - to);   // 从 from 顺时针到 to 的弧长
        return wrapTo360Deg(to - from);       // 从 from 逆时针到 to 的弧长
    };

    // const double start_swing = normalizeAngle(init_angle_swing_4_);
    const double start_swing = start_dig_angle;
    const double target_swing_norm = normalizeAngle(targetSwingAngle);
    const bool clockwise = swing_clockwise_4;
    const double total_arc = directedArcDeg(start_swing, target_swing_norm, clockwise);
    const double progressed_arc = total_arc * swing_interp;

    double swing_cmd = 0.0;
    if (clockwise)
        swing_cmd = start_swing - progressed_arc;
    else
        swing_cmd = start_swing + progressed_arc;
    swing_cmd = normalizeAngle(swing_cmd);

    // std::cout<<"***********swing_percent:    "<<swing_percent<<std::endl;
    // std::cout<<"***********swing_interp:    "<<swing_interp<<std::endl;
    // std::cout<<"***********start_swing:    "<<start_swing<<std::endl;
    // std::cout<<"***********targetSwingAngle:    "<<target_swing_norm<<std::endl;
    // std::cout<<"***********total_arc:    "<<total_arc<<std::endl;
    // std::cout<<"***********target_swing:    "<<swing_cmd<<std::endl;

    last_percent = swing_percent;
    return swing_cmd;


}

// 辅助函数：根据回转角度百分比重新计算动臂角度（阶段2）
double traj_fusion_node::calculateTargetBoomBySwingPercent(double currentSwingAngle, double targetSwingAngle, double targetBoomAngle)
{
    if (!(std::fabs(state_value - 2.0) < 1e-9))
        return init_reset_boom_4_;

    if (!std::isfinite(currentSwingAngle) ||
        !std::isfinite(targetSwingAngle) ||
        !std::isfinite(targetBoomAngle))
        return targetBoomAngle;

    const double start_swing = normalizeAngle(init_reset_swing_2_);
    const double current_swing = normalizeAngle(currentSwingAngle);
    const double target_swing = normalizeAngle(targetSwingAngle);
    const bool clockwise = swing_clockwise_2;

    const double total_arc = swingDistanceAlongTurnDirectionDeg(start_swing, target_swing, clockwise);
    if (!std::isfinite(total_arc) || total_arc < 1e-6)
        return targetBoomAngle;

    const double progressed_arc = swingDistanceAlongTurnDirectionDeg(start_swing, current_swing, clockwise);
    if (!std::isfinite(progressed_arc))
        return targetBoomAngle;

    double boom_percent = progressed_arc / total_arc;
    if (!std::isfinite(boom_percent))
        return targetBoomAngle;

    if (boom_percent < 0.0)
        boom_percent = reset_boom_last_percent_;
    if (boom_percent < reset_boom_last_percent_)
        boom_percent = reset_boom_last_percent_;
    if (boom_percent > 1.0)
        boom_percent = 1.0;
    if (progressed_arc >= total_arc)
        boom_percent = 1.0;

    // 两段折线映射：前段慢、后段快
    const double boom_interp_p1 = 0.7;
    const double boom_interp_q1 = 0.4;
    const double boom_interp =
        piecewiseLinearSwingInterp(boom_percent, boom_interp_p1, boom_interp_q1);

    const double boom_cmd = init_reset_boom_4_ + (targetBoomAngle - init_reset_boom_4_) * boom_interp;
    if (std::isfinite(boom_cmd))
    {
        if (std::isfinite(boom_percent))
            reset_boom_last_percent_ = boom_percent;
        return boom_cmd;
    }
    return targetBoomAngle;
}

//===================挖掘阶段:fusion===============================//
void traj_fusion_node::oriTrajDigCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

    traj_dig.orientation.x = msg->orientation.x;
    traj_dig.orientation.y = msg->orientation.y;
    traj_dig.orientation.z = msg->orientation.z;
    traj_dig.orientation.w = msg->orientation.w;
    traj_dig.position = msg->position;

}


//===================提升卸载阶段===============================//
void traj_fusion_node::oriTrajUnloadCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

    traj_unload.orientation.x = msg->orientation.x;
    traj_unload.orientation.y = msg->orientation.y;
    traj_unload.orientation.z = msg->orientation.z;
    traj_unload.orientation.w = msg->orientation.w;
    traj_unload.position = msg->position;


}

void traj_fusion_node::oriUnloadPostureCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    traj_unload_posture.orientation.x = msg->orientation.x;//swing angle
    traj_unload_posture.orientation.y = msg->orientation.y;//boom angle
    traj_unload_posture.orientation.z = msg->orientation.z;//arm angle
    traj_unload_posture.orientation.w = msg->orientation.w;//bucket angle
    traj_unload_posture.position = msg->position;//position

}



//===================复位阶段===============================//
void traj_fusion_node::oriTrajResetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    traj_reset.orientation.x = msg->orientation.x;
    traj_reset.orientation.y = msg->orientation.y;
    traj_reset.orientation.z = msg->orientation.z;
    traj_reset.orientation.w = msg->orientation.w;
    traj_reset.position = msg->position;
}

//===================抹平阶段===============================//
void traj_fusion_node::oriTrajSmoothCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    traj_smooth.orientation.x = msg->orientation.x;
    traj_smooth.orientation.y = msg->orientation.y;
    traj_smooth.orientation.z = msg->orientation.z;
    traj_smooth.orientation.w = msg->orientation.w;
    traj_smooth.position = msg->position;
}

//===================清台阶段===============================//
void traj_fusion_node::oriTrajCleanupCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    traj_cleanup.orientation.x = msg->orientation.x;
    traj_cleanup.orientation.y = msg->orientation.y;
    traj_cleanup.orientation.z = msg->orientation.z;
    traj_cleanup.orientation.w = msg->orientation.w;
    traj_cleanup.position = msg->position;
}

//收到朱安源的挖掘机状态阶段
void traj_fusion_node::sysSeqActionCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // 提取消息中的浮点数状态
    const int previous_traj_status = traj_status;
    state_value = msg->data;//2：复位 3：挖掘 4：卸载 1:初始化

    // std::cout<<"***********state_value:    "<<state_value<<std::endl;
    
    // 处理接收到的状态信息
    // ROS_INFO_THROTTLE(1, "Received Zhuanyuan excavator state: %.1f", state_value);

    if(std::fabs(state_value - 3.0) < 1e-9)
    {
        traj_status = 3;
    }
    else if(std::fabs(state_value - 4.0) < 1e-9)
    {
        traj_status = 4;
    }
    else if(std::fabs(state_value - 2.0) < 1e-9)
    {
        traj_status = 2;
    }
    else if(std::fabs(state_value - 1.0) < 1e-9)
    {
        traj_status = 1;
    }
    else if(std::fabs(state_value - 5.0) < 1e-9)
    {
        traj_status = 5;
    }
    else if(std::fabs(state_value - 6.0) < 1e-9)
    {
        traj_status = 6;
    }
    else if(std::fabs(state_value - 7.0) < 1e-9)
    {
        traj_status = 7;
    }
    else if(std::fabs(state_value - 3.5) < 1e-9)
    {
        traj_status = 9;
    }
    else if(std::fabs(state_value - 4.5) < 1e-9)
    {
        traj_status = 8;
    }
    else if(std::fabs(state_value - 5.5) < 1e-9)
    {
        traj_status = 10;
    }
    else if(std::fabs(state_value - 6.5) < 1e-9)
    {
        traj_status = 11;
    }
    else if(std::fabs(state_value - 7.5) < 1e-9)
    {
        traj_status = 12;
    }
    else if(std::fabs(state_value - 2.5) < 1e-9)
    {
        traj_status = 13;
    }
    else
    {
        traj_status = 14;
    }

    if (traj_status != previous_traj_status)
    {
        prev_seq_traj_status_ = previous_traj_status;
        // if (traj_status == 3 || previous_traj_status == 3)
        //     resetDigOpDetection();
    }

    //检测状态切换上升沿
    // if (std::fabs(state_value - 4.0) < 1e-9 && std::fabs(last_state_ - 3.5) < 1e-9) 
    // {
    //     // init_angle_boom_4_ = boom_angle_current;

    //     init_angle_swing_4_ = current_angle_swing_;

    //     init_rtk_heading = rtk_heading;
    //     init_bucket_height = bucket_point[2] - exc_bucket_H;

    //     std::cout<<"***********init_bucket_height:    "<<init_bucket_height<<std::endl;
    //     swing_percent = 0.01;
    //     last_percent = 0.01;
    //     last_traj_unload_swing_ = current_angle_swing_;
    //     last_traj_unload_swing_valid_ = false;
    //     //计算车尾角
    //     stop_swing_main();
        
    // }

    // if (std::fabs(state_value - 3.5) < 1e-9 || std::fabs(state_value - 4.0) < 1e-9)
    // if (std::fabs(state_value - 4.0) < 1e-9)  
    // {
    //     pub_truck_swive_angle.publish(truck_swive_angle_msg);
    //     pub_swing_direction.publish(excavator_rotates_clockwise_flag_);
        
    // }

    // if (std::fabs(state_value - 2.0) < 1e-9 && std::fabs(last_state_ - 4.5) < 1e-9) 
    // {
    //     last_traj_reset_swing_ = current_angle_swing_;
    //     last_traj_reset_swing_valid_ = false;
        
    // }


    // if (std::fabs(state_value - 4.0) < 1e-9) 
    // {
    //     init_reset_boom_4_ = boom_angle_current;   // record boom angle in state 4.0
        
    // }

    // if (std::fabs(state_value - 3.0) < 1e-9) 
    // {
    //     init_dig_bucket_3_ = bucket_angle_current;   // record bucket angle in state 3.0

    //     init_dig_swing_3_ = current_angle_swing_;

    //     init_dig_rtk_heading = rtk_heading;
    //     init_dig_bucket_height = ef_tip_rotation_z - exc_bucket_H;
    //     swing_percent = 0.0;
    //     last_percent = 0.0;
    //     last_unload_swing_raw_valid_ = false;
    //     unload_swing_unwrap_offset_deg_ = 0.0;
    //     last_traj_dig_valid_ = false;

        
    // }



    last_state_ = state_value; // 更新状态值    
}
//收到户凯亭的挖掘机的状态
void traj_fusion_node::excavatorStateCallback(const std_msgs::Int32::ConstPtr& msg)
{
    excavator_state = msg->data;//0: 初始化 1：复位 2：挖掘 3：提升 4：卸载 5：平地
}

//实时接收斗杆角度，如果达到目标最大角度则退出ACT
// 回调函数取出斗杆实时数值
void traj_fusion_node::jointsCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    arm_angle_current = msg->y;
    boom_angle_current = msg->x;
    bucket_angle_current = msg->z;

    //动臂切削角
    calculateBoomCutAngle(degreesToRadians(arm_angle_current));

    //斗杆切削角
    calculateArmCutAngle();
    //端平角计算
    calculateBucketFlatAngle();

    // 挖掘阶段：随关节角更新频率做憋压检测
    // if (traj_status == 3)
    //     dig_OP_detection();

    // MNQ = radiansToDegrees(msg->joint_attachment);
}

void traj_fusion_node::jointsVelocityCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    boom_velocity_current = msg->y;
    arm_velocity_current = msg->z;
    bucket_velocity_current = msg->w;
}

void traj_fusion_node::EFTipRotationTopicCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ef_tip_rotation_x = msg->x;
    ef_tip_rotation_y = msg->y;
    ef_tip_rotation_z = msg->z;
    // Eigen::Affine3d transform_rotation = Eigen::Affine3d::Identity();
    // transform_rotation.prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()));
    // Eigen::Vector4d bucket_translation_rotion_las = {ef_tip_rotation_x, ef_tip_rotation_y, ef_tip_rotation_z, 1.0};
    // Eigen::Vector4d bucket_position = transform_rotation * bucket_translation_rotion_las;
    // bucket_point_stamped.header.frame_id = "base";
    // bucket_point_stamped.header.stamp = ros::Time::now();
    // bucket_point_stamped.point.x = bucket_position[0];
    // bucket_point_stamped.point.y = bucket_position[1];
    // bucket_point_stamped.point.z = bucket_position[2];
}
void traj_fusion_node::ef_position_enu_callback(const interfaces::SinglePoint::ConstPtr& msg)
{
    ef_position_enu_[0] = msg->point.x;
    ef_position_enu_[1] = msg->point.y;
    ef_position_enu_[2] = msg->point.z;

    bucket_point = globalToLocal(ef_position_enu_, exc_center_enu);
}

void traj_fusion_node::ENUDataTopicCallback(const interfaces::rtk_cgi610::ConstPtr& msg)
{   
    {
        std::lock_guard<std::mutex> lock(data_mutex); // 加锁保护共享数据
        rtk_heading = msg->heading;
        rtk_lat = msg->lat;
        rtk_lon = msg->lon;
        rtk_lla_valid_ = true;
        double roll_rad = msg->roll * M_PI / 180.0;
        double pitch_rad = msg->pitch * M_PI / 180.0;
        rtk_uu = msg->uu;
        rtk_alt = msg->alt;
        // angular_velocity.data = msg->ang_z;
        // current_velovity_swing_ = msg->ang_z;
        if(swivel_angle_flag)
        {
            dig_rtk_heading_angle = msg->heading;
        }
        
        swivel_angle.data = - msg->heading + 180.0;
        if(swivel_angle.data <= 0.0)
            swivel_angle.data += 360.0;
        
        double heading_deg = - msg->heading + 90.0;
        // double heading_deg = - msg->heading + 180.0;
        if(heading_deg < 0.0)
            heading_deg += 360.0;
        double heading_rad = heading_deg * M_PI / 180.0;

        Eigen::Vector4d exc_center_t = {exc_center_W_L/1000, exc_center_W_H/1000, exc_center_W_W/1000, 1.0};
        Eigen::Affine3d transform_excenter = Eigen::Affine3d::Identity();

        transform_excenter.translate(Eigen::Vector3d(msg->ee, msg->nn, rtk_uu));
        transform_excenter.rotate(Eigen::AngleAxisd(heading_rad, Eigen::Vector3d::UnitZ()));
        transform_excenter.rotate(Eigen::AngleAxisd(-pitch_rad, Eigen::Vector3d::UnitY()));
        transform_excenter.rotate(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));
        exc_center_enu = transform_excenter * exc_center_t;
    }
    // 圆心随挖机 RTK 刷新；须在释放 data_mutex 后发布，避免与 localNWToLLA 死锁
    publishLaneCircleIntersections();
}

void traj_fusion_node::bucket_enu_callback(const interfaces::SinglePoint::ConstPtr& msg)
{
    bucket_enu_point = *msg;
}

//收到马兰神申矿卡尺寸信息
void traj_fusion_node::truckInfoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    truck_length = msg->pose.position.x;//长度
    truck_width = msg->pose.position.y;//宽度
    truck_height = msg->pose.position.z;//高度
    

}


//收到马兰神申矿卡位置信息
void traj_fusion_node::truckRTKCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //矿卡型号
    std::string truck_id = msg->header.frame_id;

    // 提取矿卡中心位置信息
    truck_rtk_position.x = msg->pose.position.x; //经度                // X坐标:ml-citation{ref="4" data="citationList"}
    truck_rtk_position.y = msg->pose.position.y; //维度                // Y坐标:ml-citation{ref="4" data="citationList"}
    truck_rtk_position.z = msg->pose.position.z; //高度                // Z坐标:ml-citation{ref="4" data="citationList"}
    
    // 提取姿态信息（四元数）
    // truck_pitch = msg->pose.orientation.x;               // 俯仰()
    truck_real_height = msg->pose.orientation.x;               // 矿卡实际高度
    m_working_height = msg->pose.orientation.y;                // 工作面高度
    truck_heading = msg->pose.orientation.z;             // 偏航

    // orientation.w==2：对接近程，每帧计算卸载点。
    // orientation.w==3：首次必算；之后经纬水平距>0.5m 或航向差>2° 才重算。
    // 其它 w：清空 cache。不看 traj_status。
    const double truck_w = msg->pose.orientation.w;
    const bool w_is_2 = (std::fabs(truck_w - 2.0) < 1e-6);
    const bool w_is_3 = (std::fabs(truck_w - 3.0) < 1e-6);
    truck_dock_success_flag_ = w_is_3;

    if (w_is_2)
    {
        last_unload_calc_pos_valid_ = false;  // 进入 w==3 时强制先算一次
        calculateTruckVertices();
        truck_geometry_calc_done_for_dock_ = truck_geom_cache_valid_;
    }
    else if (w_is_3)
    {
        bool need_calc = !last_unload_calc_pos_valid_;
        if (!need_calc)
        {
            const double lat = truck_rtk_position.y;
            const double lon = truck_rtk_position.x;
            const double dlat = lat - last_unload_calc_lat_;
            const double dlon = lon - last_unload_calc_lon_;
            const double mean_lat_rad = 0.5 * (lat + last_unload_calc_lat_) * M_PI / 180.0;
            const double dN = dlat * 111320.0;
            const double dE = dlon * 111320.0 * std::cos(mean_lat_rad);
            const bool moved = (dN * dN + dE * dE) > (kUnloadRecalcDistM_ * kUnloadRecalcDistM_);
            const bool yawed = std::fabs(wrapTo180Deg(truck_heading - last_unload_calc_heading_))
                               > kUnloadRecalcHeadingDeg_;
            need_calc = moved || yawed;
        }
        if (need_calc)
        {
            calculateTruckVertices();
            if (truck_geom_cache_valid_)
            {
                last_unload_calc_lat_ = truck_rtk_position.y;
                last_unload_calc_lon_ = truck_rtk_position.x;
                last_unload_calc_heading_ = truck_heading;
                last_unload_calc_pos_valid_ = true;
            }
            truck_geometry_calc_done_for_dock_ = truck_geom_cache_valid_;
        }
    }
    else
    {
        last_unload_calc_pos_valid_ = false;
        truck_geometry_calc_done_for_dock_ = false;
        truck_geom_cache_valid_ = false;
    }

    publishTruckGeometryCache();

}


void traj_fusion_node::truckLocationCallback(const std_msgs::Int8::ConstPtr& msg)
{
    // if (msg->data == 1)
    // {
    //     calculateTruckVertices(); 
        
    // }
    // // calculateTruckVertices();
    // // 计算成功后的结果持续发布，直到下一次停靠成功时被覆盖
    // publishTruckGeometryCache();

}

//=================================================以下是接收户凯亭状态阶段处理================================================//




double traj_fusion_node::unwrapTrajectorySwing(double prev_unwrapped, double raw_deg, bool clockwise)
{
    const double r = normalizeAngle(raw_deg);
    if (clockwise)
    {
        const double m = std::floor((prev_unwrapped - r) / 360.0 + 1e-9);
        return r + 360.0 * m;
    }
    const double m = std::ceil((prev_unwrapped - r) / 360.0 - 1e-9);
    return r + 360.0 * m;
}

// =============复位阶段2===================
void traj_fusion_node::resetStatusProcess()
{
    start1_angle_flag = false;
    start3_angle = std::isfinite(RGp_dig_point_angle) ? normalizeAngle(RGp_dig_point_angle) : current_angle_swing_;

    current_traj.orientation.x = current_angle_swing_;

    bucket_change_rate = 100.0;

    //复位阶段用动臂切削角
    current_traj.orientation.w = radiansToDegrees(boom_cut_angle);

    if (!init_reset_phase2_captured_)
    {
        init_reset_swing_2_ = current_angle_swing_;
        reset_boom_last_percent_ = 0.0;
        init_reset_phase2_captured_ = true;
    }

    {
        // 复位阶段回转轨迹：与阶段4 相同——基准偏置仅首帧；相位 reset_swing_phase_ ∈{-1,0,+1}，有效偏置 = 基准 + 相位×360°
        // 回转方向仅在进入阶段2 首周期算一次；之后沿用 swing_clockwise_2，避免运动中改向
        if (!std::isfinite(RGp_dig_point_angle))
        {
            // 目标角偶发 NaN/Inf：不参与展开与方向更新，避免污染 last_reset_swing_raw_deg_
            // if (last_reset_swing_raw_valid_)
            //     current_traj.orientation.x =
            //         last_reset_swing_raw_deg_ + reset_swing_unwrap_offset_deg_
            //         + static_cast<double>(reset_swing_phase_) * 360.0;
            // else
            //     current_traj.orientation.x = normalizeAngle(current_angle_swing_);
            current_traj.orientation.x = current_angle_swing_;
        }
        else
        {
            // 阶段2的真实回转目标角：使用挖掘点 RGp_dig_point_angle
            // const double raw_sw = normalizeAngle(RGp_dig_point_angle);

            const double reset_target = normalizeAngle(RGp_dig_point_angle);
            const double reset_cable = rear_cable_deg;
            double start_traj = normalizeAngle(unload_angle);
            // double start_traj = std::isfinite(unload_angle) ? normalizeAngle(unload_angle)
            //                                                         : current_angle_swing_;

            // 阶段2 周期打印已移除（仅保留阶段3 的周期日志）

            bool cw_reset;
            if (!reset_swing_direction_computed_this_entry_)
            {
                if (reset_swing_direction_precomputed_in_unload_)
                {
                    // 阶段4进入卸载区域时已预计算：起始 unload_angle，目标 RGp_dig_point_angle
                    cw_reset = swing_clockwise_2;
                }
                // 依据“进入阶段2前的来源阶段”选回转方向
                else if (prev_traj_status_for_timer_ == 1)  // 1 -> 2
                {
                    cw_reset = swing_clockwise_2;
                }
                else if (prev_traj_status_for_timer_ == 4 || prev_traj_status_for_timer_ == 8 
                         || prev_traj_status_for_timer_ == 6 || prev_traj_status_for_timer_ == 7
                         || prev_traj_status_for_timer_ == 11 || prev_traj_status_for_timer_ == 12
                        )  // 4 -> 2
                {
                    if(prev_traj_status_for_timer_ == 7  || prev_traj_status_for_timer_ == 12)
                    {
                        start_traj = current_angle_swing_;
                    }
                    // cw_reset = !swing_clockwise_4;
                    // 按当前回转角到挖掘目标的最小行程方向，不再取反阶段4方向
                    // const double cur_sw = normalizeAngle(current_angle_swing_);
                    const double shortest_delta = angleDiff(start_traj, reset_target);
                    const double shortest_cable = angleDiff(start_traj, reset_cable);
                    if (std::fabs(shortest_delta) < 1e-9)
                        cw_reset = !swing_clockwise_4;  // 共线默认逆时针，与阶段4 stop_swing_main 一致
                    else
                        cw_reset = (shortest_delta < 0.0);  // true=顺时针(角度减小)

                    ROS_DEBUG("===2===cw_reset: %d", static_cast<int>(cw_reset));

                    if (std::fabs(shortest_cable) < 1e-9)
                        swing_clockwise_cable = false;  // 共线默认逆时针，与阶段4 stop_swing_main 一致
                    else
                        swing_clockwise_cable = (shortest_cable < 0.0);  // true=顺时针(角度减小)

                    ROS_DEBUG("===2===swing_clockwise_cable: %d", static_cast<int>(swing_clockwise_cable));

                    reset_swing_cable_first_flag_ = false;
                    // 复位规划弧方向为 cw_reset（随后 swing_clockwise_2），应与 start_traj→线缆 最短方向一致，再比较沿复位弧上到线缆 vs 到挖掘目标
                    if (cw_reset == swing_clockwise_cable)
                    {
                        const double dist_sw_to_cable =
                            swingDistanceAlongTurnDirectionDeg(start_traj, reset_cable, cw_reset);
                        const double dist_sw_to_dig =
                            swingDistanceAlongTurnDirectionDeg(start_traj, reset_target, cw_reset);
                        if (dist_sw_to_cable < dist_sw_to_dig)
                            reset_swing_cable_first_flag_ = true;
                    }
                }
                else
                {
                    // 兜底：按当前与目标的最小行程方向，并避开 unload_angle
                    // const double shortest_delta = angleDiff(cur_sw, raw_sw);
                    // cw_reset = (shortest_delta < 0.0);
                    // if (unload_angle_valid && isAngleBetween(unload_angle, cur_sw, raw_sw, cw_reset))
                    // {
                    //     cw_reset = !cw_reset;
                    //     std::cout<<"===2===unload_angle is between cur_sw and cw_reset!   "<<std::endl;
                    // }


                    cw_reset = swing_clockwise_2;

                }

                // 阶段2方向对外发布使用 swing_clockwise_2
                swing_clockwise_2 = cw_reset;
                reset_swing_direction_computed_this_entry_ = true;
            }
            else
            {
                // 阶段2方向对外发布/展开逻辑以 swing_clockwise_2 为准
                cw_reset = swing_clockwise_2;
            }

            // std::cout<<"===2===cw_reset:    "<<static_cast<int>(cw_reset)<<std::endl;

            // if (!last_reset_swing_raw_valid_)
            // {
            //     // 规划起始点：优先卸载角（与卸载/复位路径规划一致）；unload_angle 无效时用当前机位。
            //     // 若卸载角与真实 current_angle_swing_ 差较大，首帧发布相对编码器可能有一阶跃；需连续则改用 current_angle_swing_。
            //     const double prev_align =
            //         std::isfinite(unload_angle) ? normalizeAngle(unload_angle)
            //                                     : normalizeAngle(current_angle_swing_);
            //     const double aligned = unwrapTrajectorySwing(prev_align, raw_sw, cw_reset);
            //     reset_swing_unwrap_offset_deg_ = aligned - raw_sw;
            //     reset_swing_phase_ = 0;
            //     last_reset_swing_raw_valid_ = true;
            // }
            // else
            // {
            //     const double prev_raw = last_reset_swing_raw_deg_;
            //     if (reset_swing_phase_ == 0)
            //     {
            //         if (!cw_reset && raw_sw < prev_raw - 180.0)
            //             reset_swing_phase_ = 1;
            //         else if (cw_reset && raw_sw > prev_raw + 180.0)
            //             reset_swing_phase_ = -1;
            //     }
            // }

            // const double reset_effective_offset =
            //     reset_swing_unwrap_offset_deg_ + static_cast<double>(reset_swing_phase_) * 360.0;
            // current_traj.orientation.x = raw_sw + reset_effective_offset;
            // last_reset_swing_raw_deg_ = raw_sw;

            {
                // const double reset_target = normalizeAngle(RGp_dig_point_angle);
                // const double start_traj = std::isfinite(unload_angle) ? normalizeAngle(unload_angle)
                //                                                         : normalizeAngle(current_angle_swing_);
        
                if(!cw_reset)
                {   
                    if(start_traj >= 180.0)
                    {
                        if(reset_target > 0.0 && reset_target < 180.0)
                        {
                            reset_swing_phase_ = 1;
                        }
                        else
                        {
                            reset_swing_phase_ = 0;
                        }
                    }
                    else
                    {
                        reset_swing_phase_ = 0;
                    }
                }
                else
                {
                    if(start_traj <= 180.0)
                    {
                        if(reset_target > 180.0 && reset_target < 360.0)
                        {
                            reset_swing_phase_ = -1;
                        }
                        else
                        {
                            reset_swing_phase_ = 0;
                        }
                    }
                    else
                    {       
                        reset_swing_phase_ = 0;
                    }
                }
        
                current_traj.orientation.x = reset_target + static_cast<double>(reset_swing_phase_) * 360.0;


                

                // 线缆防碰接管：复位首帧判定 reset_swing_cable_first_flag_（沿复位弧先经过 rear_cable_deg 再往挖掘）；非 [lb,ub] 采样
                if (reset_swing_cable_first_flag_)
                {
                    require_takeover = true;
                    require_takeover_flag_.data = true;
                    if (takeover_pub_count_cable_reset < 5)
                    {
                        pub_takeover.publish(require_takeover_flag_);
                        ++takeover_pub_count_cable_reset;
                    }
                    // 接管告警属异常路径，保留但限频，避免每周期刷屏阻塞主循环
                    ROS_WARN_THROTTLE(1.0,
                        "===2===planned unload swing hits cable sector ! Please takeover! swing_clockwise_cable=%d",
                        static_cast<int>(swing_clockwise_cable));

                }
            }
        }
    }


    // 回转移出矿卡后动臂and arm才允许下落：以矿卡中心为参考，当前回转角与指向中心的回转角最小夹角 > 阈值
    // 与 calculateTruckVertices 一致：center_point[0]=Δ北，center_point[1]=−Δ东；0°南/90°东/180°北/270°西
    // const double cx_cp = center_point[0];
    // const double cy_cp = center_point[1];
    // const double swing_toward_truck_center =
    //     normalizeAngle(std::atan2(-cy_cp, -cx_cp) * 180.0 / M_PI);

    // double swing_diff_to_center = current_angle_swing_ - swing_toward_truck_center;
    double swing_diff_to_center = wrapTo180Deg(current_angle_swing_ - unload_angle);
    bool bucket_moved_out_of_truck =
        (std::fabs(swing_diff_to_center) > reset_swing_clear_truck_center_deg);

    if (!bucket_moved_out_of_truck)
    {
        current_traj.orientation.y = init_reset_boom_4_;
        current_traj.orientation.z = init_reset_arm_4_;
        current_traj.orientation.w = last_traj.orientation.w;

    }
    else
    {
        // if (std::isfinite(RGp_dig_point_angle))
        // {
        //     const double boom_by_swing = calculateTargetBoomBySwingPercent(
        //         current_angle_swing_, RGp_dig_point_angle, traj_reset.orientation.y);
        //     if (std::isfinite(boom_by_swing))
        //         current_traj.orientation.y = boom_by_swing;
        //     else
        //         current_traj.orientation.y = traj_reset.orientation.y;
        // }
        // else
        // {
        //     current_traj.orientation.y = traj_reset.orientation.y;
        // }  

        

        // 铲斗齿尖距地面<50cm 且 当前回转距轨迹规划的回转角15度内，斗杆强制到垂直角度
        // double bucket_to_ground = std::fabs(bucket_terrain);  // 齿尖到地面的距离
        double swing_diff = std::fabs(current_angle_swing_ - traj_reset.orientation.x);
        if (swing_diff > 180.0) swing_diff = 360.0 - swing_diff;

        // if(swing_diff <= RESET_SWING_ANGLE_THRESH)
        // {
        //     current_traj.orientation.y = -30.0;//强制动臂下落
        // // std::cout<<"*------6666666current_traj.orientation.y:    "<<current_traj.orientation.y<<std::endl;
        // }
        if (swing_diff <= RESET_SWING_ANGLE_THRESH)
        {
            boom_fall_flag = true;//强制动臂下落
        }

        if (boom_fall_flag)
        {
            current_traj.orientation.y = -30.0;//强制动臂下落
        }
        else
        {
            current_traj.orientation.y = traj_reset.orientation.y;
        }


        if (bucket_terrain > 0 && bucket_terrain < reset_bucket_ground_dist_thresh && ef_tip_rotation_z < -0.0 && swing_diff <= RESET_SWING_ANGLE_THRESH)
        {
            current_traj.orientation.z = ARM_VERTICAL_ANGLE;
        }
        else
        {
            current_traj.orientation.z = traj_reset.orientation.z;
        }

    }



    current_traj.position.z = 2.0;

    // 阶段2 周期打印已移除（仅保留阶段3 的周期日志）
    ROS_DEBUG("===2===cw2=%d swing_cmd=%.3f",
              static_cast<int>(swing_clockwise_2), current_traj.orientation.x);

}

// ===========挖掘阶段3======================

// ################################0625实机测试版本
void traj_fusion_node::digStatusProcess()
{
    start5_angle_flag = false;

    bucket_change_rate = 2.0;

    {
        init_dig_bucket_3_ = bucket_angle_current;   // record bucket angle in state 3.0
        init_dig_swing_3_ = current_angle_swing_;
        init_dig_rtk_heading = rtk_heading;
        init_dig_bucket_height = ef_tip_rotation_z - exc_bucket_H;
        swing_percent = 0.0;
        last_percent = 0.0;
        last_unload_swing_raw_valid_ = false;
        unload_swing_phase_ = 0;
        // unload_swing_unwrap_offset_deg_ = 0.0;
        last_traj_dig_valid_ = false;
    }


    // 车尾角/回转方向的计算提前到阶段3完成：仅在进入阶段3首周期计算一次
    // if (!stop_swing_main_done_in_dig_stage_)
    // {
    //     stop_swing_main();
    //     stop_swing_main_done_in_dig_stage_ = true;
    // }

    stop_swing_main();

    if(model_traj_valid_)
    {
        act_bucket_angle = radiansToDegrees(model_traj.orientation.w);
        act_arm_angle = radiansToDegrees(model_traj.orientation.z);
        act_boom_angle = radiansToDegrees(model_traj.orientation.y);
        // current_traj.orientation.y = act_boom_angle;
    }
    else
    {
        act_bucket_angle = 10000;
        act_arm_angle = 10000;
        act_boom_angle = 10000;
        // current_traj.orientation.y = traj_dig.orientation.y;

    }

    //“动臂原始目标角”先算出来，前 10 秒原样下发；超时后以 1 deg/s 积分作为抬升下限，同时仍允许原始轨迹给更高的动臂角。
    const double raw_boom_cmd = model_traj_valid_ ? act_boom_angle : traj_dig.orientation.y;
    static bool dig_boom_integral_valid = false;
    static double dig_boom_integral_cmd_deg = 0.0;
    static double dig_boom_integral_stage_start_sec = -1.0;
    static ros::Time dig_boom_integral_last_time;

    const ros::Time now = ros::Time::now();
    const double dig_stage_start_sec = dig_stage_start_time_.toSec();
    if (std::fabs(dig_boom_integral_stage_start_sec - dig_stage_start_sec) > 1e-9)
    {
        dig_boom_integral_valid = false;
        dig_boom_integral_cmd_deg = 0.0;
        dig_boom_integral_stage_start_sec = dig_stage_start_sec;
        dig_boom_integral_last_time = ros::Time();
    }

    const double elapsed = dig_stage_start_time_.isZero() ? 0.0 : (now - dig_stage_start_time_).toSec();
    if (elapsed <= dig_boom_free_sec_)
    {
        dig_boom_integral_valid = false;
        dig_boom_integral_cmd_deg = 0.0;
        dig_boom_integral_last_time = ros::Time();
        current_traj.orientation.y = raw_boom_cmd;
    }
    else
    {
        if (!dig_boom_integral_valid)
        {
            dig_boom_integral_cmd_deg =
                std::isfinite(last_traj.orientation.y) && std::fabs(last_traj.position.z - 3.0) < 1e-9
                    ? last_traj.orientation.y
                    : raw_boom_cmd;
            dig_boom_integral_last_time = dig_stage_start_time_ + ros::Duration(dig_boom_free_sec_);
            dig_boom_integral_valid = true;
        }

        double dt = (now - dig_boom_integral_last_time).toSec();
        if (!std::isfinite(dt) || dt < 0.0)
            dt = 0.0;

        dig_boom_integral_cmd_deg += dig_boom_integrate_rate_deg_per_sec_ * dt;
        dig_boom_integral_last_time = now;
        current_traj.orientation.y = std::max(raw_boom_cmd, dig_boom_integral_cmd_deg);
        // current_traj.orientation.y = raw_boom_cmd;
    }



    //boom
    // traj_dig.orientation.y = msg->orientation.y;
    // traj_dig.orientation.y = act_boom_angle;
    
    //融合后的斗杆角
    // double arm_fusion = min(msg->orientation.z, act_arm_angle);
    double arm_fusion = traj_dig.orientation.z;

    //斗杆目标角必须要超过阈值
    if(arm_fusion > -180.0 + arm_target_threshold)
    {
        arm_target_angle = -180.0 + arm_target_threshold;
    }
    else
    {
        arm_target_angle = arm_fusion;
    }

    //未达到斗杆目标角之前用斗杆切削角
    if(arm_angle_current > arm_target_angle + 10.0)
    {
        // 触发一次减 20 分
        reward_ -= 20;

        digover_boom_applied_ = false;

        current_traj.orientation.w = std::min(arm_cut_angle,act_bucket_angle);
        // current_traj.orientation.w = act_bucket_angle;
        // double bucket_diff = std::fabs(act_bucket_angle - arm_cut_angle);
        // if (bucket_diff <= 15.0)
        // {
        //     current_traj.orientation.w = act_bucket_angle;
        // }
        // else
        // {
        //     current_traj.orientation.w = arm_cut_angle;
        // }


        //boom
        
        // dig_OP_detection();
        // traj_dig.orientation.y += dig_OP_level.data * boom_adjust_range;
    }
    //到达后铲斗端平
    else
    {
        current_traj.orientation.w = bucket_flat_angle - bucket_offset;//铲斗往里扣一点点
        digover_boom_applied_ = true;
        // 车尾角改在阶段4 开头调用 stop_swing_main()，只算一次
    }

    //swing
    current_traj.orientation.x = traj_dig.orientation.x;
    //arm
    current_traj.orientation.z = arm_target_angle;

    // dig_traj_adjust();

    // pub_dig_OP_level.publish(dig_OP_level);

    current_traj.position.z = 3.0;

    // 阶段3（0625 非模型版）单行聚合日志：原先分散在各分支的 7 行 std::endl 改为每周期一行
    ROS_INFO_STREAM("===3===dig"
                    << " model_valid=" << model_traj_valid_
                    << " act_boom=" << act_boom_angle
                    << " traj_dig_boom=" << traj_dig.orientation.y
                    << " raw_boom=" << current_traj.orientation.y
                    // << " elapsed=" << elapsed
                    // << " integral=" << dig_boom_integral_cmd_deg
                    // << " cmd_swing=" << current_traj.orientation.x
                    // << " cmd_boom=" << current_traj.orientation.y
                    // << " cmd_arm=" << current_traj.orientation.z
                    // << " cmd_bucket=" << current_traj.orientation.w
                    );

}
// ################################0625实机测试版本


// ################################0628 模型直接控制测试版本
void traj_fusion_node::digStatusProcess_model()
{
    start5_angle_flag = false;

    bucket_change_rate = 2.0;

    {
        init_dig_bucket_3_ = bucket_angle_current;   // record bucket angle in state 3.0
        init_dig_swing_3_ = current_angle_swing_;
        init_dig_rtk_heading = rtk_heading;
        init_dig_bucket_height = ef_tip_rotation_z - exc_bucket_H;
        swing_percent = 0.0;
        last_percent = 0.0;
        last_unload_swing_raw_valid_ = false;
        unload_swing_phase_ = 0;
        // unload_swing_unwrap_offset_deg_ = 0.0;
        last_traj_dig_valid_ = false;
    }


    stop_swing_main();

    if(model_traj_valid_)
    {
        act_bucket_angle = radiansToDegrees(model_traj.orientation.w);
        act_arm_angle = radiansToDegrees(model_traj.orientation.z);
        act_boom_angle = radiansToDegrees(model_traj.orientation.y);
        // current_traj.orientation.y = act_boom_angle;
    }
    else
    {
        act_bucket_angle = traj_dig.orientation.w;
        act_arm_angle = traj_dig.orientation.z;
        act_boom_angle = traj_dig.orientation.y;
        // current_traj.orientation.y = traj_dig.orientation.y;

    }

    //“动臂原始目标角”先算出来，前 10 秒原样下发；超时后以 1 deg/s 积分作为抬升下限，同时仍允许原始轨迹给更高的动臂角。
    const double raw_boom_cmd = act_boom_angle;

    static bool dig_boom_integral_valid = false;
    static double dig_boom_integral_cmd_deg = 0.0;
    static double dig_boom_integral_stage_start_sec = -1.0;
    static ros::Time dig_boom_integral_last_time;

    const ros::Time now = ros::Time::now();
    const double dig_stage_start_sec = dig_stage_start_time_.toSec();
    if (std::fabs(dig_boom_integral_stage_start_sec - dig_stage_start_sec) > 1e-9)
    {
        dig_boom_integral_valid = false;
        dig_boom_integral_cmd_deg = 0.0;
        dig_boom_integral_stage_start_sec = dig_stage_start_sec;
        dig_boom_integral_last_time = ros::Time();
    }

    const double elapsed = dig_stage_start_time_.isZero() ? 0.0 : (now - dig_stage_start_time_).toSec();
    if (elapsed <= dig_boom_free_sec_)
    {
        dig_boom_integral_valid = false;
        dig_boom_integral_cmd_deg = 0.0;
        dig_boom_integral_last_time = ros::Time();
        current_traj.orientation.y = raw_boom_cmd;
    }
    else
    {
        if (!dig_boom_integral_valid)
        {
            dig_boom_integral_cmd_deg =
                std::isfinite(last_traj.orientation.y) && std::fabs(last_traj.position.z - 3.0) < 1e-9
                    ? last_traj.orientation.y
                    : raw_boom_cmd;
            dig_boom_integral_last_time = dig_stage_start_time_ + ros::Duration(dig_boom_free_sec_);
            dig_boom_integral_valid = true;
        }

        double dt = (now - dig_boom_integral_last_time).toSec();
        if (!std::isfinite(dt) || dt < 0.0)
            dt = 0.0;

        dig_boom_integral_cmd_deg += dig_boom_integrate_rate_deg_per_sec_ * dt;
        dig_boom_integral_last_time = now;
        current_traj.orientation.y = std::max(raw_boom_cmd, dig_boom_integral_cmd_deg);
        // current_traj.orientation.y = raw_boom_cmd;
    }

    //动臂提升适应性改进，模型基础上加偏置
    // dig_traj_adjust();

    //swing
    current_traj.orientation.x = traj_dig.orientation.x;
    //arm
    current_traj.orientation.z = act_arm_angle;
    //bucket
    current_traj.orientation.w = act_bucket_angle;

    current_traj.position.z = 3.0;

    // 阶段3（0628 模型直控版）单行聚合日志：原先分散在各分支的 5 行 std::endl 改为每周期一行，
    // 保留全部字段，便于对照“轨迹停更”与“数值冻结”两种卡住形态。
    ROS_INFO_STREAM("===3===dig_model"
                    << " model_valid=" << model_traj_valid_
                    << " act_boom=" << act_boom_angle
                    << " act_arm=" << act_arm_angle
                    << " act_bucket=" << act_bucket_angle
                    << " raw_boom=" << raw_boom_cmd
                    << " elapsed=" << elapsed
                    << " integral=" << dig_boom_integral_cmd_deg
                    << " cmd_swing=" << current_traj.orientation.x
                    << " cmd_boom=" << current_traj.orientation.y
                    << " cmd_arm=" << current_traj.orientation.z
                    << " cmd_bucket=" << current_traj.orientation.w);

}

// ################################0628 模型直接控制测试版本


//==============提升卸载阶段4=================
void traj_fusion_node::raiseAndUnloadStausProcess()
{
    bucket_change_rate = 2.0;

    init_reset_boom_4_ = boom_angle_current;   // record boom angle in state 4.0
    init_reset_arm_4_ = arm_angle_current;// record arm angle in state 4.0

    



    // 车尾角/回转方向已在阶段3计算完成；此处直接使用 stop_swing_main 设置的结果

    if (!last_traj_dig_valid_)
    {
        init_angle_swing_4_ = init_dig_swing_3_;

        init_rtk_heading = init_dig_rtk_heading;
        init_bucket_height = init_dig_bucket_height;

        // std::cout<<"***********init_bucket_height:    "<<init_bucket_height<<std::endl;
        // dig_point_angle = init_dig_swing_3_;

        last_traj_dig_valid_ = true;
    }

    current_traj.orientation.y = traj_unload.orientation.y;
    current_traj.orientation.z = traj_unload.orientation.z;
    current_traj.orientation.w = std::min(init_dig_bucket_3_ - 0.0, bucket_flat_angle - bucket_offset);
    


    // std::cout << "===4====swing_clockwise_4=======:    " << swing_clockwise_4 << std::endl;


    //为了提高卸载效率，当回转到 truck_angle_for_target 附近后，铲斗外翻一点(越往外翻角度越大)，卸载过程中的铲斗角度不能比（端平角+20度）小

//    std::cout<<"+++++++ zongti  current_traj.orientation.w:    "<<current_traj.orientation.w<<std::endl;

    // double Bheight_current = bucket_point[2] - exc_bucket_H;
    // exc_bucket_H 按铲斗开度线性衰减：端平 bucket_flat_angle-bucket_offset 时系数 1，再开 35° 系数 0
    const double bucket_flat_ref_deg = bucket_flat_angle - bucket_offset;
    const double bucket_exc_open_span_deg = 35.0;
    double exc_bucket_H_scale = 1.0;
    if (bucket_exc_open_span_deg > 1e-9) {
        exc_bucket_H_scale =
            1.0 - (bucket_angle_current - bucket_flat_ref_deg) / bucket_exc_open_span_deg;
    }
    exc_bucket_H_scale = std::max(0.0, std::min(1.0, exc_bucket_H_scale));

    // double Bheight_current = ef_tip_rotation_z - relative_z - exc_bucket_H * exc_bucket_H_scale;
    double Bheight_current = ef_tip_rotation_z - exc_bucket_H * exc_bucket_H_scale;
    // double Bheight_target = center_point[2] + truck_edge_offset;
    double Bheight_target = truck_to_center + unload_height_offset;
    double Bheight_yueguo = truck_to_center + truck_edge_offset;
    double target_angle_swing_;

    // std::cout<<"***********Bheight_tip:    "<<ef_tip_rotation_z - relative_z<<std::endl;
    // std::cout<<"***********Bheight_current:    "<<Bheight_current<<std::endl;
    // std::cout<<"***********Bheight_target:    "<<Bheight_target<<std::endl;
    // std::cout<<"***********Bheight_yueguo:    "<<Bheight_yueguo<<std::endl;
    // std::cout<<"exc_bucket_H:    "<<exc_bucket_H<<std::endl;

    // std_msgs::Float32 bucket_target_height_msg;
    // bucket_target_height_msg.data = Bheight_target;
    // pub_bucket_target_height.publish(bucket_target_height_msg);

    // std::cout<<"***********Last stage dig_point_angle is:    "<<dig_point_angle<< std::endl;

    if (swing_clockwise_4)//顺时针
    {
        target_angle_swing_ = normalizeAngle(truck_swive_angle_msg.data + bucket_to_angle);
    }
    else//逆时针
    {
        target_angle_swing_ = normalizeAngle(truck_swive_angle_msg.data - bucket_to_angle);
    }


    // truck_swive_angle_msg.data 需在 init_angle_swing_4_ 与 unload_angle 之间（按回转方向）才使用，否则用 unload_angle
    double truck_angle_for_target = isAngleBetween(truck_swive_angle_msg.data, start_dig_angle, unload_angle, swing_clockwise_4)
                                       ? target_angle_swing_
                                       : unload_angle;

    // std::cout<<"*truck_angle_for_target:    "<<truck_angle_for_target<<std::endl;

    truck_angle_for_target = normalizeAngle(truck_angle_for_target);

    // std::cout<<"***********truck_angle_for_target is:    "<<truck_angle_for_target<< std::endl;


    // double swingConstraint = calculateTargetSwing(Bheight_current, Bheight_target, truck_angle_for_target);
    // std::cout<<"***********swingConstraint:    "<<swingConstraint<<std::endl;

    if (Bheight_current >= Bheight_yueguo || mpc_switch2_flag)
    {
        swing_percent = 1.0;
        // bucket_over_truck_msg.data = 1;
    }
    else
    {
        // bucket_over_truck_msg.data = 0;
    }

    // pub_bucket_over_truck.publish(bucket_over_truck_msg);

    double unload_swing_raw_deg;
    if (std::fabs(swing_percent - 1.0) < 1e-9)
    {
        unload_swing_raw_deg = normalizeAngle(unload_angle);
        // std::cout<<"***********The bucket already overstepped the mining card!! The swingConstraint is:    "<<swingConstraint<< std::endl;
    }
    else
    {
        double swingConstraint = calculateTargetSwing(Bheight_current, Bheight_target, truck_angle_for_target);

        unload_swing_raw_deg = normalizeAngle(swingConstraint);
    }

    // 阶段4 周期打印已移除（仅保留阶段3 的周期日志）
    ROS_DEBUG("===4===unload_swing_raw_deg=%.3f", unload_swing_raw_deg);

    // 轨迹层：回转方向 + 回转起始角 → 相位 unload_swing_phase_ ∈{-1,0,+1}；发布角 = raw + 基准偏置 + phase×360°
    // 逆时针 & 起始<180°：轨迹进入 [0,180°] → phase=+1；逆时针 & 起始≥180°：不修正。
    // 顺时针 & 起始<180°：不修正；顺时针 & 起始≥180°：轨迹进入 [180°,360°) → phase=-1。
    {
        const double new_target = unload_swing_raw_deg;
        // const double new_target = unload_angle;
        const bool cw = swing_clockwise_4;
        double start_angle = start_dig_angle;

        if(start5_angle_flag)
        {
            start_angle = front_lane_deg;

        }

        ROS_DEBUG("===4===start_angle=%.3f", start_angle);

        if(!cw)
        {
            if(start_angle >= 180.0)
            {
                if(new_target > 0.0 && new_target < 180.0)
                {
                    unload_swing_phase_ = 1;
                }
                else
                {
                    unload_swing_phase_ = 0;
                }
            }
            else
            {
                unload_swing_phase_ = 0;
            }
        }
        else
        {
            if(start_angle <= 180.0)
            {
                if(new_target > 180.0 && new_target < 360.0)
                {
                    unload_swing_phase_ = -1;
                }
                else
                {
                    unload_swing_phase_ = 0;
                }
            }
            else
            {       
                    unload_swing_phase_ = 0;
            }
        }

        current_traj.orientation.x = new_target + static_cast<double>(unload_swing_phase_) * 360.0;

        //车尾角发布：unload_angle先单调处理
        double unload_angle_mono = unload_angle + static_cast<double>(unload_swing_phase_) * 360.0;
        // std::cout << "===444444444444====unload_swing_phase_:    " << unload_swing_phase_ << std::endl;
        // std::cout << "===444444444444====unload_angle_mono:    " << unload_angle_mono << std::endl;

        // 车位角发布，单调处理 
        if (swing_clockwise_4)
            truck_swive_angle = unload_angle_mono + truck_tail_offset_from_unload_deg_;
        else
            truck_swive_angle = unload_angle_mono - truck_tail_offset_from_unload_deg_;

        // std::cout << "===444444444444====truck_swive_angle:    " << truck_swive_angle << std::endl;

        truck_swive_angle_pub_msg.data = truck_swive_angle;

        pub_truck_swive_angle.publish(truck_swive_angle_pub_msg);


        const double cur_swing = current_angle_swing_;
        const double cable_swing = rear_cable_deg;
        const double unload_swing = normalizeAngle(unload_angle);

        // if (swing_clockwise_4)
        //     truck_tail_angle = normalizeAngle(unload_swing + truck_tail_offset_from_unload_deg_);
        // else
        //     truck_tail_angle = normalizeAngle(unload_swing - truck_tail_offset_from_unload_deg_);

        // std::cout<<"===4===cur_swing:    "<<cur_swing<<std::endl;
        // std::cout<<"===4===cable_swing:    "<<cable_swing<<std::endl;
        // std::cout<<"===4===unload_swing:    "<<unload_swing<<std::endl;
        // std::cout<<"===4===truck_tail_angle:    "<<truck_tail_angle<<std::endl;

        // const double dist_sw_to_tail = swingDistanceAlongTurnDirectionDeg(cur_swing, truck_tail_angle, swing_clockwise_4);

        const double lb = normalizeAngle(unload_swing - 30);
        const double ub = normalizeAngle(unload_swing + 30);
        // std::cout<<"===4===lb:    "<<lb<<std::endl;
        // std::cout<<"===4===ub:    "<<ub<<std::endl;

        //线缆防撞不断确认
        if (!unload_swing_cable_first_flag_ && !isAngleOnShorterArcBetween(cur_swing, lb, ub))
        {    
            const bool cable_swing_clockwise = isClockwiseShortestTurn(cur_swing, cable_swing);
            const bool current_swing_clockwise = isClockwiseShortestTurn(cur_swing, unload_swing);
    
            // std::cout<<"===4===swing_clockwise_cable:    "<<swing_clockwise_cable<<std::endl;
    
            if (current_swing_clockwise == cable_swing_clockwise)
            {
                const double dist_sw_to_cable = swingDistanceAlongTurnDirectionDeg(cur_swing, cable_swing, cable_swing_clockwise);
                const double dist_sw_to_unload = swingDistanceAlongTurnDirectionDeg(cur_swing, unload_swing, cable_swing_clockwise);
                if (dist_sw_to_cable < dist_sw_to_unload)
                    unload_swing_cable_first_flag_ = true;
            }

        }

        // 线缆区约束：卸载阶段规划回转指令落在 [angle_cable_lb_deg, angle_cable_ub_deg] 的较短圆弧（常配成 90°）内则触发接管请求
        if (unload_swing_cable_first_flag_)
        {
            require_takeover = true;
            require_takeover_flag_.data = true;
            if (takeover_pub_count_cable_unload < 5)
            {
                pub_takeover.publish(require_takeover_flag_);
                ++takeover_pub_count_cable_unload;
            }
            // 接管告警属异常路径，保留但限频，避免每周期刷屏阻塞主循环
            ROS_WARN_THROTTLE(1.0,
                "===4===planned unload swing hits cable sector ! Please takeover! swing_clockwise_cable=%d",
                static_cast<int>(swing_clockwise_cable));

        }

    }


    const double unload_angle_diff = std::fabs(angleDiff(current_angle_swing_, unload_angle));
    
    // unload_angle_diff 进入阈值：动臂不得高于「阶段4 上一控制周期、限幅后」的下发角（last_unload_boom_cmd_*），不用 last_traj（避免沿用上一阶段挖掘末帧）
    if (unload_angle_diff <= unload_bucket_planner_enter_deg_ && last_unload_boom_cmd_valid_)
        current_traj.orientation.y = std::min(current_traj.orientation.y, last_unload_boom_cmd_deg_);



    // 铲斗：距卸载回转角 > unload_bucket_planner_enter_deg_ 保持端平角；进入阈值内改用规划 traj_unload.orientation.w（滞回）
    // 角距与 unload_angle_diff 一致：最小夹角，不按回转方向区分
    // 已切规划后：若仍在 unload 角阈值内则刷新记录的铲斗当前角；若超调则发上次记录角

    // std::cout << "===4====truck_swive_angle_msg.data:    " << truck_swive_angle_msg.data << std::endl;
    static constexpr double kSwingReachedTruckAngleEpsDeg = 3.0;
    // std::cout<<"+++++++ current_angle_swing_:    "<<current_angle_swing_<<std::endl;
    float current_angle_dif = 0;

    const double curr_swing_n = current_angle_swing_;
    bool crossed_truck_target_this_step = false;
    if (unload_truck_cross_prev_valid_)
    {
        const bool cw = swing_clockwise_4;
        const double d_pc =
            swingDistanceAlongTurnDirectionDeg(unload_truck_cross_prev_swing_deg_, curr_swing_n, cw);
        // 本步沿 cw 的轨迹弧是否扫过 truck：prev→target + target→curr ≈ prev→curr（高速跨步也不漏）
        if (d_pc >= 1.0)
        {
            const double d_pt = swingDistanceAlongTurnDirectionDeg(
                unload_truck_cross_prev_swing_deg_, truck_swive_angle_msg.data, cw);
            const double d_tc =
                swingDistanceAlongTurnDirectionDeg(truck_swive_angle_msg.data, curr_swing_n, cw);
            crossed_truck_target_this_step = std::fabs(d_pt + d_tc - d_pc) <= 2.0;
        }
    }
    unload_truck_cross_prev_swing_deg_ = curr_swing_n;
    unload_truck_cross_prev_valid_ = true;

    current_angle_dif = static_cast<float>(std::fabs(angleDiff(current_angle_swing_, truck_swive_angle_msg.data)));
    // std::cout<<"+++++++diff currentad_angle_dif:    "<<current_angle_dif<<std::endl;

    if (unload_angle_diff <= unload_bucket_planner_enter_deg_)
    {
        unload_angle_dif_flag = true;
    }

    if (current_angle_dif <= static_cast<float>(kSwingReachedTruckAngleEpsDeg) || crossed_truck_target_this_step)
        unload_swing_passed_truck_target_latch_ = true;

    current_angle_dif_flag = unload_swing_passed_truck_target_latch_;


        // 在 unload_angle 左右阈值范围内时，才刷新“上次阈值内的铲斗角”（与动臂单调限制共用 unload_angle_diff）
        if (unload_angle_dif_flag)   //8deg
        {
            if (unload_angle_diff <= unload_bucket_planner_enter_deg_)
            {
                // 动臂真实角贴近规划角后才允许铲斗外翻卸载
                // if (std::fabs(boom_angle_current - current_traj.orientation.y) <= 5.0)
                // {
                //     bucket_change_rate = 100.0;
                //     unload_planner_bucket_recorded_deg_ = bucket_angle_current;
                //     unload_planner_bucket_record_valid_ = true;
                //     // current_traj.orientation.w = std::max(traj_unload.orientation.w, bucket_flat_angle - bucket_offset + 35.0);
                //     current_traj.orientation.w = std::max(20.0, bucket_flat_angle - bucket_offset + bucket_ahead_unload_deg);
                //     // std::cout<<"+++++++ xiezai fanweinei  current_traj.orientation.w:    "<<current_traj.orientation.w<<std::endl;
                //     // std::cout<<"+++++++  unload_angle:    "<<unload_angle<<std::endl;
                // }

                bucket_change_rate = 100.0;
                unload_planner_bucket_recorded_deg_ = bucket_angle_current;
                unload_planner_bucket_record_valid_ = true;
                // current_traj.orientation.w = std::max(traj_unload.orientation.w, bucket_flat_angle - bucket_offset + 35.0);
                current_traj.orientation.w = std::max(20.0, bucket_flat_angle - bucket_offset + bucket_ahead_unload_deg);
                // std::cout<<"+++++++ xiezai fanweinei  current_traj.orientation.w:    "<<current_traj.orientation.w<<std::endl;
                // std::cout<<"+++++++  unload_angle:    "<<unload_angle<<std::endl;

                //为防止卸载高度过高，进入阈值后使用最终姿态角
                current_traj.orientation.y = traj_unload_posture.orientation.y;



                //4转2预计算
                // if (!reset_swing_direction_precomputed_in_unload_ && std::isfinite(unload_angle))
                // {
                //     const double reset_target = normalizeAngle(RGp_dig_point_angle);
                //     const double start_traj = normalizeAngle(unload_angle);
                //     const double shortest_delta = angleDiff(start_traj, reset_target);

                //     std::cout<<"===44444444===precalculate  start_traj:    "<<start_traj<<std::endl;
                //     std::cout<<"===44444444===precalculate  reset_target:    "<<reset_target<<std::endl;

                //     if (std::fabs(shortest_delta) < 1e-9)
                //         swing_clockwise_2 = false;
                //     else
                //         swing_clockwise_2 = (shortest_delta < 0.0);
                //     std::cout<<"===44444444===precalculate swing_clockwise_2:    "<<swing_clockwise_2<<std::endl;

                //     reset_swing_direction_precomputed_in_unload_ = true;        

                // }


            }
            else
            {    
                // if (unload_planner_bucket_record_valid_)
                // {
                //     current_traj.orientation.w = unload_planner_bucket_recorded_deg_;
                //     unload_planner_bucket_record_valid_ = false;

                // // std::cout<<"+++++++ chaotiao  current_traj.orientation.w:    "<<current_traj.orientation.w<<std::endl;
                // }
                // else
                //     {current_traj.orientation.w = std::max(traj_unload.orientation.w, bucket_flat_angle - bucket_offset + 35.0);;

                //     // std::cout<<"+++++++ xiezaijiao  current_traj.orientation.w:    "<<current_traj.orientation.w<<std::endl;
                //     }
                
            }
        }
        else
          {
            if (current_angle_dif_flag)
             {
                current_traj.orientation.w = bucket_flat_angle - bucket_offset + bucket_ahead_unload_deg;
            //   std::cout<<"+++++++cheweijiao current_traj.orientation.w:    "<<current_traj.orientation.w<<std::endl;
            //   std::cout<<"+++++++50 current_angle_swing_:    "<<current_angle_swing_<<std::endl;
             }
            

            else
             {
                // current_traj.orientation.w = bucket_flat_angle - bucket_offset;
                current_traj.orientation.w = std::min(init_dig_bucket_3_ - 0.0, bucket_flat_angle - bucket_offset);
                
            //   std::cout<<"+++++++ kaishi  current_traj.orientation.w:    "<<current_traj.orientation.w<<std::endl;
             }
          }   

    // std::cout<<"current_angle_swing_:    "<<current_angle_swing_<<std::endl;
    current_traj.position.z = 4.0;

    // 阶段4 周期打印已移除（仅保留阶段3 的周期日志）
    ROS_DEBUG("===4===cw4=%d swing_cmd=%.3f",
              static_cast<int>(swing_clockwise_4), current_traj.orientation.x);




 }

void traj_fusion_node::waitStatusProcess()
{
    bucket_change_rate = 100.0;
    start5_angle_flag = true;

    // if (truck_dock_success_flag_)
    // {
    //     // stop_swing_main();
    //     const double cur_sw = normalizeAngle(front_lane_deg);
    //     const double unload_sw = normalizeAngle(unload_angle);
    //     double d = unload_sw - cur_sw;
    //     while (d > 180.0)
    //         d -= 360.0;
    //     while (d <= -180.0)
    //         d += 360.0;
    //     if (std::fabs(d) < 1e-9)
    //         swing_clockwise_4 = false;
    //     else
    //         swing_clockwise_4 = (d < 0.0);

    //     std::cout<<"===5555555===precalculate:swing_clockwise_4:    "<<swing_clockwise_4<<std::endl;

        
    // }

    {
        const double new_target = front_lane_deg;
        // const double start_angle = normalizeAngle(init_dig_swing_3_);
        // const double start_angle = std::isfinite(RGp_dig_point_angle) ? normalizeAngle(RGp_dig_point_angle) : current_angle_swing_;
        // const double shortest_delta = angleDiff(start_angle, new_target);

        // if (std::fabs(shortest_delta) < 1e-9)
        //     swing_clockwise_5 = true;
        // else
        //     swing_clockwise_5 = (shortest_delta < 0.0);

        if(!swing_clockwise_5)
        {
            if(start3_angle >= 180.0)
            {
                if(new_target > 0.0 && new_target < 180.0)
                {
                    wait_swing_phase_ = 1;
                }
                else
                {
                    wait_swing_phase_ = 0;
                }
            }
            else
            {
                wait_swing_phase_ = 0;
            }
        }
        else
        {
            if(start3_angle <= 180.0)
            {
                if(new_target > 180.0 && new_target < 360.0)
                {
                    wait_swing_phase_ = -1;
                }
                else
                {
                    wait_swing_phase_ = 0;
                }
            }
            else
            {       
                wait_swing_phase_ = 0;
            }
        }

        current_traj.orientation.x = new_target + static_cast<double>(wait_swing_phase_) * 360.0;

        // std::cout<<"******55555*****current_traj.orientation.x:    "<<current_traj.orientation.x<<std::endl;

    }
    // current_traj.orientation.x = middleAngleSmallerArcDeg(angle_dig_lb1_deg, angle_dig_lb2_deg);//回转
    // current_traj.orientation.x = front_lane_deg;//回转
    current_traj.orientation.y = 51.0;//动臂
    // current_traj.orientation.z = arm_target_angle;//斗杆
    current_traj.orientation.z = -110.0;//斗杆
    current_traj.orientation.w = bucket_flat_angle - bucket_offset;//铲斗端平
    current_traj.position.z = 5.0;

}

// ===========//6阶段-抹平矿卡==============
void traj_fusion_node::smoothStatusProcess()
{
    bucket_change_rate = 100.0;
    current_traj.orientation.x = last_traj.orientation.x;
    current_traj.orientation.y = last_traj.orientation.y;
    // current_traj.orientation.z = -35.0;
    current_traj.orientation.z = traj_smooth.orientation.z;
    current_traj.orientation.w = bucket_flat_angle - bucket_offset + 30.0;
    current_traj.position.z = 6.0;

}


// ===========//7阶段-清台==============
void traj_fusion_node::cleaupStatusProcess()
{
    bucket_change_rate = 100.0;
    current_traj.orientation.x = traj_cleanup.orientation.x;
    current_traj.orientation.y = last_traj.orientation.y;//马兰申的矿卡高度-半个车轮
    // current_traj.orientation.z = -35.0;
    current_traj.orientation.z = traj_cleanup.orientation.z;
    current_traj.orientation.w = 0.0;
    current_traj.position.z = 7.0;

}

// ===========//初始化阶段1==============
void traj_fusion_node::initStatusProcess()
{
    start3_angle = current_angle_swing_;
    start1_angle_flag = true;
    bucket_change_rate = 100.0;
    // 阶段2回转方向预计算：
    // 从 current_angle_swing_ 到 RGp_dig_point_angle 先取最小行程方向，
    // 若该方向弧线上会经过卸载点 unload_angle，则反向避开卸载点。
    if (std::isfinite(current_angle_swing_) &&
        std::isfinite(RGp_dig_point_angle) &&
        std::isfinite(unload_angle))
    {
        const double cur_sw = current_angle_swing_;
        const double dig_sw = normalizeAngle(RGp_dig_point_angle);
        const double unload_sw = normalizeAngle(unload_angle);

        bool cw_to_dig = (angleDiff(cur_sw, dig_sw) < 0.0);  // true=顺时针(角度减小)
        if (isAngleBetween(unload_sw, cur_sw, dig_sw, cw_to_dig))
        {
            // cw_to_dig = !cw_to_dig;
            require_takeover = true;
            // require_takeover_flag_.data = require_takeover;
            // if (takeover_pub_count_ < 10)
            // {
            //     pub_takeover.publish(require_takeover_flag_);
            //     ++takeover_pub_count_;
            // }
            // 接管告警属异常路径，保留但限频，避免每周期刷屏阻塞主循环
            ROS_WARN_THROTTLE(1.0,
                "===1===unload_angle is between cur_sw and cw_reset! Please takeover!");
        }

        swing_clockwise_2 = cw_to_dig;
    }

    if (truck_dock_success_flag_) //1->4(from 5)
    {
        const double cur_sw = front_lane_deg;
        const double unload_sw = normalizeAngle(unload_angle);
        const double d = wrapTo180Deg(unload_sw - cur_sw);
        if (std::fabs(d) < 1e-9)
            swing_clockwise_4 = false;
        else
            swing_clockwise_4 = (d < 0.0);

        // 阶段1 周期打印已移除（仅保留阶段3 的周期日志）
        ROS_DEBUG("===1===precalculate cw4=%d", static_cast<int>(swing_clockwise_4));

        
    }

    if (!init_stage1_angles_captured_)
    {
        init_stage1_swing_ = current_angle_swing_;
        init_stage1_boom_ = boom_angle_current;
        init_stage1_arm_ = arm_angle_current;
        init_stage1_bucket_ = bucket_angle_current;
        init_stage1_angles_captured_ = true;
    }

    current_traj.orientation.x = init_stage1_swing_;
    current_traj.orientation.y = init_stage1_boom_;
    current_traj.orientation.z = init_stage1_arm_;
    current_traj.orientation.w = init_stage1_bucket_;
    current_traj.position.z = 1.0;

}

//==================================================================================================//
void traj_fusion_node::bucketTerrainCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    bucket_terrain = msg->point.z;
}

void traj_fusion_node::smolVLAOuputCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    smolVLA_traj.orientation.x = msg->orientation.x;//swing
    smolVLA_traj.orientation.y = msg->orientation.y;//boom
    smolVLA_traj.orientation.z = msg->orientation.z;//arm
    smolVLA_traj.orientation.w = msg->orientation.w;//bucket


}


// void traj_fusion_node::smolVLAStatusCallback(const std_msgs::Int8::ConstPtr& msg)
// {
//     smolVLA_status = msg->data;

//     if (smolVLA_status == 1)
//     {
//         model_traj = smolVLA_traj;
//         model_traj_valid_ = true;
//         clearFaultCode(faultMask, FAULT_0);
//     }
//     else
//     {
//         // if (faultMask == 0 && act_traj_valid_)
//         // {
//         //     model_traj = act_traj;
//         //     model_traj_valid_ = true;
//         // }
//         // else
//         // {
//         //     model_traj_valid_ = false;
//         // }
        
//         model_traj_valid_ = false;
//         addFaultCode(faultMask, FAULT_0);

        
//     }
// }


void traj_fusion_node::smolVLAStatusCallback(const std_msgs::Int8::ConstPtr& msg)
{
    smolVLA_status = msg->data;

    if (smolVLA_status == 1)
    {
        model_traj = smolVLA_traj;
        model_traj_valid_ = true;
        clearFaultCode(faultMask, FAULT_0);
        clearFaultCode(faultMask, FAULT_1);
        clearFaultCode(faultMask, FAULT_2);
        clearFaultCode(faultMask, FAULT_3);
        clearFaultCode(faultMask, FAULT_4);
        clearFaultCode(faultMask, FAULT_5);

    }
    else if(smolVLA_status == 0)//网络异常
    {
        model_traj_valid_ = false;
        clearFaultCode(faultMask, FAULT_1);
        clearFaultCode(faultMask, FAULT_2);
        clearFaultCode(faultMask, FAULT_3);
        clearFaultCode(faultMask, FAULT_4);
        clearFaultCode(faultMask, FAULT_5);
        addFaultCode(faultMask, FAULT_0);

        
    }
    else if(smolVLA_status == 2)//rtk数据收不到
    {
        model_traj_valid_ = false;
        clearFaultCode(faultMask, FAULT_0);
        clearFaultCode(faultMask, FAULT_2);
        clearFaultCode(faultMask, FAULT_3);
        clearFaultCode(faultMask, FAULT_4);
        clearFaultCode(faultMask, FAULT_5);
        addFaultCode(faultMask, FAULT_1);
    }
    else if(smolVLA_status == 3)//gridmap数据收不到
    {
        model_traj_valid_ = false;
        clearFaultCode(faultMask, FAULT_0);
        clearFaultCode(faultMask, FAULT_1);
        clearFaultCode(faultMask, FAULT_3);
        clearFaultCode(faultMask, FAULT_4);
        clearFaultCode(faultMask, FAULT_5);
        addFaultCode(faultMask, FAULT_2);
    }
    else if(smolVLA_status == 4)//range前向灰度图数据收不到
    {
        model_traj_valid_ = false;
        clearFaultCode(faultMask, FAULT_0);
        clearFaultCode(faultMask, FAULT_1);
        clearFaultCode(faultMask, FAULT_2);
        clearFaultCode(faultMask, FAULT_4);
        clearFaultCode(faultMask, FAULT_5);
        addFaultCode(faultMask, FAULT_3);
    }
    else if(smolVLA_status == 5)//camera0数据收不到
    {
        model_traj_valid_ = false;
        clearFaultCode(faultMask, FAULT_0);
        clearFaultCode(faultMask, FAULT_1);
        clearFaultCode(faultMask, FAULT_2);
        clearFaultCode(faultMask, FAULT_3);
        clearFaultCode(faultMask, FAULT_5);
        addFaultCode(faultMask, FAULT_4);
    }
    else if(smolVLA_status == 6)//camera1数据收不到
    {
        model_traj_valid_ = false;
        clearFaultCode(faultMask, FAULT_0);
        clearFaultCode(faultMask, FAULT_1);
        clearFaultCode(faultMask, FAULT_2);
        clearFaultCode(faultMask, FAULT_3);
        clearFaultCode(faultMask, FAULT_4);
        addFaultCode(faultMask, FAULT_5);
    }

}

// 添加故障码
void traj_fusion_node::addFaultCode(uint16_t& faultMask, FaultCode fault) 
{
    faultMask |= static_cast<uint16_t>(fault);
}

// 清除故障码
void traj_fusion_node::clearFaultCode(uint16_t& faultMask, FaultCode fault) 
{
    faultMask &= ~static_cast<uint16_t>(fault);
}

// 组装函数：将高16位和多个故障码组合
uint32_t traj_fusion_node::assembleFaultCode(uint16_t high16, uint16_t faultMask) 
{
    // 高16位左移16位，放到高16位
    uint32_t highPart = static_cast<uint32_t>(high16) << 16;
    // 低16位故障码保持不变
    return highPart | faultMask;
}

// 判断是否有故障发生
bool traj_fusion_node::hasFault(uint16_t faultMask) 
{
    return faultMask != 0;
}

// 对轨迹各关节角进行限幅（按阶段使用对应限制）
void traj_fusion_node::clampTrajAngles(geometry_msgs::Pose& traj, int stage)
{
    auto clamp = [](double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    };
    const StageAngleLimits* lim = nullptr;
    if (stage == 2) lim = &limits_reset_;
    else if (stage == 3) lim = &limits_dig_;
    else if (stage == 4) lim = &limits_unload_;
    if (!lim) return;  // 其他阶段不限制
    // traj.orientation.x = clamp(traj.orientation.x, lim->swing_min, lim->swing_max);
    traj.orientation.y = clamp(traj.orientation.y, lim->boom_min, lim->boom_max);
    traj.orientation.z = clamp(traj.orientation.z, lim->arm_min, lim->arm_max);
    traj.orientation.w = clamp(traj.orientation.w, lim->bucket_min, lim->bucket_max);
}

//实时接收斗杆角度，如果达到目标最大角度则退出ACT
//取act推理的第一个动作角度,仅在挖掘阶段进行融合，并发布最新的融合数据给控制层
// void traj_fusion_node::processACTOutput(const geometry_msgs::Pose::ConstPtr& msg)
void traj_fusion_node::processACTOutput()
{
    // 状态计时：切换时重置，超20s发布故障码
    if (traj_status != last_traj_status_for_timer_)
    {
        state_start_time_ = ros::Time::now();
        prev_traj_status_for_timer_ = last_traj_status_for_timer_;
        last_traj_status_for_timer_ = traj_status;
        if (traj_status == 3)
            dig_stage_start_time_ = ros::Time::now();
        clearFaultCode(faultMask, FAULT_5);  // 状态切换时清除超时故障
        // if (traj_status == 3 || prev_traj_status_for_timer_ == 3)
        //     resetDigOpDetection();
    }
    else
    {
        double elapsed = (ros::Time::now() - state_start_time_).toSec();
        if (elapsed > STATE_TIMEOUT_SEC)
        {
            // addFaultCode(faultMask, FAULT_5);
        }
    }
    // //用朱安源的状态判断是否处于挖掘状态
    // switch(traj_status)
    // {
    //     case 2:
    //         current_traj = traj_reset;
    //         // if((current_traj.orientation.y < last_traj.orientation.y) && (unload_to_reset_count < 5))
    //         // {
    //         //     current_traj.orientation.y = last_traj.orientation.y;
    //         //     unload_to_reset_count += 1;
    //         // }
    //         // std::cout<<"============222222222222"<<std::endl;
    //         break;
    //     case 3:
    //         current_traj = traj_dig;  
    //         // std::cout<<"============3333333333333"<<std::endl;
    //         break;
    //     case 4:
    //         current_traj = traj_unload;
    //         // std::cout<<"============444444444444"<<std::endl;
    //         break;
    //     case 6:
    //         current_traj.orientation.x = last_traj.orientation.x;//回转;
    //         current_traj.orientation.y = last_traj.orientation.y;//动臂(current higher than traj)
    //         current_traj.orientation.z = last_traj.orientation.z;//斗杆
    //         current_traj.orientation.w = init_dig_bucket_3_;//铲斗(current)
    //         current_traj.position = last_traj.position;
    //         // std::cout<<"============3.5========"<<std::endl;
    //         break;
    //     case 7:
    //         current_traj.orientation.x = last_traj.orientation.x;//回转;
    //         current_traj.orientation.y = init_reset_boom_4_;//动臂(current higher than traj)
    //         current_traj.orientation.z = last_traj.orientation.z;//斗杆
    //         current_traj.orientation.w = last_traj.orientation.w;//铲斗
    //         current_traj.position = last_traj.position;
    //         // std::cout<<"============4.5========"<<std::endl;
    //         break;

    //     default: 
    //         current_traj = last_traj;
    // }

    
    // std::cout<<"***********traj_status:    "<<traj_status<<std::endl;
    //用朱安源的状态判断是否处于挖掘状态
    if (traj_status != 1)
    {
        init_stage1_angles_captured_ = false;
        // takeover_pub_count_ = 0;
        require_takeover = false;
    }

    switch(traj_status)
    {
        case 1:
            initStatusProcess();
            // std::cout<<"============111111111111"<<std::endl;
            break;
        case 2:
            resetStatusProcess();
            // std::cout<<"============222222222222"<<std::endl;
            break;
        case 3:
            if (use_dig_status_process_model_)
                digStatusProcess_model();
            else
                digStatusProcess();
            // std::cout<<"============3333333333333"<<std::endl;
            break;
        case 4:
            raiseAndUnloadStausProcess();
            // std::cout<<"============444444444444"<<std::endl;
            break;
        case 5:
            waitStatusProcess();

            // std::cout<<"============555555555555"<<std::endl;
            break;
        case 6:
            smoothStatusProcess();
            // std::cout<<"============666666666666"<<std::endl;
            break;
        case 7:
            cleaupStatusProcess();
            // std::cout<<"============777777777777"<<std::endl;
            break;
        case 8:
            current_traj = last_traj;
            current_traj.position.z = 4.5;
            // std::cout<<"============888888888888"<<std::endl;
            break;
        case 9:
            current_traj = last_traj;
            current_traj.position.z = 3.5;
            // std::cout<<"============999999999999"<<std::endl;
            break;
        case 10:
            current_traj = last_traj;
            current_traj.position.z = 5.5;
            // std::cout<<"============101010101010"<<std::endl;
            break;
        case 11:
            current_traj = last_traj;
            current_traj.position.z = 6.5;
            // std::cout<<"============111111111111"<<std::endl;
            break;
        case 12:
            current_traj = last_traj;
            current_traj.position.z = 7.5;
            // std::cout<<"============121212121212"<<std::endl;
            break;
        case 13:
            current_traj = last_traj;
            current_traj.position.z = 2.5;
            // std::cout<<"============131313131313"<<std::endl;
            break;
        case 14:
            current_traj = last_traj;
            current_traj.position.z = 0.5;
            // std::cout<<"============141414141414"<<std::endl;
            break;
        default:
            current_traj = last_traj;
            current_traj.position.z = 0.5;
            break;
    }

    // 各阶段轨迹关节角限幅（按阶段使用对应限制）
    clampTrajAngles(current_traj, traj_status);

    if (traj_status == 4)
    {
        last_unload_boom_cmd_deg_ = current_traj.orientation.y;
        last_unload_boom_cmd_valid_ = true;
    }

    std_msgs::Bool swing_dir_msg;
    // if (traj_status == 1 || traj_status == 2)
    if (traj_status == 2 || traj_status == 13)
    {
        excavator_rotates_clockwise_flag_.data = swing_clockwise_2;
    }
    // else if (traj_status == 3 || traj_status == 4 || traj_status == 9)
    else if (traj_status == 4 || traj_status == 8)
    {
        excavator_rotates_clockwise_flag_.data = swing_clockwise_4;
    }
    else if (traj_status == 5 || traj_status == 10)
    {
        excavator_rotates_clockwise_flag_.data = swing_clockwise_5;
    }   


    pub_swing_direction.publish(excavator_rotates_clockwise_flag_);

    /*if (traj_status == 4)
    {
        //unload_angle先单调处理
        double unload_angle_mono = unload_angle + static_cast<double>(unload_swing_phase_) * 360.0;
        // std::cout << "===444444444444====unload_swing_phase_:    " << unload_swing_phase_ << std::endl;
        // std::cout << "===444444444444====unload_angle_mono:    " << unload_angle_mono << std::endl;

        // 车位角发布，单调处理
        std_msgs::Float32 truck_swive_angle_pub_msg;

        if (swing_clockwise_4)
            truck_swive_angle = unload_angle_mono + truck_tail_offset_from_unload_deg_;
        else
            truck_swive_angle = unload_angle_mono - truck_tail_offset_from_unload_deg_;

        // std::cout << "===444444444444====truck_swive_angle:    " << truck_swive_angle << std::endl;

        truck_swive_angle_pub_msg.data = truck_swive_angle;

        pub_truck_swive_angle.publish(truck_swive_angle_pub_msg);
    }*/

    //计算铲斗齿尖到矿卡表面的距离,以及齿尖到矿卡地面的距离
    if(std::fabs(truck_rtk_position.z - 0.0) > 1e-9)
    {
        double truck_surface = truck_rtk_position.z - rtk_alt - exc_center_W_W/1000;
        // double bucket_tooth = ef_tip_rotation_z - relative_z;
        double bucket_tooth = ef_tip_rotation_z;
        bucket_to_truck = bucket_tooth - truck_surface;

        // double truck_ground = truck_real_height - TRUCK_TIRE_HALF_HEIGHT - rtk_alt - exc_center_W_W/1000;
        double truck_ground = m_working_height - rtk_alt - exc_center_W_W/1000;
        
        bucket_to_truck_ground = bucket_tooth - truck_ground;
        // std::cout<<"***********Bheight_tip:    "<<ef_tip_rotation_z - relative_z<<std::endl;
        // std::cout << "===###########====bucket_to_truck:    " << bucket_to_truck << std::endl;
        // std::cout << "===###########====truck_to_center:    " << truck_to_center << std::endl;
        // std::cout << "===###########====bucket_to_truck_ground:    " << bucket_to_truck_ground << std::endl;
    }
    else
    {
        bucket_to_truck = 0.0;
    }

    current_traj.position.x = bucket_to_truck;
    current_traj.position.y = bucket_to_truck_ground;

    //铲斗变化率限制
    if (current_traj.orientation.w > last_traj.orientation.w)
    {
        current_traj.orientation.w = std::min(current_traj.orientation.w, last_traj.orientation.w + bucket_change_rate);

    }

    // std::cout << "===#####[status]######====current_traj.position.z:    " << current_traj.position.z << std::endl;

    pub_refDeviceTraj_Dig.publish(current_traj);

    stage2_traj.position.x = unload_angle;
    stage2_traj.position.y = front_lane_deg;
    stage2_traj.position.z = rear_cable_deg;
    pub_2stage_traj.publish(stage2_traj);
    // pub_test_refDeviceTraj_Dig.publish(current_traj);
    last_traj = current_traj;

    if (traj_status != 2)
    {
        last_reset_swing_raw_valid_ = false;
        reset_swing_unwrap_offset_deg_ = 0.0;
        reset_swing_phase_ = 0;
        reset_swing_direction_computed_this_entry_ = false;
        init_reset_phase2_captured_ = false;
        reset_boom_last_percent_ = 0.0;
        takeover_pub_count_cable_reset = 0;
        reset_swing_cable_first_flag_ = false;
    }
    if (traj_status != 4)
    {
        last_unload_swing_raw_valid_ = false;
        unload_swing_phase_ = 0;
        unload_bucket_use_planner_w_ = false;
        unload_planner_bucket_record_valid_ = false;
        last_unload_boom_cmd_valid_ = false;
        current_angle_dif_flag = false;
        unload_angle_dif_flag = false;
        unload_swing_passed_truck_target_latch_ = false;
        unload_truck_cross_prev_valid_ = false;
        takeover_pub_count_cable_unload = 0;
        unload_swing_cable_first_flag_ = false;
        if (traj_status != 2)
            reset_swing_direction_precomputed_in_unload_ = false;
    }

    // FAULT_6 只在此处同步：上面复位/卸载只发 /fault_require_takeover；
    // 离开 status 2/4 后 flag 已清，这里才能把位清掉，避免心跳里一直挂着。
    if (reset_swing_cable_first_flag_ || unload_swing_cable_first_flag_)
        addFaultCode(faultMask, FAULT_6);
    else
        clearFaultCode(faultMask, FAULT_6);

    if (traj_status != 3)
    {
        // stop_swing_main_done_in_dig_stage_ = false;
        digover_boom_applied_ = false;
    }

    // const double bucket_flat_ref_deg = bucket_flat_angle - bucket_offset;
    // const double bucket_exc_open_span_deg = 35.0;
    // double exc_bucket_H_scale = 1.0;
    // if (bucket_exc_open_span_deg > 1e-9) {
    //     exc_bucket_H_scale =
    //         1.0 - (bucket_angle_current - bucket_flat_ref_deg) / bucket_exc_open_span_deg;
    // }
    // exc_bucket_H_scale = std::max(0.0, std::min(1.0, exc_bucket_H_scale));

    // double Bucket_current = ef_tip_rotation_z - relative_z - exc_bucket_H * exc_bucket_H_scale;
    // double truck_center = truck_rtk_position.z - rtk_alt - exc_center_W_W/1000;
    // double Bucket_target = truck_center + unload_height_offset;
    // double Bucket_yueguo = truck_center + truck_edge_offset;

    // std::cout << "===ttttttttt====Bucket_current:    " << Bucket_current << std::endl;
    // std::cout << "===ttttttttt====Bucket_tip:    " << ef_tip_rotation_z - relative_z << std::endl;
    // std::cout << "===ttttttttt====truck_center:    " << truck_center << std::endl;
    // std::cout << "===ttttttttt====Bucket_target:    " << Bucket_target << std::endl;
    // std::cout << "===ttttttttt====Bucket_yueguo:    " << Bucket_yueguo << std::endl;

    // if (Bucket_current >= Bucket_yueguo)
    // {
    //     bucket_over_truck_msg.data = 1;
    // }
    // else
    // {
    //     bucket_over_truck_msg.data = 0;
    // }

    // pub_bucket_over_truck.publish(bucket_over_truck_msg);


    

}