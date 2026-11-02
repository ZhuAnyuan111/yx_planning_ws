/*
 * @Description: 矿卡停靠结束后，挖掘机开始挖掘时，生成矿卡车斗边框四个顶点位置，并根据卡车中心位置和长宽，一次性生成未来多个卸载点
 * @FilePath: /kai_docker_ws/SanyDig/SanyDig/src/localization/src/joints_coordinates_publisher_node.cpp
 * 
 * Copyright (c) 2024 by Sany Heavy Machinery - R&D Department of Intellectualization, All Rights Reserved. 
 */
#include "traj_fusion_node.hpp"

#include <cmath>
#include <limits>

#define PI_180 0.0174532925199433 //** PI / 180 **

//CGCS2000坐标系的地球方向参数
double Eth_R0 = 6378137.0;              //长半轴  a
double Eth_f = 1/298.257222101;         //扁率  f
double Eth_Rp = 6356752.31414;       //R0*(1 - f); 短半轴  b
//第一偏心及其平方
double Eth_e12 = 0.00669438002290;    //(2f - f*f);
double Eth_e11 = 0.0818191910428;   //sqrt(2f - f*f);
//第二偏心及其平方
double Eth_e22 =  0.006739496775476; //(2f - f*f)/(1 + f*f - 2*f);
double Eth_e21 =  0.0820944381519; //sqrt((2f - f*f)/(1 + f*f - 2*f));

// 高斯反算（xy_to_wgs84）所需常数
static const double ee_2 = Eth_e12;
static const double ee_4 = ee_2 * ee_2;
static const double ee_6 = ee_2 * ee_4;
static const double ee_8 = ee_4 * ee_4;
static const double ee_10 = ee_8 * ee_2;
static const double AA = 1 + 3/4.0*ee_2 + 45/64.0*ee_4 + 175/256.0*ee_6 + 11025/16384.0*ee_8 + 43659/65536.0*ee_10;
static const double BB = 0 + 3/4.0*ee_2 + 15/16.0*ee_4 + 525/512.0*ee_6 + 2205/2048.0*ee_8 + 72765/65536.0*ee_10;
static const double CC = 0 + 0 + 15/64.0*ee_4 + 105/256.0*ee_6 + 2205/4096.0*ee_8 + 10395/16384.0*ee_10;
static const double DD = 0 + 0 + 0 + 35/512.0*ee_6 + 315/2048.0*ee_8 + 31185/131072.0*ee_10;
static const double EE = 0 + 0 + 0 + 0 + 315/16384.0*ee_8 + 3645/65536.0*ee_10;


double Merdian(double Lat)  //计算子午线弧长
{
    //double PI_180 = 0.0174532925199433; //** PI / 180 **
    //CGCS2000坐标系的地球方向参数
    // double Eth_R0 = 6378137.0;              //长半轴  a
    // double Eth_f = 1/298.257222101;         //扁率  f
    // double Eth_Rp = 6356752.31414;       //R0*(1 - f); 短半轴  b
    // //第一偏心及其平方
    // double Eth_e12 = 0.00669438002290;    //(2f - f*f);
    // double Eth_e11 = 0.0818191910428;   //sqrt(2f - f*f);
    // //第二偏心及其平方
    // double Eth_e22 =  0.006739496775476; //(2f - f*f)/(1 + f*f - 2*f);
    // double Eth_e21 =  0.0820944381519; //sqrt((2f - f*f)/(1 + f*f - 2*f));


    // //WGS84 坐标系的地球方向参数
    // double Eth_R0 = 6378137.0;              //长半轴  a
    // double Eth_f = 1/298.257223563;         //扁率  f
    // double Eth_Rp = 6356752.3142452;       //R0*(1 - f); 短半轴  b
    // //第一偏心及其平方
    // double Eth_e12 = 0.006694379990141;    //(2f - f*f);
    // double Eth_e11 = 0.081819190842622;   //sqrt(2f - f*f);
    // //第二偏心及其平方
    // double Eth_e22 =  0.00673949674227643; //(2f - f*f)/(1 + f*f - 2*f);
    // double Eth_e21 =  0.08209443794969570; //sqrt((2f - f*f)/(1 + f*f - 2*f));

    double S0 = Eth_R0 * (1 - Eth_e12);
    double e2 = Eth_e12;
    double e4 = e2 * e2;
    double e6 = e4 * e2;
    double e8 = e6 * e2;
    double e10 = e8 * e2;
    double e12 = e10 * e2;
    double A1 = 1 + 3 * e2 / 4 + 45 * e4 / 64 + 175 * e6 / 256 + 11025 * e8 / 16384 + 43659 * e10 / 65536;
    double B1 = 3 * e2 / 8 + 15 * e4 / 32 + 525 * e6 / 1024 + 2205 * e8 / 4096 + 72765 * e10 / 131072 ;
    double C1 = 15 * e4 / 256 + 105 * e6 / 1024 + 2205 * e8 / 16384 + 10395 * e10 / 65536 ;
    double D1 = 35 * e6 / 3072 + 105 * e8 / 4096 + 10395 * e10 / 262144;
    double E1 = 315 * e8 / 131072 + 3465 * e10/524288 ;

    double x0 = S0 * (A1 * Lat - B1 * sin(2 * Lat) + C1 * sin(4 * Lat) - D1 * sin(6 * Lat)+ E1 * sin(8 * Lat));
    return x0;
}

//转换为enu坐标
void wgs84_to_xy(double lat, double lon, double alt, double& XX, double& YY, double& ZZ)
{    
    int MerLon = round(lon / 3) * 3;
    double Lat_rad = lat * PI_180; //转弧度
    double Lon_rad = lon * PI_180 - MerLon * PI_180;
    double RN = Eth_R0 / sqrt(1 - Eth_e12 * sin(Lat_rad) * sin(Lat_rad));
    double Lon2 = Lon_rad * Lon_rad;
    double Lon4 = Lon2 * Lon2;
    double tnLat = tan(Lat_rad);
    double tn2Lat = tnLat * tnLat;
    double tn4Lat = tn2Lat * tn2Lat;
    double csLat = cos(Lat_rad); 
    double cs2Lat = csLat * csLat; 
    double cs4Lat = cs2Lat * cs2Lat;
    double Eta2 = Eth_e22 * cs2Lat;   
    double Eta4 = Eta2 * Eta2;
    double NTBLP = RN * tnLat * cs2Lat * Lon2;
    double COE1 = (5 - tn2Lat + 9 * Eta2 + 4 * Eta4) * cs2Lat * Lon2 / 24;
    double COE2 = (61 - 58 * tn2Lat + tn4Lat) * cs4Lat * Lon4 / 720;
    
    XX = Merdian(Lat_rad) + NTBLP * (0.5 + COE1 + COE2);//北
    
    double NBLP = RN * csLat * Lon_rad;
    double COE3 = (1 - tn2Lat + Eta2) * cs2Lat * Lon2/6;
    double COE4 = (5 - 18 * tn2Lat + tn4Lat + 14 * Eta2 - 58 * tn2Lat * Eta2) * cs4Lat * Lon4 / 120;
    
    YY = NBLP*(1 + COE3 + COE4) + 500000;//东
    ZZ = alt;
    
}

void xy_to_wgs84(double XX, double YY, double ZZ, int MerLon, double &lat, double &lon, double &alt)
{
        // YY减去500000才是有效的东向偏移
    double y = YY - 500000;
    double x = XX;

    // 迭代计算底点纬度 Bf - 提高精度到0.00001秒
    constexpr int kMaxIterations = 1000;
    constexpr double kToleranceRad = 0.00001 / 3600.0; // 0.00001秒对应的弧度（ ~ 0.3mm）
    
    double Bf0 = x / (Eth_R0 * AA * (1 - Eth_e12));
    double Bf = Bf0;
    double Bf_delay = std::numeric_limits<double>::infinity();
    
    int iteration = 0;
    while (std::abs(Bf - Bf_delay) >= kToleranceRad && iteration < kMaxIterations)
    {
        Bf_delay = Bf;
        Bf = Bf0 + 1.0 / AA * (BB / 2.0 * std::sin(2 * Bf_delay)
                 - CC / 4.0 * std::sin(4 * Bf_delay)
                 + DD / 6.0 * std::sin(6 * Bf_delay)
                 - EE / 8.0 * std::sin(8 * Bf_delay));
        ++iteration;
    }

    // 计算曲率半径等参数
    double sinBf = std::sin(Bf);
    double cosBf = std::cos(Bf);
    double tanBf = std::tan(Bf);
    double sin2Bf = sinBf * sinBf;
    double cos2Bf = cosBf * cosBf;
    
    double Mf = Eth_R0 * (1 - Eth_e12) / std::pow(std::sqrt(1 - Eth_e12 * sin2Bf), 3);
    double Nf = Eth_R0 / std::sqrt(1 - Eth_e12 * sin2Bf);
    double tf = tanBf;
    double tf2 = tf * tf;
    double tf4 = tf2 * tf2;
    double Eta_f = Eth_e22 * cos2Bf;  // 第二偏心平方 * cos²(Bf)
    double Eta2 = Eta_f * Eta_f;
    
    // 计算纬度（弧度）- 保持现有精度
    double y2 = y * y;
    double y4 = y2 * y2;
    double y6 = y4 * y2;
    double Mf_2 = Mf * Mf;
    double Mf_4 = Mf_2 * Mf_2;
    double Nf_2 = Nf * Nf;
    double Nf_4 = Nf_2 * Nf_2;
    double Nf_6 = Nf_4 * Nf_2;
    
    double Lat_rad = Bf - y2 * tf / (2 * Mf * Nf) * 
                     (1 - y2 / (12 * Nf_2) * (5 + Eta2 + 3 * tf2 - 9 * Eta2 * tf2) +
                      y4 / (360 * Nf_4) * (61 + 90 * tf2 + 45 * tf4));
    
    // 计算经度（弧度）- 增加更高阶项以获得更高精度
    double Lon_rad = MerLon / 180.0 * M_PI + 
                     y / (Nf * cosBf) * 
                     (1 - y2 / (6 * Nf_2) * (1 + Eta2 + 2 * tf2) +
                      y4 / (120 * Nf_4) * (5 - 2 * tf2 + 28 * Eta2 + 3 * tf4 - 8 * Eta2 * tf2 + 24 * Eta2 * Eta2) +
                      y6 / (5040 * Nf_6) * (61 - 58 * tf2 + tf4));
    
    // 转换为角度
    lat = Lat_rad / M_PI * 180.0;
    lon = Lon_rad / M_PI * 180.0 - 0.0000002;
    alt = ZZ;
}

double compute_Angle(double x1, double y1, double x2, double y2) 
{
    // 点积与叉积
    double dot = x1 * x2 + y1 * y2;
    double cross = x1 * y2 - y1 * x2;

    // 使用 atan2 获取带符号的角度（弧度）
    double angleRad = std::atan2(cross, dot);

    // 转换为角度制
    double angleDeg = angleRad * 180.0 / M_PI;

    return angleDeg;
}



double traj_fusion_node::normalizeAngle(double angle)
{
    // 必须用 fmod 而非 while 累加：while 形式在输入为 ±inf 时永不退出，
    // 输入为极大值（如 1e300）时需迭代 ~1e297 次，会把调用它的 10Hz 主循环卡死，
    // 表现为轨迹话题彻底停更。非有限值兜底为 0。
    if (!std::isfinite(angle))
        return 0.0;
    double a = std::fmod(angle, 360.0);
    if (a < 0.0)
        a += 360.0;
    return a;
}

// 沿 clockwise 定义的回转方向，从 from 转到 to 的转角 [0,360)（与 act_traj_fusion 回转约定一致）
// static double swingTravelDegAlongDirection(double from_deg, double to_deg, bool clockwise)
// {
//     auto norm360 = [](double a) {
//         while (a < 0.0)
//             a += 360.0;
//         while (a >= 360.0)
//             a -= 360.0;
//         return a;
//     };
//     const double from = norm360(from_deg);
//     const double to = norm360(to_deg);
//     if (clockwise)
//         return norm360(from - to);
//     return norm360(to - from);
// }




/*计算挖掘机回转中心为原点，矿卡车斗四个顶点在挖掘机车体坐标系下的位置*/
// 坐标变换函数：将点从全局坐标系转换到局部坐标系
// geometry_msgs::Point globalToLocal(const geometry_msgs::Point& global_point, 
//                      const geometry_msgs::Point& local_origin, 
//                      double local_heading) {
Eigen::Vector4d traj_fusion_node::globalToLocal(const Eigen::Vector4d& global_point, 
                     const Eigen::Vector4d& local_origin) 
{
    // 计算相对位置
    double dx = global_point[0] - local_origin[0];
    double dy = global_point[1] - local_origin[1];
    double dz = global_point[2] - local_origin[2];
    

    Eigen::Vector4d localPoint;
    localPoint[0] = dx;
    localPoint[1] = dy;
    localPoint[2] = dz;

    Eigen::Affine3d local_transform = Eigen::Affine3d::Identity();
    // 应用旋转：按照Z-Y-X顺序（yaw-pitch-roll）
    local_transform.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
    localPoint = local_transform * localPoint;


    
    return localPoint;
}

// 车道中心/左/右平行线与回转中心圆的前向交点，局部 (N, W)
// θ=front_lane_deg（与回转一致：0°南）；x=lane_offset_x_m_；y=circle_radius_y_m_
bool traj_fusion_node::computeLaneCircleLocalIntersections(Eigen::Vector2d& p0_nw,
                                                           Eigen::Vector2d& pl_nw,
                                                           Eigen::Vector2d& pr_nw)
{
    const double x = lane_offset_x_m_;
    const double y = circle_radius_y_m_;
    if (!(y > 0.0))
    {
        ROS_WARN_THROTTLE(2.0, "[lane_circle] invalid circle_radius_y_m_=%.3f", y);
        return false;
    }

    const double theta = front_lane_deg * M_PI / 180.0;
    const double c = std::cos(theta);
    const double s_th = std::sin(theta);

    // P0：中心线，s=y
    p0_nw << -y * c, -y * s_th;

    const double x_abs = std::fabs(x);
    if (x_abs > y)
    {
        ROS_WARN_THROTTLE(2.0,
            "[lane_circle] |lane_offset_x_m_=%.3f| > circle_radius_y_m_=%.3f, no side intersections",
            x, y);
        pl_nw.setConstant(std::numeric_limits<double>::quiet_NaN());
        pr_nw.setConstant(std::numeric_limits<double>::quiet_NaN());
        return false;
    }

    const double s = std::sqrt(y * y - x * x);  // 弦半长
    // PL：偏距 +x（左）；PR：偏距 -x（右）
    pl_nw <<  x * s_th - s * c, -x * c - s * s_th;
    pr_nw << -x * s_th - s * c,  x * c - s * s_th;
    return true;
}

// 局部 (N, W) → WGS84：先合成全局投影 XX/YY，再 xy_to_wgs84
// XX = exc_center_enu[1] + N（北），YY = exc_center_enu[0] - W（东，含假东）
bool traj_fusion_node::localNWToLLA(double N, double W, double& lat, double& lon, double& alt)
{
    double center_e = 0.0, center_n = 0.0, center_u = 0.0, lon_cache = 0.0;
    bool valid = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        valid = rtk_lla_valid_;
        center_e = exc_center_enu[0];
        center_n = exc_center_enu[1];
        center_u = exc_center_enu[2];
        lon_cache = rtk_lon;
    }
    if (!valid)
    {
        ROS_WARN_THROTTLE(2.0, "[lane_circle] localNWToLLA: excavator RTK lat/lon not ready");
        return false;
    }

    const double YY = center_e - W;  // East
    const double XX = center_n + N;  // North
    const double ZZ = center_u;
    const int MerLon = static_cast<int>(std::round(lon_cache / 3.0) * 3);
    xy_to_wgs84(XX, YY, ZZ, MerLon, lat, lon, alt);
    return true;
}

// 三交点局部求解 + 转经纬度；Point: x=lon, y=lat, z=alt
// 返回 true 表示至少中心点 P0 有效；侧点无效时 pl/pr 为 NaN
bool traj_fusion_node::computeLaneCircleIntersectionsLLA(geometry_msgs::Point& p0_lla,
                                                         geometry_msgs::Point& pl_lla,
                                                         geometry_msgs::Point& pr_lla)
{
    Eigen::Vector2d p0_nw, pl_nw, pr_nw;
    const bool sides_ok = computeLaneCircleLocalIntersections(p0_nw, pl_nw, pr_nw);
    if (!(circle_radius_y_m_ > 0.0))
        return false;

    double lat = 0.0, lon = 0.0, alt = 0.0;
    if (!localNWToLLA(p0_nw.x(), p0_nw.y(), lat, lon, alt))
        return false;
    p0_lla.x = lon;
    p0_lla.y = lat;
    p0_lla.z = alt;

    if (!sides_ok)
    {
        pl_lla.x = pl_lla.y = pl_lla.z = std::numeric_limits<double>::quiet_NaN();
        pr_lla.x = pr_lla.y = pr_lla.z = std::numeric_limits<double>::quiet_NaN();
        return true;  // 仅中心点有效
    }

    if (!localNWToLLA(pl_nw.x(), pl_nw.y(), lat, lon, alt))
        return false;
    pl_lla.x = lon;
    pl_lla.y = lat;
    pl_lla.z = alt;

    if (!localNWToLLA(pr_nw.x(), pr_nw.y(), lat, lon, alt))
        return false;
    pr_lla.x = lon;
    pr_lla.y = lat;
    pr_lla.z = alt;
    return true;
}

void traj_fusion_node::publishLaneCircleIntersections()
{
    geometry_msgs::Point p0_lla, pl_lla, pr_lla;
    if (!computeLaneCircleIntersectionsLLA(p0_lla, pl_lla, pr_lla))
        return;

    geometry_msgs::PoseArray arr;
    arr.header.stamp = ros::Time::now();
    arr.header.frame_id = "wgs84";

    auto append = [&](const geometry_msgs::Point& lla, double tag) {
        if (!std::isfinite(lla.x) || !std::isfinite(lla.y) || !std::isfinite(lla.z))
            return;
        geometry_msgs::Pose pose;
        pose.position = lla;
        pose.orientation.w = tag;  // 0=中, 1=左, 2=右
        arr.poses.push_back(pose);
    };

    append(p0_lla, 0.0);
    append(pl_lla, 1.0);
    append(pr_lla, 2.0);

    if (!arr.poses.empty())
        pub_lane_circle_intersections_.publish(arr);
}




// //计算矿卡的车体四个顶点位置
// void traj_fusion_node::calculateTruckVertices() 
// {
//     std::lock_guard<std::mutex> lock(mutex_);
//     double truck_xx = 0.0;
//     double truck_yy = 0.0;
//     double truck_zz = 0.0;
//     wgs84_to_xy(truck_rtk_position.x, truck_rtk_position.y, truck_rtk_position.z, truck_xx, truck_yy, truck_zz);
    
    
//     // 应用旋转：按照Z-Y-X顺序（yaw-pitch-roll）
//     truck_transform.rotate(Eigen::AngleAxisd(truck_heading*M_PI/180.0, Eigen::Vector3d::UnitZ()));
//     truck_transform.rotate(Eigen::AngleAxisd(-truck_pitch*M_PI/180.0, Eigen::Vector3d::UnitY()));
//     truck_transform.rotate(Eigen::AngleAxisd(truck_roll*M_PI/180.0, Eigen::Vector3d::UnitX()));
//     truck_transform.translate(Eigen::Vector3d(truck_xx, truck_yy, truck_zz));
    
//     // 定义局部坐标系中的四个顶点（以中心为原点）
//     Eigen::Vector4d local_vertices[4] = {
//         Eigen::Vector4d(delta_dx + truck_length/2, delta_dy + truck_width/2, delta_dz, 1.0),  // 左上
//         Eigen::Vector4d(delta_dx + truck_length/2, delta_dy - truck_width/2, delta_dz, 1.0),  // 右上
//         Eigen::Vector4d(delta_dx - truck_length/2, delta_dy + truck_width/2, delta_dz, 1.0),  // 左下
//         Eigen::Vector4d(delta_dx - truck_length/2, delta_dy - truck_width/2, delta_dz, 1.0)   // 右下
//     };

//     for (int i = 0; i < 4; ++i)
//     {
//         Eigen::Vector4d global_vertex = truck_transform * local_vertices[i];
//         geometry_msgs::Point localPoint = globalToLocal(global_vertex, exc_center_enu, rtk_heading);
//         truck_vertices.points[i] = localPoint;
//     }

//     stop_swing_main();
    
//     calculateUnloadPoints();

//     truck_vertices.header.stamp = ros::Time::now();

//     pub_truck_vertices.publish(truck_vertices);
// }


//计算矿卡的车体四个顶点位置
void traj_fusion_node::calculateTruckVertices() 
{
    std::lock_guard<std::mutex> lock(mutex_);
    // //矿卡中心转化后的全局坐标enu
    double truck_xx = 0.0;
    double truck_yy = 0.0;
    double truck_zz = 0.0;

//经纬度转enu
    //需要判断矿卡信息是否为空
    if(std::fabs(truck_rtk_position.y - 0.0) < 1e-9 || std::fabs(truck_rtk_position.x - 0.0) < 1e-9 || std::fabs(truck_rtk_position.z - 0.0) < 1e-9)
    {
        ROS_WARN_THROTTLE(1.0, "There is no truck position info!");
        truck_geom_cache_valid_ = false;
        return;
    }

    wgs84_to_xy(truck_rtk_position.y, truck_rtk_position.x, truck_rtk_position.z, truck_yy, truck_xx,   truck_zz);
    
    Eigen::Vector4d  truck_center = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);

    // double truck_angle = - truck_heading + 180.0;
    // if(truck_angle <= 0.0)
    //     truck_angle += 360.0;

    double heading_data = - truck_heading + 90.0;
    if(heading_data < 0.0)
        heading_data += 360.0;
    double roll_rad = truck_roll*M_PI / 180.0;
    double pitch_rad = truck_pitch*M_PI / 180.0;
    double heading_rad = heading_data*M_PI / 180.0;
    
    Eigen::Affine3d truck_transform = Eigen::Affine3d::Identity();
    // 应用旋转：按照Z-Y-X顺序（yaw-pitch-roll）
    truck_transform.translate(Eigen::Vector3d(truck_xx, truck_yy,  truck_zz));
    truck_transform.rotate(Eigen::AngleAxisd(heading_rad, Eigen::Vector3d::UnitZ()));
    truck_transform.rotate(Eigen::AngleAxisd(-pitch_rad, Eigen::Vector3d::UnitY()));
    truck_transform.rotate(Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()));

    Eigen::Vector4d truck_transform_global = truck_transform * truck_center;

    center_point = globalToLocal(truck_transform_global, exc_center_enu);
    // Eigen::Vector4d bucket_point = globalToLocal(ef_position_enu_, exc_center_enu, rtk_heading);

    // Eigen::Affine3d local_transform = Eigen::Affine3d::Identity();
    // // 应用旋转：按照Z-Y-X顺序（yaw-pitch-roll）
    // local_transform.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
    // center_point = local_transform * center_point;
    // bucket_point = local_transform * bucket_point;


    //转化为极坐标
    // double unload_distance = std::sqrt(center_point[0]*center_point[0]+center_point[1]*center_point[1]);
    // double angle = compute_Angle(bucket_point[0],bucket_point[1],center_point[0],center_point[1]);

    geometry_msgs::Quaternion truck_unload_point;
    geometry_msgs::Quaternion truck_center_point; 
    // std::cout<<"rtk_alt:    "<<rtk_alt<<std::endl;
    // std::cout<<"exc_center_W_W:    "<<exc_center_W_W<<std::endl;
    // std::cout<<"truck_rtk_position.z:    "<<truck_rtk_position.z<<std::endl;

    truck_to_center = truck_rtk_position.z - rtk_alt - exc_center_W_W/1000;

    //中心卸载点位置
    truck_unload_point.x = center_point[0];
    truck_unload_point.y = center_point[1];
    // truck_unload_point.z = center_point[2] + unload_height_offset;//卸载点为矿卡上1.0米的高度为比较合适的卸载点
    truck_unload_point.z = truck_to_center + unload_height_offset;
    truck_unload_point.w = 1.0;

    // 前推卸载点：车体系沿 +X 前推 unload_forward_offset（含航向/俯仰/横滚），再变到挖机回转中心系
    Eigen::Vector4d unload_body_2(delta_dx + unload_forward_offset, delta_dy, delta_dz, 1.0);
    Eigen::Vector4d unload_enu_2 = truck_transform * unload_body_2;
    Eigen::Vector4d unload_local_2 = globalToLocal(unload_enu_2, exc_center_enu);
    geometry_msgs::Quaternion truck_unload_point_2;
    truck_unload_point_2.x = unload_local_2[0];
    truck_unload_point_2.y = unload_local_2[1];
    truck_unload_point_2.z = truck_to_center + unload_height_offset;
    truck_unload_point_2.w = 1.0;

    //中心点位置
    truck_center_point.x = center_point[0];
    truck_center_point.y = center_point[1];
    // truck_center_point.z = center_point[2];//
    truck_center_point.z = truck_to_center;
    truck_center_point.w = truck_heading;//矿卡航向角
    // 写入 cache（用于持续发布）
    truck_unload_point_cache_ = truck_unload_point;
    truck_unload_point_cache_2 = truck_unload_point_2;
    truck_center_point_cache_ = truck_center_point;

    // 车体水平系：center_point[0]=Δ北，center_point[1]=−Δ东（globalToLocal 内 −90°绕Z 后）
    // 指向矿卡中心水平向量 (Δ东, Δ北) = (-center_point[1], center_point[0])
    // 回转角约定：0°=正南，90°=正东，180°=正北，270°=正西；沿顺时针回转时读数减小（逆时针增大）
    {
        const double cx = center_point[0];
        const double cy = center_point[1];
        unload_angle = normalizeAngle(std::atan2(-cy, -cx) * 180.0 / M_PI);
        unload_angle_valid_ = true;

    }
    // std::cout<<"unload_angle:    "<<unload_angle<<std::endl;
    


    // 定义局部坐标系中的四个顶点（以中心为原点）
    Eigen::Vector4d local_vertices[4] = {
        Eigen::Vector4d(delta_dx + truck_length/2, delta_dy + truck_width/2, delta_dz, 1.0),  // 左上
        Eigen::Vector4d(delta_dx + truck_length/2, delta_dy - truck_width/2, delta_dz, 1.0),  // 右上
        Eigen::Vector4d(delta_dx - truck_length/2, delta_dy + truck_width/2, delta_dz, 1.0),  // 左下
        Eigen::Vector4d(delta_dx - truck_length/2, delta_dy - truck_width/2, delta_dz, 1.0)   // 右下
    };
    for (int i = 0; i < 4; ++i)
    {
        Eigen::Vector4d global_vertex = truck_transform * local_vertices[i];
        // ROS_INFO("Truck Vertex %d: x=%.3f, y=%.3f, z=%.3f", i, global_vertex[0], global_vertex[1], global_vertex[2]);

        Eigen::Vector4d temp_point = globalToLocal(global_vertex, exc_center_enu);


        geometry_msgs::Point localPoint_temp;
        localPoint_temp.x = temp_point[0];
        localPoint_temp.y = temp_point[1];
        localPoint_temp.z = temp_point[2];
        truck_vertices.points[i] = localPoint_temp;

        // std::cout<<std::fixed << std::setprecision(3)<<"truck_vertices:      "<<std::endl<<truck_vertices.points[i]<<std::endl;
    }

    truck_vertices.header.stamp = ros::Time::now();
    // 写入 cache（用于持续发布）
    truck_vertices_cache_ = truck_vertices;
    truck_geom_cache_valid_ = true;

    // 计算完成当次也发布一次
    publishTruckGeometryCache();


    //重新生成中心线卸载点，至少两个
    //==========================
    // int num = 10;
    // double delta_s = truck_length / num - 1;//num个卸载点，有num-1个间隔

    // for (int i = 0; i < num; ++i)
    // {
    //     Eigen::Vector4d temp_unload = Eigen::Vector4d(delta_dx - truck_length/2 + i*delta_s, delta_dy, delta_dz, 1.0); // 从车尾开始
    //     Eigen::Vector4d global_unload = truck_transform * temp_unload;
    //     Eigen::Vector4d temp_exc_local = globalToLocal(global_unload, exc_center_enu);
    //     geometry_msgs::Quaternion unloadPoint;  
    //     unloadPoint.x = temp_exc_local[0];
    //     unloadPoint.y = temp_exc_local[1];
    //     unloadPoint.z = temp_exc_local[2];
    //     unloadPoint.w = 0.0;
    //     local_unLoadPoints.push_back(unloadPoint);
    // }
    //只发布第一个卸载点
    // pub_split_unLoadPoint.publish(local_unLoadPoints[0]);
    //==============================


    // stop_swing_main();
    
    // calculateUnloadPoints();




    ///====================================

    //矿卡的可视化========================================================
    // Eigen::Vector3d p0, p1, p2, p3;
    // p0[0] = truck_vertices.points[0].x; //左上
    // p0[1] = truck_vertices.points[0].y; 
    // p0[2] = truck_vertices.points[0].z; 

    // p1[0] = truck_vertices.points[2].x; //左下
    // p1[1] = truck_vertices.points[2].y;
    // p1[2] = truck_vertices.points[2].z;

    // p2[0] = truck_vertices.points[3].x; //右下
    // p2[1] = truck_vertices.points[3].y;
    // p2[2] = truck_vertices.points[3].z;

    // p3[0] = truck_vertices.points[1].x; //右上
    // p3[1] = truck_vertices.points[1].y;
    // p3[2] = truck_vertices.points[1].z;

    // Eigen::Vector3d center = (p0 + p1 + p2 + p3) / 4.0;
    // double length = (p1 - p0).norm();
    // double width  = (p3 - p0).norm();
    // double height = 0.5;
    // Eigen::Vector3d x_dir = (p1 - p0).normalized();
    // Eigen::Vector3d z_dir(0, 0, 1);
    // Eigen::Vector3d y_dir = z_dir.cross(x_dir).normalized();

    // Eigen::Matrix3d R;
    // R.col(0) = x_dir;
    // R.col(1) = y_dir;
    // R.col(2) = z_dir;

    // Eigen::Quaterniond q(R);
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "map";
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "box";
    // marker.id = 0;
    // marker.type = visualization_msgs::Marker::CUBE;
    // marker.action = visualization_msgs::Marker::ADD;

    // // 位姿
    // marker.pose.position.x = center.x();
    // marker.pose.position.y = center.y();
    // marker.pose.position.z = center.z() + height / 2.0;

    // marker.pose.orientation.x = q.x();
    // marker.pose.orientation.y = q.y();
    // marker.pose.orientation.z = q.z();
    // marker.pose.orientation.w = q.w();

    // // 尺寸
    // marker.scale.x = length;
    // marker.scale.y = width;
    // marker.scale.z = height;
    // ROS_INFO("L=%.3f W=%.3f H=%.3f", length, width, height);

    // // 颜色
    // marker.color.r = 0.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;
    // marker.color.a = 0.4;  // 半透明

    // marker.lifetime = ros::Duration(0);
    // marker_pub.publish(marker);


    // //====================================================


    // //挖机铲斗到回转中心的线段可视化
    // visualization_msgs::Marker marker_1;
    // marker_1.header.frame_id = "map";
    // marker_1.header.stamp = ros::Time::now();
    // marker_1.ns = "vector";
    // marker_1.id = 0;
    // marker_1.type = visualization_msgs::Marker::ARROW;
    // marker_1.action = visualization_msgs::Marker::ADD;

    // // 两个点：起点 + 终点
    // geometry_msgs::Point start, end;
    // start.x = 0.0;
    // start.y = 0.0;
    // start.z = 0.0;

    // end.x = ef_position_enu_[0] - exc_center_enu[0];
    // end.y = ef_position_enu_[1] - exc_center_enu[1];
    // end.z = ef_position_enu_[2] - exc_center_enu[2];

    // Eigen::Vector4d LPoint;
    // LPoint[0] = end.x;
    // LPoint[1] = end.y;
    // LPoint[2] = end.z;


    // Eigen::Affine3d local_transform = Eigen::Affine3d::Identity();
    // // 应用旋转：按照Z-Y-X顺序（yaw-pitch-roll）
    // local_transform.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ()));
    // LPoint = local_transform * LPoint;

    // end.x = LPoint[0];
    // end.y = LPoint[1];
    // end.z = LPoint[2];

    // marker_1.points.push_back(start);
    // marker_1.points.push_back(end);

    // // 箭头尺寸（非常重要）
    // marker_1.scale.x = 0.2;  // shaft 直径
    // marker_1.scale.y = 0.40;  // head 直径
    // marker_1.scale.z = 0.15;  // head 长度

    // // 颜色
    // marker_1.color.r = 1.0;
    // marker_1.color.g = 0.0;
    // marker_1.color.b = 0.0;
    // marker_1.color.a = 1.0;

    // marker_1.lifetime = ros::Duration(0);

    // line_pub.publish(marker_1);
}

void traj_fusion_node::publishTruckGeometryCache()
{
    if (!truck_geom_cache_valid_)
        return;  // 未成功计算或已离站，不发布过期/空 cache
    // 持续发布时更新时间戳，避免下游因 stamp 不更新而忽略
    truck_vertices_cache_.header.stamp = ros::Time::now();
    pub_center_unLoadPoint.publish(truck_unload_point_cache_);
    pub_center_unLoadPoint_2.publish(truck_unload_point_cache_2);
    pub_center_Point.publish(truck_center_point_cache_);
    pub_truck_vertices.publish(truck_vertices_cache_);
}







//辅助函数：计算两个向量的夹角
// 正角度‌：当叉积 > 0 时，表示从第一个向量到第二个向量是‌逆时针‌旋转
// ‌负角度‌：当叉积 < 0 时，表示从第一个向量到第二个向量是‌顺时针‌旋转
// ‌零角度‌：当叉积 = 0 时，表示两向量共线
// 返回的角度范围是-180°到180°



