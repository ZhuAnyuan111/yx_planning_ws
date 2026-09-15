#pragma region version
#ifndef GP_H
#define GP_H
#define GP_VERSION "GP-mid 1.0.8"  
//#define DEBUG_MODE
#pragma endregion

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <grid_map_ros/GridMapRosConverter.hpp> 
#include <string>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <limits>
#include <iostream>
#include <utility>
#include <GP2/utils.h>
#include <interfaces/truckVertices.h>
#include <interfaces/rtk_cgi610.h>

class GP{
public:
    GP();
    void run();
    
private:
    enum FaultCode {
        FAULT_0  = 0x0001,
        FAULT_1  = 0x0002,
        FAULT_2  = 0x0004,
        FAULT_3  = 0x0008,
        FAULT_4  = 0x0010,
        FAULT_5  = 0x0020,
        FAULT_6  = 0x0040,
        FAULT_7  = 0x0080,
        FAULT_8  = 0x0100,
        FAULT_9  = 0x0200,
        FAULT_10 = 0x0400,
        FAULT_11 = 0x0800,
        FAULT_12 = 0x1000,
        FAULT_13 = 0x2000,
        FAULT_14 = 0x4000,
        FAULT_15 = 0x8000
    };

    #pragma region ROS sub

    ros::NodeHandle nh;
    ros::Timer ResetMsg_tmer;
    ros::Timer FlagCal1_tmer;
    ros::Rate rt;
    ros::Subscriber mpcSwitch_subscriber;
    ros::Subscriber mpc2Switch_subscriber;
    ros::Subscriber gridMap_subscriber;
    ros::Subscriber jointsAngle_subscriber;
    // ros::Subscriber armPitch_subscriber;
    // ros::Subscriber bktPitch_subscriber;
    ros::Subscriber truckVertices_subscriber;
    ros::Subscriber truckCenter_subscriber;
    ros::Subscriber platformRoll_subscriber;
    ros::Subscriber platformPitch_subscriber;
    ros::Subscriber swing_subscriber;
    ros::Subscriber planFlag_subscriber;
    ros::Subscriber groundDectection_subscriber;
    ros::Subscriber rtkAlt_subscriber;
    ros::Subscriber autoSetDigRegion_subscriber;
    ros::Subscriber truckPose_subscriber;
    ros::Subscriber sysSeq_subscriber;
    ros::Subscriber deviceRef_subscriber;
    ros::Subscriber D0_subscriber;
    ros::Subscriber E0_subscriber;
    ros::Subscriber F0_subscriber;
    ros::Subscriber D1_subscriber;
    ros::Subscriber E1_subscriber;
    ros::Subscriber F1_subscriber;
    ros::Subscriber O4_subscriber;
    ros::Subscriber G4_subscriber;
    ros::Subscriber F6_subscriber;
    ros::Subscriber F7_subscriber;
    #pragma endregion

    #pragma region grid map
    grid_map::GridMap map;
    grid_map::GridMapRosConverter gridconverter;
    grid_map::GridMap::Matrix elevmat;
    grid_map::GridMap::Matrix labelmat;
    double ori_coord_x = 0;
    double ori_coord_y = 0;
    double lengthX = 30;
    double lengthY = 30;
    Eigen::MatrixXd xgridmat;
    Eigen::MatrixXd ygridmat;
    Eigen::VectorXd x_range=Eigen::VectorXd::LinSpaced(300,-14.95,14.95);         // x range of gridmap
    Eigen::VectorXd y_range=Eigen::VectorXd::LinSpaced(300,-14.95,14.95);       // y range of gridmap
    double resolution = 0.1;                // gridmap resolution
    #pragma endregion

    #pragma region excavator params
    double l_CF = 6.987;              // boom length*
    double l_FQ = 2.797;        // arm length*
    double l_QV = 2.359;         //  bkt length*
    double l_MN = 0.901, l_NQ = 0.643, l_KQ = 0.680, l_KM = 0.800;       // *
    double phi_FQN = deg2rad(10.46), phi_KQV = deg2rad(104.858), phi_BCF = deg2rad(23.193);  // *
    double bkt_width=1.5;         // bkt width*
    double base2ground = 2.699;  // base origin to ground distance*
    std::vector<double> IMU_bias = {deg2rad(-0.38), deg2rad(1.04), deg2rad(6.93)};    // IMU bias
    std::vector<double> boom_angle_limit={deg2rad(-27.18),deg2rad(58.87)};        // boom joint angle limits*
    std::vector<double> arm_angle_limit={deg2rad(-153.21),deg2rad(-29.92)};        // arm joint angle limits*
    std::vector<double> bkt_angle_limit={deg2rad(-135.7),deg2rad(31.75)};          // bkt joint angle limits*
    std::vector<double> boom_pos={0.3205,0.0342,0.0};      // boom bottom position relative to swing center*
    double boom2ground = base2ground+boom_pos[2];    // boom bottom to ground distance*
    #pragma endregion

    #pragma region **GUI params**
    double track_prot=0;   // D0: Y
    int N_fans=30; // D0: Z 
    double ground_crash_prot=0;  // E0: X
    double max_depth=-1;        // E0: Y
    double ground_setting=0; // E0: Z
    double angle_ub_setting=deg2rad(230);   // F0: X
    double angle_lb_setting=deg2rad(130);   //F0: Y
    double angle_dig_ub1_setting=deg2rad(165);  //F6：X 
    double angle_dig_lb1_setting=deg2rad(135);  //F6：Y
    double angle_dig_ub2_setting=deg2rad(225);  //F7：X 
    double angle_dig_lb2_setting=deg2rad(195);  //F7：Y
    double angle_offset = deg2rad(90);     // F0: Z default angle of x-axis
    double dig_region_margin = deg2rad(10); // expand dig regions by +/- margin when constraining/visualizing
    double danger_zone_margin = deg2rad(10); // allow swing random offset to go into danger zone by +/- margin
    double truck_angle_front=0;
    double truck_angle_back=0;
    double truck_angle_mid=0;

    double last_danger_min = std::numeric_limits<double>::quiet_NaN();
    double last_danger_max = std::numeric_limits<double>::quiet_NaN();
    bool last_danger_valid = false;

    double target_ub=3.5;         // D1:X
    double target_lb=3;         // D1:Y
    double edge_extend=2;   // D1: Z
    double extend_angle=deg2rad(10);  // E1:X
    double truck_angle_prot=deg2rad(10);
    double expnt=4;   // E1: Y
    double diggable_threshold=0.15;          // E1: Z volume decrease criteria for empty bucket 
    double digx_min=6;          // F1: X min x distance
    double turn_point = 8;          // F1:Z turning point of GP predicted trajectory
    #pragma endregion

    #pragma region algorithm params
    const int array_len = 804;              // default output array length
    const double activate_agl=deg2rad(10);  // boom angle exceeds it, activate calculation
    const double max_scan_time=3;           // max time for terrain scanning
    const double trackdist_def = 3.0;          // default track dist
    const double pitch_error=deg2rad(-0.76);   // IMU_pitch error*
    const double reset_delay=1;     // seconds for resetting msg delay
    const double r_grain_size=0.05;     // interval length of terrain msg
    std::vector<double> r_limit={2,12};   // terrain range
    int r_len=(r_limit[1]-r_limit[0])/r_grain_size+1;     // terrain array length
    Eigen::VectorXd r_range=Eigen::VectorXd::LinSpaced(r_len,r_limit[0],r_limit[1]);      // terrain range vector
    double max_pt_height = -0.3;         // max height from terrain of gp point
    double min_pt_height = -0.5;      // min height from terrain of gp point
    const double height_threshold = 1;      // abnormal terrain height 
    
    #pragma endregion
   
    #pragma region detection params
    double ground_def= 4.7;        // default ground depth*
    std::vector<double> ground_coef={0,0,1,ground_def};           // default ground plane coef

    bool use_rtk_ground_def = false;
    double rtk_to_swing_center_alt_offset = 0.0;
    double ground_protect_alt = 0.0;
    double truck_pose_ground_protect_alt_offset = 0.0;
    bool ground_protect_alt_overridden_by_truck_pose = false;
    double rtk_ground_def_ref = 10.0;
    double rtk_ground_def_band = 0.0;
    double rtk_alt = std::numeric_limits<double>::quiet_NaN();
    std::string auto_dig_region_trigger_topic = "/unman_params_set";
    std::string auto_dig_region_yaml_path;
    double auto_dig_region_half_width_deg = 90.0;
    double auto_dig_region_near_offset_deg = 10.0;
    double auto_dig_region_far_offset_deg = 35.0;
    double swing_center_alt = std::numeric_limits<double>::quiet_NaN();
    double swing_center_to_ground_rel_height = std::numeric_limits<double>::quiet_NaN();
    int auto_dig_region_trigger_state = 0;
    #pragma endregion

    #pragma region IMU data
    double boom_pitch;
    double arm_pitch;
    double bkt_pitch;
    double platform_pitch;
    double platform_roll;
    double real_swing;
    
    #pragma endregion

    #pragma region sys var
    bool mpc_switch_interp=false;  // switch for interp2-related logic (default false)
    bool mpc_switch_dig=true;      // switch for 1->2->0 auto-dig gating (default true)
    float plan_flag=0;          // calculation activation flag
    float sys_seq=0;            // system seq
    std::vector<float> swing_joint_ref={0,0,0,0};     // reference theta joints and swing
    int signal=0;          // activation signal of calculation
    int signal_=0;          // activation signal of t-1
    bool flag_calfin=false;   // whether calculation has finished
    bool flag_cal1=false;   // whether to do calculation 
    bool flag_cal2=false;   // whether to do calculation
    int dig_region_ref_angles_publish_count = 0;
    bool mpc2_switch = false;  // alternative switch to compute unload point
    bool mpc2_switch_prev = false;  // for rising-edge detection
    bool unload_solution_valid = false;  // whether we have a computed unload solution
    bool unload_compute_pending = false;
    double unload_angle_deg = 0.0;  // unload angle in degrees
    double unload_distance = 8.0;   // default unload distance (can be overridden by ROS param)
    double unload_height = 3.0;
    #pragma endregion

    #pragma region intermediate var
    double area_ub, area_lb;        // chosen digging angle range
    int calculate_counter_ = 0;     // counter of calculate()
    static constexpr int WINDOW_SIZE = 10;
    std::vector<bool> flag1_vec;    // direction feasible indicator
    std::vector<double> theta_vec;  // candidate digging direction angles
    std::vector<double> thetaTruck_vec;
    std::vector<double> thetaTruckRaw_vec;
    std::vector<std::pair<double, double>> safe_sector_angle_ranges_deg;
    std::vector<double> edge_dist_vec;      // edge distance of each direction
    std::vector<double> track_dist_vec;     // track distance of each direction
    Eigen::MatrixXd terrain_mat;        // each col is the terrain height of a direction
    Eigen::MatrixXd ground_mat;         // each col is the ground height of a direction
    double vol_modifier = 0;            // adjust digging volume after an empty bucket
    std::vector<double> directionvalue_vec;   // value of a direction
    std::vector<double> directionvalue_memory={std::numeric_limits<double>::quiet_NaN()};    // the previous value of a direction
    std::vector<double> pointvalue_vec;     // value of a point
    double dig_edgedist;        // edge distance of chosen direction
    float dig_height;           // chosen point height from terrain
    float dig_height_terrain;   // chosen point road by terrain
    double dig_start_id;
    double dig_fan_start_id;
    Eigen::Vector3d realbkt_pos;
    #pragma endregion

    #pragma region output var
    float gridheight_errorflag;         // grid height is valid flag
    float gridlabel_errorflag;          // grid label is valid flag
    float gridexist_errorflag;          // grid exist flag
    float dig_flag;             // output finish flag
    float emtpy_flag;            // empty digging flag
    float dig_swing;          // chosen swing angle
    float working_angle;        // working device angle w.r.t. ground
    float dig_bktangle;            // dig point bkt angle
    float dig_start;            // dig point distance
    float dig_depth;            // dig point depth in terrain
    Eigen::VectorXd dig_terrainZ;  // terrain vector of chosen direction
    Eigen::VectorXd dig_groundZ;    // ground vector of chosen direction
    float vol_pred;             // predicted digging volume
    float dig_trackdist;        // track distance of chosen direction
    std::vector<double> dig_joints;   // joint angles in start point
    std::vector<double> GUIparam_vec;  // record GUI params in experiments
    Eigen::MatrixX3d traj_pred;      // record predicted trajectory
    float time_elapsed=0;          // calculating time
    #pragma endregion

    uint16_t heart_value_ = 0;
    uint16_t fault_mask_ = 0;

    #pragma region ROS pub
    ros::Publisher GPvisual_publisher;
    ros::Publisher RefBkt_publisher;
    ros::Publisher RealBkt_publisher;
    ros::Publisher flagGPfinish_publisher;
    ros::Publisher GPempty_publisher;
    ros::Publisher SwWkAgl_publisher;
    ros::Publisher DigStartDepth_publisher;
    ros::Publisher terrainZ_publisher;
    ros::Publisher groundZ_publisher;
    ros::Publisher AreaUbLb_publisher;
    ros::Publisher VolTrack_publisher;
    ros::Publisher GUIparam_publisher;
    ros::Publisher BktAgl_publisher;
    ros::Publisher ErrorFlag_publisher;
    ros::Publisher HeartFault_publisher;
    ros::Publisher autoSetDigRegionResult_publisher;
    ros::Publisher digRegionReferenceAngles_publisher;
    ros::Publisher Unload_publisher;
    ros::Publisher DigJoints_publisher;
    #pragma endregion

    #pragma region ROSpub msgs
    visualization_msgs::Marker GPmarker_msg;
    visualization_msgs::Marker GPtruckRange_msg;
    visualization_msgs::Marker GPtruckRawRange_msg;
    visualization_msgs::Marker GPdevice_msg;
    visualization_msgs::Marker GPrange_msg;
    visualization_msgs::Marker GPdigRange1_msg;
    visualization_msgs::Marker GPdigRange2_msg;
    visualization_msgs::Marker GPdigRange1Ext_msg;
    visualization_msgs::Marker GPdigRange2Ext_msg;
    visualization_msgs::Marker GPtraj_msg;
    visualization_msgs::Marker GPterrain_msg;
    visualization_msgs::Marker RefBkt_msg;
    visualization_msgs::Marker RealBkt_msg;
    std_msgs::Float64 flagGPfinish_msg;
    geometry_msgs::Point GPemtpy_msg;
    geometry_msgs::Point SwWkAgl_msg;
    geometry_msgs::Point DigStartDepth_msg;
    geometry_msgs::Point BktAgl_msg;
    geometry_msgs::Point dig_region_ref_angles_msg;
    geometry_msgs::Point Unload_msg;
    std_msgs::Float64MultiArray terrainZ_msg;
    std_msgs::Float64MultiArray groundZ_msg;
    geometry_msgs::Point AreaUbLb_msg;
    geometry_msgs::Point VolTrack_msg;
    std_msgs::Float64MultiArray GUIparam_msg;
    geometry_msgs::Point ErrorFlag_msg;
    std_msgs::UInt32 HeartFault_msg;
    geometry_msgs::Quaternion DigJoints_msg;  // x=swing, y=boom, z=arm, w=bucket (deg)
    #pragma endregion
    
    #pragma region main func
    void anormaly_check();       // check if the system is in normal state

    void addFaultCode(FaultCode fault);
    void clearFaultCode(FaultCode fault);
    uint32_t assembleHeartAndFault(uint16_t heart_value, uint16_t fault_mask) const;
    void updateAndPublishHeartAndFault();

    void map_construct();       // construct the map

    void area_split();   // % determine digging area
                        // angle_lb, angle_ub: maximum digging area angle
                        // area_ub, area_lb: chosen digging area angle

    void angle_split();   // split angles within areas into N_fans parts 

    void edge_calculation() ;    // calculate the edge distance

    void trackdist_calculation();    // calculate the track distance

    void terrain_section();     // create 2d terrain based on gridmap

    void ground_calculation();    // calculate the ground points for each fan

    void terrainvalue_calculation();        // calculate value of each direction

    void vol_modify();              // modify digging volume based on empty bucket

    void direction_choose();            // choose a direction

    void point_choose();            // choose dig point

    void onestep_choose();           // choose direction and point simultaneously

    bool validateOnestepChooseInputs(std::string& err) const;

    bool isInDangerZone(double angle);

    void updateThetaTruckVec(double danger_min, double danger_max, bool danger_is_continuous, std::vector<double>& theta_truck_vec);

    std::vector<std::vector<int>> computeContiguousSafeBlocksByAngle(const std::vector<int>& safe_fan_ids,
                                                                     double area_lb,
                                                                     double area_ub) const;

    void updateAndLogSafeSectorAngleRangesDeg(const std::vector<std::vector<int>>& blocks,
                                              double half_step_rad);

    void calculate();           // finish one entire calculation
    void computeUnloadPlan();   // compute unload angle and distance once
    
    #pragma endregion

    #pragma region callback func
    void mpcSwitch_callback(const std_msgs::Bool::ConstPtr& msg);

    void mpc2Switch_callback(const std_msgs::Bool::ConstPtr& msg);

    void sysSeq_callback(const std_msgs::Float32::ConstPtr& msg);

    void deviceRef_callback(const geometry_msgs::Pose::ConstPtr& msg);

    void groundDectection_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void gridMap_callback(const grid_map_msgs::GridMap::ConstPtr& gridmsg);

    void jointsAngle_callback(const geometry_msgs::Quaternion::ConstPtr& msg);

    // void armPitch_callback(const geometry_msgs::Quaternion::ConstPtr& msg);

    // void bktPitch_callback(const geometry_msgs::Quaternion::ConstPtr& msg);

void truckVertices_callback(const interfaces::truckVertices::ConstPtr& msg);

void truckCenter_callback(const geometry_msgs::Quaternion::ConstPtr& msg);

void platformPitch_callback(const geometry_msgs::Quaternion::ConstPtr& msg);

void swing_callback(const geometry_msgs::Point::ConstPtr& msg);

void truckPose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void rtkAlt_callback(const interfaces::rtk_cgi610::ConstPtr& msg);

void autoSetDigRegion_callback(const geometry_msgs::Pose::ConstPtr& msg);

void planFlag_callback(const std_msgs::Float32::ConstPtr& msg);

void D0_callback(const geometry_msgs::Point::ConstPtr& msg);

void E0_callback(const geometry_msgs::Point::ConstPtr& msg);

    void F0_callback(const geometry_msgs::Point::ConstPtr& msg);

    void D1_callback(const geometry_msgs::Point::ConstPtr& msg);

    void E1_callback(const geometry_msgs::Point::ConstPtr& msg);

    void F1_callback(const geometry_msgs::Point::ConstPtr& msg);
    
    void O4_callback(const geometry_msgs::Point::ConstPtr& msg);

    void G4_callback(const geometry_msgs::Point::ConstPtr& msg);

    void F6_callback(const geometry_msgs::Point::ConstPtr& msg);

    void F7_callback(const geometry_msgs::Point::ConstPtr& msg);

    void ResetMsg_callback(const ros::TimerEvent&);

    void FlagCal1_callback(const ros::TimerEvent&);
    #pragma endregion

    #pragma region publish func
    void initMsg();         // initialize msgs

    void msg_write();       // update msg after calculation

    void msg_reset();       // reset msg to default

    void msg_publish();     // publish msg

    void msg_print();       // print info in terminal

    void msg_update();      // update realtime msg

    #pragma endregion

    #pragma region auxiliary func
    template <typename T>
    void map_rotate(Eigen::MatrixXd& mat, double angle, const std::vector<T>& center)    // input: mat, center point ,angle; output: rotated mat
    {
        int size = mat.rows();
        Eigen::VectorXd vec1 = Eigen::VectorXd::Ones(size);
        // translate to center of rotation
        Eigen::VectorXd x0 = mat.col(0) - center[0]*vec1;
        Eigen::VectorXd y0 = mat.col(1) - center[1]*vec1;
        // rotate
        mat.col(0) = cos(angle)*x0 - sin(angle)*y0 + center[0]*vec1;
        mat.col(1) = sin(angle)*x0 + cos(angle)*y0 + center[1]*vec1;
    }

    Eigen::ArrayXXd interp2(const Eigen::MatrixXd& z_mat, const Eigen::MatrixXd& x_mat, const Eigen::MatrixXd& y_mat, const Eigen::MatrixXd& xq_mat, const Eigen::MatrixXd& yq_mat);

    std::pair<bool, std::vector<double>> inverse_kinematic(double x_start,double z_start, double bucket_angle) ;           // inverse kinematics

    void fillnan(Eigen::MatrixXd& mat,double expnt);   // fill missing values rowwise in gridmap with exp func
    
    void gridmap_correct();         // correct grid map height from IMU5 bias

    bool arm_collision(const std::vector<double>& joint_angles, const Eigen::VectorXd& terrainZ);   // whether arm collides with earth

    Eigen::Matrix3d getDeviceRealPos();           // get joint position from joint angles

    Eigen::Vector3d getDeviveRefPos();          // get joint position from ref joint angles

    double area_covered(const double& x_start, const double& x_end, const double& swing_angle);        // get covered area by digging

    void forward_kinematic();           // get realtime bkt teeth pos, based on IMU data
    #pragma endregion
};

#endif