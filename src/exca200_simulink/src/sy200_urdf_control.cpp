#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <boost/bind/bind.hpp>

using namespace boost::placeholders;
using namespace std;

class SY870Controller
{
public:
    SY870Controller() : nh("~")
    {
        traj_subscriber = nh.subscribe("/Sys_RefDeviceTraj", 1, &SY870Controller::joint_angle_callback, this);
        traj_subscriber2 = nh.subscribe("/Sys_RefDeviceTraj2", 1, &SY870Controller::joint_angle_callback2, this);     
        joint_state_pub_timer = nh.createTimer(ros::Duration(0.1), &SY870Controller::timer_callback, this);
        joint_states_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber traj_subscriber;
    ros::Subscriber traj_subscriber2;
    ros::Timer joint_state_pub_timer;
    ros::Publisher joint_states_publisher;
    sensor_msgs::JointState joint_states;

    double swing_angle = 0.0;
    double boom_angle = 0.0;
    double arm_angle = 0.0;
    double bkt_angle = 0.0;
    double swing_angle2 = 0.0;
    double boom_angle2 = 0.0;
    double arm_angle2 = 0.0;
    double bkt_angle2 = 0.0;

    void joint_angle_callback(const geometry_msgs::Pose::ConstPtr &msg);
    void joint_angle_callback2(const geometry_msgs::Pose::ConstPtr &msg);
    void timer_callback(const ros::TimerEvent &e);
};

void SY870Controller::joint_angle_callback(const geometry_msgs::Pose::ConstPtr &msg)
{
    swing_angle = msg->orientation.x;
    boom_angle = msg->orientation.y;
    arm_angle = msg->orientation.z;
    bkt_angle = msg->orientation.w ;
}
void SY870Controller::joint_angle_callback2(const geometry_msgs::Pose::ConstPtr &msg)
{
    swing_angle2 = msg->orientation.x;
    boom_angle2 = msg->orientation.y;
    arm_angle2 = msg->orientation.z;
    bkt_angle2 = msg->orientation.w;
}

void SY870Controller::timer_callback(const ros::TimerEvent &e)
{
    (void)e;
    joint_states.header.stamp = ros::Time::now();
    joint_states.header.frame_id = "base";
    joint_states.name = {"swing_joint", "boom_joint", "arm_joint", "bucket_joint","swing_joint2", "boom_joint2", "arm_joint2", "bucket_joint2"};
    joint_states.position = {swing_angle, boom_angle, arm_angle, bkt_angle,swing_angle2, boom_angle2, arm_angle2, bkt_angle2};
    joint_states_publisher.publish(joint_states);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sy200_urdf_controller_node");
    ROS_INFO("\033[1;32m----> sy870_urdf_controller_node Started.\033[0m");
    SY870Controller SY870Controller_;
    ros::spin();
    return 0;
}
