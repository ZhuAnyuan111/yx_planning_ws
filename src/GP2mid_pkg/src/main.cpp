#include <ros/ros.h>
#include <Eigen/Dense>
#include <GP2/GP.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"GP_V1_0");
    GP gp_;
    gp_.run();
    return 0;
}