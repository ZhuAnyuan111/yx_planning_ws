#include "traj_fusion_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_fusion_node");
    static constexpr const char* kTrajFusionVersion = "2026.09.08";
    ROS_INFO("\033[1;33mInitialized traj_fusion_node Node, version %s\033[0m", kTrajFusionVersion);
    std::cout << "traj_fusion_node version: [" << kTrajFusionVersion << "]" << std::endl;

    ros::AsyncSpinner spinner(2); // 2个线程够用：1个处理回调，1个余量
    spinner.start();

    traj_fusion_node node;  
    node.run(); 

    return 0;
}
