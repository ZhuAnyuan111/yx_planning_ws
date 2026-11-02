#!/usr/bin/env python3

import rospy
import threading

from exca200_action_pyfused.action_fused_node import ActionFusedNode


def main():
    rospy.init_node('action_data_fusion_node', anonymous=True)

    if not hasattr(rospy, 'spinOnce'):
        def spinOnce():
            return
        rospy.spinOnce = spinOnce

    rospy.loginfo("Action Data Fusion Node Started, parameters loaded as follows:\033[0m")
    node = ActionFusedNode()


     
    
    # 等待 ROS 连接建立，避免订阅未就绪时访问 None 变量
    rospy.loginfo("Waiting for ROS connections to establish...")
    rospy.sleep(1.0)  # 1秒延迟让订阅有时间建立
    rospy.loginfo("Waiting Done")
    
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # node.publish_10hz_task( )
    pub_safety_threading = threading.Thread(target=node.publish_10hz_task,daemon=True)
    pub_safety_threading.start()


    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        rospy.spinOnce()

        node.stop_swing_main()

        node.run()

        rate.sleep()

if __name__ == "__main__":
    main()