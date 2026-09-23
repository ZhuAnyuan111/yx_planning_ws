#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景输入发布器：为卸载规划节点提供联调所需的静态输入与激活信号。

持续发布（1Hz，latched）：
    /truck_center_unloadpoint    (geometry_msgs/Quaternion) 卡车近点 x/y/z + w=RTK航向α(deg)
    /truck_center_unloadpoint_2  (geometry_msgs/Quaternion) 卡车远点 x/y（动态卸载点选取用）
    /Sys_SUn_BucketNumber        (std_msgs/Float64)         当前装载斗数

激活信号：
    /Sys_SeqAction               (std_msgs/Float64) 延时 activate_delay 秒后发布 4.0 触发卸载。
    注意：规划节点收到 !=4.0 且正在执行时会 StopExecution，故本节点**只发 4.0**、
    绝不发 0.0；执行期间重复的 4.0 被节点忽略（幂等）。loop_activate=true 时持续
    重发 4.0，可在一次卸载完成(kDone)后自动重触发，形成循环演示。

本节点不修改 dump 节点。所有值均可通过私有参数覆盖。
"""

import rospy
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64


class ScenarioPub:
    def __init__(self):
        self.near = (rospy.get_param('~truck_near_x', 6.0),
                     rospy.get_param('~truck_near_y', 1.5),
                     rospy.get_param('~truck_near_z', 0.0))
        self.far = (rospy.get_param('~truck_far_x', 8.0),
                    rospy.get_param('~truck_far_y', 2.0))
        self.heading = rospy.get_param('~truck_heading_deg', 20.0)
        self.bucket_number = rospy.get_param('~bucket_number', 2.0)

        self.auto_activate = rospy.get_param('~auto_activate', True)
        self.activate_delay = rospy.get_param('~activate_delay', 2.0)
        self.loop_activate = rospy.get_param('~loop_activate', False)
        rate = rospy.get_param('~publish_rate', 1.0)

        self.pub_near = rospy.Publisher('/truck_center_unloadpoint', Quaternion,
                                        queue_size=1, latch=True)
        self.pub_far = rospy.Publisher('/truck_center_unloadpoint_2', Quaternion,
                                       queue_size=1, latch=True)
        self.pub_bucket = rospy.Publisher('/Sys_SUn_BucketNumber', Float64,
                                          queue_size=1, latch=True)
        self.pub_seq = rospy.Publisher('/Sys_SeqAction', Float64,
                                       queue_size=1, latch=True)

        self.start = rospy.Time.now()
        self.activated = False
        rospy.Timer(rospy.Duration(1.0 / rate), self.tick)

        rospy.loginfo("[scenario_pub] near=(%.2f,%.2f,%.2f) far=(%.2f,%.2f) heading=%.1fdeg "
                      "bucket=%d auto_activate=%s delay=%.1fs loop=%s",
                      self.near[0], self.near[1], self.near[2],
                      self.far[0], self.far[1], self.heading, int(self.bucket_number),
                      self.auto_activate, self.activate_delay, self.loop_activate)

    def tick(self, _event):
        # 静态场景输入：持续刷新（latched，晚订阅者也能立即收到）
        qn = Quaternion()
        qn.x, qn.y, qn.z, qn.w = self.near[0], self.near[1], self.near[2], self.heading
        self.pub_near.publish(qn)

        qf = Quaternion()
        qf.x, qf.y, qf.z, qf.w = self.far[0], self.far[1], 0.0, self.heading
        self.pub_far.publish(qf)

        self.pub_bucket.publish(Float64(self.bucket_number))

        # 激活信号：延时后只发 4.0
        if not self.auto_activate:
            return
        elapsed = (rospy.Time.now() - self.start).to_sec()
        if elapsed < self.activate_delay:
            return
        if self.loop_activate or not self.activated:
            self.pub_seq.publish(Float64(4.0))
            self.activated = True


if __name__ == '__main__':
    rospy.init_node('scenario_pub')
    ScenarioPub()
    rospy.spin()
