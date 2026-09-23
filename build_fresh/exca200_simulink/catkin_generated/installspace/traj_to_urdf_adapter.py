#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""deg->rad 适配中继：把卸载规划节点的角度量喂给 URDF 可视化桥接节点。

现有桥接节点 sy200_urdf_control 订阅 /Sys_RefDeviceTraj(2)（geometry_msgs/Pose），
直接把 orientation.x/y/z/w 当作 /joint_states.position（**弧度**）发布给
robot_state_publisher。而 dump_trajectory_node 的输出与反馈都是**角度 deg**，
且话题名不同，因此需要本适配节点做话题中继 + 单位换算：

  robot1（原点主模型，/Sys_RefDeviceTraj）  <- 实际反馈
        /joints_angle(x/y/z=boom/arm/bucket deg) + /heading2swing_topic(swing deg)
  robot2（y=-15 偏移对比机，/Sys_RefDeviceTraj2） <- 规划指令
        /RefDeviceTraj_Dump(orientation.x/y/z/w = swing/boom/arm/bucket deg)

输出 orientation 顺序与桥接节点一致：x=swing, y=boom, z=arm, w=bucket（rad）。
若某关节在 RViz 中转向相反，把对应 sign_* 参数设为 -1 即可。
本节点不修改 dump 节点与桥接节点。
"""

import math

import rospy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float32

D2R = math.pi / 180.0


class TrajToUrdfAdapter:
    def __init__(self):
        self.sign = {
            'swing': rospy.get_param('~sign_swing', 1.0),
            'boom': rospy.get_param('~sign_boom', 1.0),
            'arm': rospy.get_param('~sign_arm', 1.0),
            'bucket': rospy.get_param('~sign_bucket', 1.0),
        }
        # 实际反馈（robot1）
        self.fb = {'swing': 0.0, 'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}
        # 规划指令（robot2）
        self.cmd = {'swing': 0.0, 'boom': 0.0, 'arm': 0.0, 'bucket': 0.0}

        rospy.Subscriber('/joints_angle', Quaternion, self.fb_joints_cb, queue_size=1)
        rospy.Subscriber('/heading2swing_topic', Float32, self.fb_swing_cb, queue_size=1)
        rospy.Subscriber('/RefDeviceTraj_Dump', Pose, self.cmd_cb, queue_size=1)

        self.pub_actual = rospy.Publisher('/Sys_RefDeviceTraj', Pose, queue_size=1)
        self.pub_cmd = rospy.Publisher('/Sys_RefDeviceTraj2', Pose, queue_size=1)

        rate = rospy.get_param('~publish_rate', 20.0)
        rospy.Timer(rospy.Duration(1.0 / rate), self.publish)
        rospy.loginfo("[traj_adapter] ready: robot1=actual feedback, robot2=planned command, "
                      "sign sw/bm/arm/bkt=%.0f/%.0f/%.0f/%.0f",
                      self.sign['swing'], self.sign['boom'],
                      self.sign['arm'], self.sign['bucket'])

    def fb_joints_cb(self, msg):
        # x/y/z = boom/arm/bucket (deg)
        self.fb['boom'] = msg.x
        self.fb['arm'] = msg.y
        self.fb['bucket'] = msg.z

    def fb_swing_cb(self, msg):
        self.fb['swing'] = msg.data

    def cmd_cb(self, msg):
        # orientation.x/y/z/w = swing/boom/arm/bucket (deg)
        self.cmd['swing'] = msg.orientation.x
        self.cmd['boom'] = msg.orientation.y
        self.cmd['arm'] = msg.orientation.z
        self.cmd['bucket'] = msg.orientation.w

    def _to_pose(self, src):
        p = Pose()
        p.orientation.x = self.sign['swing'] * src['swing'] * D2R
        p.orientation.y = self.sign['boom'] * src['boom'] * D2R
        p.orientation.z = self.sign['arm'] * src['arm'] * D2R
        p.orientation.w = self.sign['bucket'] * src['bucket'] * D2R
        return p

    def publish(self, _event):
        self.pub_actual.publish(self._to_pose(self.fb))
        self.pub_cmd.publish(self._to_pose(self.cmd))


if __name__ == '__main__':
    rospy.init_node('traj_to_urdf_adapter')
    TrajToUrdfAdapter()
    rospy.spin()
