#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""闭环执行模拟器（excavator executor simulator）。

订阅卸载规划节点的执行指令 /RefDeviceTraj_Dump（geometry_msgs/Pose，
orientation.x/y/z/w = swing/boom/arm/bucket，单位 deg），用**速率限制**把指令
转成"实际关节角"，回发反馈话题：
    /joints_angle          (geometry_msgs/Quaternion, x/y/z = boom/arm/bucket, deg)
    /heading2swing_topic   (std_msgs/Float32,          data = swing, deg)

从而让 dump_trajectory_node 的闭环状态机（段推进 / Phase1->Phase2 切换 /
高度门控 / 段完成判定）能够真正跑起来。速率上限可调，用于模拟下游阀口饱和 /
限速器，压测"双阶跃 + PCHIP"策略。

本节点不修改 dump 节点，仅作为其下游执行环节的替身。
"""

import math

import rospy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float32


def wrap180(deg):
    """把角度差归一化到 [-180, 180)。"""
    return (deg + 180.0) % 360.0 - 180.0


def wrap360(deg):
    """把角度归一化到 [0, 360)。"""
    return deg % 360.0


class ExecSim:
    def __init__(self):
        # 各关节最大速率 (deg/s)，调小可模拟阀口饱和 / 下游限速
        self.rate = {
            'swing': rospy.get_param('~rate_swing_dps', 30.0),
            'boom': rospy.get_param('~rate_boom_dps', 15.0),
            'arm': rospy.get_param('~rate_arm_dps', 20.0),
            'bucket': rospy.get_param('~rate_bucket_dps', 40.0),
        }
        # 初始（挖掘终止）位姿，作为轨迹起点反馈；deg
        self.cur = {
            'swing': rospy.get_param('~init_swing_deg', 180.0),
            'boom': rospy.get_param('~init_boom_deg', 5.0),
            'arm': rospy.get_param('~init_arm_deg', -110.0),
            'bucket': rospy.get_param('~init_bucket_deg', -110.0),
        }
        # swing 内部保持连续（不包裹），仅在发布时包裹到 [0,360)
        self.swing_cont = self.cur['swing']
        self.cmd = dict(self.cur)

        self.rate_hz = rospy.get_param('~publish_rate', 50.0)
        self.last = rospy.Time.now()

        rospy.Subscriber('/RefDeviceTraj_Dump', Pose, self.cmd_cb, queue_size=1)
        self.pub_joints = rospy.Publisher('/joints_angle', Quaternion, queue_size=1)
        self.pub_swing = rospy.Publisher('/heading2swing_topic', Float32, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.step)

        rospy.loginfo("[exec_sim] ready: init swing=%.1f boom=%.1f arm=%.1f bkt=%.1f deg, "
                      "rate sw/bm/arm/bkt=%.1f/%.1f/%.1f/%.1f dps",
                      self.cur['swing'], self.cur['boom'], self.cur['arm'], self.cur['bucket'],
                      self.rate['swing'], self.rate['boom'], self.rate['arm'], self.rate['bucket'])

    def cmd_cb(self, msg):
        # orientation.x/y/z/w = swing/boom/arm/bucket (deg)
        self.cmd['swing'] = msg.orientation.x
        self.cmd['boom'] = msg.orientation.y
        self.cmd['arm'] = msg.orientation.z
        self.cmd['bucket'] = msg.orientation.w

    def step(self, _event):
        now = rospy.Time.now()
        dt = (now - self.last).to_sec()
        self.last = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / self.rate_hz  # 首帧或时钟跳变兜底

        # boom/arm/bucket：朝指令做速率限制逼近
        for j in ('boom', 'arm', 'bucket'):
            diff = self.cmd[j] - self.cur[j]
            max_step = self.rate[j] * dt
            self.cur[j] += max(-max_step, min(max_step, diff))

        # swing：按最短角差逼近（处理 0/360 跨界），内部连续、发布包裹
        sw_err = wrap180(self.cmd['swing'] - wrap360(self.swing_cont))
        max_step = self.rate['swing'] * dt
        self.swing_cont += max(-max_step, min(max_step, sw_err))
        self.cur['swing'] = wrap360(self.swing_cont)

        # 发布反馈
        q = Quaternion()
        q.x = self.cur['boom']
        q.y = self.cur['arm']
        q.z = self.cur['bucket']
        q.w = 0.0
        self.pub_joints.publish(q)
        self.pub_swing.publish(Float32(self.cur['swing']))


if __name__ == '__main__':
    rospy.init_node('exec_sim')
    ExecSim()
    rospy.spin()
