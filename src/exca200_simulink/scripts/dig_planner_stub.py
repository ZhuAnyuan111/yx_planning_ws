#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""挖掘阶段桩节点（L4 编排层，Phase 2 交付）。

背景
----
`dig_trajectory_planner` 目前只有纯 C++ 库（DP 求解器 / 地形 / FK / 后处理），
无 ROS node 入口，也未定义地形输入契约。为了打通全循环仿真（dig → swing → dump → reset），
本桩节点以**运动学脚本**方式生成一段典型的"下探→收斗→抬臂"挖掘动作，
话题接口与真实 dig planner 保持一致，将来替换为真实节点即可无缝对接。

话题接口（契约见 docs/sim_arch.md §3）
--------------------------------------
订阅：
  /Sys_SDi_FlagDigExcute   (std_msgs/Float64)  data==1.0 触发一轮挖掘
  /joints_angle            (geometry_msgs/Quaternion)  x/y/z = boom/arm/bucket (deg)
  /heading2swing_topic     (std_msgs/Float32)  swing (deg)
  /terrain_profile         (nav_msgs/Path)     可选，桩不消费，仅为将来对接预留

发布：
  /RefDeviceTraj_Dig       (geometry_msgs/Pose)
    orientation.x/y/z/w = swing/boom/arm/bucket (deg)
    position.x = 1.0 执行中 / 0.0 结束
  /Sys_RUn_FlagDigFinish   (std_msgs/Float64, latched)  1.0 完成

动作脚本（默认参数，可通过私有参数覆盖）
---------------------------------------
以触发时刻的实时反馈姿态 KF0 为起点，按标准反铲"远→近单向冲程"四关键帧插值
（齿尖走向经 kinematics FK 验证，swing=180；joint 限位 boom[-25,50]/arm[-150,-30]/bucket[-135,30]）：
  KF0 (t=0)              : 起点——斗杆完全打开(arm=-30 打开极限，齿尖 x≈11.2m)；由上游 gp_dig_joints 复位到此
  KF1 (t=t_down)         : 内收下探——boom 下压 + arm 内收 + bucket 张开，齿尖 (10.55,-2.18)
  KF2 (t=t_down+t_curl)  : 挖到最低——boom 压至 5°、arm 内收到 -80、bucket 张到 +10，齿尖最低 z≈-4.2m
  KF3 (t=...+t_lift)     : 收兜回拉——bucket 由张转关翻卷兜土，齿尖 (5.18,-2.81)
  KF4 (t=...+t_hold)     : 终点——斗杆垂直(θa=boom+arm=-90°)+铲斗端平(θk=boom+arm+bucket=-180°)，
                           齿尖 x≈4.7m 身前收住、离坑抬起；姿态角-180 正好衔接 dump 维持的 bucket_attitude_target

挖掘是一条"起点斗杆完全打开伸手够料、单向往机身内收回拉、终点收到斗杆垂直停在身前"的冲程；
终点不回起点，dig→dump 交接由 dump planner 现读 dig 结束时的实时位姿，与本 dig 轨迹解耦。本桩不动 swing。
"""

import threading

import rospy
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Float64


def _lerp(a, b, t):
    return a + (b - a) * t


class DigPlannerStub:
    """挖掘阶段桩：激活后按四段关键帧脚本发布 /RefDeviceTraj_Dig。"""

    def __init__(self):
        # 触发
        self.activate_topic = rospy.get_param('~activate_topic', '/Sys_SDi_FlagDigExcute')
        self.activate_value = float(rospy.get_param('~activate_value', 1.0))
        # 关键帧相对偏移（deg），基于当前反馈姿态叠加
        self.delta = {
            # 标准反铲"远→近单向冲程"四段（delta 为相对 KF0 起点的关节角叠加量，全程不动 swing）。
            # KF0 起点由上游 gp_dig_joints 复位到"斗杆完全打开"的远位姿(boom20/arm-30/bucket-30，齿尖 x≈11.2)。
            # 齿尖走向经 kinematics FK 验证：x 单调回收 11.2→4.7（远→近、终点不回起点）；
            # boom 平滑下压[20→5]再微抬到 8；arm 从完全打开(-30)连续内收到斗杆垂直(θa=-90，arm-98)；
            # bucket 张开到+10(切深)再收兜到-90(终点端平，姿态角-180)。
            'kf1': {  # 内收下探：boom 下压 + arm 内收 + bucket 张开，往下方土里扎。
                      # 齿尖 (11.22,0.42)→(10.55,-2.18)。
                'boom': float(rospy.get_param('~kf1_boom_delta_deg', -8.0)),
                'arm': float(rospy.get_param('~kf1_arm_delta_deg', -25.0)),
                'bucket': float(rospy.get_param('~kf1_bucket_delta_deg', 25.0)),
            },
            'kf2': {  # 挖到最低：boom 压到 5°、arm 内收到 -80、bucket 张到 +10（切削角）。
                      # 齿尖 (10.55,-2.18)→(8.77,-4.20)，挖掘最低点 z≈-4.2m 出现在此帧。
                'boom': float(rospy.get_param('~kf2_boom_delta_deg', -15.0)),
                'arm': float(rospy.get_param('~kf2_arm_delta_deg', -50.0)),
                'bucket': float(rospy.get_param('~kf2_bucket_delta_deg', 40.0)),
            },
            'kf3': {  # 收兜回拉：arm 继续内收接近垂直、bucket 由张转关翻卷兜土，把料往机身方向刮起。
                      # 齿尖 (8.77,-4.20)→(5.18,-2.81)。
                'boom': float(rospy.get_param('~kf3_boom_delta_deg', -12.0)),
                'arm': float(rospy.get_param('~kf3_arm_delta_deg', -62.0)),
                'bucket': float(rospy.get_param('~kf3_bucket_delta_deg', -40.0)),
            },
            'kf4': {  # 终点——斗杆垂直 + 铲斗端平：arm 收到 θa=boom+arm=-90°、bucket 使姿态角
                      # θk=boom+arm+bucket=-180°(端平)、boom 微抬到 8°(绝对 8/-98/-90)，齿尖离坑
                      # 抬起停在身前 (4.65,-1.79)，不回起点。姿态角-180 正好衔接 dump 维持的
                      # bucket_attitude_target_deg=-180；KF4 无需归位 carry——交接由 dump 现读实时位姿。
                'boom': float(rospy.get_param('~kf4_boom_delta_deg', -12.0)),
                'arm': float(rospy.get_param('~kf4_arm_delta_deg', -68.0)),
                'bucket': float(rospy.get_param('~kf4_bucket_delta_deg', -60.0)),
            },
        }
        # 各段时长（秒）
        self.t_down = float(rospy.get_param('~t_down_sec', 2.0))
        self.t_curl = float(rospy.get_param('~t_curl_sec', 1.5))
        self.t_lift = float(rospy.get_param('~t_lift_sec', 1.5))
        self.t_hold = float(rospy.get_param('~t_hold_sec', 0.5))
        # 动作脚本名义结束时刻（此后进入"等 plant 回到 carry 姿态"的收敛等待）
        self.motion_end = self.t_down + self.t_curl + self.t_lift + self.t_hold
        # 位置感知结束：脚本走完后，等反馈进入 carry 目标邻域再发完成旗标，
        # 避免 plant 因速率限制尚未归位就交接给 dump。
        self.settle_tol_deg = float(rospy.get_param('~settle_tol_deg', 2.0))
        self.settle_max_sec = float(rospy.get_param('~settle_max_sec', 6.0))

        # 关节限位（deg，与 dump/URDF 一致）：dig 全程绝对目标——尤其终点 KF4=dump 的 WP1
        # ——必须落在限位内，否则 dump 会因 WP1 越限、修正量 > waypoint_clamp_abort_deg(5°) 直接 abort。
        # 桩的 delta 是相对实时 KF0 叠加，若 dig 在 reset 未归位时触发（KF0 非名义远位姿 arm=-30），
        # 绝对终点可能越限，故对每个关键帧绝对角做裁剪兜底（名义循环裁剪为 no-op，不改变设计姿态）。
        self.joint_limits = {
            'boom': (float(rospy.get_param('~boom_min_deg', -25.0)),
                     float(rospy.get_param('~boom_max_deg', 50.0))),
            'arm': (float(rospy.get_param('~arm_min_deg', -150.0)),
                    float(rospy.get_param('~arm_max_deg', -30.0))),
            'bucket': (float(rospy.get_param('~bucket_min_deg', -135.0)),
                       float(rospy.get_param('~bucket_max_deg', 30.0))),
        }

        self.rate_hz = float(rospy.get_param('~publish_rate', 20.0))

        # 反馈
        self.fb = {'boom': None, 'arm': None, 'bucket': None, 'swing': None}
        rospy.Subscriber('/joints_angle', Quaternion, self._joints_cb, queue_size=1)
        rospy.Subscriber('/heading2swing_topic', Float32, self._swing_cb, queue_size=1)
        # 预留：地形剖面（桩不消费）
        rospy.Subscriber('/terrain_profile', Path, self._terrain_cb, queue_size=1)

        # 激活
        rospy.Subscriber(self.activate_topic, Float64, self._activate_cb, queue_size=1)

        # 输出
        self.pub_traj = rospy.Publisher('/RefDeviceTraj_Dig', Pose, queue_size=5)
        self.pub_finish = rospy.Publisher('/Sys_RUn_FlagDigFinish', Float64,
                                          queue_size=1, latch=True)
        self.pub_finish.publish(Float64(0.0))

        # 状态机
        self.state = 'idle'  # idle | running | done
        self.start_pose = None
        self.t_start = None
        self._settle_count = 0
        self.lock = threading.Lock()

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)
        rospy.loginfo("[dig_stub] ready; activate=%s=%.1f, "
                      "script t_down=%.1f t_curl=%.1f t_lift=%.1f t_hold=%.1f s",
                      self.activate_topic, self.activate_value, self.t_down, self.t_curl,
                      self.t_lift, self.t_hold)

    # ---------- 回调 ----------
    def _joints_cb(self, msg):
        self.fb['boom'], self.fb['arm'], self.fb['bucket'] = msg.x, msg.y, msg.z

    def _swing_cb(self, msg):
        self.fb['swing'] = msg.data

    def _terrain_cb(self, _msg):
        pass  # 桩不消费地形，仅为将来对接预留订阅

    def _activate_cb(self, msg):
        if abs(msg.data - self.activate_value) > 1e-6:
            return
        with self.lock:
            if self.state == 'running':
                rospy.loginfo_throttle(2.0, "[dig_stub] already running, ignore re-trigger")
                return
            if self.fb['boom'] is None or self.fb['swing'] is None:
                rospy.logwarn_throttle(2.0, "[dig_stub] feedback not ready, ignore trigger")
                return
            self.start_pose = dict(self.fb)
            self.t_start = rospy.Time.now()
            self.state = 'running'
            self._settle_count = 0
            self.pub_finish.publish(Float64(0.0))
            rospy.loginfo("[dig_stub] triggered; start pose sw/bm/arm/bkt=%.1f/%.1f/%.1f/%.1f",
                          self.start_pose['swing'], self.start_pose['boom'],
                          self.start_pose['arm'], self.start_pose['bucket'])

    # ---------- 主循环 ----------
    def _tick(self, _event):
        with self.lock:
            state = self.state
            start_pose = self.start_pose
            t_start = self.t_start
        if state != 'running' or start_pose is None or t_start is None:
            return

        elapsed = (rospy.Time.now() - t_start).to_sec()
        target = self._script(start_pose, elapsed)

        # 位置感知结束判定：动作脚本走完后（target 已回到 carry=KF4），
        # 等 plant 反馈进入 carry 邻域连续 2 帧再发完成旗标；设超时兜底防卡死。
        finished = False
        if elapsed >= self.motion_end:
            err = max(abs(self.fb['boom'] - target['boom']),
                      abs(self.fb['arm'] - target['arm']),
                      abs(self.fb['bucket'] - target['bucket']))
            if err < self.settle_tol_deg:
                self._settle_count += 1
                finished = self._settle_count >= 2
            else:
                self._settle_count = 0
            if elapsed > self.motion_end + self.settle_max_sec:
                finished = True  # 兜底：收敛超时也结束，避免整循环卡死
        running = not finished

        msg = Pose()
        msg.position.x = 1.0 if running else 0.0
        msg.orientation.x = target['swing']
        msg.orientation.y = target['boom']
        msg.orientation.z = target['arm']
        msg.orientation.w = target['bucket']
        self.pub_traj.publish(msg)

        if finished and state == 'running':
            with self.lock:
                self.state = 'done'
            self.pub_finish.publish(Float64(1.0))
            rospy.loginfo("[dig_stub] script finished (%.2fs, settle_tol=%.1f deg)",
                          elapsed, self.settle_tol_deg)

    def _script(self, start, t):
        """按时间 t 计算目标姿态：四段线性插值。"""
        # swing 保持不变（回转由 dump 阶段负责）
        swing = start['swing']
        # 各关键帧绝对姿态
        kf0 = {k: start[k] for k in ('boom', 'arm', 'bucket')}
        kf1 = {k: start[k] + self.delta['kf1'][k] for k in ('boom', 'arm', 'bucket')}
        kf2 = {k: start[k] + self.delta['kf2'][k] for k in ('boom', 'arm', 'bucket')}
        kf3 = {k: start[k] + self.delta['kf3'][k] for k in ('boom', 'arm', 'bucket')}
        kf4 = {k: start[k] + self.delta['kf4'][k] for k in ('boom', 'arm', 'bucket')}

        # 关节限位裁剪：保证每个关键帧绝对角（含终点 KF4=dump 的 WP1）落在限位内。
        # 线性插值两端均已裁剪 ⇒ 中间目标必在限位内（区间凸），末尾再兜底裁剪一次。
        for kf in (kf0, kf1, kf2, kf3, kf4):
            for k in ('boom', 'arm', 'bucket'):
                lo, hi = self.joint_limits[k]
                kf[k] = min(max(kf[k], lo), hi)

        if t < self.t_down:
            a = t / max(self.t_down, 1e-6)
            tgt = {k: _lerp(kf0[k], kf1[k], a) for k in kf0}
        elif t < self.t_down + self.t_curl:
            a = (t - self.t_down) / max(self.t_curl, 1e-6)
            tgt = {k: _lerp(kf1[k], kf2[k], a) for k in kf1}
        elif t < self.t_down + self.t_curl + self.t_lift:
            a = (t - self.t_down - self.t_curl) / max(self.t_lift, 1e-6)
            tgt = {k: _lerp(kf2[k], kf3[k], a) for k in kf2}
        else:
            a = min(1.0, (t - self.t_down - self.t_curl - self.t_lift)
                    / max(self.t_hold, 1e-6))
            tgt = {k: _lerp(kf3[k], kf4[k], a) for k in kf3}
        for k in ('boom', 'arm', 'bucket'):
            lo, hi = self.joint_limits[k]
            tgt[k] = min(max(tgt[k], lo), hi)
        tgt['swing'] = swing
        return tgt


def main():
    rospy.init_node('dig_planner_stub')
    DigPlannerStub()
    rospy.spin()


if __name__ == '__main__':
    main()
