#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通用阶段触发助手（L4 编排层，由 dump_trigger.py 泛化）。

一次 roslaunch 后可反复跑指定阶段的"复位→触发→等完成"循环，支持 dump / reset / dig。

背景
----
各 planner 在阶段结束后若立即被重触发，会因起点姿态不合适而被 waypoint 限位拦停 abort。
本助手在每次触发前，先把执行端（plant）驱动回"挖掘起始位姿"（或其他合适的起点），
再发激活值，从而稳定重复同一条演示。

用法
----
1) 单步触发（推荐手动/联调）：
     rostopic pub -1 /phase_trigger/go std_msgs/Empty
   或（向后兼容）：
     rostopic pub -1 /dump_trigger/go std_msgs/Empty
2) 自动循环：设 ~auto_loop_sec > 0，则每隔若干秒自动跑一轮。

阶段配置（~phase 参数指定，可选 dump / reset / dig）
-------------------------------------------------
每个阶段的激活话题、激活值、命令话题、完成检测方式内置在 PHASES 表中，
可通过私有参数 ~activate_topic / ~activate_value / ~cmd_topic 覆盖。

复位机制
--------
本节点通过 /RefDeviceTraj_Dump（plant 中优先级最低的兼容槽）以 position.x=1
驱动 plant 回到 ~dig_* 起始位姿；期间要求目标 planner 处于 idle（不发布其命令话题），
否则目标 planner 的更高优先级命令会覆盖本节点的复位指令。

完成检测
--------
1) 优先：订阅阶段专属 finish flag 话题（如 dump 的 /Sys_RUn_FlagUnloadExcuteFinish）
2) 回退：监听 cmd_topic 的 position.x 从 1 下降到 0（reset/dig 无独立 finish 话题）
两者任一触发即视为完成；~wait_timeout_sec 超时后强制返回。

不修改任何 planner 与 plant。契约见 docs/sim_arch.md。
"""

import threading

import rospy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Empty, Float32, Float64


# 各阶段内置配置
PHASES = {
    'dump': {
        'activate_topic': '/Sys_SeqAction',
        'activate_value': 4.0,
        'cmd_topic': '/RefDeviceTraj_Unload',
        'finish_flag_topic': '/Sys_RUn_FlagUnloadExcuteFinish',
    },
    'reset': {
        'activate_topic': '/Sys_SRe_FlagResetExcute',
        'activate_value': 1.0,   # 1=dig-cycle, 2=bench
        'cmd_topic': '/RefDeviceTraj_Reset',
        'finish_flag_topic': None,  # 靠 cmd position.x 1→0 下降沿
    },
    'dig': {
        'activate_topic': '/Sys_SDi_FlagDigExcute',
        'activate_value': 1.0,
        'cmd_topic': '/RefDeviceTraj_Dig',
        'finish_flag_topic': None,
    },
}


class PhaseTrigger:
    """通用阶段触发助手。"""

    def __init__(self):
        phase = rospy.get_param('~phase', 'dump').lower()
        if phase not in PHASES:
            raise ValueError(f"unknown phase '{phase}', must be one of {list(PHASES)}")
        cfg = PHASES[phase]
        self.phase = phase

        # 起始位姿（deg）：swing/boom/arm/bucket，需与 plant 的 init_* 一致
        self.dig = {
            'swing': rospy.get_param('~dig_swing_deg', 180.0),
            'boom': rospy.get_param('~dig_boom_deg', 5.0),
            'arm': rospy.get_param('~dig_arm_deg', -110.0),
            'bucket': rospy.get_param('~dig_bucket_deg', -110.0),
        }
        self.reset_dur = rospy.get_param('~reset_dur_sec', 5.0)
        self.reset_rate = rospy.get_param('~reset_cmd_rate', 20.0)
        self.settle_tol = rospy.get_param('~settle_tol_deg', 6.0)
        self.wait_timeout = rospy.get_param('~wait_timeout_sec', 60.0)
        self.auto_loop = rospy.get_param('~auto_loop_sec', 0.0)

        # 阶段配置（可被私有参数覆盖）
        self.activate_topic = rospy.get_param('~activate_topic', cfg['activate_topic'])
        self.activate_value = float(rospy.get_param('~activate_value', cfg['activate_value']))
        self.cmd_topic = rospy.get_param('~cmd_topic', cfg['cmd_topic'])
        self.finish_flag_topic = rospy.get_param('~finish_flag_topic', cfg['finish_flag_topic'])
        if self.finish_flag_topic == '':
            self.finish_flag_topic = None

        # 复位通道：plant 中优先级最低的 /RefDeviceTraj_Dump
        self.reset_topic = rospy.get_param('~reset_topic', '/RefDeviceTraj_Dump')

        self.pub_reset = rospy.Publisher(self.reset_topic, Pose, queue_size=5)
        self.pub_activate = rospy.Publisher(self.activate_topic, Float64, queue_size=5)

        # 完成信号
        self.finish_flag_val = [None]
        self.cmd_position_x = [None]
        self.cmd_prev_x = [None]
        self.cmd_done_edge = [False]
        self.fb = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0, 'swing': 0.0}

        rospy.Subscriber('/joints_angle', Quaternion, self._joints_cb, queue_size=1)
        rospy.Subscriber('/heading2swing_topic', Float32, self._swing_cb, queue_size=1)
        rospy.Subscriber(self.cmd_topic, Pose, self._cmd_cb, queue_size=1)
        if self.finish_flag_topic:
            rospy.Subscriber(self.finish_flag_topic, Float64, self._finish_flag_cb, queue_size=1)

        # 触发入口：新话题 + 向后兼容 dump_trigger
        self.trigger_topic = rospy.get_param('~trigger_topic', '/phase_trigger/go')
        rospy.Subscriber(self.trigger_topic, Empty, self._go_cb, queue_size=1)
        if phase == 'dump':
            rospy.Subscriber('/dump_trigger/go', Empty, self._go_cb, queue_size=1)

        self.busy = False
        self.lock = threading.Lock()

        if self.auto_loop > 1e-6:
            rospy.Timer(rospy.Duration(self.auto_loop), lambda e: self._start_once())

        rospy.loginfo(
            "[phase_trigger:%s] ready; activate=%s=%.1f cmd=%s finish_flag=%s reset=%s; "
            "trigger: rostopic pub -1 %s std_msgs/Empty",
            phase, self.activate_topic, self.activate_value, self.cmd_topic,
            self.finish_flag_topic or '(position.x 1->0)', self.reset_topic,
            self.trigger_topic,
        )

    # ---------- 回调 ----------
    def _joints_cb(self, msg):
        self.fb['boom'], self.fb['arm'], self.fb['bucket'] = msg.x, msg.y, msg.z

    def _swing_cb(self, msg):
        self.fb['swing'] = msg.data

    def _cmd_cb(self, msg):
        self.cmd_position_x[0] = msg.position.x
        # 检测 1 → 0 下降沿
        prev = self.cmd_prev_x[0]
        if prev is not None and prev >= 0.5 and msg.position.x < 0.5:
            self.cmd_done_edge[0] = True
        self.cmd_prev_x[0] = msg.position.x

    def _finish_flag_cb(self, msg):
        self.finish_flag_val[0] = msg.data

    def _go_cb(self, _msg):
        self._start_once()

    def _start_once(self):
        with self.lock:
            if self.busy:
                rospy.loginfo_throttle(3.0, "[phase_trigger:%s] busy, ignore re-trigger", self.phase)
                return
            self.busy = True
        threading.Thread(target=self._run_cycle, daemon=True).start()

    # ---------- 一轮：复位→触发→等完成 ----------
    def _run_cycle(self):
        try:
            self._reset_to_dig()
            self._fire()
            self._wait_done()
            rospy.loginfo("[phase_trigger:%s] cycle finished, ready for next", self.phase)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("[phase_trigger:%s] cycle error: %s", self.phase, e)
        finally:
            with self.lock:
                self.busy = False

    def _dig_pose(self, running=True):
        p = Pose()
        p.position.x = 1.0 if running else 0.0
        p.orientation.x = self.dig['swing']
        p.orientation.y = self.dig['boom']
        p.orientation.z = self.dig['arm']
        p.orientation.w = self.dig['bucket']
        return p

    def _settle_ok(self):
        """反馈关节角是否已收敛到起始位姿附近（不含 swing，swing 收敛慢）。"""
        return (abs(self.fb['boom'] - self.dig['boom']) < self.settle_tol
                and abs(self.fb['arm'] - self.dig['arm']) < self.settle_tol
                and abs(self.fb['bucket'] - self.dig['bucket']) < self.settle_tol)

    def _reset_to_dig(self):
        rospy.loginfo("[phase_trigger:%s] resetting plant to dig pose ...", self.phase)
        start = rospy.Time.now()
        r = rospy.Rate(self.reset_rate)
        while not rospy.is_shutdown():
            self.pub_reset.publish(self._dig_pose(running=True))
            elapsed = (rospy.Time.now() - start).to_sec()
            if self._settle_ok() and elapsed > 1.0:
                break
            if elapsed > self.reset_dur + 3.0:
                rospy.logwarn("[phase_trigger:%s] reset not settled in %.1fs, proceed anyway",
                              self.phase, self.reset_dur + 3.0)
                break
            r.sleep()
        # 释放复位通道：发一帧 position.x=0，让 plant 认为该源已结束
        self.pub_reset.publish(self._dig_pose(running=False))
        rospy.sleep(0.5)

    def _fire(self):
        rospy.loginfo("[phase_trigger:%s] firing %s=%.1f",
                      self.phase, self.activate_topic, self.activate_value)
        # 清除历史完成信号
        self.finish_flag_val[0] = None
        self.cmd_done_edge[0] = False
        self.cmd_prev_x[0] = None
        self.pub_activate.publish(Float64(self.activate_value))
        rospy.sleep(0.2)
        self.pub_activate.publish(Float64(self.activate_value))  # 补一发确保触发

    def _wait_done(self):
        rospy.loginfo("[phase_trigger:%s] waiting for phase done ...", self.phase)
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            # 途径 1：finish flag 话题
            if self.finish_flag_topic and self.finish_flag_val[0] is not None \
                    and self.finish_flag_val[0] >= 0.5:
                rospy.sleep(0.5)
                if self.finish_flag_val[0] is not None and self.finish_flag_val[0] >= 0.5:
                    rospy.loginfo("[phase_trigger:%s] done via finish flag", self.phase)
                    return
            # 途径 2：cmd position.x 1→0 下降沿
            if self.cmd_done_edge[0]:
                rospy.sleep(0.5)
                rospy.loginfo("[phase_trigger:%s] done via position.x falling edge", self.phase)
                return
            if (rospy.Time.now() - start).to_sec() > self.wait_timeout:
                rospy.logwarn("[phase_trigger:%s] wait done timeout (%.1fs)",
                              self.phase, self.wait_timeout)
                return
            rospy.sleep(0.1)


def main():
    rospy.init_node('phase_trigger')
    PhaseTrigger()
    rospy.spin()


if __name__ == '__main__':
    main()
