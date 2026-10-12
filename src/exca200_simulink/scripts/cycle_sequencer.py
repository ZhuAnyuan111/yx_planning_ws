#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全循环编排器（L4 编排层，Phase 4 交付）。

状态机：IDLE → DIG → DUMP → RESET → DIG → ...
（"TRANSIT_TO_TRUCK" 由 dump 的 Seg1 回转内部完成；"TRANSIT_TO_DIG" 由 reset 内部完成）

触发协议（契约见 docs/sim_arch.md §3.4）：
  DIG   : /Sys_SeqAction_Dig         = 3.0   完成: /Sys_RUn_FlagDigFinish==1.0
                                              或 /RefDeviceTraj_Dig.position.x 1→0
  DUMP  : /Sys_SeqAction             = 4.0   完成: /Sys_RUn_FlagUnloadExcuteFinish==1.0
                                              或 /RefDeviceTraj_Unload.position.x 1→0
  RESET : /Sys_SRe_FlagResetExcute   = 1.0   完成: /RefDeviceTraj_Reset.position.x 1→0

每循环结束发布 /cycle_report (std_msgs/String, JSON)：
  {
    "cycle_id": int,
    "phase_durations_sec": {"dig": float, "dump": float, "reset": float},
    "cycle_duration_sec": float,
    "status": "ok" | "timeout:<phase>" | "fault:<phase>",
    "timestamp": float
  }

参数：
  ~cycle_count      目标循环数（0 = 无限，直到 Ctrl-C）
  ~phase_timeout_sec  单阶段超时（默认 60s）
  ~inter_phase_delay_sec  阶段间停顿（默认 0.5s，让 plant 稳定）
  ~auto_start       启动后自动开跑（默认 true）
  ~start_topic      外部启动话题（默认 /cycle_sequencer/start，std_msgs/Empty）

不修改任何 planner 与 plant。
"""

import json
import threading
import time

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, Float64, String


PHASE_ORDER = ('dig', 'dump', 'reset')

PHASE_CFG = {
    'dig': {
        'activate_topic': '/Sys_SeqAction_Dig',
        'activate_value': 3.0,
        'cmd_topic': '/RefDeviceTraj_Dig',
        'finish_flag_topic': '/Sys_RUn_FlagDigFinish',
    },
    'dump': {
        'activate_topic': '/Sys_SeqAction',
        'activate_value': 4.0,
        'cmd_topic': '/RefDeviceTraj_Unload',
        'finish_flag_topic': '/Sys_RUn_FlagUnloadExcuteFinish',
    },
    'reset': {
        'activate_topic': '/Sys_SRe_FlagResetExcute',
        'activate_value': 1.0,
        'cmd_topic': '/RefDeviceTraj_Reset',
        'finish_flag_topic': None,
    },
}


class PhaseMonitor:
    """监听单个阶段的完成信号（finish flag 或 position.x 下降沿）。"""

    def __init__(self, name, cfg):
        self.name = name
        self.cmd_topic = cfg['cmd_topic']
        self.finish_flag_topic = cfg['finish_flag_topic']
        self.done_event = threading.Event()
        self.prev_x = None
        self.finish_val = None
        if self.finish_flag_topic:
            rospy.Subscriber(self.finish_flag_topic, Float64, self._flag_cb, queue_size=1)
        rospy.Subscriber(self.cmd_topic, Pose, self._cmd_cb, queue_size=1)

    def _flag_cb(self, msg):
        self.finish_val = msg.data
        if msg.data >= 0.5:
            self.done_event.set()

    def _cmd_cb(self, msg):
        x = msg.position.x
        if self.prev_x is not None and self.prev_x >= 0.5 and x < 0.5:
            self.done_event.set()
        self.prev_x = x

    def reset(self):
        self.done_event.clear()
        self.prev_x = None
        self.finish_val = None


class CycleSequencer:
    """全循环编排器主类。"""

    def __init__(self):
        self.cycle_count = int(rospy.get_param('~cycle_count', 0))
        self.phase_timeout = float(rospy.get_param('~phase_timeout_sec', 60.0))
        self.inter_phase_delay = float(rospy.get_param('~inter_phase_delay_sec', 0.5))
        self.auto_start = bool(rospy.get_param('~auto_start', True))
        self.start_topic = rospy.get_param('~start_topic', '/cycle_sequencer/start')

        # 激活发布者
        self.pubs = {}
        for name, cfg in PHASE_CFG.items():
            self.pubs[name] = rospy.Publisher(cfg['activate_topic'], Float64,
                                              queue_size=5, latch=False)

        # 完成监视器
        self.monitors = {name: PhaseMonitor(name, cfg) for name, cfg in PHASE_CFG.items()}

        # 报告
        self.pub_report = rospy.Publisher('/cycle_report', String, queue_size=10)
        self.pub_state = rospy.Publisher('/cycle_sequencer/state', String,
                                         queue_size=1, latch=True)

        # 启动入口
        rospy.Subscriber(self.start_topic, Empty, self._start_cb, queue_size=1)

        self.running = False
        self.lock = threading.Lock()
        self.cycle_id = 0

        self._set_state('IDLE')
        if self.auto_start:
            # 延迟 2s 等所有节点 ready
            rospy.Timer(rospy.Duration(2.0), lambda e: self._start_cb(None), oneshot=True)

        rospy.loginfo("[sequencer] ready; cycle_count=%s phase_timeout=%.1fs "
                      "auto_start=%s start_topic=%s",
                      self.cycle_count or 'inf', self.phase_timeout,
                      self.auto_start, self.start_topic)

    def _set_state(self, s):
        self.pub_state.publish(String(s))

    def _start_cb(self, _msg):
        with self.lock:
            if self.running:
                rospy.loginfo_throttle(5.0, "[sequencer] already running, ignore")
                return
            self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _fire(self, phase):
        cfg = PHASE_CFG[phase]
        self.monitors[phase].reset()
        # 补发两次确保触发（幂等）
        self.pubs[phase].publish(Float64(cfg['activate_value']))
        rospy.sleep(0.15)
        self.pubs[phase].publish(Float64(cfg['activate_value']))
        rospy.loginfo("[sequencer] fired %s: %s=%.1f",
                      phase, cfg['activate_topic'], cfg['activate_value'])

    def _wait_done(self, phase):
        """返回 (done: bool, elapsed: float)。"""
        t0 = time.time()
        done = self.monitors[phase].done_event.wait(timeout=self.phase_timeout)
        return done, time.time() - t0

    def _run(self):
        try:
            while not rospy.is_shutdown():
                self.cycle_id += 1
                cid = self.cycle_id
                rospy.loginfo("[sequencer] === cycle %d start ===", cid)
                cycle_t0 = time.time()
                durations = {}
                status = 'ok'

                for phase in PHASE_ORDER:
                    if rospy.is_shutdown():
                        return
                    self._set_state(f'{phase.upper()}')
                    self._fire(phase)
                    done, elapsed = self._wait_done(phase)
                    durations[phase] = round(elapsed, 3)
                    if not done:
                        status = f'timeout:{phase}'
                        rospy.logwarn("[sequencer] cycle %d phase %s TIMEOUT (%.1fs)",
                                      cid, phase, elapsed)
                        break
                    rospy.loginfo("[sequencer] cycle %d phase %s done in %.2fs",
                                  cid, phase, elapsed)
                    rospy.sleep(self.inter_phase_delay)

                cycle_dur = round(time.time() - cycle_t0, 3)
                report = {
                    'cycle_id': cid,
                    'phase_durations_sec': durations,
                    'cycle_duration_sec': cycle_dur,
                    'status': status,
                    'timestamp': time.time(),
                }
                self.pub_report.publish(String(json.dumps(report)))
                self._set_state('IDLE' if status == 'ok' else f'FAULT({status})')
                rospy.loginfo("[sequencer] === cycle %d end: status=%s duration=%.2fs ===",
                              cid, status, cycle_dur)

                if self.cycle_count > 0 and cid >= self.cycle_count:
                    rospy.loginfo("[sequencer] reached target cycle_count=%d, stop",
                                  self.cycle_count)
                    break
        except Exception as e:  # noqa: BLE001
            rospy.logerr("[sequencer] unexpected error: %s", e)
        finally:
            with self.lock:
                self.running = False


def main():
    rospy.init_node('cycle_sequencer')
    CycleSequencer()
    rospy.spin()


if __name__ == '__main__':
    main()
