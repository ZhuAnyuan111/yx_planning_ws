#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""KPI 采集器（L5 度量层，Phase 5 交付）。

订阅关键话题，按循环汇总指标，写入 CSV + JSON。

订阅：
  /cycle_report            (std_msgs/String, JSON)  cycle_sequencer 每循环报告
  /joints_angle            (geometry_msgs/Quaternion)  实际关节角 (deg)
  /heading2swing_topic     (std_msgs/Float32)          实际 swing (deg)
  /RefDeviceTraj_Unload    (geometry_msgs/Pose)        dump 指令
  /RefDeviceTraj_Reset     (geometry_msgs/Pose)        reset 指令
  /RefDeviceTraj_Dig       (geometry_msgs/Pose)        dig 指令

每循环指标：
  - cycle_id, status, cycle_duration_sec
  - phase_durations_sec (dig/dump/reset)
  - max_tracking_error_deg (各阶段指令 vs 实际的最大关节误差)
  - final_tracking_error_deg (循环结束时各关节误差)
  - cmd_count (各阶段收到的指令帧数)

输出：
  ~output_dir/run_<ts>.csv   每行一个循环
  ~output_dir/run_<ts>.json  结构化全量数据

参数：
  ~output_dir    输出目录（默认 metrics/）
  ~max_cycles    采集多少个循环后自动退出（0 = 无限，Ctrl-C 退出）
"""

import csv
import json
import os
import threading
import time

import rospy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float32, String


def _wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


class KpiCollector:
    def __init__(self):
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        default_out = os.path.join(pkg_dir, 'metrics')
        self.output_dir = rospy.get_param('~output_dir', default_out)
        self.max_cycles = int(rospy.get_param('~max_cycles', 0))
        os.makedirs(self.output_dir, exist_ok=True)

        ts = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.output_dir, f'run_{ts}.csv')
        self.json_path = os.path.join(self.output_dir, f'run_{ts}.json')

        self.lock = threading.Lock()
        self.cycles = []           # 全量 JSON 数据
        self.csv_rows = []         # CSV 行
        self.csv_fields = [
            'cycle_id', 'status', 'cycle_duration_sec',
            'dur_dig', 'dur_dump', 'dur_reset',
            'max_err_dig', 'max_err_dump', 'max_err_reset',
            'final_err_boom', 'final_err_arm', 'final_err_bucket', 'final_err_swing',
            'cmd_count_dig', 'cmd_count_dump', 'cmd_count_reset',
            'timestamp',
        ]

        # 实时状态
        self.actual = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0, 'swing': 0.0}
        self.cmd = {
            'dig': {'swing': 0.0, 'boom': 0.0, 'arm': 0.0, 'bucket': 0.0, 'count': 0, 'max_err': 0.0},
            'dump': {'swing': 0.0, 'boom': 0.0, 'arm': 0.0, 'bucket': 0.0, 'count': 0, 'max_err': 0.0},
            'reset': {'swing': 0.0, 'boom': 0.0, 'arm': 0.0, 'bucket': 0.0, 'count': 0, 'max_err': 0.0},
        }
        self.current_phase = None  # 由 /cycle_sequencer/state 推断

        # 订阅
        rospy.Subscriber('/joints_angle', Quaternion, self._joints_cb, queue_size=1)
        rospy.Subscriber('/heading2swing_topic', Float32, self._swing_cb, queue_size=1)
        rospy.Subscriber('/RefDeviceTraj_Dig', Pose, self._make_cmd_cb('dig'), queue_size=1)
        rospy.Subscriber('/RefDeviceTraj_Unload', Pose, self._make_cmd_cb('dump'), queue_size=1)
        rospy.Subscriber('/RefDeviceTraj_Reset', Pose, self._make_cmd_cb('reset'), queue_size=1)
        rospy.Subscriber('/cycle_report', String, self._report_cb, queue_size=1)
        rospy.Subscriber('/cycle_sequencer/state', String, self._state_cb, queue_size=1)

        rospy.loginfo("[kpi] collector ready; output=%s max_cycles=%s",
                      self.output_dir, self.max_cycles or 'inf')

    # ---------- 回调 ----------
    def _joints_cb(self, msg):
        with self.lock:
            self.actual['boom'] = msg.x
            self.actual['arm'] = msg.y
            self.actual['bucket'] = msg.z

    def _swing_cb(self, msg):
        with self.lock:
            self.actual['swing'] = msg.data

    def _make_cmd_cb(self, phase):
        def cb(msg):
            with self.lock:
                c = self.cmd[phase]
                c['swing'] = msg.orientation.x
                c['boom'] = msg.orientation.y
                c['arm'] = msg.orientation.z
                c['bucket'] = msg.orientation.w
                c['count'] += 1
                # 跟踪误差（仅在执行中 position.x=1 时统计）
                if msg.position.x >= 0.5:
                    err = max(
                        abs(c['boom'] - self.actual['boom']),
                        abs(c['arm'] - self.actual['arm']),
                        abs(c['bucket'] - self.actual['bucket']),
                        abs(_wrap180(c['swing'] - self.actual['swing'])),
                    )
                    if err > c['max_err']:
                        c['max_err'] = err
        return cb

    def _state_cb(self, msg):
        s = msg.data.upper()
        if 'DIG' in s:
            self.current_phase = 'dig'
        elif 'DUMP' in s:
            self.current_phase = 'dump'
        elif 'RESET' in s:
            self.current_phase = 'reset'
        else:
            self.current_phase = None

    def _report_cb(self, msg):
        try:
            report = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with self.lock:
            # 终态误差
            final_err = {
                'boom': abs(self.cmd.get(self.current_phase, {}).get('boom', 0) - self.actual['boom']),
                'arm': abs(self.cmd.get(self.current_phase, {}).get('arm', 0) - self.actual['arm']),
                'bucket': abs(self.cmd.get(self.current_phase, {}).get('bucket', 0) - self.actual['bucket']),
                'swing': abs(_wrap180(self.cmd.get(self.current_phase, {}).get('swing', 0) - self.actual['swing'])),
            }
            durations = report.get('phase_durations_sec', {})
            row = {
                'cycle_id': report.get('cycle_id'),
                'status': report.get('status', 'unknown'),
                'cycle_duration_sec': report.get('cycle_duration_sec', 0),
                'dur_dig': durations.get('dig', ''),
                'dur_dump': durations.get('dump', ''),
                'dur_reset': durations.get('reset', ''),
                'max_err_dig': round(self.cmd['dig']['max_err'], 3),
                'max_err_dump': round(self.cmd['dump']['max_err'], 3),
                'max_err_reset': round(self.cmd['reset']['max_err'], 3),
                'final_err_boom': round(final_err['boom'], 3),
                'final_err_arm': round(final_err['arm'], 3),
                'final_err_bucket': round(final_err['bucket'], 3),
                'final_err_swing': round(final_err['swing'], 3),
                'cmd_count_dig': self.cmd['dig']['count'],
                'cmd_count_dump': self.cmd['dump']['count'],
                'cmd_count_reset': self.cmd['reset']['count'],
                'timestamp': report.get('timestamp', time.time()),
            }
            self.csv_rows.append(row)
            self.cycles.append({**report, 'kpi': row})
            # 重置 per-cycle 累计量
            for ph in self.cmd:
                self.cmd[ph]['max_err'] = 0.0
                self.cmd[ph]['count'] = 0
            self._flush()
            cid = report.get('cycle_id')
            rospy.loginfo("[kpi] cycle %s recorded: status=%s dur=%.2fs",
                          cid, row['status'], row['cycle_duration_sec'])
            if self.max_cycles > 0 and len(self.cycles) >= self.max_cycles:
                rospy.loginfo("[kpi] reached max_cycles=%d, shutdown", self.max_cycles)
                rospy.signal_shutdown('kpi max_cycles reached')

    # ---------- 输出 ----------
    def _flush(self):
        # CSV
        with open(self.csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=self.csv_fields)
            w.writeheader()
            w.writerows(self.csv_rows)
        # JSON
        with open(self.json_path, 'w') as f:
            json.dump({'cycles': self.cycles}, f, indent=2, ensure_ascii=False)


def main():
    rospy.init_node('kpi_collector')
    KpiCollector()
    rospy.spin()


if __name__ == '__main__':
    main()
