#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
项目          修改人     修改日期       改动
无人挖掘机    wuql17    2026/06/31    自动依据关节角及旋转角变化发布数据采集开关信号
无人挖掘机    wuql17    2026/06/31    添加自动截断逻辑，如果超过十分钟，强制发0



"""

import math
import threading
from collections import deque

import rospy
from interfaces.msg import JointAngles
from std_msgs.msg import Float32, Int8


class DataCaptureSignalPublisher:
    def __init__(self):
        self.publish_rate_hz = max(0.1, float(rospy.get_param("~publish_rate_hz", 1.0)))
        self.window_seconds = max(0.1, float(rospy.get_param("~window_seconds", 30.0)))
        self.angle_threshold_deg = float(rospy.get_param("~angle_threshold_deg", 10.0))
        self.rise_cooldown_seconds = max(0.0, float(rospy.get_param("~rise_cooldown_seconds", 600.0)))
        self.max_on_duration_seconds = max(0.0, float(rospy.get_param("~max_on_duration_seconds", 480.0)))

        self._lock = threading.Lock()
        self._bucket_deg_history = deque()
        self._swivel_deg_history = deque()
        self._current_signal = 0
        self._last_fall_time = None
        self._signal_one_start_time = None

        self._publisher = rospy.Publisher("/data_capture_switch_manual", Int8, queue_size=10)
        self._joints_sub = rospy.Subscriber("/joints_angles", JointAngles, self._joints_angles_callback, queue_size=10)
        self._swivel_sub = rospy.Subscriber("/swivel_angle", Float32, self._swivel_angle_callback, queue_size=10)

    def _joints_angles_callback(self, msg):
        bucket_deg = math.degrees(float(msg.joint_bucket))
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._append_sample(self._bucket_deg_history, now, bucket_deg)

    def _swivel_angle_callback(self, msg):
        swivel_deg = float(msg.data)
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._append_sample(self._swivel_deg_history, now, swivel_deg)

    def _append_sample(self, history, stamp_sec, value_deg):
        history.append((stamp_sec, value_deg))
        self._prune_history(history, stamp_sec)

    def _prune_history(self, history, now_sec):
        cutoff = now_sec - self.window_seconds
        while history and history[0][0] < cutoff:
            history.popleft()

    @staticmethod
    def _history_delta(history):
        if not history:
            return 0.0
        values = [sample[1] for sample in history]
        return max(values) - min(values)

    def _motion_exceeds_threshold_locked(self, now_sec):
        self._prune_history(self._bucket_deg_history, now_sec)
        self._prune_history(self._swivel_deg_history, now_sec)

        if not self._bucket_deg_history or not self._swivel_deg_history:
            return False

        bucket_delta = self._history_delta(self._bucket_deg_history)
        swivel_delta = self._history_delta(self._swivel_deg_history)
        return bucket_delta > self.angle_threshold_deg or swivel_delta > self.angle_threshold_deg

    def _cooldown_allows_rise_locked(self, now_sec):
        if self._last_fall_time is None:
            return True
        return now_sec - self._last_fall_time >= self.rise_cooldown_seconds

    def update_signal(self):
        now_sec = rospy.Time.now().to_sec()
        with self._lock:
            motion_active = self._motion_exceeds_threshold_locked(now_sec)
            next_signal = 1 if motion_active else 0

            if self._current_signal == 0 and next_signal == 1 and not self._cooldown_allows_rise_locked(now_sec):
                next_signal = 0

            if (
                self._current_signal == 1
                and self._signal_one_start_time is not None
                and now_sec - self._signal_one_start_time >= self.max_on_duration_seconds
            ):
                next_signal = 0

            if self._current_signal == 1 and next_signal == 0:
                self._last_fall_time = now_sec
                self._signal_one_start_time = None
                rospy.loginfo("data_capture_switch changed 1 -> 0; cooldown %.1fs started", self.rise_cooldown_seconds)
            elif self._current_signal == 0 and next_signal == 1:
                self._signal_one_start_time = now_sec
                rospy.loginfo("data_capture_switch changed 0 -> 1")

            self._current_signal = next_signal
            return self._current_signal

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        rospy.loginfo(
            "data_capture_signal_pub started. window=%.1fs threshold=%.1fdeg cooldown=%.1fs max_on=%.1fs publish_rate=%.1fHz",
            self.window_seconds,
            self.angle_threshold_deg,
            self.rise_cooldown_seconds,
            self.max_on_duration_seconds,
            self.publish_rate_hz,
        )

        self._publisher.publish(Int8(data=0))
        while not rospy.is_shutdown():
            signal = self.update_signal()
            self._publisher.publish(Int8(data=signal))
            rate.sleep()


def main():
    rospy.init_node("data_capture_signal_pub", anonymous=False)
    node = DataCaptureSignalPublisher()
    node.spin()


if __name__ == "__main__":
    main()
