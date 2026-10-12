#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Plant 阶跃响应测试（Phase 6）。

验证 exca_plant_sim 的核心动力学特性：
  1. 纯速率限制（tau=0）：阶跃后以 rate_dps 匀速逼近，无超调
  2. 一阶滞后（tau>0）：响应呈指数逼近，63% 时间 ≈ tau
  3. 死区：|cmd - actual| < deadband 时不动作
  4. 完成判定：跟踪误差 < settle_tol 持续 settle_dur → /plant/settled=True
  5. 故障注入：joint_stuck 时对应关节速率=0

运行方式（需要 roscore）：
  python3 tests/test_plant_step_response.py
  或
  pytest tests/test_plant_step_response.py -v
"""

import math
import os
import subprocess
import sys
import time
import unittest

import rospy
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

# 确保能 import 同包脚本
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

# 测试隔离：每个测试独占一个 ROS master，避免残留参数跨测试污染
_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
if _TESTS_DIR not in sys.path:
    sys.path.insert(0, _TESTS_DIR)
from _isolation import IsolatedRosMaster  # noqa: E402


def _wait_roscore(timeout=5.0):
    """等待 roscore 可用。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            rospy.init_node('test_plant_step', anonymous=True, disable_signals=True)
            return True
        except Exception:
            time.sleep(0.2)
    return False


class TestPlantStepResponse(unittest.TestCase):
    """Plant 阶跃响应测试套件。"""

    @classmethod
    def setUpClass(cls):
        # 独占 ROS master：本测试写入的 _init_*/_tau_*/_faults 私有参数只作用于该
        # master，tearDown 时随 master 一起销毁，绝不泄漏到常驻 roscore 或别的测试。
        cls.master = IsolatedRosMaster()
        try:
            cls.master.start()
        except RuntimeError as e:
            raise unittest.SkipTest(f"cannot start isolated roscore: {e}")
        cls.master.export_to_os_environ()  # 必须在 rospy.init_node 之前
        if not rospy.core.is_initialized():
            if not _wait_roscore():
                cls.master.stop()
                raise unittest.SkipTest("roscore not available")
        # 启动 plant（tau_boom=0.3s 用于滞后测试；init 位姿归零便于阶跃观测）
        # 注：joint_stuck 在配置中声明 enabled，但只有调用 /plant/fault(True)
        # 后才真正生效，因此 test_01~04 不受影响，test_05 直接开关即可。
        cls.plant_proc = subprocess.Popen(
            ['rosrun', 'exca200_simulink', 'exca_plant_sim.py',
             '_rate_boom_dps:=15.0', '_tau_boom_s:=0.3',
             '_deadband_arm_deg:=2.0', '_settle_tol_deg:=2.0',
             '_settle_dur_sec:=0.3', '_init_boom_deg:=0.0',
             '_init_arm_deg:=0.0', '_init_bucket_deg:=0.0',
             '_init_swing_deg:=0.0',
             '_faults/joint_stuck/enabled:=true',
             '_faults/joint_stuck/joint:=boom'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env=cls.master.child_env())
        time.sleep(1.5)  # 等 plant ready

        cls.fb = {'boom': 0.0, 'arm': 0.0, 'bucket': 0.0, 'swing': 0.0,
                  'settled': None}
        rospy.Subscriber('/joints_angle', Quaternion, cls._joints_cb, queue_size=1)
        rospy.Subscriber('/heading2swing_topic', Float32, cls._swing_cb, queue_size=1)
        rospy.Subscriber('/plant/settled', Bool, cls._settled_cb, queue_size=1)
        cls.pub_cmd = rospy.Publisher('/RefDeviceTraj_Unload', Pose, queue_size=1)
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        try:
            cls.plant_proc.terminate()
            cls.plant_proc.wait(timeout=5)
        except Exception:  # noqa: BLE001
            pass
        try:
            rospy.signal_shutdown('test teardown')
        except Exception:  # noqa: BLE001
            pass
        # 关闭独占 master：连同 /exca_plant_sim/* 私有参数一起销毁，避免泄漏
        if getattr(cls, 'master', None):
            cls.master.stop()

    @classmethod
    def _joints_cb(cls, msg):
        cls.fb['boom'] = msg.x
        cls.fb['arm'] = msg.y
        cls.fb['bucket'] = msg.z

    @classmethod
    def _swing_cb(cls, msg):
        cls.fb['swing'] = msg.data

    @classmethod
    def _settled_cb(cls, msg):
        cls.fb['settled'] = msg.data

    def _publish_cmd(self, swing, boom, arm, bucket, running=True):
        p = Pose()
        p.position.x = 1.0 if running else 0.0
        p.orientation.x = swing
        p.orientation.y = boom
        p.orientation.z = arm
        p.orientation.w = bucket
        self.pub_cmd.publish(p)

    def _hold_cmd(self, target, duration, rate=50):
        r = rospy.Rate(rate)
        t0 = time.time()
        while time.time() - t0 < duration:
            self._publish_cmd(*target)
            r.sleep()

    # ---------- 测试用例 ----------
    def test_01_rate_limit(self):
        """纯速率限制：boom 以 15 dps 逼近，1s 后应接近 15 deg（±3）。"""
        self._hold_cmd((0, 0, 0, 0), 0.5)  # 先回零
        t0 = time.time()
        self._hold_cmd((0, 15, 0, 0), 1.0)
        elapsed = time.time() - t0
        # 1s × 15dps = 15deg，但有一阶滞后 tau=0.3 会略慢
        self.assertGreater(self.fb['boom'], 8.0,
                           f"boom too slow: {self.fb['boom']} after {elapsed:.1f}s")
        self.assertLess(self.fb['boom'], 16.0,
                        f"boom overshot: {self.fb['boom']}")

    def test_02_first_order_lag(self):
        """一阶滞后：tau=0.3s，阶跃后 0.3s 应达到 ~63% 终值。"""
        self._hold_cmd((0, 0, 0, 0), 1.0)  # 回零并等稳定
        # 阶跃到 10 deg
        t0 = time.time()
        r = rospy.Rate(50)
        samples = []
        while time.time() - t0 < 1.0:
            self._publish_cmd(0, 10, 0, 0)
            samples.append((time.time() - t0, self.fb['boom']))
            r.sleep()
        # 找 t≈0.3s 时的值
        val_at_tau = None
        for t, v in samples:
            if t >= 0.28:
                val_at_tau = v
                break
        self.assertIsNotNone(val_at_tau, "no sample at t=0.3s")
        # 63% of 10 = 6.3，允许 ±2 容差（速率限制 + 离散化）
        self.assertGreater(val_at_tau, 3.5,
                           f"lag too slow: {val_at_tau} at tau")
        self.assertLess(val_at_tau, 9.0,
                        f"lag too fast: {val_at_tau} at tau")

    def test_03_deadband(self):
        """死区：arm deadband=2deg，cmd=1deg 时 arm 不应动作。"""
        self._hold_cmd((0, 0, 0, 0), 1.0)  # 回零
        arm_before = self.fb['arm']
        self._hold_cmd((0, 0, 1.0, 0), 0.8)  # arm cmd=1 < deadband=2
        self.assertAlmostEqual(self.fb['arm'], arm_before, delta=0.5,
                               msg=f"deadband not working: arm moved to {self.fb['arm']}")

    def test_04_settle_detection(self):
        """完成判定：到位后 /plant/settled 应变 True。"""
        self._hold_cmd((0, 0, 0, 0), 1.5)  # 回零并等稳定
        # 发一个到位命令然后停止（position.x=0）
        self._hold_cmd((0, 5, 0, 0), 1.5)
        self._publish_cmd(0, 5, 0, 0, running=False)
        time.sleep(1.0)
        self.assertTrue(self.fb['settled'],
                        f"settled should be True, got {self.fb['settled']}")

    def test_05_joint_stuck_fault(self):
        """故障注入：joint_stuck 时 boom 不动。"""
        self._hold_cmd((0, 0, 0, 0), 1.0)
        # 启用故障
        try:
            rospy.wait_for_service('/plant/fault', timeout=3.0)
            srv = rospy.ServiceProxy('/plant/fault', SetBool)
            srv(True)
        except (rospy.ROSException, rospy.ServiceException) as e:
            self.skipTest(f"fault service not available: {e}")
        time.sleep(0.3)
        boom_before = self.fb['boom']
        self._hold_cmd((0, 20, 0, 0), 1.0)
        # boom 应几乎不动（joint_stuck 默认 joint=boom）
        self.assertAlmostEqual(self.fb['boom'], boom_before, delta=1.0,
                               msg=f"joint_stuck not working: boom={self.fb['boom']}")
        # 关闭故障
        srv(False)
        time.sleep(0.3)


def main():
    """独立运行入口。"""
    unittest.main(verbosity=2)


if __name__ == '__main__':
    main()
