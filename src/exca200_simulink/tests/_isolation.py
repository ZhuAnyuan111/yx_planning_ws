#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试隔离工具：为每个回归测试分配独立的 ROS master。

背景
----
回归测试用 SIGKILL 结束 roslaunch/rosrun 子进程，roslaunch 来不及清理它在
参数服务器上写入的私有参数（如 /exca_plant_sim/init_swing_deg）。若多个测试
共用一个常驻 roscore，这些残留参数会跨测试泄漏：例如 test_plant_step_response
把 init 位姿设为 0、tau_boom=0.3、faults/joint_stuck/enabled=true，随后
test_full_cycle 的 plant 读到这些残留值，导致初始位姿错误、dump 规划中止。

本模块让每个测试启动一个独占的 roscore（随机空闲端口），测试结束后连同其上的
所有参数一起销毁，从根本上消除跨测试污染，并使测试可并行、可乱序运行。

用法
----
    from _isolation import IsolatedRosMaster

    master = IsolatedRosMaster()
    master.start()                 # 启动独占 roscore 并等待就绪
    env = master.child_env()       # 带 ROS_MASTER_URI 的子进程环境
    proc = subprocess.Popen([...], env=env, ...)
    ...
    master.stop()                  # 关闭 roscore（连同其参数）

若测试进程自身要用 rospy，请在 start() 之后、rospy.init_node() 之前调用
master.export_to_os_environ()，把 ROS_MASTER_URI 写入本进程环境。
"""

import os
import signal
import socket
import subprocess
import time


def _free_port():
    """向 OS 申请一个空闲 TCP 端口并返回端口号。"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _port_open(port, host='127.0.0.1', timeout=0.2):
    """探测 host:port 是否可连接（roscore 就绪的最简判据）。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        return s.connect_ex((host, port)) == 0


class IsolatedRosMaster:
    """独占式 ROS master（roscore）生命周期管理。"""

    def __init__(self):
        self.port = _free_port()
        self.uri = 'http://127.0.0.1:%d' % self.port
        self.proc = None

    def start(self, timeout=20.0):
        """启动 roscore 并阻塞等待其端口就绪。失败抛 RuntimeError。"""
        env = os.environ.copy()
        env['ROS_MASTER_URI'] = self.uri
        env['ROS_IP'] = '127.0.0.1'
        env['ROS_HOSTNAME'] = 'localhost'
        self.proc = subprocess.Popen(
            ['roscore', '-p', str(self.port)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env=env, preexec_fn=os.setsid)
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.proc.poll() is not None:
                raise RuntimeError('roscore exited early (rc=%s)' % self.proc.returncode)
            if _port_open(self.port):
                time.sleep(1.0)  # 端口开放后再给 XML-RPC/参数服务器一点初始化时间
                return True
            time.sleep(0.2)
        self.stop()
        raise RuntimeError('roscore not ready within %.1fs on %s' % (timeout, self.uri))

    def child_env(self, base=None):
        """返回指向本独占 master 的子进程环境（含实时刷盘）。"""
        e = base.copy() if base is not None else os.environ.copy()
        e['ROS_MASTER_URI'] = self.uri
        e['ROS_IP'] = '127.0.0.1'
        e['ROS_HOSTNAME'] = 'localhost'
        e['PYTHONUNBUFFERED'] = '1'
        return e

    def export_to_os_environ(self):
        """把 ROS_MASTER_URI 写入本进程环境（供测试进程内 rospy.init_node 使用）。

        必须在 rospy.init_node() 之前调用。
        """
        os.environ['ROS_MASTER_URI'] = self.uri
        os.environ['ROS_IP'] = '127.0.0.1'
        os.environ['ROS_HOSTNAME'] = 'localhost'

    def stop(self):
        """关闭 roscore 进程组（其上的所有参数随之销毁）。幂等。"""
        if self.proc is None:
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            self.proc.wait(timeout=8)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        finally:
            self.proc = None
