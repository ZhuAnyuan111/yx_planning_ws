#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Dump 阶段回归测试（Phase 6）。

运行 baseline 场景下的 dump 单阶段仿真，比对 KPI 与黄金基线。
若黄金基线不存在，自动生成（首次运行 = 建立基线）。

运行方式：
  python3 tests/test_dump_regression.py [--update-golden]
  或
  pytest tests/test_dump_regression.py -v

依赖：roscore + 已编译的 exca200_simulink / dump_trajectory_planner。
"""

import json
import os
import re
import signal
import subprocess
import sys
import time
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(TESTS_DIR)
GOLDEN_DIR = os.path.join(TESTS_DIR, 'golden')
GOLDEN_FILE = os.path.join(GOLDEN_DIR, 'baseline_dump.json')
SCENARIO_FILE = os.path.join(PKG_DIR, 'scenarios', 'baseline.yaml')
TOLERANCE = 0.05  # 5% 相对容差

# 测试隔离：每个测试独占一个 ROS master，避免残留参数跨测试污染
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)
from _isolation import IsolatedRosMaster  # noqa: E402

# roslaunch output="screen" 行首形如：[INFO] [1729234567.890123]: message
_TS_RE = re.compile(r'\[\d+\.\d+\]')

# phase_trigger 一轮完成的权威标记（"[phase_trigger:dump] cycle finished, ..."）
_FINISH_MARK = 'cycle finished'


def _run_dump_sim(timeout_sec=120.0):
    """启动 dump_viz_sim_single_trigger.launch（headless），自动触发一轮，收集 KPI。

    auto_loop_sec=6.0 让 phase_trigger 在启动 6s 后自动跑第一轮
    （复位→触发→等完成）。roslaunch 输出重定向到文件并轮询（避免管道块缓冲
    导致 readline 实时性丢失）；捕获首轮 'firing' 与 'done via' 的墙钟时间戳，
    计算 dump 执行时长（对 tau_boom 等动力学参数敏感，可作为回归 KPI）。

    注意：roslaunch（Python）在被重定向到文件时默认块缓冲自身 stdout，轮询可能
    实时看不到 'cycle finished'。故 (1) 通过 PYTHONUNBUFFERED=1 强制 roslaunch
    及其 Python 子节点行缓冲实时落盘；(2) 成功判定以 kill 后对完整日志文件的解析
    为准（权威），轮询仅作提前退出优化。

    隔离：本次运行挂载一个独占的 ROS master（随机空闲端口），避免其它测试
    遗留在常驻 roscore 上的私有参数（如 /exca_plant_sim/init_*）污染本测试。
    """
    master = IsolatedRosMaster()
    master.start()
    env = master.child_env()  # 含 ROS_MASTER_URI / ROS_IP / PYTHONUNBUFFERED
    launch_args = [
        'roslaunch', 'exca200_simulink', 'dump_viz_sim_single_trigger.launch',
        'rviz:=false', 'auto_loop_sec:=6.0',
    ]
    log_path = os.path.join(TESTS_DIR, '.dump_run.log')
    start = time.time()
    polled_finish = False
    try:
        with open(log_path, 'w') as logf:
            proc = subprocess.Popen(launch_args, stdout=logf,
                                    stderr=subprocess.STDOUT, env=env,
                                    preexec_fn=os.setsid)
            try:
                while time.time() - start < timeout_sec:
                    time.sleep(1.0)
                    if proc.poll() is not None:
                        break  # launch 自行退出
                    with open(log_path, 'r', errors='replace') as rf:
                        if _FINISH_MARK in rf.read():
                            polled_finish = True
                            break
                time.sleep(0.5)
            finally:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=10)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
    finally:
        master.stop()

    # kill 后对完整日志文件做权威解析：首轮 firing / done via 时间戳 + 成功标记
    fire_ts = None
    done_ts = None
    finish_detected = False
    try:
        with open(log_path, 'r', errors='replace') as rf:
            for line in rf:
                if _FINISH_MARK in line:
                    finish_detected = True
                ts = _parse_ts(line)
                if fire_ts is None and 'firing' in line and 'Sys_SeqAction' in line:
                    fire_ts = ts
                if done_ts is None and 'done via' in line:
                    done_ts = ts
    except OSError:
        pass
    # 轮询若已看到完成，同样视为成功（提前退出路径）
    finish_detected = finish_detected or polled_finish

    elapsed = time.time() - start
    dump_duration = None
    if fire_ts is not None and done_ts is not None and done_ts >= fire_ts:
        dump_duration = round(done_ts - fire_ts, 3)
    return {
        'success': finish_detected,
        'elapsed_sec': round(elapsed, 2),
        'dump_duration_sec': dump_duration,
    }


def _parse_ts(text):
    """从 roslaunch 日志行解析墙钟时间戳（秒），失败返回 None。"""
    m = _TS_RE.search(text)
    if not m:
        return None
    try:
        return float(m.group(0).strip('[]'))
    except ValueError:
        return None


def _extract_kpi(result):
    """从运行结果提取可比对的 KPI。

    - success：是否检测到一轮完成
    - dump_duration_sec：firing→done 的执行时长（对 tau/rate 敏感）
    elapsed_sec 受机器负载影响大，仅作记录不参与容差比对。
    """
    kpi = {'success': result['success']}
    if result.get('dump_duration_sec') is not None:
        kpi['dump_duration_sec'] = result['dump_duration_sec']
    return kpi


def _compare_kpi(actual, golden, tol=TOLERANCE):
    """比对 KPI，返回 (pass, diffs)。"""
    diffs = []
    for k, gv in golden.items():
        av = actual.get(k)
        if av is None:
            diffs.append(f"missing key: {k}")
            continue
        if isinstance(gv, (int, float)) and isinstance(av, (int, float)):
            if gv == 0:
                if abs(av) > tol:
                    diffs.append(f"{k}: golden=0 actual={av}")
            elif abs(av - gv) / abs(gv) > tol:
                diffs.append(f"{k}: golden={gv} actual={av} "
                             f"(diff={abs(av-gv)/abs(gv)*100:.1f}%)")
        elif av != gv:
            diffs.append(f"{k}: golden={gv} actual={av}")
    return len(diffs) == 0, diffs


class TestDumpRegression(unittest.TestCase):
    """Dump 阶段回归测试。"""

    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(SCENARIO_FILE):
            raise unittest.SkipTest(f"scenario not found: {SCENARIO_FILE}")

    def test_dump_baseline(self):
        """运行 baseline dump 仿真，比对黄金基线。"""
        update = '--update-golden' in sys.argv
        result = _run_dump_sim()
        kpi = _extract_kpi(result)

        if update or not os.path.isfile(GOLDEN_FILE):
            os.makedirs(GOLDEN_DIR, exist_ok=True)
            with open(GOLDEN_FILE, 'w') as f:
                json.dump(kpi, f, indent=2)
            print(f"[golden] written: {GOLDEN_FILE}")
            if not update:
                self.skipTest("golden baseline created (first run); re-run to validate")
            return

        with open(GOLDEN_FILE) as f:
            golden = json.load(f)

        passed, diffs = _compare_kpi(kpi, golden)
        if not passed:
            self.fail(f"KPI regression detected:\n" + "\n".join(f"  - {d}" for d in diffs))
        print(f"[regression] PASS: {kpi}")


def main():
    unittest.main(verbosity=2)


if __name__ == '__main__':
    main()
