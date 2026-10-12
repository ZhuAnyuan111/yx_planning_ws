#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全循环回归测试（decision 节点版）。

运行 full_cycle_sim.launch（baseline 场景），检测 decision 节点完成至少一个
完整卡车装载循环（DISPATCH_TRUCK_LEAVE），比对 KPI 与黄金基线。
若黄金基线不存在，自动生成（首次运行 = 建立基线）。

运行方式：
  python3 tests/test_full_cycle_regression.py [--update-golden]
  或
  pytest tests/test_full_cycle_regression.py -v

依赖：roscore + 已编译的 exca200_simulink / excavator_system_decision /
      dump_trajectory_planner / reset_trajectory_planner。
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
GOLDEN_FILE = os.path.join(GOLDEN_DIR, 'full_cycle.json')
SCENARIO_FILE = os.path.join(PKG_DIR, 'scenarios', 'baseline.yaml')
TOLERANCE = 0.10  # 全循环容差 10%

# 测试隔离
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)
from _isolation import IsolatedRosMaster  # noqa: E402

# Decision 节点日志标记
_LEAVE_MARK = '[TruckInteractionScenario] -> DISPATCH_TRUCK_LEAVE'
_DIGGING_MARK = '[TruckInteractionScenario] -> DIGGING_CYCLE'
# ROS log timestamp pattern: [ INFO] [1234567890.123456789]:
_TS_RE = re.compile(r'\[\s*\w+\]\s*\[(\d+\.\d+)\]')


def _run_full_cycle(timeout_sec=180.0, cycles=1):
    """启动 full_cycle_sim.launch（headless），等待 decision 完成指定循环数。

    通过解析 decision 节点终端日志检测 DISPATCH_TRUCK_LEAVE 出现次数。
    """
    master = IsolatedRosMaster()
    master.start()
    env = master.child_env()
    launch_args = [
        'roslaunch', 'exca200_simulink', 'full_cycle_sim.launch',
        'rviz:=false',
        f'scenario_file:={SCENARIO_FILE}',
        'truck_full_threshold:=1',
        'auto_start_delay:=2.0',
        'truck_entry_delay:=2.0',
        'truck_loading_delay:=1.0',
        'truck_leave_delay:=2.0',
    ]
    log_path = os.path.join(TESTS_DIR, '.full_cycle_run.log')
    start = time.time()
    polled_count = 0
    try:
        with open(log_path, 'w') as logf:
            proc = subprocess.Popen(launch_args, stdout=logf,
                                    stderr=subprocess.STDOUT, env=env,
                                    preexec_fn=os.setsid)
            try:
                while time.time() - start < timeout_sec:
                    time.sleep(1.0)
                    if proc.poll() is not None:
                        break
                    # 轮询日志检查完成
                    with open(log_path, 'r', errors='replace') as rf:
                        content = rf.read()
                    polled_count = content.count(_LEAVE_MARK)
                    if polled_count >= cycles:
                        break
                time.sleep(1.0)
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

    # 权威解析：从完整日志提取 cycle 信息
    cycle_durations = []
    leave_count = 0
    digging_ts = None
    try:
        with open(log_path, 'r', errors='replace') as rf:
            for line in rf:
                if _DIGGING_MARK in line:
                    m = _TS_RE.search(line)
                    if m:
                        digging_ts = float(m.group(1))
                if _LEAVE_MARK in line:
                    leave_count += 1
                    m = _TS_RE.search(line)
                    if m and digging_ts is not None:
                        cycle_durations.append(round(float(m.group(1)) - digging_ts, 3))
                        digging_ts = None
    except OSError:
        pass

    elapsed = time.time() - start
    success = leave_count >= cycles
    return {
        'success': success,
        'elapsed_sec': round(elapsed, 2),
        'cycles_completed': leave_count,
        'cycle_durations_sec': cycle_durations,
    }


def _extract_kpi(result):
    kpi = {
        'success': result['success'],
        'cycles_completed': result['cycles_completed'],
    }
    durs = result.get('cycle_durations_sec') or []
    if durs:
        kpi['mean_cycle_duration_sec'] = round(sum(durs) / len(durs), 3)
    return kpi


def _compare_kpi(actual, golden, tol=TOLERANCE):
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


class TestFullCycleRegression(unittest.TestCase):
    """全循环回归测试（decision 节点编排）。"""

    @classmethod
    def setUpClass(cls):
        if not os.path.isfile(SCENARIO_FILE):
            raise unittest.SkipTest(f"scenario not found: {SCENARIO_FILE}")

    def test_full_cycle_baseline(self):
        """运行 baseline 全循环仿真（decision 编排），比对黄金基线。"""
        update = '--update-golden' in sys.argv
        result = _run_full_cycle()
        kpi = _extract_kpi(result)

        if not result['success']:
            self.fail(f"Decision node did not complete a full truck cycle within timeout. "
                      f"elapsed={result['elapsed_sec']}s, "
                      f"cycles_completed={result['cycles_completed']}")

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
