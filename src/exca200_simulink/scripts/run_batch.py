#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量回归运行器（L6 入口层，Phase 5 交付）。

遍历 scenarios/*.yaml × N 次重复，对每个 case 启动 full_cycle_sim.launch（headless），
等待 cycle_sequencer 完成目标循环数或超时，收集 KPI，汇总报告。

用法：
  python3 run_batch.py --scenarios scenarios/ --repeats 3 --cycles 2
  python3 run_batch.py --scenarios scenarios/baseline.yaml scenarios/tight_truck.yaml --repeats 1 --cycles 1
  python3 run_batch.py --list   # 仅列出可用场景

输出：
  metrics/batch_<ts>/
    ├── <scenario>_<rep>/run_*.csv
    ├── <scenario>_<rep>/run_*.json
    └── summary.md

依赖：roslaunch, rostopic, 以及已编译的 exca200_simulink 包。
"""

import argparse
import glob
import json
import os
import signal
import subprocess
import sys
import time

import yaml


PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SCENARIO_DIR = os.path.join(PKG_DIR, 'scenarios')
DEFAULT_METRICS_DIR = os.path.join(PKG_DIR, 'metrics')
LAUNCH_FILE = 'full_cycle_sim.launch'
PKG_NAME = 'exca200_simulink'


def find_scenarios(paths):
    """展开场景路径列表（支持目录 / glob / 单文件）。"""
    files = []
    for p in paths:
        if os.path.isdir(p):
            files.extend(sorted(glob.glob(os.path.join(p, '*.yaml'))))
        elif '*' in p or '?' in p:
            files.extend(sorted(glob.glob(p)))
        elif os.path.isfile(p):
            files.append(p)
        else:
            print(f"[warn] scenario not found: {p}", file=sys.stderr)
    return files


def load_plant_overrides(scenario_path):
    """读取场景 YAML 中的 plant_overrides 段（如 saturated_valve 的 rate_boom_dps）。"""
    try:
        with open(scenario_path) as f:
            d = yaml.safe_load(f) or {}
        return d.get('plant_overrides', {}) or {}
    except (OSError, yaml.YAMLError):
        return {}


def run_one_case(scenario_path, repeat_idx, cycles, timeout_sec, batch_dir, rviz=False):
    """运行单个 case，返回 (case_dir, success, kpi_json_path)。"""
    name = os.path.splitext(os.path.basename(scenario_path))[0]
    case_name = f"{name}_rep{repeat_idx}"
    case_dir = os.path.join(batch_dir, case_name)
    os.makedirs(case_dir, exist_ok=True)

    overrides = load_plant_overrides(scenario_path)
    launch_args = [
        'roslaunch', PKG_NAME, LAUNCH_FILE,
        f'rviz:={"true" if rviz else "false"}',
        f'scenario_file:={scenario_path}',
        f'cycle_count:={cycles}',
        'auto_start:=true',
        f'phase_timeout_sec:={max(30.0, timeout_sec / max(cycles, 1) / 3)}',
    ]
    for k, v in overrides.items():
        launch_args.append(f'{k}:={v}')

    log_path = os.path.join(case_dir, 'roslaunch.log')
    print(f"[batch] === {case_name} ===")
    print(f"[batch] cmd: {' '.join(launch_args)}")

    env = os.environ.copy()
    with open(log_path, 'w') as logf:
        proc = subprocess.Popen(launch_args, stdout=logf, stderr=subprocess.STDOUT,
                                env=env, preexec_fn=os.setsid)
        # 等待完成：监控 /cycle_report 数量或超时
        start = time.time()
        completed = 0
        success = False
        while time.time() - start < timeout_sec:
            if proc.poll() is not None:
                break
            # 查询已发布的 cycle_report 数量
            try:
                out = subprocess.check_output(
                    ['rostopic', 'echo', '-n', '1', '--filter',
                     f'"m.data"', '/cycle_report'],
                    timeout=2, stderr=subprocess.DEVNULL, env=env)
                if out:
                    completed += 1
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
                pass
            if completed >= cycles:
                success = True
                break
            time.sleep(1.0)

        # 终止 roslaunch
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    # 收集 KPI 文件（kpi_collector 写入 metrics/，需要拷贝到 case_dir）
    kpi_json = None
    for f in sorted(glob.glob(os.path.join(DEFAULT_METRICS_DIR, 'run_*.json')),
                    key=os.path.getmtime, reverse=True):
        if os.path.getmtime(f) > start - 5:
            kpi_json = f
            break
    if kpi_json:
        import shutil
        shutil.copy2(kpi_json, case_dir)
        csv_src = kpi_json.replace('.json', '.csv')
        if os.path.isfile(csv_src):
            import shutil
            shutil.copy2(csv_src, case_dir)

    status = 'OK' if success else 'TIMEOUT/FAIL'
    print(f"[batch] {case_name}: {status} ({completed}/{cycles} cycles, "
          f"{time.time()-start:.1f}s)")
    return case_dir, success, kpi_json


def write_summary(batch_dir, results):
    """生成 summary.md 汇总表。"""
    summary_path = os.path.join(batch_dir, 'summary.md')
    lines = [
        f"# 批量回归报告",
        f"",
        f"生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"",
        f"| Case | Status | Cycles | Avg Cycle (s) | Max Err (deg) |",
        f"|------|--------|--------|---------------|---------------|",
    ]
    for case_name, success, kpi_json in results:
        status = 'PASS' if success else 'FAIL'
        avg_dur = ''
        max_err = ''
        if kpi_json and os.path.isfile(kpi_json):
            try:
                with open(kpi_json) as f:
                    data = json.load(f)
                cycles = data.get('cycles', [])
                if cycles:
                    durs = [c.get('cycle_duration_sec', 0) for c in cycles]
                    avg_dur = f"{sum(durs)/len(durs):.2f}"
                    errs = []
                    for c in cycles:
                        kpi = c.get('kpi', {})
                        for k in ('max_err_dig', 'max_err_dump', 'max_err_reset'):
                            v = kpi.get(k, 0)
                            if isinstance(v, (int, float)):
                                errs.append(v)
                    if errs:
                        max_err = f"{max(errs):.2f}"
            except (json.JSONDecodeError, OSError):
                pass
        lines.append(f"| {case_name} | {status} | {len(results)} | {avg_dur} | {max_err} |")

    lines.extend([
        f"",
        f"## 失败 Case 列表",
        f"",
    ])
    failed = [r for r in results if not r[1]]
    if failed:
        for case_name, _, _ in failed:
            lines.append(f"- {case_name}")
    else:
        lines.append("(无)")

    with open(summary_path, 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"[batch] summary written: {summary_path}")


def main():
    parser = argparse.ArgumentParser(description='挖掘机仿真批量回归运行器')
    parser.add_argument('--scenarios', nargs='+',
                        default=[DEFAULT_SCENARIO_DIR],
                        help='场景文件/目录/glob（默认 scenarios/）')
    parser.add_argument('--repeats', type=int, default=1, help='每个场景重复次数')
    parser.add_argument('--cycles', type=int, default=1, help='每次运行循环数')
    parser.add_argument('--timeout', type=float, default=300.0,
                        help='单 case 超时秒数（默认 300）')
    parser.add_argument('--rviz', action='store_true', help='启用 RViz（默认 headless）')
    parser.add_argument('--list', action='store_true', help='仅列出可用场景')
    args = parser.parse_args()

    scenarios = find_scenarios(args.scenarios)
    if args.list:
        print("可用场景：")
        for s in scenarios:
            print(f"  {s}")
        return 0
    if not scenarios:
        print("[error] no scenarios found", file=sys.stderr)
        return 1

    ts = time.strftime('%Y%m%d_%H%M%S')
    batch_dir = os.path.join(DEFAULT_METRICS_DIR, f'batch_{ts}')
    os.makedirs(batch_dir, exist_ok=True)

    print(f"[batch] scenarios={len(scenarios)} repeats={args.repeats} "
          f"cycles={args.cycles} timeout={args.timeout}s")
    print(f"[batch] output: {batch_dir}")

    results = []
    for sc in scenarios:
        for rep in range(1, args.repeats + 1):
            case_dir, success, kpi_json = run_one_case(
                sc, rep, args.cycles, args.timeout, batch_dir, rviz=args.rviz)
            results.append((os.path.basename(case_dir), success, kpi_json))

    write_summary(batch_dir, results)
    passed = sum(1 for _, s, _ in results if s)
    print(f"[batch] DONE: {passed}/{len(results)} passed")
    return 0 if passed == len(results) else 2


if __name__ == '__main__':
    sys.exit(main())
