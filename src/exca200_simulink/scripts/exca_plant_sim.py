#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""挖掘机被控对象仿真器（L1 plant）。

由 exec_sim.py 泛化而来，向后兼容其全部行为。契约见 docs/sim_arch.md §6。

功能
----
- 多源命令仲裁：订阅 /RefDeviceTraj_{Unload,Reset,Dig,Dump}，按 cmd_priority 与
  "最近有效"窗口选出当前活动源。
- 关节模型（每关节独立参数）：
    cmd ──[deadband]──[一阶滞后 tau]──[速率限幅 rate]──▶ actual
                                                     │
                                        [feedback delay + noise]
                                                     ▼
                                                publish
- 反馈发布：
    /joints_angle          (Quaternion, x=boom y=arm z=bucket, deg)
    /heading2swing_topic   (Float32, deg, [0,360))
    /Swing_topic           (Float32, deg, 与 heading2swing 同值，兼容 reset 命名)
    /plant/settled         (Bool, 跟踪误差 < settle_tol 且持续 settle_dur)
    /plant/active_cmd      (String, 当前活动命令源名)
  注：/RealBktPosXYZ 由 dump_trajectory_planner 发布（其精确运动学），plant 不重复发布。
- 故障注入（Phase 6 使用，通过 /plant/fault service 动态开关）：
    joint_stuck / sensor_dropout / latency_spike / cmd_freeze

参数
----
所有参数支持三种来源，优先级从高到低：
  1) 节点私有参数（~rate_boom_dps 等，兼容旧 launch 的 <param .../> 注入）
  2) YAML 文件（~plant_config 指定路径）
  3) 内置默认值（与旧 exec_sim 一致）

本节点不修改任何 planner。
"""

import collections
import copy
import math
import random
import threading

import rospy
import yaml
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, SetBoolResponse


D2R = math.pi / 180.0

# "无限期"故障的哨兵时长（100 年）——用 now + 该时长表示"直到手动关闭"，
# 不能用固定 rospy.Time(常量)，否则常量若落在过去会导致故障判定恒为假。
FOREVER_SEC = 86400 * 365 * 100

# 关节名统一顺序
JOINTS = ('swing', 'boom', 'arm', 'bucket')

# 命令源 → 优先级默认值（数字越小优先级越高）
DEFAULT_CMD_PRIORITY = {
    'RefDeviceTraj_Unload': 10,
    'RefDeviceTraj_Reset': 20,
    'RefDeviceTraj_Dig': 30,
    'RefDeviceTraj_Dump': 90,
}


def wrap180(deg):
    """角度差归一化到 [-180, 180)。"""
    return (deg + 180.0) % 360.0 - 180.0


def wrap360(deg):
    """角度归一化到 [0, 360)。"""
    return deg % 360.0


def _default_params():
    """内置默认值：与旧 exec_sim.py 完全一致，保证向后兼容。"""
    p = {
        'publish_rate_hz': 50.0,
        # 初始位姿（挖掘终止姿态）
        'init_swing_deg': 180.0,
        'init_boom_deg': 5.0,
        'init_arm_deg': -110.0,
        'init_bucket_deg': -110.0,
        # 完成判定
        'settle_tol_deg': 3.0,
        'settle_dur_sec': 0.5,
        # 命令源新鲜度窗口
        'cmd_timeout_sec': 0.5,
    }
    for j in JOINTS:
        p[f'rate_{j}_dps'] = {'swing': 30.0, 'boom': 15.0,
                              'arm': 20.0, 'bucket': 40.0}[j]
        p[f'tau_{j}_s'] = 0.0
        p[f'deadband_{j}_deg'] = 0.0
        p[f'sigma_{j}_deg'] = 0.0
    p['feedback_delay_ms'] = 0.0
    p['cmd_priority'] = dict(DEFAULT_CMD_PRIORITY)
    # 故障注入默认全关
    p['faults'] = {
        'joint_stuck': {'enabled': False, 'joint': 'boom', 'duration_sec': 0.0},
        'sensor_dropout': {'enabled': False, 'drop_prob': 0.0},
        'latency_spike': {'enabled': False, 'extra_delay_ms': 0.0},
        'cmd_freeze': {'enabled': False, 'duration_sec': 0.0},
    }
    return p


def _flatten(d, prefix=''):
    """把嵌套 dict 平铺成 a.b.c 形式，便于与私有参数合并。"""
    out = {}
    for k, v in d.items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(_flatten(v, prefix=key + '.'))
        else:
            out[key] = v
    return out


def _unflatten(flat):
    """把 a.b.c 形式的平铺 dict 还原成嵌套。"""
    out = {}
    for k, v in flat.items():
        parts = k.split('.')
        cur = out
        for p in parts[:-1]:
            cur = cur.setdefault(p, {})
        cur[parts[-1]] = v
    return out


class CmdSource:
    """单个命令源的运行时状态。"""

    __slots__ = ('name', 'priority', 'topic', 'cmd', 'last_rx', 'executing', 'sub')

    def __init__(self, name, priority, topic):
        self.name = name
        self.priority = priority
        self.topic = topic
        self.cmd = {j: 0.0 for j in JOINTS}
        self.last_rx = None  # rospy.Time
        self.executing = False
        self.sub = None


class ExcaPlantSim:
    """挖掘机被控对象仿真器主类。"""

    def __init__(self):
        # ---- 参数加载：内置默认 → YAML → 私有参数（后者覆盖前者）----
        self.p = _default_params()
        cfg_path = rospy.get_param('~plant_config', '')
        if cfg_path:
            try:
                with open(cfg_path, 'r') as f:
                    yaml_cfg = yaml.safe_load(f) or {}
                self._merge_dict(self.p, yaml_cfg)
                rospy.loginfo("[plant] loaded config: %s", cfg_path)
            except OSError as e:
                rospy.logwarn("[plant] cannot read %s: %s", cfg_path, e)
        # 私有参数覆盖（兼容旧 launch 的 <param name="rate_boom_dps" .../> 注入）
        flat = _flatten(self.p)
        for k in list(flat.keys()):
            if rospy.has_param('~' + k):
                flat[k] = rospy.get_param('~' + k)
        # 单独处理 init_* / rate_* / tau_* / deadband_* / sigma_* 这些平铺 key
        for j in JOINTS:
            for prefix in ('rate_', 'tau_', 'deadband_', 'sigma_'):
                suffix = {'rate_': '_dps', 'tau_': '_s',
                          'deadband_': '_deg', 'sigma_': '_deg'}[prefix]
                key = f'{prefix}{j}{suffix}'
                if rospy.has_param('~' + key):
                    flat[key] = rospy.get_param('~' + key)
        for k in ('publish_rate_hz', 'feedback_delay_ms', 'cmd_timeout_sec',
                  'settle_tol_deg', 'settle_dur_sec',
                  'init_swing_deg', 'init_boom_deg',
                  'init_arm_deg', 'init_bucket_deg'):
            if rospy.has_param('~' + k):
                flat[k] = rospy.get_param('~' + k)
        self.p = _unflatten(flat)

        # ---- 命令源 ----
        self.sources = {}
        for name, prio in self.p['cmd_priority'].items():
            self.sources[name] = CmdSource(name, prio, '/' + name)
        self.active_src = None  # 当前活动源名

        # ---- 关节状态 ----
        self.actual = {
            'swing': self.p['init_swing_deg'],
            'boom': self.p['init_boom_deg'],
            'arm': self.p['init_arm_deg'],
            'bucket': self.p['init_bucket_deg'],
        }
        # 一阶滞后内部状态（滞后后的"目标"，再由速率限幅逼近）
        self.lag_state = dict(self.actual)
        # swing 内部连续（不 wrap），仅在发布时 wrap360
        self.swing_cont = self.actual['swing']
        self.last_cmd = dict(self.actual)
        # swing 一阶滞后内部状态（对最短角差 offset 做滞后）
        self._swing_lag_offset = 0.0

        # ---- 反馈延迟环形缓冲 ----
        delay_ms = float(self.p['feedback_delay_ms'])
        rate_hz = float(self.p['publish_rate_hz'])
        buf_len = max(1, int(round(delay_ms * 1e-3 * rate_hz)) + 1)
        self.fb_buf = collections.deque(maxlen=buf_len)
        # 预填初始状态，避免启动瞬间输出 0
        init_snap = dict(self.actual)
        for _ in range(buf_len):
            self.fb_buf.append(init_snap)

        # ---- 完成判定 ----
        self.settle_since = None  # rospy.Time
        self.is_settled = False

        # ---- 故障注入运行时 ----
        self.fault_lock = threading.Lock()
        self.fault_state = {
            'joint_stuck_until': None,
            'cmd_freeze_until': None,
            'latency_spike_extra_ms': 0.0,
            'sensor_drop_prob': 0.0,
        }

        # ---- 时钟 ----
        self.last_step = rospy.Time.now()

        # ---- 订阅 ----
        for src in self.sources.values():
            src.sub = rospy.Subscriber(src.topic, Pose,
                                       self._make_cmd_cb(src.name),
                                       queue_size=1)

        # ---- 发布 ----
        self.pub_joints = rospy.Publisher('/joints_angle', Quaternion, queue_size=1)
        self.pub_swing = rospy.Publisher('/heading2swing_topic', Float32, queue_size=1)
        self.pub_swing_alt = rospy.Publisher('/Swing_topic', Float32, queue_size=1)
        self.pub_settled = rospy.Publisher('/plant/settled', Bool, queue_size=1, latch=True)
        self.pub_active = rospy.Publisher('/plant/active_cmd', String, queue_size=1, latch=True)

        # ---- 服务：故障注入 ----
        self.srv_fault = rospy.Service('/plant/fault', SetBool, self._fault_srv)

        # ---- 主循环 ----
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self._step)

        rospy.loginfo(
            "[plant] ready: init sw/bm/arm/bkt=%.1f/%.1f/%.1f/%.1f deg, "
            "rate=%.1f/%.1f/%.1f/%.1f dps, tau=%.2f/%.2f/%.2f/%.2f s, "
            "delay=%.0fms, sources=%s",
            self.actual['swing'], self.actual['boom'],
            self.actual['arm'], self.actual['bucket'],
            self.p['rate_swing_dps'], self.p['rate_boom_dps'],
            self.p['rate_arm_dps'], self.p['rate_bucket_dps'],
            self.p['tau_swing_s'], self.p['tau_boom_s'],
            self.p['tau_arm_s'], self.p['tau_bucket_s'],
            self.p['feedback_delay_ms'],
            sorted(self.sources.keys()),
        )

    # ---------- 参数工具 ----------
    @staticmethod
    def _merge_dict(dst, src):
        for k, v in src.items():
            if isinstance(v, dict) and isinstance(dst.get(k), dict):
                ExcaPlantSim._merge_dict(dst[k], v)
            else:
                dst[k] = v

    # ---------- 命令回调 ----------
    def _make_cmd_cb(self, name):
        def cb(msg):
            src = self.sources[name]
            # orientation.x/y/z/w = swing/boom/arm/bucket (deg)
            src.cmd['swing'] = msg.orientation.x
            src.cmd['boom'] = msg.orientation.y
            src.cmd['arm'] = msg.orientation.z
            src.cmd['bucket'] = msg.orientation.w
            src.last_rx = rospy.Time.now()
            # position.x = 1.0 执行中 / 0.0 阶段结束
            src.executing = bool(msg.position.x >= 0.5)
        return cb

    # ---------- 主循环 ----------
    def _step(self, _event):
        now = rospy.Time.now()
        dt = (now - self.last_step).to_sec()
        self.last_step = now
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / float(self.p['publish_rate_hz'])

        # 1) 命令源仲裁
        active = self._select_active_cmd(now)
        if active is not None and self.active_src != active.name:
            self.active_src = active.name
            self.pub_active.publish(String(active.name))

        # 2) cmd_freeze 故障：忽略命令
        with self.fault_lock:
            frozen = (self.fault_state['cmd_freeze_until'] is not None
                      and now < self.fault_state['cmd_freeze_until'])
        if active is None or frozen:
            target = dict(self.last_cmd)  # 保持上一次目标
        else:
            target = dict(active.cmd)

        # 3) 每关节：deadband → 一阶滞后 → 速率限幅
        for j in ('boom', 'arm', 'bucket'):
            self._advance_joint(j, target[j], dt)
        self._advance_swing(target['swing'], dt, now)

        self.last_cmd = dict(target)

        # 4) 反馈快照入队（用于延迟回放）
        snap = dict(self.actual)
        snap['swing_pub'] = wrap360(self.swing_cont)
        self.fb_buf.append(snap)

        # 5) 取延迟后的快照发布（含噪声 / dropout）
        self._publish_feedback(now)

        # 6) 完成判定
        self._update_settled(target, now)

    def _advance_joint(self, j, cmd_deg, dt):
        """boom/arm/bucket 通用推进：deadband → 一阶滞后 → 速率限幅。"""
        # deadband
        err = cmd_deg - self.lag_state[j]
        deadband = float(self.p[f'deadband_{j}_deg'])
        if abs(err) < deadband:
            lag_target = self.lag_state[j]
        else:
            lag_target = cmd_deg
        # 一阶滞后：state += (target - state) * dt / tau
        tau = float(self.p[f'tau_{j}_s'])
        if tau > 1e-6:
            alpha = min(1.0, dt / tau)
            self.lag_state[j] += (lag_target - self.lag_state[j]) * alpha
        else:
            self.lag_state[j] = lag_target
        # joint_stuck 故障
        with self.fault_lock:
            stuck = (self.fault_state['joint_stuck_until'] is not None
                     and rospy.Time.now() < self.fault_state['joint_stuck_until']
                     and self.p['faults']['joint_stuck'].get('joint') == j)
        if stuck:
            return
        # 速率限幅
        diff = self.lag_state[j] - self.actual[j]
        max_step = float(self.p[f'rate_{j}_dps']) * dt
        self.actual[j] += max(-max_step, min(max_step, diff))

    def _advance_swing(self, cmd_deg, dt, now):
        """swing 特殊处理：按最短角差逼近，内部连续，发布 wrap360。"""
        err = wrap180(cmd_deg - wrap360(self.swing_cont))
        deadband = float(self.p['deadband_swing_deg'])
        if abs(err) < deadband:
            lag_target_offset = 0.0
        else:
            lag_target_offset = err
        tau = float(self.p['tau_swing_s'])
        if tau > 1e-6:
            alpha = min(1.0, dt / tau)
            # 对 offset 做一阶滞后（等价于对连续角做滞后）
            self._swing_lag_offset += (lag_target_offset - self._swing_lag_offset) * alpha
            eff_offset = self._swing_lag_offset
        else:
            eff_offset = lag_target_offset
            self._swing_lag_offset = 0.0
        with self.fault_lock:
            stuck = (self.fault_state['joint_stuck_until'] is not None
                     and now < self.fault_state['joint_stuck_until']
                     and self.p['faults']['joint_stuck'].get('joint') == 'swing')
        if stuck:
            return
        max_step = float(self.p['rate_swing_dps']) * dt
        self.swing_cont += max(-max_step, min(max_step, eff_offset))
        self.actual['swing'] = wrap360(self.swing_cont)

    def _select_active_cmd(self, now):
        """在 cmd_timeout_sec 窗口内、executing=True 的源中按优先级选一个。"""
        timeout = float(self.p['cmd_timeout_sec'])
        candidates = []
        for src in self.sources.values():
            if src.last_rx is None:
                continue
            if (now - src.last_rx).to_sec() > timeout:
                continue
            if not src.executing:
                continue
            candidates.append(src)
        if not candidates:
            # 无活动源时，若最近有非 executing 的源，保持其最后 cmd（避免漂移）
            recent = [s for s in self.sources.values()
                      if s.last_rx is not None
                      and (now - s.last_rx).to_sec() <= timeout]
            if recent:
                recent.sort(key=lambda s: s.priority)
                return recent[0]
            return None
        candidates.sort(key=lambda s: s.priority)
        return candidates[0]

    def _publish_feedback(self, now):
        """从延迟缓冲取一帧，加噪声，可能 dropout，然后发布。"""
        snap = self.fb_buf[0]  # 队首 = 最老 = 延迟后的当前帧
        # 噪声
        noisy = {}
        for j in JOINTS:
            sigma = float(self.p[f'sigma_{j}_deg'])
            noisy[j] = snap[j] + (random.gauss(0.0, sigma) if sigma > 0 else 0.0)
        swing_pub = snap.get('swing_pub', wrap360(noisy['swing']))
        if float(self.p['sigma_swing_deg']) > 0:
            swing_pub = wrap360(swing_pub + random.gauss(0.0, float(self.p['sigma_swing_deg'])))

        # latency_spike：额外延迟表现为"再多回放一帧"——简化为丢一次发布
        # sensor_dropout
        with self.fault_lock:
            drop_p = self.fault_state['sensor_drop_prob']
        if drop_p > 0.0 and random.random() < drop_p:
            return

        # /joints_angle
        q = Quaternion()
        q.x = noisy['boom']
        q.y = noisy['arm']
        q.z = noisy['bucket']
        q.w = 0.0
        self.pub_joints.publish(q)
        # /heading2swing_topic + /Swing_topic
        self.pub_swing.publish(Float32(swing_pub))
        self.pub_swing_alt.publish(Float32(swing_pub))
        # 注：/RealBktPosXYZ 由 dump_trajectory_planner 的 status_reporter 用其精确
        # 运动学发布（reset planner 订阅它做"物理提前结束"判定）。plant 不再重复
        # 发布，避免双发布者冲突；standalone reset/dig 无该话题时，reset planner 会
        # 自动禁用提前结束、退回回转到位判定（见其 IsPhysicalGoalReached）。

    def _update_settled(self, target, now):
        """跟踪误差 < settle_tol 且持续 settle_dur → settled=True。"""
        tol = float(self.p['settle_tol_deg'])
        dur = float(self.p['settle_dur_sec'])
        err_max = max(abs(wrap180(target[j] - self.actual[j])) for j in JOINTS)
        if err_max < tol:
            if self.settle_since is None:
                self.settle_since = now
            elif not self.is_settled and (now - self.settle_since).to_sec() >= dur:
                self.is_settled = True
                self.pub_settled.publish(Bool(True))
        else:
            self.settle_since = None
            if self.is_settled:
                self.is_settled = False
                self.pub_settled.publish(Bool(False))

    # ---------- 故障注入服务 ----------
    def _live_faults(self):
        """返回当前生效的 faults 配置：内置默认 ← 启动配置 ← 运行时 ~faults 参数。

        这样可以在不重启节点的情况下，通过
        `rosparam set /<node>/faults/joint_stuck/enabled true` 或
        launch 私有参数 `_faults/joint_stuck/enabled:=true` 动态选择故障，
        再由 /plant/fault(True) 触发。
        """
        base = copy.deepcopy(_default_params()['faults'])
        self._merge_dict(base, self.p.get('faults', {}))
        live = rospy.get_param('~faults', {})
        if isinstance(live, dict):
            self._merge_dict(base, live)
        return base

    def _fault_srv(self, req):
        """SetBool：True=按当前生效的 faults 配置启用，False=全部关闭。

        故障类型与参数（joint/duration/drop_prob/...）来自 `~faults`（可在运行时
        用 rosparam set 或 launch 私有参数覆盖），本服务负责动态开/关。
        """
        now = rospy.Time.now()
        with self.fault_lock:
            if req.data:
                f = self._live_faults()
                self.p['faults'] = f  # 同步快照，供 _step 读取 joint 名等
                if f['joint_stuck']['enabled']:
                    d = float(f['joint_stuck']['duration_sec'])
                    self.fault_state['joint_stuck_until'] = (
                        now + rospy.Duration(d) if d > 0 else now + rospy.Duration(FOREVER_SEC))
                if f['sensor_dropout']['enabled']:
                    self.fault_state['sensor_drop_prob'] = float(f['sensor_dropout']['drop_prob'])
                if f['latency_spike']['enabled']:
                    self.fault_state['latency_spike_extra_ms'] = float(
                        f['latency_spike']['extra_delay_ms'])
                if f['cmd_freeze']['enabled']:
                    d = float(f['cmd_freeze']['duration_sec'])
                    self.fault_state['cmd_freeze_until'] = (
                        now + rospy.Duration(d) if d > 0 else now + rospy.Duration(FOREVER_SEC))
                msg = 'faults enabled'
            else:
                self.fault_state['joint_stuck_until'] = None
                self.fault_state['cmd_freeze_until'] = None
                self.fault_state['latency_spike_extra_ms'] = 0.0
                self.fault_state['sensor_drop_prob'] = 0.0
                msg = 'faults cleared'
        rospy.loginfo("[plant] fault service: %s", msg)
        return SetBoolResponse(success=True, message=msg)


def main():
    rospy.init_node('exca_plant_sim')
    ExcaPlantSim()
    rospy.spin()


if __name__ == '__main__':
    main()
