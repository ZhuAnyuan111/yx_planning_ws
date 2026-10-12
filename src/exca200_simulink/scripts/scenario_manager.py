#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景管理器（L2 场景层，Phase 3 交付，YAML 驱动）。

由 scenario_pub.py 泛化而来：
- 从 YAML 文件加载全部场景参数（卡车 / 挖掘目标 / 地形 / 激活 / 扰动）
- 提供 ROS service 热切换场景（/scenario/reload, /scenario/select）
- 支持扰动注入（卡车位置抖动 / RTK 偏置 / 装载斗数抖动）
- 向后兼容 scenario_pub.py 的全部私有参数（当 ~scenario_file 未指定时，
  行为退化为纯参数驱动，等价旧 scenario_pub）

话题契约见 docs/sim_arch.md §3.2 / §3.4。

用法
----
1) YAML 驱动（推荐）：
     rosrun exca200_simulink scenario_manager.py \
       _scenario_file:=$(rospack find exca200_simulink)/scenarios/baseline.yaml
2) 纯参数驱动（兼容旧 launch）：
     <node name="scenario_pub" pkg="exca200_simulink" type="scenario_manager.py">
       <param name="truck_near_x" value="6.0"/>
       ...
     </node>
3) 运行时热切换：
     rosservice call /scenario/select "name: 'tight_truck'"
     rosservice call /scenario/reload "{}"
"""

import os
import random
import threading

import rospy
import yaml
from geometry_msgs.msg import PointStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Header, String
from std_srvs.srv import Trigger, TriggerResponse


# 默认场景（与 config/scenario_default.yaml 一致）
_DEFAULT_SCENARIO = {
    'truck': {
        'near_x': -6.0, 'near_y': 1.5, 'near_z': 0.0,
        'far_x': -8.0, 'far_y': 2.0, 'heading_deg': 20.0,
    },
    'bucket_number': 2.0,
    'gp_dig_joints': {'swing_deg': 180.0, 'boom_deg': 5.0,
                      'arm_deg': -110.0, 'bucket_deg': -110.0},
    'clean_dig_joints': {'swing_deg': 180.0, 'boom_deg': 5.0,
                         'arm_deg': -110.0, 'bucket_deg': -110.0},
    'bucket_terrain_z': -1.0,
    'terrain_profile': {'points': []},
    'activations': {
        'publish_rate_hz': 1.0,
        'auto_activate': True,
        'activate_delay_sec': 2.0,
        'loop_activate': False,
        'dump_activate_value': 4.0,
        'reset_activate_value': 1.0,
        'dig_activate_value': 3.0,
    },
    'perturbations': {
        'truck_pos_noise_sigma_m': 0.0,
        'truck_heading_noise_sigma_deg': 0.0,
        'rtk_bias_deg': 0.0,
        'bucket_number_jitter': 0,
    },
}


def _deep_merge(dst, src):
    """递归合并 src 到 dst（src 覆盖 dst）。"""
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def _load_yaml(path):
    if not path:
        return {}
    if not os.path.isfile(path):
        rospy.logwarn("[scenario] file not found: %s", path)
        return {}
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        rospy.logerr("[scenario] failed to load %s: %s", path, e)
        return {}


class ScenarioManager:
    """YAML 驱动的场景管理器。"""

    def __init__(self):
        self.lock = threading.RLock()
        self.scenario_file = rospy.get_param('~scenario_file', '')
        self.publish_bucket_number = rospy.get_param('~publish_bucket_number', True)
        self.scenario_dir = rospy.get_param(
            '~scenario_dir',
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'scenarios'))
        self.current_name = '(default)'
        self.scenario = self._build_scenario()

        # 激活状态
        self.start_time = rospy.Time.now()
        self.activated = {'dump': False, 'reset': False, 'dig': False}

        # 发布者
        self.pub_near = rospy.Publisher('/truck_center_unloadpoint', Quaternion,
                                        queue_size=1, latch=True)
        self.pub_far = rospy.Publisher('/truck_center_unloadpoint_2', Quaternion,
                                       queue_size=1, latch=True)
        self.pub_truck_center = rospy.Publisher('/truck_center_point', Quaternion,
                                                queue_size=1, latch=True)
        self.pub_bucket_num = rospy.Publisher('/Sys_SUn_BucketNumber', Float64,
                                              queue_size=1, latch=True)
        self.pub_gp = rospy.Publisher('/GP_dig_joints', Quaternion,
                                      queue_size=1, latch=True)
        self.pub_clean = rospy.Publisher('/clean_digPoint', Quaternion,
                                         queue_size=1, latch=True)
        self.pub_terrain_z = rospy.Publisher('/bucket_terrain_delta_position',
                                             PointStamped, queue_size=1, latch=True)
        self.pub_terrain_path = rospy.Publisher('/terrain_profile', Path,
                                                queue_size=1, latch=True)
        # 激活话题
        self.pub_seq_dump = rospy.Publisher('/Sys_SeqAction', Float64,
                                            queue_size=1, latch=True)
        self.pub_seq_reset = rospy.Publisher('/Sys_SRe_FlagResetExcute', Float64,
                                             queue_size=1, latch=True)
        self.pub_seq_dig = rospy.Publisher('/Sys_SeqAction_Dig', Float64,
                                           queue_size=1, latch=True)
        # 当前场景名（供 sequencer / kpi 查询）
        self.pub_name = rospy.Publisher('/scenario/current', String,
                                        queue_size=1, latch=True)

        # 服务
        rospy.Service('/scenario/reload', Trigger, self._srv_reload)
        # /scenario/select 用 Trigger + 参数方式简化（避免自定义 srv 依赖）
        rospy.Service('/scenario/select', Trigger, self._srv_select)

        # 主循环
        rate_hz = float(self.scenario['activations'].get('publish_rate_hz', 1.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate_hz), self._tick)

        self.pub_name.publish(String(self.current_name))
        rospy.loginfo("[scenario] loaded '%s' from %s; truck=(%.2f,%.2f) heading=%.1f "
                      "bucket=%d auto_activate=%s",
                      self.current_name, self.scenario_file or '(params)',
                      self.scenario['truck']['near_x'],
                      self.scenario['truck']['near_y'],
                      self.scenario['truck']['heading_deg'],
                      int(self.scenario['bucket_number']),
                      self.scenario['activations']['auto_activate'])

    # ---------- 场景构建 ----------
    def _build_scenario(self):
        """合并：内置默认 → YAML 文件 → 私有参数（向后兼容旧 launch）。"""
        sc = {}
        _deep_merge(sc, _DEFAULT_SCENARIO)

        # 1) YAML 文件
        if self.scenario_file:
            _deep_merge(sc, _load_yaml(self.scenario_file))
            self.current_name = os.path.splitext(
                os.path.basename(self.scenario_file))[0]

        # 2) 私有参数覆盖（兼容旧 scenario_pub 的 <param name="truck_near_x" .../>）
        self._apply_private_params(sc)
        return sc

    def _apply_private_params(self, sc):
        """把旧 scenario_pub 风格的私有参数映射到新结构。"""
        mapping = {
            'truck_near_x': ('truck', 'near_x'),
            'truck_near_y': ('truck', 'near_y'),
            'truck_near_z': ('truck', 'near_z'),
            'truck_far_x': ('truck', 'far_x'),
            'truck_far_y': ('truck', 'far_y'),
            'truck_heading_deg': ('truck', 'heading_deg'),
            'bucket_number': ('bucket_number',),
            'bucket_terrain_z': ('bucket_terrain_z',),
            'gp_swing_deg': ('gp_dig_joints', 'swing_deg'),
            'gp_boom_deg': ('gp_dig_joints', 'boom_deg'),
            'gp_arm_deg': ('gp_dig_joints', 'arm_deg'),
            'gp_bucket_deg': ('gp_dig_joints', 'bucket_deg'),
            'clean_swing_deg': ('clean_dig_joints', 'swing_deg'),
            'clean_boom_deg': ('clean_dig_joints', 'boom_deg'),
            'clean_arm_deg': ('clean_dig_joints', 'arm_deg'),
            'clean_bucket_deg': ('clean_dig_joints', 'bucket_deg'),
            'auto_activate': ('activations', 'auto_activate'),
            'activate_delay': ('activations', 'activate_delay_sec'),
            'loop_activate': ('activations', 'loop_activate'),
            'publish_rate': ('activations', 'publish_rate_hz'),
        }
        for pname, path in mapping.items():
            if rospy.has_param('~' + pname):
                val = rospy.get_param('~' + pname)
                cur = sc
                for k in path[:-1]:
                    cur = cur.setdefault(k, {})
                cur[path[-1]] = val

    # ---------- 服务 ----------
    def _srv_reload(self, _req):
        with self.lock:
            self.scenario = self._build_scenario()
            self.start_time = rospy.Time.now()
            self.activated = {k: False for k in self.activated}
            self.pub_name.publish(String(self.current_name))
        rospy.loginfo("[scenario] reloaded '%s'", self.current_name)
        return TriggerResponse(success=True, message=f"reloaded {self.current_name}")

    def _srv_select(self, _req):
        """通过 ~select_name 参数指定要切换的场景名（避免自定义 srv）。

        用法：
          rosparam set /scenario_manager/select_name tight_truck
          rosservice call /scenario/select "{}"
        """
        name = rospy.get_param('~select_name', '')
        if not name:
            return TriggerResponse(success=False,
                                   message="set ~select_name first (e.g. rosparam set .../select_name tight_truck)")
        path = os.path.join(self.scenario_dir, f"{name}.yaml")
        if not os.path.isfile(path):
            return TriggerResponse(success=False, message=f"scenario not found: {path}")
        with self.lock:
            self.scenario_file = path
            self.scenario = self._build_scenario()
            self.start_time = rospy.Time.now()
            self.activated = {k: False for k in self.activated}
            self.pub_name.publish(String(self.current_name))
        rospy.loginfo("[scenario] selected '%s' -> %s", name, path)
        return TriggerResponse(success=True, message=f"selected {name}")

    # ---------- 主循环 ----------
    def _tick(self, _event):
        with self.lock:
            sc = self.scenario
        truck = sc['truck']
        pert = sc['perturbations']
        act = sc['activations']

        # 扰动
        nx = truck['near_x'] + random.gauss(0, pert['truck_pos_noise_sigma_m']) \
            if pert['truck_pos_noise_sigma_m'] > 0 else truck['near_x']
        ny = truck['near_y'] + random.gauss(0, pert['truck_pos_noise_sigma_m']) \
            if pert['truck_pos_noise_sigma_m'] > 0 else truck['near_y']
        heading = truck['heading_deg'] + pert['rtk_bias_deg']
        if pert['truck_heading_noise_sigma_deg'] > 0:
            heading += random.gauss(0, pert['truck_heading_noise_sigma_deg'])
        fx = truck['far_x'] + random.gauss(0, pert['truck_pos_noise_sigma_m']) \
            if pert['truck_pos_noise_sigma_m'] > 0 else truck['far_x']
        fy = truck['far_y'] + random.gauss(0, pert['truck_pos_noise_sigma_m']) \
            if pert['truck_pos_noise_sigma_m'] > 0 else truck['far_y']
        bkt_num = sc['bucket_number']
        jitter = int(pert.get('bucket_number_jitter', 0))
        if jitter > 0:
            bkt_num = max(0, bkt_num + random.randint(-jitter, jitter))

        # 卡车近点 / 远点 / 中心
        qn = Quaternion(); qn.x, qn.y, qn.z, qn.w = nx, ny, truck['near_z'], heading
        self.pub_near.publish(qn)
        qf = Quaternion(); qf.x, qf.y, qf.z, qf.w = fx, fy, 0.0, heading
        self.pub_far.publish(qf)
        qc = Quaternion(); qc.x, qc.y, qc.z, qc.w = nx, ny, truck['near_z'], heading
        self.pub_truck_center.publish(qc)
        if self.publish_bucket_number:
            self.pub_bucket_num.publish(Float64(bkt_num))

        # 挖掘目标关节角
        gp = sc['gp_dig_joints']
        qg = Quaternion()
        qg.x, qg.y, qg.z, qg.w = gp['swing_deg'], gp['boom_deg'], gp['arm_deg'], gp['bucket_deg']
        self.pub_gp.publish(qg)

        cl = sc['clean_dig_joints']
        qcl = Quaternion()
        qcl.x, qcl.y, qcl.z, qcl.w = cl['swing_deg'], cl['boom_deg'], cl['arm_deg'], cl['bucket_deg']
        self.pub_clean.publish(qcl)

        # 地形高度桩
        ps = PointStamped()
        ps.header = Header(stamp=rospy.Time.now(), frame_id='base')
        ps.point.z = sc['bucket_terrain_z']
        self.pub_terrain_z.publish(ps)

        # 地形剖面
        path = Path()
        path.header = Header(stamp=rospy.Time.now(), frame_id='base')
        # terrain_profile.points 为 [[x, z], ...] 列表（桩可为空）
        from geometry_msgs.msg import PoseStamped
        for pt in sc['terrain_profile'].get('points', []) or []:
            ps2 = PoseStamped()
            ps2.header = path.header
            ps2.pose.position.x = float(pt[0])
            ps2.pose.position.z = float(pt[1])
            path.poses.append(ps2)
        self.pub_terrain_path.publish(path)

        # 激活：延时后按配置触发（默认只触发 dump，向后兼容旧行为）
        if not act['auto_activate']:
            return
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        if elapsed < act['activate_delay_sec']:
            return
        loop = act['loop_activate']
        if loop or not self.activated['dump']:
            self.pub_seq_dump.publish(Float64(act['dump_activate_value']))
            self.activated['dump'] = True


def main():
    rospy.init_node('scenario_manager')
    ScenarioManager()
    rospy.spin()


if __name__ == '__main__':
    main()
