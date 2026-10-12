#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[已废弃 - 请使用 scenario_manager.py]

保留为向后兼容薄壳，行为等价于 scenario_manager.py 的纯参数驱动模式
（即不指定 ~scenario_file 时，全部走 <param .../> 私有参数）。
现有 launch（dump_viz_sim*.launch / dig_viz_sim.launch / reset_viz_sim.launch）
无需改动即可继续使用本节点。

契约见 docs/sim_arch.md。
"""

import os
import sys

import rospy

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from scenario_manager import ScenarioManager  # noqa: E402


def main():
    rospy.init_node('scenario_pub')
    rospy.logwarn("[scenario_pub] DEPRECATED: shim over scenario_manager.py; "
                  "please migrate launch files to use scenario_manager directly.")
    ScenarioManager()
    rospy.spin()


if __name__ == '__main__':
    main()
