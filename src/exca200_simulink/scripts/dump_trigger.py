#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[已废弃 - 请使用 phase_trigger.py]

保留为向后兼容薄壳，行为等价于 `phase_trigger.py _phase:=dump`。
现有 launch（dump_viz_sim_single_trigger.launch）无需改动即可继续使用本节点。

契约见 docs/sim_arch.md。
"""

import os
import sys

import rospy

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from phase_trigger import PhaseTrigger  # noqa: E402


def main():
    rospy.init_node('dump_trigger')
    # 强制 phase=dump（除非用户显式覆盖）
    if not rospy.has_param('~phase'):
        rospy.set_param('~phase', 'dump')
    rospy.logwarn("[dump_trigger] DEPRECATED: shim over phase_trigger.py; "
                  "please migrate launch files to use phase_trigger directly.")
    PhaseTrigger()
    rospy.spin()


if __name__ == '__main__':
    main()
