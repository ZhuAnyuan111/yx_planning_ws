#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[已废弃 - 请使用 exca_plant_sim.py]

本文件保留为向后兼容薄壳。所有功能已迁移至 exca_plant_sim.py（L1 plant）。

行为等价性：默认参数下（tau=0, deadband=0, sigma=0, feedback_delay=0），
exca_plant_sim 与旧 exec_sim 完全一致——纯速率限制跟踪 /RefDeviceTraj_Unload，
发布 /joints_angle 与 /heading2swing_topic。

现有 launch 文件（dump_viz_sim*.launch）无需改动即可继续使用本节点，
但推荐逐步迁移到 exca_plant_sim 以获得多源命令仲裁 / 一阶滞后 / 故障注入等能力。

契约见 docs/sim_arch.md。
"""

import os
import sys

import rospy

# 保证同目录下 exca_plant_sim.py 可导入（rosrun 与直接 python 执行均适用）
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from exca_plant_sim import ExcaPlantSim  # noqa: E402,F401


def main():
    rospy.init_node('exec_sim')
    rospy.logwarn("[exec_sim] DEPRECATED: shim over exca_plant_sim; "
                  "please migrate launch files to use exca_plant_sim directly.")
    ExcaPlantSim()
    rospy.spin()


if __name__ == '__main__':
    main()
