#!/usr/bin/env bash
# record_cycle.sh — 按循环分段录制 rosbag（L5 度量层，Phase 5 交付）。
#
# 用法：
#   ./record_cycle.sh [output_dir] [max_cycles]
#
# 录制话题（契约见 docs/sim_arch.md）：
#   /joints_angle /heading2swing_topic /Swing_topic /RealBktPosXYZ
#   /RefDeviceTraj_Unload /RefDeviceTraj_Reset /RefDeviceTraj_Dig
#   /Sys_SeqAction /Sys_SRe_FlagResetExcute /Sys_SeqAction_Dig
#   /Sys_RUn_FlagUnloadExcuteFinish /Sys_RUn_FlagDigFinish
#   /cycle_report /cycle_sequencer/state /plant/active_cmd /plant/settled
#   /truck_center_unloadpoint /truck_center_unloadpoint_2 /truck_center_point
#   /GP_dig_joints /clean_digPoint /bucket_terrain_delta_position
#
# 分段策略：监听 /cycle_report，每收到一条就 split 一次 bag。
# 若 rosbag 不支持外部触发 split，则用 --duration 按固定时长分段。

set -euo pipefail

OUTPUT_DIR="${1:-$(dirname "$(dirname "$(readlink -f "$0")")")/metrics/bags}"
MAX_CYCLES="${2:-0}"
SPLIT_SEC="${SPLIT_SEC:-30}"

mkdir -p "$OUTPUT_DIR"
TS=$(date +%Y%m%d_%H%M%S)
BAG_PREFIX="${OUTPUT_DIR}/cycle_${TS}"

TOPICS=(
  /joints_angle
  /heading2swing_topic
  /Swing_topic
  /RealBktPosXYZ
  /RefDeviceTraj_Unload
  /RefDeviceTraj_Reset
  /RefDeviceTraj_Dig
  /Sys_SeqAction
  /Sys_SRe_FlagResetExcute
  /Sys_SeqAction_Dig
  /Sys_RUn_FlagUnloadExcuteFinish
  /Sys_RUn_FlagDigFinish
  /cycle_report
  /cycle_sequencer/state
  /plant/active_cmd
  /plant/settled
  /truck_center_unloadpoint
  /truck_center_unloadpoint_2
  /truck_center_point
  /GP_dig_joints
  /clean_digPoint
  /bucket_terrain_delta_position
)

echo "[record] output: ${BAG_PREFIX}_*.bag"
echo "[record] split every ${SPLIT_SEC}s (or per /cycle_report)"
echo "[record] topics: ${#TOPICS[@]}"

# 使用 --duration 分段（rosbag record 原生支持）
exec rosbag record \
  --output-prefix "$BAG_PREFIX" \
  --duration "${SPLIT_SEC}" \
  --split \
  "${TOPICS[@]}"
