#!/usr/bin/env bash
set -euo pipefail

REQ_ENTRY=1.0
ENTERING=2.0
LOADING=3.0
LEAVE_CONFIRMED=4.0
BOOM_UP_THRESHOLD=34.0
TRUCK_COUNT="${TRUCK_COUNT:-0}"
BUCKETS_PER_TRUCK="${BUCKETS_PER_TRUCK:-3}"
RESET_BUCKET_ON_START="${RESET_BUCKET_ON_START:-0}"
TMP_FILES=()

cleanup_tmp_files() {
  if ((${#TMP_FILES[@]} > 0)); then
    rm -f "${TMP_FILES[@]}"
  fi
}

trap cleanup_tmp_files EXIT

step() {
  printf '\n[%s] %s\n' "$(date +'%H:%M:%S')" "$*"
}

pause() {
  sleep "${1:-0.3}"
}

pub_bool() {
  local topic="$1"
  local value="$2"
  rostopic pub -1 "$topic" std_msgs/Bool "data: ${value}"
}

pub_float32() {
  local topic="$1"
  local value="$2"
  rostopic pub -1 "$topic" std_msgs/Float32 "data: ${value}"
}

pub_float64() {
  local topic="$1"
  local value="$2"
  rostopic pub -1 "$topic" std_msgs/Float64 "data: ${value}"
}

pub_pose_stamped() {
  local topic="$1"
  local code="$2"
  rostopic pub -1 "$topic" geometry_msgs/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: ${code}}}}"
}

pub_pose() {
  local topic="$1"
  rostopic pub -1 "$topic" geometry_msgs/Pose \
    "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}}"
}

pub_unman_params() {
  local orientation_w="$1"
  rostopic pub -1 /unman_params_set geometry_msgs/Pose \
    "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: ${orientation_w}}}"
}

pub_bucket_reset_pulse() {
  pub_unman_params 0.0
  pause 0.1
  pub_unman_params 1.0
}

pub_joint_angles() {
  local boom="$1"
  local arm="$2"
  local bucket="$3"
  rostopic pub -1 /joints_angle geometry_msgs/Quaternion \
    "{x: ${boom}, y: ${arm}, z: ${bucket}, w: 0.0}"
}

pub_real_swing() {
  local angle="$1"
  rostopic pub -1 /heading2swing_topic std_msgs/Float32 "data: ${angle}"
}

pub_truck_center_unloadpoint() {
  local x="$1"
  local y="$2"
  rostopic pub -1 /truck_center_unloadpoint geometry_msgs/Quaternion \
    "{x: ${x}, y: ${y}, z: 0.0, w: 0.0}"
}

pub_ref_swing() {
  local angle="$1"
  rostopic pub -1 /Sys_SCo_Traj geometry_msgs/Pose \
    "{position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: ${angle}, y: 0.0, z: 0.0, w: 0.0}}"
}

make_unload_point_yaml() {
  local flag="$1"
  local i
  printf 'layout:\n  dim: []\n  data_offset: 0\ndata: [%s' "$flag"
  for ((i = 0; i < 9999; ++i)); do
    printf ', 238.17001342773438'
  done
  printf ']\n'
}

pub_unload_plan() {
  local topic="$1"
  local flag="$2"
  local yaml_file
  yaml_file="$(mktemp "${TMPDIR:-/tmp}/truck_interaction_unload_plan.XXXXXX.yaml")"
  TMP_FILES+=("${yaml_file}")
  make_unload_point_yaml "$flag" >"${yaml_file}"
  rostopic pub -1 "$topic" std_msgs/Float32MultiArray -f "${yaml_file}"
}

pub_unload_plan_done() {
  pub_unload_plan /Sys_RUn_UnloadPointSequence 1.0
  pause 0.1
  pub_unload_plan /Sys_RUn_UnloadPointSequence 0.0
}

finish_initial_dig_wait() {
  step "INITIAL_WAIT_TRUCK_ENTRY: provide angles and select DigWaitFsm"
  pub_joint_angles "${BOOM_UP_THRESHOLD}" 0.0 0.0
  pub_real_swing 0.0
  pub_ref_swing 0.0
  pause 0.3

  step "INITIAL_WAIT_TRUCK_ENTRY: dig execute feedback"
  pub_pose /RefDeviceTraj_Dig
  pause 0.4
}

finish_reset_dig_wait() {
  step "WAIT_TRUCK_ENTRY: ground plan done"
  pub_float64 /Sys_RGp_FlagGpPlanFinish 1.0
  pause 0.3

  step "WAIT_TRUCK_ENTRY: reset execute feedback"
  pub_pose /RefDeviceTraj_Reset
  pause 0.3

  step "WAIT_TRUCK_ENTRY: dig execute feedback"
  pub_pose /RefDeviceTraj_Dig
  pause 0.3

  step "WAIT_TRUCK_ENTRY: boom up feedback"
  pub_joint_angles "${BOOM_UP_THRESHOLD}" 0.0 0.0
  pub_real_swing 0.0
  pub_ref_swing 0.0
  pause 0.4
}

enter_truck() {
  local truck="$1"
  local wait_mode="$2"

  step "Truck ${truck}: request entry"
  pub_pose_stamped /truck_pose_topic "${REQ_ENTRY}"
  pause 0.3

  if [[ "${wait_mode}" == "initial" ]]; then
    finish_initial_dig_wait
  else
    finish_reset_dig_wait
  fi

  step "Truck ${truck}: entering"
  pub_pose_stamped /truck_pose_topic "${ENTERING}"
  pause 0.3

  step "Truck ${truck}: loading"
  pub_pose_stamped /truck_pose_topic "${LOADING}"
  pause 0.5
}

load_one_bucket() {
  local truck="$1"
  local bucket="$2"

  step "Truck ${truck} bucket ${bucket}: unload execute feedback"
  pub_pose /RefDeviceTraj_Unload
  pause 0.3

  if ((bucket < BUCKETS_PER_TRUCK)); then
    step "Truck ${truck} bucket ${bucket}: ground plan done"
    pub_float64 /Sys_RGp_FlagGpPlanFinish 1.0
    pause 0.2

    step "Truck ${truck} bucket ${bucket}: reset execute feedback"
    pub_pose /RefDeviceTraj_Reset
    pause 0.3

    step "Truck ${truck} bucket ${bucket}: dig execute and unload plan feedback"
    pub_pose /RefDeviceTraj_Dig
    pause 0.2
    pub_unload_plan_done
    pause 0.5
  else
    pause 0.5
  fi
}

load_truck() {
  local truck="$1"
  local bucket

  step "Truck ${truck}: initial unload plan done"
  pub_unload_plan_done
  pause 0.5

  for ((bucket = 1; bucket <= BUCKETS_PER_TRUCK; ++bucket)); do
    load_one_bucket "${truck}" "${bucket}"
  done

  step "Truck ${truck}: smooth finish after ${BUCKETS_PER_TRUCK} buckets"
  pub_bool /FlagSmoothFinish true
  pause 0.5

  step "Truck ${truck}: leave confirmed"
  pub_pose_stamped /truck_pose_topic "${LEAVE_CONFIRMED}"
  pause 0.5

}

main() {
  if ! command -v rostopic >/dev/null 2>&1; then
    echo "rostopic not found. Source a ROS1 environment first."
    exit 1
  fi

  if ((TRUCK_COUNT > 0)); then
    step "Start TRUCK_INTERACTION (${TRUCK_COUNT} trucks, ${BUCKETS_PER_TRUCK} buckets per truck)"
  else
    step "Start TRUCK_INTERACTION (loop forever, ${BUCKETS_PER_TRUCK} buckets per truck)"
  fi

  if [[ "${RESET_BUCKET_ON_START}" == "1" || "${RESET_BUCKET_ON_START}" == "true" ]]; then
    step "Reset bucket number: /unman_params_set orientation.w 0 -> 1"
    pub_bucket_reset_pulse
    pause 0.3
  fi

  pub_bool /Mpc_Switch true
  pause 0.5

  # theta = atan2(-y, -x) = 180 deg. With real swing 0 deg this selects
  # DigWaitFsm, which is the initial-flow branch exercised by this script.
  pub_truck_center_unloadpoint 1.0 0.0
  pause 0.3

  local truck=1
  while ((TRUCK_COUNT <= 0 || truck <= TRUCK_COUNT)); do
    if ((truck == 1)); then
      enter_truck "${truck}" initial
    else
      enter_truck "${truck}" reset
    fi
    load_truck "${truck}"
    ((++truck))
  done

  step "Done"
}

main "$@"
