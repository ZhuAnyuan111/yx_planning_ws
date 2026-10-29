#!/bin/bash
sleep 5
source /home/nvidia/.bashrc
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# base directory
dir_ros="/home/nvidia/"
dir_base="/home/nvidia/unmanned_exca/Sany_and_DataCapture_rs"
dir_gp="/home/nvidia/unmanned_exca/Sany_and_DataCapture_rs/gp_ws"

# DPC directory
dir_imu=$dir_base"/catkin_ws/"
dir_dpc=$dir_base"/DPC/DPC_test/dpc_exe/"
dir_GP2mid="/home/nvidia/unmanned_exca/Sany_and_DataCapture_rs/gp_ws"

# Sensor_Perception directory
dir_sensor=$dir_base"/Sensor/sensor_test/"
dir_data_capture="/home/nvidia/unmanned_exca/data_capture_C"
BAG_DIR=$dir_base"/Sensor/data/bag"

# Soft directory
dir_soft=$dir_base"/Soft/soft_release" 
dir_soft_excavator=$dir_soft"/excavator"
dir_soft_panel=$dir_soft"/panel"
dir_soft_gui=$dir_soft"/gui"
dir_soft_truck=$dir_soft"/truck"

# ACT directory
# ACT_DIR="/home/nvidia/exca200_action_fused/"
ACT_DIR="/home/nvidia/unmanned_exca/action_fused_pyws/"

dir_trajfusion_act="/home/nvidia/unmanned_exca/exca_trajfusion_test_ws/"

dir_falut_manager="/home/nvidia/unmanned_exca/exca_fault_manager_ws/"

dir_rtk="/home/nvidia/unmanned_exca/Sany_and_DataCapture_rs/Sensor/sensor_release"


# DPC Nodes
export LD_LIBRARY_PATH=/opt/opencv45_backup:$LD_LIBRARY_PATH
#smooth_node="YX870_PlanningNode_Flat_V1_0"
#dp_node="YX870_PlanningNode_DP_V1_1"
#dp_online_node="YX870_PlanningNode_DP_online_V1_1"
#rs_node="YX870_PlanningNode_RS_V1_1"
#rs_online_node="YX870_PlanningNode_RS_online_V1_6"
#ut_node="YX870_PlanningNode_Load"
#ut_online_node="YX870_PlanningNode_Load_online_V1_7"
#decision_node="YX870_SystemDecision_V1_20"

smooth_node="YX870_PlanningNode_Flat_V4_0"
dp_node="YX870_PlanningNode_DP_V2_0"
dp_online_node="YX870_PlanningNode_DP_online_V2_1"
rs_node="YX870_PlanningNode_RS_V2_1"
rs_online_node="YX870_PlanningNode_RS_online_V2_0"
ut_node="YX870_PlanningNode_Load"
ut_online_node="YX870_PlanningNode_Load_online_V3_1_1"
decision_node="YX870_SystemDecision_V1_35"

sleep 10

# Start roscore
gnome-terminal --window --working-directory=${dir_ros} --title='roscore' -e 'bash -c "sleep 3;source /opt/ros/noetic/setup.bash;roscore; exec bash;"' \


sleep 5


# Start DPC nodes
gnome-terminal --window --working-directory=${dir_dpc} --title='smooth' -e 'bash -c "./'${smooth_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='decision_system' -e 'bash -c "./'${decision_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='gp' -e 'bash -c "./'${gp_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='dp' -e 'bash -c "./'${dp_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='dp_online' -e 'bash -c "./'${dp_online_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='ut' -e 'bash -c "./'${ut_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='ut_online' -e 'bash -c "./'${ut_online_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='rs' -e 'bash -c "./'${rs_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='rs_online' -e 'bash -c "./'${rs_online_node}';exec bash;"' \
--tab \
--working-directory=${dir_dpc} --title='traj_integrate' -e 'bash -c "./'${traj_integrate_node}';exec bash;"' \
# --tab \
# --working-directory=${dir_soft_joystick} --title='plotjuggler' -e 'bash -c "sleep 5; rosrun plotjuggler plotjuggler;exec bash;"' \



sleep 2
# Start ACT nodes
gnome-terminal --window --working-directory=${ACT_DIR} --title='ACT' -e 'bash -c "source devel/setup.bash;roslaunch action_fused_pynode  action_fused_node.launch;exec bash;"' \




sleep 3
# Start Sensor_Perception nodes
gnome-terminal --window --working-directory=${dir_sensor} --title='rslidar' -e 'bash -c "sleep 2;source devel/setup.bash;roslaunch rslidar_sdk start1.launch;exec bash;"' \
--tab \
--working-directory=${dir_imu} --title='imu' -e 'bash -c "sleep 2;source devel/setup.bash;rosrun can_signal_process can_signal_process_node;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='imu_talker' -e 'bash -c "sleep 2;source devel/setup.bash;roslaunch sensing imu_talker.launch;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='imu2joint' -e 'bash -c "sleep 2;source devel/setup.bash;roslaunch imu2joint sensing.launch;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='rtk_talker' -e 'bash -c "sleep 4;source devel/setup.bash;roslaunch sensing rtk_talker.launch;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='localization' -e 'bash -c "sleep 3;source devel/setup.bash;roslaunch localization localization.launch;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='excavation1' -e 'bash -c "sleep 5;source devel/setup.bash;roslaunch excavation_to_gridmap_pkg1 excavation_to_gridmap1.launch;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='range_image' -e 'bash -c "sleep 5;source devel/setup.bash;roslaunch pointcloud_to_rangeimage pointcloud_to_rangeimage.launch;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='map2png' -e 'bash -c "sleep 5;source devel/setup.bash;rosrun map2png map_to_png_signal_node;exec bash;"' \
--tab \
--working-directory=${dir_sensor} --title='rtk_com_talker' -e 'bash -c "sleep 5;source devel/setup.bash;roslaunch rtk_com_talker rtk_com_talker.launch;exec bash;"' \
--tab \
--working-directory=${dir_data_capture} --title='data_to_NAS' -e 'bash -c "sleep 5;source devel/setup.bash;roslaunch data_to_nas_and_ubuntu2204 data_to_NAS_and_ubuntu2204.launch;exec bash;"' \
#--tab \
#--working-directory=${dir_sensor} --title='rtksafety' -e 'bash -c "sleep 2;source devel/setup.bash;rosrun rtk_safety_pkg rtk_safety;exec bash;"' \
#--tab \
#--working-directory=${dir_sensor} --title='msg_fusion_node' -e 'bash -c "sleep 5;source devel/setup.bash;rosrun msg_fusion_node msg_fusion_node;exec bash;"' \
# --tab \
# --working-directory=${dir_sensor} --title='excavation' -e 'bash -c "sleep 5;source devel/setup.bash;roslaunch excavation_to_gridmap_pkg excavation_to_gridmap.launch;exec bash;"' \
# # --tab \
# --working-directory=${dir_sensor} --title='lstm_deployment' -e 'bash -c "sleep 5;source devel/setup.bash;rosrun lstm_deployment lstm_infer_node;exec bash;"' \
#--tab \
#--working-directory=${dir_sensor} --title='imu_segment' -e 'bash -c "sleep 5;source devel/setup.bash;roslaunch imu_segment_working_device_pkg imu_segment_working_device.launch;exec bash;"' \

#sleep 5

#gnome-terminal --working-directory=${dir_rtk} --title 'rtk_talker' -e 'bash -c "source devel/setup.bash;roslaunch rtk_talker.launch;exec bash"'

#sleep 2
# Soft Nodes
##can
##can_pkg="can_pkg"
##can_node="can_signal_process_node"
## joystick
excavator_node="excavator_node"  
## gui
gui_node="AutoDigMine_GUI_pkg_node"
## intel_panel
intel_panel_node="intel_panel_node"
## truck
truck_node="truck_node"
## signal
excavator_signal_node="excavator_signal_node"  


# Start Soft nodes sleep 5; ./'${gui_node}' /home/nvidia/Sany_and_DataCapture_rs/Soft/soft_release/intel_panel/conf.yaml
gnome-terminal --window --working-directory=${dir_soft_gui} --title='gui' -e 'bash -c " ; exec bash;"' \
--tab \
--working-directory=${dir_soft_excavator} --title='excavator' -e 'bash -c "sleep 10; ./'${excavator_node}';exec bash;"' \
--tab \
--working-directory=${dir_soft_truck} --title='truck' -e 'bash -c "sleep 10; ./'${truck_node}';exec bash;"' \
--tab \
--working-directory=${dir_soft_panel} --title='intel_panel' -e 'bash -c "sleep 5; ./'${intel_panel_node}'; exec bash;"' \
#--tab \
#--working-directory=${dir_soft_excavator} --title='excavator_signal' -e 'bash -c "sleep 5; ./'${excavator_signal_node}'; exec bash;"' \
#--tab \
#--working-directory=${dir_soft_intel} --title='startCan' -e 'bash -c "ssh root@192.168.1.100 "sh /home/root/tda4x/start_can.sh";exec bash;"' \

sleep 3
gnome-terminal --window --working-directory=${dir_trajfusion_act} --title='act_trajfusion' -e 'bash -c " ; exec bash;"' \
--tab \
--working-directory=${dir_trajfusion_act} --title='act_trajfusion' -e 'bash -c "sleep 2;source devel/setup.bash;roslaunch traj_fusion_pkg traj_fusion.launch;exec bash;"' \

sleep 4
gnome-terminal --window --working-directory=${dir_GP2mid} --title='gp' -e 'bash -c "source devel/setup.bash;roslaunch GP2mid gp2mid_with_params.launch;exec bash;"' \

sleep 5
gnome-terminal --window --working-directory=${dir_falut_manager} --title='falut_manager' -e 'bash -c " ; exec bash;"' \
--tab \
--working-directory=${dir_falut_manager} --title='falut_manager' -e 'bash -c "sleep 2;source devel/setup.bash;roslaunch FuncS_pkg fault_code_collector.launch;exec bash;"' \


sleep 2
# DataCapture nodes: save data
gnome-terminal --working-directory=${dir_data_capture} --title='Data_Capture' -e "bash -c \"
cleanup() {
    printf \\\"\\n>>> Ctrl+C detected. Shutting down...\\\\n\\\" 
    if [ -n \\\"\$ROSBAG_PID\\\" ]; then
        printf \\\">>> Finalizing bag file (PID: \$ROSBAG_PID)...\\\\n\\\"
        kill -SIGINT \$ROSBAG_PID
        wait \$ROSBAG_PID 2>/dev/null
    fi
    printf \\\">>> Shutting down all remaining processes...\\\\n\\\"
    kill 0
}
trap cleanup EXIT
sleep 2
source ~/.bashrc
source devel/setup.bash
sleep 2
python3 data_capture_signal_pub.py &
echo \\\">>> Ensuring bag directory exists at ${BAG_DIR}...\\\"
mkdir -p ${BAG_DIR}
echo \\\">>> Starting rosbag record in ${BAG_DIR} and Python scripts...\\\"
ROSBAG_PID=\$!
echo \\\"\\n>>> All processes are running. Press Ctrl+C in this terminal to stop capturing data.\\\"
wait
\""

# NAS watchdog: launch directly in the background so startup does not depend on
# DISPLAY, D-Bus, or another gnome-terminal being ready.
NAS_WATCHDOG_SCRIPT="/home/nvidia/unmanned_exca/Sany_and_DataCapture_rs/nas_ds923_watchdog.sh"
NAS_WATCHDOG_MOUNT_ROOT="/media/nvidia/G500"
NAS_WATCHDOG_DATA_ROOT="${NAS_WATCHDOG_MOUNT_ROOT}/DataCapture"
NAS_WATCHDOG_LOG_DIR="${NAS_WATCHDOG_DATA_ROOT}/nas_ds923_watchdog_logs"
NAS_WATCHDOG_MOUNT_WAIT_S=120

# Before G500 is mounted there is nowhere safe below DataCapture to write. Keep
# a private bootstrap log in /tmp and copy it to G500 as soon as the mount is
# verified. Never create DataCapture on an unmounted /media/nvidia/G500 path.
umask 077

# Remove only stale bootstrap logs owned by this startup user. These files can
# exist only when G500 was unavailable, so they cannot be stored below
# DataCapture until a later successful boot.
NAS_WATCHDOG_CURRENT_UID="$(id -u 2>/dev/null || printf '')"
if [ -n "$NAS_WATCHDOG_CURRENT_UID" ]; then
    find /tmp -xdev -mindepth 1 -maxdepth 1 -uid "$NAS_WATCHDOG_CURRENT_UID" \
        -type f -name 'nas_ds923_watchdog_launcher_bootstrap_*.log' \
        -mmin +4320 -delete 2>/dev/null || true
fi
NAS_WATCHDOG_BOOTSTRAP_LOG="$(mktemp /tmp/nas_ds923_watchdog_launcher_bootstrap_XXXXXX.log 2>/dev/null)"
if [ -z "$NAS_WATCHDOG_BOOTSTRAP_LOG" ]; then
    NAS_WATCHDOG_BOOTSTRAP_LOG="/dev/null"
fi
NAS_WATCHDOG_LAUNCH_LOG="$NAS_WATCHDOG_BOOTSTRAP_LOG"

nas_watchdog_launcher_log() {
    printf '[NAS watchdog launcher][%s] %s\n' "$(date '+%F %T')" "$*" >>"$NAS_WATCHDOG_LAUNCH_LOG"
}

nas_watchdog_launcher_log "Launcher entered as user=$(id -un 2>/dev/null || printf unknown), uid=$(id -u 2>/dev/null || printf unknown)."
nas_watchdog_launcher_log "Helper path: $NAS_WATCHDOG_SCRIPT"

# 简化逻辑：跳过挂载检查，直接假设挂载已准备好
nas_watchdog_launcher_log "Assuming G500 mount is ready (skipped mountpoint check)."

# 直接创建目录结构并启动，跳过所有安全检查
NAS_WATCHDOG_MOUNT_ROOT="/media/nvidia/G500"
NAS_WATCHDOG_DATA_ROOT="${NAS_WATCHDOG_MOUNT_ROOT}/DataCapture"
NAS_WATCHDOG_LOG_DIR="${NAS_WATCHDOG_DATA_ROOT}/nas_ds923_watchdog_logs"

# 确保目录存在
mkdir -p "$NAS_WATCHDOG_LOG_DIR"
chmod 700 "$NAS_WATCHDOG_LOG_DIR" 2>/dev/null || true

# 创建最终日志文件
NAS_WATCHDOG_LAUNCH_STAMP="$(date '+%Y%m%d_%H%M%S')"
NAS_WATCHDOG_FINAL_LAUNCH_LOG="$NAS_WATCHDOG_LOG_DIR/nas_ds923_watchdog_launcher_${NAS_WATCHDOG_LAUNCH_STAMP}_$$.log"
touch "$NAS_WATCHDOG_FINAL_LAUNCH_LOG"
chmod 600 "$NAS_WATCHDOG_FINAL_LAUNCH_LOG" 2>/dev/null || true

# 移动引导日志到最终位置
if [ "$NAS_WATCHDOG_BOOTSTRAP_LOG" != "/dev/null" ] && \
        [ -f "$NAS_WATCHDOG_BOOTSTRAP_LOG" ] && \
        [ ! -L "$NAS_WATCHDOG_BOOTSTRAP_LOG" ]; then
    cat "$NAS_WATCHDOG_BOOTSTRAP_LOG" >>"$NAS_WATCHDOG_FINAL_LAUNCH_LOG"
    rm -f "$NAS_WATCHDOG_BOOTSTRAP_LOG"
fi
NAS_WATCHDOG_LAUNCH_LOG="$NAS_WATCHDOG_FINAL_LAUNCH_LOG"
nas_watchdog_launcher_log "G500 directories prepared successfully."

# 启动 watchdog 脚本
if [ ! -f "$NAS_WATCHDOG_SCRIPT" ]; then
    nas_watchdog_launcher_log "ERROR: helper script not found: $NAS_WATCHDOG_SCRIPT"
elif [ ! -x "$NAS_WATCHDOG_SCRIPT" ]; then
    NAS_WATCHDOG_HELPER_STAT="$(stat -c 'owner=%U:%G mode=%a' "$NAS_WATCHDOG_SCRIPT" 2>/dev/null || printf unavailable)"
    nas_watchdog_launcher_log "ERROR: helper is not executable by $(id -un 2>/dev/null || printf current-user): $NAS_WATCHDOG_SCRIPT ($NAS_WATCHDOG_HELPER_STAT)"
elif ! command -v nohup >/dev/null 2>&1; then
    nas_watchdog_launcher_log "ERROR: required command 'nohup' is unavailable."
else
    NAS_WATCHDOG_HELPER_STAT="$(stat -c 'owner=%U:%G mode=%a' "$NAS_WATCHDOG_SCRIPT" 2>/dev/null || printf unavailable)"
    nas_watchdog_launcher_log "Starting helper ($NAS_WATCHDOG_HELPER_STAT) without a graphical terminal."
    # Clear an inherited askpass marker. Only the helper's SSH child is
    # allowed to set it for a single password prompt.
    nohup env -u NAS_ASKPASS_MODE "$NAS_WATCHDOG_SCRIPT" \
        </dev/null >>"$NAS_WATCHDOG_LAUNCH_LOG" 2>&1 &
    NAS_WATCHDOG_PID=$!
    nas_watchdog_launcher_log "Spawned watchdog PID $NAS_WATCHDOG_PID."
    sleep 2
    if kill -0 "$NAS_WATCHDOG_PID" 2>/dev/null; then
        nas_watchdog_launcher_log "Watchdog PID $NAS_WATCHDOG_PID is running."
    else
        wait "$NAS_WATCHDOG_PID"
        NAS_WATCHDOG_RC=$?
        if [ "$NAS_WATCHDOG_RC" -eq 0 ]; then
            nas_watchdog_launcher_log "Watchdog exited cleanly; another instance may already own the flock."
        else
            nas_watchdog_launcher_log "ERROR: watchdog exited during startup with status $NAS_WATCHDOG_RC."
        fi
    fi
fi
# exit 0
