#!/usr/bin/env python3

import queue
import time
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String


class DecisionTestGui:
    REQ_ENTRY = 1.0
    ENTERING = 2.0
    LOADING = 3.0
    LEAVE_CONFIRMED = 4.0

    BOOM_UP_THRESHOLD = 34.0

    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.output_vars = {}
        self.entries = {}
        self.log_text = None
        self.last_action = tk.StringVar(value="Ready")
        self.angle_stream_job = None
        self.angle_stream_values = None

        rospy.init_node("decision_test_gui", anonymous=True, disable_signals=True)
        self._init_publishers()
        self._init_subscribers()
        self._build_ui()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.after(100, self._drain_events)

    def _init_publishers(self):
        self.pubs = {
            "mpc_switch": rospy.Publisher("/Mpc_Switch", Bool, queue_size=10),
            "mpc_switch2": rospy.Publisher("/Mpc_Switch2", Bool, queue_size=10),
            "truck_pose": rospy.Publisher("/truck_pose_topic", PoseStamped, queue_size=10),
            "truck_full": rospy.Publisher("/FlagTruckFull", Bool, queue_size=10),
            "joint_angles": rospy.Publisher("/joints_angle", Quaternion, queue_size=10),
            "real_swing": rospy.Publisher("/heading2swing_topic", Float32, queue_size=10),
            "ref_swing": rospy.Publisher("/Sys_SCo_Traj", Pose, queue_size=10),
            "truck_center_unloadpoint": rospy.Publisher(
                "/truck_center_unloadpoint", Quaternion, queue_size=10
            ),
            "upper_lower_alignment_finished": rospy.Publisher("/FlagUpperLowerAlignmentFinish", Bool, queue_size=10),
            "unman_params": rospy.Publisher("/unman_params_set", Pose, queue_size=10),
            "ground_plan_feedback": rospy.Publisher("/Sys_RGp_FlagGpPlanFinish", Float64, queue_size=10),
            "unload_plan_feedback": rospy.Publisher("/Sys_RUn_UnloadPointSequence", Float32MultiArray, queue_size=10),
            "reset_execute_feedback": rospy.Publisher("/RefDeviceTraj_Reset", Pose, queue_size=10),
            "dig_execute_feedback": rospy.Publisher("/RefDeviceTraj_Dig", Pose, queue_size=10),
            "unload_execute_feedback": rospy.Publisher("/RefDeviceTraj_Unload", Pose, queue_size=10),
            "smooth_finish": rospy.Publisher("/FlagSmoothFinish", Bool, queue_size=10),
            "clean_finish": rospy.Publisher("/FlagCleanFinish", Float32, queue_size=10),
            "walk_finish": rospy.Publisher("/FlagWalkFinish", Bool, queue_size=10),
        }

    def _init_subscribers(self):
        specs = [
            ("State", "/system_decision/state_text", String, lambda msg: msg.data),
            ("SeqPlan", "/Sys_SeqPlan", Float64, lambda msg: self._fmt_float(msg.data)),
            ("SeqAction", "/Sys_SeqAction", Float64, lambda msg: self._fmt_float(msg.data)),
            ("SeqTruck", "/Sys_SeqTruck", Float64, lambda msg: self._fmt_float(msg.data)),
            ("FaultTakeover", "/fault_require_takeover", Bool, lambda msg: str(msg.data)),
            ("BucketNumber", "/Sys_SUn_BucketNumber", Float64, lambda msg: self._fmt_float(msg.data)),
            ("TruckLoadNumber", "/Sys_TruckLoadNumber", Float32, lambda msg: self._fmt_float(msg.data)),
            ("GroundPlanCmd", "/Sys_SGp_FlagGpPlan", Float32, lambda msg: self._fmt_float(msg.data)),
            ("UnloadPlanCmd", "/Sys_SUn_FlagUnloadPlan", Float64, lambda msg: self._fmt_float(msg.data)),
            ("ResetExecCmd", "/Sys_SRe_FlagResetExcute", Float64, lambda msg: self._fmt_float(msg.data)),
            ("DigExecCmd", "/Sys_SDi_FlagDigExcute", Float64, lambda msg: self._fmt_float(msg.data)),
            ("GroundLevelExecCmd", "/Sys_SLe_FlagGroundLevelExcute", Float64, lambda msg: self._fmt_float(msg.data)),
            ("UnloadExecCmd", "/Sys_SUn_FlagUnloadExcute", Float64, lambda msg: self._fmt_float(msg.data)),
            ("LoadingCmd", "/digger_loading_topic", PoseStamped,
             lambda msg: self._fmt_float(msg.pose.orientation.w)),
        ]

        for name, topic, msg_type, formatter in specs:
            self.output_vars[name] = tk.StringVar(value="-")
            rospy.Subscriber(topic, msg_type, self._make_output_callback(name, formatter), queue_size=10)

    def _build_ui(self):
        self.root.title("Excavator System Decision Test GUI")
        self.root.geometry("1180x760")
        self.root.minsize(980, 640)

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", padding=(8, 5))
        style.configure("Section.TLabelframe", padding=(8, 6))

        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        controls = ttk.Frame(main)
        controls.grid(row=0, column=0, sticky="nsw", padx=(0, 10))

        monitor = ttk.Frame(main)
        monitor.grid(row=0, column=1, sticky="nsew")
        monitor.columnconfigure(0, weight=1)
        monitor.rowconfigure(1, weight=1)

        self._build_scenario_controls(controls)
        self._build_truck_controls(controls)
        self._build_feedback_controls(controls)
        self._build_status_controls(controls)
        self._build_monitor(monitor)

    def _build_scenario_controls(self, parent):
        frame = self._section(parent, "Scenario", 0)
        self._button(frame, "Start Truck", lambda: self._publish_bool("mpc_switch", True, "/Mpc_Switch true"), 0, 0)
        self._button(frame, "Stop Truck", lambda: self._publish_bool("mpc_switch", False, "/Mpc_Switch false"), 0, 1)
        self._button(frame, "Request Staging", lambda: self._publish_bool("mpc_switch2", True, "/Mpc_Switch2 true"), 1, 0)
        self._button(frame, "Reset Staging Trigger", lambda: self._publish_bool("mpc_switch2", False, "/Mpc_Switch2 false"), 1, 1)

    def _build_truck_controls(self, parent):
        frame = self._section(parent, "Truck Event", 1)
        self._button(frame, "Request Entry 1", lambda: self._publish_truck_event(self.REQ_ENTRY), 0, 0)
        self._button(frame, "Entering 2", lambda: self._publish_truck_event(self.ENTERING), 0, 1)
        self._button(frame, "Loading 3", lambda: self._publish_truck_event(self.LOADING), 1, 0)
        self._button(frame, "Leave Confirmed 4", lambda: self._publish_truck_event(self.LEAVE_CONFIRMED), 1, 1)

    def _build_feedback_controls(self, parent):
        frame = self._section(parent, "Feedback", 2)

        self._button(frame, "Ground Plan Done", lambda: self._publish_float64("ground_plan_feedback", 1.0, "ground plan done"), 0, 0)
        self._button(frame, "Ground Plan False", lambda: self._publish_float64("ground_plan_feedback", 0.0, "ground plan false"), 0, 1)

        self._button(frame, "Unload Plan 1", lambda: self._publish_unload_plan(1.0), 1, 0)
        self._button(frame, "Unload Plan 0", lambda: self._publish_unload_plan(0.0), 1, 1)
        self._button(frame, "Unload Plan 1->0", lambda: self._publish_plan_pair(self._publish_unload_plan), 1, 2)

        self._button(frame, "Reset Exec Done", lambda: self._publish_execute_done("reset_execute_feedback", "reset execute done"), 2, 0)
        self._button(frame, "Dig Exec Done", lambda: self._publish_execute_done("dig_execute_feedback", "dig execute done"), 2, 1)
        self._button(frame, "Unload Exec Done", lambda: self._publish_execute_done("unload_execute_feedback", "unload execute done"), 2, 3)

        self._button(frame, "Smooth True", lambda: self._publish_bool("smooth_finish", True, "smooth finish true"), 3, 0)
        self._button(frame, "Smooth False", lambda: self._publish_bool("smooth_finish", False, "smooth finish false"), 3, 1)
        self._button(frame, "Clean Finish 1", lambda: self._publish_float32("clean_finish", 1.0, "clean finish 1"), 4, 0)
        self._button(frame, "Clean Finish 2", lambda: self._publish_float32("clean_finish", 2.0, "clean finish 2"), 4, 1)
        self._button(frame, "Clean Finish 0", lambda: self._publish_float32("clean_finish", 0.0, "clean finish 0"), 4, 2)
        self._button(frame, "Walk Finish", lambda: self._publish_bool("walk_finish", True, "walk finish"), 5, 0)
        self._button(frame, "Upper/Lower Align Done",
                     lambda: self._publish_bool("upper_lower_alignment_finished", True,
                                                "upper/lower alignment done"),
                     5, 1)
        self._button(frame, "Upper/Lower Align Reset",
                     lambda: self._publish_bool("upper_lower_alignment_finished", False,
                                                "upper/lower alignment reset"),
                     5, 2)

    def _build_status_controls(self, parent):
        frame = self._section(parent, "Status Input", 3)

        self._entry(frame, "Boom", "boom", self.BOOM_UP_THRESHOLD, 0, 0)
        self._entry(frame, "Arm", "arm", 0.0, 0, 1)
        self._entry(frame, "Bucket", "bucket", 0.0, 0, 2)
        self._entry(frame, "Swing", "swing", 0.0, 1, 0)
        self._entry(frame, "Ref Swing", "ref_swing", 0.0, 1, 1)
        self._entry(frame, "Truck Center X", "truck_center_x", -1.0, 2, 0)
        self._entry(frame, "Truck Center Y", "truck_center_y", 0.0, 2, 1)
        self._button(frame, "Start Angle Stream", self._start_angle_stream, 3, 0)
        self._button(frame, "Stop Angle Stream", self._stop_angle_stream, 3, 1)
        self._button(frame, "Truck Full True", lambda: self._publish_bool("truck_full", True, "truck full true"), 4, 0)
        self._button(frame, "Truck Full False", lambda: self._publish_bool("truck_full", False, "truck full false"), 4, 1)

        self._button(frame, "Bucket Reset Pulse", self._publish_bucket_reset_pulse, 5, 0)
        self._button(frame, "unman w=0", lambda: self._publish_unman_params(0.0), 5, 1)
        self._button(frame, "unman w=1", lambda: self._publish_unman_params(1.0), 5, 2)

    def _build_monitor(self, parent):
        status = ttk.Frame(parent)
        status.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        status.columnconfigure(1, weight=1)
        ttk.Label(status, text="Last").grid(row=0, column=0, sticky="w", padx=(0, 6))
        ttk.Label(status, textvariable=self.last_action).grid(row=0, column=1, sticky="ew")

        frame = ttk.LabelFrame(parent, text="Decision Output", style="Section.TLabelframe")
        frame.grid(row=1, column=0, sticky="nsew")
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

        names = list(self.output_vars.keys())
        split = (len(names) + 1) // 2
        for idx, name in enumerate(names):
            base_col = 0 if idx < split else 2
            row = idx if idx < split else idx - split
            ttk.Label(frame, text=name).grid(row=row, column=base_col, sticky="w", padx=(4, 8), pady=3)
            ttk.Label(frame, textvariable=self.output_vars[name], anchor="w").grid(
                row=row, column=base_col + 1, sticky="ew", padx=(0, 16), pady=3
            )

        log_frame = ttk.LabelFrame(parent, text="Publish Log", style="Section.TLabelframe")
        log_frame.grid(row=2, column=0, sticky="nsew", pady=(8, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        parent.rowconfigure(2, weight=1)

        self.log_text = tk.Text(log_frame, height=10, wrap="word", state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)
        self._button(log_frame, "Clear Log", self._clear_log, 1, 0)

    def _section(self, parent, title, row):
        frame = ttk.LabelFrame(parent, text=title, style="Section.TLabelframe")
        frame.grid(row=row, column=0, sticky="ew", pady=(0, 8))
        for col in range(3):
            frame.columnconfigure(col, weight=1)
        return frame

    def _button(self, parent, text, command, row, column, columnspan=1):
        button = ttk.Button(parent, text=text, command=command)
        button.grid(row=row, column=column, columnspan=columnspan, sticky="ew", padx=3, pady=3)
        return button

    def _entry(self, parent, label, name, value, row, column):
        container = ttk.Frame(parent)
        container.grid(row=row, column=column, sticky="ew", padx=3, pady=3)
        container.columnconfigure(0, weight=1)
        ttk.Label(container, text=label).grid(row=0, column=0, sticky="w")
        entry = ttk.Entry(container, width=10)
        entry.insert(0, str(value))
        entry.grid(row=1, column=0, sticky="ew")
        self.entries[name] = entry

    def _make_output_callback(self, name, formatter):
        def callback(msg):
            try:
                value = formatter(msg)
            except Exception as exc:
                value = "format error: {}".format(exc)
            self.events.put(("output", name, value))

        return callback

    def _publish(self, key, msg, label):
        self.pubs[key].publish(msg)
        self._log(label)

    def _publish_bool(self, key, value, label):
        msg = Bool()
        msg.data = bool(value)
        self._publish(key, msg, label)

    def _publish_float32(self, key, value, label):
        msg = Float32()
        msg.data = float(value)
        self._publish(key, msg, label)

    def _publish_float64(self, key, value, label):
        msg = Float64()
        msg.data = float(value)
        self._publish(key, msg, label)

    def _publish_truck_event(self, code):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.orientation.w = float(code)
        self._publish("truck_pose", msg, "truck event {}".format(self._fmt_float(code)))

    def _publish_execute_done(self, key, label):
        self._publish(key, self._make_pose(0.0, 0.0, 0.0), label)

    def _read_angle_configuration(self):
        boom = self._read_float("boom")
        arm = self._read_float("arm")
        bucket = self._read_float("bucket")
        swing = self._read_float("swing")
        ref_swing = self._read_float("ref_swing")
        truck_center_x = self._read_float("truck_center_x")
        truck_center_y = self._read_float("truck_center_y")
        if any(value is None for value in (
                boom, arm, bucket, swing, ref_swing, truck_center_x, truck_center_y)):
            return None
        return boom, arm, bucket, swing, ref_swing, truck_center_x, truck_center_y

    def _publish_angle_configuration(self, values):
        boom, arm, bucket, swing, ref_swing, truck_center_x, truck_center_y = values
        msg = Quaternion()
        msg.x = boom
        msg.y = arm
        msg.z = bucket
        msg.w = 0.0
        self.pubs["joint_angles"].publish(msg)

        real_swing = Float32()
        real_swing.data = swing
        self.pubs["real_swing"].publish(real_swing)
        ref_pose = self._make_pose(0.0, 0.0, 0.0)
        ref_pose.orientation.x = ref_swing
        self.pubs["ref_swing"].publish(ref_pose)

        truck_center = Quaternion()
        truck_center.x = truck_center_x
        truck_center.y = truck_center_y
        truck_center.z = 0.0
        truck_center.w = 0.0
        self.pubs["truck_center_unloadpoint"].publish(truck_center)

    def _start_angle_stream(self):
        values = self._read_angle_configuration()
        if values is None:
            return

        self._stop_angle_stream(log=False)
        self.angle_stream_values = values
        self._publish_angle_configuration(values)
        self._log(
            "angle stream started: boom={} arm={} bucket={} swing={} ref_swing={} "
            "truck_center_x={} truck_center_y={}".format(*values)
        )
        self._schedule_angle_stream()

    def _schedule_angle_stream(self):
        if self.angle_stream_values is None or rospy.is_shutdown():
            self.angle_stream_job = None
            return
        self.angle_stream_job = self.root.after(100, self._publish_angle_stream_tick)

    def _publish_angle_stream_tick(self):
        self.angle_stream_job = None
        if self.angle_stream_values is None or rospy.is_shutdown():
            return
        self._publish_angle_configuration(self.angle_stream_values)
        self._schedule_angle_stream()

    def _stop_angle_stream(self, log=True):
        if self.angle_stream_job is not None:
            self.root.after_cancel(self.angle_stream_job)
            self.angle_stream_job = None
        was_running = self.angle_stream_values is not None
        self.angle_stream_values = None
        if log and was_running:
            self._log("angle stream stopped")

    def _publish_unman_params(self, w_value):
        msg = self._make_pose(0.0, 0.0, 0.0)
        msg.orientation.w = float(w_value)
        self._publish("unman_params", msg, "unman params w={}".format(w_value))

    def _publish_bucket_reset_pulse(self):
        self._publish_unman_params(0.0)
        self.root.after(100, lambda: self._publish_unman_params(1.0))

    def _publish_unload_plan(self, flag):
        msg = Float32MultiArray()
        msg.data = self._make_unload_plan_data(flag)
        self._publish("unload_plan_feedback", msg, "unload plan flag {}".format(flag))

    def _publish_plan_pair(self, publish_fn):
        publish_fn(1.0)
        self.root.after(100, lambda: publish_fn(0.0))

    def _make_unload_plan_data(self, flag):
        data = [float(flag)]
        data.extend([238.17001342773438] * 9999)
        return data

    def _make_pose(self, x, y, z):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 0.0
        return msg

    def _read_float(self, name):
        text = self.entries[name].get().strip()
        try:
            return float(text)
        except ValueError:
            messagebox.showerror("Invalid value", "{} must be a number".format(name))
            return None

    def _fmt_float(self, value):
        return "{:.3f}".format(float(value))

    def _log(self, text):
        line = "[{}] {}".format(time.strftime("%H:%M:%S"), text)
        self.last_action.set(text)
        if self.log_text is None:
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def _drain_events(self):
        try:
            while True:
                kind, name, value = self.events.get_nowait()
                if kind == "output":
                    self.output_vars[name].set(value)
        except queue.Empty:
            pass

        if not rospy.is_shutdown():
            self.root.after(100, self._drain_events)

    def _on_close(self):
        self._stop_angle_stream(log=False)
        rospy.signal_shutdown("GUI closed")
        self.root.destroy()


def main():
    root = tk.Tk()
    DecisionTestGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
