#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""flow_hud.py - Global full-flow debug HUD for the decision-node FSM.

Purpose
-------
Aggregate the production decision node's status topics into one always-visible
RViz text panel plus a rolling stage-transition timeline, so the whole cycle
(dig -> unload -> reset -> level -> next truck) can be debugged from a global
view without echoing topics.

This node is READ-ONLY: it only subscribes to existing topics and publishes
visualization markers. It never touches the decision node or any planner.

Subscribes (see excavator_system_decision/src/topic_interface.cpp):
  /system_decision/state_text   String   "TruckStage:OpState:StagingPre:StagingCyc"
  /Sys_SeqAction                Float64  action code (1..9 / x.5 done pulses)
  /Sys_SUn_BucketNumber         Float64  current bucket count
  /Sys_SeqTruck                 Float64  truck sequence
  /Sys_TruckLoadNumber          Float64  truck load number
  /fault_require_takeover       Bool     takeover monitor triggered
  /truck_pose_topic             PoseStamped  orientation.w = truck event code
  /Sys_SDi_FlagDigExcute        Float64  dig trigger flag
  /Sys_SRe_FlagResetExcute      Float64  reset trigger flag
  /Sys_SUn_FlagUnloadPlan       Float64  unload-plan trigger flag
  /Sys_SUn_FlagUnloadExcute     Float64  unload-execute trigger flag
  /Sys_SGp_FlagGpPlan           Float64  ground-plan trigger flag

Publishes:
  /decision_hud   visualization_msgs/MarkerArray   panel + event timeline

Topic contract lives in docs/sim_arch.md.
"""

import threading
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, String
from visualization_msgs.msg import Marker, MarkerArray


# seq_action code -> human label (verified against decision FSM sources)
SEQ_ACTION_LABEL = {
    1.0: "TRUCK_POSITION_CHECK",
    2.0: "RESET_EXECUTE",
    2.5: "RESET_DONE",
    3.0: "DIG_EXECUTE",
    3.5: "DIG_DONE",
    4.0: "UNLOAD_EXECUTE",
    4.5: "UNLOAD_DONE(+1 bucket)",
    5.0: "BOOM_UP",
    6.0: "LEVEL_TRUCK",
    6.5: "LEVEL_DONE",
    7.0: "GROUND_LEVEL(staging)",
    7.5: "GROUND_LEVEL_DONE",
    8.0: "WALK(staging)",
    9.0: "SWING_RESET(staging)",
}

# truck event code (orientation.w) -> label
TRUCK_EVENT_LABEL = {
    0.0: "-",
    1.0: "REQUEST_ENTRY",
    2.0: "ENTERING",
    3.0: "LOADING",
    4.0: "LEAVE_CONFIRMED",
}


def seq_action_label(v):
    if v is None:
        return "-"
    # snap to the nearest known code within 1e-6 (floats are exact here)
    label = SEQ_ACTION_LABEL.get(round(v, 2))
    if label is None:
        return "UNKNOWN(%.2f)" % v
    return label


class FlowHud:
    def __init__(self):
        self.lock = threading.Lock()

        # ---- parameters ----
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.rate_hz = float(rospy.get_param("~rate_hz", 5.0))
        anchor = rospy.get_param("~anchor", [0.0, 0.0, 8.0])
        self.anchor = (float(anchor[0]), float(anchor[1]), float(anchor[2]))
        self.text_scale = float(rospy.get_param("~text_scale", 0.6))
        # 每行是独立 marker，垂直行距 = text_scale * spacing_factor（>1 才不会叠字）
        self.spacing_factor = float(rospy.get_param("~line_spacing_factor", 1.5))
        self.event_lines = int(rospy.get_param("~event_lines", 6))
        self.flash_sec = float(rospy.get_param("~flash_sec", 0.6))
        self.show_flags = bool(rospy.get_param("~show_flags", True))

        # ---- cached state ----
        self.state_text = ""
        self.seq_action = None
        self.bucket_number = None
        self.seq_truck = None
        self.truck_load = None
        self.truck_event = None
        self.takeover = False
        self.flags = {"dig": 0.0, "reset": 0.0, "unload_plan": 0.0,
                      "unload_exec": 0.0, "ground_plan": 0.0}

        # ---- transition log ----
        self.events = deque(maxlen=self.event_lines)
        self._last_sig = None
        self._last_sig_time = None
        self._last_flash_time = rospy.Time(0)

        # ---- publishers ----
        self.pub = rospy.Publisher("/decision_hud", MarkerArray, queue_size=1)

        # ---- subscribers ----
        rospy.Subscriber("/system_decision/state_text", String, self._cb_state)
        rospy.Subscriber("/Sys_SeqAction", Float64, self._cb_seq_action)
        rospy.Subscriber("/Sys_SUn_BucketNumber", Float64, self._cb_bucket)
        rospy.Subscriber("/Sys_SeqTruck", Float64, self._cb_seq_truck)
        rospy.Subscriber("/Sys_TruckLoadNumber", Float64, self._cb_truck_load)
        rospy.Subscriber("/fault_require_takeover", Bool, self._cb_takeover)
        rospy.Subscriber("/truck_pose_topic", PoseStamped, self._cb_truck_pose)
        rospy.Subscriber("/Sys_SDi_FlagDigExcute", Float64,
                         lambda m: self._cb_flag("dig", m.data))
        rospy.Subscriber("/Sys_SRe_FlagResetExcute", Float64,
                         lambda m: self._cb_flag("reset", m.data))
        rospy.Subscriber("/Sys_SUn_FlagUnloadPlan", Float64,
                         lambda m: self._cb_flag("unload_plan", m.data))
        rospy.Subscriber("/Sys_SUn_FlagUnloadExcute", Float64,
                         lambda m: self._cb_flag("unload_exec", m.data))
        rospy.Subscriber("/Sys_SGp_FlagGpPlan", Float64,
                         lambda m: self._cb_flag("ground_plan", m.data))

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._on_timer)
        rospy.loginfo("[flow_hud] HUD up on /decision_hud (frame=%s, %.1fHz)",
                      self.frame_id, self.rate_hz)

    # ------------------------------------------------------------------ subs
    def _cb_state(self, msg):
        with self.lock:
            self.state_text = msg.data
            self._record_transition()

    def _cb_seq_action(self, msg):
        with self.lock:
            self.seq_action = msg.data
            self._record_transition()

    def _cb_bucket(self, msg):
        with self.lock:
            self.bucket_number = msg.data

    def _cb_seq_truck(self, msg):
        with self.lock:
            self.seq_truck = msg.data

    def _cb_truck_load(self, msg):
        with self.lock:
            self.truck_load = msg.data

    def _cb_takeover(self, msg):
        with self.lock:
            self.takeover = bool(msg.data)

    def _cb_truck_pose(self, msg):
        with self.lock:
            code = float(msg.pose.orientation.w)
            if code != self.truck_event:
                self.truck_event = code
                self._push_event("TRUCK_EVENT -> %s" %
                                 TRUCK_EVENT_LABEL.get(round(code, 2), "%.1f" % code))

    def _cb_flag(self, name, data):
        with self.lock:
            self.flags[name] = float(data)

    # ------------------------------------------------------------- transitions
    def _record_transition(self):
        """Log a line whenever (stage | seq_action) signature changes."""
        now = rospy.Time.now()
        sig = "%s|%s" % (self.state_text.split(":")[0],
                         "%.1f" % self.seq_action if self.seq_action is not None else "-")
        if self._last_sig is None:
            self._last_sig = sig
            self._last_sig_time = now
            return
        if sig != self._last_sig:
            dur = (now - self._last_sig_time).to_sec() if self._last_sig_time else 0.0
            label = "STAGE %s | act %s" % (
                self.state_text.split(":")[0] if self.state_text else "-",
                seq_action_label(self.seq_action))
            self._push_event("+%.1fs  %s" % (dur, label), now)
            self._last_sig = sig
            self._last_sig_time = now

    def _push_event(self, text, when=None):
        when = when or rospy.Time.now()
        self.events.append((when, text))
        self._last_flash_time = when

    # ------------------------------------------------------------------ timer
    def _on_timer(self, _evt):
        with self.lock:
            lines = self._build_lines()
            arr = MarkerArray()
            # 先清空上一帧（行数会随时减少，避免残留叠字）
            wipe = Marker()
            wipe.header.frame_id = self.frame_id
            wipe.header.stamp = rospy.Time.now()
            wipe.action = Marker.DELETEALL
            arr.markers.append(wipe)
            spacing = self.text_scale * self.spacing_factor
            for idx, (text, rgb) in enumerate(lines):
                if not text:
                    continue  # 空行仅占位，靠 z 下移留白，不发 marker
                m = self._base_marker("hud_line", idx, rgb)
                m.text = text
                m.pose.position.x = self.anchor[0]
                m.pose.position.y = self.anchor[1]
                m.pose.position.z = self.anchor[2] - idx * spacing
                arr.markers.append(m)
        self.pub.publish(arr)

    def _base_marker(self, ns, mid, rgb):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.frame_locked = True
        m.scale.z = self.text_scale
        m.color.r, m.color.g, m.color.b, m.color.a = rgb[0], rgb[1], rgb[2], 1.0
        return m

    def _status_color(self):
        """red on takeover, flashing yellow right after a transition, else green."""
        if self.takeover:
            return (1.0, 0.2, 0.2)
        age = (rospy.Time.now() - self._last_flash_time).to_sec()
        if age <= self.flash_sec:
            return (1.0, 0.85, 0.1)
        return (0.3, 0.95, 0.4)

    def _build_lines(self):
        """Return ordered list of (text, (r,g,b)) from top to bottom."""
        stage_parts = self.state_text.split(":") if self.state_text else ["-", "-", "-", "-"]
        while len(stage_parts) < 4:
            stage_parts.append("-")
        status = self._status_color()
        white = (0.9, 0.9, 0.9)
        gray = (0.7, 0.7, 0.75)
        dim = (0.45, 0.45, 0.5)

        truck_evt = TRUCK_EVENT_LABEL.get(
            round(self.truck_event, 1) if self.truck_event is not None else None, "-")
        lines = [
            ("== DECISION FLOW HUD ==", status),
            ("Stage : %s" % stage_parts[0], white),
            ("Op : %s (Staging: %s / %s)" % (stage_parts[1], stage_parts[2], stage_parts[3]), white),
            ("SeqAct: %s" % seq_action_label(self.seq_action), status),
            ("Bucket: %s  Truck: %s  Load: %s" % (
                _f(self.bucket_number), _f(self.seq_truck, 0), _f(self.truck_load, 0)), white),
            ("TruckEvt: %s  Takeover: %s" % (truck_evt, "ON" if self.takeover else "off"), white),
        ]
        if self.show_flags:
            lines.append(("Flags : " + " ".join(
                "%s%s" % (k, "*" if self.flags[k] >= 0.5 else "")
                for k in ("dig", "reset", "unload_plan", "unload_exec", "ground_plan")), gray))
        lines.append(("", gray))  # 空行分隔
        lines.append(("-- transitions (newest) --", dim))
        rows = [t for (_, t) in reversed(list(self.events))]
        for i, r in enumerate(rows):
            lines.append((r, gray if i == 0 else dim))
        return lines



def _f(v, nd=1):
    return ("%." + str(nd) + "f") % v if v is not None else "-"


def main():
    rospy.init_node("flow_hud", anonymous=False)
    FlowHud()
    rospy.spin()


if __name__ == "__main__":
    main()
