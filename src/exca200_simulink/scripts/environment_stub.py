#!/usr/bin/env python3
"""environment_stub.py — 模拟 decision 节点所需的"外部世界"信号。

职责
----
1. 卡车事件自动序列：根据 decision state_text 自动推进
   request_entry → loading → leave_confirmed → 循环。
2. 整地规划反馈：收到 /Sys_SGp_FlagGpPlan 后延迟发布完成旗标。
3. 满车判定：监听 /Sys_SUn_BucketNumber，超阈值后发布 /FlagTruckFull。
4. 参考回转角：从 /truck_center_unloadpoint 计算 atan2(-y,-x) → /Sys_SCo_Traj。
5. 整平完成反馈：state_text 含 LEVEL_TRUCK 后延迟发布 /FlagSmoothFinish。
6. 静态 latched 信号：/FlagCleanFinish, /FlagWalkFinish, /FlagUpperLowerAlignmentFinish,
   /unman_params_set, /Mpc_Switch2。
7. 自动启动：延迟后发布 /Mpc_Switch=true 触发 decision 开始工作。

话题契约见 docs/sim_arch.md。
"""

import math
import threading

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from std_msgs.msg import Bool, Float32, Float64, String


class EnvironmentStub:
    """为 excavator_system_decision 节点提供全部外部世界反馈桩。"""

    def __init__(self):
        # ---------- 参数 ----------
        self.auto_truck_events = rospy.get_param('~auto_truck_events', True)
        self.truck_entry_delay = float(rospy.get_param('~truck_entry_delay_sec', 3.0))
        self.truck_loading_delay = float(rospy.get_param('~truck_loading_delay_sec', 2.0))
        self.truck_leave_delay = float(rospy.get_param('~truck_leave_delay_sec', 5.0))
        self.ground_plan_delay = float(rospy.get_param('~ground_plan_delay_sec', 1.0))
        self.smooth_finish_delay = float(rospy.get_param('~smooth_finish_delay_sec', 2.0))
        self.auto_start_delay = float(rospy.get_param('~auto_start_delay_sec', 2.0))
        self.truck_full_threshold = int(rospy.get_param('~truck_full_threshold', 99))
        self.truck_event_rate = float(rospy.get_param('~truck_event_rate_hz', 2.0))
        self.sco_traj_rate = float(rospy.get_param('~sco_traj_rate_hz', 10.0))

        # ---------- 状态 ----------
        self._lock = threading.Lock()
        self._state_text = ''
        self._truck_event_code = 0.0  # 当前发布的卡车事件码
        self._truck_phase = 'idle'    # idle | entry_pending | entry | loading | leave_pending | leave
        self._phase_enter_time = None
        self._smooth_sent = False
        self._truck_full_sent = False
        self._ground_plan_pending = False
        self._ground_plan_time = None
        self._bucket_number = 0
        self._truck_center_angle = None

        # ---------- 发布者 ----------
        self.pub_truck_pose = rospy.Publisher('/truck_pose_topic', PoseStamped, queue_size=5)
        self.pub_ground_plan_finish = rospy.Publisher('/Sys_RGp_FlagGpPlanFinish', Float64,
                                                      queue_size=1, latch=True)
        self.pub_truck_full = rospy.Publisher('/FlagTruckFull', Bool, queue_size=1, latch=True)
        self.pub_smooth = rospy.Publisher('/FlagSmoothFinish', Bool, queue_size=1, latch=True)
        self.pub_clean = rospy.Publisher('/FlagCleanFinish', Float32, queue_size=1, latch=True)
        self.pub_walk = rospy.Publisher('/FlagWalkFinish', Bool, queue_size=1, latch=True)
        self.pub_align = rospy.Publisher('/FlagUpperLowerAlignmentFinish', Bool,
                                         queue_size=1, latch=True)
        self.pub_unman = rospy.Publisher('/unman_params_set', Pose, queue_size=1, latch=True)
        self.pub_mpc2 = rospy.Publisher('/Mpc_Switch2', Bool, queue_size=1, latch=True)
        self.pub_mpc = rospy.Publisher('/Mpc_Switch', Bool, queue_size=1)
        self.pub_sco_traj = rospy.Publisher('/Sys_SCo_Traj', Pose, queue_size=5)

        # ---------- 订阅者 ----------
        rospy.Subscriber('/system_decision/state_text', String, self._state_text_cb, queue_size=1)
        rospy.Subscriber('/Sys_SGp_FlagGpPlan', Float32, self._ground_plan_cb, queue_size=1)
        rospy.Subscriber('/Sys_SUn_BucketNumber', Float64, self._bucket_number_cb, queue_size=1)
        rospy.Subscriber('/truck_center_unloadpoint', Quaternion, self._truck_center_cb, queue_size=1)

        # ---------- 静态 latched 发布（启动时一次）----------
        self.pub_clean.publish(Float32(0.0))
        self.pub_walk.publish(Bool(False))
        self.pub_align.publish(Bool(False))
        self.pub_truck_full.publish(Bool(False))
        self.pub_smooth.publish(Bool(False))
        unman = Pose()
        unman.orientation.w = 1.0
        self.pub_unman.publish(unman)
        self.pub_mpc2.publish(Bool(False))

        # ---------- 定时器 ----------
        # 卡车事件持续发布（truck_event_stale_sec=2.0 要求事件持续刷新）
        rospy.Timer(rospy.Duration(1.0 / self.truck_event_rate), self._truck_event_tick)
        # /Sys_SCo_Traj 10Hz 发布
        rospy.Timer(rospy.Duration(1.0 / self.sco_traj_rate), self._sco_traj_tick)
        # 主逻辑 tick（检查延迟、状态转换）
        rospy.Timer(rospy.Duration(0.1), self._logic_tick)
        # 自动启动
        if self.auto_start_delay >= 0:
            rospy.Timer(rospy.Duration(self.auto_start_delay), self._auto_start_cb, oneshot=True)

        rospy.loginfo("[env_stub] ready; auto_truck=%s entry_delay=%.1f loading_delay=%.1f "
                      "leave_delay=%.1f gp_delay=%.1f smooth_delay=%.1f start_delay=%.1f "
                      "full_threshold=%d",
                      self.auto_truck_events, self.truck_entry_delay, self.truck_loading_delay,
                      self.truck_leave_delay, self.ground_plan_delay, self.smooth_finish_delay,
                      self.auto_start_delay, self.truck_full_threshold)

    # ===================== 回调 =====================

    def _state_text_cb(self, msg):
        with self._lock:
            self._state_text = msg.data

    def _ground_plan_cb(self, msg):
        """收到 /Sys_SGp_FlagGpPlan（Float32）非零 → 开始整地规划延迟。"""
        if msg.data == 0.0:
            return
        with self._lock:
            if not self._ground_plan_pending:
                self._ground_plan_pending = True
                self._ground_plan_time = rospy.Time.now()
                rospy.loginfo("[env_stub] ground plan requested, will finish in %.1fs",
                              self.ground_plan_delay)

    def _bucket_number_cb(self, msg):
        """监听 decision 发布的 bucket_number，判断是否触发满车。"""
        with self._lock:
            self._bucket_number = int(msg.data)
            if self._bucket_number >= self.truck_full_threshold and not self._truck_full_sent:
                self._truck_full_sent = True
                self.pub_truck_full.publish(Bool(True))
                rospy.loginfo("[env_stub] truck FULL at bucket_number=%d (threshold=%d)",
                              self._bucket_number, self.truck_full_threshold)
            elif self._bucket_number < self.truck_full_threshold and self._truck_full_sent:
                # bucket_number 被 decision 重置（新卡车），清除满车标志
                self._truck_full_sent = False
                self.pub_truck_full.publish(Bool(False))

    def _truck_center_cb(self, msg):
        """从 /truck_center_unloadpoint (Quaternion x,y) 计算参考回转角。"""
        theta = math.degrees(math.atan2(-msg.y, -msg.x))
        if theta < 0.0:
            theta += 360.0
        with self._lock:
            self._truck_center_angle = theta

    def _auto_start_cb(self, _event):
        """延迟后发布 /Mpc_Switch=true 启动 decision。"""
        rospy.loginfo("[env_stub] auto-start: publishing /Mpc_Switch=true")
        self.pub_mpc.publish(Bool(True))

    # ===================== 定时逻辑 =====================

    def _logic_tick(self, _event):
        """主逻辑：检查延迟条件，推进状态机。"""
        now = rospy.Time.now()
        with self._lock:
            state = self._state_text

            # --- 整地规划完成 ---
            if self._ground_plan_pending and self._ground_plan_time is not None:
                elapsed = (now - self._ground_plan_time).to_sec()
                if elapsed >= self.ground_plan_delay:
                    self._ground_plan_pending = False
                    self._ground_plan_time = None
                    self.pub_ground_plan_finish.publish(Float64(1.0))
                    rospy.loginfo("[env_stub] ground plan FINISHED")

            # --- 整平完成 ---
            if 'LEVEL_TRUCK' in state and not self._smooth_sent:
                if self._phase_enter_time is None or 'LEVEL_TRUCK' not in getattr(self, '_last_state_for_smooth', ''):
                    self._last_state_for_smooth = state
                    self._smooth_timer_start = now
                elif (now - self._smooth_timer_start).to_sec() >= self.smooth_finish_delay:
                    self._smooth_sent = True
                    self.pub_smooth.publish(Bool(True))
                    rospy.loginfo("[env_stub] FlagSmoothFinish=true (LEVEL_TRUCK responded)")
            elif 'LEVEL_TRUCK' not in state:
                # 重置整平状态以便下一循环
                if self._smooth_sent:
                    self._smooth_sent = False
                    self.pub_smooth.publish(Bool(False))
                self._last_state_for_smooth = ''

            # --- 卡车事件自动序列 ---
            if self.auto_truck_events:
                self._update_truck_phase(state, now)

    def _update_truck_phase(self, state, now):
        """根据 state_text 推进卡车事件状态机。"""
        # IDLE → 延迟后开始 request_entry
        if self._truck_phase == 'idle':
            if self._phase_enter_time is None:
                self._phase_enter_time = now
            elif (now - self._phase_enter_time).to_sec() >= self.truck_entry_delay:
                self._truck_phase = 'entry'
                self._truck_event_code = 1.0
                self._phase_enter_time = now
                rospy.loginfo("[env_stub] truck event → request_entry (w=1.0)")

        # INITIAL_WAIT_TRUCK_ENTRY / WAIT_TRUCK_ENTRY → 保持 request_entry
        elif self._truck_phase == 'entry':
            # 延迟后自动切换为 loading（模拟卡车收到 permit_entry 后驶入就位）
            # decision 需要同时满足 dig_wait_finished + event_code==3.0 才转移，
            # 所以提前发 3.0 不会误触发（decision 会等 dig_wait 完成）。
            if (now - self._phase_enter_time).to_sec() >= self.truck_loading_delay:
                self._truck_phase = 'loading'
                self._truck_event_code = 3.0
                self._phase_enter_time = now
                rospy.loginfo("[env_stub] truck event → loading (w=3.0)")

        # DIGGING_CYCLE → 保持 loading
        elif self._truck_phase == 'loading':
            # DISPATCH_TRUCK_LEAVE → 延迟后发 leave_confirmed
            if 'DISPATCH_TRUCK_LEAVE' in state:
                self._truck_phase = 'leave_pending'
                self._phase_enter_time = now
                rospy.loginfo("[env_stub] truck leave pending (waiting %.1fs)", self.truck_leave_delay)

        # leave_pending → 延迟后发 leave_confirmed
        elif self._truck_phase == 'leave_pending':
            if (now - self._phase_enter_time).to_sec() >= self.truck_leave_delay:
                self._truck_phase = 'leave'
                self._truck_event_code = 4.0
                self._phase_enter_time = now
                rospy.loginfo("[env_stub] truck event → leave_confirmed (w=4.0)")

        # leave → 等 WAIT_TRUCK_ENTRY 出现后循环
        elif self._truck_phase == 'leave':
            if 'WAIT_TRUCK_ENTRY' in state or 'INITIAL_WAIT' in state:
                self._truck_phase = 'entry'
                self._truck_event_code = 1.0
                self._phase_enter_time = now
                rospy.loginfo("[env_stub] truck event → request_entry (w=1.0) [next truck]")

    def _truck_event_tick(self, _event):
        """持续发布当前卡车事件码（防止 truck_event_stale_sec 超时）。"""
        with self._lock:
            code = self._truck_event_code
        if code == 0.0:
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.orientation.w = code
        self.pub_truck_pose.publish(msg)

    def _sco_traj_tick(self, _event):
        """发布 /Sys_SCo_Traj（Pose, orientation.x = 参考回转角 deg）。"""
        with self._lock:
            angle = self._truck_center_angle
        if angle is None:
            return
        msg = Pose()
        msg.orientation.x = angle
        self.pub_sco_traj.publish(msg)


if __name__ == '__main__':
    rospy.init_node('environment_stub')
    node = EnvironmentStub()
    rospy.spin()
