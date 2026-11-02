class FlatBoomPidController:
    """
    动臂高度插值 PID 控制器（位置式）

    控制目标：斗齿尖端海拔高度与矿卡海拔高度的插值恒定。
    输出：动臂手柄开度（-1000 ~ 1000，浮点）

    特性：
    - 积分分离：误差较大时暂停积分，防止超调
    - 死区抗饱和：当下位机存在死区时，防止输出在死区内时积分持续累积
    - 微分低通滤波：抑制测量噪声对微分项的放大
    - 输出限幅至 [-1000, 1000]
    - 所有参数可在运行中在线修改
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1000.0,
        output_max: float = 1000.0,
        integral_sep_threshold: float = 0.5,  # 误差绝对值超过此值则取消积分作用
        derivative_filter_alpha: float = 0.1,  # 微分低通滤波系数 (0~1, 越小越平滑)
        derivative_filter_enable: bool = True,
        actuator_deadband_low: float = -100.0,  # 下位机死区下界
        actuator_deadband_high: float = 100.0,  # 下位机死区上界
        deadband_anti_windup_enable: bool = True,
    ):
        # PID 参数
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # 限幅
        self.output_min = output_min
        self.output_max = output_max

        # 积分分离
        self.integral_sep_threshold = integral_sep_threshold

        # 微分滤波
        self.derivative_filter_alpha = derivative_filter_alpha
        self.derivative_filter_enable = derivative_filter_enable

        # 下位机死区及抗饱和
        if actuator_deadband_low > actuator_deadband_high:
            raise ValueError("actuator_deadband_low must be <= actuator_deadband_high")
        self.actuator_deadband_low = actuator_deadband_low
        self.actuator_deadband_high = actuator_deadband_high
        self.deadband_anti_windup_enable = deadband_anti_windup_enable

        # 内部状态初始化
        self.reset()

    def reset(self):
        """重置控制器内部状态（积分累加器、历史误差、微分滤波器状态）"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._derivative_filtered = 0.0  # 微分滤波后的上一次输出

    def update(self, target: float, actual: float, dt: float) -> float:
        """
        计算 PID 输出

        参数：
            target: 目标插值 (m)
            actual: 当前实际插值 (m)
            dt: 控制周期 (秒)，由外部传入

        返回：
            手柄开度 (浮点，范围通常 -1000 ~ 1000)
        """
        # 1. 误差计算
        error = target - actual  # 正误差表示实际小于目标，需提升动臂 -> 输出正开度

        # 2. 比例项
        p_term = self.kp * error

        # 3. 积分项（带积分分离 + 死区抗饱和）
        if self.ki != 0.0 and dt > 0:
            # 积分分离判断：误差过大时不积分
            if abs(error) <= self.integral_sep_threshold:
                # 计算初步输出（仅 P，用于死区抗饱和判断）
                tentative_output = p_term + self._integral  # I 还未加上本次增量

                # 死区抗饱和：如果下位机死区存在且功能启用，判断是否应该暂停积分
                integrate = True
                if self.deadband_anti_windup_enable:
                    # 若当前输出（限幅前）已落在死区范围内，
                    # 且当前误差符号使得输出将更深入死区，则停止积分
                    if self.actuator_deadband_low <= tentative_output <= self.actuator_deadband_high:
                        # 输出已在死区内，判断误差方向
                        if (error > 0 and tentative_output >= self.actuator_deadband_high) or \
                           (error < 0 and tentative_output <= self.actuator_deadband_low):
                            # 误差正在试图将输出推离死区 → 允许积分
                            pass
                        else:
                            # 误差会将输出推向死区更深处 → 停止积分
                            integrate = False

                if integrate:
                    self._integral += self.ki * error * dt

        i_term = self._integral

        # 4. 微分项（带可选低通滤波）
        if self.kd != 0.0 and dt > 0:
            raw_derivative = (error - self._prev_error) / dt
            if self.derivative_filter_enable:
                # 一阶低通滤波
                self._derivative_filtered = (
                    self.derivative_filter_alpha * raw_derivative
                    + (1.0 - self.derivative_filter_alpha) * self._derivative_filtered
                )
                d_term = self.kd * self._derivative_filtered
            else:
                d_term = self.kd * raw_derivative
        else:
            d_term = 0.0

        # 5. 合成原始输出
        output = p_term + i_term + d_term

        # 6. 输出限幅
        output = max(self.output_min, min(self.output_max, output))

        # 7. 保存历史误差（必须在下次调用前保存）
        self._prev_error = error

        return output

    # ---------- 便捷 setter 方法 ----------
    def set_kp(self, kp: float):
        self.kp = kp

    def set_ki(self, ki: float):
        self.ki = ki

    def set_kd(self, kd: float):
        self.kd = kd

    def set_integral_sep_threshold(self, threshold: float):
        self.integral_sep_threshold = threshold

    def set_derivative_filter(self, enable: bool, alpha: float = None):
        """
        设置微分滤波开关及系数
        :param enable: True/False
        :param alpha: 滤波系数，0~1，若为 None 则保持不变
        """
        self.derivative_filter_enable = enable
        if alpha is not None:
            self.derivative_filter_alpha = max(0.0, min(1.0, alpha))

    def set_actuator_deadband(self, low: float, high: float):
        """设置下位机死区范围"""
        if low > high:
            raise ValueError("deadband low must be <= high")
        self.actuator_deadband_low = low
        self.actuator_deadband_high = high

    def set_deadband_anti_windup(self, enable: bool):
        self.deadband_anti_windup_enable = enable