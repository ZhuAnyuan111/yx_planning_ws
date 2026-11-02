from dataclasses import dataclass

@dataclass
class JointRateLimit:
    """单个关节的变化率限制配置"""
    up: float = 1000.0
    down: float = 1000.0


@dataclass
class RateLimitConfig:
    """所有关节的变化率限制配置，方便批量赋值"""
    swing: JointRateLimit = None
    boom: JointRateLimit = None
    arm: JointRateLimit = None
    bucket: JointRateLimit = None

    def __post_init__(self):
        if self.swing is None:
            self.swing = JointRateLimit()
        if self.boom is None:
            self.boom = JointRateLimit()
        if self.arm is None:
            self.arm = JointRateLimit()
        if self.bucket is None:
            self.bucket = JointRateLimit()

    def set_all(self, up: float = None, down: float = None):
        """批量设置所有关节的变化率"""
        for joint in [self.swing, self.boom, self.arm, self.bucket]:
            if up is not None:
                joint.up = up
            if down is not None:
                joint.down = down

    def set_joints(self, swing: tuple = None, boom: tuple = None, 
                   arm: tuple = None, bucket: tuple = None):
        """批量设置指定关节的变化率，每个参数为 (up, down) 元组"""
        if swing is not None:
            self.swing.up, self.swing.down = swing
        if boom is not None:
            self.boom.up, self.boom.down = boom
        if arm is not None:
            self.arm.up, self.arm.down = arm
        if bucket is not None:
            self.bucket.up, self.bucket.down = bucket