

class MotorModel:
    '''
    Simulate a motor
    '''
    def __init__(
            self,
            time_constant_s: float = 0.15,
            max_speed_dps: float = 1500.0,
            gear_ratio: float = 1.0
        ):
        '''
        Args:
            time_constant_s (): Responding time (second)
            max_speed_dps: Maximum motor output speed (degree / seconds)
            gear_ratio: Given that output axis rotates 1 revolution, how many rotation the motor rotates
        '''
        self.time_constant_s = time_constant_s
        self.max_speed_dps = max_speed_dps
        self.gear_ratio = gear_ratio

        self.angle_deg = 0.0
        self.speed_dps = 0.0
        self._target_speed_dps = 0.0
        self._braking = False

    def step(self, dt_s: float) -> None:
        if self._braking:
            alpha = min(1.0, dt_s / max(self.time_constant_s * 0.3, 1e-6)) # damping coefficient
            self.speed_dps += -self.speed_dps * alpha # exponential

        else:
            alpha = min(1.0, dt_s / max(self.time_constant_s, 1e-6))
            self.speed_dps += (self._target_speed_dps - self.speed_dps) * alpha

        self.angle_deg += self.speed_dps * dt_s

    def run(self, speed_dps: float) -> None:
        self._braking = False
        clamped = max(-self.max_speed_dps, min(self.max_speed_dps, speed_dps))
        self._target_speed_dps = clamped

    def stop(self) -> None:
        '''
        Release motor
        '''
        self._braking = False
        self._target_speed_dps = 0.0
    
    def brake(self) -> None:
        '''
        Brake the motor with electronic resistance
        '''
        self._braking = True
        self._target_speed_dps = 0.0

    def reset_angle(self, angle_deg: float = 0.0) -> None:
        self.angle_deg = angle_deg

    def angle(self) -> float:
        return self.angle_deg

    def speed(self) -> float:
        return self.speed_dps

    def is_stalled(self, speed_threshold_dps: float = 1.0) -> bool:
        return abs(self._target_speed_dps) > speed_threshold_dps and abs(self.speed_dps) < speed_threshold_dps