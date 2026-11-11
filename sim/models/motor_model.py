
from .model import Model


class MotorModel(Model):
    '''
    Simulate a motor.
    '''
    def __init__(
            self,
            time_constant_s: float = 0.15,
            max_speed_dps: float = 1500.0,
            gear_ratio: float = 1.0
        ):
        '''
        Create the model.

        Arg:
        time_constant_s (float): Response time in seconds.
        max_speed_dps (float): Maximum motor speed in deg/s.
        gear_ratio (float): Motor revolutions per output revolution.
        '''
        super().__init__(time_constant_s)
        self.max_speed_dps: float = max_speed_dps
        self.gear_ratio: float = gear_ratio

        self._angle_deg: float = 0.0
        self._speed_dps: float = 0.0
        self._target_speed_dps: float = 0.0
        self._braking: bool = False
        self._hold_angle_deg = None

    def step(self, dt_s: float) -> None:
        if self._hold_angle_deg is not None:
            error = self._hold_angle_deg - self._angle_deg
            requested_speed = 10.0 * error
            self._target_speed_dps = max(
                -self.max_speed_dps,
                min(self.max_speed_dps, requested_speed),
            )

        if self._braking:
            alpha = min(1.0, dt_s / max(self.time_constant_s * 0.3, 1e-6)) # damping coefficient
            self._speed_dps += -self._speed_dps * alpha # exponential

        else:
            alpha = min(1.0, dt_s / max(self.time_constant_s, 1e-6))
            self._speed_dps += (self._target_speed_dps - self._speed_dps) * alpha

        self._angle_deg += self._speed_dps * dt_s

    def run(self, speed_dps: float) -> None:
        self._hold_angle_deg = None
        self._braking = False
        clamped = max(-self.max_speed_dps, min(self.max_speed_dps, speed_dps))
        self._target_speed_dps = clamped

    def stop(self) -> None:
        '''
        Release the motor.
        '''
        self._hold_angle_deg = None
        self._braking = False
        self._target_speed_dps = 0.0
    
    def brake(self) -> None:
        '''
        Brake the motor with electrical resistance.
        '''
        self._hold_angle_deg = None
        self._braking = True
        self._target_speed_dps = 0.0

    # def set_angle(self, angle_deg: float = 0.0) -> None:
    #     self._angle_deg = angle_deg

    def get_angle(self) -> float:
        return self._angle_deg

    angle = property(get_angle)

    def get_speed(self) -> float:
        return self._speed_dps

    speed = property(get_speed)

    @property
    def output_angle(self) -> float:
        '''
        Return the gear output angle in degrees.
        '''
        return self.angle / self.gear_ratio

    @property
    def output_speed(self) -> float:
        '''
        Return the gear output speed in deg/s.
        '''
        return self.speed / self.gear_ratio

    def is_stalled(self, speed_threshold_dps: float = 1.0) -> bool:
        return abs(self._target_speed_dps) > speed_threshold_dps and abs(self._speed_dps) < speed_threshold_dps

    def hold(self, angle_deg: float | None = None) -> None:
        self._braking = False
        self._hold_angle_deg = (
            self._angle_deg if angle_deg is None else float(angle_deg)
        )
