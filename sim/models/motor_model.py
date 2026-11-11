
from .model import Model
from ..validation import number


class MotorModel(Model):
    '''
    Simulate raw motor shaft motion and gear output readings.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in seconds.
        time_constant_s (float): Read-only device response time constant in seconds.
        max_speed_dps (float): Positive raw motor shaft speed limit in deg/s.
        gear_ratio (float): Motor revolutions per gear output revolution.
        _angle_deg (float): Accumulated raw motor shaft angle in degrees.
        _speed_dps (float): Current raw motor shaft speed in deg/s.
        _target_speed_dps (float): Requested raw motor shaft speed after limiting, in
            deg/s.
        _braking (bool): Whether passive electrical braking is active.
        _hold_angle_deg (float | None): Raw shaft angle to hold in degrees, or None when
            released.
        _max_speed_dps (float): Stored value backing max_speed_dps.
        _gear_ratio (float): Stored positive value backing gear_ratio.
        angle (float): Read-only accumulated raw shaft angle in degrees.
        speed (float): Read-only raw shaft speed in deg/s.
        output_angle (float): Raw shaft angle divided by gear_ratio, in degrees.
        output_speed (float): Raw shaft speed divided by gear_ratio, in deg/s.
    '''
    def __init__(
            self,
            time_constant_s: float = 0.15,
            max_speed_dps: float = 1500.0,
            gear_ratio: float = 1.0
        ) -> None:
        '''
        Create the model.

        Arg:
        time_constant_s (float): Response time in seconds.
        max_speed_dps (float): Maximum motor speed in deg/s.
        gear_ratio (float): Motor revolutions per output revolution.
        '''
        number(max_speed_dps, 'max_speed_dps', positive=True)
        number(gear_ratio, 'gear_ratio', positive=True)
        super().__init__(time_constant_s)
        self.max_speed_dps: float = max_speed_dps
        self.gear_ratio: float = gear_ratio

        self._angle_deg: float = 0.0
        self._speed_dps: float = 0.0
        self._target_speed_dps: float = 0.0
        self._braking: bool = False
        self._hold_angle_deg: float | None = None

    def step(self, dt_s: float) -> None:
        '''
        Update shaft speed and angle with the motor response model.

        Apply position feedback when holding, or passive damping when braking.

        Arg:
            dt_s (float): Positive simulation step in seconds.
        '''
        number(dt_s, 'dt_s', positive=True)
        if self._hold_angle_deg is not None:
            error = self._hold_angle_deg - self._angle_deg
            requested_speed = 10.0 * error
            self._target_speed_dps = max(
                -self.max_speed_dps,
                min(self.max_speed_dps, requested_speed),
            )

        if self._braking:
            # damping coefficient
            alpha = min(1.0, dt_s / max(self.time_constant_s * 0.3, 1e-6))
            self._speed_dps += -self._speed_dps * alpha # exponential

        else:
            alpha = min(1.0, dt_s / max(self.time_constant_s, 1e-6))
            self._speed_dps += (self._target_speed_dps 
                                - self._speed_dps) * alpha

        self._angle_deg += self._speed_dps * dt_s

    def run(self, speed_dps: float) -> None:
        '''
        Set the motor shaft speed target and release braking or holding.

        Clamp the target to the motor shaft speed limit.

        Arg:
            speed_dps (float): Signed motor shaft speed in deg/s.
        '''
        number(speed_dps, 'speed_dps')
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
        '''
        Return accumulated motor shaft angle in degrees.
        '''
        return self._angle_deg

    @property
    def angle(self) -> float:
        '''
        Return accumulated motor shaft angle in degrees.
        '''
        return self.get_angle()

    def get_speed(self) -> float:
        '''
        Return motor shaft speed in deg/s.
        '''
        return self._speed_dps

    @property
    def speed(self) -> float:
        '''
        Return motor shaft speed in deg/s.
        '''
        return self.get_speed()

    @property
    def max_speed_dps(self) -> float:
        '''
        Return the positive motor shaft speed limit in deg/s.
        '''
        return self._max_speed_dps

    @max_speed_dps.setter
    def max_speed_dps(self, value: float) -> None:
        '''
        Set the positive motor shaft speed limit.

        Arg:
            value (float): Finite speed limit in deg/s.
        '''
        number(value, 'max_speed_dps', positive=True)
        self._max_speed_dps: float = value

    @property
    def gear_ratio(self) -> float:
        '''
        Return motor revolutions per gear output revolution.
        '''
        return self._gear_ratio

    @gear_ratio.setter
    def gear_ratio(self, value: float) -> None:
        '''
        Set the positive gear reduction ratio.

        Arg:
            value (float): Motor revolutions per gear output revolution.
        '''
        number(value, 'gear_ratio', positive=True)
        self._gear_ratio: float = value

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
        '''
        Check whether a commanded shaft is nearly stationary.

        This check has no time filter and may be True during startup.

        Arg:
            speed_threshold_dps (float): Non-negative motor shaft speed threshold.

        Returns:
            bool: Whether target speed is above the threshold and actual speed is below it.
        '''
        number(speed_threshold_dps, 'speed_threshold_dps', minimum=0)
        return (
            abs(self._target_speed_dps) > speed_threshold_dps 
            and abs(self._speed_dps) < speed_threshold_dps
        ) # A simple way to determine stall.

    def hold(self, angle_deg: float | None = None) -> None:
        '''
        Actively hold a motor shaft angle.

        Arg:
            angle_deg (float | None): Motor shaft target in degrees,
                or the current shaft angle when None.
        '''
        if angle_deg is not None:
            number(angle_deg, 'angle_deg')
        self._braking = False
        self._hold_angle_deg = (
            self._angle_deg if angle_deg is None else float(angle_deg)
        )
