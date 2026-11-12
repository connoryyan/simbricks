import math
import random
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .drive_base_model import DriveBaseModel

from .model import Model
from ..validation import boolean, integer, number


class IMUModel(Model):
    '''
    Integrate planar gyro readings and calibrate bias while stationary.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in
            seconds.
        time_constant_s (float): Read-only device response time constant in
            seconds.
        gravity_mm_s2 (float): Standard gravity used for upward acceleration,
            in mm/s/s.
        body (DriveBaseModel | None): Drive base body supplying motion, or None
            for an
            idle hub.
        bias_dps (float): Simulated raw gyro offset in deg/s.
        noise_dps (float): Gyro noise standard deviation in deg/s.
        _rng (random.Random): Seeded random generator used for simulated noise.
        _bias_estimate (float): Estimated gyro offset subtracted from raw
            readings, in
            deg/s.
        raw_rate (float): Latest right-hand gyro rate before bias correction,
            in deg/s.
        calibrated_rate (float): Latest right-hand gyro rate after bias
            correction, in
            deg/s.
        _previous_raw_rate (float | None): Previous gyro sample for variation
            checks, or
            None before sampling.
        raw_rotation (float): Accumulated right-hand raw rotation in degrees.
        calibrated_rotation (float): Accumulated right-hand bias-corrected
            rotation in
            degrees.
        _heading_offset (float): Clockwise heading offset set by
            reset_heading(), in
            degrees.
        heading_correction (float): Gyro scale denominator in degrees per full
            turn;
            normally 360.
        angular_velocity_threshold (float): Maximum gyro sample variation
            allowed for
            calibration, in deg/s.
        acceleration_threshold (float): Maximum horizontal acceleration allowed
            for
            calibration, in mm/s/s.
        _stationary_s (float): Current consecutive stationary duration in
            seconds.
        _window_s (float): Elapsed time in the current bias calibration window,
            in
            seconds.
        _bias_sum (float): Integral of raw gyro rate over the calibration
            window, in
            degrees.
        _calibration_count (int): Number of bias updates used by the current
            running
            average.
        _elapsed (float): Total time processed by this IMU in seconds.
        _last_calibration (float): Time of the last bias update in seconds;
            negative
            infinity before calibration.
        _acceleration (tuple): Forward, left and upward acceleration in mm/s/s,
            including gravity.
        in_use (bool): Whether active gyro control blocks heading resets and
            settings
            changes.
        _last_yaw (float): Most recently sampled true body heading in
            counter-clockwise
            radians.
        _last_speed (float): Most recently sampled forward body speed in mm/s.
    '''
    step_priority: float = 1.5
    gravity_mm_s2: float = 9806.65

    def __init__(
        self,
        body: 'DriveBaseModel | None' = None,
        *,
        bias_dps: float = 0.0,
        noise_dps: float = 0.0,
        seed: int | None = None,
    ) -> None:
        '''
        Create an IMU at the axle centre in the robot reference frame.

        Arg:
            body (DriveBaseModel | None): Physical body, or None for an idle
                hub.
            bias_dps (float): Simulated raw z gyro bias in deg/s.
            noise_dps (float): Gyro noise standard deviation in deg/s.
            seed (int | None): Random seed for repeatable measurements.
        '''
        from .drive_base_model import DriveBaseModel
        if body is not None and not isinstance(body, DriveBaseModel):
            raise TypeError('body must be a DriveBaseModel or None')
        number(bias_dps, 'bias_dps')
        number(noise_dps, 'noise_dps', minimum=0)
        if seed is not None:
            integer(seed, 'seed')
        self.body: DriveBaseModel | None = body
        self.bias_dps: float = bias_dps
        self.noise_dps: float = noise_dps
        self._rng: random.Random = random.Random(seed)
        self._bias_estimate: float = 0.0
        self.raw_rate: float = 0.0
        self.calibrated_rate: float = 0.0
        self._previous_raw_rate: float | None = None
        self.raw_rotation: float = 0.0
        self.calibrated_rotation: float = 0.0
        self._heading_offset: float = 0.0
        self.heading_correction: float = 360.0
        self.angular_velocity_threshold: float = 2.0
        self.acceleration_threshold: float = 2500.0
        self._stationary_s: float = 0.0
        self._window_s: float = 0.0
        self._bias_sum: float = 0.0
        self._calibration_count: int = 0
        self._elapsed: float = 0.0
        self._last_calibration: float = -math.inf
        self._acceleration: tuple[float, float, float] = (
            0.0, 0.0, self.gravity_mm_s2)
        self.in_use: bool = False
        self.rebase_pose()
        super().__init__()

    def rebase_pose(self) -> None:
        '''
        Update the IMU reference after placement so it does not count as a
            turn.
        '''
        self._last_yaw: float = (
            self.body.heading_rad if self.body is not None else 0.0
        )
        self._last_speed: float = (
            self.body.speed_mm_s() if self.body is not None else 0.0
        )

    def step(self, dt_s: float) -> None:
        '''
        Integrate gyro data and update bias after each stationary second.
        '''
        number(dt_s, 'dt_s', positive=True)
        number(self.bias_dps, 'bias_dps')
        number(self.noise_dps, 'noise_dps', minimum=0)
        number(self.angular_velocity_threshold,
               'angular_velocity_threshold', positive=True)
        number(self.acceleration_threshold,
               'acceleration_threshold', positive=True)
        yaw = (self.body.heading_rad if self.body is not None
               else self._last_yaw)
        true_rate = math.degrees(yaw - self._last_yaw) / dt_s
        speed = self.body.speed_mm_s() if self.body is not None else 0.0
        ax, ay = (self.body.acceleration_mm_s2() if self.body is not None
                  else (0.0, 0.0))
        self._acceleration = (ax, ay, self.gravity_mm_s2)
        self._last_yaw, self._last_speed = yaw, speed
        self._elapsed += dt_s
        # Add bias and noise.
        self.raw_rate = true_rate + self.bias_dps + \
            self._rng.gauss(0, self.noise_dps)
        variation = (0.0 if self._previous_raw_rate is None
                     else abs(self.raw_rate - self._previous_raw_rate))
        self._previous_raw_rate = self.raw_rate
        self.calibrated_rate = self.raw_rate - self._bias_estimate
        self.raw_rotation += self.raw_rate * dt_s
        self.calibrated_rotation += self.calibrated_rate * dt_s

        # The planar simulator also checks translation, so steady driving
        # cannot accidentally calibrate the gyro.
        # This simplifies the actual stationary condition.
        wheels_idle = (
            self.body is None or (
                abs(self.body._left.speed) < 0.1
                and abs(self.body._right.speed) < 0.1
            )
        )
        body_speed = math.hypot(
            *self.body.velocity_mm_s()) if self.body is not None else 0.0
        stationary = (body_speed < 0.1 and abs(true_rate) < 0.1
                      and wheels_idle
                      and variation < self.angular_velocity_threshold
                      and math.hypot(ax, ay) < self.acceleration_threshold)
        if not stationary:
            # Calibrate reading when stationary. Reset stationary time.
            self._stationary_s = self._window_s = self._bias_sum = 0.0
            return
        self._stationary_s += dt_s
        self._window_s += dt_s
        self._bias_sum += self.raw_rate * dt_s
        if self._window_s >= 1.0 - 1e-9:
            if not self.ready():
                self._calibration_count = 0
            self._calibration_count += 1
            # Same decreasing average weight as the official stationary
            # callback.
            weight = 1 / min(self._calibration_count, 20)
            self._bias_estimate += weight * \
                (self._bias_sum / self._window_s - self._bias_estimate)
            self._last_calibration = self._elapsed
            self._window_s = self._bias_sum = 0.0

    def heading(self) -> float:
        '''
        Return continuous clockwise heading in degrees.
        '''
        number(self.heading_correction, 'heading_correction', positive=True)
        return -self.calibrated_rotation * 360 / \
            self.heading_correction - self._heading_offset

    def reset_heading(self, angle: float = 0.0) -> None:
        '''
        Set the reported heading without changing integration or physical pose.
        '''
        number(angle, 'angle')
        boolean(self.in_use, 'in_use')
        if self.in_use:
            raise RuntimeError(
                'Stop the gyro drive base before resetting its heading')
        self._heading_offset += self.heading() - angle

    def ready(self) -> bool:
        '''
        Return True after calibration within the last ten minutes.
        '''
        return self._elapsed - self._last_calibration < 600

    def stationary(self) -> bool:
        '''
        Return True after at least one stationary second.
        '''
        return self._stationary_s >= 1.0 - 1e-9
