import math
import random

from .model import Model


class IMUModel(Model):
    '''
    Simulate a level IMU with gyro integration and stationary bias calibration.
    '''
    step_priority = 1.5
    gravity_mm_s2 = 9806.65

    def __init__(self, body=None, *, bias_dps=0.0, noise_dps=0.0, seed=None):
        '''
        Create an IMU at the axle centre in the robot reference frame.

        Arg:
            body (DriveBaseModel | None): Physical body, or None for an idle hub.
            bias_dps (float): Simulated raw z gyro bias in deg/s.
            noise_dps (float): Gyro noise standard deviation in deg/s.
            seed (int | None): Random seed for repeatable measurements.
        '''
        if not all(math.isfinite(v) for v in (bias_dps, noise_dps)) or noise_dps < 0:
            raise ValueError('Gyro bias and noise must be finite; noise must be non-negative')
        self.body = body
        self.bias_dps = bias_dps
        self.noise_dps = noise_dps
        self._rng = random.Random(seed)
        self._bias_estimate = 0.0
        self.raw_rate = self.calibrated_rate = 0.0
        self._previous_raw_rate = None
        self.raw_rotation = self.calibrated_rotation = 0.0
        self._heading_offset = 0.0
        self.heading_correction = 360.0
        self.angular_velocity_threshold = 2.0
        self.acceleration_threshold = 2500.0
        self._stationary_s = self._window_s = self._bias_sum = 0.0
        self._calibration_count = 0
        self._elapsed = 0.0
        self._last_calibration = -math.inf
        self._acceleration = (0.0, 0.0, self.gravity_mm_s2)
        self.in_use = False
        self.rebase_pose()
        super().__init__()

    def rebase_pose(self):
        '''
        Treat simulator placement as setup without adding a gyro rotation.
        '''
        self._last_yaw = self.body.heading_rad if self.body is not None else 0.0
        self._last_speed = self.body.speed_mm_s() if self.body is not None else 0.0

    def step(self, dt_s):
        '''
        Integrate gyro data and update bias after each stationary second.
        '''
        if dt_s <= 0:
            raise ValueError('IMU step must be positive')
        yaw = self.body.heading_rad if self.body is not None else self._last_yaw
        true_rate = math.degrees(yaw-self._last_yaw) / dt_s
        speed = self.body.speed_mm_s() if self.body is not None else 0.0
        ax, ay = self.body.acceleration_mm_s2() if self.body is not None else (0.0, 0.0)
        self._acceleration = (ax, ay, self.gravity_mm_s2)
        self._last_yaw, self._last_speed = yaw, speed
        self._elapsed += dt_s
        self.raw_rate = true_rate + self.bias_dps + self._rng.gauss(0, self.noise_dps)
        variation = (0.0 if self._previous_raw_rate is None
                     else abs(self.raw_rate-self._previous_raw_rate))
        self._previous_raw_rate = self.raw_rate
        self.calibrated_rate = self.raw_rate-self._bias_estimate
        self.raw_rotation += self.raw_rate*dt_s
        self.calibrated_rotation += self.calibrated_rate*dt_s

        # The planar simulator also checks translation, so steady driving cannot
        # accidentally calibrate the gyro. This replaces the hardware idle gate.
        wheels_idle = (self.body is None or
                       (abs(self.body._left.speed) < 0.1 and abs(self.body._right.speed) < 0.1))
        body_speed = math.hypot(*self.body.velocity_mm_s()) if self.body is not None else 0.0
        stationary = (body_speed < 0.1 and abs(true_rate) < 0.1
                      and wheels_idle
                      and variation < self.angular_velocity_threshold
                      and math.hypot(ax, ay) < self.acceleration_threshold)
        if not stationary:
            self._stationary_s = self._window_s = self._bias_sum = 0.0
            return
        self._stationary_s += dt_s
        self._window_s += dt_s
        self._bias_sum += self.raw_rate*dt_s
        if self._window_s >= 1.0-1e-9:
            if not self.ready():
                self._calibration_count = 0
            self._calibration_count += 1
            # Same decreasing average weight as the official stationary callback.
            weight = 1 / min(self._calibration_count, 20)
            self._bias_estimate += weight*(self._bias_sum/self._window_s-self._bias_estimate)
            self._last_calibration = self._elapsed
            self._window_s = self._bias_sum = 0.0

    def heading(self):
        '''
        Return continuous clockwise heading in degrees.
        '''
        return -self.calibrated_rotation*360/self.heading_correction-self._heading_offset

    def reset_heading(self, angle=0.0):
        '''
        Set the reported heading without changing integration or physical pose.
        '''
        if not math.isfinite(angle):
            raise ValueError('Heading must be finite')
        if self.in_use:
            raise OSError('Stop the gyro drive base before resetting its heading')
        self._heading_offset += self.heading()-angle

    def ready(self):
        '''
        Return True after calibration within the last ten minutes.
        '''
        return self._elapsed-self._last_calibration < 600

    def stationary(self):
        '''
        Return True after at least one stationary second.
        '''
        return self._stationary_s >= 1.0-1e-9
