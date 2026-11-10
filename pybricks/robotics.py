from sim.initialization import requires_init
import math

from sim.clock import clock
from sim.models import DriveBaseModel

from .parameters import Stop
from .pupdevices import Motor

KP_DISTANCE = 3.0 # 1/s, feedback gain on distance error
KP_ANGLE = 3.0 # 1/s, feedback gain on angle error
DISTANCE_TOL_MM = 1.0
ANGLE_TOL_DEG = 0.5
SETTLE_TIMEOUT_S = 3.0 # give up this long after the planned end of a move


class _Profile:
    def __init__(self, total: float, vmax: float, accel: float):
        if not all(math.isfinite(v) for v in (total, vmax, accel)):
            raise ValueError("Profile values must be finite")
        if vmax <= 0 or accel <= 0:
            raise ValueError("vmax and accel must be positive")

        self.sign = 1.0 if total >= 0 else -1.0
        self.dist = abs(total)
        self.accel = accel
        if self.dist == 0:
            self.v_peak = self.t_acc = self.t_cruise = self.duration = 0.0
            return
        # Not enough room to reach vmax -> triangular profile.
        self.v_peak = min(vmax, math.sqrt(accel * self.dist))
        self.t_acc = self.v_peak / accel
        d_acc = 0.5 * accel * self.t_acc ** 2
        self.t_cruise = max(0.0, (self.dist - 2 * d_acc) / self.v_peak)
        self.duration = 2 * self.t_acc + self.t_cruise

    def at(self, t: float) -> tuple[float, float]:
        if self.dist == 0 or t >= self.duration:
            return self.sign * self.dist, 0.0
        a = self.accel
        if t < self.t_acc:
            pos, vel = 0.5 * a * t * t, a * t
        elif t < self.t_acc + self.t_cruise:
            d_acc = 0.5 * a * self.t_acc ** 2
            pos, vel = d_acc + self.v_peak * (t - self.t_acc), self.v_peak
        else:
            td = self.duration - t
            pos, vel = self.dist - 0.5 * a * td * td, a * td
        return self.sign * pos, self.sign * vel


class _Maneuver:
    '''
    Store a straight or turn action.
    '''

    def __init__(self, d0, a0, dist_profile, angle_profile, then):
        self.d0, self.a0 = d0, a0 # odometry at the start of the move
        self.dist_profile = dist_profile
        self.angle_profile = angle_profile
        self.then = then
        self.t = 0.0
        self.duration = max(dist_profile.duration, angle_profile.duration)


class DriveBase:
    step_priority = 2

    def __init__(self, left_motor, right_motor, wheel_diameter: float,
                 axle_track: float):
        if not isinstance(left_motor, Motor) or not isinstance(right_motor, Motor):
            raise TypeError("left_motor and right_motor must be Motor objects")
        self._left = left_motor
        self._right = right_motor
        # The body model validates the geometry and registers itself on the clock.
        self._model = DriveBaseModel(
            left_model=left_motor.model,
            right_model=right_motor.model,
            wheel_diameter=wheel_diameter,
            axle_track=axle_track,
            left_sign=left_motor.positive_direction.value,
            right_sign=right_motor.positive_direction.value,
        )
        self._mm_per_deg = self._model.mm_per_deg
        self._track = self._model.axle_track
        # A real hub has one IMU shared by hub.imu and drive base control.
        from sim.hub import get_active_hub
        hub = get_active_hub()
        if hub is not None:
            try:
                hub.bind_body(self._model)
            except ValueError:
                self._model.detach()
                raise
        self._use_gyro = False

        # Default limits (simulation defaults, not the real firmware's).
        self._straight_speed = 200.0  # mm/s
        self._straight_accel = 700.0  # mm/s^2
        self._turn_rate = 200.0       # deg/s
        self._turn_accel = 1000.0     # deg/s^2

        # reset() only moves these offsets; the model's totals are never reset.
        self._distance_offset = 0.0
        self._angle_offset = 0.0

        self._maneuver: _Maneuver | None = None

        clock.add_steppable(self)

    @property
    def model(self) -> DriveBaseModel:
        '''
        Return the body model for the simulator.
        '''
        return self._model

    # Odometry as Pybricks reports it: the model's running totals minus the
    # offsets set by reset().
    @property
    def _distance(self) -> float:
        return self._model.distance_mm - self._distance_offset

    @property
    def _angle(self) -> float:
        if self._use_gyro:
            return self._model.imu.heading()
        return self._model.angle_deg - self._angle_offset

    def use_gyro(self, use_gyro: bool) -> None:
        '''
        Stop the robot and select gyro or wheel heading feedback.
        '''
        self.stop()
        self._use_gyro = bool(use_gyro)
        if self._use_gyro:
            self._model.imu

    def step(self, dt_s: float) -> None:
        if self._maneuver is not None:
            self._run_maneuver(dt_s)

    def _run_maneuver(self, dt_s: float) -> None:
        m = self._maneuver
        m.t += dt_s

        d_ref, v_ref = m.dist_profile.at(m.t)
        a_ref, w_ref = m.angle_profile.at(m.t)
        d_err = (m.d0 + d_ref) - self._distance
        a_err = (m.a0 + a_ref) - self._angle

        # Feed-forward from the reference plus feedback on the tracking error.
        self._set_wheels(v_ref + KP_DISTANCE * d_err, w_ref + KP_ANGLE * a_err)

        planned_done = m.t >= m.duration
        settled = abs(d_err) < DISTANCE_TOL_MM and abs(a_err) < ANGLE_TOL_DEG
        timed_out = m.t >= m.duration + SETTLE_TIMEOUT_S
        if planned_done and (settled or timed_out):
            self._maneuver = None
            self._apply_then(m.then)

    def _set_wheels(self, speed_mm_s: float, turn_rate_dps: float) -> None:
        '''
        Convert body speed and clockwise turn rate into wheel commands.

        Arg:
            speed_mm_s (float): Body speed in mm/s.
            turn_rate_dps (float): Body turn rate in deg/s.
        '''
        w = math.radians(turn_rate_dps)
        v_left = speed_mm_s + w * self._track / 2.0
        v_right = speed_mm_s - w * self._track / 2.0
        self._left.run(v_left / self._mm_per_deg)
        self._right.run(v_right / self._mm_per_deg)

    def _apply_then(self, then: Stop) -> None:
        if self._model._imu is not None:
            self._model.imu.in_use = self._use_gyro and then == Stop.HOLD
        if then in (Stop.COAST, Stop.COAST_SMART):
            self._left.stop()
            self._right.stop()
        else:
            # leave HOLD later
            self._left.brake()
            self._right.brake()

    @requires_init
    def drive(self, speed: float, turn_rate: float) -> None:
        self._maneuver = None
        if self._use_gyro:
            self._model.imu.in_use = True
        self._set_wheels(speed, turn_rate)

    def stop(self) -> None:
        self._maneuver = None
        if self._model._imu is not None:
            self._model.imu.in_use = False
        self._left.stop()
        self._right.stop()

    def brake(self) -> None:
        self._maneuver = None
        if self._model._imu is not None:
            self._model.imu.in_use = False
        self._left.brake()
        self._right.brake()

    @requires_init
    def straight(self, distance: float, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        self._start(distance, 0.0, then, wait)

    @requires_init
    def turn(self, angle: float, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        self._start(0.0, angle, then, wait)

    @requires_init
    def done(self) -> bool:
        return self._maneuver is None

    def stalled(self) -> bool:
        return False  # no load/torque model yet

    @requires_init
    def distance(self) -> float:
        return self._distance

    @requires_init
    def angle(self) -> float:
        return self._angle

    @requires_init
    def state(self) -> tuple[float, float, float, float]:
        turn_rate = (-self._model.imu.calibrated_rate if self._use_gyro
                     else self._model.encoder_turn_rate_dps())
        return (self._distance, self._model.encoder_speed_mm_s(),
                self._angle, turn_rate)

    def reset(self, distance: float = 0, angle: float = 0) -> None:
        '''
        Reset the estimated distance and angle.
        '''
        self.brake()
        self._distance_offset = self._model.distance_mm - float(distance)
        self._angle_offset = self._model.angle_deg - float(angle)
        if self._use_gyro:
            self._model.imu.reset_heading(float(angle))

    def settings(self, straight_speed=None, straight_acceleration=None,
                 turn_rate=None, turn_acceleration=None):
        if (straight_speed, straight_acceleration, turn_rate, turn_acceleration) == (None,) * 4:
            return (self._straight_speed, self._straight_accel,
                    self._turn_rate, self._turn_accel)
        if straight_speed is not None:
            self._straight_speed = float(straight_speed)
        if straight_acceleration is not None:
            self._straight_accel = float(straight_acceleration)
        if turn_rate is not None:
            self._turn_rate = float(turn_rate)
        if turn_acceleration is not None:
            self._turn_accel = float(turn_acceleration)

    ##########################################################################
    # Simulator-only helpers (not part of the official API).
    # They just forward to the body model.
    ##########################################################################
    def place(self, x_mm: float, y_mm: float, heading_deg: float = 0.0) -> None:
        self._model.place(x_mm, y_mm, heading_deg)

    def pose(self) -> tuple[float, float, float]:
        return self._model.pose()

    def _start(self, distance: float, angle: float, then: Stop, wait: bool) -> None:
        # Hold the other axis at its starting value (straight keeps heading,
        # turn keeps position) by giving it a zero-length profile.
        m = _Maneuver(
            self._distance, self._angle,
            _Profile(distance, self._straight_speed, self._straight_accel),
            _Profile(angle, self._turn_rate, self._turn_accel),
            then,
        )
        self._maneuver = m
        if self._use_gyro:
            self._model.imu.in_use = True
        if wait:
            timeout_ms = (m.duration + SETTLE_TIMEOUT_S + 1.0) * 1000.0
            clock.advance_until(self.done, timeout_ms=timeout_ms)
