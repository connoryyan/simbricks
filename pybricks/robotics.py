import math

from sim.clock import clock

from .parameters import Stop

KP_DISTANCE = 3.0 # 1/s, feedback gain on distance error
KP_ANGLE = 3.0 # 1/s, feedback gain on angle error
DISTANCE_TOL_MM = 1.0
ANGLE_TOL_DEG = 0.5
SETTLE_TIMEOUT_S = 3.0 # give up this long after the planned end of a move


class _Profile:

    def __init__(self, total: float, vmax: float, accel: float):
        self.sign = 1.0 if total >= 0 else -1.0
        self.dist = abs(total)
        self.accel = accel
        if self.dist == 0:
            self.v_peak = self.t_acc = self.t_cruise = self.duration = 0.0
            return
        # Not enough room to reach vmax -> triangular profile.
        self.v_peak = min(vmax, math.sqrt(accel * self.dist)) # v^2 = 2ax
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
    """One straight() or turn() in progress."""

    def __init__(self, d0, a0, dist_profile, angle_profile, then):
        self.d0, self.a0 = d0, a0  # odometry at the start of the move
        self.dist_profile = dist_profile
        self.angle_profile = angle_profile
        self.then = then
        self.t = 0.0
        self.duration = max(dist_profile.duration, angle_profile.duration)


class DriveBase:
    def __init__(self, left_motor, right_motor, wheel_diameter: float,
                 axle_track: float, use_gyro: bool = False):
        if wheel_diameter <= 0 or axle_track <= 0:
            raise ValueError("wheel_diameter and axle_track must be positive")

        self._left = left_motor
        self._right = right_motor
        self._mm_per_deg = math.pi * wheel_diameter / 360.0
        self._track = axle_track
        self.use_gyro = use_gyro

        # Default limits (simulation defaults, not the real firmware's).
        self._straight_speed = 200.0 # mm/s
        self._straight_accel = 700.0 # mm/s^2
        self._turn_rate = 200.0 # deg/s
        self._turn_accel = 1000.0 # deg/s^2

        # True pose in the world frame.
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0 # radians, counter-clockwise

        # Odometry as Pybricks reports it.
        self._distance = 0.0 # mm
        self._angle = 0.0 # deg, clockwise positive

        self._last_l = left_motor.angle()
        self._last_r = right_motor.angle()
        self._maneuver: _Maneuver | None = None
        self._mounts: list = []

        clock.add_steppable(self)

    def step(self, dt_s: float) -> None:
        self._update_odometry()
        if self._maneuver is not None:
            self._run_maneuver(dt_s)
        self._update_mounts()

    def _update_odometry(self) -> None:
        l, r = self._left.angle(), self._right.angle()
        dl = (l - self._last_l) * self._mm_per_deg
        dr = (r - self._last_r) * self._mm_per_deg
        self._last_l, self._last_r = l, r

        ds = (dl + dr) / 2.0
        dtheta_cw = (dl - dr) / self._track  # radians, clockwise positive

        mid_heading = self._heading - dtheta_cw / 2.0
        self._x += ds * math.cos(mid_heading)
        self._y += ds * math.sin(mid_heading)
        self._heading -= dtheta_cw

        self._distance += ds
        self._angle += math.degrees(dtheta_cw)

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
        """Convert body speed + turn rate (clockwise positive) into wheel commands."""
        w = math.radians(turn_rate_dps)
        v_left = speed_mm_s + w * self._track / 2.0
        v_right = speed_mm_s - w * self._track / 2.0
        self._left.run(v_left / self._mm_per_deg)
        self._right.run(v_right / self._mm_per_deg)

    def _apply_then(self, then: Stop) -> None:
        if then in (Stop.COAST, Stop.COAST_SMART):
            self._left.stop()
            self._right.stop()
        else:
            # leave HOLD later
            self._left.brake()
            self._right.brake()

    # ------------------------------------------------------------------
    # Official API
    # ------------------------------------------------------------------
    def drive(self, speed: float, turn_rate: float) -> None:
        self._maneuver = None
        self._set_wheels(speed, turn_rate)

    def stop(self) -> None:
        self._maneuver = None
        self._left.stop()
        self._right.stop()

    def brake(self) -> None:
        self._maneuver = None
        self._left.brake()
        self._right.brake()

    def straight(self, distance: float, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        self._start(distance, 0.0, then, wait)

    def turn(self, angle: float, then: Stop = Stop.HOLD, wait: bool = True) -> None: 
        self._start(0.0, angle, then, wait)

    def done(self) -> bool:
        return self._maneuver is None

    def stalled(self) -> bool:
        return False  # no load/torque model yet

    def distance(self) -> float:
        return self._distance

    def angle(self) -> float:
        return self._angle

    def state(self) -> tuple[float, float, float, float]:
        vl = self._left.speed() * self._mm_per_deg
        vr = self._right.speed() * self._mm_per_deg
        speed = (vl + vr) / 2.0
        turn_rate = math.degrees((vl - vr) / self._track)
        return self._distance, speed, self._angle, turn_rate

    def reset(self, distance: float = 0, angle: float = 0) -> None:
        self._distance = float(distance)
        self._angle = float(angle)

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

    # ------------------------------------------------------------------
    # Simulator-only helpers (not part of the official API)
    # ------------------------------------------------------------------
    def place(self, x_mm: float, y_mm: float, heading_deg: float = 0.0) -> None:
        """Put the robot somewhere in the world (heading counter-clockwise from +x)."""
        self._x, self._y = x_mm, y_mm
        self._heading = math.radians(heading_deg)
        self._update_mounts()

    def pose(self) -> tuple[float, float, float]:
        """True pose: (x_mm, y_mm, heading_deg counter-clockwise)."""
        return self._x, self._y, math.degrees(self._heading)

    def mount(self, sensor, forward_mm: float = 0.0, left_mm: float = 0.0) -> None:
        """Attach a sensor to the robot body at an offset from the axle center.

        The sensor must implement _set_pose(x_mm, y_mm, heading_deg).
        """
        self._mounts.append((sensor, forward_mm, left_mm))
        self._update_mounts()

    def _update_mounts(self) -> None:
        c, s = math.cos(self._heading), math.sin(self._heading)
        heading_deg = math.degrees(self._heading)
        for sensor, forward, left in self._mounts:
            x = self._x + forward * c - left * s
            y = self._y + forward * s + left * c
            sensor._set_pose(x, y, heading_deg)

    # ------------------------------------------------------------------
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
        if wait:
            timeout_ms = (m.duration + SETTLE_TIMEOUT_S + 1.0) * 1000.0
            clock.advance_until(self.done, timeout_ms=timeout_ms)