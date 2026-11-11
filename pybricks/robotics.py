from sim.initialization import requires_init
import math

from sim.clock import clock
from sim.models import DriveBaseModel
from sim.validation import boolean, number

from .parameters import Stop
from .pupdevices import Motor

KP_DISTANCE = 3.0 # 1/s, feedback gain on distance error
KP_ANGLE = 3.0 # 1/s, feedback gain on angle error
DISTANCE_TOL_MM = 1.0
ANGLE_TOL_DEG = 0.5
SETTLE_TIMEOUT_S = 3.0 # give up this long after the planned end of a move


class _Profile:
    '''
    Store a triangular or trapezoidal motion reference.

    Attributes:
        sign (float): Travel direction, either -1 or 1.
        dist (float): Absolute target distance or angle in the selected units.
        accel (float): Positive acceleration in units per second squared.
        v_peak (float): Peak speed reached by the profile, in units per second.
        t_acc (float): Duration of acceleration in seconds.
        t_cruise (float): Duration of constant-speed travel in seconds.
        duration (float): Total planned duration in seconds.
    '''
    def __init__(self, total: float, vmax: float, accel: float) -> None:
        '''
        Create a triangular or trapezoidal motion profile.

        Arg:
            total (float): Signed distance or angle to travel.
            vmax (float): Positive maximum speed in units per second.
            accel (float): Positive acceleration in units per second squared.
        '''
        number(total, 'total')
        number(vmax, 'vmax', positive=True)
        number(accel, 'accel', positive=True)

        self.sign: float = 1.0 if total >= 0 else -1.0
        self.dist: float = abs(total)
        self.accel: float = accel
        if self.dist == 0:
            self.v_peak: float = 0.0
            self.t_acc: float = 0.0
            self.t_cruise: float = 0.0
            self.duration: float = 0.0
            return
        # Not enough room to reach vmax -> triangular profile.
        self.v_peak = min(vmax, math.sqrt(accel) * math.sqrt(self.dist))
        self.t_acc = self.v_peak / accel
        d_acc = 0.5 * self.v_peak * self.t_acc
        self.t_cruise = max(0.0, (self.dist - 2 * d_acc) / self.v_peak)
        self.duration = 2 * self.t_acc + self.t_cruise
        number(self.duration, 'Profile duration', positive=True)

    def at(self, t: float) -> tuple[float, float]:
        '''
        Return the reference position and speed at a time.

        Arg:
            t (float): Non-negative elapsed time in seconds.

        Returns:
            tuple[float, float]: Signed position and speed.
        '''
        number(t, 't', minimum=0)
        if self.dist == 0 or t >= self.duration:
            return self.sign * self.dist, 0.0
        a = self.accel
        if t < self.t_acc:
            pos, vel = 0.5 * a * t * t, a * t
        elif t < self.t_acc + self.t_cruise:
            d_acc = 0.5 * self.v_peak * self.t_acc
            pos, vel = d_acc + self.v_peak * (t - self.t_acc), self.v_peak
        else:
            td = self.duration - t
            pos, vel = self.dist - 0.5 * a * td * td, a * td
        return self.sign * pos, self.sign * vel


class DriveBase:
    '''
    Control two wheel motors using encoder or gyro feedback.

    Attributes:
        step_priority (int): Clock update phase; smaller phases run first.
        _left (Motor): Motor driving the left wheel.
        _right (Motor): Motor driving the right wheel.
        _model (DriveBaseModel): Physical body model used for motion and sensor
            mounting.
        _mm_per_deg (float): Wheel travel in mm per degree of gear output rotation.
        _track (float): Distance between wheel contact points in mm.
        _use_gyro (bool): Whether heading feedback comes from the IMU.
        _straight_speed (float): Straight motion speed limit in mm/s.
        _straight_accel (float): Straight motion acceleration limit in mm/s/s.
        _turn_rate (float): Turn rate limit in deg/s.
        _turn_accel (float): Turn acceleration limit in deg/s/s.
        _distance_offset (float): Offset subtracted from encoder distance, in mm.
        _angle_offset (float): Offset subtracted from encoder heading, in degrees.
        _maneuver (dict): Active move data: start values, profiles, stop mode and times;
            None when inactive.
        model (DriveBaseModel): Physical body model exposed for simulator setup.
        _distance (float): Reported encoder travel after the offset, in mm.
        _angle (float): Reported clockwise heading from the selected feedback source.
    '''
    step_priority: int = 2

    def __init__(self, left_motor: Motor, right_motor: Motor, wheel_diameter: float,
                 axle_track: float) -> None:
        '''
        Create a drive base from two wheel motors.

        Arg:
            left_motor (Motor): Motor driving the left wheel.
            right_motor (Motor): Motor driving the right wheel.
            wheel_diameter (float): Positive wheel diameter in mm.
            axle_track (float): Positive distance between wheel contacts in mm.
        '''
        if not isinstance(left_motor, Motor) or not isinstance(right_motor, Motor):
            raise TypeError("left_motor and right_motor must be Motor objects")
        self._left: Motor = left_motor
        self._right: Motor = right_motor

        # The body model validates the geometry and registers itself on the clock.
        self._model: DriveBaseModel = DriveBaseModel(
            left_model=left_motor.model,
            right_model=right_motor.model,
            wheel_diameter=wheel_diameter,
            axle_track=axle_track,
            left_sign=left_motor.positive_direction.value,
            right_sign=right_motor.positive_direction.value,
        )

        self._mm_per_deg: float = self._model.mm_per_deg
        self._track: float = self._model.axle_track

        # A real hub has one IMU shared by hub.imu and drive base control.
        from sim.hub import get_active_hub
        hub = get_active_hub()
        if hub is not None:
            try:
                hub.bind_body(self._model)
            except ValueError:
                self._model.detach()
                raise
        self._use_gyro: bool = False

        # Default limits (simulation defaults, not the real firmware's).
        self._straight_speed: float = 200.0  # mm/s
        self._straight_accel: float = 700.0  # mm/s^2
        self._turn_rate: float = 200.0       # deg/s
        self._turn_accel: float = 1000.0     # deg/s^2

        # reset() only moves these offsets; the model's totals are never reset.
        self._distance_offset: float = 0.0
        self._angle_offset: float = 0.0

        self._maneuver: dict[str, float | _Profile | Stop] | None = None

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
        '''
        Return encoder distance after the reset offset in mm.
        '''
        return self._model.distance_mm - self._distance_offset

    @property
    def _angle(self) -> float:
        '''
        Return clockwise heading from the selected feedback source.
        '''
        if self._use_gyro:
            return self._model.imu.heading()
        return self._model.angle_deg - self._angle_offset

    def use_gyro(self, use_gyro: bool) -> None:
        '''
        Stop the robot and select its heading feedback.

        Arg:
            use_gyro (bool): Use IMU heading when True, or wheel odometry when False.
        '''
        boolean(use_gyro, 'use_gyro')
        self.stop()
        self._use_gyro = use_gyro
        if self._use_gyro:
            self._model.imu

    def step(self, dt_s: float) -> None:
        '''
        Update the active drive base maneuver.

        Arg:
            dt_s (float): Positive simulation step in seconds.
        '''
        number(dt_s, 'dt_s', positive=True)
        if self._maneuver is not None:
            self._run_maneuver(dt_s)

    def _run_maneuver(self, dt_s: float) -> None:
        '''
        Track motion references and finish the active maneuver.

        Arg:
            dt_s (float): Positive time since the previous update in seconds.
        '''
        number(dt_s, 'dt_s', positive=True)
        m = self._maneuver
        if m is None:
            raise RuntimeError('No active maneuver')
        m['t'] += dt_s

        d_ref, v_ref = m['dist_profile'].at(m['t'])
        a_ref, w_ref = m['angle_profile'].at(m['t'])
        d_err = (m['d0'] + d_ref) - self._distance
        a_err = (m['a0'] + a_ref) - self._angle

        # Feed-forward from the reference plus feedback on the tracking error.
        self._set_wheels(v_ref + KP_DISTANCE * d_err, w_ref + KP_ANGLE * a_err)

        planned_done = m['t'] >= m['duration']
        settled = abs(d_err) < DISTANCE_TOL_MM and abs(a_err) < ANGLE_TOL_DEG
        timed_out = m['t'] >= m['duration'] + SETTLE_TIMEOUT_S
        if planned_done and (settled or timed_out):
            self._maneuver = None
            self._apply_then(m['then'])

    def _set_wheels(self, speed_mm_s: float, turn_rate_dps: float) -> None:
        '''
        Convert body speed and clockwise turn rate into wheel commands.

        Arg:
            speed_mm_s (float): Body speed in mm/s.
            turn_rate_dps (float): Body turn rate in deg/s.
        '''
        number(speed_mm_s, 'speed_mm_s')
        number(turn_rate_dps, 'turn_rate_dps')
        w = math.radians(turn_rate_dps)
        v_left = speed_mm_s + w * self._track / 2.0
        v_right = speed_mm_s - w * self._track / 2.0
        number(v_left / self._mm_per_deg * self._left.model.gear_ratio, 'left shaft speed')
        number(v_right / self._mm_per_deg * self._right.model.gear_ratio, 'right shaft speed')
        self._left.run(v_left / self._mm_per_deg)
        self._right.run(v_right / self._mm_per_deg)

    def _apply_then(self, then: Stop) -> None:
        '''
        Apply a stop mode to both wheels after a move.

        Arg:
            then (Stop): Coast, brake or actively hold both wheel positions.
        '''
        if not isinstance(then, Stop):
            raise TypeError('then must be a Stop')
        if self._model._imu is not None:
            self._model.imu.in_use = self._use_gyro and then == Stop.HOLD
        if then in (Stop.COAST, Stop.COAST_SMART):
            self._left.stop()
            self._right.stop()
        elif then == Stop.HOLD:
            self._left.hold()
            self._right.hold()
        else:
            self._left.brake()
            self._right.brake()

    @requires_init
    def drive(self, speed: float, turn_rate: float) -> None:
        '''
        Drive at a forward speed and clockwise turn rate.

        Cancel any active straight or turn maneuver.

        Arg:
            speed (float): Signed forward speed in mm/s.
            turn_rate (float): Clockwise turn rate in deg/s.
        '''
        number(speed, 'speed')
        number(turn_rate, 'turn_rate')
        self._set_wheels(speed, turn_rate)
        self._maneuver = None
        if self._use_gyro:
            self._model.imu.in_use = True

    def stop(self) -> None:
        '''
        Cancel the current move and release both wheel motors.
        '''
        self._maneuver = None
        if self._model._imu is not None:
            self._model.imu.in_use = False
        self._left.stop()
        self._right.stop()

    def brake(self) -> None:
        '''
        Cancel the current move and passively brake both wheel motors.
        '''
        self._maneuver = None
        if self._model._imu is not None:
            self._model.imu.in_use = False
        self._left.brake()
        self._right.brake()

    @requires_init
    def hold(self) -> None:
        '''
        Cancel the current move and hold both wheel positions.

        Return immediately. Later simulation steps apply the holding control.
        '''
        self._maneuver = None
        self._apply_then(Stop.HOLD)

    @requires_init
    def straight(self, distance: float, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        '''
        Move along a straight line while holding the starting heading.

        Arg:
            distance (float): Signed travel distance in mm.
            then (Stop): Stop mode after the move.
            wait (bool): Advance simulated time until the move finishes.
        '''
        self._start(distance, 0.0, then, wait)

    @requires_init
    def turn(self, angle: float, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        '''
        Turn in place while holding the starting distance.

        Arg:
            angle (float): Clockwise rotation in degrees.
            then (Stop): Stop mode after the move.
            wait (bool): Advance simulated time until the move finishes.
        '''
        self._start(0.0, angle, then, wait)

    @requires_init
    def done(self) -> bool:
        '''
        Return whether no straight or turn maneuver is active.

        This does not mean the wheels have stopped during drive().
        '''
        return self._maneuver is None

    def stalled(self) -> bool:
        '''
        Return False because drive base load is not simulated.
        '''
        return False  # no load/torque model yet

    @requires_init
    def distance(self) -> float:
        '''
        Return encoder distance after the reset offset.

        Returns:
            float: Signed travel distance in mm.
        '''
        return self._distance

    @requires_init
    def angle(self) -> float:
        '''
        Return clockwise heading from wheel or gyro feedback.

        Returns:
            float: Heading in degrees after the selected reset offset.
        '''
        return self._angle

    @requires_init
    def state(self) -> tuple[float, float, float, float]:
        '''
        Return distance, speed, heading and turn rate.

        Speed uses wheel encoders. Turn rate uses the selected heading source.

        Returns:
            tuple: Distance in mm, speed in mm/s, clockwise angle in degrees,
                and clockwise turn rate in deg/s.
        '''
        turn_rate = (-self._model.imu.calibrated_rate if self._use_gyro
                     else self._model.encoder_turn_rate_dps())
        return (self._distance, self._model.encoder_speed_mm_s(),
                self._angle, turn_rate)

    def reset(self, distance: float = 0, angle: float = 0) -> None:
        '''
        Reset reported distance and heading without moving the body.

        Brake both motors and release gyro holding before changing offsets.

        Arg:
            distance (float): New reported distance in mm.
            angle (float): New clockwise heading in degrees.
        '''
        number(distance, 'distance')
        number(angle, 'angle')
        self.brake()
        self._distance_offset = self._model.distance_mm - float(distance)
        self._angle_offset = self._model.angle_deg - float(angle)
        if self._use_gyro:
            self._model.imu.reset_heading(float(angle))

    def settings(self, straight_speed: float | None = None, straight_acceleration: float | None = None,
                 turn_rate: float | None = None, turn_acceleration: float | None = None) -> tuple[float, ...] | None:
        '''
        Get or set positive motion limits.

        Leave a value unchanged when its argument is None.

        Arg:
            straight_speed (float | None): Maximum straight speed in mm/s.
            straight_acceleration (float | None): Straight acceleration in mm/s/s.
            turn_rate (float | None): Maximum clockwise turn rate in deg/s.
            turn_acceleration (float | None): Turn acceleration in deg/s/s.

        Returns:
            tuple | None: Current limits without arguments, or None after an update.
        '''
        values = (straight_speed, straight_acceleration, turn_rate, turn_acceleration)
        names = ('straight_speed', 'straight_acceleration', 'turn_rate', 'turn_acceleration')
        for name, value in zip(names, values):
            if value is not None:
                number(value, name, positive=True)
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
        '''
        Place the simulated body without resetting encoder totals.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.
            heading_deg (float): Counter-clockwise heading from the world x axis.
        '''
        self._model.place(x_mm, y_mm, heading_deg)

    def pose(self) -> tuple[float, float, float]:
        '''
        Return the true body pose in the world.

        Returns:
            tuple: World x and y in mm, then counter-clockwise heading in degrees.
        '''
        return self._model.pose()

    def _start(self, distance: float, angle: float, then: Stop, wait: bool) -> None:
        '''
        Start a maneuver with separate distance and angle profiles.

        Arg:
            distance (float): Signed distance in mm.
            angle (float): Clockwise rotation in degrees.
            then (Stop): Stop mode after the maneuver.
            wait (bool): Advance simulated time until the maneuver finishes.
        '''
        if not isinstance(then, Stop):
            raise TypeError('then must be a Stop')
        boolean(wait, 'wait')
        # Hold the other axis at its starting value (straight keeps heading,
        # turn keeps position) by giving it a zero-length profile.
        dist_profile = _Profile(distance, self._straight_speed, self._straight_accel)
        angle_profile = _Profile(angle, self._turn_rate, self._turn_accel)
        m = {
            'd0': self._distance,
            'a0': self._angle,
            'dist_profile': dist_profile,
            'angle_profile': angle_profile,
            'then': then,
            't': 0.0,
            'duration': max(dist_profile.duration, angle_profile.duration),
        }
        timeout_ms = (m['duration'] + SETTLE_TIMEOUT_S + 1.0) * 1000.0
        number(timeout_ms, 'Maneuver timeout', positive=True)
        self._maneuver = m
        if self._use_gyro:
            self._model.imu.in_use = True
        if wait:
            clock.advance_until(self.done, timeout_ms=timeout_ms)
