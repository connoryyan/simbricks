import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .imu_model import IMUModel

from sim.clock import clock
from .model import Model
from .motor_model import MotorModel
from .sensor_model import SensorModel
from ..validation import boolean, integer, number


class DriveBaseModel(Model):
    '''
    Simulate body motion separately from wheel encoder odometry.

    Attributes:
        step_priority (int): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in
            seconds.
        time_constant_s (float): Read-only device response time constant in
            seconds.
        _left (MotorModel): Raw motor model driving the left wheel.
        _right (MotorModel): Raw motor model driving the right wheel.
        _left_sign (int): Raw left motor direction for forward wheel travel, -1
            or 1.
        _right_sign (int): Raw right motor direction for forward wheel travel,
            -1 or 1.
        mm_per_deg (float): Wheel travel in mm per degree of gear output
            rotation.
        wheel_diameter (float): Wheel diameter in mm.
        axle_track (float): Distance between wheel contact points in mm.
        x_mm (float): True world x coordinate of the axle center in mm.
        y_mm (float): True world y coordinate of the axle center in mm.
        heading_rad (float): True counter-clockwise heading from the world x
            axis, in
            radians.
        distance_mm (float): Accumulated signed encoder distance in mm; it may
            differ
            from true travel.
        angle_deg (float): Accumulated clockwise encoder heading in degrees.
        _last_l (float): Last left output angle after direction mapping, in
            degrees.
        _last_r (float): Last right output angle after direction mapping, in
            degrees.
        _mounts (dict): Sensor models mapped to forward/left offsets, ground
            height and
            relative yaw.
        _imu (IMUModel | None): Attached IMU model, or None before first use.
        friction_enabled (bool): Whether finite contact friction replaces ideal
            rolling.
        _mass_kg (float): Body mass used by friction dynamics, in kg.
        _yaw_inertia_kg_m2 (float): Body yaw moment of inertia in kg m squared.
        _contact_stiffness (float): Wheel contact force per slip speed, in
            N/(m/s).
        _vx_m_s (float): True world x velocity in m/s.
        _vy_m_s (float): True world y velocity in m/s.
        _omega_rad_s (float): True counter-clockwise yaw rate in rad/s.
        _ax_m_s2 (float): True world x acceleration in m/s/s.
        _ay_m_s2 (float): True world y acceleration in m/s/s.
        imu (IMUModel): Attached IMU model, created on first access when
            needed.
    '''
    step_priority: int = 1

    def __init__(
        self,
        time_constant_s: float = 0.15,
        left_model: MotorModel | None = None,
        right_model: MotorModel | None = None,
        wheel_diameter: float = None,
        axle_track: float = None,
        left_sign: int = 1,
        right_sign: int = 1,
    ) -> None:
        '''
        Create a body model using two motor models.

        Arg:
            time_constant_s (float): Positive response time in seconds.
            left_model (MotorModel): Model driving the left wheel.
            right_model (MotorModel): Model driving the right wheel.
            wheel_diameter (float): Positive wheel diameter in mm.
            axle_track (float): Positive wheel contact spacing in mm.
            left_sign (int): Raw left motor direction for forward motion, -1 or
                1.
            right_sign (int): Raw right motor direction for forward motion, -1
                or 1.
        '''
        if not isinstance(left_model, MotorModel):
            raise TypeError(f"Expected a MotorModel object but got "
                            f"{left_model.__class__.__name__}")
        if not isinstance(right_model, MotorModel):
            raise TypeError(f"Expected a MotorModel object but got "
                            f"{right_model.__class__.__name__}")

        number(wheel_diameter, 'wheel_diameter', positive=True)
        number(axle_track, 'axle_track', positive=True)
        if left_model is right_model:
            raise ValueError(
                "left_model and right_model must be different models")
        integer(left_sign, 'left_sign')
        integer(right_sign, 'right_sign')
        if left_sign not in (-1, 1) or right_sign not in (-1, 1):
            raise ValueError("Wheel direction signs must be -1 or 1")

        mm_per_deg = math.pi * (wheel_diameter / 360.0)
        track_m = axle_track / 1000
        inertia = (1.2 / 6) * track_m * track_m
        number(mm_per_deg, 'mm_per_deg', positive=True)
        number(inertia, 'yaw_inertia_kg_m2', positive=True)
        super().__init__(time_constant_s)

        self._left: MotorModel = left_model
        self._right: MotorModel = right_model
        # Map gear output rotation into each wheel's forward direction.
        self._left_sign: int = left_sign
        self._right_sign: int = right_sign
        # wheel rim travel per wheel degree
        self.mm_per_deg: float = mm_per_deg
        self.wheel_diameter: float = wheel_diameter
        self.axle_track: float = axle_track

        # True pose in the world frame.
        self.x_mm: float = 0.0
        self.y_mm: float = 0.0
        self.heading_rad: float = 0.0  # counter-clockwise

        # Running totals since creation. Never reset: DriveBase keeps its own
        # offsets for its resettable distance()/angle().
        self.distance_mm: float = 0.0
        self.angle_deg: float = 0.0       # clockwise positive

        self._last_l: float = left_model.output_angle * left_sign
        self._last_r: float = right_model.output_angle * right_sign
        self._mounts: dict[SensorModel, dict[str, float]] = {}
        self._imu: IMUModel | None = None
        self.friction_enabled: bool = False
        self._mass_kg: float = 1.2
        self._yaw_inertia_kg_m2: float = inertia
        self._contact_stiffness: float = 30.0
        self._vx_m_s: float = 0.0
        self._vy_m_s: float = 0.0
        self._omega_rad_s: float = 0.0
        self._ax_m_s2: float = 0.0
        self._ay_m_s2: float = 0.0

    def configure_friction(
        self,
        enabled: bool = True,
        *,
        mass_kg: float = 1.2,
        yaw_inertia_kg_m2: float | None = None,
        contact_stiffness: float = 30.0,
    ) -> None:
        '''
        Configure finite wheel friction or ideal rolling.

        Arg:
            enabled (bool): Enable finite friction dynamics.
            mass_kg (float): Body mass in kg.
            yaw_inertia_kg_m2 (float | None): Yaw inertia, or a square-body
                estimate.
            contact_stiffness (float): Contact force per slip speed in N/(m/s).
        '''
        boolean(enabled, 'enabled')
        number(mass_kg, 'mass_kg', positive=True)
        number(contact_stiffness, 'contact_stiffness', positive=True)
        inertia = (mass_kg * (self.axle_track / 1000)**2 / 6
                   if yaw_inertia_kg_m2 is None else yaw_inertia_kg_m2)
        number(inertia, 'yaw_inertia_kg_m2', positive=True)
        # Preserve world velocity when changing modes. No encoder offsets
        # change.
        if enabled and not self.friction_enabled:
            speed = self.encoder_speed_mm_s() / 1000
            self._vx_m_s = speed * math.cos(self.heading_rad)
            self._vy_m_s = speed * math.sin(self.heading_rad)
            self._omega_rad_s = -math.radians(self.encoder_turn_rate_dps())
        self._mass_kg = mass_kg
        self._yaw_inertia_kg_m2 = inertia
        self._contact_stiffness = contact_stiffness
        self.friction_enabled = bool(enabled)

    @property
    def imu(self) -> 'IMUModel':
        '''
        Return the body IMU and create it on first use.
        '''
        if self._imu is None:
            from .imu_model import IMUModel
            self._imu = IMUModel(self)
        return self._imu

    def step(self, dt_s: float) -> None:
        '''
        Update encoder odometry, true body motion and mounted sensors.

        Arg:
            dt_s (float): Positive simulation step in seconds.
        '''
        number(dt_s, 'dt_s', positive=True)
        old_vx, old_vy = self._vx_m_s, self._vy_m_s
        ds, dtheta_cw = self._update_odometry()
        if self.friction_enabled:
            self._step_friction(dt_s)
        else:
            mid_heading = self.heading_rad - dtheta_cw / 2.0
            self.x_mm += ds * math.cos(mid_heading)
            self.y_mm += ds * math.sin(mid_heading)
            self.heading_rad -= dtheta_cw
            speed = self.encoder_speed_mm_s() / 1000
            self._vx_m_s = speed * math.cos(self.heading_rad)
            self._vy_m_s = speed * math.sin(self.heading_rad)
            self._omega_rad_s = -math.radians(self.encoder_turn_rate_dps())
        self._ax_m_s2 = (self._vx_m_s - old_vx) / dt_s
        self._ay_m_s2 = (self._vy_m_s - old_vy) / dt_s
        self._update_mounts()

    def _update_odometry(self) -> tuple[float, float]:
        '''
        Accumulate encoder travel from gear output angle changes.

        Returns:
            tuple[float, float]: Forward distance in mm and clockwise rotation
                in radians.
        '''
        left_angle = self._left.output_angle * self._left_sign
        right_angle = self._right.output_angle * self._right_sign
        dl = (left_angle - self._last_l) * self.mm_per_deg
        dr = (right_angle - self._last_r) * self.mm_per_deg
        self._last_l, self._last_r = left_angle, right_angle

        ds = (dl + dr) / 2.0
        # radians, clockwise positive
        dtheta_cw = (dl - dr) / self.axle_track

        self.distance_mm += ds
        self.angle_deg += math.degrees(dtheta_cw)
        return ds, dtheta_cw

    def _step_friction(self, dt_s: float) -> None:
        '''
        Integrate contact forces with a shared friction circle at each wheel.
        '''
        import world
        field = world.get_current_world()
        half_track = self.axle_track / 2000
        wheel_speeds = (
            self._left.output_speed
            * self._left_sign
            * self.mm_per_deg
            / 1000,
            self._right.output_speed
            * self._right_sign
            * self.mm_per_deg
            / 1000)
        # Small steps keep the compliant contact response stable, including
        # light
        # bodies and small yaw inertias. At most 1 ms per contact integration.
        stable_dt = min(
            0.001, self._mass_kg / (8 * self._contact_stiffness),
            self._yaw_inertia_kg_m2
            / (8 * self._contact_stiffness * half_track ** 2))
        count = math.ceil(dt_s / stable_dt)
        h = dt_s / count
        for _ in range(count):
            c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
            forward = c * self._vx_m_s + s * self._vy_m_s
            lateral = -s * self._vx_m_s + c * self._vy_m_s
            fx = fy = torque = 0.0
            for wheel_speed, offset in zip(
                    wheel_speeds, (half_track, -half_track)):
                mu = field.get_friction_at(self.x_mm - s * offset * 1000,
                                           self.y_mm + c * offset * 1000)
                if not math.isfinite(mu) or mu < 0:
                    raise ValueError(
                        'Ground friction must be finite and non-negative')
                slip = wheel_speed - (forward - self._omega_rad_s * offset)
                wheel_fx = self._contact_stiffness * slip
                wheel_fy = -self._contact_stiffness * lateral
                magnitude = math.hypot(wheel_fx, wheel_fy)
                limit = mu * self._mass_kg * 9.80665 / 2
                if magnitude > limit:
                    scale = limit / magnitude
                    wheel_fx *= scale
                    wheel_fy *= scale
                fx += wheel_fx
                fy += wheel_fy
                torque -= offset * wheel_fx
            old_vx, old_vy, old_omega = (
                self._vx_m_s, self._vy_m_s, self._omega_rad_s,
            )
            self._vx_m_s += (c * fx - s * fy) / self._mass_kg * h
            self._vy_m_s += (s * fx + c * fy) / self._mass_kg * h
            self._omega_rad_s += torque / self._yaw_inertia_kg_m2 * h
            self.x_mm += (old_vx + self._vx_m_s) * 0.5 * h * 1000
            self.y_mm += (old_vy + self._vy_m_s) * 0.5 * h * 1000
            self.heading_rad += (old_omega + self._omega_rad_s) * 0.5 * h

    def pose(self) -> tuple[float, float, float]:
        '''
        Return the world pose of the robot.

        Returns:
            tuple[float, float, float]: x and y in mm, then counter-clockwise
                heading in degrees.
        '''
        return self.x_mm, self.y_mm, math.degrees(self.heading_rad)

    def speed_mm_s(self) -> float:
        '''
        Return the forward speed of the body.

        Returns:
            float: Speed of the robot in mm/s.
        '''
        if self.friction_enabled:
            return (math.cos(self.heading_rad) * self._vx_m_s
                    + math.sin(self.heading_rad) * self._vy_m_s) * 1000
        return self.encoder_speed_mm_s()

    def encoder_speed_mm_s(self) -> float:
        '''
        Return forward speed estimated from wheel encoders in mm/s.
        '''
        vl = self._left.output_speed * self._left_sign * self.mm_per_deg
        vr = self._right.output_speed * self._right_sign * self.mm_per_deg
        return (vl + vr) / 2.0

    def turn_rate_dps(self) -> float:
        '''
        Return the clockwise turn rate of the body.

        Returns:
            float: Turn rate in deg/s, clockwise positive.
        '''
        if self.friction_enabled:
            return -math.degrees(self._omega_rad_s)
        return self.encoder_turn_rate_dps()

    def encoder_turn_rate_dps(self) -> float:
        '''
        Return clockwise turn rate estimated from wheel encoders in deg/s.
        '''
        vl = self._left.output_speed * self._left_sign * self.mm_per_deg
        vr = self._right.output_speed * self._right_sign * self.mm_per_deg
        return math.degrees((vl - vr) / self.axle_track)

    def velocity_mm_s(self) -> tuple[float, float]:
        '''
        Return the actual world x and y velocities in mm/s.
        '''
        return self._vx_m_s * 1000, self._vy_m_s * 1000

    def acceleration_mm_s2(self) -> tuple[float, float]:
        '''
        Return actual forward and left acceleration in mm/s/s.
        '''
        c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
        return ((c * self._ax_m_s2 + s * self._ay_m_s2) * 1000,
                (-s * self._ax_m_s2 + c * self._ay_m_s2) * 1000)

    def place(
        self,
        x_mm: float,
        y_mm: float,
        heading_deg: float = 0.0,
    ) -> None:
        '''
        Set the world position and counter-clockwise heading of the robot.
        '''
        number(x_mm, 'x_mm')
        number(y_mm, 'y_mm')
        number(heading_deg, 'heading_deg')
        self.x_mm, self.y_mm = x_mm, y_mm
        self.heading_rad = math.radians(heading_deg)
        self._vx_m_s = self._vy_m_s = self._omega_rad_s = 0.0
        self._ax_m_s2 = self._ay_m_s2 = 0.0
        if self._imu is not None:
            self._imu.rebase_pose()
        self._update_mounts()

    def mount(
        self,
        sensor_model: SensorModel,
        forward_mm: float = 0.0,
        left_mm: float = 0.0,
        above_mm: float = 0.0,
        yaw_deg: float = 0.0,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0,
    ) -> None:
        '''
        Attach a sensor model to the body.

        Measure offsets from the axle centre. Keep pitch and roll fixed during
            motion.
        World height is half the wheel diameter plus above_mm.

        Arg:
            sensor_model (SensorModel): Sensor model to attach.
            forward_mm (float): Forward offset in mm.
            left_mm (float): Left offset in mm.
            above_mm (float): Upward offset from the axle centre in mm.
            yaw_deg (float): Counter-clockwise yaw relative to the body in
                degrees.
            pitch_deg (float): Downward pitch in degrees.
            roll_deg (float): Roll in degrees by the right hand rule.
        '''
        if not isinstance(sensor_model, SensorModel):
            raise TypeError("sensor_model must be a SensorModel")
        if sensor_model in self._mounts:
            raise ValueError("Sensor model is already mounted on this body")
        values = (forward_mm, left_mm, above_mm, yaw_deg, pitch_deg, roll_deg)
        names = ('forward_mm', 'left_mm', 'above_mm',
                 'yaw_deg', 'pitch_deg', 'roll_deg')
        for name, value in zip(names, values):
            number(value, name)
        forward, left, above, yaw, pitch, roll = values
        z = self.wheel_diameter / 2.0 + above
        number(z, 'sensor height')
        if z < 0:
            raise ValueError("Sensor cannot be below the ground")
        sensor_model.set_tilt(pitch, roll)
        self._mounts[sensor_model] = {
            'forward_mm': forward,
            'left_mm': left,
            'z_mm': z,
            'yaw_deg': yaw,
        }
        self._update_mounts()

    def _update_mounts(self) -> None:
        '''
        Update mounted sensor poses from the current body pose.
        '''
        c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
        heading_deg = math.degrees(self.heading_rad)
        for sensor_model, mount in self._mounts.items():
            x = self.x_mm + mount['forward_mm'] * c - mount['left_mm'] * s
            y = self.y_mm + mount['forward_mm'] * s + mount['left_mm'] * c
            sensor_model.set_pose(
                x, y, mount['z_mm'], heading_deg + mount['yaw_deg'])

    def detach(self) -> None:
        '''
        Remove the model from the clock.
        '''
        clock.remove_steppable(self)
        if self._imu is not None:
            clock.remove_steppable(self._imu)
