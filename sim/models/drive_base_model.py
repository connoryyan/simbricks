import math

from sim.clock import clock
from .model import Model
from .motor_model import MotorModel
from .sensor_model import SensorModel


class DriveBaseModel(Model):
    step_priority = 1

    def __init__(self, time_constant_s: float = 0.15,
                 left_model = None, right_model = None,
                 wheel_diameter: float = None, axle_track: float = None,
                 left_sign: int = 1, right_sign: int = 1):
        
        if not isinstance(left_model, MotorModel):
            raise TypeError(f"Expected a MotorModel object but got "
                            f"{left_model.__class__.__name__}")
        if not isinstance(right_model, MotorModel):
            raise TypeError(f"Expected a MotorModel object but got "
                            f"{right_model.__class__.__name__}")

        if not isinstance(wheel_diameter, (int, float)):
            raise TypeError(f"Expected int or float but got "
                            f"{wheel_diameter.__class__.__name__}")
        if not isinstance(axle_track, (int, float)):
            raise TypeError(f"Expected int or float but got "
                            f"{axle_track.__class__.__name__}")
        
        if wheel_diameter <= 0 or axle_track <= 0:
            raise ValueError("wheel_diameter and axle_track must be positive")
        if left_model is right_model:
            raise ValueError("left_model and right_model must be different models")
        if any(type(sign) is not int or sign not in (-1, 1)
               for sign in (left_sign, right_sign)):
            raise ValueError("Wheel direction signs must be -1 or 1")

        super().__init__(time_constant_s)

        self._left: MotorModel = left_model
        self._right: MotorModel = right_model
        # Map gear output rotation into each wheel's forward direction.
        self._left_sign = left_sign
        self._right_sign = right_sign
        # wheel rim travel per wheel degree
        self.mm_per_deg: float = math.pi * wheel_diameter / 360.0
        self.wheel_diameter: float = wheel_diameter
        self.axle_track: float = axle_track

        # True pose in the world frame.
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.heading_rad = 0.0 # counter-clockwise

        # Running totals since creation. Never reset: DriveBase keeps its own
        # offsets for its resettable distance()/angle().
        self.distance_mm = 0.0
        self.angle_deg = 0.0       # clockwise positive

        self._last_l: float = left_model.output_angle * left_sign
        self._last_r: float = right_model.output_angle * right_sign
        self._mounts: dict[SensorModel, dict[str, float]] = {}
        self._imu = None
        self.friction_enabled = False
        self._mass_kg = 1.2
        self._yaw_inertia_kg_m2 = self._mass_kg * (axle_track / 1000)**2 / 6
        self._contact_stiffness = 30.0
        self._vx_m_s = self._vy_m_s = self._omega_rad_s = 0.0
        self._ax_m_s2 = self._ay_m_s2 = 0.0

    def configure_friction(self, enabled=True, *, mass_kg=1.2,
                           yaw_inertia_kg_m2=None, contact_stiffness=30.0):
        '''
        Configure finite wheel friction or ideal rolling.

        Arg:
            enabled (bool): Enable finite friction dynamics.
            mass_kg (float): Body mass in kg.
            yaw_inertia_kg_m2 (float | None): Yaw inertia, or a square-body estimate.
            contact_stiffness (float): Contact force per slip speed in N/(m/s).
        '''
        inertia = (mass_kg * (self.axle_track/1000)**2 / 6
                   if yaw_inertia_kg_m2 is None else yaw_inertia_kg_m2)
        if not all(math.isfinite(v) and v > 0 for v in (mass_kg, inertia, contact_stiffness)):
            raise ValueError('Mass, inertia and contact stiffness must be finite and positive')
        # Preserve world velocity when changing modes. No encoder offsets change.
        if enabled and not self.friction_enabled:
            speed = self.encoder_speed_mm_s()/1000
            self._vx_m_s = speed*math.cos(self.heading_rad)
            self._vy_m_s = speed*math.sin(self.heading_rad)
            self._omega_rad_s = -math.radians(self.encoder_turn_rate_dps())
        self._mass_kg = mass_kg
        self._yaw_inertia_kg_m2 = inertia
        self._contact_stiffness = contact_stiffness
        self.friction_enabled = bool(enabled)

    @property
    def imu(self):
        '''
        Return the body IMU and create it on first use.
        '''
        if self._imu is None:
            from .imu_model import IMUModel
            self._imu = IMUModel(self)
        return self._imu

    def step(self, dt_s: float) -> None:
        if not math.isfinite(dt_s) or dt_s <= 0:
            raise ValueError('Body step must be finite and positive')
        old_vx, old_vy = self._vx_m_s, self._vy_m_s
        ds, dtheta_cw = self._update_odometry()
        if self.friction_enabled:
            self._step_friction(dt_s)
        else:
            mid_heading = self.heading_rad - dtheta_cw / 2.0
            self.x_mm += ds * math.cos(mid_heading)
            self.y_mm += ds * math.sin(mid_heading)
            self.heading_rad -= dtheta_cw
            speed = self.encoder_speed_mm_s()/1000
            self._vx_m_s = speed*math.cos(self.heading_rad)
            self._vy_m_s = speed*math.sin(self.heading_rad)
            self._omega_rad_s = -math.radians(self.encoder_turn_rate_dps())
        self._ax_m_s2 = (self._vx_m_s-old_vx)/dt_s
        self._ay_m_s2 = (self._vy_m_s-old_vy)/dt_s
        self._update_mounts()

    def _update_odometry(self) -> tuple[float, float]:
        l = self._left.output_angle * self._left_sign
        r = self._right.output_angle * self._right_sign
        dl = (l - self._last_l) * self.mm_per_deg
        dr = (r - self._last_r) * self.mm_per_deg
        self._last_l, self._last_r = l, r

        ds = (dl + dr) / 2.0
        dtheta_cw = (dl - dr) / self.axle_track      # radians, clockwise positive

        self.distance_mm += ds
        self.angle_deg += math.degrees(dtheta_cw)
        return ds, dtheta_cw

    def _step_friction(self, dt_s):
        '''
        Integrate contact forces with a shared friction circle at each wheel.
        '''
        import world
        field = world.get_current_world()
        half_track = self.axle_track/2000
        wheel_speeds = (self._left.output_speed*self._left_sign*self.mm_per_deg/1000,
                        self._right.output_speed*self._right_sign*self.mm_per_deg/1000)
        # Small steps keep the compliant contact response stable, including light
        # bodies and small yaw inertias. At most 1 ms per contact integration.
        stable_dt = min(0.001, self._mass_kg/(8*self._contact_stiffness),
                        self._yaw_inertia_kg_m2/(8*self._contact_stiffness*half_track**2))
        count = math.ceil(dt_s/stable_dt)
        h = dt_s/count
        for _ in range(count):
            c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
            forward = c*self._vx_m_s + s*self._vy_m_s
            lateral = -s*self._vx_m_s + c*self._vy_m_s
            fx = fy = torque = 0.0
            for wheel_speed, offset in zip(wheel_speeds, (half_track, -half_track)):
                mu = field.get_friction_at(self.x_mm-s*offset*1000,
                                           self.y_mm+c*offset*1000)
                if not math.isfinite(mu) or mu < 0:
                    raise ValueError('Ground friction must be finite and non-negative')
                slip = wheel_speed - (forward-self._omega_rad_s*offset)
                wheel_fx = self._contact_stiffness*slip
                wheel_fy = -self._contact_stiffness*lateral
                magnitude = math.hypot(wheel_fx, wheel_fy)
                limit = mu*self._mass_kg*9.80665/2
                if magnitude > limit:
                    scale = limit/magnitude
                    wheel_fx *= scale
                    wheel_fy *= scale
                fx += wheel_fx
                fy += wheel_fy
                torque -= offset*wheel_fx
            old_vx, old_vy, old_omega = self._vx_m_s, self._vy_m_s, self._omega_rad_s
            self._vx_m_s += (c*fx-s*fy)/self._mass_kg*h
            self._vy_m_s += (s*fx+c*fy)/self._mass_kg*h
            self._omega_rad_s += torque/self._yaw_inertia_kg_m2*h
            self.x_mm += (old_vx+self._vx_m_s)*0.5*h*1000
            self.y_mm += (old_vy+self._vy_m_s)*0.5*h*1000
            self.heading_rad += (old_omega+self._omega_rad_s)*0.5*h

    def pose(self) -> tuple[float, float, float]:
        '''
        Return the world pose of the robot.

        Returns:
            tuple[float, float, float]: x and y in mm, then counter-clockwise heading in degrees.
        '''
        return self.x_mm, self.y_mm, math.degrees(self.heading_rad)

    def speed_mm_s(self) -> float:
        '''
        Return the forward speed of the body.

        Returns:
            float: Speed of the robot in mm/s.
        '''
        if self.friction_enabled:
            return (math.cos(self.heading_rad)*self._vx_m_s
                    + math.sin(self.heading_rad)*self._vy_m_s)*1000
        return self.encoder_speed_mm_s()

    def encoder_speed_mm_s(self):
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

    def encoder_turn_rate_dps(self):
        '''
        Return clockwise turn rate estimated from wheel encoders in deg/s.
        '''
        vl = self._left.output_speed * self._left_sign * self.mm_per_deg
        vr = self._right.output_speed * self._right_sign * self.mm_per_deg
        return math.degrees((vl - vr) / self.axle_track)

    def velocity_mm_s(self):
        '''
        Return the actual world x and y velocities in mm/s.
        '''
        return self._vx_m_s*1000, self._vy_m_s*1000

    def acceleration_mm_s2(self):
        '''
        Return actual forward and left acceleration in mm/s/s.
        '''
        c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
        return ((c*self._ax_m_s2+s*self._ay_m_s2)*1000,
                (-s*self._ax_m_s2+c*self._ay_m_s2)*1000)

    def place(self, x_mm: float, y_mm: float, 
              heading_deg: float = 0.0) -> None:
        '''
        Set the world position and counter-clockwise heading of the robot.
        '''
        if not all(math.isfinite(v) for v in (x_mm, y_mm, heading_deg)):
            raise ValueError("Robot pose must be finite")
        self.x_mm, self.y_mm = x_mm, y_mm
        self.heading_rad = math.radians(heading_deg)
        self._vx_m_s = self._vy_m_s = self._omega_rad_s = 0.0
        self._ax_m_s2 = self._ay_m_s2 = 0.0
        if self._imu is not None:
            self._imu.rebase_pose()
        self._update_mounts()

    def mount(self, sensor_model: SensorModel,
              forward_mm: float = 0.0,
              left_mm: float = 0.0,
              above_mm: float = 0.0,
              yaw_deg: float = 0.0,
              pitch_deg: float = 0.0,
              roll_deg: float = 0.0) -> None:
        '''
        Attach a sensor model to the body.

        Measure offsets from the axle centre. Keep pitch and roll fixed during motion.
        World height is half the wheel diameter plus above_mm.

        Arg:
            sensor_model (SensorModel): Sensor model to attach.
            forward_mm (float): Forward offset in mm.
            left_mm (float): Left offset in mm.
            above_mm (float): Upward offset from the axle centre in mm.
            yaw_deg (float): Counter-clockwise yaw relative to the body in degrees.
            pitch_deg (float): Downward pitch in degrees.
            roll_deg (float): Roll in degrees by the right hand rule.
        '''
        if not isinstance(sensor_model, SensorModel):
            raise TypeError("sensor_model must be a SensorModel")
        if sensor_model in self._mounts:
            raise ValueError("Sensor model is already mounted on this body")
        values = tuple(float(v) for v in
                       (forward_mm, left_mm, above_mm, yaw_deg, pitch_deg, roll_deg))
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Mount values must be finite")
        forward, left, above, yaw, pitch, roll = values
        z = self.wheel_diameter / 2.0 + above
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
        c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
        heading_deg = math.degrees(self.heading_rad)
        for sensor_model, mount in self._mounts.items():
            x = self.x_mm + mount['forward_mm'] * c - mount['left_mm'] * s
            y = self.y_mm + mount['forward_mm'] * s + mount['left_mm'] * c
            sensor_model.set_pose(x, y, mount['z_mm'], heading_deg + mount['yaw_deg'])

    def detach(self) -> None:
        '''
        Remove the model from the clock.
        '''
        clock.remove_steppable(self)
        if self._imu is not None:
            clock.remove_steppable(self._imu)
