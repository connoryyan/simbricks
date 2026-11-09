import math

from sim.clock import clock
from .model import Model
from .motor_model import MotorModel


class DriveBaseModel(Model):
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
        # Map raw motor rotation into each wheel's forward direction.
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

        self._last_l: float = left_model.angle * left_sign
        self._last_r: float = right_model.angle * right_sign
        self._mounts: list = []

    def step(self, dt_s: float) -> None:
        self._update_odometry()
        self._update_mounts()

    def _update_odometry(self) -> None:
        l = self._left.angle * self._left_sign
        r = self._right.angle * self._right_sign
        dl = (l - self._last_l) * self.mm_per_deg
        dr = (r - self._last_r) * self.mm_per_deg
        self._last_l, self._last_r = l, r

        ds = (dl + dr) / 2.0
        dtheta_cw = (dl - dr) / self.axle_track      # radians, clockwise positive

        mid_heading = self.heading_rad - dtheta_cw / 2.0
        self.x_mm += ds * math.cos(mid_heading)
        self.y_mm += ds * math.sin(mid_heading)
        self.heading_rad -= dtheta_cw

        self.distance_mm += ds
        self.angle_deg += math.degrees(dtheta_cw)

    def pose(self) -> tuple[float, float, float]:
        '''
        Return the true pose of the robot (x_mm, y_mm, heading_deg counter-clockwise).

        Returns:
            tuple[float, float, float]: True pose of the robot (x/mm, y/mm, heading/deg counter-clockwise).
        '''
        return self.x_mm, self.y_mm, math.degrees(self.heading_rad)

    def speed_mm_s(self) -> float:
        '''
        Current forward speed of the body, from the motors' speeds.

        Returns:
            float: Speed of the robot in mm/s.
        '''
        vl = self._left.speed * self._left_sign * self.mm_per_deg
        vr = self._right.speed * self._right_sign * self.mm_per_deg
        return (vl + vr) / 2.0

    def turn_rate_dps(self) -> float:
        '''
        Current turn rate in deg/s, clockwise positive.

        Returns:
            float: Turn rate in deg/s, clockwise positive.
        '''
        vl = self._left.speed * self._left_sign * self.mm_per_deg
        vr = self._right.speed * self._right_sign * self.mm_per_deg
        return math.degrees((vl - vr) / self.axle_track)

    def place(self, x_mm: float, y_mm: float, 
              heading_deg: float = 0.0) -> None:
        '''
        Put the robot somewhere in the world (heading counter-clockwise from +x).
        '''
        self.x_mm, self.y_mm = x_mm, y_mm
        self.heading_rad = math.radians(heading_deg)
        self._update_mounts()

    def mount(self, sensor, 
              forward_mm: float = 0.0, 
              left_mm: float = 0.0,
              above_mm: float = 0.0) -> None:
        '''
        Attach a sensor at an offset from the axle center.
        '''
        self._mounts.append((sensor, forward_mm, left_mm, above_mm))
        self._update_mounts()

    def _update_mounts(self) -> None:
        c, s = math.cos(self.heading_rad), math.sin(self.heading_rad)
        heading_deg = math.degrees(self.heading_rad)
        for sensor, forward, left, above in self._mounts:
            x = self.x_mm + forward * c - left * s
            y = self.y_mm + forward * s + left * c
            z = self.wheel_diameter / 2 + above
            sensor.set_pose(x, y, z, heading_deg)

    def detach(self) -> None:
        '''
        Stop updating this model (it is removed from the clock).
        '''
        clock.remove_steppable(self)
