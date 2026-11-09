import math
import random

from .. import world as world_module
from .model import Model

# What the ultrasonic sensor returns when nothing is measured.
MAX_DISTANCE_MM = 2000 # 2m


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SensorModel(Model):
    def __init__(self, time_constant_s: float = 0.15, noise: float = 0.0, seed: int | None = None):
        '''
        Create a sensor model.

        Arg:
            time_constant_s (float): Response time in seconds.
            noise (float): Noise standard deviation in sensor units. Zero disables noise.
            seed (int | None): Random seed for repeatable noise.
        '''
        super().__init__(time_constant_s)
        self._noise_std: float = noise
        self._rng: random.Random = random.Random(seed)

        self._x_mm = 0.0
        self._y_mm = 0.0
        self._z_mm = 0.0
        self._yaw_deg = 0.0
        self._pitch_deg = 0.0
        self._roll_deg = 0.0

    def step(self, dt_s: float) -> None:
        pass

    @property
    def world(self) -> world_module.World:
        return world_module.current_world

    def set_pose(self, x_mm: float, y_mm: float, z_mm: float, yaw_deg: float) -> None:
        '''
        Set the world position and yaw of the sensor.

        Keep the fixed pitch and roll unchanged. Use this method in the simulator.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.
            z_mm (float): Height above the ground in mm.
            yaw_deg (float): Counter-clockwise angle from the world x axis in degrees.
        '''
        values = tuple(float(v) for v in (x_mm, y_mm, z_mm, yaw_deg))
        if not all(math.isfinite(v) for v in values) or values[2] < 0:
            raise ValueError("Sensor pose must be finite and above or on the ground")
        self._x_mm, self._y_mm, self._z_mm, self._yaw_deg = values

    def set_tilt(self, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> None:
        '''
        Set the fixed tilt of the sensor.

        Use Rz(yaw) * Ry(pitch) * Rx(roll) for rotation.

        Arg:
            pitch_deg (float): Downward pitch in degrees.
            roll_deg (float): Roll in degrees by the right hand rule.
        '''
        values = tuple(float(v) for v in (pitch_deg, roll_deg))
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Sensor tilt must be finite")
        self._pitch_deg, self._roll_deg = values

    def pose(self) -> tuple[float, float, float, float, float, float]:
        '''
        Return the world pose of the sensor.

        Returns:
            tuple[float, float, float, float, float, float]: x, y, z in mm, then yaw, pitch and roll in degrees.
        '''
        return (self._x_mm, self._y_mm, self._z_mm,
                self._yaw_deg, self._pitch_deg, self._roll_deg)

    def _noisy(self, value: float) -> float:
        if self._noise_std > 0:
            value += self._rng.gauss(0.0, self._noise_std)
        return value


class ColorSensorModel(SensorModel):
    '''
    Simulate a LEGO SPIKE color sensor.
    '''

    def reflection(self) -> int:
        value = self.world.get_reflection_at(self._x_mm, self._y_mm)
        return int(round(_clamp(self._noisy(value), 0, 100)))


class UltrasonicSensorModel(SensorModel):
    '''
    Simulate a LEGO SPIKE ultrasonic sensor.
    '''

    def distance(self) -> int:
        '''
        Return the measured distance.

        Returns:
            int: Distance in mm. Return 2000 when no object is detected.
        '''
        d = self.world.raycast_distance(self._x_mm, self._y_mm, self._yaw_deg,
                                        MAX_DISTANCE_MM)
        return int(round(_clamp(self._noisy(d), 0, MAX_DISTANCE_MM)))
