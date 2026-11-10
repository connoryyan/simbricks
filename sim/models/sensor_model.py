import math
import random

import world as world_module
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
        return world_module.get_current_world()

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

    def view_direction(self) -> tuple[float, float, float]:
        '''
        Return the unit measurement direction in world coordinates.

        Roll does not change the centre ray.

        Returns:
            tuple[float, float, float]: Direction along x, y and z.
        '''
        yaw = math.radians(self._yaw_deg)
        pitch = math.radians(self._pitch_deg)
        return (math.cos(pitch) * math.cos(yaw),
                math.cos(pitch) * math.sin(yaw), -math.sin(pitch))


class ColorSensorModel(SensorModel):
    '''
    Simulate a LEGO SPIKE color sensor.
    '''

    def __init__(self, time_constant_s: float = 0.15, noise: float = 0.0,
                 seed: int | None = None, *, reference_distance_mm: float = 10.0,
                 max_distance_mm: float = 100.0):
        '''
        Create a colour sensor with a simple distance response.

        Arg:
            time_constant_s (float): Response time in seconds.
            noise (float): Reflection noise standard deviation.
            seed (int | None): Random seed for repeatable noise.
            reference_distance_mm (float): Distance with full signal strength in mm.
            max_distance_mm (float): Maximum surface distance in mm.
        '''
        if not all(math.isfinite(v) and v > 0
                   for v in (reference_distance_mm, max_distance_mm)):
            raise ValueError("Sensor distances must be finite and positive")
        if reference_distance_mm > max_distance_mm:
            raise ValueError("Reference distance cannot exceed maximum distance")
        super().__init__(time_constant_s, noise, seed)
        self.reference_distance_mm = reference_distance_mm
        self.max_distance_mm = max_distance_mm

    def reflection(self) -> int:
        '''
        Measure reflected light at the first surface along the centre ray.

        Add ambient light to the active reflected signal.
        Use a capped inverse-square distance response and cosine angle response.
        This is a simple model for calibration, not a hardware response curve.

        Returns:
            int: Total signal from 0 to 100. Return ambient light without a surface hit.
        '''
        value = self.world.get_ambient_at(self._x_mm, self._y_mm, self._z_mm)
        hit = self.world.raycast_surface(
            self._x_mm, self._y_mm, self._z_mm,
            *self.view_direction(), self.max_distance_mm,
        )
        if hit is not None:
            strength = (1.0 if hit.distance_mm <= self.reference_distance_mm
                        else (self.reference_distance_mm / hit.distance_mm) ** 2)
            value += hit.reflection * strength * hit.incidence_cosine
        return int(round(_clamp(self._noisy(value), 0, 100)))

    def ambient(self) -> int:
        '''
        Measure ambient light without the active reflected signal.

        Returns:
            int: Ambient light from 0 to 100 in the simulated signal scale.
        '''
        value = self.world.get_ambient_at(self._x_mm, self._y_mm, self._z_mm)
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
                                        MAX_DISTANCE_MM, z_mm=self._z_mm,
                                        pitch_deg=self._pitch_deg)
        return int(round(_clamp(self._noisy(d), 0, MAX_DISTANCE_MM)))


class ForceSensorModel(SensorModel):
    '''
    Store scripted force and button displacement for a force sensor.
    '''
    def __init__(self):
        super().__init__()
        self.force_n = 0.0
        self.distance_mm = 0.0

    def set_measurement(self, force_n=0.0, distance_mm=0.0):
        '''
        Set force in N and button displacement in mm for simulator input.
        '''
        if not math.isfinite(force_n) or not 0 <= force_n <= 10:
            raise ValueError('Force must be between 0 and 10 N')
        if not math.isfinite(distance_mm) or not 0 <= distance_mm <= 8:
            raise ValueError('Button displacement must be between 0 and 8 mm')
        self.force_n, self.distance_mm = force_n, distance_mm
