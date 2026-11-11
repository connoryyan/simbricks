import math
import random

import world as world_module
from .model import Model
from ..validation import integer, number


def _clamp(value: float, low: float, high: float) -> float:
    '''
    Limit a finite value to an ordered range.

    Arg:
        value (float): Input value.
        low (float): Inclusive lower bound.
        high (float): Inclusive upper bound.

    Returns:
        float: Value restricted to the bounds.
    '''
    number(value, 'value')
    number(low, 'low')
    number(high, 'high')
    if low > high:
        raise ValueError('low cannot exceed high')
    return max(low, min(high, value))


class SensorModel(Model):
    '''
    Store a sensor pose and provide repeatable reading noise.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in seconds.
        time_constant_s (float): Read-only device response time constant in seconds.
        _noise_std (float): Reading noise standard deviation in sensor units.
        _rng (random.Random): Seeded random generator used for simulated noise.
        _x_mm (float): World x coordinate of the sensor in mm.
        _y_mm (float): World y coordinate of the sensor in mm.
        _z_mm (float): Sensor height above the ground in mm.
        _yaw_deg (float): Counter-clockwise sensor yaw from the world x axis, in
            degrees.
        _pitch_deg (float): Fixed downward sensor pitch in degrees.
        _roll_deg (float): Fixed right-hand roll in degrees; it does not change the
            center ray.
        world (world_module.World): Active world, created as a default white field on
            first use if absent.
    '''
    def __init__(self, time_constant_s: float = 0.15, noise: float = 0.0, seed: int | None = None) -> None:
        '''
        Create a sensor model.

        Arg:
            time_constant_s (float): Response time in seconds.
            noise (float): Noise standard deviation in sensor units. Zero disables noise.
            seed (int | None): Random seed for repeatable noise.
        '''
        number(noise, 'noise', minimum=0)
        if seed is not None:
            integer(seed, 'seed')
        rng = random.Random(seed)
        super().__init__(time_constant_s)
        self._noise_std: float = noise
        self._rng: random.Random = rng

        self._x_mm: float = 0.0
        self._y_mm: float = 0.0
        self._z_mm: float = 0.0
        self._yaw_deg: float = 0.0
        self._pitch_deg: float = 0.0
        self._roll_deg: float = 0.0

    def step(self, dt_s: float) -> None:
        '''
        Validate a simulation step without sampling the sensor.

        Arg:
            dt_s (float): Positive step duration in seconds.
        '''
        number(dt_s, 'dt_s', positive=True)

    @property
    def world(self) -> world_module.World:
        '''
        Return the active world or create the default white field.
        '''
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
        values = (x_mm, y_mm, z_mm, yaw_deg)
        for name, value in zip(('x_mm', 'y_mm', 'z_mm', 'yaw_deg'), values):
            number(value, name)
        number(z_mm, 'z_mm', minimum=0)
        self._x_mm, self._y_mm, self._z_mm, self._yaw_deg = values

    def set_tilt(self, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> None:
        '''
        Set the fixed tilt of the sensor.

        Use Rz(yaw) * Ry(pitch) * Rx(roll) for rotation.

        Arg:
            pitch_deg (float): Downward pitch in degrees.
            roll_deg (float): Roll in degrees by the right hand rule.
        '''
        values = (pitch_deg, roll_deg)
        number(pitch_deg, 'pitch_deg')
        number(roll_deg, 'roll_deg')
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
        '''
        Add seeded Gaussian noise to a sensor value.

        Arg:
            value (float): Finite noise-free reading.

        Returns:
            float: Reading with noise, or the unchanged value when noise is zero.
        '''
        number(value, 'value')
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
    Measure ambient light and distance-dependent surface reflection.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in seconds.
        time_constant_s (float): Read-only device response time constant in seconds.
        _noise_std (float): Reading noise standard deviation in sensor units.
        _rng (random.Random): Seeded random generator used for simulated noise.
        _x_mm (float): World x coordinate of the sensor in mm.
        _y_mm (float): World y coordinate of the sensor in mm.
        _z_mm (float): Sensor height above the ground in mm.
        _yaw_deg (float): Counter-clockwise sensor yaw from the world x axis, in
            degrees.
        _pitch_deg (float): Fixed downward sensor pitch in degrees.
        _roll_deg (float): Fixed right-hand roll in degrees; it does not change the
            center ray.
        world (world_module.World): Active world, created as a default white field on
            first use if absent.
        reference_distance_mm (float): Maximum surface distance giving full distance
            strength, in mm.
        max_distance_mm (float): Maximum range for active surface reflection, in mm.
    '''

    def __init__(self, time_constant_s: float = 0.15, noise: float = 0.0,
                 seed: int | None = None, *, reference_distance_mm: float = 10.0,
                 max_distance_mm: float = 100.0) -> None:
        '''
        Create a colour sensor with a simple distance response.

        Arg:
            time_constant_s (float): Response time in seconds.
            noise (float): Reflection noise standard deviation.
            seed (int | None): Random seed for repeatable noise.
            reference_distance_mm (float): Distance with full signal strength in mm.
            max_distance_mm (float): Maximum surface distance in mm.
        '''
        number(reference_distance_mm, 'reference_distance_mm', positive=True)
        number(max_distance_mm, 'max_distance_mm', positive=True)
        if reference_distance_mm > max_distance_mm:
            raise ValueError("Reference distance cannot exceed maximum distance")
        super().__init__(time_constant_s, noise, seed)
        self.reference_distance_mm: float = reference_distance_mm
        self.max_distance_mm: float = max_distance_mm

    def reflection(self) -> int:
        '''
        Measure reflected light at the first surface along the centre ray.

        Add ambient light to the active reflected signal.
        Use a capped inverse-square distance response and cosine angle response.
        This is a simple model for calibration, not a hardware response curve.

        Returns:
            int: Total signal from 0 to 100. Return ambient light without a surface hit.
        '''
        number(self.reference_distance_mm, 'reference_distance_mm', positive=True)
        number(self.max_distance_mm, 'max_distance_mm', positive=True)
        if self.reference_distance_mm > self.max_distance_mm:
            raise ValueError('Reference distance cannot exceed maximum distance')
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
    Measure surface distance along the sensor center ray.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in seconds.
        time_constant_s (float): Read-only device response time constant in seconds.
        _noise_std (float): Reading noise standard deviation in sensor units.
        _rng (random.Random): Seeded random generator used for simulated noise.
        _x_mm (float): World x coordinate of the sensor in mm.
        _y_mm (float): World y coordinate of the sensor in mm.
        _z_mm (float): Sensor height above the ground in mm.
        _yaw_deg (float): Counter-clockwise sensor yaw from the world x axis, in
            degrees.
        _pitch_deg (float): Fixed downward sensor pitch in degrees.
        _roll_deg (float): Fixed right-hand roll in degrees; it does not change the
            center ray.
        world (world_module.World): Active world, created as a default white field on
            first use if absent.
        MAX_DISTANCE_MM (int): Maximum ultrasonic range and no-hit reading, in mm.
    '''
    # What the ultrasonic sensor returns when nothing is measured.
    MAX_DISTANCE_MM = 2000 # 2m

    def distance(self) -> int:
        '''
        Return the measured distance.

        Returns:
            int: Distance in mm. Return 2000 when no object is detected.
        '''
        d = self.world.raycast_distance(self._x_mm, self._y_mm, self._yaw_deg,
                                        self.MAX_DISTANCE_MM, z_mm=self._z_mm,
                                        pitch_deg=self._pitch_deg)
        return int(round(_clamp(self._noisy(d), 0, self.MAX_DISTANCE_MM)))


class ForceSensorModel(SensorModel):
    '''
    Store scripted force and button displacement.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in seconds.
        time_constant_s (float): Read-only device response time constant in seconds.
        _noise_std (float): Reading noise standard deviation in sensor units.
        _rng (random.Random): Seeded random generator used for simulated noise.
        _x_mm (float): World x coordinate of the sensor in mm.
        _y_mm (float): World y coordinate of the sensor in mm.
        _z_mm (float): Sensor height above the ground in mm.
        _yaw_deg (float): Counter-clockwise sensor yaw from the world x axis, in
            degrees.
        _pitch_deg (float): Fixed downward sensor pitch in degrees.
        _roll_deg (float): Fixed right-hand roll in degrees; it does not change the
            center ray.
        world (world_module.World): Active world, created as a default white field on
            first use if absent.
        force_n (float): Scripted force from 0 to 10 N.
        distance_mm (float): Scripted button displacement from 0 to 8 mm.
    '''
    def __init__(self) -> None:
        '''
        Create a released force sensor with zero force and displacement.
        '''
        super().__init__()
        self.force_n: float = 0.0
        self.distance_mm: float = 0.0

    def set_measurement(self, force_n: float = 0.0, distance_mm: float = 0.0) -> None:
        '''
        Set scripted force and button displacement.

        Arg:
            force_n (float): Force from 0 to 10 N.
            distance_mm (float): Button displacement from 0 to 8 mm.
        '''
        number(force_n, 'force_n', minimum=0, maximum=10)
        number(distance_mm, 'distance_mm', minimum=0, maximum=8)
        self.force_n, self.distance_mm = force_n, distance_mm
