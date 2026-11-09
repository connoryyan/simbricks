import random
from abc import ABC, abstractmethod

from .. import world as world_module
from .model import Model

# What the ultrasonic sensor returns when nothing is measured.
MAX_DISTANCE_MM = 2000 # 2m


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SensorModel(Model):
    def __init__(self, time_constant_s: float = 0.15, noise: float = 0.0, seed: int | None = None):
        '''
        Args:
            noise (float): Standard deviation of Gaussian measurement noise, in the
                sensor's own unit (% for reflection, mm for distance). 0 means
                a perfect, fully deterministic sensor.
            seed (int): Seed for the noise, so noisy runs can be reproduced.
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

    def set_pose(self, x_mm: float, y_mm: float, z_mm: float, heading_deg: float) -> None:
        '''
        Called by whatever carries the sensor. Not part of the official API.
        '''
        self._x_mm, self._y_mm, self._z_mm, self._yaw_deg = x_mm, y_mm, z_mm, heading_deg

    def _noisy(self, value: float) -> float:
        if self._noise_std > 0:
            value += self._rng.gauss(0.0, self._noise_std)
        return value


class ColorSensorModel(SensorModel):
    '''
    Model to simulate LEGO SPIKE Color Sensor
    '''

    def reflection(self) -> int:
        value = self.world.get_reflection_at(self._x_mm, self._y_mm)
        return int(round(_clamp(self._noisy(value), 0, 100)))


class UltrasonicSensorModel(SensorModel):
    '''
    Model to simulate LEGO SPIKE Ultrasonic Sensor. Looks along the heading of whatever carries it.
    '''

    def distance(self) -> int:
        '''
        distance() -> int: mm. Returns 2000 when nothing is measured.
        '''
        d = self.world.raycast_distance(self._x_mm, self._y_mm, self._yaw_deg,
                                        MAX_DISTANCE_MM)
        return int(round(_clamp(self._noisy(d), 0, MAX_DISTANCE_MM)))