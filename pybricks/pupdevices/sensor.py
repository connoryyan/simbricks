import random
from abc import ABC, abstractmethod

from sim.world import World, current_world

from ..parameters import Port
from .device import Device

# What the ultrasonic sensor returns when nothing is measured.
MAX_DISTANCE_MM = 2000 # 2m


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Sensor(ABC, Device):
    def __init__(self, port: Port, noise: float = 0.0, seed: int | None = None):
        '''
        Args:
            port (Port): Port the sensor is plugged into.
            world (World) : Use this world instead of the global one (handy in tests).
            noise (float): Standard deviation of Gaussian measurement noise, in the
                sensor's own unit (% for reflection, mm for distance). 0 means
                a perfect, fully deterministic sensor.
            seed (int): Seed for the noise, so noisy runs can be reproduced.
        '''
        super().__init__(port)
        self._model = 
        self._noise_std: float = noise
        self._rng: random.Random = random.Random(seed)

        self._x_mm = 0.0
        self._y_mm = 0.0
        self._z_mm = 0.0
        self._yaw_deg = 0.0
        self._pitch_deg = 0.0
        self._roll_deg = 0.0

    def _set_pose(self, x_mm: float, y_mm: float, z_mm: float, heading_deg: float) -> None:
        '''
        Called by whatever carries the sensor. Not part of the official API.
        '''
        self._x_mm, self._y_mm, self_y_mm, self._heading_deg = x_mm, y_mm, z_mm, heading_deg

    def _noisy(self, value: float) -> float:
        if self._noise_std > 0:
            value += self._rng.gauss(0.0, self._noise_std)
        return value


class ColorSensor(Sensor):
    '''
    LEGO SPIKE Color Sensor (reflection only for now).
    '''

    def reflection(self) -> int:
        value = self.world.get_reflection_at(self._x_mm, self._y_mm)
        return int(round(_clamp(self._noisy(value), 0, 100)))

    def ambient(self) -> int:
        raise NotImplementedError("ColorSensor.ambient() needs ambient light in the World")

    def color(self):
        raise NotImplementedError("ColorSensor.color() needs surface colors in the World")

    def hsv(self):
        raise NotImplementedError("ColorSensor.hsv() needs surface colors in the World")


class UltrasonicSensor(Sensor):
    '''
    LEGO SPIKE Ultrasonic Sensor. Looks along the heading of whatever carries it.
    '''

    def distance(self) -> int:
        '''
        distance() -> int: mm. Returns 2000 when nothing is measured.
        '''
        d = self.world.raycast_distance(self._x_mm, self._y_mm, self._yaw_deg,
                                        MAX_DISTANCE_MM)
        return int(round(_clamp(self._noisy(d), 0, MAX_DISTANCE_MM)))

    def presence(self) -> bool:
        # Other ultrasonic sensors are not simulated.
        return False
