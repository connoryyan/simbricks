import random
from abc import ABC, abstractmethod

from sim.models import SensorModel, ColorSensorModel, UltrasonicSensorModel

from ..parameters import Port
from .device import Device

# What the ultrasonic sensor returns when nothing is measured.
MAX_DISTANCE_MM = 2000 # 2m


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Sensor(Device):
    def __init__(self, port: Port):
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

    def set_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        heading_deg: float,
    ) -> None:
        self._model.set_pose(x_mm, y_mm, z_mm, heading_deg)


class ColorSensor(Sensor):
    '''
    LEGO SPIKE Color Sensor (reflection only for now).
    '''
    def __init__(self, port: Port):
        super().__init__(port)
        self._model = ColorSensorModel()

    def reflection(self) -> int:
        return self._model.reflection()

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
    def __init__(self, port: Port):
        super().__init__(port)
        self._model = UltrasonicSensorModel()

    def distance(self) -> int:
        '''
        distance() -> int: mm. Returns 2000 when nothing is measured.
        '''
        return self._model.distance()
