import math

from sim.models import SensorModel, ColorSensorModel, UltrasonicSensorModel, ForceSensorModel

from ..parameters import Port
from .device import Device

class Sensor(Device):
    def __init__(self, port: Port):
        '''
        Create the sensor.

        Arg:
            port (Port): Port the sensor is plugged into.
        '''
        super().__init__(port)

    @property
    def model(self) -> SensorModel:
        '''
        Return the sensor model for the simulator.
        '''
        if not isinstance(self._model, SensorModel):
            raise TypeError("Sensor has no sensor model")
        return self._model

    def set_pose(
        self,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        yaw_deg: float,
    ) -> None:
        '''
        Set the world position and yaw of the sensor.

        Keep the fixed pitch and roll unchanged. Use this method in the simulator.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.
            z_mm (float): Height above the ground in mm.
            yaw_deg (float): Counter-clockwise angle from the world x axis in degrees.
        '''
        self.model.set_pose(x_mm, y_mm, z_mm, yaw_deg)

    def set_tilt(self, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> None:
        '''
        Set the fixed tilt of the sensor.

        Use Rz(yaw) * Ry(pitch) * Rx(roll) for rotation.

        Arg:
            pitch_deg (float): Downward pitch in degrees.
            roll_deg (float): Roll in degrees by the right hand rule.
        '''
        self.model.set_tilt(pitch_deg, roll_deg)

    def pose(self) -> tuple[float, float, float, float, float, float]:
        '''
        Return the world pose of the sensor.

        Returns:
            tuple[float, float, float, float, float, float]: x, y, z in mm, then yaw, pitch and roll in degrees.
        '''
        return self.model.pose()


class ColorSensor(Sensor):
    '''
    Represent a LEGO SPIKE color sensor.
    '''
    def __init__(self, port: Port):
        super().__init__(port)
        self._model = ColorSensorModel()

    def reflection(self) -> int:
        '''
        Return reflected light from the surface seen by the sensor.

        Returns:
            int: Total signal from 0 to 100. Return ambient light without a surface hit.
        '''
        return self._model.reflection()

    def ambient(self) -> int:
        '''
        Return ambient light without the active reflected signal.

        Returns:
            int: Ambient light from 0 to 100 in the simulated signal scale.
        '''
        return self._model.ambient()

    def color(self):
        raise NotImplementedError("ColorSensor.color() needs surface colors in the World")

    def hsv(self):
        raise NotImplementedError("ColorSensor.hsv() needs surface colors in the World")


class UltrasonicSensor(Sensor):
    '''
    Represent a LEGO SPIKE ultrasonic sensor.
    '''
    def __init__(self, port: Port):
        super().__init__(port)
        self._model = UltrasonicSensorModel()

    def distance(self) -> int:
        '''
        Return the measured distance.

        Returns:
            int: Distance in mm. Return 2000 when no object is detected.
        '''
        return self._model.distance()


class ForceSensor(Sensor):
    '''
    Provide a LEGO SPIKE force sensor with scripted simulator input.
    '''
    def __init__(self, port: Port):
        super().__init__(port)
        self._model = ForceSensorModel()

    def force(self):
        '''
        Return measured force in N.
        '''
        return self._model.force_n

    def distance(self):
        '''
        Return button displacement in mm.
        '''
        return self._model.distance_mm

    def pressed(self, force=3):
        '''
        Return True when measured force reaches the threshold in N.
        '''
        if not math.isfinite(force) or force < 0:
            raise ValueError('Force threshold must be finite and non-negative')
        return self.force() >= force

    def touched(self):
        '''
        Return True when the button moves or measures a force.
        '''
        return self.distance() > 0 or self.force() > 0
