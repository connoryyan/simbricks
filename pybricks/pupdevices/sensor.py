from sim.initialization import requires_init
import math

from sim.models import SensorModel, ColorSensorModel, UltrasonicSensorModel, ForceSensorModel, IMUModel

from ..parameters import Axis, Port, Side
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

    @requires_init
    def reflection(self) -> int:
        '''
        Return reflected light from the surface seen by the sensor.

        Returns:
            int: Total signal from 0 to 100. Return ambient light without a surface hit.
        '''
        return self._model.reflection()

    @requires_init
    def ambient(self) -> int:
        '''
        Return ambient light without the active reflected signal.

        Returns:
            int: Ambient light from 0 to 100 in the simulated signal scale.
        '''
        return self._model.ambient()

    @requires_init
    def color(self):
        raise NotImplementedError("ColorSensor.color() needs surface colors in the World")

    @requires_init
    def hsv(self):
        raise NotImplementedError("ColorSensor.hsv() needs surface colors in the World")


class UltrasonicSensor(Sensor):
    '''
    Represent a LEGO SPIKE ultrasonic sensor.
    '''
    def __init__(self, port: Port):
        super().__init__(port)
        self._model = UltrasonicSensorModel()

    @requires_init
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

    @requires_init
    def force(self):
        '''
        Return measured force in N.
        '''
        return self._model.force_n

    @requires_init
    def distance(self):
        '''
        Return button displacement in mm.
        '''
        return self._model.distance_mm

    @requires_init
    def pressed(self, force=3):
        '''
        Return True when measured force reaches the threshold in N.
        '''
        if not math.isfinite(force) or force < 0:
            raise ValueError('Force threshold must be finite and non-negative')
        return self.force() >= force

    @requires_init
    def touched(self):
        '''
        Return True when the button moves or measures a force.
        '''
        return self.distance() > 0 or self.force() > 0


def _project(vector, axis):
    if axis is None:
        return vector
    values = axis.value if isinstance(axis, Axis) else axis
    if not isinstance(values, (tuple, list)) or len(values) != 3:
        raise ValueError('axis must be an Axis or a three-component vector')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError('axis must be finite')
    length = math.sqrt(sum(v*v for v in values))
    if length == 0:
        raise ValueError('axis cannot be zero')
    return sum(v*a for v, a in zip(vector, values))/length


class IMU:
    '''
    Provide Pybricks-style IMU readings for a level robot.
    '''
    def __init__(self, model: IMUModel):
        if not isinstance(model, IMUModel) and not callable(model):
            raise TypeError('model must be an IMUModel or a model provider')
        self._model = model

    @property
    def model(self):
        return self._model() if callable(self._model) else self._model

    @requires_init
    def heading(self):
        '''
        Return continuous clockwise heading in degrees.
        '''
        return self.model.heading()

    @requires_init
    def reset_heading(self, angle=0.0):
        '''
        Reset the reported heading without moving the robot.
        '''
        self.model.reset_heading(angle)

    @requires_init
    def tilt(self, calibrated=True):
        '''
        Return pitch and roll as zero for a level robot.
        '''
        return (0, 0)

    @requires_init
    def up(self, calibrated=True):
        '''
        Return the top side for a level hub.
        '''
        return Side.TOP

    @requires_init
    def pitch(self):
        '''
        Return zero pitch in degrees for the simulator.
        '''
        return 0

    @requires_init
    def roll(self):
        '''
        Return zero roll in degrees for the simulator.
        '''
        return 0

    @requires_init
    def angular_velocity(self, axis=None, calibrated=True):
        '''
        Return right hand angular velocity in deg/s along an axis or as a tuple.
        '''
        rate = self.model.calibrated_rate if calibrated else self.model.raw_rate
        return _project((0.0, 0.0, rate), axis)

    @requires_init
    def rotation(self, axis, calibrated=True):
        '''
        Return integrated right hand rotation in degrees along an axis.
        '''
        rotation = self.model.calibrated_rotation if calibrated else self.model.raw_rotation
        return _project((0.0, 0.0, rotation), axis)

    @requires_init
    def acceleration(self, axis=None, calibrated=True):
        '''
        Return axle-centre acceleration including gravity in mm/s/s.
        '''
        return _project(self.model._acceleration, axis)

    @requires_init
    def ready(self):
        return self.model.ready()

    @requires_init
    def stationary(self):
        return self.model.stationary()

    def settings(self, angular_velocity_threshold=None, acceleration_threshold=None,
                 heading_correction=None):
        '''
        Get or set stationary thresholds and heading correction.

        Arg:
            angular_velocity_threshold (float | None): Stationary gyro limit in deg/s.
            acceleration_threshold (float | None): Stationary acceleration limit in mm/s/s.
            heading_correction (float | None): Reported degrees per true full rotation.

        Returns:
            tuple | None: Current settings when called without arguments.
        '''
        names = ('angular_velocity_threshold', 'acceleration_threshold', 'heading_correction')
        values = (angular_velocity_threshold, acceleration_threshold, heading_correction)
        if all(v is None for v in values):
            return tuple(getattr(self.model, name) for name in names)
        updated = tuple(getattr(self.model, name) if v is None else float(v)
                        for name, v in zip(names, values))
        if not all(math.isfinite(v) and v > 0 for v in updated):
            raise ValueError('IMU settings must be finite and positive')
        if self.model.in_use:
            raise OSError('Stop the gyro drive base before changing IMU settings')
        for name, value in zip(names, updated):
            setattr(self.model, name, value)
