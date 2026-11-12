import math
from typing import Callable

from sim.initialization import requires_init
from sim.validation import boolean, number, vector as validate_vector
from sim.models import (
    SensorModel,
    ColorSensorModel,
    UltrasonicSensorModel,
    ForceSensorModel,
    IMUModel,
)

from ..parameters import Axis, Color, Port, Side
from .device import Device


class Sensor(Device):
    '''
    Provide common sensor mounting and pose access.

    Attributes:
        port (Port): Port the device is connected to.
        _model (Model | None): Simulator model, or None before a concrete
            device creates
            one.
        model (SensorModel): Simulator model exposed for setup and inspection.
    '''

    def __init__(self, port: Port) -> None:
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

        Pitch and roll will not be set as they are constant for the simulated
        level robot.

        Arg:
            x_mm (float): World x coordinate in mm.
            y_mm (float): World y coordinate in mm.
            z_mm (float): Height above the ground in mm.
            yaw_deg (float): Counter-clockwise angle from the world x axis in
                degrees.
        '''
        self.model.set_pose(x_mm, y_mm, z_mm, yaw_deg)

    def set_tilt(self, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> None:
        '''
        Set the fixed tilt of the sensor.

        Arg:
            pitch_deg (float): Downward pitch in degrees.
            roll_deg (float): Roll in degrees by the right hand rule.
        '''
        self.model.set_tilt(pitch_deg, roll_deg)

    def pose(self) -> tuple[float, float, float, float, float, float]:
        '''
        Return the world pose of the sensor.

        Returns:
            tuple[float, float, float, float, float, float]: x, y, z in mm,
                then yaw, pitch and roll in degrees.
        '''
        return self.model.pose()


class ColorSensor(Sensor):
    '''
    Read reflected and ambient light from the simulated world.

    Attributes:
        port (Port): Port the device is connected to.
        _model (ColorSensorModel): Color sensor model for surface rays, ambient
            light
            and noise.
        model (SensorModel): Simulator model exposed for setup and inspection.
    '''

    def __init__(self, port: Port) -> None:
        '''
        Create a color sensor and its simulator model.

        Arg:
            port (Port): Port the sensor is plugged into.
        '''
        super().__init__(port)
        self._model: ColorSensorModel = ColorSensorModel()

    @requires_init
    def reflection(self) -> int:
        '''
        Return reflected light from the surface seen by the sensor.

        Returns:
            int: Total signal from 0 to 100. Return ambient light
                 without a surface hit.
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
    def color(self) -> Color | None:
        '''
        Raise NotImplementedError because surface colors are not simulated.
        '''
        raise NotImplementedError(
            "ColorSensor.color() needs surface colors in the World")

    @requires_init
    def hsv(self) -> tuple[float, float, float]:
        '''
        Raise NotImplementedError because surface HSV values are not simulated.
        '''
        raise NotImplementedError(
            "ColorSensor.hsv() needs surface colors in the World")


class UltrasonicSensor(Sensor):
    '''
    Measure the distance to a simulated surface.

    Attributes:
        port (Port): Port the device is connected to.
        _model (UltrasonicSensorModel): Ultrasonic model casting a ray from the
            sensor
            pose.
        model (SensorModel): Simulator model exposed for setup and inspection.
    '''

    def __init__(self, port: Port) -> None:
        '''
        Create an ultrasonic sensor and its simulator model.

        Arg:
            port (Port): Port the sensor is plugged into.
        '''
        super().__init__(port)
        self._model: UltrasonicSensorModel = UltrasonicSensorModel()

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
    Read scripted force and button displacement.

    Attributes:
        port (Port): Port the device is connected to.
        _model (ForceSensorModel): Force sensor model storing scripted force
            and
            displacement.
        model (SensorModel): Simulator model exposed for setup and inspection.
    '''

    def __init__(self, port: Port) -> None:
        '''
        Create a force sensor with scripted simulator input.

        Arg:
            port (Port): Port the sensor is plugged into.
        '''
        super().__init__(port)
        self._model: ForceSensorModel = ForceSensorModel()

    @requires_init
    def force(self) -> float:
        '''
        Return measured force in N.
        '''
        return self._model.force_n

    @requires_init
    def distance(self) -> float:
        '''
        Return button displacement in mm.
        '''
        return self._model.distance_mm

    @requires_init
    def pressed(self, force: float = 3) -> bool:
        '''
        Return True when measured force reaches the threshold in N.
        '''
        number(force, 'force', minimum=0)
        return self.force() >= force

    @requires_init
    def touched(self) -> bool:
        '''
        Return True when the button moves or measures a force.
        '''
        return self.distance() > 0 or self.force() > 0


def _project(
    vector: tuple[float, float, float] | list[float],
    axis: Axis | tuple[float, float, float] | list[float] | None,
) -> float | tuple[float, float, float]:
    '''
    Project a three-dimensional vector onto a normalized axis.

    Arg:
        vector (tuple | list): Three finite vector components.
        axis (Axis | tuple | list | None): Non-zero projection axis.
            Return the full vector when None.

    Returns:
        float | tuple: Component along the axis, or the full vector.
    '''
    vector = validate_vector(vector, 'vector', 3)
    if axis is None:
        return vector
    values = axis.value if isinstance(axis, Axis) else axis
    values = validate_vector(values, 'axis', 3)
    length = math.hypot(*values)
    number(length, 'axis length')
    if length == 0:
        raise ValueError('axis cannot be zero')
    return sum(v * (a / length) for v, a in zip(vector, values))


class IMU:
    '''
    Read the shared IMU model for a level robot.

    Attributes:
        _model (IMUModel | callable): IMU model or provider returning the
            current shared
            model.
        model (IMUModel): Resolved and validated IMU model.
    '''

    def __init__(self, model: IMUModel | Callable[[], IMUModel]) -> None:
        '''
        Connect the IMU interface to a simulator model.

        Arg:
            model (IMUModel | callable): Model or function returning the
                current model.
        '''
        if not isinstance(model, IMUModel) and not callable(model):
            raise TypeError('model must be an IMUModel or a model provider')
        self._model: IMUModel | Callable[[], IMUModel] = model

    @property
    def model(self) -> IMUModel:
        '''
        Return the current validated IMU model.
        '''
        model = self._model() if callable(self._model) else self._model
        if not isinstance(model, IMUModel):
            raise TypeError('IMU model provider must return an IMUModel')
        return model

    @requires_init
    def heading(self) -> float:
        '''
        Return continuous clockwise heading in degrees.
        '''
        return self.model.heading()

    @requires_init
    def reset_heading(self, angle: float = 0.0) -> None:
        '''
        Set reported clockwise heading without moving the body.

        Arg:
            angle (float): New reported heading in degrees.
        '''
        self.model.reset_heading(angle)

    @requires_init
    def tilt(self, calibrated: bool = True) -> tuple[int, int]:
        '''
        Return zero pitch and roll for a level robot.

        Arg:
            calibrated (bool): Compatibility flag; both values return the same
                tilt.

        Returns:
            tuple[int, int]: Pitch and roll in degrees.
        '''
        boolean(calibrated, 'calibrated')
        return (0, 0)

    @requires_init
    def up(self, calibrated: bool = True) -> Side:
        '''
        Return the top side of a level hub.

        Arg:
            calibrated (bool): Compatibility flag; both values return Side.TOP.

        Returns:
            Side: Side.TOP.
        '''
        boolean(calibrated, 'calibrated')
        return Side.TOP

    @requires_init
    def pitch(self) -> int:
        '''
        Return zero pitch in degrees for the simulator.
        '''
        return 0

    @requires_init
    def roll(self) -> int:
        '''
        Return zero roll in degrees for the simulator.
        '''
        return 0

    @requires_init
    def angular_velocity(
        self,
        axis: Axis | tuple[float, float, float] | list[float] | None = None,
        calibrated: bool = True,
    ) -> float | tuple[float, float, float]:
        '''
        Return right-hand angular velocity along an axis or as a vector.

        Arg:
            axis (Axis | tuple | list | None): Non-zero axis, or None for all
                components.
            calibrated (bool): Subtract the estimated gyro bias when True.

        Returns:
            float | tuple: Angular velocity in deg/s.
        '''
        boolean(calibrated, 'calibrated')
        rate = (self.model.calibrated_rate if calibrated
                else self.model.raw_rate)
        return _project((0.0, 0.0, rate), axis)

    @requires_init
    def rotation(
        self,
        axis: Axis | tuple[float, float, float] | list[float] | None,
        calibrated: bool = True,
    ) -> float:
        '''
        Return integrated right-hand rotation along an axis.

        Arg:
            axis (Axis | tuple | list): Non-zero projection axis.
            calibrated (bool): Use bias-corrected integration when True.

        Returns:
            float: Rotation in degrees.
        '''
        boolean(calibrated, 'calibrated')
        if axis is None:
            raise TypeError('axis must be an Axis or a three-component vector')
        rotation = (self.model.calibrated_rotation if calibrated
                    else self.model.raw_rotation)
        return _project((0.0, 0.0, rotation), axis)

    @requires_init
    def acceleration(
        self,
        axis: Axis | tuple[float, float, float] | list[float] | None = None,
        calibrated: bool = True,
    ) -> float | tuple[float, float, float]:
        '''
        Return axle-center acceleration including gravity.

        Arg:
            axis (Axis | tuple | list | None): Non-zero axis, or None for all
                components.
            calibrated (bool): Compatibility flag; simulated acceleration is
                unchanged.

        Returns:
            float | tuple: Acceleration in mm/s/s.
        '''
        boolean(calibrated, 'calibrated')
        return _project(self.model._acceleration, axis)

    @requires_init
    def ready(self) -> bool:
        '''
        Return whether a stationary calibration occurred in the last ten
            minutes.
        '''
        return self.model.ready()

    @requires_init
    def stationary(self) -> bool:
        '''
        Return whether the robot has been stationary for at least one second.
        '''
        return self.model.stationary()

    def settings(
        self,
        angular_velocity_threshold: float | None = None,
        acceleration_threshold: float | None = None,
        heading_correction: float | None = None,
    ) -> tuple[float, ...] | None:
        '''
        Get or set stationary thresholds and heading correction.

        Arg:
            angular_velocity_threshold (float | None): Stationary gyro limit in
                deg/s.
            acceleration_threshold (float | None): Stationary acceleration
                limit in mm/s/s.
            heading_correction (float | None): Reported degrees per true full
                rotation.

        Returns:
            tuple | None: Current settings when called without arguments.
        '''
        names = ('angular_velocity_threshold',
                 'acceleration_threshold', 'heading_correction')
        values = (angular_velocity_threshold,
                  acceleration_threshold, heading_correction)
        if all(v is None for v in values):
            return tuple(getattr(self.model, name) for name in names)
        for name, value in zip(names, values):
            if value is not None:
                number(value, name, positive=True)
        updated = tuple(getattr(self.model, name) if v is None else v
                        for name, v in zip(names, values))
        if self.model.in_use:
            raise OSError(
                'Stop the gyro drive base before changing IMU settings')
        for name, value in zip(names, updated):
            setattr(self.model, name, value)
