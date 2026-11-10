import math

from ..parameters import Axis, Side
from sim.models.imu_model import IMUModel


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

    def heading(self):
        '''
        Return continuous clockwise heading in degrees.
        '''
        return self.model.heading()

    def reset_heading(self, angle=0.0):
        '''
        Reset the reported heading without moving the robot.
        '''
        self.model.reset_heading(angle)

    def tilt(self, calibrated=True):
        '''
        Return pitch and roll as zero for a level robot.
        '''
        return (0, 0)

    def up(self, calibrated=True):
        '''
        Return the top side for a level hub.
        '''
        return Side.TOP

    def pitch(self):
        '''
        Return zero pitch in degrees for the simulator.
        '''
        return 0

    def roll(self):
        '''
        Return zero roll in degrees for the simulator.
        '''
        return 0

    def angular_velocity(self, axis=None, calibrated=True):
        '''
        Return right hand angular velocity in deg/s along an axis or as a tuple.
        '''
        rate = self.model.calibrated_rate if calibrated else self.model.raw_rate
        return _project((0.0, 0.0, rate), axis)

    def rotation(self, axis, calibrated=True):
        '''
        Return integrated right hand rotation in degrees along an axis.
        '''
        rotation = self.model.calibrated_rotation if calibrated else self.model.raw_rotation
        return _project((0.0, 0.0, rotation), axis)

    def acceleration(self, axis=None, calibrated=True):
        '''
        Return axle-centre acceleration including gravity in mm/s/s.
        '''
        return _project(self.model._acceleration, axis)

    def ready(self):
        return self.model.ready()

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
