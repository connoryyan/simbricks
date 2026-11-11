from abc import ABC, abstractmethod

from ..clock import clock
from ..validation import number


class Model(ABC):
    '''
    Define a physical device updated by the simulation clock.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        _time_constant_s (float): Stored device response time constant in seconds.
        time_constant_s (float): Read-only device response time constant in seconds.
    '''

    # Independent physical devices are stepped before bodies and controllers.
    step_priority: float = 0

    def __init__(self, time_constant_s: float = 0.15) -> None:
        '''
        Create the model.

        Arg:
        time_constant_s (float): Response time in seconds.
        '''
        number(time_constant_s, 'time_constant_s', positive=True)
        self._time_constant_s: float = time_constant_s
        clock.add_steppable(self)

    def get_time_constant_s(self) -> float:
        '''
        Return the device response time constant in seconds.
        '''
        return self._time_constant_s

    @property
    def time_constant_s(self) -> float:
        '''
        Return the device response time constant in seconds.
        '''
        return self.get_time_constant_s()

    @abstractmethod
    def step(self, dt_s: float) -> None:
        '''
        Update the model by one simulation step.

        Arg:
            dt_s (float): Positive step duration in seconds.
        '''
        pass
