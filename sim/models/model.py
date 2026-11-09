from abc import ABC, abstractmethod

from ..clock import clock


class Model(ABC):
    '''
    Define a device model.
    '''

    # Independent physical devices are stepped before bodies and controllers.
    step_priority = 0

    def __init__(self, time_constant_s: float = 0.15):
        '''
        Create the model.

        Arg:
        time_constant_s (float): Response time in seconds.
        '''
        self._time_constant_s: float = time_constant_s
        clock.add_steppable(self)

    def get_time_constant_s(self):
        return self._time_constant_s
    time_constant_s = property(get_time_constant_s)

    @abstractmethod
    def step(self, dt_s: float) -> None:
        pass
