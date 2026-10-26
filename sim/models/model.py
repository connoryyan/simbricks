from abc import ABC, abstractmethod


class Model(ABC):
    '''
    An abstract class of the model of a device.
    '''

    def __init__(self, time_constant_s: float = 0.15):
        '''
        Args:
            time_constant_s (float): Responding time (second)
        '''
        self._time_constant_s: float = time_constant_s

    def get_time_constant_s(self):
        return self._time_constant_s
    time_constant_s = property(get_time_constant_s)

    @abstractmethod
    def step(self, dt_s: float) -> None:
        pass