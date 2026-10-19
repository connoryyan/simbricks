from .parameters import Direction, Port, Stop
from abc import ABC, abstractmethod
from sim.motor_model import MotorModel
from sim.clock import clock
from tools import wait
# from typing import Collection 
# ABC for sized iterable container classes.


def _is_int(num):
    return isinstance(num, int) and not isinstance(num, bool)

def _effective_gear_ratio(gears: list[int] | list[list[int]]) -> float:
    '''
    Calculate gear ratio from a given gears parameter

    Args:
        gears (Collection): parameters of a gear chain (e.g. [12, 36] or 
                            [[12, 36], [20, 16, 40]])
    '''
    if gears is None or not isinstance(gears, list):
        return 1.0

    def chain_ratio(chain: list[int]) -> float:
        ratio = 1.0

        for i in range(len(chain) - 1):
            gear = chain[i]
            if not _is_int(gear) or gear == 0.0:
                return 1.0

            ratio *= chain[i + 1] / chain[i]

        return ratio 

    first = gears[0]
    if isinstance(first, list):
        total = 1.0
        for chain in gears:
            total *= chain_ratio(chain)
        return total

    return chain_ratio(gears)

class Device(ABC):
    '''
    Simulate an electronic component of the robot. 
    '''
    def __init__(self, port: Port):
        self.port = port


class Actuator(Device, ABC):
    '''
    Motor class ABC
    '''

    def __init__(self, port: Port, 
                 positive_direction: Direction = Direction.CLOCKWISE):
        super().__init__(port)
        self._direction = positive_direction
        self._model = MotorModel()
        

    @abstractmethod
    def stop():
        pass

    @abstractmethod
    def brake():
        pass


class DCMotor(Actuator):
    '''
    Motor without angle feedback.
    '''

    def __init__(self, port: Port, positive_direction: Direction = Direction.CLOCKWISE):
        super().__init__(port)
        self._model = MotorModel()
        # clock.register(self._model)

    def dc(self, duty: float) -> None:
        '''
        Run DC motor
        '''
        sign = self._direction.value
        speed = sign * duty / 100.0 * self._model.max_speed_dps
        self._model.run(speed)


class Motor(Actuator):
    '''
    Motor with angle feedback.
    '''

    def __init__(self, port: Port, positive_direction: Direction = Direction.CLOCKWISE):
        super().__init__(port)

    def run(self, speed: float):
        self._model.run(speed)

    def run_time(self, speed: float, time: float, then: Stop):
        self._model.run(speed)
        clock.advance_ms(time)

    def run_angle(self, speed: float, angle: float, then: Stop):
        self._model.run(speed)
        current_angle = self._model.angle()
        clock.advance_until(self._model.angle)
        
    
        

