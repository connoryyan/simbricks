from abc import ABC, abstractmethod

from ..parameters import Direction, Port, Stop
from sim.models.motor_model import MotorModel
from sim.clock import clock

from .device import Device


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


class Actuator(Device, ABC):
    '''
    Motor class ABC
    '''

    def __init__(self, port: Port,
                 positive_direction: Direction = Direction.CLOCKWISE):
        super().__init__(port)
        self._direction: Direction = positive_direction
        self._model: MotorModel = MotorModel()
        clock.add_steppable(self._model)

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
        self._model.run(self._direction.value * speed)

    def run_time(self, speed: float, time: float, then: Stop, wait: bool = True):
        self.run(speed)

        if wait:
            clock.advance_ms(time)
            self._apply_then(then)

    def run_angle(self, speed: float, angle: float, then: Stop = None, wait: bool = True):
        direction = (1 if speed >= 0 else -1) * (1 if angle >= 0 else -1)
        self.run(direction * abs(speed))

        start_angle = self._model.get_angle()

        def reached() -> bool:
            traveled = (self._model.get_angle() - start_angle) * direction
            return traveled >= abs(angle)

        if wait:
            clock.advance_until(reached, timeout_ms=60_000)
            self._apply_then(then)

    def _apply_then(self, then: Stop) -> None:
        if then == Stop.BRAKE:
            self.brake()
        elif then == Stop.COAST or then == Stop.COAST_SMART:
            self.stop()
        elif then == Stop.HOLD:
            self.brake() # Leave hold later

    def stop(self):
        self._model.stop()

    def brake(self):
        self._model.brake()

    def angle(self):
        return self._direction.value * self._model.get_angle()

    def speed(self):
        return self._direction.value * self._model.get_speed()

    def reset_angle(self, angle=None):
        self._model.angle = (0.0 if angle is None 
                             else self._direction.value * angle)
    
    def stalled(self) -> bool:
        return self._model.is_stalled()