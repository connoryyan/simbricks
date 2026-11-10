from sim.initialization import requires_init
import math
from abc import ABC, abstractmethod

from ..parameters import Direction, Port, Stop
from sim.models.motor_model import MotorModel
from sim.clock import clock

from .device import Device


def _is_int(num):
    return isinstance(num, int) and not isinstance(num, bool)


def _effective_gear_ratio(gears: list[int] | list[list[int]]) -> float:
    '''
    Calculate the gear ratio of a gear chain.

    Arg:
        gears (list[int] | list[list[int]]): Gear teeth in one or more chains.

    Returns:
        float: Output gear teeth divided by input gear teeth across all chains.
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


class Actuator(Device):
    '''
    Define a motor device.
    '''

    def __init__(self, port: Port,
                 positive_direction: Direction = Direction.CLOCKWISE):
        super().__init__(port)
        self._direction: Direction = positive_direction
        self._model: MotorModel = MotorModel()

    @abstractmethod
    def stop():
        pass

    @abstractmethod
    def brake():
        pass


class DCMotor(Actuator):
    '''
    Represent a motor without angle feedback.
    '''

    def __init__(self, port: Port, positive_direction: Direction = Direction.CLOCKWISE):
        super().__init__(port, positive_direction)

    @requires_init
    def dc(self, duty: float) -> None:
        '''
        Run the motor at a duty cycle.
        '''
        sign = self._direction.value
        speed = sign * duty / 100.0 * self._model.max_speed_dps
        self._model.run(speed)

    def stop(self):
        self._model.stop()

    def brake(self):
        self._model.brake()


class Motor(DCMotor):
    '''
    Represent a motor with angle feedback.
    '''

    def __init__(self, port: Port, 
                 positive_direction: Direction = Direction.CLOCKWISE):
        super().__init__(port, positive_direction)

        self._angle_offset = 0.0
        self._action = None
        self._last_action_error = None

    @property
    def model(self) -> MotorModel:
        '''
        Return the motor model for the simulator.
        '''
        return self._model

    @property
    def positive_direction(self) -> Direction:
        return self._direction

    @requires_init
    def run(self, speed: float):
        if not math.isfinite(speed):
            raise ValueError("speed must be finite")
        self._model.run(self._direction.value * speed)

    @requires_init
    def run_time(self, speed: float, time: float, 
                 then: Stop = Stop.HOLD, wait: bool = True):
        if not isinstance(then, Stop):
            raise ValueError("Unsupported stop mode")
        if not math.isfinite(time) or time < 0:
            raise ValueError("time must be finite and non-negative")
        self.run(speed)

        if wait:
            clock.advance_ms(time)
            self._apply_then(then)

    @requires_init
    def run_angle(self, speed: float, angle: float, 
                  then: Stop = Stop.HOLD, wait: bool = True) -> None:
        '''
        Run the motor through an angle.

        Arg:
            speed (float): Motor speed in deg/s.
            angle (float): Rotation angle in degrees.
            then (Stop): Stop mode after the move.
            wait (bool): Wait for the move to finish. Automatic completion requires True.
        '''
        if not isinstance(then, Stop):
            raise ValueError("Unsupported stop mode")
        if not math.isfinite(speed) or not math.isfinite(angle):
            raise ValueError("speed and angle must be finite")
        if angle != 0 and speed == 0:
            raise ValueError("speed must be non-zero")

        direction = (1 if speed >= 0 else -1) * (1 if angle >= 0 else -1)

        start = self._physical_angle()
        target = start + direction * abs(angle)

        def reached() -> bool:
            return (self._physical_angle() - target) * direction >= 0

        if angle == 0:
            self._apply_then(then, target_physical=start)
            return

        if angle != 0:
            self.run(direction * abs(speed))

        if wait:
            completed = clock.advance_until(reached, timeout_ms=60_000)
            if not completed:
                self.brake()
                raise TimeoutError("Motor move timed out")

            self._apply_then(then, target_physical=target)

    def _apply_then(self, then: Stop,
                    target_physical: float | None = None) -> None:
        if then == Stop.BRAKE:
            self.brake()
        elif then == Stop.COAST or then == Stop.COAST_SMART:
            self.stop()
        elif then == Stop.HOLD:
            raw_target = (
                None
                if target_physical is None
                else self._direction.value * target_physical
            )
            self._model.hold(raw_target)
        else:
            raise ValueError("Unsupported stop mode")

    def _physical_angle(self) -> float:
        return self._direction.value * self._model.get_angle()

    @requires_init
    def angle(self):
        return self._physical_angle() - self._angle_offset

    @requires_init
    def speed(self):
        return self._direction.value * self._model.get_speed()

    def reset_angle(self, angle: float = 0.0) -> None:
        self._angle_offset = self._physical_angle() - float(angle)
    
    @requires_init
    def stalled(self) -> bool:
        return self._model.is_stalled()

    @requires_init
    def hold(self) -> None:
        self._action = None
        self._model.hold()
