from sim.initialization import requires_init
from abc import ABC, abstractmethod

from ..parameters import Direction, Port, Stop
from sim.models.motor_model import MotorModel
from sim.clock import clock
from sim.validation import boolean, number

from .device import Device


def _is_int(num: object) -> bool:
    return isinstance(num, int) and not isinstance(num, bool)


def _effective_gear_ratio(gears: list | tuple | None) -> float:
    '''
    Calculate the gear ratio of a gear chain.

    Arg:
        gears (list | tuple | None): Gear teeth in one or more chains.

    Returns:
        float: Output gear teeth divided by input gear teeth across
               all chains.
    '''
    if gears is None:
        return 1.0
    if not isinstance(gears, (list, tuple)):
        raise TypeError('gears must be a list or tuple')
    if not gears:
        raise ValueError('gears cannot be empty')
    trains = gears if isinstance(gears[0], (list, tuple)) else (gears,)
    first_product = last_product = 1
    for train in trains:
        if not isinstance(train, (list, tuple)):
            raise TypeError('Each gear train must be a list or tuple')
        if not train:
            raise ValueError('Gear trains cannot be empty')
        for gear in train:
            if not _is_int(gear):
                raise TypeError('Gear teeth must be integers')
            if gear <= 0:
                raise ZeroDivisionError('Gears must be positive integers')
        first_product *= train[0]
        last_product *= train[-1]
    try:
        ratio = last_product / first_product
    except OverflowError:
        raise ValueError('Gear ratio must be finite') from None
    number(ratio, 'gear_ratio', positive=True)
    return ratio


class Actuator(Device):
    '''
    Represent a motor device with a physical model.

    Attributes:
        port (Port): Port the device is connected to.
        _model (MotorModel): Physical motor model storing raw shaft motion.
        _direction (Direction): Direction treated as positive for motor
            commands and
            readings.
    '''

    def __init__(
        self,
        port: Port,
        positive_direction: Direction = Direction.CLOCKWISE,
    ) -> None:
        '''
        Create a motor device and its physical model.

        Arg:
            port (Port): Port the motor is plugged into.
            positive_direction (Direction): Positive motor direction.
        '''
        if not isinstance(positive_direction, Direction):
            raise TypeError('positive_direction must be a Direction')
        super().__init__(port)
        self._direction: Direction = positive_direction
        self._model: MotorModel = MotorModel()

    @abstractmethod
    def stop() -> None:
        '''
        Release the motor.
        '''
        pass

    @abstractmethod
    def brake() -> None:
        '''
        Apply passive motor braking.
        '''
        pass


class DCMotor(Actuator):
    '''
    Control motor duty without angle feedback.

    Attributes:
        port (Port): Port the device is connected to.
        _model (MotorModel): Physical motor model storing raw shaft motion.
        _direction (Direction): Direction treated as positive for motor
            commands and
            readings.
    '''

    def __init__(
        self,
        port: Port,
        positive_direction: Direction = Direction.CLOCKWISE,
    ) -> None:
        '''
        Create a motor without angle feedback.

        Arg:
            port (Port): Port the motor is plugged into.
            positive_direction (Direction): Positive motor direction.
        '''
        super().__init__(port, positive_direction)

    @requires_init
    # Raise NotInitializedError if run this before running init() function.
    def dc(self, duty: float) -> None:
        '''
        Run the motor at a duty cycle.

        Arg:
            duty (float): Motor duty from -100 to 100 percent.
        '''
        number(duty, 'duty', minimum=-100, maximum=100)
        sign = self._direction.value
        speed = sign * duty / 100.0 * self._model.max_speed_dps
        self._model.run(speed)

    def stop(self) -> None:
        '''
        Release the motor and let its speed decay.
        '''
        self._model.stop()

    def brake(self) -> None:
        '''
        Slow the motor with passive electrical braking.
        '''
        self._model.brake()


class Motor(DCMotor):
    '''
    Control and measure motion at the gear output.

    Attributes:
        port (Port): Port the device is connected to.
        _model (MotorModel): Physical motor model storing raw shaft motion.
        _direction (Direction): Direction treated as positive for motor
            commands and
            readings.
        _angle_offset (float): Reported output angle offset in degrees, set by
            reset_angle().
        _action (None): Reserved action state; currently unused and None.
        _last_action_error (BaseException | None): Reserved action error;
            currently
            unused and None.
        model (MotorModel): Simulator model exposed for setup and inspection.
        positive_direction (Direction): Direction treated as positive at the
            gear
            output.
    '''

    def __init__(
        self,
        port: Port,
        positive_direction: Direction = Direction.CLOCKWISE,
        gears: list | tuple | None = None,
    ) -> None:
        '''
        Create a motor with angles and speeds at the gear output.

        Arg:
            port (Port): Port the motor is plugged into.
            positive_direction (Direction): Positive motor direction.
            gears (list | tuple | None): Gear teeth from motor to output.
        '''
        gear_ratio = _effective_gear_ratio(gears)
        super().__init__(port, positive_direction)
        self._model.gear_ratio = gear_ratio

        self._angle_offset: float = 0.0
        self._action: None = None
        self._last_action_error: BaseException | None = None

    @property
    def model(self) -> MotorModel:
        '''
        Return the motor model for the simulator.
        '''
        return self._model

    @property
    def positive_direction(self) -> Direction:
        '''
        Return the positive motor direction.
        '''
        return self._direction

    @requires_init
    def run(self, speed: float) -> None:
        '''
        Run the gear output at a constant speed.

        Arg:
            speed (float): Signed output speed in deg/s.
        '''
        number(speed, 'speed')
        raw_speed = self._direction.value * speed * self._model.gear_ratio
        number(raw_speed, 'Motor shaft speed')
        self._model.run(raw_speed)

    @requires_init
    def run_time(
        self,
        speed: float,
        time: float,
        then: Stop = Stop.HOLD,
        wait: bool = True,
    ) -> None:
        '''
        Run the gear output for a duration and apply a stop mode.

        With wait=False, only start the motor; no automatic stop is scheduled.

        Arg:
            speed (float): Signed output speed in deg/s.
            time (float): Non-negative duration in ms.
            then (Stop): Stop mode after a blocking move.
            wait (bool): Advance simulated time until the move finishes.
        '''
        if not isinstance(then, Stop):
            raise TypeError('then must be a Stop')
        number(time, 'time', minimum=0)
        boolean(wait, 'wait')
        self.run(speed)

        if wait:
            clock.advance_ms(time)
            self._apply_then(then)

    @requires_init
    def run_angle(
        self,
        speed: float,
        angle: float,
        then: Stop = Stop.HOLD,
        wait: bool = True,
    ) -> None:
        '''
        Run the motor through an angle.

        Arg:
            speed (float): Motor speed in deg/s.
            angle (float): Rotation angle in degrees.
            then (Stop): Stop mode after the move.
            wait (bool): Wait for the move to finish. Automatic completion
                requires True.
        '''
        if not isinstance(then, Stop):
            raise TypeError('then must be a Stop')
        number(speed, 'speed')
        number(angle, 'angle')
        boolean(wait, 'wait')
        if angle != 0 and speed == 0:
            raise ValueError("speed must be non-zero")

        direction = (1 if speed >= 0 else -1) * (1 if angle >= 0 else -1)

        start = self._physical_angle()
        target = start + direction * abs(angle)
        number(target * self._model.gear_ratio, 'Motor shaft target')

        def reached() -> bool:
            return (self._physical_angle() - target) * direction >= 0

        if angle == 0:
            self._apply_then(then, target_physical=start)
            return

        self.run(direction * abs(speed))

        if wait:
            completed = clock.advance_until(reached, timeout_ms=60_000)
            if not completed:
                self.brake()
                raise TimeoutError("Motor move timed out")

            self._apply_then(then, target_physical=target)

    def _apply_then(
        self,
        then: Stop,
        target_physical: float | None = None,
    ) -> None:
        '''
        Apply the stop mode after a motor move.

        Arg:
            then (Stop): Stop mode to apply.
            target_physical (float | None): Signed output target before the
                reset offset.
                Use the current motor angle when None.
        '''
        if not isinstance(then, Stop):
            raise TypeError('then must be a Stop')
        if target_physical is not None:
            number(target_physical, 'target_physical')
        if then == Stop.BRAKE:
            self.brake()
        elif then == Stop.COAST or then == Stop.COAST_SMART:
            self.stop()
        else:
            raw_target = (
                None if target_physical is None else self._direction.value
                * target_physical
                * self._model.gear_ratio)
            self._model.hold(raw_target)

    def _physical_angle(self) -> float:
        '''
        Return the signed output angle before the reset offset.
        '''
        return self._direction.value * self._model.output_angle

    @requires_init
    def angle(self) -> float:
        '''
        Return the gear output angle after the reset offset.

        Returns:
            float: Signed output angle in degrees.
        '''
        return self._physical_angle() - self._angle_offset

    @requires_init
    def speed(self) -> float:
        '''
        Return the signed gear output speed.

        Returns:
            float: Output speed in deg/s.
        '''
        return self._direction.value * self._model.output_speed

    def reset_angle(self, angle: float = 0.0) -> None:
        '''
        Set the reported output angle without moving the motor.

        Arg:
            angle (float): New reported output angle in degrees.
        '''
        number(angle, 'angle')
        self._angle_offset = self._physical_angle() - float(angle)

    @requires_init
    def stalled(self) -> bool:
        '''
        Return whether a commanded motor is nearly stationary.

        This simple speed check may be True during startup.
        It does not measure load or require a minimum stall duration.

        Returns:
            bool: Whether target speed is non-zero and actual speed is near
                zero.
        '''
        return self._model.is_stalled()

    @requires_init
    def hold(self) -> None:
        '''
        Stop the motor and actively hold its current shaft angle.
        '''
        self._action = None
        self._model.hold()
