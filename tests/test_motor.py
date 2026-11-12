import pytest

from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import DCMotor, Motor
from pybricks.pupdevices.actuator import Actuator
from pybricks.tools import wait
from sim.clock import clock
from sim.models import MotorModel
from sim.models.model import Model


def test_dc_motor_and_stall(initialized):
    motor = DCMotor(Port.A, Direction.COUNTERCLOCKWISE)
    motor.dc(50)
    assert motor._model._target_speed_dps == -750
    motor.brake()
    assert motor._model._braking
    motor.stop()
    assert not motor._model._braking
    motor = Motor(Port.B)
    motor.run(100)
    assert motor.stalled()
    clock.advance_ms(100)
    assert not motor.stalled()
    motor.hold()
    assert motor.model._hold_angle_deg == motor.model.angle
    with pytest.raises(ValueError, match='finite'):
        motor.run(float('nan'))


def test_motor_nonblocking_and_coast(initialized):
    motor = Motor(Port.A)
    motor.run_time(100, 10, wait=False)
    assert clock.time_ms == 0
    motor.run_angle(100, 10, wait=False)
    assert clock.time_ms == 0
    for stop in (Stop.COAST, Stop.COAST_SMART):
        motor.run_angle(100, 0, then=stop)
        assert motor.model._target_speed_dps == 0
        assert not motor.model._braking
    with pytest.raises(TypeError, match='then'):
        motor._apply_then(None)


def test_motor_timeout_brakes(initialized, monkeypatch):
    motor = Motor(Port.A)
    monkeypatch.setattr(clock, 'advance_until', lambda *args, **kwargs: False)
    with pytest.raises(TimeoutError, match='timed out'):
        motor.run_angle(100, 90)
    assert motor.model._braking


def test_abstract_method_defaults():
    assert Model.step(None, 0.01) is None
    assert Actuator.stop() is None
    assert Actuator.brake() is None


def test_motor_speed_limit_cannot_be_corrupted():
    model = MotorModel()
    for value in (None, True, '100'):
        with pytest.raises(TypeError):
            model.max_speed_dps = value
        assert model.max_speed_dps == 1500
    for value in (0, -1, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            model.max_speed_dps = value
        assert model.max_speed_dps == 1500
    model.max_speed_dps = 100
    model.run(200)
    assert model._target_speed_dps == 100


def test_one_model_per_motor(initialized):
    for motor_type in (DCMotor, Motor):
        clock._steppables.clear()
        motor_type(Port.A)
        assert len(clock.steppables) == 1
        assert isinstance(next(iter(clock.steppables)), MotorModel)


def test_signed_moves_from_nonzero_angle_in_both_motor_directions(initialized):
    for direction in Direction:
        for speed, angle in ((300, 90), (-300, 90), (300, -90), (-300, -90)):
            clock._steppables.clear()
            motor = Motor(Port.A, direction)
            motor.run_angle(300, 120)
            wait(2500)
            motor.reset_angle(37)
            start = motor.angle()
            before = clock.time_ms
            motor.run_angle(speed, angle)
            assert clock.time_ms > before
            assert clock.time_ms - before < 2000
            wait(2500)
            expected = start + (90 if speed * angle > 0 else -90)
            assert motor.angle() == pytest.approx(expected, abs=0.1, rel=0)
            assert motor.speed() == pytest.approx(0, abs=0.1, rel=0)


def test_run_time_defaults_to_hold(initialized):
    motor = Motor(Port.A)
    motor.run_time(300, 100)
    held_angle = motor.angle()
    wait(2500)
    assert motor.angle() == pytest.approx(held_angle, abs=0.1, rel=0)
    assert motor.speed() == pytest.approx(0, abs=0.1, rel=0)


def test_zero_angle_does_not_start_or_advance_time(initialized):
    motor = Motor(Port.A)
    motor.run_angle(0, 0)
    assert clock.time_ms == 0
    assert motor.angle() == 0
    assert motor.model._target_speed_dps == 0


def test_invalid_arguments_do_not_change_running_motor(initialized):
    motor = Motor(Port.A)
    motor.run(123)
    for move in (
        lambda: motor.run_angle(300, 90, then=None),
        lambda: motor.run_time(300, 100, then=None),
    ):
        with pytest.raises(TypeError):
            move()
        assert motor.model._target_speed_dps == 123
        assert clock.time_ms == 0
    invalid_moves = (
        lambda: motor.run_angle(0, 90),
        lambda: motor.run_angle(float('nan'), 90),
        lambda: motor.run_angle(300, float('inf')),
        lambda: motor.run_time(300, -1),
        lambda: motor.run_time(300, float('nan')),
    )
    for move in invalid_moves:
        with pytest.raises(ValueError):
            move()
        assert motor.model._target_speed_dps == 123
        assert clock.time_ms == 0


def test_explicit_brake_mode(initialized):
    motor = Motor(Port.A)
    motor.run_angle(-300, 90, then=Stop.BRAKE)
    assert motor.angle() <= -90
    assert motor.model._braking
    wait(1000)
    assert motor.speed() == pytest.approx(0, abs=0.01, rel=0)


def test_motor_arguments_preserve_running_state(initialized):
    for bad in [None, '10', True, float('nan'), float('inf')]:
        error = ValueError if isinstance(bad, float) else TypeError
        motor = Motor(Port.A)
        motor.run(123)
        for operation in (
            lambda: motor.dc(bad),
            lambda: motor.run(bad),
            lambda: motor.run_time(100, bad),
            lambda: motor.run_angle(100, bad),
            lambda: motor.reset_angle(bad),
            lambda: motor.model.run(bad),
            lambda: motor.model.hold(
                bad) if bad is not None else motor.model.step(bad),
            lambda: motor.model.step(bad),
            lambda: motor.model.is_stalled(bad),
            lambda: setattr(motor.model, 'gear_ratio', bad),
        ):
            with pytest.raises(error):
                operation()
            assert motor.model._target_speed_dps == 123
            assert motor.model._hold_angle_deg is None
            assert motor.model.gear_ratio == 1
            assert motor.angle() == 0
            assert clock.time_ms == 0


def test_motor_flags_ranges_and_overflow(initialized):
    motor = Motor(Port.A, gears=[12, 36])
    motor.run(123)
    for operation in (
        lambda: motor.run_time(100, 10, wait='yes'),
        lambda: motor.run_angle(100, 90, wait=1),
        lambda: motor.run_time(100, 10, then=1),
        lambda: motor.run_angle(100, 90, then='hold'),
    ):
        with pytest.raises(TypeError):
            operation()
    for operation in (
        lambda: motor.dc(101),
        lambda: motor.dc(-101),
        lambda: motor.model.step(0),
        lambda: motor.model.is_stalled(-1),
        lambda: motor.run(1e+308),
        lambda: motor.run_angle(100, 1e+308),
    ):
        with pytest.raises(ValueError):
            operation()
    assert motor.model._target_speed_dps == 369
    assert clock.time_ms == 0
    for gears in ([1, 10 ** 400], [10 ** 400, 1]):
        before = clock.steppables
        with pytest.raises(ValueError):
            Motor(Port.B, gears=gears)
        assert clock.steppables == before
