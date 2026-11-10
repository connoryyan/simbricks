import pytest

from pybricks.parameters import Axis, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, ForceSensor, Motor, DCMotor
from pybricks.pupdevices.actuator import Actuator, _effective_gear_ratio, _is_int
from pybricks.pupdevices.sensor import IMU, Sensor
from sim.clock import clock
from sim.models import ForceSensorModel, IMUModel
from sim.models.model import Model


def test_gear_ratio():
    assert _is_int(3)
    assert not _is_int(True)
    assert not _is_int(3.0)
    assert _effective_gear_ratio(None) == 1
    assert _effective_gear_ratio('invalid') == 1
    assert _effective_gear_ratio([12, 36]) == 3
    assert _effective_gear_ratio([[12, 36], [20, 40]]) == 6
    assert _effective_gear_ratio([0, 36]) == 1
    assert _effective_gear_ratio([12.0, 36]) == 1
    assert _effective_gear_ratio([12]) == 1


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
    with pytest.raises(ValueError, match='stop mode'):
        motor._apply_then(None)


def test_motor_timeout_brakes(initialized, monkeypatch):
    motor = Motor(Port.A)
    monkeypatch.setattr(clock, 'advance_until', lambda *args, **kwargs: False)
    with pytest.raises(TimeoutError, match='timed out'):
        motor.run_angle(100, 90)
    assert motor.model._braking


def test_sensor_errors(initialized):
    sensor = Sensor(Port.A)
    with pytest.raises(TypeError, match='sensor model'):
        sensor.model
    color = ColorSensor(Port.B)
    with pytest.raises(NotImplementedError, match='surface colors'):
        color.color()
    with pytest.raises(NotImplementedError, match='surface colors'):
        color.hsv()
    force = ForceSensor(Port.C)
    with pytest.raises(ValueError):
        force.pressed(-1)
    with pytest.raises(ValueError):
        ForceSensorModel().set_measurement(distance_mm=9)


def test_imu_validation(initialized):
    with pytest.raises(TypeError):
        IMU(object())
    imu = IMU(IMUModel())
    for axis in ((0, 0, 0), (0, 0, float('inf')), (0, 0, 'z')):
        with pytest.raises(ValueError):
            imu.rotation(axis)
    assert -Axis.X == (-1, 0, 0)
    imu.model.in_use = True
    with pytest.raises(OSError, match='Stop'):
        imu.settings(heading_correction=359)
    for options in ({'bias_dps': float('nan')}, {'noise_dps': -1}):
        with pytest.raises(ValueError):
            IMUModel(**options)
    with pytest.raises(ValueError):
        imu.model.step(0)
    with pytest.raises(ValueError):
        imu.reset_heading(float('inf'))


def test_abstract_method_defaults():
    assert Model.step(None, 0.01) is None
    assert Actuator.stop() is None
    assert Actuator.brake() is None
