import math

import pytest

from pybricks.parameters import Axis
from pybricks.pupdevices.sensor import IMU, _project
from pybricks.tools import wait
from sim.clock import clock
from sim.hub import HubModel
from sim.models import IMUModel


def test_imu_validation(initialized):
    with pytest.raises(TypeError):
        IMU(object())
    imu = IMU(IMUModel())
    for axis in ((0, 0, 0), (0, 0, float('inf'))):
        with pytest.raises(ValueError):
            imu.rotation(axis)
    with pytest.raises(TypeError):
        imu.rotation((0, 0, 'z'))
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


def test_level_interfaces_and_gravity(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    assert imu.tilt() == (0, 0)
    assert imu.tilt(calibrated=False) == (0, 0)
    assert imu.pitch() == 0
    assert imu.roll() == 0
    assert imu.acceleration() == (0, 0, 9806.65)
    assert imu.acceleration(Axis.X) == 0
    assert imu.acceleration(-Axis.Z) == -9806.65


def test_heading_is_continuous_and_clockwise(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    for _ in range(40):
        robot.model.heading_rad -= math.radians(20)
        wait(5)
    assert imu.heading() == pytest.approx(800, abs=5e-08, rel=0)
    assert imu.rotation(Axis.Z) == pytest.approx(-800, abs=5e-08, rel=0)
    assert imu.rotation(-Axis.Z) == pytest.approx(800, abs=5e-08, rel=0)


def test_reset_only_changes_heading_offset(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    robot.turn(90)
    wait(500)
    pose, rotation = (robot.pose(), imu.rotation(Axis.Z))
    imu.reset_heading(42)
    assert imu.heading() == pytest.approx(42, abs=5e-08, rel=0)
    assert robot.pose() == pose
    assert imu.rotation(Axis.Z) == rotation


def test_placement_does_not_inject_gyro_rotation(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    robot.model.place(100, 200, 90)
    wait(5)
    assert imu.heading() == 0
    assert imu.angular_velocity(Axis.Z) == 0


def test_real_angular_velocity_has_right_hand_sign(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    robot.drive(0, 60)
    wait(1000)
    assert imu.angular_velocity(Axis.Z) < -59
    assert imu.angular_velocity(-Axis.Z) > 59
    assert imu.angular_velocity(Axis.X) == 0


def test_stationary_calibration_removes_bias_without_resetting_yaw(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    imu.model.bias_dps = 1.0
    assert not imu.ready()
    wait(500)
    assert not imu.stationary()
    wait(600)
    assert imu.ready()
    assert imu.stationary()
    assert imu.angular_velocity(Axis.Z) == pytest.approx(0, abs=5e-08, rel=0)
    assert imu.angular_velocity(Axis.Z, calibrated=False) == pytest.approx(1, abs=5e-08, rel=0)
    assert imu.heading() == pytest.approx(-1, abs=0.01, rel=0)
    heading = imu.heading()
    wait(2000)
    assert imu.heading() == pytest.approx(heading, abs=5e-08, rel=0)
    assert imu.rotation(Axis.Z, calibrated=False) > 3


def test_translation_is_not_stationary_and_does_not_learn_bias(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    imu.model.bias_dps = 1
    robot.drive(100, 0)
    wait(2000)
    assert not imu.stationary()
    assert not imu.ready()
    assert imu.angular_velocity(Axis.Z) == pytest.approx(1, abs=5e-08, rel=0)


def test_constant_bias_above_stationary_threshold_can_calibrate(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    imu.model.bias_dps = 10
    wait(1100)
    assert imu.ready()
    assert imu.angular_velocity(Axis.Z) == pytest.approx(0, abs=5e-08, rel=0)
    assert imu.angular_velocity(Axis.Z, calibrated=False) == pytest.approx(10, abs=5e-08, rel=0)


def test_accelerometer_does_not_correct_yaw_drift(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    imu.model.bias_dps = 1
    robot.drive(100, 0)
    wait(1000)
    assert imu.heading() == pytest.approx(-1, abs=5e-08, rel=0)
    assert imu.tilt() == (0, 0)


def test_gyro_drivebase_uses_imu_and_reset_releases_hold(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    robot.use_gyro(True)
    robot.turn(90)
    wait(500)
    assert imu.heading() == pytest.approx(90, abs=1, rel=0)
    assert robot.angle() == pytest.approx(imu.heading(), abs=5e-08, rel=0)
    with pytest.raises(RuntimeError):
        imu.reset_heading(0)
    robot.reset(distance=10, angle=20)
    assert imu.heading() == 20
    assert robot.angle() == 20
    assert robot.distance() == 10
    imu.reset_heading(0)
    assert robot.angle() == 0


def test_reading_does_not_advance_time_or_resample_noise(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    imu.model.noise_dps = 0.1
    wait(5)
    first = imu.angular_velocity()
    for _ in range(100):
        assert imu.angular_velocity() == first
    assert clock.time_ms == 5


def test_axis_and_settings_validation_is_atomic(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    for axis in ((0, 0, 0), (1, 2), (0, 0, float('nan'))):
        with pytest.raises(ValueError):
            imu.angular_velocity(axis)
    with pytest.raises(TypeError):
        imu.angular_velocity('z')
    before = imu.settings()
    with pytest.raises(ValueError):
        imu.settings(angular_velocity_threshold=5, heading_correction=0)
    assert imu.settings() == before
    imu.settings(heading_correction=180)
    robot.model.heading_rad -= math.radians(45)
    wait(5)
    assert imu.heading() == pytest.approx(90, abs=5e-08, rel=0)


def test_detach_removes_imu_and_body_from_clock(hub_robot):
    hub, robot = hub_robot
    imu = hub.imu
    robot.model.detach()
    assert imu.model not in clock.steppables
    assert robot.model not in clock.steppables


def test_imu_seed_repeats_noise():
    first = IMUModel(noise_dps=0.1, seed=42)
    second = IMUModel(noise_dps=0.1, seed=42)
    first.step(0.005)
    second.step(0.005)
    assert first.raw_rate == second.raw_rate


def test_project_checks_the_input_vector():
    assert _project([1, 2, 3], None) == (1, 2, 3)
    assert _project((1, 2, 3), Axis.Z) == 3
    for value in (None, '123', (1, 2, True)):
        with pytest.raises(TypeError):
            _project(value, None)
    for value in ((1, 2), (1, 2, float('nan'))):
        with pytest.raises(ValueError):
            _project(value, Axis.Z)


def test_imu_configuration_is_checked_before_stepping():
    imu = IMUModel()
    for name, value, error in (
        ('bias_dps', '0', TypeError),
        ('noise_dps', -1, ValueError),
        ('angular_velocity_threshold', 0, ValueError),
        ('acceleration_threshold', float('nan'), ValueError),
    ):
        previous = getattr(imu, name)
        setattr(imu, name, value)
        with pytest.raises(error):
            imu.step(0.01)
        assert imu._elapsed == 0
        assert imu.raw_rotation == 0
        setattr(imu, name, previous)
    imu.heading_correction = 0
    with pytest.raises(ValueError):
        imu.heading()
    imu.heading_correction = 360
    imu.in_use = 'yes'
    with pytest.raises(TypeError):
        imu.reset_heading(10)
    assert imu.heading() == 0


def test_imu_provider_flags_and_axes(initialized):
    imu = IMU(IMUModel())
    for operation in (
        lambda: IMU(lambda: None).heading(),
        lambda: imu.tilt(calibrated=1),
        lambda: imu.up(calibrated=None),
        lambda: imu.angular_velocity(calibrated='yes'),
        lambda: imu.rotation(Axis.Z, calibrated=1),
        lambda: imu.rotation(None),
        lambda: imu.acceleration(calibrated=0),
        lambda: imu.angular_velocity((True, 0, 1)),
        lambda: imu.settings(angular_velocity_threshold=5, heading_correction='360'),
    ):
        with pytest.raises(TypeError):
            operation()
    assert imu.settings() == (2, 2500, 360)
    with pytest.raises(ValueError):
        imu.acceleration((1.7e+308, 1.7e+308, 1.7e+308))
    with pytest.raises(TypeError):
        HubModel().bind_body(object())
