import pytest

from pybricks.hubs import InventorHub, PrimeHub, _Buttons
from pybricks.parameters import Axis, Button, Direction, Port, Side
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait


def _make_robot():
    return DriveBase(Motor(Port.B, Direction.COUNTERCLOCKWISE),
                     Motor(Port.D), 56, 112)


def test_official_initialization_sequence(initialized):
    hub = PrimeHub()
    robot = _make_robot()
    robot.use_gyro(True)
    timer = StopWatch()
    while not hub.imu.ready() and timer.time() < 2000:
        wait(10)
    assert hub.imu.ready()
    hub.imu.reset_heading(0)
    assert hub.imu.model is robot.model.imu
    robot.turn(90)
    wait(500)
    assert hub.imu.heading() == pytest.approx(90, abs=1, rel=0)
    assert robot.angle() == pytest.approx(hub.imu.heading(), abs=5e-08, rel=0)
    with pytest.raises(RuntimeError):
        hub.imu.reset_heading(0)
    robot.stop()
    hub.imu.reset_heading(0)
    assert robot.angle() == 0
    assert hub.imu.up() == Side.TOP


def test_hub_calibrates_before_drivebase_exists(initialized):
    hub = PrimeHub()
    wait(1100)
    assert hub.imu.ready()
    model = hub.imu.model
    robot = _make_robot()
    assert robot.model.imu is model
    assert hub.imu.ready()


def test_hub_created_after_drivebase_adopts_existing_imu(initialized):
    robot = _make_robot()
    robot.use_gyro(True)
    robot.turn(45)
    robot.stop()
    wait(3000)
    model = robot.model.imu
    heading = model.heading()
    hub = PrimeHub()
    assert hub.imu.model is model
    assert hub.imu.heading() == pytest.approx(heading, abs=5e-08, rel=0)
    assert hub.imu.ready()


def test_multiple_hub_handles_share_hardware(initialized):
    first, second = (PrimeHub(), InventorHub())
    assert first.model is second.model
    robot = _make_robot()
    assert first.imu.model is second.imu.model
    assert first.imu.model is robot.model.imu
    first.model.pressed_buttons.add(Button.LEFT)
    assert second.buttons.pressed() == {Button.LEFT}
    second.buttons.pressed().clear()
    assert first.buttons.pressed() == {Button.LEFT}


def test_unsupported_hub_mounting_is_explicit(initialized):
    with pytest.raises(NotImplementedError):
        PrimeHub(top_side=Axis.X, front_side=Axis.Z)


def test_second_active_robot_is_rejected_without_leaking_body(initialized):
    hub = PrimeHub()
    first = _make_robot()
    with pytest.raises(ValueError):
        _make_robot()
    assert hub.model.body is first.model
    assert hub.imu.model is first.model.imu


def test_hub_button_configuration_has_explicit_errors(initialized):
    with pytest.raises(TypeError):
        _Buttons(None)
    hub = PrimeHub()
    hub.model.pressed_buttons = [Button.LEFT]
    with pytest.raises(TypeError, match='set'):
        hub.buttons.pressed()
    hub.model.pressed_buttons = {'LEFT'}
    with pytest.raises(TypeError, match='Button'):
        hub.buttons.pressed()
    hub.model.pressed_buttons = {Button.LEFT}
    assert hub.buttons.pressed() == {Button.LEFT}


def test_hub_rejects_multiple_existing_bodies(robot):
    DriveBase(Motor(Port.C), Motor(Port.D), 56, 112)
    with pytest.raises(ValueError, match='multiple'):
        PrimeHub()
