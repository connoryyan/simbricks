import pytest

import sim
import world
from maps.flat_white.world import FlatWhiteWorld
from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase, _Profile
from sim.clock import clock
from sim.models import DriveBaseModel, MotorModel


@pytest.fixture
def robot(initialized):
    return DriveBase(Motor(Port.A), Motor(Port.B), 56, 112)


def test_profile_validation_and_positions():
    for args in ((float('nan'), 1, 1), (1, 0, 1), (1, 1, -1)):
        with pytest.raises(ValueError):
            _Profile(*args)
    profile = _Profile(-100, 10, 10)
    assert profile.at(0.5) == (-1.25, -5)
    assert profile.at(2) == (-15, -10)
    assert profile.at(10.5) == (-98.75, -5)
    assert profile.at(11) == (-100, 0)
    assert _Profile(0, 1, 1).at(0) == (0, 0)


def test_robot_settings_and_stop_modes(robot):
    assert robot.settings() == (200, 700, 200, 1000)
    robot.settings(straight_speed=100)
    robot.settings(straight_acceleration=200)
    robot.settings(turn_rate=90)
    robot.settings(turn_acceleration=300)
    assert robot.settings() == (100, 200, 90, 300)
    assert not robot.stalled()
    robot.use_gyro(False)
    robot.reset(distance=10, angle=20)
    assert robot.distance() == 10
    assert robot.angle() == 20
    for mode in (Stop.COAST, Stop.COAST_SMART, Stop.BRAKE):
        robot.straight(5, then=mode)
        assert robot.done()
        assert robot._left.model._braking == (mode == Stop.BRAKE)
    robot.turn(5, wait=False)
    assert not robot.done()
    clock.advance_until(robot.done, timeout_ms=5000)
    assert robot.done()
    assert repr(Color(1, 2, 3)) == 'Color(h=1, s=2, v=3)'


def test_robot_gyro_brake_and_maneuver(robot):
    robot.use_gyro(True)
    robot.drive(100, 10)
    assert robot.model.imu.in_use
    robot.stop()
    robot.turn(10)
    assert robot.model.imu.in_use
    robot.brake()
    assert not robot.model.imu.in_use
    assert len(robot.state()) == 4


def test_invalid_body_geometry():
    left, right = MotorModel(), MotorModel()
    for diameter, track, error in (('56', 112, TypeError), (56, None, TypeError), (0, 112, ValueError)):
        with pytest.raises(error):
            DriveBaseModel(left_model=left, right_model=right, wheel_diameter=diameter, axle_track=track)
    with pytest.raises(TypeError):
        DriveBase(object(), object(), 56, 112)


def test_invalid_physics_steps_and_surface(robot):
    for dt in (0, -1, float('nan')):
        with pytest.raises(ValueError):
            robot.model.step(dt)
    field = FlatWhiteWorld()
    world.current_world = field
    robot.model.configure_friction(yaw_inertia_kg_m2=0.01)
    assert robot.model.turn_rate_dps() == 0
    robot.model.configure_friction(enabled=True)
    field.get_friction_at = lambda x, y: -1
    with pytest.raises(ValueError, match='Ground friction'):
        robot.model.step(0.005)
    robot.model.configure_friction(enabled=False)
    robot.model.detach()
    assert robot.model not in clock.steppables


def test_hub_rejects_multiple_existing_bodies(robot):
    DriveBase(Motor(Port.C), Motor(Port.D), 56, 112)
    with pytest.raises(ValueError, match='multiple'):
        PrimeHub()


def test_run_requires_main():
    def init():
        pass

    with pytest.raises(TypeError, match='main'):
        sim.run(init, None)
