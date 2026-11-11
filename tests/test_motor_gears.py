import pytest

from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.pupdevices.actuator import _effective_gear_ratio
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock


def test_gear_trains_and_idlers():
    assert _effective_gear_ratio(None) == 1
    assert _effective_gear_ratio([12, 20, 36]) == 3
    assert _effective_gear_ratio([[12, 20, 36], [20, 40]]) == 6
    assert _effective_gear_ratio((12, 36)) == 3
    assert _effective_gear_ratio(((12, 36), (40, 20))) == 1.5
    assert _effective_gear_ratio([36, 12]) == pytest.approx(1 / 3)
    assert _effective_gear_ratio([12]) == 1


def test_invalid_gears_do_not_register_a_motor():
    for gears in ([], [[]], [[], [12, 36]]):
        with pytest.raises(ValueError, match='empty'):
            Motor(Port.A, gears=gears)
    for gears in ('gears', 3, [True, 36], [12, 3.5, 36], [[12, 36], 20]):
        with pytest.raises(TypeError):
            Motor(Port.A, gears=gears)
    for gears in ([0, 36], [12, 0], [-12, 36], [12, -20, 36]):
        with pytest.raises(ZeroDivisionError, match='positive integers'):
            Motor(Port.A, gears=gears)
    assert clock.steppables == ()


@pytest.mark.parametrize('direction', [Direction.CLOCKWISE, Direction.COUNTERCLOCKWISE])
def test_geared_angle_speed_and_hold(direction, initialized):
    motor = Motor(Port.A, direction, gears=[12, 36])
    sign = direction.value
    motor.run(100)
    assert motor.model._target_speed_dps == sign * 300
    wait(1500)
    assert motor.speed() == pytest.approx(100, abs=0.01)
    assert motor.angle() == pytest.approx(sign * motor.model.angle / 3)
    before = motor.model.angle
    motor.reset_angle(37)
    assert motor.angle() == 37
    assert motor.model.angle == before
    motor.run_angle(100, 90)
    wait(2500)
    assert motor.angle() == pytest.approx(127, abs=0.1)
    assert motor.speed() == pytest.approx(0, abs=0.1)
    assert motor.model._hold_angle_deg == pytest.approx(before + sign * 270)
    motor.run_angle(100, -90)
    wait(2500)
    assert motor.angle() == pytest.approx(37, abs=0.1)
    motor.run_time(100, 200)
    held = motor.angle()
    wait(2500)
    assert motor.angle() == pytest.approx(held, abs=0.1)
    motor.run(100)
    wait(100)
    motor.hold()
    held = motor.angle()
    wait(2500)
    assert motor.angle() == pytest.approx(held, abs=0.1)


@pytest.mark.parametrize('gears, ratio', [([12, 36], 3), ([36, 12], 1 / 3)])
def test_gears_scale_output_limit_but_not_duty(gears, ratio, initialized):
    motor = Motor(Port.A, gears=gears)
    motor.run(10_000)
    wait(2000)
    assert motor.model.speed == pytest.approx(1500, abs=0.01)
    assert motor.speed() == pytest.approx(1500 / ratio, abs=0.01)
    motor.dc(50)
    assert motor.model._target_speed_dps == 750
    wait(2000)
    assert motor.speed() == pytest.approx(750 / ratio, abs=0.01)
    motor.dc(-50)
    wait(2000)
    assert motor.speed() == pytest.approx(-750 / ratio, abs=0.01)


def test_compound_gears_run_angle_and_brake(initialized):
    motor = Motor(Port.A, gears=[[12, 36], [20, 40]])
    assert motor.model.gear_ratio == 6
    motor.run_angle(100, 90, then=Stop.BRAKE)
    assert motor.model.angle >= 540
    assert motor.model._braking


def test_drivebase_with_different_wheel_gearing(initialized):
    left = Motor(Port.B, Direction.COUNTERCLOCKWISE, gears=[12, 36])
    right = Motor(Port.D, gears=[[12, 36], [20, 40]])
    robot = DriveBase(left, right, 56, 112)
    robot.settings(straight_speed=80, turn_rate=60)
    robot.straight(300)
    wait(1000)
    assert robot.distance() == pytest.approx(300, abs=3)
    assert robot.pose() == pytest.approx((300, 0, 0), abs=3)
    assert left.model.angle / -3 == pytest.approx(right.model.angle / 6, abs=0.01)
    robot.turn(90)
    wait(1000)
    assert robot.angle() == pytest.approx(90, abs=2)
    assert robot.pose()[2] == pytest.approx(-90, abs=2)


@pytest.mark.parametrize('friction', [False, True])
def test_geared_drivebase_matches_direct_drive(friction, initialized):
    geared = DriveBase(Motor(Port.A, gears=[12, 36]), Motor(Port.B, gears=[12, 24]), 56, 112)
    direct = DriveBase(Motor(Port.C), Motor(Port.D), 56, 112)
    geared.model.configure_friction(enabled=friction)
    direct.model.configure_friction(enabled=friction)
    geared.drive(100, 30)
    direct.drive(100, 30)
    wait(1000)
    assert geared.pose() == pytest.approx(direct.pose(), abs=1e-8)
    assert geared.state() == pytest.approx(direct.state(), abs=1e-8)
    geared.brake()
    direct.brake()
    wait(1000)
    assert geared.pose() == pytest.approx(direct.pose(), abs=1e-8)


def test_existing_geared_rotation_is_not_counted_as_new_motion(initialized):
    left = Motor(Port.A, gears=[12, 36])
    right = Motor(Port.B, gears=[12, 24])
    left.run_angle(100, 45)
    right.run_angle(100, -90)
    wait(2000)
    left.hold()
    right.hold()
    robot = DriveBase(left, right, 56, 112)
    wait(5)
    assert robot.distance() == pytest.approx(0, abs=0.01)
    assert robot.angle() == pytest.approx(0, abs=0.01)
