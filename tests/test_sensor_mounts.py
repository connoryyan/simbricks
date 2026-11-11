import math

import pytest

from maps.scriptable.world import ScriptableWorld
from pybricks.parameters import Port
from pybricks.pupdevices import ColorSensor, UltrasonicSensor
from pybricks.tools import wait
from sim.models import ColorSensorModel, DriveBaseModel, MotorModel
import world


def _assert_pose(actual, expected):
    for a, b in zip(actual, expected):
        assert a == pytest.approx(b, abs=5e-09, rel=0)


def test_mount_uses_axle_height_and_only_stores_model(robot):
    sensor = ColorSensor(Port.C)
    robot.model.mount(sensor.model, 50, 10, -18, 15, 90, 5)
    _assert_pose(sensor.pose(), (50, 10, 10, 15, 90, 5))
    assert sensor.pose() == sensor.model.pose()
    assert robot.model._mounts[sensor.model] == {'forward_mm': 50, 'left_mm': 10, 'z_mm': 10, 'yaw_deg': 15}
    with pytest.raises(AttributeError):
        sensor.model = ColorSensorModel()


def test_place_rotates_offset_and_yaw_but_preserves_tilt(robot):
    sensor = ColorSensor(Port.C)
    robot.model.mount(sensor.model, 50, 10, -18, 15, 90, 5)
    robot.place(100, 200, 90)
    _assert_pose(sensor.pose(), (90, 250, 10, 105, 90, 5))
    robot.place(-20, -30, -90)
    _assert_pose(sensor.pose(), (-10, -80, 10, -75, 90, 5))


def test_motion_updates_multiple_mounts_from_actual_body_pose(robot):
    mounts = (
        (ColorSensor(Port.C), 50, 10, -18, 15, 90, 5),
        (UltrasonicSensor(Port.D), 30, -12, 22, -20, 0, -3),
    )
    for sensor, *args in mounts:
        robot.model.mount(sensor.model, *args)
    robot.straight(300)
    robot.turn(90)
    wait(300)
    x, y, yaw = robot.pose()
    c, s = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
    for sensor, forward, left, above, offset_yaw, pitch, roll in mounts:
        _assert_pose(
            sensor.pose(),
            (
                x + forward * c - left * s,
                y + forward * s + left * c,
                28 + above,
                yaw + offset_yaw,
                pitch,
                roll,
            ),
        )


def test_direct_pose_and_tilt_have_single_source(robot):
    sensor = UltrasonicSensor(Port.D)
    sensor.set_tilt(pitch_deg=25, roll_deg=-5)
    sensor.set_pose(x_mm=1, y_mm=2, z_mm=30, yaw_deg=45)
    assert sensor.pose() == (1, 2, 30, 45, 25, -5)
    sensor.model.set_pose(3, 4, 50, -60)
    assert sensor.pose() == (3, 4, 50, -60, 25, -5)


def test_raw_models_can_mount_without_sensor_wrapper(robot):
    body = DriveBaseModel(
        left_model=MotorModel(),
        right_model=MotorModel(),
        wheel_diameter=56,
        axle_track=112,
    )
    sensor = ColorSensorModel()
    body.mount(sensor, forward_mm=40, above_mm=-18, pitch_deg=90)
    body.place(100, 200, 90)
    _assert_pose(sensor.pose(), (100, 240, 10, 90, 90, 0))


def test_duplicate_and_invalid_mounts_do_not_change_sensor(robot):
    sensor = ColorSensor(Port.C)
    before = sensor.pose()
    for kwargs in ({'above_mm': -29}, {'yaw_deg': float('nan')}, {'pitch_deg': float('inf')}):
        with pytest.raises(ValueError):
            robot.model.mount(sensor.model, **kwargs)
        assert sensor.pose() == before
        assert len(robot.model._mounts) == 0
    robot.model.mount(sensor.model)
    with pytest.raises(ValueError):
        robot.model.mount(sensor.model)
    with pytest.raises(TypeError):
        robot.model.mount(sensor)


def test_invalid_pose_updates_are_atomic(robot):
    sensor = ColorSensor(Port.C)
    sensor.set_pose(1, 2, 10, 30)
    before = sensor.pose()
    with pytest.raises(ValueError):
        sensor.set_pose(10, 20, -1, 50)
    with pytest.raises(ValueError):
        sensor.set_tilt(90, float('nan'))
    with pytest.raises(ValueError):
        robot.place(0, 0, float('inf'))
    assert sensor.pose() == before


def test_existing_sensor_reads_still_work(robot):
    field = ScriptableWorld()
    world.current_world = field
    color, ultrasonic = (ColorSensor(Port.C), UltrasonicSensor(Port.D))
    robot.model.mount(color.model, above_mm=-18, pitch_deg=90)
    robot.model.mount(ultrasonic.model, above_mm=22)
    field.set_reflection(20)
    field.set_distance(350)
    assert color.reflection() == 25
    assert ultrasonic.distance() == 350
