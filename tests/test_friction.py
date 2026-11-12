import math

import pytest

from pybricks.parameters import Port
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait


def _drive_wheels(robot, speed=1000):
    robot._left.run(speed)
    robot._right.run(speed)


def test_ideal_mode_preserves_encoder_based_pose(robot, flat_field):
    _drive_wheels(robot)
    wait(500)
    assert robot.pose()[0] == pytest.approx(robot.distance(), abs=5e-08, rel=0)


def test_mirrored_motor_directions_preserve_friction_trajectory(
    robot,
    flat_field,
):
    baseline_left, baseline_right = (Motor(Port.A), Motor(Port.F))
    baseline = DriveBase(baseline_left, baseline_right, 56, 112)
    flat_field.friction_coefficient = 0.02
    robot.model.configure_friction()
    baseline.model.configure_friction()
    robot._left.run(1000)
    robot._right.run(700)
    baseline_left.run(1000)
    baseline_right.run(700)
    wait(500)
    assert robot._left.model.speed < 0
    assert baseline_left.model.speed > 0
    for actual, expected in zip(robot.pose(), baseline.pose()):
        assert actual == pytest.approx(expected, abs=5e-10, rel=0)
    assert robot.distance() == pytest.approx(
        baseline.distance(), abs=5e-08, rel=0,
    )
    assert robot.angle() == pytest.approx(baseline.angle(), abs=5e-08, rel=0)
    robot.brake()
    baseline.brake()
    wait(500)
    for actual, expected in zip(robot.pose(), baseline.pose()):
        assert actual == pytest.approx(expected, abs=5e-10, rel=0)


def test_zero_friction_spins_wheels_without_moving_body(robot, flat_field):
    flat_field.friction_coefficient = 0
    robot.model.configure_friction()
    _drive_wheels(robot)
    wait(1000)
    assert robot.distance() > 400
    assert robot.pose() == (0, 0, 0)
    assert robot.model.speed_mm_s() == 0
    assert robot.state()[1] > 400


def test_low_friction_limits_acceleration_and_distance(robot, flat_field):
    flat_field.friction_coefficient = 0.01
    robot.model.configure_friction()
    _drive_wheels(robot)
    for _ in range(200):
        wait(5)
        ax, ay = robot.model.acceleration_mm_s2()
        assert math.hypot(ax, ay) <= 0.01 * 9806.65 + 1e-06
    assert robot.pose()[0] > 40
    assert robot.pose()[0] < 55
    assert robot.distance() > 400


def test_high_traction_reduces_slip(robot, flat_field):
    flat_field.friction_coefficient = 1
    robot.model.configure_friction()
    _drive_wheels(robot, 300)
    wait(1000)
    assert robot.pose()[0] == pytest.approx(robot.distance(), abs=5, rel=0)


def test_body_keeps_sliding_after_wheels_brake(robot, flat_field):
    flat_field.friction_coefficient = 0.02
    robot.model.configure_friction()
    _drive_wheels(robot)
    wait(1000)
    robot.brake()
    wait(300)
    start = robot.pose()[0]
    assert abs(robot._left.speed()) < 2
    assert robot.model.speed_mm_s() > 100
    wait(200)
    assert robot.pose()[0] - start > 10
    wait(2000)
    assert robot.model.speed_mm_s() == pytest.approx(0, abs=0.01, rel=0)


def test_frictionless_body_preserves_world_momentum(robot, flat_field):
    robot.model.configure_friction()
    _drive_wheels(robot, 300)
    wait(1000)
    flat_field.friction_coefficient = 0
    before = robot.model.velocity_mm_s()
    robot.brake()
    wait(500)
    assert robot.model.velocity_mm_s() == before


def test_different_wheel_surfaces_produce_real_yaw_and_imu_error(
    robot,
    flat_field,
):
    flat_field.get_friction_at = lambda x, y: 0.01 if y > 0 else 0.8
    robot.model.configure_friction()
    imu = robot.model.imu
    _drive_wheels(robot)
    wait(300)
    assert robot.angle() == pytest.approx(0, abs=5e-08, rel=0)
    assert abs(robot.pose()[2]) > 1
    assert imu.heading() == pytest.approx(-robot.pose()[2], abs=5e-07, rel=0)
    robot.use_gyro(True)
    assert robot.angle() == pytest.approx(imu.heading(), abs=5e-08, rel=0)


def test_friction_circle_also_limits_lateral_force(robot, flat_field):
    flat_field.friction_coefficient = 0.02
    robot.model.configure_friction()
    robot.model._vy_m_s = 0.2
    _drive_wheels(robot)
    wait(5)
    ax, ay = robot.model.acceleration_mm_s2()
    assert ax > 0
    assert ay < 0
    assert math.hypot(ax, ay) <= 0.02 * 9806.65 + 1e-06
    wait(1500)
    assert robot.pose()[1] > 0
    assert abs(robot.model.velocity_mm_s()[1]) < 200
    wait(6000)
    assert abs(robot.model.velocity_mm_s()[1]) < 1


def test_imu_uses_actual_acceleration_and_lateral_motion(robot, flat_field):
    flat_field.friction_coefficient = 0
    robot.model.configure_friction()
    imu = robot.model.imu
    robot.model._vy_m_s = 0.1
    wait(1100)
    assert not imu.stationary()
    assert not imu.ready()
    assert imu._acceleration[:2] == (0, 0)


def test_mounts_follow_true_pose_and_placement_clears_momentum(
    robot,
    flat_field,
):
    sensor = ColorSensor(Port.C)
    robot.model.mount(sensor.model, forward_mm=40, above_mm=-18, pitch_deg=90)
    robot.model.configure_friction()
    flat_field.friction_coefficient = 0
    _drive_wheels(robot)
    wait(100)
    assert sensor.pose() == (40, 0, 10, 0, 90, 0)
    robot.model._vy_m_s = 0.1
    robot.model.place(100, 200, 90)
    assert robot.model.velocity_mm_s() == (0, 0)
    assert sensor.pose()[0] == pytest.approx(100, abs=5e-08, rel=0)
    assert sensor.pose()[1] == pytest.approx(240, abs=5e-08, rel=0)


def test_invalid_parameters_do_not_change_configuration(robot, flat_field):
    for args in ({'mass_kg': 0}, {'yaw_inertia_kg_m2': -1},
                 {'contact_stiffness': float('nan')}):
        with pytest.raises(ValueError):
            robot.model.configure_friction(**args)
        assert not robot.model.friction_enabled
    for mu in (-1, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            flat_field.friction_coefficient = mu
