from pathlib import Path
import math
import subprocess
import sys

import pytest

import sim
from maps.flat_white.world import FlatWhiteWorld
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase, _Profile
from pybricks.tools import wait
from sim.clock import clock
from sim.models import DriveBaseModel, MotorModel
import world


def _check_hold_keeps_wheel_positions_after_moves(gyro, gears):
    left = Motor(Port.B, Direction.COUNTERCLOCKWISE, gears=gears)
    right = Motor(Port.D, gears=gears)
    robot = DriveBase(left, right, 56, 112)
    robot.use_gyro(gyro)
    robot.straight(100, then=Stop.HOLD)
    assert robot.done()
    assert left.model._hold_angle_deg == left.model.angle
    assert right.model._hold_angle_deg == right.model.angle
    assert not left.model._braking
    assert not right.model._braking
    wait(2500)
    assert robot.distance() == pytest.approx(100, abs=2)
    robot.turn(45, then=Stop.HOLD)
    targets = (left.model._hold_angle_deg, right.model._hold_angle_deg)
    wait(2500)
    assert robot.angle() == pytest.approx(45, abs=1)
    # Displace the shafts to check that HOLD restores their saved targets.
    left.model._angle_deg += 30
    right.model._angle_deg -= 20
    wait(3000)
    assert left.model.angle == pytest.approx(targets[0], abs=0.01)
    assert right.model.angle == pytest.approx(targets[1], abs=0.01)
    assert left.speed() == pytest.approx(0, abs=0.01)
    assert right.speed() == pytest.approx(0, abs=0.01)
    if gyro:
        assert robot.model.imu.in_use
        with pytest.raises(RuntimeError):
            robot.model.imu.reset_heading(0)
    robot.brake()
    assert left.model._hold_angle_deg is None
    assert right.model._hold_angle_deg is None
    if gyro:
        assert not robot.model.imu.in_use


def test_hold_without_gyro(initialized):
    _check_hold_keeps_wheel_positions_after_moves(False, None)


def test_hold_with_gears(initialized):
    _check_hold_keeps_wheel_positions_after_moves(False, [12, 36])


def test_gyro_hold(initialized):
    _check_hold_keeps_wheel_positions_after_moves(True, None)


def test_gyro_hold_with_gears(initialized):
    _check_hold_keeps_wheel_positions_after_moves(True, [12, 36])


def _check_other_stop_modes_release_hold(mode):
    left, right = (Motor(Port.A), Motor(Port.B))
    robot = DriveBase(left, right, 56, 112)
    robot.use_gyro(True)
    robot.straight(10)
    assert robot.model.imu.in_use
    robot.straight(0, then=mode)
    assert left.model._hold_angle_deg is None
    assert right.model._hold_angle_deg is None
    assert left.model._braking == (mode == Stop.BRAKE)
    assert right.model._braking == (mode == Stop.BRAKE)
    assert not robot.model.imu.in_use


def test_brake_releases_hold(initialized):
    _check_other_stop_modes_release_hold(Stop.BRAKE)


def test_coast_releases_hold(initialized):
    _check_other_stop_modes_release_hold(Stop.COAST)


def test_smart_coast_releases_hold(initialized):
    _check_other_stop_modes_release_hold(Stop.COAST_SMART)


def test_new_drive_command_releases_hold(initialized):
    left, right = (Motor(Port.A), Motor(Port.B))
    robot = DriveBase(left, right, 56, 112)
    robot.straight(10)
    assert left.model._hold_angle_deg is not None
    robot.drive(100, 0)
    assert left.model._hold_angle_deg is None
    assert right.model._hold_angle_deg is None
    assert left.model._target_speed_dps > 0
    robot.stop()
    assert left.model._target_speed_dps == 0


def test_hold_cancels_a_running_move(robot):
    robot.straight(300, wait=False)
    wait(100)
    assert not robot.done()
    targets = (robot._left.model.angle, robot._right.model.angle)
    before = clock.time_ms
    robot.hold()
    assert robot.done()
    assert clock.time_ms == before
    assert robot._left.model._hold_angle_deg == targets[0]
    assert robot._right.model._hold_angle_deg == targets[1]
    assert robot.model._imu is None
    wait(3000)
    assert robot._left.model.angle == pytest.approx(targets[0], abs=0.01)
    assert robot._right.model.angle == pytest.approx(targets[1], abs=0.01)


def test_hold_after_drive_keeps_gyro_in_use(robot):
    robot.use_gyro(True)
    robot.drive(100, 10)
    wait(100)
    robot.hold()
    assert robot._left.model._hold_angle_deg is not None
    assert robot._right.model._hold_angle_deg is not None
    assert robot.model.imu.in_use
    with pytest.raises(RuntimeError):
        robot.model.imu.reset_heading(0)
    robot.stop()
    assert not robot.model.imu.in_use
    assert robot._left.model._hold_angle_deg is None
    assert robot._right.model._hold_angle_deg is None
    robot.hold()
    assert robot.model.imu.in_use
    robot.brake()
    assert not robot.model.imu.in_use


def test_hold_requires_initialization():
    left, right = Motor(Port.A), Motor(Port.B)
    robot = DriveBase(left, right, 56, 112)
    with pytest.raises(sim.NotInitializedError):
        robot.hold()
    assert left.model._hold_angle_deg is None
    assert right.model._hold_angle_deg is None
    assert clock.time_ms == 0


def test_model_import_does_not_load_pybricks(initialized):
    subprocess.run(
        [sys.executable, '-B', '-c',
         "from sim.models import DriveBaseModel, MotorModel; import sys; "
         "assert not any(n == 'pybricks' or n.startswith('pybricks.') "
         "for n in sys.modules)"],
        cwd=Path(__file__).resolve().parents[1],
        check=True,)


def test_raw_models_drive_without_device_wrappers(initialized):
    left, right = (MotorModel(), MotorModel())
    body = DriveBaseModel(
        left_model=left,
        right_model=right,
        wheel_diameter=56,
        axle_track=112,
        left_sign=-1,
    )
    left.run(-360)
    right.run(360)
    wait(1000)
    assert body.distance_mm > 100
    assert body.angle_deg == pytest.approx(0, abs=5e-08, rel=0)
    assert body.speed_mm_s() > 0
    assert body.turn_rate_dps() == pytest.approx(0, abs=5e-08, rel=0)


def test_straight_and_turn_for_each_direction_pair(initialized):
    for left_direction in Direction:
        for right_direction in Direction:
            clock._steppables.clear()
            left = Motor(Port.A, left_direction)
            right = Motor(Port.B, right_direction)
            db = DriveBase(left, right, 56, 112)
            assert db.model._left is left.model
            assert db.model._right is right.model
            db.straight(300)
            wait(300)
            x, y, heading = db.pose()
            assert x == pytest.approx(300, abs=5, rel=0)
            assert y == pytest.approx(0, abs=5, rel=0)
            assert heading == pytest.approx(0, abs=2, rel=0)
            db.turn(90)
            wait(300)
            assert db.angle() == pytest.approx(90, abs=2, rel=0)
            assert db.pose()[2] == pytest.approx(-90, abs=2, rel=0)


def test_encoder_reset_does_not_move_body(initialized):
    left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
    right = Motor(Port.B)
    db = DriveBase(left, right, 56, 112)
    db.straight(300)
    db.brake()
    wait(1500)
    pose = db.pose()
    distance = db.distance()
    left.reset_angle(123)
    right.reset_angle(-45)
    assert left.angle() == pytest.approx(123, abs=5e-08, rel=0)
    assert right.angle() == pytest.approx(-45, abs=5e-08, rel=0)
    wait(10)
    assert db.distance() == pytest.approx(distance, abs=1e-06, rel=0)
    for before, after in zip(pose, db.pose()):
        assert before == pytest.approx(after, abs=1e-06, rel=0)
    assert db.state()[1] == pytest.approx(0, abs=1e-06, rel=0)


def test_turn_rate_uses_wheel_directions(initialized):
    left = MotorModel()
    right = MotorModel()
    body = DriveBaseModel(
        left_model=left,
        right_model=right,
        wheel_diameter=56,
        axle_track=112,
        left_sign=-1,
    )
    left.run(-360)
    right.run(-360)
    wait(1000)
    assert body.speed_mm_s() == pytest.approx(0, abs=5e-08, rel=0)
    expected = math.degrees(720 * body.mm_per_deg / 112)
    assert body.turn_rate_dps() == pytest.approx(expected, abs=1, rel=0)
    assert body.angle_deg > 0


def test_invalid_models_and_signs_are_rejected(initialized):
    left, right = (MotorModel(), MotorModel())
    for first, second in ((object(), right), (left, object())):
        with pytest.raises(TypeError):
            DriveBaseModel(left_model=first, right_model=second,
                           wheel_diameter=56, axle_track=112)
    with pytest.raises(ValueError):
        DriveBaseModel(left_model=left, right_model=left,
                       wheel_diameter=56, axle_track=112)
    with pytest.raises(TypeError):
        DriveBaseModel(
            left_model=left,
            right_model=right,
            wheel_diameter=56,
            axle_track=112,
            left_sign=True,
        )
    for sign in (0, 2):
        with pytest.raises(ValueError):
            DriveBaseModel(
                left_model=left,
                right_model=right,
                wheel_diameter=56,
                axle_track=112,
                left_sign=sign,
            )


def test_wheel_direction_types_and_extreme_geometry():
    left, right = (MotorModel(), MotorModel())
    before = clock.steppables
    for sign in (True, '1', 1.0):
        with pytest.raises(TypeError, match='right_sign'):
            DriveBaseModel(
                left_model=left,
                right_model=right,
                wheel_diameter=56,
                axle_track=112,
                right_sign=sign,
            )
    with pytest.raises(ValueError):
        DriveBaseModel(
            left_model=left,
            right_model=right,
            wheel_diameter=56,
            axle_track=112,
            right_sign=2,
        )
    for diameter, track in ((1e-323, 112), (56, 1e+308), (56, 1e-300)):
        with pytest.raises(ValueError):
            DriveBaseModel(left_model=left, right_model=right,
                           wheel_diameter=diameter, axle_track=track)
    assert clock.steppables == before


def test_motion_helpers_validate_before_changing_state(initialized):
    left, right = (Motor(Port.A), Motor(Port.B))
    robot = DriveBase(left, right, 56, 112)
    left.run(100)
    right.run(200)
    for operation in (
        lambda: robot._set_wheels(100, '20'),
        lambda: robot._apply_then(None),
        lambda: left._apply_then(Stop.BRAKE, target_physical='0'),
        lambda: robot._run_maneuver('0.01'),
    ):
        with pytest.raises(TypeError):
            operation()
        assert left.model._target_speed_dps == 100
        assert right.model._target_speed_dps == 200
    with pytest.raises(RuntimeError, match='No active maneuver'):
        robot._run_maneuver(0.01)
    assert robot._maneuver is None


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
    left, right = (MotorModel(), MotorModel())
    for diameter, track, error in (
        ('56', 112, TypeError),
        (56, None, TypeError),
            (0, 112, ValueError)):
        with pytest.raises(error):
            DriveBaseModel(left_model=left, right_model=right,
                           wheel_diameter=diameter, axle_track=track)
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


def test_drivebase_arguments_are_atomic(robot):
    for bad in [None, '10', True, float('nan'), float('inf')]:
        error = ValueError if isinstance(bad, float) else TypeError
        robot.drive(100, 20)
        robot.straight(10, wait=False)
        maneuver = robot._maneuver
        targets = (robot._left.model._target_speed_dps,
                   robot._right.model._target_speed_dps)
        settings = robot.settings()
        for operation in (
            lambda: robot.drive(bad, 0),
            lambda: robot.drive(100, bad),
            lambda: robot.straight(bad),
            lambda: robot.turn(bad),
            lambda: robot.reset(distance=bad),
            lambda: robot.reset(angle=bad),
            lambda: robot.place(0, bad),
            lambda: robot.step(bad),
            lambda: robot.model.step(bad),
            lambda: robot.model.configure_friction(mass_kg=bad),
            lambda: robot.model.configure_friction(contact_stiffness=bad),
        ):
            with pytest.raises(error):
                operation()
            assert robot._maneuver is maneuver
            assert robot.settings() == settings
            assert (robot._left.model._target_speed_dps,
                    robot._right.model._target_speed_dps) == targets
            assert robot.pose() == (0, 0, 0)
            assert not robot.model.friction_enabled
            assert clock.time_ms == 0


def test_robot_settings_flags_and_speed_overflow(robot):
    settings = robot.settings()
    for operation in (
        lambda: robot.use_gyro(1),
        lambda: robot.straight(10, wait=1),
        lambda: robot.turn(10, then=1),
        lambda: robot.model.configure_friction(enabled='yes'),
        lambda: robot.settings(straight_speed=100, turn_rate='90'),
    ):
        with pytest.raises(TypeError):
            operation()
    for operation in (
        lambda: robot.settings(straight_speed=100, turn_acceleration=0),
        lambda: robot.drive(1e+308, 1e+308),
        lambda: _Profile(10, 100, 100).at(-1),
        lambda: _Profile(1e+308, 1e-300, 100),
        lambda: robot.straight(1e+308, wait=False),
    ):
        with pytest.raises(ValueError):
            operation()
    assert robot.settings() == settings
    assert robot._maneuver is None
    assert robot._left.model._target_speed_dps == 0


def test_small_profile_values_remain_finite():
    profile = _Profile(1e-300, 1e-300, 1e-300)
    assert profile.duration > 0
    assert profile.at(profile.duration)[0] == 1e-300
