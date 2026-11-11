from pathlib import Path
import runpy
import sys

import pytest

import world
from examples.force_contact import force_contact as example
from maps.force_contact.world import ForceContactWorld
from pybricks.parameters import Port
from pybricks.pupdevices import ForceSensor
from sim.clock import clock
from world import BoxObstacle


def test_force_contact_map():
    field = ForceContactWorld()
    assert field.friction_coefficient == 0.04
    assert field.lines[0].points[-1] == (1350, 400)
    assert field.obstacles[0].xmin == 1350
    assert field.obstacles[0].height_mm == 150


def test_button_compression_and_release(initialized):
    sensor = ForceSensor(Port.A)
    wall = BoxObstacle(100, -20, 200, 20, 150)
    contact = example.BumperContact(sensor, wall)
    for x, expected in ((90, 0), (100, 0), (103, 3), (120, 8)):
        sensor.set_pose(x, 0, 40, 0)
        contact.step(0.005)
        assert sensor.force() == expected
        assert sensor.distance() == expected
    sensor.set_pose(103, 30, 40, 0)
    contact.step(0.005)
    assert not sensor.touched()
    sensor.set_pose(103, 0, 160, 0)
    contact.step(0.005)
    assert sensor.force() == 0


def test_contact_arguments():
    sensor = ForceSensor(Port.A)
    wall = BoxObstacle(100, -20, 200, 20, 150)
    with pytest.raises(TypeError):
        example.BumperContact(None, wall)
    with pytest.raises(TypeError):
        example.BumperContact(sensor, None)
    with pytest.raises(ValueError):
        example.BumperContact(sensor, wall).step(0)
    with pytest.raises(TypeError):
        example.run(None)


def test_slippery_robot_stops_on_force_contact(capsys):
    frames = []
    field, robot, sensor = example.run(frames)
    wall = field.obstacles[0]
    assert robot.model.friction_enabled
    assert robot._use_gyro
    assert robot._left.positive_direction == example.Direction.COUNTERCLOCKWISE
    assert robot.model.imu.ready()
    assert robot.model.imu.heading() == pytest.approx(0, abs=1)
    assert sensor.pressed(example.CONTACT_FORCE_N)
    assert sensor.force() <= 8
    assert 0 < sensor.distance() <= 8
    assert robot.pose()[0] + 55 < wall.xmin
    assert wall.ymin < robot.pose()[1] < wall.ymax
    assert abs(robot.model.speed_mm_s()) < 0.01
    assert len(frames) > 100
    assert frames[0][1] == (0, 0, 0)
    assert all(len(frame[2]) == 1 for frame in frames)
    assert not any(isinstance(item, example.BumperContact) for item in clock.steppables)
    assert not any(isinstance(item, example.PoseRecorder) for item in clock.steppables)
    assert 'Contact detected' in capsys.readouterr().out


def test_contact_timeout_stops_and_detaches(monkeypatch):
    monkeypatch.setattr(example, 'CONTACT_TIMEOUT_MS', 0)
    frames = []
    with pytest.raises(TimeoutError, match='No force sensor contact'):
        example.run(frames)
    assert len(frames) > 1
    assert not any(isinstance(item, example.BumperContact) for item in clock.steppables)
    assert not any(isinstance(item, example.PoseRecorder) for item in clock.steppables)
    bodies = [item for item in clock.steppables if isinstance(item, example.DriveBase)]
    assert abs(bodies[0].model.speed_mm_s()) < 0.01


def test_calibration_timeout_still_saves_video(monkeypatch, capsys):
    saved = []

    def save(path, field, frames):
        saved.append((path, field, frames))

    monkeypatch.setattr(example, 'CALIBRATION_TIMEOUT_MS', 0)
    monkeypatch.setattr(example, 'save_video', save)
    example.main()
    assert 'IMU calibration timed out' in capsys.readouterr().err
    assert len(saved) == 1
    assert saved[0][0].name == 'force_contact.mp4'
    assert saved[0][1] is world.current_world
    assert len(saved[0][2]) > 1


def test_example_script_entrypoint(monkeypatch):
    saved = []
    monkeypatch.setattr(sys, 'path', sys.path.copy())

    def save(path, field, frames):
        saved.append((path, frames))

    # Replace the exporter imported by the script, while keeping the real run loop.
    import render.video
    monkeypatch.setattr(render.video, 'save_video', save)
    path = Path(example.__file__)
    runpy.run_path(str(path), run_name='__main__')
    assert len(saved) == 1
    assert len(saved[0][1]) > 100


def test_main_without_recorded_frames(monkeypatch):
    def save(path, field, frames):
        pytest.fail('No frames')

    monkeypatch.setattr(example, 'run', lambda frames: None)
    monkeypatch.setattr(example, 'save_video', save)
    example.main()
