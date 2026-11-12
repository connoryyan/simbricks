import math

import pytest

from maps.flat_white.world import FlatWhiteWorld
from maps.scriptable.world import ScriptableWorld
from maps.straight_line.world import LineWorld
from pybricks.parameters import Port
from pybricks.pupdevices import ColorSensor, ForceSensor, UltrasonicSensor
from pybricks.pupdevices.sensor import Sensor
from sim.clock import clock
from sim.models import ColorSensorModel, ForceSensorModel, SensorModel
from sim.models.sensor_model import _clamp
import world


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


def test_clamp_and_sensor_noise_check_arguments():
    assert _clamp(4, 0, 3) == 3
    assert _clamp(-1, 0, 3) == 0
    assert _clamp(1, 0, 3) == 1
    with pytest.raises(ValueError, match='low'):
        _clamp(1, 3, 0)
    with pytest.raises(TypeError):
        _clamp('1', 0, 3)
    sensor = SensorModel()
    with pytest.raises(ValueError):
        sensor._noisy(float('inf'))


def test_color_sensor_distance_configuration_is_rechecked():
    sensor = ColorSensorModel()
    sensor.reference_distance_mm = '10'
    with pytest.raises(TypeError):
        sensor.reflection()
    sensor.reference_distance_mm = 10
    sensor.max_distance_mm = -1
    with pytest.raises(ValueError):
        sensor.reflection()
    sensor.max_distance_mm = 5
    with pytest.raises(ValueError, match='Reference'):
        sensor.reflection()


def _color_at(height, pitch=90, yaw=0, y=0):
    sensor = ColorSensor(Port.C)
    sensor.set_pose(0, y, height, yaw)
    sensor.set_tilt(pitch)
    return sensor


def test_height_changes_signal_strength(initialized, white_field):
    assert _color_at(1e-300).reflection() == 85
    assert _color_at(5).reflection() == 85
    assert _color_at(10).reflection() == 85
    assert _color_at(20).reflection() == 21
    assert _color_at(40).reflection() == 5


def test_tilt_changes_both_distance_and_incidence(initialized, white_field):
    assert _color_at(10, 45).reflection() == 30
    assert _color_at(10, 30).reflection() == 11


def test_sensor_looks_at_ray_hit_instead_of_point_below_it(
    initialized,
    white_field,
):
    world.current_world = LineWorld(ambient_light=0)
    sensor = _color_at(10, 45, -90, y=15)
    assert sensor.reflection() == 4
    sensor.set_pose(0, 15, 10, 90)
    assert sensor.reflection() == 30
    sensor.set_tilt(90)
    assert sensor.reflection() == 85


def test_unseen_surface_returns_zero(initialized, white_field):
    for height, pitch in ((10, 0), (10, -90), (0, 90), (101, 90)):
        assert _color_at(height, pitch).reflection() == 0
    assert _color_at(80, 30).reflection() == 0


def test_range_boundary_is_inclusive(initialized, white_field):
    assert _color_at(100).reflection() == 1
    assert _color_at(100.001).reflection() == 0


def test_roll_does_not_change_centre_ray(initialized, white_field):
    sensor = _color_at(10, 45, 30)
    direction = sensor.model.view_direction()
    reading = sensor.reflection()
    sensor.set_tilt(45, 123)
    assert sensor.model.view_direction() == direction
    assert sensor.reflection() == reading
    assert sum((v * v for v in direction)
               ) == pytest.approx(1, abs=5e-08, rel=0)


def test_distance_response_can_be_calibrated(initialized, white_field):
    sensor = ColorSensorModel(reference_distance_mm=20, max_distance_mm=200)
    sensor.set_pose(0, 0, 20, 0)
    sensor.set_tilt(90)
    assert sensor.reflection() == 85
    for kwargs in (
        {'reference_distance_mm': 0},
        {'max_distance_mm': float('nan')},
        {'reference_distance_mm': 200, 'max_distance_mm': 100},
    ):
        before = len(clock.steppables)
        with pytest.raises(ValueError):
            ColorSensorModel(**kwargs)
        assert len(clock.steppables) == before


def test_noise_is_repeatable_and_readings_stay_bounded(
    initialized,
    white_field,
):
    first = ColorSensorModel(noise=500, seed=42)
    second = ColorSensorModel(noise=500, seed=42)
    for sensor in (first, second):
        sensor.set_pose(0, 0, 10, 0)
        sensor.set_tilt(90)
    a = [first.reflection() for _ in range(30)]
    b = [second.reflection() for _ in range(30)]
    assert a == b
    assert all((isinstance(v, int) and 0 <= v <= 100 for v in a))


def test_surface_hit_contains_geometry_and_material(initialized, white_field):
    hit = world.current_world.raycast_surface(
        1, 2, 10, math.sqrt(0.5), 0, -math.sqrt(0.5), 100)
    assert hit.x_mm == pytest.approx(11, abs=5e-08, rel=0)
    assert hit.y_mm == pytest.approx(2, abs=5e-08, rel=0)
    assert hit.z_mm == 0
    assert hit.distance_mm == pytest.approx(math.sqrt(200), abs=5e-08, rel=0)
    assert hit.reflection == 85


def test_ultrasonic_uses_height_and_pitch(initialized, white_field):
    sensor = UltrasonicSensor(Port.D)
    sensor.set_pose(0, 0, 100, 0)
    assert sensor.distance() == 2000
    sensor.set_tilt(90)
    assert sensor.distance() == 100
    sensor.set_tilt(30)
    assert sensor.distance() == 200
    sensor.set_pose(0, 0, 50, 0)
    assert sensor.distance() == 100
    sensor.set_tilt(-90)
    assert sensor.distance() == 2000


def test_no_surface_returns_configured_ambient_light(initialized, white_field):
    world.current_world = FlatWhiteWorld(ambient_light=17)
    for height, pitch in ((10, 0), (10, -90), (101, 90), (80, 30)):
        sensor = _color_at(height, pitch)
        assert sensor.reflection() == 17
        assert sensor.ambient() == 17


def test_active_signal_fades_towards_ambient(initialized, white_field):
    world.current_world = FlatWhiteWorld(ambient_light=10)
    sensor = _color_at(10)
    assert sensor.reflection() == 95
    assert sensor.ambient() == 10
    sensor.set_pose(0, 0, 20, 0)
    assert sensor.reflection() == 31
    sensor.set_pose(0, 0, 200, 0)
    assert sensor.reflection() == 10
    world.current_world.ambient_light = 25
    assert sensor.reflection() == 25
    world.current_world = LineWorld(ambient_light=12)
    assert sensor.ambient() == 12


def test_ambient_and_total_signal_are_bounded(initialized, white_field):
    world.current_world = FlatWhiteWorld(ambient_light=40)
    sensor = _color_at(10)
    assert sensor.reflection() == 100
    assert sensor.ambient() == 40
    for value in (-1, 101, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            world.current_world.ambient_light = value
    assert sensor.ambient() == 40


def test_default_ambient_is_shared_by_world_types(initialized, white_field):
    sensor = _color_at(200)
    for field in (FlatWhiteWorld(), LineWorld(), ScriptableWorld()):
        world.current_world = field
        assert sensor.ambient() == 5
        assert sensor.reflection() == 5


def test_sensor_pose_and_measurement_validation(initialized):
    for bad in [None, '10', True, float('nan'), float('inf')]:
        error = ValueError if isinstance(bad, float) else TypeError
        color = ColorSensor(Port.C)
        force = ForceSensor(Port.D)
        for operation in (
            lambda: color.set_pose(bad, 0, 10, 0),
            lambda: color.set_tilt(90, bad),
            lambda: force.pressed(bad),
            lambda: force.model.set_measurement(force_n=bad),
            lambda: force.model.set_measurement(force_n=3, distance_mm=bad),
        ):
            with pytest.raises(error):
                operation()
            assert color.pose() == (0, 0, 0, 0, 0, 0)
            assert force.force() == force.distance() == 0


def test_force_sensor_methods_use_scripted_measurements(initialized):
    sensor = ForceSensor(Port.A)
    assert sensor.force() == 0
    assert sensor.distance() == 0
    assert not sensor.pressed()
    assert not sensor.touched()
    sensor.model.set_measurement(force_n=0, distance_mm=0.2)
    assert sensor.touched()
    assert not sensor.pressed()
    sensor.model.set_measurement(force_n=4, distance_mm=5)
    assert sensor.force() == 4
    assert sensor.distance() == 5
    assert sensor.pressed()
    assert not sensor.pressed(force=5)
    with pytest.raises(ValueError):
        sensor.model.set_measurement(11, 0)
    assert sensor.force() == 4
