import pytest

from maps.flat_white.world import FlatWhiteWorld
from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Port
from pybricks.pupdevices import ColorSensor, Motor
from sim.clock import SimClock, clock
from sim.models import ColorSensorModel, DriveBaseModel, IMUModel, MotorModel, SensorModel
from world import BoxObstacle, Polyline


def test_random_seed_must_be_an_integer():
    for bad in [True, '42', 42.0, b'42']:
        for create in (lambda: SensorModel(seed=bad), lambda: IMUModel(seed=bad)):
            with pytest.raises(TypeError, match='seed'):
                create()
        assert clock.steppables == ()


def test_constructor_numeric_types():
    for bad in [None, '10', True, [], object()]:
        left, right = (MotorModel(), MotorModel())
        constructors = [
            lambda: SimClock(bad),
            lambda: MotorModel(time_constant_s=bad),
            lambda: MotorModel(max_speed_dps=bad),
            lambda: MotorModel(gear_ratio=bad),
            lambda: SensorModel(noise=bad),
            lambda: IMUModel(noise_dps=bad),
            lambda: IMUModel(bias_dps=bad),
            lambda: DriveBaseModel(
                left_model=left,
                right_model=right,
                wheel_diameter=bad,
                axle_track=112,
            ),
            lambda: DriveBaseModel(
                left_model=left,
                right_model=right,
                wheel_diameter=56,
                axle_track=bad,
            ),
            lambda: FlatWhiteWorld(reflection=bad),
            lambda: FlatWhiteWorld(ambient_light=bad),
            lambda: Polyline([(0, 0), (10, 0)], width_mm=bad),
            lambda: BoxObstacle(0, 0, 10, 10, bad),
            lambda: Color(h=bad),
        ]
        before = clock.steppables
        for create in constructors:
            with pytest.raises(TypeError):
                create()
            assert clock.steppables == before


def test_constructor_nonfinite_values():
    for bad in [float('nan'), float('inf'), -float('inf')]:
        for create in (
            lambda: SimClock(bad),
            lambda: MotorModel(time_constant_s=bad),
            lambda: MotorModel(max_speed_dps=bad),
            lambda: MotorModel(gear_ratio=bad),
            lambda: SensorModel(noise=bad),
            lambda: ColorSensorModel(reference_distance_mm=bad),
            lambda: ColorSensorModel(max_distance_mm=bad),
            lambda: Color(s=bad),
        ):
            before = clock.steppables
            with pytest.raises(ValueError):
                create()
            assert clock.steppables == before


def test_constructor_ranges_and_enums():
    for create in (
        lambda: SimClock(0),
        lambda: MotorModel(time_constant_s=-1),
        lambda: MotorModel(max_speed_dps=0),
        lambda: MotorModel(gear_ratio=0),
        lambda: SensorModel(noise=-1),
        lambda: Color(h=361),
        lambda: Color(s=-1),
        lambda: Color(v=101),
        lambda: Polyline([(0, 0), (1e+308, 0)]),
    ):
        with pytest.raises(ValueError):
            create()
    for create in (
        lambda: Motor('A'),
        lambda: ColorSensor(1),
        lambda: Motor(Port.A, positive_direction=1),
        lambda: PrimeHub(top_side='Z'),
        lambda: PrimeHub(front_side=Port.A),
        lambda: IMUModel(body=object()),
        lambda: SensorModel(seed={}),
    ):
        before = clock.steppables
        with pytest.raises(TypeError):
            create()
        assert clock.steppables == before
