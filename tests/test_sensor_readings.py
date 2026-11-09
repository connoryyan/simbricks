import math
import unittest

from pybricks.parameters import Port
from pybricks.pupdevices import ColorSensor, UltrasonicSensor
from sim import world
from sim.clock import clock
from sim.models import ColorSensorModel


class SensorReadingTests(unittest.TestCase):
    def setUp(self):
        self.previous_models = clock._steppables.copy()
        self.previous_world = world.current_world
        clock._steppables.clear()
        world.current_world = world.FlatWhiteWorld(ambient_light=0)

    def tearDown(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        world.current_world = self.previous_world

    def color_at(self, height, pitch=90, yaw=0, y=0):
        sensor = ColorSensor(Port.C)
        sensor.set_pose(0, y, height, yaw)
        sensor.set_tilt(pitch)
        return sensor

    def test_height_changes_signal_strength(self):
        self.assertEqual(self.color_at(1e-300).reflection(), 85)
        self.assertEqual(self.color_at(5).reflection(), 85)
        self.assertEqual(self.color_at(10).reflection(), 85)
        self.assertEqual(self.color_at(20).reflection(), 21)
        self.assertEqual(self.color_at(40).reflection(), 5)

    def test_tilt_changes_both_distance_and_incidence(self):
        self.assertEqual(self.color_at(10, 45).reflection(), 30)
        self.assertEqual(self.color_at(10, 30).reflection(), 11)

    def test_sensor_looks_at_ray_hit_instead_of_point_below_it(self):
        world.current_world = world.LineWorld(ambient_light=0)
        sensor = self.color_at(10, 45, -90, y=15)
        self.assertEqual(sensor.reflection(), 4)
        sensor.set_pose(0, 15, 10, 90)
        self.assertEqual(sensor.reflection(), 30)
        sensor.set_tilt(90)
        self.assertEqual(sensor.reflection(), 85)

    def test_unseen_surface_returns_zero(self):
        for height, pitch in ((10, 0), (10, -90), (0, 90), (101, 90)):
            with self.subTest(height=height, pitch=pitch):
                self.assertEqual(self.color_at(height, pitch).reflection(), 0)
        # Vertical height is in range, but the oblique path is too long.
        self.assertEqual(self.color_at(80, 30).reflection(), 0)

    def test_range_boundary_is_inclusive(self):
        self.assertEqual(self.color_at(100).reflection(), 1)
        self.assertEqual(self.color_at(100.001).reflection(), 0)

    def test_roll_does_not_change_centre_ray(self):
        sensor = self.color_at(10, 45, 30)
        direction = sensor.model.view_direction()
        reading = sensor.reflection()
        sensor.set_tilt(45, 123)
        self.assertEqual(sensor.model.view_direction(), direction)
        self.assertEqual(sensor.reflection(), reading)
        self.assertAlmostEqual(sum(v * v for v in direction), 1)

    def test_distance_response_can_be_calibrated(self):
        sensor = ColorSensorModel(reference_distance_mm=20, max_distance_mm=200)
        sensor.set_pose(0, 0, 20, 0)
        sensor.set_tilt(90)
        self.assertEqual(sensor.reflection(), 85)
        for kwargs in ({"reference_distance_mm": 0},
                       {"max_distance_mm": float("nan")},
                       {"reference_distance_mm": 200, "max_distance_mm": 100}):
            with self.subTest(kwargs=kwargs):
                before = len(clock._steppables)
                with self.assertRaises(ValueError):
                    ColorSensorModel(**kwargs)
                self.assertEqual(len(clock._steppables), before)

    def test_noise_is_repeatable_and_readings_stay_bounded(self):
        first = ColorSensorModel(noise=500, seed=42)
        second = ColorSensorModel(noise=500, seed=42)
        for sensor in (first, second):
            sensor.set_pose(0, 0, 10, 0)
            sensor.set_tilt(90)
        a = [first.reflection() for _ in range(30)]
        b = [second.reflection() for _ in range(30)]
        self.assertEqual(a, b)
        self.assertTrue(all(isinstance(v, int) and 0 <= v <= 100 for v in a))

    def test_surface_hit_contains_geometry_and_material(self):
        hit = world.current_world.raycast_surface(
            1, 2, 10, math.sqrt(0.5), 0, -math.sqrt(0.5), 100)
        self.assertAlmostEqual(hit.x_mm, 11)
        self.assertAlmostEqual(hit.y_mm, 2)
        self.assertEqual(hit.z_mm, 0)
        self.assertAlmostEqual(hit.distance_mm, math.sqrt(200))
        self.assertEqual(hit.reflection, 85)

    def test_ultrasonic_uses_height_and_pitch(self):
        sensor = UltrasonicSensor(Port.D)
        sensor.set_pose(0, 0, 100, 0)
        self.assertEqual(sensor.distance(), 2000)
        sensor.set_tilt(90)
        self.assertEqual(sensor.distance(), 100)
        sensor.set_tilt(30)
        self.assertEqual(sensor.distance(), 200)
        sensor.set_pose(0, 0, 50, 0)
        self.assertEqual(sensor.distance(), 100)
        sensor.set_tilt(-90)
        self.assertEqual(sensor.distance(), 2000)

    def test_no_surface_returns_configured_ambient_light(self):
        world.current_world = world.FlatWhiteWorld(ambient_light=17)
        for height, pitch in ((10, 0), (10, -90), (101, 90), (80, 30)):
            with self.subTest(height=height, pitch=pitch):
                sensor = self.color_at(height, pitch)
                self.assertEqual(sensor.reflection(), 17)
                self.assertEqual(sensor.ambient(), 17)

    def test_active_signal_fades_towards_ambient(self):
        world.current_world = world.FlatWhiteWorld(ambient_light=10)
        sensor = self.color_at(10)
        self.assertEqual(sensor.reflection(), 95)
        self.assertEqual(sensor.ambient(), 10)
        sensor.set_pose(0, 0, 20, 0)
        self.assertEqual(sensor.reflection(), 31)
        sensor.set_pose(0, 0, 200, 0)
        self.assertEqual(sensor.reflection(), 10)
        world.current_world.ambient_light = 25
        self.assertEqual(sensor.reflection(), 25)
        world.current_world = world.LineWorld(ambient_light=12)
        self.assertEqual(sensor.ambient(), 12)

    def test_ambient_and_total_signal_are_bounded(self):
        world.current_world = world.FlatWhiteWorld(ambient_light=40)
        sensor = self.color_at(10)
        self.assertEqual(sensor.reflection(), 100)
        self.assertEqual(sensor.ambient(), 40)
        for value in (-1, 101, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                world.current_world.ambient_light = value
        self.assertEqual(sensor.ambient(), 40)

    def test_default_ambient_is_shared_by_world_types(self):
        sensor = self.color_at(200)
        for field in (world.FlatWhiteWorld(), world.LineWorld(), world.ScriptableWorld()):
            world.current_world = field
            self.assertEqual(sensor.ambient(), 5)
            self.assertEqual(sensor.reflection(), 5)


if __name__ == "__main__":
    unittest.main()
