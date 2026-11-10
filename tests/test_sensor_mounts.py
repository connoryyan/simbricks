from tests.initialization_helpers import initialize_existing
from maps.scriptable.world import ScriptableWorld
import math
import unittest

from pybricks.parameters import Port
from pybricks.pupdevices import ColorSensor, Motor, UltrasonicSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock
from sim.models import ColorSensorModel, DriveBaseModel, MotorModel
import world


class SensorMountTests(unittest.TestCase):
    def setUp(self):
        self.previous_models = clock._steppables.copy()
        self.previous_time = clock.time_ms
        self.previous_world = world.current_world
        clock._steppables.clear()
        clock.time_ms = 0
        self.db = DriveBase(Motor(Port.A), Motor(Port.B), 56, 112)

    def tearDown(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time
        world.current_world = self.previous_world

    def assertPose(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=8)

    def test_mount_uses_axle_height_and_only_stores_model(self):
        sensor = ColorSensor(Port.C)
        self.db.model.mount(sensor.model, 50, 10, -18, 15, 90, 5)
        self.assertPose(sensor.pose(), (50, 10, 10, 15, 90, 5))
        self.assertEqual(sensor.pose(), sensor.model.pose())
        self.assertIs(self.db.model._mounts[0].model, sensor.model)
        with self.assertRaises(AttributeError):
            sensor.model = ColorSensorModel()

    def test_place_rotates_offset_and_yaw_but_preserves_tilt(self):
        sensor = ColorSensor(Port.C)
        self.db.model.mount(sensor.model, 50, 10, -18, 15, 90, 5)
        self.db.place(100, 200, 90)
        self.assertPose(sensor.pose(), (90, 250, 10, 105, 90, 5))
        self.db.place(-20, -30, -90)
        self.assertPose(sensor.pose(), (-10, -80, 10, -75, 90, 5))

    def test_motion_updates_multiple_mounts_from_actual_body_pose(self):
        mounts = (
            (ColorSensor(Port.C), 50, 10, -18, 15, 90, 5),
            (UltrasonicSensor(Port.D), 30, -12, 22, -20, 0, -3),
        )
        for sensor, *args in mounts:
            self.db.model.mount(sensor.model, *args)
        initialize_existing()
        self.db.straight(300)
        initialize_existing()
        self.db.turn(90)
        initialize_existing()
        wait(300)
        x, y, yaw = self.db.pose()
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        for sensor, forward, left, above, offset_yaw, pitch, roll in mounts:
            self.assertPose(sensor.pose(), (
                x + forward * c - left * s,
                y + forward * s + left * c,
                28 + above, yaw + offset_yaw, pitch, roll,
            ))

    def test_direct_pose_and_tilt_have_single_source(self):
        sensor = UltrasonicSensor(Port.D)
        sensor.set_tilt(pitch_deg=25, roll_deg=-5)
        sensor.set_pose(x_mm=1, y_mm=2, z_mm=30, yaw_deg=45)
        self.assertEqual(sensor.pose(), (1, 2, 30, 45, 25, -5))
        sensor.model.set_pose(3, 4, 50, -60)
        self.assertEqual(sensor.pose(), (3, 4, 50, -60, 25, -5))

    def test_raw_models_can_mount_without_sensor_wrapper(self):
        body = DriveBaseModel(left_model=MotorModel(), right_model=MotorModel(),
                              wheel_diameter=56, axle_track=112)
        sensor = ColorSensorModel()
        body.mount(sensor, forward_mm=40, above_mm=-18, pitch_deg=90)
        body.place(100, 200, 90)
        self.assertPose(sensor.pose(), (100, 240, 10, 90, 90, 0))

    def test_duplicate_and_invalid_mounts_do_not_change_sensor(self):
        sensor = ColorSensor(Port.C)
        before = sensor.pose()
        for kwargs in ({"above_mm": -29}, {"yaw_deg": float("nan")},
                       {"pitch_deg": float("inf")}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    self.db.model.mount(sensor.model, **kwargs)
                self.assertEqual(sensor.pose(), before)
                self.assertEqual(len(self.db.model._mounts), 0)
        self.db.model.mount(sensor.model)
        with self.assertRaises(ValueError):
            self.db.model.mount(sensor.model)
        with self.assertRaises(TypeError):
            self.db.model.mount(sensor)

    def test_invalid_pose_updates_are_atomic(self):
        sensor = ColorSensor(Port.C)
        sensor.set_pose(1, 2, 10, 30)
        before = sensor.pose()
        with self.assertRaises(ValueError):
            sensor.set_pose(10, 20, -1, 50)
        with self.assertRaises(ValueError):
            sensor.set_tilt(90, float("nan"))
        with self.assertRaises(ValueError):
            self.db.place(0, 0, float("inf"))
        self.assertEqual(sensor.pose(), before)

    def test_existing_sensor_reads_still_work(self):
        field = ScriptableWorld()
        world.current_world = field
        color, ultrasonic = ColorSensor(Port.C), UltrasonicSensor(Port.D)
        self.db.model.mount(color.model, above_mm=-18, pitch_deg=90)
        self.db.model.mount(ultrasonic.model, above_mm=22)
        field.set_reflection(20)
        field.set_distance(350)
        initialize_existing()
        self.assertEqual(color.reflection(), 25)
        initialize_existing()
        self.assertEqual(ultrasonic.distance(), 350)


if __name__ == "__main__":
    unittest.main()
