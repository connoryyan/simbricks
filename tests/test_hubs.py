import unittest

from pybricks.hubs import PrimeHub, InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from sim.clock import clock


class HubTests(unittest.TestCase):
    def setUp(self):
        self.previous_models = clock._steppables.copy()
        self.previous_time = clock.time_ms
        clock._steppables.clear()
        clock.time_ms = 0

    def tearDown(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time

    def robot(self):
        return DriveBase(Motor(Port.B, Direction.COUNTERCLOCKWISE), Motor(Port.D), 56, 112)

    def test_official_initialization_sequence(self):
        hub = PrimeHub()
        robot = self.robot()
        robot.use_gyro(True)
        timer = StopWatch()
        while not hub.imu.ready() and timer.time() < 2000:
            wait(10)
        self.assertTrue(hub.imu.ready())
        hub.imu.reset_heading(0)
        self.assertIs(hub.imu.model, robot.model.imu)
        robot.turn(90)
        wait(500)
        self.assertAlmostEqual(hub.imu.heading(), 90, delta=1)
        self.assertAlmostEqual(robot.angle(), hub.imu.heading())
        with self.assertRaises(OSError):
            hub.imu.reset_heading(0)
        robot.stop()
        hub.imu.reset_heading(0)
        self.assertEqual(robot.angle(), 0)
        self.assertEqual(hub.imu.up(), Side.TOP)

    def test_hub_calibrates_before_drivebase_exists(self):
        hub = PrimeHub()
        wait(1100)
        self.assertTrue(hub.imu.ready())
        model = hub.imu.model
        robot = self.robot()
        self.assertIs(robot.model.imu, model)
        self.assertTrue(hub.imu.ready())

    def test_hub_created_after_drivebase_adopts_existing_imu(self):
        robot = self.robot()
        robot.use_gyro(True)
        robot.turn(45)
        robot.stop()
        wait(3000)
        model = robot.model.imu
        heading = model.heading()
        hub = PrimeHub()
        self.assertIs(hub.imu.model, model)
        self.assertAlmostEqual(hub.imu.heading(), heading)
        self.assertTrue(hub.imu.ready())

    def test_multiple_hub_handles_share_hardware(self):
        first, second = PrimeHub(), InventorHub()
        self.assertIs(first.model, second.model)
        robot = self.robot()
        self.assertIs(first.imu.model, second.imu.model)
        self.assertIs(first.imu.model, robot.model.imu)
        first.model.pressed_buttons.add(Button.LEFT)
        self.assertEqual(second.buttons.pressed(), {Button.LEFT})
        second.buttons.pressed().clear()
        self.assertEqual(first.buttons.pressed(), {Button.LEFT})

    def test_force_sensor_methods_use_scripted_measurements(self):
        sensor = ForceSensor(Port.A)
        self.assertEqual(sensor.force(), 0)
        self.assertEqual(sensor.distance(), 0)
        self.assertFalse(sensor.pressed())
        self.assertFalse(sensor.touched())
        sensor.model.set_measurement(force_n=0, distance_mm=0.2)
        self.assertTrue(sensor.touched())
        self.assertFalse(sensor.pressed())
        sensor.model.set_measurement(force_n=4, distance_mm=5)
        self.assertEqual(sensor.force(), 4)
        self.assertEqual(sensor.distance(), 5)
        self.assertTrue(sensor.pressed())
        self.assertFalse(sensor.pressed(force=5))
        with self.assertRaises(ValueError):
            sensor.model.set_measurement(11, 0)
        self.assertEqual(sensor.force(), 4)

    def test_unsupported_hub_mounting_is_explicit(self):
        with self.assertRaises(NotImplementedError):
            PrimeHub(top_side=Axis.X, front_side=Axis.Z)

    def test_second_active_robot_is_rejected_without_leaking_body(self):
        hub = PrimeHub()
        first = self.robot()
        with self.assertRaises(ValueError):
            self.robot()
        self.assertIs(hub.model.body, first.model)
        self.assertIs(hub.imu.model, first.model.imu)


if __name__ == '__main__':
    unittest.main()
