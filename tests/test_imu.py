from tests.initialization_helpers import initialize_existing
import math
import unittest

from pybricks.hubs import PrimeHub
from pybricks.parameters import Axis, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock


class IMUTests(unittest.TestCase):
    def setUp(self):
        self.previous_models = {priority: group.copy() for priority, group in clock._steppables.items()}
        self.previous_time = clock.time_ms
        clock._steppables.clear()
        clock.time_ms = 0
        self.hub = PrimeHub()
        self.db = DriveBase(Motor(Port.A), Motor(Port.B), 56, 112)
        self.imu = self.hub.imu
        initialize_existing()

    def tearDown(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time

    def test_level_interfaces_and_gravity(self):
        initialize_existing()
        self.assertEqual(self.imu.tilt(), (0, 0))
        initialize_existing()
        self.assertEqual(self.imu.tilt(calibrated=False), (0, 0))
        initialize_existing()
        self.assertEqual(self.imu.pitch(), 0)
        initialize_existing()
        self.assertEqual(self.imu.roll(), 0)
        initialize_existing()
        self.assertEqual(self.imu.acceleration(), (0, 0, 9806.65))
        initialize_existing()
        self.assertEqual(self.imu.acceleration(Axis.X), 0)
        initialize_existing()
        self.assertEqual(self.imu.acceleration(-Axis.Z), -9806.65)

    def test_heading_is_continuous_and_clockwise(self):
        for _ in range(40):
            self.db.model.heading_rad -= math.radians(20)
            initialize_existing()
            wait(5)
        initialize_existing()
        self.assertAlmostEqual(self.imu.heading(), 800)
        initialize_existing()
        self.assertAlmostEqual(self.imu.rotation(Axis.Z), -800)
        initialize_existing()
        self.assertAlmostEqual(self.imu.rotation(-Axis.Z), 800)

    def test_reset_only_changes_heading_offset(self):
        initialize_existing()
        self.db.turn(90)
        initialize_existing()
        wait(500)
        initialize_existing()
        pose, rotation = self.db.pose(), self.imu.rotation(Axis.Z)
        initialize_existing()
        self.imu.reset_heading(42)
        initialize_existing()
        self.assertAlmostEqual(self.imu.heading(), 42)
        self.assertEqual(self.db.pose(), pose)
        initialize_existing()
        self.assertEqual(self.imu.rotation(Axis.Z), rotation)

    def test_placement_does_not_inject_gyro_rotation(self):
        self.db.model.place(100, 200, 90)
        initialize_existing()
        wait(5)
        initialize_existing()
        self.assertEqual(self.imu.heading(), 0)
        initialize_existing()
        self.assertEqual(self.imu.angular_velocity(Axis.Z), 0)

    def test_real_angular_velocity_has_right_hand_sign(self):
        initialize_existing()
        self.db.drive(0, 60)
        initialize_existing()
        wait(1000)
        initialize_existing()
        self.assertLess(self.imu.angular_velocity(Axis.Z), -59)
        initialize_existing()
        self.assertGreater(self.imu.angular_velocity(-Axis.Z), 59)
        initialize_existing()
        self.assertEqual(self.imu.angular_velocity(Axis.X), 0)

    def test_stationary_calibration_removes_bias_without_resetting_yaw(self):
        self.imu.model.bias_dps = 1.0
        initialize_existing()
        self.assertFalse(self.imu.ready())
        initialize_existing()
        wait(500)
        initialize_existing()
        self.assertFalse(self.imu.stationary())
        initialize_existing()
        wait(600)
        initialize_existing()
        self.assertTrue(self.imu.ready())
        initialize_existing()
        self.assertTrue(self.imu.stationary())
        initialize_existing()
        self.assertAlmostEqual(self.imu.angular_velocity(Axis.Z), 0)
        initialize_existing()
        self.assertAlmostEqual(self.imu.angular_velocity(Axis.Z, calibrated=False), 1)
        initialize_existing()
        self.assertAlmostEqual(self.imu.heading(), -1, delta=0.01)
        initialize_existing()
        heading = self.imu.heading()
        initialize_existing()
        wait(2000)
        initialize_existing()
        self.assertAlmostEqual(self.imu.heading(), heading)
        initialize_existing()
        self.assertGreater(self.imu.rotation(Axis.Z, calibrated=False), 3)

    def test_translation_is_not_stationary_and_does_not_learn_bias(self):
        self.imu.model.bias_dps = 1
        initialize_existing()
        self.db.drive(100, 0)
        initialize_existing()
        wait(2000)
        initialize_existing()
        self.assertFalse(self.imu.stationary())
        initialize_existing()
        self.assertFalse(self.imu.ready())
        initialize_existing()
        self.assertAlmostEqual(self.imu.angular_velocity(Axis.Z), 1)

    def test_constant_bias_above_stationary_threshold_can_calibrate(self):
        self.imu.model.bias_dps = 10
        initialize_existing()
        wait(1100)
        initialize_existing()
        self.assertTrue(self.imu.ready())
        initialize_existing()
        self.assertAlmostEqual(self.imu.angular_velocity(Axis.Z), 0)
        initialize_existing()
        self.assertAlmostEqual(self.imu.angular_velocity(Axis.Z, calibrated=False), 10)

    def test_accelerometer_does_not_correct_yaw_drift(self):
        self.imu.model.bias_dps = 1
        initialize_existing()
        self.db.drive(100, 0)
        initialize_existing()
        wait(1000)
        initialize_existing()
        self.assertAlmostEqual(self.imu.heading(), -1)
        initialize_existing()
        self.assertEqual(self.imu.tilt(), (0, 0))

    def test_gyro_drivebase_uses_imu_and_reset_releases_hold(self):
        self.db.use_gyro(True)
        initialize_existing()
        self.db.turn(90)
        initialize_existing()
        wait(500)
        initialize_existing()
        self.assertAlmostEqual(self.imu.heading(), 90, delta=1)
        initialize_existing()
        self.assertAlmostEqual(self.db.angle(), self.imu.heading())
        with self.assertRaises(OSError):
            initialize_existing()
            self.imu.reset_heading(0)
        self.db.reset(distance=10, angle=20)
        initialize_existing()
        self.assertEqual(self.imu.heading(), 20)
        initialize_existing()
        self.assertEqual(self.db.angle(), 20)
        initialize_existing()
        self.assertEqual(self.db.distance(), 10)
        initialize_existing()
        self.imu.reset_heading(0)
        initialize_existing()
        self.assertEqual(self.db.angle(), 0)

    def test_reading_does_not_advance_time_or_resample_noise(self):
        self.imu.model.noise_dps = 0.1
        initialize_existing()
        wait(5)
        initialize_existing()
        first = self.imu.angular_velocity()
        for _ in range(100):
            initialize_existing()
            self.assertEqual(self.imu.angular_velocity(), first)
        self.assertEqual(clock.time_ms, 5)

    def test_axis_and_settings_validation_is_atomic(self):
        for axis in ((0, 0, 0), (1, 2), 'z', (0, 0, float('nan'))):
            with self.assertRaises(ValueError):
                initialize_existing()
                self.imu.angular_velocity(axis)
        before = self.imu.settings()
        with self.assertRaises(ValueError):
            self.imu.settings(angular_velocity_threshold=5, heading_correction=0)
        self.assertEqual(self.imu.settings(), before)
        self.imu.settings(heading_correction=180)
        self.db.model.heading_rad -= math.radians(45)
        initialize_existing()
        wait(5)
        initialize_existing()
        self.assertAlmostEqual(self.imu.heading(), 90)

    def test_detach_removes_imu_and_body_from_clock(self):
        self.db.model.detach()
        self.assertNotIn(self.imu.model, clock.steppables)
        self.assertNotIn(self.db.model, clock.steppables)


if __name__ == '__main__':
    unittest.main()
