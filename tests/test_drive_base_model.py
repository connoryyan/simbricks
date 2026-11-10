from tests.initialization_helpers import initialize_existing
import math
import subprocess
import sys
import unittest
from pathlib import Path

from pybricks.parameters import Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock
from sim.models import DriveBaseModel, MotorModel


class DriveBaseModelTests(unittest.TestCase):
    def setUp(self):
        self.previous_models = clock._steppables.copy()
        self.previous_time = clock.time_ms
        clock._steppables.clear()
        clock.time_ms = 0

    def tearDown(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time

    def test_model_import_does_not_load_pybricks(self):
        subprocess.run(
            [sys.executable, "-B", "-c",
             "from sim.models import DriveBaseModel, MotorModel; "
             "import sys; "
             "assert not any(n == 'pybricks' or n.startswith('pybricks.') "
             "for n in sys.modules)"],
            cwd=Path(__file__).resolve().parents[1],
            check=True,
        )

    def test_raw_models_drive_without_device_wrappers(self):
        left, right = MotorModel(), MotorModel()
        body = DriveBaseModel(
            left_model=left, right_model=right,
            wheel_diameter=56, axle_track=112,
            left_sign=-1,
        )
        initialize_existing()
        left.run(-360)
        initialize_existing()
        right.run(360)
        initialize_existing()
        wait(1000)
        self.assertGreater(body.distance_mm, 100)
        self.assertAlmostEqual(body.angle_deg, 0)
        self.assertGreater(body.speed_mm_s(), 0)
        self.assertAlmostEqual(body.turn_rate_dps(), 0)

    def test_straight_and_turn_for_each_direction_pair(self):
        for left_direction in Direction:
            for right_direction in Direction:
                with self.subTest(left=left_direction, right=right_direction):
                    clock._steppables.clear()
                    left = Motor(Port.A, left_direction)
                    right = Motor(Port.B, right_direction)
                    db = DriveBase(left, right, 56, 112)
                    self.assertIs(db.model._left, left.model)
                    self.assertIs(db.model._right, right.model)
                    initialize_existing()
                    db.straight(300)
                    initialize_existing()
                    wait(300)
                    x, y, heading = db.pose()
                    self.assertAlmostEqual(x, 300, delta=5)
                    self.assertAlmostEqual(y, 0, delta=5)
                    self.assertAlmostEqual(heading, 0, delta=2)
                    initialize_existing()
                    db.turn(90)
                    initialize_existing()
                    wait(300)
                    initialize_existing()
                    self.assertAlmostEqual(db.angle(), 90, delta=2)
                    self.assertAlmostEqual(db.pose()[2], -90, delta=2)

    def test_encoder_reset_does_not_move_body(self):
        left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        right = Motor(Port.B)
        db = DriveBase(left, right, 56, 112)
        initialize_existing()
        db.straight(300)
        initialize_existing()
        wait(1500)
        pose = db.pose()
        initialize_existing()
        distance = db.distance()
        left.reset_angle(123)
        right.reset_angle(-45)
        initialize_existing()
        self.assertAlmostEqual(left.angle(), 123)
        initialize_existing()
        self.assertAlmostEqual(right.angle(), -45)
        initialize_existing()
        wait(10)
        initialize_existing()
        self.assertAlmostEqual(db.distance(), distance, delta=1e-6)
        for before, after in zip(pose, db.pose()):
            self.assertAlmostEqual(before, after, delta=1e-6)
        initialize_existing()
        self.assertAlmostEqual(db.state()[1], 0, delta=1e-6)

    def test_turn_rate_uses_wheel_directions(self):
        left = MotorModel()
        right = MotorModel()
        body = DriveBaseModel(
            left_model=left, right_model=right,
            wheel_diameter=56, axle_track=112, left_sign=-1,
        )
        initialize_existing()
        left.run(-360)
        initialize_existing()
        right.run(-360)
        initialize_existing()
        wait(1000)
        self.assertAlmostEqual(body.speed_mm_s(), 0)
        expected = math.degrees(720 * body.mm_per_deg / 112)
        self.assertAlmostEqual(body.turn_rate_dps(), expected, delta=1)
        self.assertGreater(body.angle_deg, 0)

    def test_invalid_models_and_signs_are_rejected(self):
        left, right = MotorModel(), MotorModel()
        for first, second in ((object(), right), (left, object())):
            with self.assertRaises(TypeError):
                DriveBaseModel(left_model=first, right_model=second,
                               wheel_diameter=56, axle_track=112)
        with self.assertRaises(ValueError):
            DriveBaseModel(left_model=left, right_model=left,
                           wheel_diameter=56, axle_track=112)
        for sign in (0, 2, True):
            with self.assertRaises(ValueError):
                DriveBaseModel(left_model=left, right_model=right,
                               wheel_diameter=56, axle_track=112,
                               left_sign=sign)


if __name__ == "__main__":
    unittest.main()
