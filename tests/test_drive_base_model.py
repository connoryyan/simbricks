from tests.initialization_helpers import initialize_existing
import math
import subprocess
import sys
import pytest
from pathlib import Path
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock
from sim.models import DriveBaseModel, MotorModel

class TestDriveBaseModel:

    def setup_method(self):
        self.previous_models = {priority: group.copy() for priority, group in clock._steppables.items()}
        self.previous_time = clock.time_ms
        clock._steppables.clear()
        clock.time_ms = 0

    def teardown_method(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time

    def test_model_import_does_not_load_pybricks(self):
        subprocess.run([sys.executable, '-B', '-c', "from sim.models import DriveBaseModel, MotorModel; import sys; assert not any(n == 'pybricks' or n.startswith('pybricks.') for n in sys.modules)"], cwd=Path(__file__).resolve().parents[1], check=True)

    def test_raw_models_drive_without_device_wrappers(self):
        left, right = (MotorModel(), MotorModel())
        body = DriveBaseModel(left_model=left, right_model=right, wheel_diameter=56, axle_track=112, left_sign=-1)
        initialize_existing()
        left.run(-360)
        initialize_existing()
        right.run(360)
        initialize_existing()
        wait(1000)
        assert body.distance_mm > 100
        assert body.angle_deg == pytest.approx(0, abs=5e-08, rel=0)
        assert body.speed_mm_s() > 0
        assert body.turn_rate_dps() == pytest.approx(0, abs=5e-08, rel=0)

    def test_straight_and_turn_for_each_direction_pair(self):
        for left_direction in Direction:
            for right_direction in Direction:
                clock._steppables.clear()
                left = Motor(Port.A, left_direction)
                right = Motor(Port.B, right_direction)
                db = DriveBase(left, right, 56, 112)
                assert db.model._left is left.model
                assert db.model._right is right.model
                initialize_existing()
                db.straight(300)
                initialize_existing()
                wait(300)
                x, y, heading = db.pose()
                assert x == pytest.approx(300, abs=5, rel=0)
                assert y == pytest.approx(0, abs=5, rel=0)
                assert heading == pytest.approx(0, abs=2, rel=0)
                initialize_existing()
                db.turn(90)
                initialize_existing()
                wait(300)
                initialize_existing()
                assert db.angle() == pytest.approx(90, abs=2, rel=0)
                assert db.pose()[2] == pytest.approx(-90, abs=2, rel=0)

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
        assert left.angle() == pytest.approx(123, abs=5e-08, rel=0)
        initialize_existing()
        assert right.angle() == pytest.approx(-45, abs=5e-08, rel=0)
        initialize_existing()
        wait(10)
        initialize_existing()
        assert db.distance() == pytest.approx(distance, abs=1e-06, rel=0)
        for before, after in zip(pose, db.pose()):
            assert before == pytest.approx(after, abs=1e-06, rel=0)
        initialize_existing()
        assert db.state()[1] == pytest.approx(0, abs=1e-06, rel=0)

    def test_turn_rate_uses_wheel_directions(self):
        left = MotorModel()
        right = MotorModel()
        body = DriveBaseModel(left_model=left, right_model=right, wheel_diameter=56, axle_track=112, left_sign=-1)
        initialize_existing()
        left.run(-360)
        initialize_existing()
        right.run(-360)
        initialize_existing()
        wait(1000)
        assert body.speed_mm_s() == pytest.approx(0, abs=5e-08, rel=0)
        expected = math.degrees(720 * body.mm_per_deg / 112)
        assert body.turn_rate_dps() == pytest.approx(expected, abs=1, rel=0)
        assert body.angle_deg > 0

    def test_invalid_models_and_signs_are_rejected(self):
        left, right = (MotorModel(), MotorModel())
        for first, second in ((object(), right), (left, object())):
            with pytest.raises(TypeError):
                DriveBaseModel(left_model=first, right_model=second, wheel_diameter=56, axle_track=112)
        with pytest.raises(ValueError):
            DriveBaseModel(left_model=left, right_model=left, wheel_diameter=56, axle_track=112)
        for sign in (0, 2, True):
            with pytest.raises(ValueError):
                DriveBaseModel(left_model=left, right_model=right, wheel_diameter=56, axle_track=112, left_sign=sign)
