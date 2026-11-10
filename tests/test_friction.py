import math
import pytest
import sim
import world
from maps.flat_white.world import FlatWhiteWorld
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock

class TestFriction:

    def setup_method(self):
        self.old_models = {priority: group.copy() for priority, group in clock._steppables.items()}
        self.old_time = clock.time_ms
        self.old_state = clock._initialization
        self.old_world = world.current_world
        clock._steppables.clear()
        clock.time_ms = 0
        self.left = Motor(Port.B, Direction.COUNTERCLOCKWISE)
        self.right = Motor(Port.D)
        self.db = DriveBase(self.left, self.right, 56, 112)
        self.field = FlatWhiteWorld()

        def init():
            world.current_world = self.field
        sim.initialize(init)

    def teardown_method(self):
        clock._steppables.clear()
        clock._steppables.update(self.old_models)
        clock.time_ms = self.old_time
        clock._initialization = self.old_state
        world.current_world = self.old_world

    def drive_wheels(self, speed=1000):
        self.left.run(speed)
        self.right.run(speed)

    def test_ideal_mode_preserves_encoder_based_pose(self):
        self.drive_wheels()
        wait(500)
        assert self.db.pose()[0] == pytest.approx(self.db.distance(), abs=5e-08, rel=0)

    def test_mirrored_motor_directions_preserve_friction_trajectory(self):
        baseline_left, baseline_right = (Motor(Port.A), Motor(Port.F))
        baseline = DriveBase(baseline_left, baseline_right, 56, 112)
        self.field.friction_coefficient = 0.02
        self.db.model.configure_friction()
        baseline.model.configure_friction()
        self.left.run(1000)
        self.right.run(700)
        baseline_left.run(1000)
        baseline_right.run(700)
        wait(500)
        assert self.left.model.speed < 0
        assert baseline_left.model.speed > 0
        for actual, expected in zip(self.db.pose(), baseline.pose()):
            assert actual == pytest.approx(expected, abs=5e-10, rel=0)
        assert self.db.distance() == pytest.approx(baseline.distance(), abs=5e-08, rel=0)
        assert self.db.angle() == pytest.approx(baseline.angle(), abs=5e-08, rel=0)
        self.db.brake()
        baseline.brake()
        wait(500)
        for actual, expected in zip(self.db.pose(), baseline.pose()):
            assert actual == pytest.approx(expected, abs=5e-10, rel=0)

    def test_zero_friction_spins_wheels_without_moving_body(self):
        self.field.friction_coefficient = 0
        self.db.model.configure_friction()
        self.drive_wheels()
        wait(1000)
        assert self.db.distance() > 400
        assert self.db.pose() == (0, 0, 0)
        assert self.db.model.speed_mm_s() == 0
        assert self.db.state()[1] > 400

    def test_low_friction_limits_acceleration_and_distance(self):
        self.field.friction_coefficient = 0.01
        self.db.model.configure_friction()
        self.drive_wheels()
        for _ in range(200):
            wait(5)
            ax, ay = self.db.model.acceleration_mm_s2()
            assert math.hypot(ax, ay) <= 0.01 * 9806.65 + 1e-06
        assert self.db.pose()[0] > 40
        assert self.db.pose()[0] < 55
        assert self.db.distance() > 400

    def test_high_traction_reduces_slip(self):
        self.field.friction_coefficient = 1
        self.db.model.configure_friction()
        self.drive_wheels(300)
        wait(1000)
        assert self.db.pose()[0] == pytest.approx(self.db.distance(), abs=5, rel=0)

    def test_body_keeps_sliding_after_wheels_brake(self):
        self.field.friction_coefficient = 0.02
        self.db.model.configure_friction()
        self.drive_wheels()
        wait(1000)
        self.db.brake()
        wait(300)
        start = self.db.pose()[0]
        assert abs(self.left.speed()) < 2
        assert self.db.model.speed_mm_s() > 100
        wait(200)
        assert self.db.pose()[0] - start > 10
        wait(2000)
        assert self.db.model.speed_mm_s() == pytest.approx(0, abs=0.01, rel=0)

    def test_frictionless_body_preserves_world_momentum(self):
        self.db.model.configure_friction()
        self.drive_wheels(300)
        wait(1000)
        self.field.friction_coefficient = 0
        before = self.db.model.velocity_mm_s()
        self.db.brake()
        wait(500)
        assert self.db.model.velocity_mm_s() == before

    def test_different_wheel_surfaces_produce_real_yaw_and_imu_error(self):
        self.field.get_friction_at = lambda x, y: 0.01 if y > 0 else 0.8
        self.db.model.configure_friction()
        imu = self.db.model.imu
        self.drive_wheels()
        wait(300)
        assert self.db.angle() == pytest.approx(0, abs=5e-08, rel=0)
        assert abs(self.db.pose()[2]) > 1
        assert imu.heading() == pytest.approx(-self.db.pose()[2], abs=5e-07, rel=0)
        self.db.use_gyro(True)
        assert self.db.angle() == pytest.approx(imu.heading(), abs=5e-08, rel=0)

    def test_friction_circle_also_limits_lateral_force(self):
        self.field.friction_coefficient = 0.02
        self.db.model.configure_friction()
        self.db.model._vy_m_s = 0.2
        self.drive_wheels()
        wait(5)
        ax, ay = self.db.model.acceleration_mm_s2()
        assert ax > 0
        assert ay < 0
        assert math.hypot(ax, ay) <= 0.02 * 9806.65 + 1e-06
        wait(1500)
        assert self.db.pose()[1] > 0
        assert abs(self.db.model.velocity_mm_s()[1]) < 200
        wait(6000)
        assert abs(self.db.model.velocity_mm_s()[1]) < 1

    def test_imu_uses_actual_acceleration_and_lateral_motion(self):
        self.field.friction_coefficient = 0
        self.db.model.configure_friction()
        imu = self.db.model.imu
        self.db.model._vy_m_s = 0.1
        wait(1100)
        assert not imu.stationary()
        assert not imu.ready()
        assert imu._acceleration[:2] == (0, 0)

    def test_mounts_follow_true_pose_and_placement_clears_momentum(self):
        sensor = ColorSensor(Port.C)
        self.db.model.mount(sensor.model, forward_mm=40, above_mm=-18, pitch_deg=90)
        self.db.model.configure_friction()
        self.field.friction_coefficient = 0
        self.drive_wheels()
        wait(100)
        assert sensor.pose() == (40, 0, 10, 0, 90, 0)
        self.db.model._vy_m_s = 0.1
        self.db.model.place(100, 200, 90)
        assert self.db.model.velocity_mm_s() == (0, 0)
        assert sensor.pose()[0] == pytest.approx(100, abs=5e-08, rel=0)
        assert sensor.pose()[1] == pytest.approx(240, abs=5e-08, rel=0)

    def test_invalid_parameters_do_not_change_configuration(self):
        for args in ({'mass_kg': 0}, {'yaw_inertia_kg_m2': -1}, {'contact_stiffness': float('nan')}):
            with pytest.raises(ValueError):
                self.db.model.configure_friction(**args)
            assert not self.db.model.friction_enabled
        for mu in (-1, float('nan'), float('inf')):
            with pytest.raises(ValueError):
                self.field.friction_coefficient = mu
