from tests.initialization_helpers import initialize_existing
import pytest
from pybricks.hubs import PrimeHub, InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Axis, Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from sim.clock import clock

class TestHub:

    def setup_method(self):
        self.previous_models = {priority: group.copy() for priority, group in clock._steppables.items()}
        self.previous_time = clock.time_ms
        clock._steppables.clear()
        clock.time_ms = 0

    def teardown_method(self):
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
        initialize_existing()
        while not hub.imu.ready() and timer.time() < 2000:
            initialize_existing()
            wait(10)
        initialize_existing()
        assert hub.imu.ready()
        initialize_existing()
        hub.imu.reset_heading(0)
        assert hub.imu.model is robot.model.imu
        initialize_existing()
        robot.turn(90)
        initialize_existing()
        wait(500)
        initialize_existing()
        assert hub.imu.heading() == pytest.approx(90, abs=1, rel=0)
        initialize_existing()
        assert robot.angle() == pytest.approx(hub.imu.heading(), abs=5e-08, rel=0)
        with pytest.raises(OSError):
            initialize_existing()
            hub.imu.reset_heading(0)
        robot.stop()
        initialize_existing()
        hub.imu.reset_heading(0)
        initialize_existing()
        assert robot.angle() == 0
        initialize_existing()
        assert hub.imu.up() == Side.TOP

    def test_hub_calibrates_before_drivebase_exists(self):
        hub = PrimeHub()
        initialize_existing()
        wait(1100)
        initialize_existing()
        assert hub.imu.ready()
        model = hub.imu.model
        robot = self.robot()
        assert robot.model.imu is model
        initialize_existing()
        assert hub.imu.ready()

    def test_hub_created_after_drivebase_adopts_existing_imu(self):
        robot = self.robot()
        robot.use_gyro(True)
        initialize_existing()
        robot.turn(45)
        robot.stop()
        initialize_existing()
        wait(3000)
        model = robot.model.imu
        initialize_existing()
        heading = model.heading()
        hub = PrimeHub()
        assert hub.imu.model is model
        initialize_existing()
        assert hub.imu.heading() == pytest.approx(heading, abs=5e-08, rel=0)
        initialize_existing()
        assert hub.imu.ready()

    def test_multiple_hub_handles_share_hardware(self):
        first, second = (PrimeHub(), InventorHub())
        assert first.model is second.model
        robot = self.robot()
        assert first.imu.model is second.imu.model
        assert first.imu.model is robot.model.imu
        first.model.pressed_buttons.add(Button.LEFT)
        initialize_existing()
        assert second.buttons.pressed() == {Button.LEFT}
        initialize_existing()
        second.buttons.pressed().clear()
        initialize_existing()
        assert first.buttons.pressed() == {Button.LEFT}

    def test_force_sensor_methods_use_scripted_measurements(self):
        sensor = ForceSensor(Port.A)
        initialize_existing()
        assert sensor.force() == 0
        initialize_existing()
        assert sensor.distance() == 0
        initialize_existing()
        assert not sensor.pressed()
        initialize_existing()
        assert not sensor.touched()
        sensor.model.set_measurement(force_n=0, distance_mm=0.2)
        initialize_existing()
        assert sensor.touched()
        initialize_existing()
        assert not sensor.pressed()
        sensor.model.set_measurement(force_n=4, distance_mm=5)
        initialize_existing()
        assert sensor.force() == 4
        initialize_existing()
        assert sensor.distance() == 5
        initialize_existing()
        assert sensor.pressed()
        initialize_existing()
        assert not sensor.pressed(force=5)
        with pytest.raises(ValueError):
            sensor.model.set_measurement(11, 0)
        initialize_existing()
        assert sensor.force() == 4

    def test_unsupported_hub_mounting_is_explicit(self):
        with pytest.raises(NotImplementedError):
            PrimeHub(top_side=Axis.X, front_side=Axis.Z)

    def test_second_active_robot_is_rejected_without_leaking_body(self):
        hub = PrimeHub()
        first = self.robot()
        with pytest.raises(ValueError):
            self.robot()
        assert hub.model.body is first.model
        assert hub.imu.model is first.model.imu
