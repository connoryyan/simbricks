import pytest
import sim
import world
from maps.flat_white.world import FlatWhiteWorld
from pybricks.hubs import PrimeHub
from pybricks.parameters import Port
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from sim.clock import clock

class TestInitialization:

    def test_error_has_a_default_message_and_accepts_custom_messages(self):
        message = str(sim.NotInitializedError())
        assert 'Simulation is not initialized' in message
        assert 'sim.initialize(init)' in message
        assert 'sim.run(init, main)' in message
        assert str(sim.NotInitializedError('Custom setup error')) == 'Custom setup error'

    def test_missing_init_uses_default_message(self):
        with pytest.raises(sim.NotInitializedError) as error:
            wait(10)
        assert str(error.value) == str(sim.NotInitializedError())

    def setup_method(self):
        self.old_models = {priority: group.copy() for priority, group in clock._steppables.items()}
        self.old_time = clock.time_ms
        self.old_state = clock._initialization
        self.old_world = world.current_world
        clock._steppables.clear()
        clock.time_ms = 0
        clock._initialization = None
        world.current_world = None

    def teardown_method(self):
        clock._steppables.clear()
        clock._steppables.update(self.old_models)
        clock.time_ms = self.old_time
        clock._initialization = self.old_state
        world.current_world = self.old_world

    def test_missing_init_blocks_operations_without_changing_physics(self):
        motor = Motor(Port.A)
        sensor = ColorSensor(Port.C)
        hub = PrimeHub()
        for operation in (lambda: motor.run(300), lambda: motor.dc(20), sensor.reflection, sensor.ambient, hub.imu.ready, lambda: wait(10), motor.hold):
            with pytest.raises(sim.NotInitializedError):
                operation()
        assert clock.time_ms == 0
        assert motor.model._target_speed_dps == 0

    def test_empty_init_allows_program_without_devices_or_field(self):

        def init():
            pass

        def main():
            wait(10)
            return 'done'
        assert sim.run(init, main) == 'done'
        assert clock.time_ms == 10
        assert world.current_world is None

    def test_motor_only_init_does_not_require_a_field(self):
        motor = Motor(Port.A)

        def init():
            motor.reset_angle(12)
        sim.initialize(init)
        motor.run(100)
        wait(100)
        assert motor.angle() > 12
        assert world.current_world is None

    def test_program_controls_mounts_places_and_field(self):
        left, right = (Motor(Port.A), Motor(Port.B))
        robot = DriveBase(left, right, 56, 112)
        sensor = ColorSensor(Port.C)

        def init():
            world.current_world = FlatWhiteWorld()
            robot.model.mount(sensor.model, forward_mm=50, left_mm=10, above_mm=-18, pitch_deg=90)
            robot.model.place(100, 200, 90)
            return sensor.pose()
        pose = sim.initialize(init)
        assert pose[0] == pytest.approx(90, abs=5e-08, rel=0)
        assert pose[1] == pytest.approx(250, abs=5e-08, rel=0)
        assert pose[2:] == (10, 90, 90, 0)
        assert sensor.reflection() == 90

    def test_missing_or_misnamed_init_raises_custom_error(self):

        def other():
            pass
        for callback in (None, 123, other):
            with pytest.raises(sim.NotInitializedError):
                sim.initialize(callback)
            with pytest.raises(sim.NotInitializedError):
                sim.run(callback, lambda: None)

    def test_failed_init_does_not_run_main_and_disables_operations(self):
        events = []

        def init():
            events.append('init')
            raise ValueError('setup failed')

        def main():
            events.append('main')
        with pytest.raises(ValueError, match='setup failed'):
            sim.run(init, main)
        assert events == ['init']
        with pytest.raises(sim.NotInitializedError):
            wait(10)

    def test_init_signature_and_return_value_belong_to_program(self):

        def init(position, *, name):
            return (position, name)
        assert sim.initialize(init, (1, 2, 3), name='test') == ((1, 2, 3), 'test')

    def test_init_can_calibrate_imu_and_wait(self):
        hub = PrimeHub()

        def init():
            while not hub.imu.ready():
                wait(10)
            hub.imu.reset_heading(0)
        sim.initialize(init)
        assert hub.imu.ready()
        assert hub.imu.heading() == 0

    def test_no_inventory_restrictions_after_init(self):

        def init():
            pass
        sim.initialize(init)
        motor = Motor(Port.A)
        motor.run(100)
        wait(10)
        assert motor.angle() > 0
        world.current_world = FlatWhiteWorld()
        sensor = ColorSensor(Port.C)
        sensor.set_pose(0, 0, 10, 0)
        sensor.set_tilt(90)
        assert sensor.reflection() == 90
