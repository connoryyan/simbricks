from tests.initialization_helpers import initialize_existing
import pytest
from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import DCMotor, Motor
from pybricks.tools import wait
from sim.clock import SimClock, clock
from sim.models import MotorModel

class TestMotorMove:

    def setup_method(self):
        self.previous_models = {priority: group.copy() for priority, group in clock._steppables.items()}
        self.previous_time = clock.time_ms
        clock._steppables.clear()
        clock.time_ms = 0

    def teardown_method(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time

    def test_one_model_per_motor(self):
        for motor_type in (DCMotor, Motor):
            clock._steppables.clear()
            motor_type(Port.A)
            assert len(clock.steppables) == 1
            assert isinstance(next(iter(clock.steppables)), MotorModel)

    def test_signed_moves_from_nonzero_angle_in_both_motor_directions(self):
        for direction in Direction:
            for speed, angle in ((300, 90), (-300, 90), (300, -90), (-300, -90)):
                clock._steppables.clear()
                motor = Motor(Port.A, direction)
                initialize_existing()
                motor.run_angle(300, 120)
                initialize_existing()
                wait(2500)
                motor.reset_angle(37)
                initialize_existing()
                start = motor.angle()
                before = clock.time_ms
                initialize_existing()
                motor.run_angle(speed, angle)
                assert clock.time_ms > before
                assert clock.time_ms - before < 2000
                initialize_existing()
                wait(2500)
                expected = start + (90 if speed * angle > 0 else -90)
                initialize_existing()
                assert motor.angle() == pytest.approx(expected, abs=0.1, rel=0)
                initialize_existing()
                assert motor.speed() == pytest.approx(0, abs=0.1, rel=0)

    def test_run_time_defaults_to_hold(self):
        motor = Motor(Port.A)
        initialize_existing()
        motor.run_time(300, 100)
        initialize_existing()
        held_angle = motor.angle()
        initialize_existing()
        wait(2500)
        initialize_existing()
        assert motor.angle() == pytest.approx(held_angle, abs=0.1, rel=0)
        initialize_existing()
        assert motor.speed() == pytest.approx(0, abs=0.1, rel=0)

    def test_zero_angle_does_not_start_or_advance_time(self):
        motor = Motor(Port.A)
        initialize_existing()
        motor.run_angle(0, 0)
        assert clock.time_ms == 0
        initialize_existing()
        assert motor.angle() == 0
        assert motor.model._target_speed_dps == 0

    def test_invalid_arguments_do_not_change_running_motor(self):
        motor = Motor(Port.A)
        initialize_existing()
        motor.run(123)
        initialize_existing()
        invalid_moves = (lambda: motor.run_angle(300, 90, then=None), lambda: motor.run_time(300, 100, then=None), lambda: motor.run_angle(0, 90), lambda: motor.run_angle(float('nan'), 90), lambda: motor.run_angle(300, float('inf')), lambda: motor.run_time(300, -1), lambda: motor.run_time(300, float('nan')))
        for move in invalid_moves:
            with pytest.raises(ValueError):
                move()
            assert motor.model._target_speed_dps == 123
            assert clock.time_ms == 0

    def test_explicit_brake_mode(self):
        motor = Motor(Port.A)
        initialize_existing()
        motor.run_angle(-300, 90, then=Stop.BRAKE)
        initialize_existing()
        assert motor.angle() <= -90
        assert motor.model._braking
        initialize_existing()
        wait(1000)
        initialize_existing()
        assert motor.speed() == pytest.approx(0, abs=0.01, rel=0)

class TestClockPhase:

    def test_groups_step_devices_in_phase_order(self):
        events = []

        class Device:

            def __init__(self, name, priority):
                self.name = name
                self.step_priority = priority

            def step(self, dt_s):
                events.append(self.name)
        sim = SimClock()
        for name, priority in (('recorder', 3), ('imu', 1.5), ('controller', 2), ('body', 1), ('left', 0), ('right', 0)):
            sim.add_steppable(Device(name, priority))
        assert isinstance(sim._steppables, dict)
        assert all((isinstance(group, set) for group in sim._steppables.values()))
        sim.advance_ms(10)
        for offset in (0, 6):
            assert set(events[offset:offset + 2]) == {'left', 'right'}
            assert events[offset + 2:offset + 6] == ['body', 'imu', 'controller', 'recorder']

    def test_registration_changes_apply_on_next_tick(self):
        sim = SimClock()
        events = []

        class Device:
            step_priority = 1

            def __init__(self, name):
                self.name = name

            def step(self, dt_s):
                events.append(self.name)
        old = Device('old')
        new = Device('new')

        class Updater:
            step_priority = 0

            def step(self, dt_s):
                sim.remove_steppable(old)
                sim.add_steppable(new)
        sim.add_steppable(Updater())
        sim.add_steppable(old)
        sim.add_steppable(old)
        sim.advance_ms(5)
        assert events == ['old']
        sim.advance_ms(5)
        assert events == ['old', 'new']

    def test_priority_changes_require_registration_again(self):
        sim = SimClock()

        class Device:
            step_priority = 0

            def step(self, dt_s):
                pass
        device = Device()
        sim.add_steppable(device)
        device.step_priority = 2
        sim.add_steppable(device)
        assert device in sim._steppables[0]
        assert device not in sim._steppables[2]
        sim.remove_steppable(device)
        sim.add_steppable(device)
        assert device not in sim._steppables[0]
        assert device in sim._steppables[2]
