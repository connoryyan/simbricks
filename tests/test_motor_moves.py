import unittest

from pybricks.parameters import Direction, Port, Stop
from pybricks.pupdevices import DCMotor, Motor
from pybricks.tools import wait
from sim.clock import SimClock, clock
from sim.models import MotorModel


class MotorMoveTests(unittest.TestCase):
    def setUp(self):
        self.previous_models = clock._steppables.copy()
        self.previous_time = clock.time_ms
        clock._steppables.clear()
        clock.time_ms = 0

    def tearDown(self):
        clock._steppables.clear()
        clock._steppables.update(self.previous_models)
        clock.time_ms = self.previous_time

    def test_one_model_per_motor(self):
        for motor_type in (DCMotor, Motor):
            with self.subTest(motor_type=motor_type):
                clock._steppables.clear()
                motor_type(Port.A)
                self.assertEqual(len(clock._steppables), 1)
                self.assertIsInstance(next(iter(clock._steppables)), MotorModel)

    def test_signed_moves_from_nonzero_angle_in_both_motor_directions(self):
        for direction in Direction:
            for speed, angle in ((300, 90), (-300, 90),
                                 (300, -90), (-300, -90)):
                with self.subTest(direction=direction, speed=speed, angle=angle):
                    clock._steppables.clear()
                    motor = Motor(Port.A, direction)
                    motor.run_angle(300, 120)
                    wait(2500)
                    motor.reset_angle(37)
                    start = motor.angle()
                    before = clock.time_ms
                    motor.run_angle(speed, angle)
                    self.assertGreater(clock.time_ms, before)
                    self.assertLess(clock.time_ms - before, 2000)
                    wait(2500)
                    expected = start + (90 if speed * angle > 0 else -90)
                    self.assertAlmostEqual(motor.angle(), expected, delta=0.1)
                    self.assertAlmostEqual(motor.speed(), 0, delta=0.1)

    def test_run_time_defaults_to_hold(self):
        motor = Motor(Port.A)
        motor.run_time(300, 100)
        held_angle = motor.angle()
        wait(2500)
        self.assertAlmostEqual(motor.angle(), held_angle, delta=0.1)
        self.assertAlmostEqual(motor.speed(), 0, delta=0.1)

    def test_zero_angle_does_not_start_or_advance_time(self):
        motor = Motor(Port.A)
        motor.run_angle(0, 0)
        self.assertEqual(clock.time_ms, 0)
        self.assertEqual(motor.angle(), 0)
        self.assertEqual(motor.model._target_speed_dps, 0)

    def test_invalid_arguments_do_not_change_running_motor(self):
        motor = Motor(Port.A)
        motor.run(123)
        invalid_moves = (
            lambda: motor.run_angle(300, 90, then=None),
            lambda: motor.run_time(300, 100, then=None),
            lambda: motor.run_angle(0, 90),
            lambda: motor.run_angle(float("nan"), 90),
            lambda: motor.run_angle(300, float("inf")),
            lambda: motor.run_time(300, -1),
            lambda: motor.run_time(300, float("nan")),
        )
        for move in invalid_moves:
            with self.subTest(move=move):
                with self.assertRaises(ValueError):
                    move()
                self.assertEqual(motor.model._target_speed_dps, 123)
                self.assertEqual(clock.time_ms, 0)

    def test_explicit_brake_mode(self):
        motor = Motor(Port.A)
        motor.run_angle(-300, 90, then=Stop.BRAKE)
        self.assertLessEqual(motor.angle(), -90)
        self.assertTrue(motor.model._braking)
        wait(1000)
        self.assertAlmostEqual(motor.speed(), 0, delta=0.01)


class ClockPhaseTests(unittest.TestCase):
    def test_set_steps_devices_in_phase_order(self):
        events = []

        class Device:
            def __init__(self, name, priority):
                self.name = name
                self.step_priority = priority

            def step(self, dt_s):
                events.append(self.name)

        sim = SimClock()
        for name, priority in (("controller", 2), ("body", 1),
                               ("left", 0), ("right", 0)):
            sim.add_steppable(Device(name, priority))
        self.assertIsInstance(sim._steppables, set)
        sim.advance_ms(10)
        for offset in (0, 4):
            self.assertEqual(set(events[offset:offset + 2]), {"left", "right"})
            self.assertEqual(events[offset + 2:offset + 4], ["body", "controller"])


if __name__ == "__main__":
    unittest.main()
