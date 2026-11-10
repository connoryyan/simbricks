import pytest

from sim.clock import NotSteppableError, SimClock


class Device:
    def __init__(self, priority=0):
        self.step_priority = priority
        self.steps = []

    def step(self, dt_s):
        self.steps.append(dt_s)


def test_reject_invalid_device_and_priority():
    clock = SimClock()
    for device in (object(), None, type('Invalid', (), {'step': 5})()):
        with pytest.raises(NotSteppableError, match='step'):
            clock.add_steppable(device)
    with pytest.raises(ValueError, match='priority'):
        clock.add_steppable(Device(7))
    assert clock.steppables == ()
    error = NotSteppableError('custom message')
    assert str(error) == error.msg == 'custom message'


def test_partial_step_and_zero_duration():
    clock = SimClock()
    device = Device()
    clock.add_steppable(device)
    clock.advance_ms(12)
    assert device.steps == [0.005, 0.005, 0.002]
    assert clock.time_ms == 12
    clock.advance_ms(0)
    assert len(device.steps) == 3
    clock.remove_steppable(device)
    clock.remove_steppable(device)
    assert clock.steppables == ()


def test_device_without_priority_uses_first_phase():
    class PlainDevice:
        def step(self, dt_s):
            self.dt = dt_s

    clock = SimClock()
    device = PlainDevice()
    clock.add_steppable(device)
    clock.advance_ms(5)
    assert device.dt == 0.005


def test_wait_condition_success_and_timeout():
    clock = SimClock()
    assert clock.advance_until(lambda: True)
    assert clock.time_ms == 0
    assert clock.advance_until(lambda: clock.time_ms >= 10)
    assert clock.time_ms == 10
    assert not clock.advance_until(lambda: False, timeout_ms=6, poll_step_ms=3)
    assert clock.time_ms == 16
    for step in (0, -1):
        with pytest.raises(ValueError, match='positive'):
            clock.advance_until(lambda: True, poll_step_ms=step)
