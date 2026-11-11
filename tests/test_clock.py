import pytest

from sim.clock import NotSteppableError, SimClock, clock


class Device:

    '''
    Record updates for clock registry tests.

    Attributes:
        step_priority (float): Clock update phase; smaller phases run first.
        steps (list): Recorded simulation step durations in seconds.
    '''
    def __init__(self, priority=0):
        self.step_priority: float = priority
        self.steps: list[float] = []

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

        '''
        Test clock updates for a device without an explicit priority.

        Attributes:
            dt (float): Most recent step duration in seconds, available after step().
        '''
        def step(self, dt_s):
            self.dt: float = dt_s
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


def test_clock_rejects_a_bad_step_after_configuration():
    sim = SimClock()
    sim.step_ms = 0
    with pytest.raises(ValueError, match='step_ms'):
        sim.advance_ms(10)
    assert sim.time_ms == 0


def test_groups_step_devices_in_phase_order():
    events = []

    class Device:

        '''
        Record named updates for clock phase tests.

        Attributes:
            name (str): Name appended to the clock test event list.
            step_priority (float): Clock update phase; smaller phases run first.
        '''
        def __init__(self, name, priority):
            self.name: str = name
            self.step_priority: float = priority

        def step(self, dt_s):
            events.append(self.name)
    sim = SimClock()
    for name, priority in (
        ('recorder', 3),
        ('imu', 1.5),
        ('controller', 2),
        ('body', 1),
        ('left', 0),
        ('right', 0),
    ):
        sim.add_steppable(Device(name, priority))
    assert isinstance(sim._steppables, dict)
    assert all((isinstance(group, set) for group in sim._steppables.values()))
    sim.advance_ms(10)
    for offset in (0, 6):
        assert set(events[offset:offset + 2]) == {'left', 'right'}
        assert events[offset + 2:offset + 6] == ['body', 'imu', 'controller', 'recorder']


def test_registration_changes_apply_on_next_tick():
    sim = SimClock()
    events = []

    class Device:
        '''
        Record named updates for clock phase tests.

        Attributes:
            name (str): Name appended to the clock test event list.
            step_priority (int): Clock update phase; smaller phases run first.
        '''
        step_priority = 1

        def __init__(self, name):
            self.name: str = name

        def step(self, dt_s):
            events.append(self.name)
    old = Device('old')
    new = Device('new')

    class Updater:
        '''
        Change registered devices during a clock tick.

        Attributes:
            step_priority (int): Clock update phase; smaller phases run first.
        '''
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


def test_priority_changes_require_registration_again():
    sim = SimClock()

    class Device:
        '''
        Record updates for clock registry tests.

        Attributes:
            step_priority (int): Clock update phase; smaller phases run first.
        '''
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


def test_clock_arguments_and_timeout_boundary():
    sim = SimClock()
    for operation in (
        lambda: sim.advance_ms('10'),
        lambda: sim.advance_ms(True),
        lambda: sim.advance_until(None),
        lambda: sim.advance_until(lambda: True, timeout_ms='10'),
        lambda: sim.advance_until(lambda: True, poll_step_ms=True),
    ):
        with pytest.raises(TypeError):
            operation()
    for operation in (
        lambda: sim.advance_ms(-1),
        lambda: sim.advance_ms(float('inf')),
        lambda: sim.advance_until(lambda: True, timeout_ms=-1),
        lambda: sim._tick(0),
    ):
        with pytest.raises(ValueError):
            operation()
    assert sim.time_ms == 0
    assert not sim.advance_until(lambda: False, timeout_ms=0)
    assert sim.time_ms == 0
    assert not sim.advance_until(lambda: False, timeout_ms=7)
    assert sim.time_ms == 7
