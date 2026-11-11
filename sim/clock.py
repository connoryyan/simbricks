from typing import TYPE_CHECKING
from .validation import number

# Avoid circular imports.
if TYPE_CHECKING:
    from typing import Protocol

    class Steppable(Protocol):
        '''
        Describe an object that advances through step(dt_s).
        '''
        def step(self, dt_s: float) -> None: ...


STEP_MS = 5 # dt
STEP_PRIORITIES = (0, 1, 1.5, 2, 3)


class NotSteppableError(Exception):
    '''
    Report an object without a callable step() method.

    Attributes:
        msg (str): Error message supplied to the exception.
        args (tuple): Exception arguments containing the error message.
    '''
    def __init__(self,
                 msg: str = "Object must have step() method") -> None:
        super().__init__(msg)
        self.msg: str = msg


class SimClock:
    '''
    Advance virtual time and update devices in fixed phases.

    Attributes:
        step_ms (float): Default simulation step duration in ms.
        step_s (float): Step duration in seconds, computed at construction.
        time_ms (float): Current simulation time in ms.
        _steppables (dict): Device sets grouped by update phase; order within a set is
            unspecified.
        _initialization (bool | str | None): None before setup, 'initializing' during
            init(), or True after success.
        steppables (tuple): Snapshot of registered devices in phase order.
    '''
    def __init__(self, step_ms: int = STEP_MS) -> None:
        '''
        Create a clock with fixed update phases.

        Arg:
            step_ms (float): Positive default simulation step in ms.
        '''
        number(step_ms, 'step_ms', positive=True)
        self.step_ms: float = step_ms
        self.step_s: float = step_ms / 1000.0
        self.time_ms: float = 0

        # use dict of sets to store distinct steppable objects with different 
        # priorities.
        self._steppables: dict[float, set[Steppable]] = {
            priority: set() for priority in STEP_PRIORITIES
        }
        self._initialization: bool | str | None = None

    @property
    def steppables(self) -> 'tuple[Steppable, ...]':
        '''
        Return a flat snapshot of registered devices in phase order.
        '''
        return tuple(device for priority in STEP_PRIORITIES
                     for device in self._steppables.get(priority, ()))

    def add_steppable(self, steppable) -> None:
        '''
        Register a device in its update phase without duplicates.

        Arg:
            steppable (object): Hashable object with a callable step(dt_s) method.
        '''
        if not callable(getattr(steppable, "step", None)):
            raise NotSteppableError()

        # A priority change takes effect only after removing and adding a device.
        if any(steppable in group for group in self._steppables.values()):
            return
        priority = getattr(steppable, 'step_priority', 0)
        number(priority, 'step_priority')
        if priority not in STEP_PRIORITIES:
            raise ValueError(f'Unsupported step priority: {priority}')
        self._steppables.setdefault(priority, set()).add(steppable)

    def remove_steppable(self, steppable) -> None:
        '''
        Remove a device from all update phases.

        Arg:
            steppable (object): Registered device, or an absent device to ignore.
        '''
        for group in self._steppables.values():
            group.discard(steppable)

    def _tick(self, dt_ms: float) -> None:
        '''
        Update every device once and advance simulated time.

        Registry changes during a tick take effect on the next tick.

        Arg:
            dt_ms (float): Positive step duration in ms.
        '''
        number(dt_ms, 'dt_ms', positive=True)
        dt_s = dt_ms / 1000.0
        # Snapshot every phase before stepping. Additions and removals take
        # effect on the next tick. Devices in one phase must be independent.
        phases = tuple(tuple(self._steppables.get(priority, ()))
                       for priority in STEP_PRIORITIES)
        for group in phases:
            for device in group:
                device.step(dt_s)
        self.time_ms += dt_ms

    def advance_ms(self, duration_ms: float) -> None:
        '''
        Advance simulated time by a duration.

        Use a shorter final step when the duration is not a multiple of step_ms.

        Arg:
            duration_ms (float): Non-negative duration in ms.
        '''
        number(duration_ms, 'duration_ms', minimum=0)
        number(self.step_ms, 'step_ms', positive=True)
        remaining = duration_ms
        while remaining > 0:
            # the last step may be shorter than step_ms
            this_step = min(self.step_ms, remaining)
            self._tick(this_step)
            remaining -= this_step

    def advance_until(self, condition, timeout_ms: float | None = None,
                      poll_step_ms: int | None = None) -> bool:
        '''
        Advance simulated time until a condition becomes true or times out.

        Check the condition before each step. Do not advance past the timeout.

        Arg:
            condition (callable): Function whose truth value ends the wait.
            timeout_ms (float | None): Non-negative timeout in ms, or no limit.
            poll_step_ms (float | None): Positive update interval in ms,
                or the default clock step when None.

        Returns:
            bool: True when the condition succeeds, or False after a timeout.
        '''
        if not callable(condition):
            raise TypeError('condition must be callable')
        if timeout_ms is not None:
            number(timeout_ms, 'timeout_ms', minimum=0)
        step = self.step_ms if poll_step_ms is None else poll_step_ms
        number(step, 'poll_step_ms', positive=True)
        elapsed = 0.0
        while not condition():
            if timeout_ms is not None and elapsed >= timeout_ms:
                return False
            dt_ms = step if timeout_ms is None else min(step, timeout_ms - elapsed)
            self._tick(dt_ms)
            elapsed += dt_ms
        return True


clock = SimClock()
