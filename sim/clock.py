from typing import TYPE_CHECKING

# Avoid circular imports.
if TYPE_CHECKING:
    from .models.model import Model


STEP_MS = 5 # dt
STEP_PRIORITIES = (0, 1, 1.5, 2, 3)


class NotSteppableError(Exception):
    def __init__(self,
                 msg = "Object must have step() method"):
        super().__init__(msg)
        self.msg = msg


class SimClock:
    def __init__(self, step_ms: int = STEP_MS):
        self.step_ms = step_ms
        self.step_s = step_ms / 1000.0
        self.time_ms = 0
        self._steppables: dict[float, set[Model]] = {
            priority: set() for priority in STEP_PRIORITIES
        }
        self._initialization = None

    @property
    def steppables(self):
        '''
        Return a flat snapshot of registered devices in phase order.
        '''
        return tuple(device for priority in STEP_PRIORITIES
                     for device in self._steppables.get(priority, ()))

    def add_steppable(self, steppable) -> None:
        '''
        Add a device which needs to be updated every frame.

        Arg:
            steppable (Model): An object with step() method.
        '''
        if not callable(getattr(steppable, "step", None)):
            raise NotSteppableError()

        # A priority change takes effect only after removing and adding a device.
        if any(steppable in group for group in self._steppables.values()):
            return
        priority = getattr(steppable, 'step_priority', 0)
        if priority not in STEP_PRIORITIES:
            raise ValueError(f'Unsupported step priority: {priority}')
        self._steppables.setdefault(priority, set()).add(steppable)

    def remove_steppable(self, steppable) -> None:
        for group in self._steppables.values():
            group.discard(steppable)

    def _tick(self, dt_ms: float) -> None:
        '''
        Advance every device by dt_ms.
        '''
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
        remaining = duration_ms
        while remaining > 0:
            # the last step may be shorter than step_ms
            this_step = min(self.step_ms, remaining)
            self._tick(this_step)
            remaining -= this_step

    def advance_until(self, condition, timeout_ms: float | None = None,
                      poll_step_ms: int | None = None) -> bool:
        step = self.step_ms if poll_step_ms is None else poll_step_ms
        if step <= 0:
            raise ValueError("poll_step_ms must be positive")
        elapsed = 0.0
        while not condition():
            self._tick(step)
            elapsed += step
            if timeout_ms is not None and elapsed >= timeout_ms:
                return False
        return True


clock = SimClock()
