

STEP_MS = 5 # dt


class SimClock:
    def __init__(self, step_ms: int = STEP_MS):
        self.step_ms = step_ms
        self.step_s = step_ms / 1000.0
        self.time_ms = 0
        self._steppables = set() # aggregation

    def add_steppable(self, steppable) -> None:
        '''
        Add a device which needs to be updated every frame
        '''
        if steppable not in self._steppables:
            self._steppables.add(steppable)

    def remove_steppable(self, steppable) -> None:
        if steppable in self._steppables:
            self._steppables.remove(steppable)

    def _tick(self, dt_ms: float) -> None:
        '''
        Advance every device by dt_ms.
        '''
        dt_s = dt_ms / 1000.0
        for s in list(self._steppables):  # copy: devices may be removed during a step
            s.step(dt_s)
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
