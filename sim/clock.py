

STEP_MS = 5 # dt


class SimClock:
    def __init__(self, step_ms: int = STEP_MS):
        self.step_ms = step_ms
        self.step_s = step_ms / 1000.0
        self.time_ms = 0
        self._steppables = set() # aggregation

    def add_device(self, steppable) -> None:
        '''
        Add a device which needs to be updated every frame
        '''
        if steppable not in self._steppables:
            self._steppables.append(steppable)

    def remove_device(self, steppable) -> None:
        if steppable in self._steppables:
            self._steppables.remove(steppable)

    def advance_ms(self, duration_ms: float) -> None:
        '''
        Push the time forward.
        '''
        remaining = duration_ms
        while remaining > 0:
            this_step = min(self.step_ms, remaining) 
            # If the remaining time is less than a step, treat it as a step
            for s in self._steppables:
                s.step(this_step / 1000.0)
            self.time_ms += this_step
            remaining -= this_step

    def advance_until(self, condition, timeout_ms: float | None = None, poll_step_ms: int | None = None) -> bool:
        '''
        Advance the simulation until the condition is met.
        '''
        step = poll_step_ms if poll_step_ms else self.step_ms
        # Make sure not none.
        elapsed = 0.0
        while not condition():
            for s in self._steppables:
                s.step(step / 1000.0)
            self.time_ms += step
            elapsed += step
            if timeout_ms is not None and elapsed >= timeout_ms:
                return False
        return True


clock = SimClock()
