from sim.clock import clock


def wait(time_ms: float) -> None:
    clock.advance_ms(time_ms)


class StopWatch:
    def __init__(self):
        self._start_ms = clock.time_ms
        self._paused = False
        self._paused_at_ms = 0
        self._accumulated_pause_ms = 0

    def time(self) -> int:
        if self._paused:
            return self._paused_at_ms - self._start_ms - self._accumulated_pause_ms
        return clock.time_ms - self._start_ms - self._accumulated_pause_ms

    def pause(self) -> None:
        if not self._paused:
            self._paused = True
            self._paused_at_ms = clock.time_ms

    def resume(self) -> None:
        if self._paused:
            self._accumulated_pause_ms += clock.time_ms - self._paused_at_ms
            self._paused = False

    def reset(self) -> None:
        self._start_ms = clock.time_ms
        self._accumulated_pause_ms = 0
        self._paused = False
