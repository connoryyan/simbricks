from sim.initialization import requires_init
from sim.clock import clock


@requires_init
def wait(time_ms: float) -> None:
    '''
    Advance simulated time while updating registered devices.

    Arg:
        time_ms (float): Non-negative duration in ms.
    '''
    clock.advance_ms(time_ms)


class StopWatch:
    '''
    Measure simulation time while excluding paused intervals.

    Attributes:
        _start_ms (float): Simulation time when the stopwatch last started or reset, in
            ms.
        _paused (bool): Whether elapsed stopwatch time is frozen.
        _paused_at_ms (float): Simulation time when the current pause began, in ms.
        _accumulated_pause_ms (float): Total duration of completed pauses, in ms.
    '''
    def __init__(self) -> None:
        '''
        Start a stopwatch using the current simulation time.
        '''
        self._start_ms: float = clock.time_ms
        self._paused: bool = False
        self._paused_at_ms: float = 0
        self._accumulated_pause_ms: float = 0

    def time(self) -> int:
        '''
        Return elapsed simulation time without paused intervals.

        Returns:
            int | float: Elapsed time in ms.
        '''
        if self._paused:
            return self._paused_at_ms - self._start_ms - self._accumulated_pause_ms
        return clock.time_ms - self._start_ms - self._accumulated_pause_ms

    def pause(self) -> None:
        '''
        Pause the stopwatch without advancing simulated time.
        '''
        if not self._paused:
            self._paused = True
            self._paused_at_ms = clock.time_ms

    def resume(self) -> None:
        '''
        Resume the stopwatch without counting the paused interval.
        '''
        if self._paused:
            self._accumulated_pause_ms += clock.time_ms - self._paused_at_ms
            self._paused = False

    def reset(self) -> None:
        '''
        Reset elapsed time to zero and resume the stopwatch.
        '''
        self._start_ms = clock.time_ms
        self._accumulated_pause_ms = 0
        self._paused = False
