import threading
from enum import Enum, auto
from typing import Callable


class RepeatMode(Enum):
    """
    How the time interval of a RepeatedEvent is measured
    """

    """
    The interval is the time between one call starting and the next starting. Calls are triggered on schedule
    regardless of whether the previous call has finished, so calls may overlap.
    """
    START_TO_START = auto()

    """
    The interval is the time between one call ending and the next starting. Calls never overlap.
    """
    END_TO_START = auto()

    """
    Like START_TO_START, except a call will not begin until the previous call has ended. i.e. the next call starts
    once BOTH the interval has passed since the previous call started AND the previous call has finished.
    Calls never overlap.
    """
    START_TO_START_NO_OVERLAP = auto()


class RepeatedEvent:
    """
    Schedule a function to trigger repeatedly. How the interval is interpreted depends on the RepeatMode:

    - START_TO_START: trigger every x seconds, regardless of how long the function takes. If the function is slow
      enough (or the interval short enough), multiple instances could be running at once.
    - END_TO_START: wait x seconds after the function finishes before triggering it again.
    - START_TO_START_NO_OVERLAP: trigger every x seconds, but if the previous call is still running, wait for it
      to finish first.
    """

    def __init__(self, time_interval: float, mode: RepeatMode, function: Callable, *args, **kwargs):
        """
        Create a new repeated event

        Args:
            time_interval: how long the event should wait before triggering again (see RepeatMode)
            mode: how the time interval is measured
            function: the function to call when triggered
            args: the inputs to the function
            kwargs: the inputs to the function
        """
        self.time_interval = time_interval
        self._mode = mode
        self._function = function
        self._args = args
        self._kwargs = kwargs
        self._is_running = False

        # Guards _is_running, _generation and _timers
        self._state_lock = threading.Lock()
        # Held while the function runs (only used by START_TO_START_NO_OVERLAP)
        self._run_lock = threading.Lock()
        # Incremented on every start/stop, so stale calls from a previous run can tell they should not reschedule
        self._current_generation = 0
        self._timers: set[threading.Timer] = set()

    @property
    def is_running(self) -> bool:
        """
        Returns:
            True is this event is currently running, False otherwise
        """
        return self._is_running

    def _schedule(self, generation: int) -> None:
        """
        Schedule the next call. Must be called with self._state_lock held

        Args:
            generation: the generation this call belongs to
        """
        timer = threading.Timer(self.time_interval, self._run, args=(generation,))
        self._timers.add(timer)
        timer.start()

    def _run(self, generation: int) -> None:
        """
        The function that is scheduled to run

        Calls the user-defined function and does some administrative tasks

        Args:
            generation: the generation this call belongs to; if it is out of date, the event was stopped
        """
        timer = threading.current_thread()
        no_overlap = self._mode is RepeatMode.START_TO_START_NO_OVERLAP
        schedule_at_start = self._mode in (RepeatMode.START_TO_START, RepeatMode.START_TO_START_NO_OVERLAP)

        if no_overlap:
            # Wait for the previous call (if any) to finish
            self._run_lock.acquire()
        try:
            with self._state_lock:
                self._timers.discard(timer)
                if generation != self._current_generation:
                    return  # stopped (possibly while we were waiting)
                if schedule_at_start:
                    # Schedule the next call now, measured from the start of this one
                    self._schedule(generation)

            self._function(*self._args, **self._kwargs)
        finally:
            if no_overlap:
                self._run_lock.release()
            if not schedule_at_start:
                # Schedule the next call now, measured from the end of this one
                with self._state_lock:
                    if generation == self._current_generation:
                        self._schedule(generation)

    def start(self) -> None:
        """
        Starts this event running, if it is not already
        """
        with self._state_lock:
            if not self._is_running:
                self._is_running = True
                self._current_generation += 1
                self._schedule(self._current_generation)

    def stop(self) -> None:
        """
        Stops this event from running, including any future calls

        Any calls that have already started will continue to run
        """
        with self._state_lock:
            self._is_running = False
            self._current_generation += 1
            for timer in self._timers:
                timer.cancel()
            self._timers.clear()