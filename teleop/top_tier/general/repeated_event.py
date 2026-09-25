import threading
from typing import Callable


class RepeatedEvent:
    """
    Schedule a function to trigger repeatedly, every x seconds

    Note that the time the function takes to run is included in the gap. i.e. this class will trigger the event every
    x seconds regardless of how long it took the function to run. This means that, if the function call is long enough
    or the time interval is short enough, multiple instances of the function could be running at once.
    """

    def __init__(self, time_interval: float, function: Callable, *args, **kwargs):
        """
        Create a new repeated event

        Args:
            time_interval: how long the event should wait before triggering again
            function: the function to call when triggered
            args: the inputs to the function
            kwargs: the inputs to the function
        """
        self.time_interval = time_interval
        self._function = function
        self._args = args
        self._kwargs = kwargs
        self._is_running = False
        self._timers: list[threading.Timer] = []

    @property
    def is_running(self) -> bool:
        """
        Returns:
            True is this event is currently running, False otherwise
        """
        return self._is_running

    def _run(self, current_timer: threading.Timer):
        """
        The function that is scheduled to run

        Calls the user-defined function and does some administrative tasks

        Args:
            current_timer: the Timer running this function call
        """
        if self.is_running:
            self._force_start()
        self._function(*self._args, **self._kwargs)
        self._timers.remove(current_timer)

    def _force_start(self) -> None:
        """
        Add another event to the queue, ignoring whether one is there already
        """
        args = []
        timer = threading.Timer(self.time_interval, self._run, args=args)
        args.append(timer)
        self._timers.append(timer)
        timer.start()

    def start(self) -> None:
        """
        Starts this event running, if it is not already
        """
        if not self.is_running:
            self._is_running = True
            self._force_start()

    def stop(self) -> None:
        """
        Stops this event from running, including any future calls

        Any calls that have already started will continue to run
        """
        self._is_running = False
        for timer in self._timers:
            timer.cancel()
