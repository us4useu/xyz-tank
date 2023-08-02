"""A list of events, that can be signaled by view."""

from abc import ABC
from dataclasses import dataclass


class Event(ABC):
    pass


@dataclass(frozen=True)
class StartMeasurementEvent(Event):
    """
    User selected option "Start measurement" (event).

    :param settings: path to the settings file.
    """
    settings: str


@dataclass(frozen=True)
class SaveMeasurementEvent(Event):
    """
    User selected option "Save measurement" (event).

    :param output: path to the output .pkl file
    """
    output: str


@dataclass(frozen=True)
class ResumeMeasurementEvent(Event):
    """
    User selected option "Resume measurement" (event).
    """
    pass


@dataclass(frozen=True)
class StopMeasurementEvent(Event):
    """
    User selected option "Stop measurement" (event).
    """
    pass


@dataclass(frozen=True)
class ExitApplicationEvent:
    """
    User selected option to stop the application.
    """
    pass