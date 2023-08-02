from abc import ABC, abstractmethod

from xyztank.events import *
from xyztank.model import XyzSystem
from xyztank.logging import get_logger


class ActionContext:
    def __init__(self, model: XyzSystem, event: Event):
        self.model = model
        self.event = event


class Action(ABC):

    def __init__(self):
        self.log = get_logger(type(self).__name__)

    @abstractmethod
    def execute(self, ctx: ActionContext):
        raise ValueError("Abstract method.")


class StartMeasurementAction(Action):
    """
    Starts new measurement.
    """

    def execute(self, ctx: ActionContext):
        event: StartMeasurementEvent = ctx.event
        model: XyzSystem = ctx.model

        model.run_settings(event.settings)


class SaveMeasurementAction(Action):
    """
    Saves current measurement result.
    """

    def execute(self, ctx: ActionContext):
        event: SaveMeasurementEvent = ctx.event
        model: XyzSystem = ctx.model

        model.save_measurement(event.output)


class StopMeasurementAction(Action):
    """
    Stops current measurement.
    """

    def execute(self, ctx: ActionContext):
        model: XyzSystem = ctx.model
        model.stop_measurement()


class ResumeMeasurementAction(Action):
    """
    Resumes previous measurement.
    """

    def execute(self, ctx: ActionContext):
        model: XyzSystem = ctx.model
        model.start_measurement()