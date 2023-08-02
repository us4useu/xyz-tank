from xyztank.view import View
import multiprocessing as mp
from xyztank.controller.actions import *


class Controller:

    def __init__(self, model: XyzSystem, view: View, event_queue: mp.Queue):
        self.log = get_logger(type(self).__name__)
        self.model = model
        self.view = view
        self.event_queue = event_queue
        self._actions = {
            StartMeasurementEvent: StartMeasurementAction(),
            StopMeasurementEvent: StopMeasurementAction(),
            SaveMeasurementEvent: SaveMeasurementAction(),
            ResumeMeasurementEvent: ResumeMeasurementAction()
        }
        self._process = None

    def start(self):
        """
        Starts controller.

        Currently the controller is started in a separte Python interpreter
        process (see package multiprocessing).
        """
        self._process = mp.Process(target=self.main_loop)
        self._process.start()

    def main_loop(self):
        """
        Controller's main loop.
        """
        while True:
            event = self.event_queue.get()
            if isinstance(event, ExitApplicationEvent):
                self.model.exit()
                return
            action = self._actions.get(type(event), None)
            if action is None:
                self.log.error(f"Unknown event type: {type(event)}")
            else:
                ctx = ActionContext(model=self.model, event=event)
                try:
                    action.execute(ctx=ctx)
                except Exception as e:
                    self.log.exception(f"Error while running action: "
                                       f"{type(action)}:")
                    print(e)

    def join(self):
        self._process.join()
