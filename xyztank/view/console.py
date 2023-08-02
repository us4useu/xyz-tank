import multiprocessing as mp

from xyztank.view.base import View
from xyztank.version import __version__

from xyztank.events import *


class ConsoleView(View):
    """
    Console-based implementation of view.
    """

    def __init__(self, event_queue: mp.Queue):
        super().__init__(event_queue)
        print(f"XYZ-tank {__version__} interactive mode")

    def main_loop(self):
        actions = {
            "1": self._start_measurement,
            "2": self._stop_measurement,
            "3": self._save_measurement,
           # "4": self._resume_measurement
        }
        while True:
            print("------------------------------------------")
            print("Menu: ")
            print("1: Start Measurement")
            print("2: Stop Measurement")
            print("3: Save Measurement")
            #print("4: Resume Measurement")
            print("q: Exit")
            print("Select key and press enter.")
            print("------------------------------------------")
            cmd = input()
            cmd = self._sanitize_input(cmd)

            if cmd == "q":
                self.send_event(ExitApplicationEvent())
                return
            else:
                action = actions.get(cmd, None)
                if action is None:
                    print(f"Unknown command: '{cmd}'")
                else:
                    action()

    def _start_measurement(self):
        settings = input("Please provide path to the settings: ")
        settings = self._sanitize_input(settings)
        self.send_event(StartMeasurementEvent(settings=settings))

    def _resume_measurement(self):
        print("Resuming measurement...")
        self.send_event(ResumeMeasurementEvent())

    def _save_measurement(self):
        output = input("Please provide path to the output file: ")
        output = self._sanitize_input(output)
        self.send_event(SaveMeasurementEvent(output=output))

    def _stop_measurement(self):
        actions = {
            "1": self._resume_measurement,
            "2": self._save_measurement,
            "3": self._start_measurement
        }
        print("Stopping measurement...")
        self.send_event(StopMeasurementEvent())
        print("------------------------------------------")
        print("Menu: ")
        print("1: Resume Measurement")
        print("2: Save Measurement")
        print("3: Start Another Measurement")
        print("q: Return to Previous Menu")
        print("Select key and press enter.")
        print("------------------------------------------")
        cmd = input()
        cmd = self._sanitize_input(cmd)

        if cmd == "q":
            return
        else:
            action = actions.get(cmd, None)
            if action is None:
                print(f"Unknown command: '{cmd}'")
            else:
                action()

    def _sanitize_input(self, s: str):
        return s.strip().lower()
