import logging
import time
import importlib
import sys
import math
import threading
import numpy as np
from enum import Enum
from dataclasses import dataclass
import pickle
from typing import Tuple

from xyztank.model import *
from xyztank.logging import get_logger


@dataclass(frozen=True)
class Tank:
    """
    Tank (aquarium).

    :param name: name of the tank
    :param dimensions: dimensions of the tank, width (OX), depth (OY),
      height (OZ)
    """
    name: str
    dimensions: Tuple[float, float, float]


class ScanRoute:
    """
    A route and method to scan grid from executed measurement plan.

    :param indexes_x: an array which holds x-indexes of points that will be
    scanned (in order)
    :param indexes_y: an array which holds y-indexes of points that will be
    scanned (in order)
    :param indexes_z: an array which holds z-indexes of points that will be
    scanned (in order)
    :param which_motor_and_side: tells which xyz motor will be moved and to
     which side while system is moving to next point
     {0: motor_x to left,
      1: motor_x to right,
      10: motor_y to left
      11: motor_y to right,
      21: motor_x to right}
      right - moving away from local (0.0.0)
      left - moving closer to local (0.0.0)
    """
    def __init__(self, indexes_x, indexes_y, indexes_z, which_motor_and_side):
        self.indexes_x = indexes_x
        self.indexes_y = indexes_y
        self.indexes_z = indexes_z
        self.which_motor_and_side = which_motor_and_side


@dataclass(frozen=True)
class MeasurementPlan:
    """
    A plan of measurement to execute in the system.

    :param name: name of the measurement
    :param tank: the tank in which the measurement was made
    :param position: distance xyz from global point (0.0.0)
    :param grid_precision: distance between points in grid in all axis
    :param scan_route: enables scanning in the previously planned order
    """
    name: str
    tank: Tank
    position: Tuple[float, float, float]
    grid: Tuple[np.ndarray, np.ndarray, np.ndarray]
    grid_precision: Tuple[float, float, float]


class XyzSystemState(Enum):
    """
    State of the XYZ system.

    - AT_THE_BEGINNING: XYZ is at the beginning point (0.0.0).
    - STOPPED: XYZ is not performing any measurement right now.
    - RUNNING: XYZ is currently running some measurement.
    - FINISHED: XYZ finished performing its previous measurement.
    """
    AT_THE_BEGINNING = 0
    STOPPED = 1
    RUNNING = 2
    FINISHED = 3


@dataclass(frozen=True)
class MeasurementProgress:
    """
    Progress of measurement.

    :param data: currently acquired portion of data
    :param percent: percentage of plan execution
    :param last_measurement_point: number of last point in which was conducted
    measurement
    """
    data: np.ndarray
    percent: int
    last_measurement_point: int


@dataclass(frozen=True)
class MeasurementResult:
    """
    Measurement result.

    :param plan: executed plan
    :param date: date of measurement (epoch timestamp)
    :param data: measurement data
    """
    plan: MeasurementPlan
    date: int
    data: np.ndarray


class Motor:
    """
       Motor that moves in x,y or z.

       :param position: position in one axis from global point 000 in which is
       currently thing that motor moves
       """
    def __init__(self, position):
        self.position = position

    def _rotate_left(self, distance): #check_
        self.position = self.position - distance
        #move_left(distance)

    def _rotate_right(self, distance): #check_
        self.position = self.position + distance
        #move_right(distance)


class XyzSystem:

    def __init__(self):
        self.log = get_logger(type(self).__name__)
        self.measurement_plan = None  # Current measurement plan
        self.measurement_progress = None
        self.measurement_thread = None
        self.state = XyzSystemState.STOPPED
        self.motor_x = Motor(position=5e-3)  # it has to be changed after
        self.motor_y = Motor(position=6e-3)  # discussion about positioning
        self.motor_z = Motor(position=7e-3)
        self.scan_route = None

    def run_settings(self, settings_path: str):
        """
        Load settings from the path, configure the system and run measurement.

        :param settings_path: path to the settings file
        """
        settings = self._load_settings(settings_path)
        self.run_measurement(settings.plan)

    def run_measurement(self, plan: MeasurementPlan):
        """
        Configure the system and run measurement.

        :param plan: measurement plan to run
        """
        self.configure_measurement(plan)
        self._move_at_the_beginning()
        return self.start_measurement()

    def configure_measurement(self, plan: MeasurementPlan):
        """
        Configure measurement plan for this system.

        :param plan: measurement plan to configure
        """
        self.measurement_progress = None
        self.log.info(f"Configuring measurement")
        if self.state == XyzSystemState.RUNNING:
            raise ValueError("The system is busy.")
        self.measurement_plan = plan
        self._make_scan_route()

    def start_measurement(self):
        """
        Start or resume the currently configured measurement.
        """
        self.log.info(f"Starting measurement")
        self._set_to_running()
        self.measurement_thread = threading.Thread(target=self._acquire_data)
        self.measurement_thread.start()

    def resume_measurement(self):
        """
                Resume the previously configured measurement.
        """
        pass

    def stop_measurement(self):
        """
        Stop measurement currently in progress.
        """
        if self.state != XyzSystemState.RUNNING:
            self.log.warn("There is no measurement currently running.")
        else:
            self.log.info(f"Stopping the current measurement")
            self._set_state_to_stopped()

    def save_measurement(self, path):
        """
        Saves the last measurement to given output file.

        :param path: path to the output file
        """
        progress = self.measurement_progress
        if progress is None:
            self.log.error("So far no measurement has been performed, "
                           "run some measurement plan first.")
        elif progress.percent != 100:
            self.log.warn("Note: attempting to save partially acquired data,"
                          f"current progress: {progress.percent}")
        plan = self.measurement_plan
        result = MeasurementResult(
            plan=plan,
            date=time.time_ns() // 1000,
            data=progress.data
        )
        pickle.dump(result, open(path, "wb"))
        self.log.info(f"Saved measurement to {path}")

    def get_progress(self) -> MeasurementProgress:
        """
        Returns current progress in executing the measurement plan.

        :return: current measurement progress
        """
        return self.measurement_progress

    def get_plan(self) -> MeasurementPlan:
        """
        Returns currently executed measurement plan.

        :return: current measurement plan
        """
        return self.measurement_plan

    def get_scan_route(self) -> ScanRoute:
        """
        Returns scan route based on current measurement plan.

        :return: scan route
        """
        return self.scan_route

    def get_motor_x(self) -> Motor:
        """
        Returns motor working in x-axis.

        :return: motor in x-axis
        """
        return self.motor_x

    def get_motor_y(self) -> Motor:
        """
        Returns motor working in y-axis.

        :return: motor in y-axis
        """
        return self.motor_y

    def get_motor_z(self) -> Motor:
        """
        Returns motor working in z-axis.

        :return: motor in z-axis
        """
        return self.motor_z

    def get_motors(self) -> Tuple[Motor, Motor, Motor]:
        """
               Returns all working motors.

               :return: motor in x-axis, motor in y-axis, motor in z-axis
               """
        return self.motor_x, self.motor_y, self.motor_z

    def exit(self):
        """
        Stop any measurement that is currently running in the system
        then close the system.
        """
        if self.state == XyzSystemState.RUNNING:
            self.stop_measurement()
        self.log.info("Closed the handle to the system.")

    def _acquire_data(self):
        grid_x, grid_y, grid_z = self.measurement_plan.grid
        if self.measurement_progress is None:
            # when measurement is started for the first time
            # the last axis == 2: pressure +/- values
            result_shape = (len(grid_z), len(grid_y), len(grid_x), 2)
            result = np.zeros(result_shape, dtype=np.float32)
            start = 0
            n_percent_count = 0
        else:
            # when measurement is resumed
            current_measurement_progress = self.get_progress()
            result = current_measurement_progress.data
            percent = current_measurement_progress.percent
            last_measurement_point = current_measurement_progress.last_measurement_point
            start = last_measurement_point + 1
            n_percent_count = int(math.floor(percent/10))

        n_values = len(grid_x) * len(grid_y) * len(grid_z)
        scan_route = self.get_scan_route()
        indexes_z = scan_route.indexes_z
        indexes_y = scan_route.indexes_y
        indexes_x = scan_route.indexes_x
        which_motor_and_side = scan_route.which_motor_and_side
        actions = {
            0: self._move_motor_x_left,
            1: self._move_motor_x_right,
            10: self._move_motor_y_left,
            11: self._move_motor_y_right,
            21: self._move_motor_z_right
        }
        n_percent_values = []
        for j in range(9):
            n_percent_values.append(int(math.ceil(j + 1) * n_values / 10))

        for i in range(start, int(n_values)):
            percent = int(math.ceil((i/n_values) * 100))
            result[indexes_z[i]][indexes_y[i]][indexes_x[i]][0] = 100 * indexes_z[i] + 10 * indexes_y[i] + indexes_x[i]
            result[indexes_z[i]][indexes_y[i]][indexes_x[i]][1] = 0
            self.measurement_progress = MeasurementProgress(
                data=result, percent=percent, last_measurement_point=i)
            if i == n_percent_values[n_percent_count]:
                if n_percent_count != len(n_percent_values) - 1:
                    n_percent_count += 1
                self.log.info(f"Measurement in progress: "
                              f"{self.measurement_progress.percent}%")
                print(f"Measurement in progress: "
                              f"{self.measurement_progress.percent}%")  # while loginfo doesn't work
                time.sleep(1)
            if self.state == XyzSystemState.STOPPED:
                # Someone stopped the measurement, exit.
                return
            action = actions.get(which_motor_and_side[i], None)
            if action is not None:
                action()

        self._set_state_to_finished()
        self.measurement_progress = MeasurementProgress(
            data=result, percent=100, last_measurement_point=i)
        self.log.info("Measurement finished.")
        print("Measurement finished.")  # while loginfo doesn't work

    def _set_to_running(self):
        if self.state == XyzSystemState.RUNNING:
            raise ValueError("The system is already busy.")
        self.state = XyzSystemState.RUNNING

    def _set_state_to_stopped(self):
        if self.state != XyzSystemState.RUNNING:
            self.log.warn("There is no measurement currently running.")
        self.state = XyzSystemState.STOPPED

    def _set_state_to_beginning(self):
        if self.state == XyzSystemState.RUNNING:
            raise ValueError("You can't do this action while system is busy.")
        self.state = XyzSystemState.AT_THE_BEGINNING

    def _set_state_to_finished(self):
        self.state = XyzSystemState.FINISHED

    def _load_settings(self, path: str):
        module_name = "xyztank_settings"
        spec = importlib.util.spec_from_file_location(module_name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
        return module

    def _move_at_the_beginning(self):
        """
        Moves all motors to local (0.0.0) point - left bottom corner of grid.
        """
        plan = self.get_plan()
        motors = self.get_motors()
        distances = []
        for i in range(3):
            distances.append(motors[i].position - plan.position[i])
            if distances[i] > 0:
                motors[i]._rotate_left(distances[i])
            else:
                motors[i]._rotate_right(abs(distances[i]))

        self._set_state_to_beginning()

    def _make_scan_route(self):
        """
        Count scan route for measurement based on current plan and add member
        scan_route.
        """
        plan = self.get_plan()
        grid_x, grid_y, grid_z = plan.grid
        len_grid_z = len(grid_z)
        len_grid_y = len(grid_y)
        len_grid_x = len(grid_x)
        all_xyz_values = len_grid_z * len_grid_y * len_grid_x
        all_xy_values = len_grid_x * len_grid_y
        all_yz_values = len_grid_z * len_grid_y
        indexes_z = np.arange(0, len_grid_z, dtype='int32')
        indexes_z = np.repeat(indexes_z, all_xy_values)
        indexes_y = np.arange(0, len_grid_y, dtype='int32')
        indexes_y = np.repeat(indexes_y, len_grid_x)
        indexes_y_forth = indexes_y
        indexes_y_back = np.flip(indexes_y)
        indexes_y_double = np.concatenate((indexes_y_forth, indexes_y_back))
        indexes_y_double = indexes_y_double.reshape([indexes_y_double.size, 1])
        if len_grid_z % 2 == 0:
            indexes_y = np.repeat(indexes_y_double, len_grid_z / 2, axis=1)
            indexes_y = indexes_y.transpose()
            indexes_y = indexes_y.flatten()
        else:
            indexes_y = np.repeat(indexes_y_double, (len_grid_z - 1) / 2,
                                  axis=1)
            indexes_y = indexes_y.transpose()
            indexes_y = indexes_y.flatten()
            indexes_y = np.concatenate((indexes_y, indexes_y_forth))

        indexes_x = np.arange(0, len_grid_x, dtype='int32')
        indexes_x_forth = indexes_x
        indexes_x_back = np.flip(indexes_x)
        indexes_x_double = np.concatenate((indexes_x_forth, indexes_x_back))
        indexes_x_double = indexes_x_double.reshape([indexes_x_double.size, 1])
        which_side = np.repeat(np.array([1, 0], dtype='int32'), len_grid_x)
        which_side = which_side.reshape([which_side.size, 1])
        if all_yz_values % 2 == 0:
            indexes_x = np.repeat(indexes_x_double, int(all_yz_values / 2),
                                  axis=1)
            indexes_x = indexes_x.transpose()
            indexes_x = indexes_x.flatten()
            which_side = np.repeat(which_side, int(all_yz_values / 2), axis=1)
            which_side = which_side.transpose()
            which_side = which_side.flatten()
        else:
            indexes_x = np.repeat(indexes_x_double,
                                  int((all_yz_values - 1) / 2), axis=1)
            indexes_x = indexes_x.transpose()
            indexes_x = indexes_x.flatten()
            indexes_x = np.concatenate((indexes_x, indexes_x_forth))
            which_side = np.repeat(which_side, int((all_yz_values - 1) / 2),
                                   axis=1)
            which_side = which_side.transpose()
            which_side = which_side.flatten()
            which_side = np.concatenate(
                (which_side, np.ones(len_grid_x, dtype='int32'),))

        y_movement = np.arange(len_grid_x - 1, all_xyz_values, len_grid_x)
        y_movement = y_movement.reshape(
            [int(y_movement.size / len_grid_y), len_grid_y])
        y_movement_forth = y_movement[::2, :]
        y_movement_back = y_movement[1::2, :]
        y_movement_forth = y_movement_forth.flatten()
        y_movement_back = y_movement_back.flatten()
        z_movement = np.arange(all_xy_values - 1, all_xyz_values, all_xy_values)
        which_motor = np.zeros(all_xyz_values, dtype='int32')
        which_motor[y_movement] = 1
        which_motor[z_movement] = 2
        which_motor[-1] = 3
        which_side[y_movement_forth] = 1
        which_side[y_movement_back] = 0
        which_side[z_movement] = 1
        which_side[-1] = 2
        which_motor_and_side = 10 * which_motor + which_side
        self.scan_route = ScanRoute(indexes_x, indexes_y, indexes_z,
                               which_motor_and_side)

    def _move_motor_x_right(self):
        plan = self.get_plan()
        motor = self.get_motor_x()
        motor._rotate_right(plan.grid_precision[0])
        # print("x right")

    def _move_motor_x_left(self):
        plan = self.get_plan()
        motor = self.get_motor_x()
        motor._rotate_left(plan.grid_precision[0])
        # print("x left")

    def _move_motor_y_right(self):
        plan = self.get_plan()
        motor = self.get_motor_y()
        motor._rotate_right(plan.grid_precision[1])
        # print("y right")

    def _move_motor_y_left(self):
        plan = self.get_plan()
        motor = self.get_motor_y()
        motor._rotate_left(plan.grid_precision[1])
        # print("y left")

    def _move_motor_z_right(self):
        plan = self.get_plan()
        motor = self.get_motor_z()
        motor._rotate_right(plan.grid_precision[2])
        # print("z right")
