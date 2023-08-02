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

from serial import Serial
import can
import xyztank.TMCL as TMCL

# port=can.Bus(interface='socketcan',
#               channel='vcan0',
#               receive_own_messages=True)
port=Serial("COM")       #HERE INSERT USED SERIAL PORT
bus=TMCL.connect(port)
module=bus.get_module(1)


@dataclass(frozen=True)
class Tank:
    """
    Tank (aquarium).

    :param name: name of the tank
    :param dimensions: dimensions of the tank, width (OX), depth (OY),
      height (OZ)
    :param position: coordinates x,y,z  of point which is closer bottom left
    corner of tank in global coordinate system
    """
    name: str
    dimensions: Tuple[float, float, float]
    position: Tuple[float, float, float]


@dataclass(frozen=False)
class Hydrophone:
    """
    Hydrophone.

    :param name: name of the hydrophone
    :param dimensions: dimensions of the hydrophone, width (OX), depth (OY),
    height (OZ)
    :param position: coordinates x,y,z  of center point of hydrophone
     in global coordinate system
    :safety_margin: minimal distances in m in each axis that other objects
    should be away from hydrophone
    """
    name: str
    dimensions: Tuple[float, float, float]
    position: Tuple[float, float, float]
    safety_margin: float

    def set_dimensions(self, dimension_x, dimension_y, dimension_z):
        self.dimensions = (dimension_x, dimension_y, dimension_z)

    def get_dimensions(self):
        return self.dimensions


@dataclass(frozen=False)
class UltrasoundTransducer:
    """
    UltrasoundTransducer.

    :param name: name of the ultrasound transducer
    :param dimensions: dimensions of the ultrasound transducer, width (OX),
    depth (OY), height (OZ)
    :param position: coordinates x,y,z  of center point of bottom of the
    ultrasound transducer in global coordinate system
    """
    name: str
    dimensions: Tuple[float, float, float]
    position: Tuple[float, float, float]


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
      20: motor_z to left
      21: motor_z to right}
      right - moving away from local and global (0.0.0)
      left - moving closer to local and global (0.0.0)
    """
    def __init__(self, indexes_x, indexes_y, indexes_z, which_motor_and_side):
        self.indexes_x = indexes_x
        self.indexes_y = indexes_y
        self.indexes_z = indexes_z
        self.which_motor_and_side = which_motor_and_side


@dataclass(frozen=False)
class MeasurementPlan:
    """
    A plan of measurement to execute in the system.

    :param name: name of the measurement
    :param tank: the tank in which the measurement was made
    :param min_position: minimal values of x, y, z coordinates in global
    coordinates system (also measurement start point)
    :param max_position: maximal values of x, y, z coordinates in global
    coordinates system
    :param grid_precision: distance between points in grid in all axis
    :param scan_route: enables scanning in the previously planned order
    """
    name: str
    tank: Tank
    is_vertical: bool
    min_position: Tuple[float, float, float]
    max_position: Tuple[float, float, float]
    grid: Tuple[np.ndarray, np.ndarray, np.ndarray]
    grid_precision: Tuple[float, float, float]

    def set_grid(self):
        grid = []
        for i in range(3):
            grid.append(np.arange(self.min_position[i], self.max_position[i] +
                                  0.5 * self.grid_precision[i],
                                  self.grid_precision[i]))

        self.grid = (grid[0], grid[1], grid[2])

    def check_positions(self, hydrophone: Hydrophone):
        """
        Check whether max and min position were given correctly and whether grid
        is suitable for size of aquarium (when we consider size of hydrophone too).
        """

        for i in range(3):
            if self.min_position[i] > self.max_position[i]:
                raise ValueError("Min position can't be bigger than max "
                                 "position")

        for i in range(3):
            if (self.min_position[i] - 0.5 * hydrophone.dimensions[i] - hydrophone.safety_margin) < self.tank.position[i]:
                raise ValueError("Grid is too big for aquarium")

            if (self.max_position[i] + 0.5 * hydrophone.dimensions[i] +
                hydrophone.safety_margin) > (self.tank.position[i] +
                                                self.tank.dimensions[i]):
                raise ValueError("Grid is too big for aquarium")




class XyzSystemState(Enum):
    """
    State of the XYZ system.

    - AT_THE_BEGINNING: XYZ is at the local beginning point (0.0.0).
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

       :param position: position in one axis from global point 000 in which
       motor currently is.
       :param max_distance: big number which surely makes motor rotate till
       0 in one axis in global coordinate system.

       """
    def __init__(self, axis):
        self.position = 0
        self.max_distance = 3.0
        self.motor=module.get_motor(axis)

    def rotate_left(self, distance):
        if self.position-distance > self.max_distance:
            raise ValueError("Requester position higher than max position")
        if self.position-distance < 0:
            raise ValueError("Requester position lower than min")
        else:
            self.position = self.position - distance
            self.motor.move_absolute(self.position)

    def rotate_right(self, distance):
        if self.position+distance > self.max_distance:
            raise ValueError("Requester position higher than max position")
        if self.position+distance < 0:
            raise ValueError("Requester position lower than min")
        else:
            self.position = self.position + distance
            self.motor.move_absolute(self.position)


class XyzSystem:

    def __init__(self):
        self.log = get_logger(type(self).__name__)
        self.measurement_plan = None  # Current measurement plan
        self.measurement_progress = None
        self.measurement_thread = None
        self.state = XyzSystemState.STOPPED
        self.motor_x = Motor(0)
        self.motor_y = Motor(1)
        self.motor_z = Motor(2)
        self.scan_route = None
        self.hydrophone = None
        self.ultrasound_transducer = None

    def run_settings(self, settings_path: str):
        """
        Load settings from the path, configure the system and run measurement.

        :param settings_path: path to the settings file
        """
        settings = self._load_settings(settings_path)
        self.hydrophone = settings.hydrophone
        self.ultrasound_transducer = settings.ultrasound_transducer
        self.run_measurement(settings.plan)

    def run_measurement(self, plan: MeasurementPlan):
        """
        Configure the system and run measurement.

        :param plan: measurement plan to run
        """
        self.configure_measurement(plan)
        self.move_at_the_global_beginning()
        self._move_at_the_local_beginning()
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
        if plan.is_vertical is not True:
            hydrophone_x, hydrophone_y, hydrophone_z = self.hydrophone.get_dimensions()
            hydrophone_y, hydrophone_z = hydrophone_z, hydrophone_y
            self.hydrophone.set_dimensions(hydrophone_x, hydrophone_y, hydrophone_z)

        plan.check_positions(self.hydrophone)
        plan.set_grid()
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
        if self.state == XyzSystemState.RUNNING:
            self.log.warn("There is a measurement currently running.")
        else:
            self.log.info(f"Resuming the current measurement")
            self._set_to_running()

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
            20: self._move_motor_z_left,
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

    def _move_at_the_local_beginning(self):
        """
        Moves all motors to local (0.0.0) point.

        In vertical measurement local point is max z, min y, min x of grid.
        In horizontal measurement local point is min z, max y, min x of grid.

        """
        plan = self.get_plan()
        motors = self.get_motors()
        distances = []
        distances.append(motors[0].position - plan.min_position[0])
        if plan.is_vertical is True:
            distances.append(motors[1].position - plan.min_position[1])
            distances.append(motors[2].position - plan.max_position[2])
        else:
            distances.append(motors[1].position - plan.max_position[1])
            distances.append(motors[2].position - plan.min_position[2])

        for i in range(3):
            if distances[i] > 0:
                motors[i].rotate_left(distances[i])
            else:
                motors[i].rotate_right(abs(distances[i]))

        self._set_state_to_beginning()

    def move_at_the_global_beginning(self):
        """
        Moves all motors to global (0.0.0) point.
        """
        motors = self.get_motors()
        for i in range(3):
            motors[i].rotate_left(motors[i].max_distance)
            motors[i].position = 0

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
        is_vertical = plan.is_vertical;

        if is_vertical is not True:
            a = len_grid_y
            len_grid_y = len_grid_z
            len_grid_z = a

        all_xyz_values = len_grid_z * len_grid_y * len_grid_x
        all_xy_values = len_grid_x * len_grid_y
        all_yz_values = len_grid_z * len_grid_y
        indexes_z = np.arange(0, len_grid_z, dtype='int32')
        indexes_z = np.repeat(indexes_z, all_xy_values)
        indexes_z = np.flip(indexes_z)
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
        which_side[z_movement] = 0
        which_side[-1] = 2
        which_motor_and_side = 10 * which_motor + which_side

        if is_vertical is not True:
            a = indexes_y
            indexes_y = indexes_z
            indexes_z = a
            which_motor_and_side[which_motor_and_side == 11] = 21
            which_motor_and_side[which_motor_and_side == 10] = 3
            which_motor_and_side[which_motor_and_side == 20] = 4
            which_motor_and_side[which_motor_and_side == 3] = 20
            which_motor_and_side[which_motor_and_side == 4] = 10

        self.scan_route = ScanRoute(indexes_x, indexes_y, indexes_z,
                                    which_motor_and_side)

    def _move_motor_x_right(self):
        plan = self.get_plan()
        motor = self.get_motor_x()
        motor.rotate_right(plan.grid_precision[0])
        # print("x right")

    def _move_motor_x_left(self):
        plan = self.get_plan()
        motor = self.get_motor_x()
        motor.rotate_left(plan.grid_precision[0])
        # print("x left")

    def _move_motor_y_right(self):
        plan = self.get_plan()
        motor = self.get_motor_y()
        motor.rotate_right(plan.grid_precision[1])
        # print("y right")

    def _move_motor_y_left(self):
        plan = self.get_plan()
        motor = self.get_motor_y()
        motor.rotate_left(plan.grid_precision[1])
        # print("y left")

    def _move_motor_z_right(self):
        plan = self.get_plan()
        motor = self.get_motor_z()
        motor.rotate_right(plan.grid_precision[2])
        # print("z right")

    def _move_motor_z_left(self):
        plan = self.get_plan()
        motor = self.get_motor_z()
        motor.rotate_left(plan.grid_precision[2])
        # print("z left")
