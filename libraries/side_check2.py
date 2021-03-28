#!/usr/bin/python3

from collections import deque
import os
import sys
from libraries import move_lib_new_base as mc  # dependent on where script is run


class SideCheck:
    def __init__(self, robot_inst):
        self.params = {
            "WHEEL_RADIUS": mc.WHEEL_RADIUS,  # from movement library
            "SPEED": mc.NORM_SPEED,  # from movement library
            "DISTANCE_TO_WALL": 2.5,  # should be setup parameter while installing in carriage
            "MAX_POLE_LENGTH": 0.25,
        }

        self._robot = robot_inst

        self._current_side_distance = 0
        self._previous_side_distance = 0
        self._empty_space = 0
        self._empty_start = None
        self._occupied_space = 0
        self._occupied_start = None

        self._stopped_start = 0

        self._enable_updates = True  # once a table is found, set to false to keep distances to help with navigation
        self._length_morse = deque(
            [], maxlen=3
        )  # queue used for checking for tables - when there was a table, elements should be {seat, leg-space, table pole}
        self._distance_wall = deque(
            [], maxlen=2
        )  # 2x the empty space distance - should be same as distance to wall

        self.max_distance_to_wall = (
            2.0  # Fix setup parameter - add 0.1 to base parameter
        )

    def get_distance_since(self, time):
        """Function get_distance_since: Returns the distance that the robot has travelled since the given time.

        Parameters:
            :param time: time when the distance measure started.
        """
        return (self.params["WHEEL_RADIUS"] * self.params["SPEED"]) * (
            self._robot.getTime() - time
        )

    def that_a_table(self):
        """Function that_a_table to check if the thing the robot just passed a table.

        Parameters:
            :param distances: A list of 5 distances recorded by the robot.

        Returns:
            :return boolean: True if it is assumed to be a table, false otherwise
        """
        if len(self._length_morse) != 3:
            # if there aren't 3 elements, either not enough data to decide or incorrect list passed
            return False
        else:
            # if it was a table, the order of distances should be:
            # {seat, leg-space, table pole}
            pole = self._length_morse[2]
            seat = self._length_morse[0]
            if pole < seat and pole < self.params["MAX_POLE_LENGTH"]:
                print("We just passed a table pole!")
                print(self._length_morse)
                self._enable_updates = False
                return True
            else:
                return False

    def restart_scanning(self):
        """Function that re-enables the side_check method when the robot is done cleaning."""
        # Add the difference of the beginning and end time of stopping to the variables holding times
        # If zero, then not currently in use, leave zero
        self._occupied_start += (
            0
            if (self._occupied_start == 0)
            else (self._robot.getTime() - self._stopped_start)
        )
        self._empty_start += (
            0
            if (self._empty_start == 0)
            else (self._robot.getTime() - self._stopped_start)
        )
        self._enable_updates = True

    def stop_scanning(self):
        """Function stop_scanning to be called when the robot is stopping for any other reason than this instance"""
        self._stopped_start = self._robot.getTime()  # save the time when it stopped
        self._enable_updates = False

    def side_check(self, side_sensor):
        # side_sensor  # corresponding distance sensor in seat level
        self._previous_side_distance = self._current_side_distance
        self._current_side_distance = side_sensor.getValue()
        # Check if sensor data is withing expected range - filter out faulty readings
        if self._current_side_distance > self.max_distance_to_wall:
            self._current_side_distance = self.max_distance_to_wall

        # check if there is a larger than noise variance in the distance sensor
        print("Checking for table")
        if (
            abs(self._current_side_distance - self._previous_side_distance) > 0.3
            and self._enable_updates
        ):
            print(self._length_morse)
            # if it's a rising edge, distance grows, end of chair/pole
            if self._current_side_distance > self._previous_side_distance:
                print("End of object")
                # save start time of empty space
                self._empty_start = self._robot.getTime()
                self.params["DISTANCE_TO_WALL"] = (
                    self._current_side_distance + self.params["DISTANCE_TO_WALL"]
                ) / 2
                # reset empty space distance
                self._empty_space = 0
                # calculate distance if there was a start
                self._occupied_space = (
                    self.get_distance_since(self._occupied_start)
                    if (self._occupied_start is not None)
                    else 0
                )
                self._length_morse.append(self._occupied_space)
                # call self.that_a_table here - based on seat-pole-seat sizes, only needed after solids
                if self.that_a_table():  # if a table is found:
                    # A table is found, so _lenght_morse is {seat, leg-space, table pole}
                    # return estimated table width to the main controller (_length_morse[1] * 2)
                    # also return estimated length of table pole and recorded distance to last chair
                    return (
                        self._length_morse[1] * 2,
                        self._length_morse[2],
                        self._distance_wall[0],
                    )

            # falling edge, distance shrinks, start of chair/pole
            else:
                self._occupied_start = self._robot.getTime()
                print("End of empty space - Object started")
                self._occupied_space = 0
                # calculate distance if there was a start
                self._empty_space = (
                    self.get_distance_since(self._empty_start)
                    if (self._occupied_start is not None)
                    else 0
                )
                self._length_morse.append(self._empty_space)
                self._distance_wall.append(self._current_side_distance)
                print("Side:", self._current_side_distance)
        return None, None, None
