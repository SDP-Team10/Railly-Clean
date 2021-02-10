# this is a thing

# note to self - what to do when the thing is stopped by something else - obstacle detection?
# note to note - that's milestone 3 yous problem - maybe add a boolen is_stopped and substract that time from end?

from collections import deque


class SideCheck:
    # can import these as json?
    def __init__(self):
        self.params = {
            "WHEEL_RADIUS": 0.0985,  # get from webots
            "SPEED": 4.875,  # get from webots
            "DISTANCE_TO_WALL": 1.7,  # should be setup parameter while installing in carriage
        }

        self._current_side_distance = 0
        self._previous_side_distance = 0
        self._empty_space = 0
        self._empty_start = None
        self._occupied_space = 0
        self._occupied_start = None

        self._enable_updates = True  # once a table is found, set to false to keep distances to help with navigation
        self._distance_morse = deque([], maxlen=7)

    def get_distance_since(self, time, wheel_radius, speed, robot):
        """Funtion get_distance_since: Returns the distance that the robot has travelled since the given time.

        Parameters:
            :param time: time when the distance measure started.
            :param wheel_radius: radius of the wheel in meters - declare as constant in webots controller.
            :param speed: angular velocity of the robot - assumes constant speed.
            :param robot: robot object in webots.
        """
        return (wheel_radius * speed) * (robot.getTime() - time)

    def that_a_table(self, distances):
        """Function that_a_table to check if the thing the robot just passed a table.

        Parameters:
            :param distances: A list of 5 distances recorded by the robot.

        Returns:
            :return boolean: True if it is assumed to be a table, false othersiwe
        """
        if len(distances) != 5:
            # if there aren't 5 elements, either not enough data to decide or incorrect list passed
            return False
        else:
            # if it was a table, the order of distances should be:
            # {seat, legspace, table pole, legspace, seat}
            seat = (distances[0] + distances[-1]) / 2
            table_pole = distances[2]
            if table_pole < (seat / 2) and table_pole < 0.2:
                print("We just passed a table! Should move back in front of the pole!")
                self._enable_updates = False
                return True
            else:
                return False

    def done_cleaning(self):
        """Function that re-enables the side_check method when the robot is done cleaning."""
        self._enable_updates = True

    def stop_scanning(self):
        """Function stop_scanning to be called when the robot is stopping for any other reason than this instance"""
        self._enable_updates = False

    def unoccupied_table(self, distance_sensor, distances):
        """Function unoccupied_table to check if the table is occupeid - UNUSED.

        Function is currently unused, no hardware support implemented after change of scope
        This function assumes the robot is currently right in front of a table.
        Ergo, after funtion that_a_table, move the robot back before calling this one.
        Returns True if the seats are detected to be unoccupied, False otherwise
        """
        # start by moving back to the previous seat
        # robot should move back (distances[2] / 2) + distances[1] + (distances[0] / 2)
        self.move_distance(((distances[2] / 2) + distances[1] + (distances[0] / 2)), -1)
        # robot should active slightyly-higher distance sensor
        if distance_sensor.getValue() < (self.params["DISTANCE_TO_WALL"] - 0.2):
            # occupant detected!
            # move back to pole - same as previous distance
            self.move_distance(
                ((distances[2] / 2) + distances[1] + (distances[0] / 2)), 1
            )
            self._enable_updates = True
            # carry on operations
            return False
        else:
            # this row of seats were unoccupied!
            # move next to front seats distances (distances[0] / 2)  + distances[1:4] + (distances[4] / 2)
            self.move_distance(
                (distances[0] / 2) + sum(distances[1:4]) + (distances[4] / 2), 1
            )
            # check top sensor again!
            if distance_sensor.getValue() < (self.params["DISTANCE_TO_WALL"] - 0.2):
                # occupant detected!
                # move back to end of seat -> distances[4] / 2
                self.move_distance((distances[4] / 2), 1)
                self._enable_updates = True
                # carry on operations
                return False
            else:
                # This table is clear of passangers!
                # move back to pole <- (distances[4] / 2) + distances[3] + (distances[2] / 2)
                self.move_distance(
                    (distances[0] / 2) + sum(distances[1:4]) + (distances[4] / 2), -1
                )
                self._enable_updates = True
                return True

    def move_distance(self, distance, direction):
        """Function move distance - makes the robot move a predetermined distance then stop

        Parameters:
            :param distance: Distance to move in meters
            :param direction: positive for forward, negative for backwards
        """
        # pls help movement team?

    def side_check(self, robot, side_sensor):
        # use global variables and hope that state is stored between loops
        # side_sensor = ds[3]  # corresponding distance sensor in seat level
        # passanger_sensor = ds[4]  # distance sensor in passenet butt level - unused

        self._current_side_distance = side_sensor.getValue()
        # check if there is a larger than noise vairance in the distance sensor
        if (
            abs(self._current_side_distance - self._previous_side_distance) > 0.3
            and self._enable_updates
        ):
            # if it's a rising edge, distance grows, end of chair/pole
            if self._current_side_distance > self._previous_side_distance:
                # save start time of empty space
                self._empty_start = robot.getTime()
                print("End of object - Empty space stared")
                # reset empty space distance
                self._empty_space = 0
                # calculate distance if there was a start
                self._occupied_space = (
                    self.get_distance_since(
                        self._occupied_start,
                        self.params["WHEEL_RADIUS"],
                        self.params["SPEED"],
                        robot,
                    )
                    if (self._occupied_start is not None)
                    else 0
                )
                self._distance_morse.append(self._occupied_space)
                # call self.that_a_table here - based on seat-pole-seat sizes, only needed after solids
                if self.that_a_table(self._distance_morse):  # if a table is found:
                    # return estimated table width to the main controller (_distance_morse[1] + _distance_morse[3])
                    return self._distance_morse[1] + self._distance_morse[3]
                    # move back to the center of the pole
                    # robot should move back distances[3:] + ( distance[2] / 2 ) - half the pole, seat, legspace
                    # self.move_distance(
                    #    sum(self._distance_morse[3:]) + (self._distance_morse[2] / 2),
                    #    -1,
                    # )
                    # if self.unoccupied_table(
                    #     passanger_sensor, self._distance_morse
                    # ):  # check if table is unoccupied
                    #     # do the cleaning
                    #     pass
            # falling edge, distance shrinks, start of chair/pole
            else:
                self._occupied_start = robot.getTime()
                print("End of empty spae - Object started")
                self._occupied_space = 0
                # calculate distance if there was a start
                self._empty_space = (
                    self.get_distance_since(
                        self._empty_start,
                        self.params["WHEEL_RADIUS"],
                        self.params["SPEED"],
                        robot,
                    )
                    if (self._occupied_start is not None)
                    else 0
                )
                self._distance_morse.append(self._empty_space)
