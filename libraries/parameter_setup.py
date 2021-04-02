# this is a thing

import json


class Parameters:
    """ A class containing hardware parameters and their setup """

    # default factory variables
    wheel_radius = 0.05  # metres
    wall_distance = 2.5  # metres
    pole_width = 0.2  # metres
    turn_multiplier = 0.45  # scalar
    move_multiplier = 0.45  # scalar
    camera_width = 128  # pixels
    camera_heigth = 128  # pixels
    relative_table_height = -0.2  # metres

    @classmethod
    def setup(
        Parameters,
        webots=True,
        robot=None,
        path_to_json="/Railly-Clean/libraries/params.json",
    ):
        # check if it is called form webots - if yes, just read in the params we can from the robot object
        with open(path_to_json, "r") as read_json:
            params = json.load(read_json)
            wheel_radius = params["Wheel radius"]
            wall_distance = params["Distance to wall"]
            pole_width = params["Pole width"]
            turn_multiplier = params["Turn multiplier"]
            move_multiplier = params["Move multiplier"]
            camera_width = (
                params["Front camera width"]
                if not webots
                else robot.getDevice("front_camera").getWidth()
            )
            camera_heigth = (
                params["Front camera height"]
                if not webots
                else robot.getDevice("front_camera").getHeight()
            )
            relative_table_height = params["Table height"] - 0.97


# for testing
if __name__ == "__main__":
    Parameters.setup(webots=False, path_to_json="params.json")
    print(Parameters.wall_distance)