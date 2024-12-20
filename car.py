from util import Context, Config, ReadData, ControlData
import numpy as np


config: Config = Config()


# get current location of robot
def get_location(context: Context, youbot_data: ReadData):
    return youbot_data.localization


# move car to pick location
def move_to_pick(context: Context, youbot_data: ReadData):
    result: bool = False
    control_data: ControlData = ControlData()

    if context.state_counter == 1:
        # pick location is already defined. (Context.pick_location_id)
        control_data.control_cb = move_cb
        control_data.read_lidar = True
        control_data.wheels_velocity_el = [0.0, 0.0, 0.0]  # init
        print("move_to_pick")
    else:
        result = True

    return result, control_data


# move car to place location
def move_to_place(context: Context, youbot_data: ReadData):
    result: bool = False
    control_data: ControlData = ControlData

    if context.state_counter == 1:
        # pick location is already defined. (Context.place_location_id)
        control_data.control_cb = move_cb
        control_data.read_lidar = True
        control_data.wheels_velocity_el = [0.0, 0.0, 0.0]  # init
        print("move_to_place")
    else:
        result = True

    return result, control_data


# move car to base
def move_to_base(context: Context, youbot_data: ReadData):
    result: bool = False
    control_data: ControlData = ControlData

    if context.state_counter == 1:
        # pick location is already defined. (Context.base)
        control_data.control_cb = move_cb
        control_data.read_lidar = True
        control_data.wheels_velocity_el = [0.0, 0.0, 0.0]  # init
        print("move_to_base")
    else:
        result = True

    return result, control_data


def move_cb(context: Context, youbot_data: ReadData, control_data: ControlData):

    forwback_vel = control_data.wheels_velocity_el[0]
    side_vel = control_data.wheels_velocity_el[1]
    rot_vel = control_data.wheels_velocity_el[2]

    result = False

    if context.path_planning_state:
        # Update the ControlData.wheels_velocity
        control_data.wheels_velocity = [
            -forwback_vel - side_vel - rot_vel,
            -forwback_vel + side_vel - rot_vel,
            -forwback_vel + side_vel + rot_vel,
            -forwback_vel - side_vel + rot_vel,
        ]
        # if close enough to target, stop path planning
        distance = np.linalg.norm(
            np.array(context.goal_location) - np.array(youbot_data.localization[:3])
        )
        # print(distance)
        if distance < 0.5:
            print(f"Goal reached.")
            context.path_planning_state = False
            result = True
    else:
        print("Path planning already completed.")
        result = True

    return result
