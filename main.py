# Copyright 2024 @With-Robot 3rd
#
# Licensed under the MIT License;
#     https://opensource.org/license/mit

import cv2
import numpy as np
import math

from pynput import keyboard
from pynput.keyboard import Listener

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from scipy.spatial.transform import Rotation as R

from util import State, Context, Mission, ReadData, ControlData, Config
from car import get_location, move_to_pick, move_to_place, move_to_base
from manipulator import find_target, pick_target, place_target, approach_to_target

from flask import Flask, request
import threading

client = None
app = Flask(__name__)


#
# A class for the entire pick-and-place operation
#
class PickAndPlace:
    def __init__(self):
        print("class PickAndPlace instance generated.")
        # coppeliasim simulation instance
        self.sim = RemoteAPIClient().require("sim")
        print(f"PickAndPlace member sim: {self.sim}")
        self.simOMPL = RemoteAPIClient().require("simOMPL")
        print(f"PickAndPlace member simOMPL: {self.simOMPL}")
        # Context of Robot
        self.context = Context()
        # Simulation run flag
        self.run_flag = True
        self.f_len = self.getGlobalFocalLength()

    # key even listener
    def on_press(self, key):
        # Pressing 'a' key will start the mission.
        if key == keyboard.KeyCode.from_char("a"):
            # set the pick & place locations
            self.init_mission("/bedroom2", "/bedroom1", "Cube")

        # Pressing 'q' key will terminate the simulation
        if key == keyboard.KeyCode.from_char("q"):
            self.run_flag = False

    def init_mission(self, pick_location, place_location, target):
        # set the pick & place locations
        self.context.mission = Mission(pick_location, place_location, target)
        if self.context.mission.pick_location in self.predefined_points:
            # register pick & place locations
            self.context.pick_location_id = self.goal_locations[
                self.context.mission.pick_location
            ]
        if self.context.mission.place_location in self.predefined_points:
            self.context.place_location_id = self.goal_locations[
                self.context.mission.place_location
            ]

    # init coppeliasim objects
    def init_coppelia(self):
        # robot (root object of the tree)
        self.youBot = self.sim.getObject("/youBot")
        # reference dummy
        self.youBot_ref = self.sim.getObject("/youBot_ref")
        # collision box
        self.collVolumeHandle = self.sim.getObject("/youBot_coll")
        # Wheel joints
        self.wheels = []
        self.wheels.append(self.sim.getObject("/rollingJoint_fl"))
        self.wheels.append(self.sim.getObject("/rollingJoint_rl"))
        self.wheels.append(self.sim.getObject("/rollingJoint_fr"))
        self.wheels.append(self.sim.getObject("/rollingJoint_rr"))

        # lidar
        self.lidars = []
        for i in range(13):
            self.lidars.append(self.sim.getObject(f"/lidar_{i+1:02d}"))

        # manipulator 5 joints
        self.joints = []
        for i in range(5):
            self.joints.append(self.sim.getObject(f"/youBotArmJoint{i}"))

        # Gripper Joint
        self.joints.append(self.sim.getObject(f"/youBotGripperJoint1"))
        self.joints.append(self.sim.getObject(f"/youBotGripperJoint2"))

        # camera
        self.camera_1 = self.sim.getObject(f"/camera_1")

        # joint & wheel control mode
        self.set_joint_ctrl_mode(self.wheels, self.sim.jointdynctrl_velocity)
        self.set_joint_ctrl_mode(self.wheels, self.sim.jointdynctrl_position)

        # goal locations (pre-positioned dummies)
        self.predefined_points = [
            "/bedroom1",
            "/bedroom2",
            "/toilet",
            "/entrance",
            "/dining",
            "/livingroom",
            "/balcony_init",
            "/balcony_end",
        ]
        # goal locations id
        self.goal_locations = {}
        for goal in self.predefined_points:
            self.goal_locations[goal] = self.sim.getObject(goal)

    # set joints dynamic control mode
    def set_joint_ctrl_mode(self, objects, ctrl_mode):
        for obj in objects:
            self.sim.setObjectInt32Param(
                obj, self.sim.jointintparam_dynctrlmode, ctrl_mode
            )

    # read youbot data
    def read_youbot(self, lidar=False, camera=False):
        read_data = ReadData()
        # read localization of youbot
        p = self.sim.getObjectPosition(self.youBot_ref)
        o = self.sim.getObjectQuaternion(self.youBot_ref)
        read_data.localization = np.array(p + o)  # [x,y,z,qw,qx,qy,qz]
        # read localization of canera
        cp = self.sim.getObjectPosition(self.camera_1)
        co = self.sim.getObjectOrientation(self.camera_1)
        read_data.cam_localization = np.array(cp + co)
        # read H matrix of youbot
        m = self.sim.getObjectMatrix(self.youBot_ref, -1)
        read_data.robot_mat = m
        # read wheel joints
        wheels = []  # Includes the angular positions of the wheels
        for wheel in self.wheels:  # self.wheels contains "ID Of 4 wheels"
            theta = self.sim.getJointPosition(wheel)
            wheels.append(theta)
        read_data.wheels = wheels
        # read manipulator joints
        joints = []
        for joint in self.joints:
            theta = self.sim.getJointPosition(joint)
            joints.append(theta)
        read_data.joints = joints
        # read lidar
        if lidar:
            scans = []
            for id in self.lidars:
                scans.append(self.sim.readProximitySensor(id))
            read_data.scans = scans
        # read camera
        if camera:
            result = self.sim.getVisionSensorImg(self.camera_1)
            img = np.frombuffer(result[0], dtype=np.uint8)
            img = img.reshape((result[1][1], result[1][0], 3))
            img = cv2.flip(img, 1)
            read_data.img = img
        # return read_data
        return read_data

    def find_path(self, youbot_data: ReadData, config: Config, goal_id):
        """Path planning implementation with retries until a path is found."""
        while True:  # Retry until path planning succeeds
            print(f"Attempting path planning to Goal ID: {goal_id}")
            if goal_id != None:
                self.goal_id = goal_id
                self.context.goal_location = self.sim.getObjectPosition(self.goal_id)
            else:  # Case of MoveToBase (only goal position info.)
                self.goal_id = None
                self.context.goal_location = self.context.base
            obstaclesCollection = self.sim.createCollection(0)
            self.sim.addItemToCollection(
                obstaclesCollection, self.sim.handle_all, -1, 0
            )
            self.sim.addItemToCollection(
                obstaclesCollection, self.sim.handle_tree, self.youBot, 1
            )
            collPairs = [self.collVolumeHandle, obstaclesCollection]

            search_algo = self.simOMPL.Algorithm.BiTRRT

            if self.context.path_planning_state:
                task = self.simOMPL.createTask("t")
                self.simOMPL.setAlgorithm(task, search_algo)
                # youbot_data.localization is np.array
                startPos = youbot_data.localization[:3]  # [x,y,z,qw,qx,qy,qz]
                if isinstance(startPos, np.ndarray):
                    startPos = startPos.tolist()
                # try-except
                try:
                    ss = [
                        self.simOMPL.createStateSpace(
                            "2d",
                            self.simOMPL.StateSpaceType.position2d,
                            self.collVolumeHandle,
                            [
                                startPos[0] - 10,
                                startPos[1] - 10,
                            ],
                            [
                                startPos[0] + 10,
                                startPos[1] + 10,
                            ],
                            1,
                        )
                    ]
                    self.simOMPL.setStateSpace(task, ss)
                    self.simOMPL.setCollisionPairs(task, collPairs)
                    self.simOMPL.setStartState(task, startPos[:2])
                    self.simOMPL.setGoalState(
                        task, list(self.context.goal_location[:2])
                    )  # context.goal_location is 'python list'
                    self.simOMPL.setStateValidityCheckingResolution(task, 0.01)
                    self.simOMPL.setup(task)
                except Exception as e:
                    print(f"Error in setStartState: {e}")
                    raise

                if self.simOMPL.solve(task, 0.1):
                    self.simOMPL.simplifyPath(task, 0.1)
                    path = self.simOMPL.getPath(task)
                    print(f"Path found: {path}")  # Check the result of planning.
                    path_3d = []
                    for i in range(0, len(path) // 2):
                        path_3d.extend([path[2 * i], path[2 * i + 1], 0.0])
                    self.path_data = path_3d
                    break  # Exit loop when path planning succeeds

                else:
                    print("Path planning failed. Retrying...")
                    # self.simOMPL.destroyTask(task)  # Cleanup and retry

    def control_youbot(self, config: Config, control_data: ControlData):
        control_data.exec_count += 1
        # robot's odometry and sensor data
        read_data = self.read_youbot(
            lidar=control_data.read_lidar, camera=control_data.read_camera
        )
        result = True

        path_3d = self.path_data
        if control_data.wheels_velocity is not None:
            currPos = read_data.localization[:3]
            # change numpy array to python list
            if isinstance(currPos, np.ndarray):
                currPos = currPos.tolist()
            if path_3d and isinstance(path_3d, list):
                pathLengths, totalDist = self.sim.getPathLengths(
                    path_3d, 3
                )  # pathLengths: list

                closet_dist = self.sim.getClosestPosOnPath(
                    path_3d, pathLengths, currPos
                )
                # change numpy array to python list
                if isinstance(closet_dist, np.ndarray):
                    closet_dist = closet_dist.tolist()
                if closet_dist <= self.context.prev_dist:
                    closet_dist += totalDist / 200
                self.context.prev_dist = closet_dist

                targetPoint = self.sim.getPathInterpolatedConfig(
                    path_3d, pathLengths, closet_dist
                )  # targetPoint: list

                # Calc. the velocity for each of the 4 mecanum wheels
                m = read_data.robot_mat
                # change the numpy array to python lsit
                if isinstance(m, np.ndarray):
                    m = m.tolist()
                m_inv = self.sim.getMatrixInverse(m)
                rel_p = self.sim.multiplyVector(m_inv, targetPoint)
                rel_o = math.atan2(rel_p[1], rel_p[0]) - math.pi / 2

                p_parm = config.drive_parms["p_parm"]
                p_parm_rot = config.drive_parms["p_parm_rot"]
                max_v = config.drive_parms["max_v"]
                max_v_rot = config.drive_parms["max_v_rot"]
                accel_f = config.drive_parms["accel_f"]

                forwback_vel = rel_p[1] * p_parm
                side_vel = rel_p[0] * p_parm

                v = (forwback_vel**2 + side_vel**2) ** 0.5
                if v > max_v:
                    forwback_vel *= max_v / v
                    side_vel *= max_v / v

                rot_vel = -rel_o * p_parm_rot
                if abs(rot_vel) > max_v_rot:
                    rot_vel = max_v_rot * rot_vel / abs(rot_vel)

                prev_forwback_vel = control_data.wheels_velocity_el[0]
                prev_side_vel = control_data.wheels_velocity_el[1]
                prev_rot_vel = control_data.wheels_velocity_el[2]

                df = forwback_vel - prev_forwback_vel
                ds = side_vel - prev_side_vel
                dr = rot_vel - prev_rot_vel

                if abs(df) > max_v * accel_f:
                    df = max_v * accel_f * df / abs(df)
                if abs(ds) > max_v * accel_f:
                    ds = max_v * accel_f * ds / abs(ds)
                if abs(dr) > max_v_rot * accel_f:
                    dr = max_v_rot * accel_f * dr / abs(dr)

                forwback_vel = prev_forwback_vel + df
                side_vel = prev_side_vel + ds
                rot_vel = prev_rot_vel + dr

                # control_data 에 세가지 요소 저장
                control_data.wheels_velocity_el[0] = forwback_vel
                control_data.wheels_velocity_el[1] = side_vel
                control_data.wheels_velocity_el[2] = rot_vel
            else:
                forwback_vel = 0
                side_vel = 0
                rot_vel = 0

            self.sim.setJointTargetVelocity(
                self.wheels[0], -forwback_vel - side_vel - rot_vel
            )
            self.sim.setJointTargetVelocity(
                self.wheels[1], -forwback_vel + side_vel - rot_vel
            )
            self.sim.setJointTargetVelocity(
                self.wheels[2], -forwback_vel + side_vel + rot_vel
            )
            self.sim.setJointTargetVelocity(
                self.wheels[3], -forwback_vel - side_vel + rot_vel
            )

            if self.goal_id is not None:
                if (
                    np.linalg.norm(
                        np.array(self.sim.getObjectPosition(self.goal_id, -1))
                        - np.array(self.sim.getObjectPosition(self.youBot_ref, -1))
                    )
                    < 0.6
                ):
                    # self.sim.removeDrawingObject(self.context.line_container)
                    self.context.path_planning_state = False
                    control_data.wheels_velocity = None
                    result = True
            elif self.context.base is not None:
                if (
                    np.linalg.norm(
                        np.array(self.context.base)
                        - np.array(self.sim.getObjectPosition(self.youBot_ref, -1))
                    )
                    < 0.6
                ):
                    self.context.path_planning_state = False
                    control_data.wheels_velocity = None
                    result = True
            else:
                return True

        if control_data.wheels_position is not None:
            diff_sum = 0
            for i, wheel in enumerate(control_data.wheels_position):
                diff = abs(wheel - read_data.wheels[i])
                diff_sum += diff
                diff = min(diff, control_data.delta)
                if read_data.wheels[i] < wheel:
                    target = read_data.wheels[i] + diff
                else:
                    target = read_data.wheels[i] - diff
                self.sim.setJointTargetPosition(self.wheels[i], target)
            result = diff_sum < 0.02
        if control_data.joints_position is not None:
            diff_sum = 0
            for i, joint in enumerate(control_data.joints_position):
                diff = abs(joint - read_data.joints[i])
                diff_sum += diff
                diff = min(diff, control_data.delta)
                if read_data.joints[i] < joint:
                    target = read_data.joints[i] + diff
                else:
                    target = read_data.joints[i] - diff
                self.sim.setJointTargetPosition(self.joints[i], target)
            result = diff_sum < 0.02
        if control_data.gripper_state is not None:
            p1 = self.sim.getJointPosition(self.joints[-2])
            p2 = self.sim.getJointPosition(self.joints[-1])
            p1 += -0.005 if control_data.gripper_state else 0.005
            p2 += 0.005 if control_data.gripper_state else -0.005
            self.sim.setJointTargetPosition(self.joints[-2], p1)
            self.sim.setJointTargetPosition(self.joints[-1], p2)
            result = control_data.exec_count > 10
        if control_data.control_cb is not None:
            result = control_data.control_cb(self.context, read_data, control_data)
        return result

    def getGlobalFocalLength(self):
        camera_1 = self.sim.getObject(f"/camera_1")
        res, perspAngle = self.sim.getObjectFloatParameter(
            camera_1, self.sim.visionfloatparam_perspective_angle
        )
        res, resolution = self.sim.getVisionSensorResolution(camera_1)
        # distance per pixel
        planeWidth = 2 * math.tan(perspAngle / 2)
        distancePerPixel = planeWidth / resolution
        # global focal length
        # pixelFocalLength = (resolution / 2) / math.tan(perspAngle / 2)
        # globalFocalLength = pixelFocalLength * distancePerPixel
        return 1 / distancePerPixel

    # run coppeliasim simulator
    def run_coppelia(self):
        # register a keyboard listener
        # Listener(on_press=self.on_press).start()
        # start simulation
        self.sim.setStepping(True)
        self.sim.startSimulation()

        config = Config()
        control_data = None

        # execution of the simulation
        while self.run_flag:
            if control_data:  # if control data exists complete control
                if self.control_youbot(config, control_data):
                    control_data = None
                self.sim.step()
                continue

            self.context.inc_state_counter()

            if self.context.state == State.StandBy:
                if self.context.mission is not None:
                    print(f"state: {self.context.state}")
                    self.context.set_state(State.MoveToPick)
                    base = get_location(self.context, self.read_youbot(lidar=True))
                    self.context.base = base[:3]  # [x,y,z]
                    print(f"base : {self.context.base}")
            elif self.context.state == State.MoveToPick:
                if self.context.state_counter == 1:
                    print(f"state: {self.context.state}")
                    self.set_joint_ctrl_mode(
                        self.wheels, self.sim.jointdynctrl_velocity
                    )
                if self.context.path_planning_state:
                    self.find_path(
                        self.read_youbot(),
                        config,
                        goal_id=self.context.pick_location_id,
                    )
                result, control_data = move_to_pick(
                    self.context, self.read_youbot(lidar=True)
                )
                if result:
                    self.path_data = None  # Init path data
                    self.context.set_state(State.FindTarget)
            elif self.context.state == State.FindTarget:
                if self.context.state_counter == 1:
                    print(f"state: {self.context.state}")
                    self.set_joint_ctrl_mode(
                        self.wheels, self.sim.jointdynctrl_position
                    )
                result, control_data = find_target(
                    self.context, self.read_youbot(camera=True)
                )
                if result:
                    self.context.set_state(State.ApproachToTarget)
            elif self.context.state == State.ApproachToTarget:
                print(f"state: {self.context.state}")
                result, control_data = approach_to_target(
                    self.context, self.read_youbot(lidar=True)
                )
                if result:
                    self.context.set_state(State.PickTarget)
            elif self.context.state == State.PickTarget:
                print(f"state: {self.context.state}")
                result, control_data = pick_target(
                    self.context, self.read_youbot(camera=True), self.f_len
                )
                # For debugging
                print(f"state : {self.context.state}, result : {result}")
                if result:
                    self.context.set_state(State.MoveToPlace)
                    # Prepare for path planning
                    self.context.path_planning_state = True
            elif self.context.state == State.MoveToPlace:
                if self.context.state_counter == 1:
                    print(f"state: {self.context.state}")
                    self.set_joint_ctrl_mode(
                        self.wheels, self.sim.jointdynctrl_velocity
                    )
                if self.context.path_planning_state:
                    self.find_path(
                        self.read_youbot(),
                        config,
                        goal_id=self.context.place_location_id,
                    )
                result, control_data = move_to_place(
                    self.context, self.read_youbot(lidar=True)
                )
                if result:
                    self.path_data = None  # Init path data
                    self.context.set_state(State.PlaceTarget)
            elif self.context.state == State.PlaceTarget:
                if self.context.state_counter == 1:
                    print(f"state: {self.context.state}")
                    ControlData.control_cb = None  # Init the callback in ControlData
                    self.set_joint_ctrl_mode(
                        self.wheels, self.sim.jointdynctrl_position
                    )
                result, control_data = place_target(
                    self.context, self.read_youbot(camera=True)
                )
                if result:
                    self.context.set_state(State.MoveToBase)
                    # Prepare for path planning
                    self.context.path_planning_state = True
            elif self.context.state == State.MoveToBase:
                if self.context.state_counter == 1:
                    print(f"state: {self.context.state}")
                    self.set_joint_ctrl_mode(
                        self.wheels, self.sim.jointdynctrl_velocity
                    )
                if self.context.path_planning_state:
                    self.find_path(self.read_youbot(), config, goal_id=None)
                result, control_data = move_to_base(
                    self.context, self.read_youbot(lidar=True)
                )
                print(f"state: {self.context.state}, result : {result}")
                if result:
                    self.context.set_state(State.StandBy)
                    self.context.mission = None  # clear mission
                    self.run_flag = False  # try 1 loop

            # Run Simulation Step
            self.sim.step()

        # Stop Simulation
        self.sim.stopSimulation()


@app.route("/stop")
def stop():
    client.run_flag = False
    return


@app.route("/mission", methods=["POST"])
def mission():
    params = request.get_json()
    # set the pick & place locations
    client.init_mission(
        params["pick_location"], params["place_location"], params["target"]
    )
    return ""


if __name__ == "__main__":
    client = PickAndPlace()
    client.init_coppelia()

    # start flask web server
    threading.Thread(
        target=lambda: app.run(
            host="0.0.0.0", port=5555, debug=False, use_reloader=False
            # host="127.0.0.1", port=5555, debug=False, use_reloader=False
        ),
        daemon=True,
    ).start()

    # run coppeliaq
    client.run_coppelia()
