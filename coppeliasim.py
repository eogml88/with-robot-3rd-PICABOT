import numpy as np
import cv2

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from util import ReadData, ControlData


class Coppeliasim:
    def __init__(self):
        # coppeliasim simulation instance
        self.sim = RemoteAPIClient().require("sim")
        # ReadData of Robot
        self.read_data = ReadData()
        # ControlData of Robot
        self.control_data = ControlData()
        # Simulation run flag
        self.run_flag = True

        # reference dummy
        self.youBot_ref = self.sim.getObject("/youBot_ref")
        # Wheel joints
        self.joints = []
        self.joints.append(self.sim.getObject("/rollingJoint_fl"))
        self.joints.append(self.sim.getObject("/rollingJoint_rl"))
        self.joints.append(self.sim.getObject("/rollingJoint_fr"))
        self.joints.append(self.sim.getObject("/rollingJoint_rr"))
        # manipulator 5 joints
        for i in range(5):
            self.joints.append(self.sim.getObject(f"/youBotArmJoint{i}"))
        # Gripper Joint
        self.joints.append(self.sim.getObject(f"/youBotGripperJoint1"))
        self.joints.append(self.sim.getObject(f"/youBotGripperJoint2"))
        # camera
        self.camera_1 = self.sim.getObject(f"/camera_1")
        # lidar
        self.lidar = self.sim.getObjectHandle("/fastHokuyo")
        self.lidar_1 = self.sim.getObjectHandle("/fastHokuyo_sensor1")
        self.lidar_2 = self.sim.getObjectHandle("/fastHokuyo_sensor1")
        self.lidar_script = self.sim.getScript(
            self.sim.scripttype_childscript, self.lidar
        )
        # set joint control model
        self.set_joint_ctrl_mode(self.joints, self.sim.jointdynctrl_position)

    # set joints dynamic control mode
    def set_joint_ctrl_mode(self, objects, ctrl_mode):
        for obj in objects:
            self.sim.setObjectInt32Param(
                obj, self.sim.jointintparam_dynctrlmode, ctrl_mode
            )

    # read youbot data
    def read_youbot(self):
        # read localization of youbot
        p = self.sim.getObjectPosition(self.youBot_ref)
        o = self.sim.getObjectQuaternion(self.youBot_ref)
        self.read_data.localization = np.array(p + o)  # [x,y,z,qw,qx,qy,qz]

        p1 = self.sim.getObjectPosition(self.lidar_1)
        p2 = self.sim.getObjectPosition(self.lidar_2)
        self.read_data.scan_position = (np.array(p1[:2]), np.array(p2[:2]))

        # read manipulator joints
        joints = []
        for joint in self.joints:
            theta = self.sim.getJointPosition(joint)
            joints.append(theta)
        self.read_data.joints = np.array(joints)

        # read camera
        if self.read_data.img_flag:
            result = self.sim.getVisionSensorImg(self.camera_1)
            img = np.frombuffer(result[0], dtype=np.uint8)
            img = img.reshape((result[1][1], result[1][0], 3))
            img = cv2.flip(img, 0)
            self.read_data.img = img
        else:
            self.read_data.img = None

        # read lidar
        if self.read_data.scan_flg:
            data = self.sim.callScriptFunction("getMeasuredData", self.lidar_script)
            if data is not None and len(data) > 0:
                self.read_data.scan = np.array(data)
            else:
                self.read_data.scan = None
        else:
            self.read_data.scan = None

    # control youbot
    def control_youbot(self):
        if self.control_data.wheels_position is not None:
            for i, wheel in enumerate(self.control_data.wheels_position):
                index = i
                diff = abs(wheel - self.read_data.joints[index])
                diff = min(diff, np.pi)
                if self.read_data.joints[index] < wheel:
                    target = self.read_data.joints[index] + diff
                else:
                    target = self.read_data.joints[index] - diff
                self.sim.setJointTargetPosition(self.joints[index], target)

    def run(self, callback):
        # start simulation
        self.sim.setStepping(True)
        self.sim.startSimulation()

        # execution of the simulation
        while self.run_flag:
            # read youbot data
            self.read_youbot()
            # callback
            callback(self.read_data, self.control_data)
            # control youbot
            self.control_youbot()
            # Run Simulation Step
            self.sim.step()

        # Stop Simulation
        self.sim.stopSimulation()
