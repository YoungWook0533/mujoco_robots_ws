import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import threading
from scipy.spatial.transform import Rotation as R
import threading
import time

from dyros_robot_controller.utils import cubic_spline, cubic_dot_spline, rotation_cubic, rotation_cubic_dot, get_phi

from mujoco_ros_sim import ControllerInterface

from .robot_data import PiperRobotData

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

class PiperController(ControllerInterface):

    def __init__(self, node: Node, dt: float, mj_joint_dict: list):
        super().__init__(node, dt, mj_joint_dict)

        self.robot_data = PiperRobotData(self.node, mj_joint_dict["joint_names"])

        self.key_sub = self.node.create_subscription(Int32, 'piper_controller/mode_input', self.keyCallback, 10)
        # self.target_pose_sub = self.node.create_subscription(PiperRobotData, 'piper_controller/target_pose', self.targetPoseCallback, 10)

    def starting(self) -> None:
        self.is_mode_changed = False
        self.mode = 'stop'
        self.control_start_time = self.current_time
        
        self.q_init = self.q
        self.qdot_init = self.qdot
        self.q_desired = self.q_init
        self.qdot_desired = np.zeros(6)
        
        self.x_init = self.x
        self.target_pose = self.x_init
        self.x_desired = self.x_init
        self.xdot_init = self.xdot
        self.xdot_desired = np.zeros(6)

        self.is_target_pose_changed = False

    def updateState(self, 
                    pos_dict: dict, 
                    vel_dict: dict, 
                    tau_ext_dict: dict,
                    sensor_dict: dict, 
                    current_time: float) -> None:
        
        self.current_time = current_time
        self.robot_data.updateState(pos_dict, vel_dict, tau_ext_dict, sensor_dict)
        
        self.q = self.robot_data.q
        self.qdot = self.robot_data.qdot
        self.x = self.robot_data.getPose()
        self.xdot = self.robot_data.getVelocity()
        
    def compute(self) -> None:
        if self.is_mode_changed:
            self.q_init = self.q
            self.qdot_init = self.qdot
            self.q_desired = self.q_init
            self.qdot_desired = np.zeros(7)
            self.x_init = self.x
            self.x_desired = self.x_init
            self.xdot_init = self.xdot
            self.xdot_desired = np.zeros(6)
            self.target_pose = self.x_init
            self.control_start_time = self.current_time
            self.is_mode_changed = False
                
        # Compute desired joint positions based on the current control mode.
        if self.mode == 'init':
            target_q = np.array([0, 0, 0, 0, 0, 0])
            self.q_desired = cubic_spline(
                self.current_time,              # Current time
                self.control_start_time,        # Start time of control
                self.control_start_time + 2.0,  # End time of interpolation (2 seconds later)
                self.q_init,                    # Initial joint positions
                target_q,                       # Target joint positions
                np.zeros(6),                    # Initial velocity (assumed zero)
                np.zeros(6)                     # Final velocity (assumed zero)
            )
        elif self.mode == 'home':
            target_q = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0])
            self.q_desired = cubic_spline(
                self.current_time,              # Current time
                self.control_start_time,        # Start time of control
                self.control_start_time + 2.0,  # End time of interpolation (2 seconds later)
                self.q_init,                    # Initial joint positions
                target_q,                       # Target joint positions
                np.zeros(6),                    # Initial velocity (assumed zero)
                np.zeros(6)                     # Final velocity (assumed zero)
            )
        # elif self.mode == 'CLIK':
        #     self.q_desired = self.CLIK(self.target_pose, duration=2.0)
        else:
            self.q_desired = self.q_init
    
    def getCtrlInput(self) -> dict:
        ctrl_dict = {}
        for i, joint_name in enumerate(self.robot_data.rd_joint_names):
            joint_name = joint_name.replace('_joint', '')  # Replace 'joint' with 'trn' in the joint name
            ctrl_dict[joint_name] = self.q_desired[i]
        return ctrl_dict
    
    # =============================================================================================
    def keyCallback(self, msg: Int32):
        self.node.get_logger().info(f"[PiperController] Key input received: {msg.data}")
        if msg.data == 1:
            self.setMode('init')
        elif msg.data == 2:
            self.setMode('home')
        elif msg.data == 3:
            self.setMode('RRTmove')
        elif msg.data == 4:
            self.setMode('CLIK')
                    
    def setMode(self, mode: str):
        self.is_mode_changed = True
        self.node.get_logger().info(f"[PiperController] Mode changed: {mode}")
        self.mode = mode

    # def targetPoseCallback(self, msg: PoseStamped):
    #     self.node.get_logger().info(f"[UR5eController] Target pose received: {msg}")
    #     self.is_target_pose_changed = True
    #     # Convert the Pose message to a 4x4 transformation matrix
    #     quat = [msg.pose.orientation.x,
    #             msg.pose.orientation.y,
    #             msg.pose.orientation.z,
    #             msg.pose.orientation.w]

    #     T = np.eye(4)
    #     T[:3, :3] = R.from_quat(quat).as_matrix()
    #     T[:3, 3] = [msg.pose.position.x,
    #                 msg.pose.position.y,
    #                 msg.pose.position.z]

    #     self.target_pose = T