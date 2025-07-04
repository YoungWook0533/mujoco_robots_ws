import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import threading
from scipy.spatial.transform import Rotation as R
import threading
import time

from dyros_robot_controller.utils import cubic_spline, cubic_dot_spline

from mujoco_ros_sim import ControllerInterface

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


"""
Husky MuJoCo Joint/Sensor Imformation
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 |                      | _Free  |  7 |  6 |     0 |    0
  1 | front_left_wheel     | _Hinge |  1 |  1 |     7 |    6
  2 | front_right_wheel    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_wheel      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_right_wheel     | _Hinge |  1 |  1 |    10 |    9

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | left_wheel           | _Joint  | front_left_wheel
  1 | right_wheel          | _Joint  | front_right_wheel

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:husky_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:husky_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:husky_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:husky_site

"""
class HuskyController(ControllerInterface):
    
    def __init__(self, node: Node, dt: float, mj_joint_dict: list):
        super().__init__(node, dt, mj_joint_dict)
                
        self.key_sub = self.node.create_subscription(Int32, 'husky_controller/mode_input', self.keyCallback, 10)
        self.target_pose_sub = self.node.create_subscription(Pose, 'husky_controller/target_pose', self.targetPoseCallback, 10)
        self.target_vel_sub = self.node.create_subscription(Twist, 'husky_controller/cmd_vel', self.targetVelocityCallback, 1)
        
        self.base_pose_pub = self.node.create_publisher(Pose, 'husky_controller/base_pose', 10)
        self.base_vel_pub = self.node.create_publisher(Twist, 'husky_controller/base_vel', 10)
        
        self.base_vel = np.zeros(2)          # [v, w] wrt base frame
        self.base_vel_desired = np.zeros(2)  # [v, w] wrt base frame
        self.base_vel_init = np.zeros(2)     # [v, w] wrt base frame
        
        self.base_pose = np.zeros(3)         # [x, y, theta] wrt world frame
        self.base_pose_desired = np.zeros(3) # [x, y, theta] wrt world frame
        self.base_pose_init = np.zeros(3)    # [x, y, theta] wrt world frame
        
        self.wheel_vel = np.zeros(2)          # [w_left, w_right]
        self.wheel_vel_desired = np.zeros(2)  # [w_left, w_right]
        self.wheel_vel_init = np.zeros(2)     # [w_left, w_right]
               
    def starting(self) -> None:
        self.is_mode_changed = False
        self.mode = 'stop'
        self.control_start_time = self.current_time
        
        self.base_vel_desired = self.base_vel.copy()
        self.base_pose_desired = self.base_pose.copy()
        self.wheel_vel_desired = self.wheel_vel.copy()
        
        self.node.create_timer(0.01, self.pubBasePoseCallback)
        self.node.create_timer(0.01, self.pubBaseVelCallback)
                
    def updateState(self, 
                    pos_dict: dict, 
                    vel_dict: dict, 
                    tau_ext_dict: dict,
                    sensor_dict: dict, 
                    current_time: float) -> None:
        
        self.current_time = current_time
        
        self.wheel_vel[0] = vel_dict['front_left_wheel']
        self.wheel_vel[1] = vel_dict['front_right_wheel']
        
        self.base_pose[:2] = sensor_dict['position_sensor'][:2]  # [x, y]
        self.base_pose[2] = R.from_quat(sensor_dict["orientation_sensor"], scalar_first=True).as_euler('zyx', degrees=False)[2] # theta
        
        ### By FK
        self.base_vel = self.FK(self.wheel_vel)
        
        ### By sensor
        # tmp_base_vel = sensor_dict['linear_velocity_sensor'][:2] # wrt world frame
        # tmp_base_vel = np.array([[np.cos(self.base_pose[2]), -np.sin(self.base_pose[2])],
        #                          [np.sin(self.base_pose[2]),  np.cos(self.base_pose[2])]]).dot(tmp_base_vel) # Convert to base frame
        # self.base_vel[0] = tmp_base_vel[0]
        # self.base_vel[1] = sensor_dict['angular_velocity_sensor'][2]
        

    def compute(self) -> None:
        if self.is_mode_changed:
            self.base_pose_init = self.base_pose.copy()
            self.base_vel_init = self.base_vel.copy()
            self.wheel_vel_init = self.wheel_vel.copy()
            self.control_start_time = self.current_time
            self.is_mode_changed = False
            
        # Compute desired joint positions based on the current control mode.
        if self.mode == 'stop':
            base_vel_target = np.zeros(2)
            self.wheel_vel_desired = self.IK(base_vel_target)
        elif self.mode == 'base_vel_tracking':
            self.wheel_vel_desired = self.IK(self.base_vel_desired)
        elif self.mode == 'base_pose_tracking':
            self.wheel_vel_desired = self.Kanayama(self.base_pose_desired)
        else:
            self.wheel_vel_desired = np.zeros(2)
    
    def getCtrlInput(self) -> dict:
        ctrl_dict = {"left_wheel": self.wheel_vel_desired[0],
                     "right_wheel": self.wheel_vel_desired[1]}
        return ctrl_dict
    
    # =============================================================================================
    def keyCallback(self, msg: Int32):
        self.node.get_logger().info(f"[HuskyController] Key input received: {msg.data}")
        if msg.data == 1:
            self.setMode('stop')
        elif msg.data == 2:
            self.setMode('base_vel_tracking')
        elif msg.data == 3:
            self.setMode('base_pose_tracking')
                    
    def setMode(self, mode: str):
        self.is_mode_changed = True
        self.node.get_logger().info(f"[HuskyController] Mode changed: {mode}")
        self.mode = mode
        
    def IK(self, target_vel: np.ndarray) -> np.ndarray:
        """Inverse Kinematics calculation.

        Args:
            target_vel (np.ndarray): target linear and angular velocity [v, w]

        Returns:
            np.ndarray: desired wheel angular velocities [w_left, w_right]
        """
        L = 0.2854*2*1.875  # Distance between wheels
        R = 0.1651    # radius of wheels
        base_vel_lim = np.array([1.0, 3.0])  # [linear_vel, angular_vel]
        base_acc_lim = np.array([2.0, 6.0])  # [linear_acc, angular_acc]
        
        v = target_vel[0]
        w = target_vel[1]
        
        # Limit the linear and angular velocities
        v = np.clip(target_vel[0], -base_vel_lim[0], base_vel_lim[0])
        w = np.clip(target_vel[1], -base_vel_lim[1], base_vel_lim[1])
        
        # TODO: Limit the linear and angular accelerations 
        # # Limit the linear and angular accelerations
        # v = np.clip((v - self.base_vel[0]) / self.dt, -base_acc_lim[0], base_acc_lim[0]) * self.dt + self.base_vel[0]
        # w = np.clip((w - self.base_vel[1]) / self.dt, -base_acc_lim[1], base_acc_lim[1]) * self.dt + self.base_vel[1]
        # self.node.get_logger().info(f"{(v - self.base_vel[0]) / self.dt}")
        
        w_left = (v - L/2 * w) / R
        w_right = (v + L/2 * w) / R
        return np.array([w_left, w_right])
    
    def FK(self, wheel_vel: np.ndarray) -> np.ndarray:
        """Forward Kinematics calculation.

        Args:
            wheel_vel (np.ndarray): wheel angular velocities [w_left, w_right]

        Returns:
            np.ndarray: base linear and angular velocity [v, w]
        """
        w_left, w_right = wheel_vel
        L = 0.2854*2*1.875  # Distance between wheels
        R = 0.1651  # radius of wheels
        v = (w_left + w_right) * R / 2.0
        w = (w_right - w_left) * R / L
        return np.array([v, w])
    
    def Kanayama(self, goal_pose: np.ndarray) -> np.ndarray:
        pose_error = goal_pose - self.base_pose
        pose_error = np.array([[ np.cos(self.base_pose[2]), np.sin(self.base_pose[2]), 0],
                               [-np.sin(self.base_pose[2]), np.cos(self.base_pose[2]), 0],
                               [0                         , 0,                         1]]) @ pose_error
        Kx, Ky, Ke = 100, 1, 10
        base_vel_desired = np.zeros(2)
        base_vel_desired[0] = Kx * pose_error[0]
        base_vel_desired[1] = Ke * np.sin(pose_error[2])
        
        return base_vel_desired

    def targetPoseCallback(self, msg: Pose):
        self.node.get_logger().info(f"[HuskyController] Target pose received: {msg}")
        
        posi = np.array([msg.position.x, msg.position.y, msg.position.z])
        quat = [msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w]

        self.base_pose_desired[:2] = posi[:2]  # [x, y]
        self.base_pose_desired[2] = R.from_quat(quat, scalar_first=True).as_euler('zyx', degrees=False)[2]  # theta
        
    def targetVelocityCallback(self, msg: Twist):        
        # Convert the Twist message to a 2D velocity vector
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        self.base_vel_desired = np.array([linear_vel, angular_vel])
        
    def pubBasePoseCallback(self):
        # Publish base pose
        base_pose_msg = Pose()
        base_pose_msg.position.x = self.base_pose[0]
        base_pose_msg.position.y = self.base_pose[1]
        
        quat = R.from_euler('z', self.base_pose[2], degrees=False).as_quat(scalar_first=True)
        base_pose_msg.orientation.x = quat[0]
        base_pose_msg.orientation.y = quat[1]
        base_pose_msg.orientation.z = quat[2]
        base_pose_msg.orientation.w = quat[3]
        
        self.base_pose_pub.publish(base_pose_msg)
        
    def pubBaseVelCallback(self):
        # Publish base velocity
        base_vel_msg = Twist()
        base_vel_msg.linear.x = self.base_vel[0]
        base_vel_msg.angular.z = self.base_vel[1]
        
        self.base_vel_pub.publish(base_vel_msg)
        
