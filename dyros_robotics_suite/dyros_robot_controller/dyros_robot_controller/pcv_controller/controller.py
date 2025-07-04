import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import threading
from scipy.spatial.transform import Rotation as R
import threading
import time
from typing import Tuple
import math

from dyros_robot_controller.utils import cubic_spline, cubic_dot_spline

from mujoco_ros_sim import ControllerInterface

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


"""
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 |                      | _Free  |  7 |  6 |     0 |    0
  1 | front_left_steer     | _Hinge |  1 |  1 |     7 |    6
  2 | front_left_rotate    | _Hinge |  1 |  1 |     8 |    7
  3 | rear_left_steer      | _Hinge |  1 |  1 |     9 |    8
  4 | rear_left_rotate     | _Hinge |  1 |  1 |    10 |    9
  5 | rear_right_steer     | _Hinge |  1 |  1 |    11 |   10
  6 | rear_right_rotate    | _Hinge |  1 |  1 |    12 |   11
  7 | front_right_steer    | _Hinge |  1 |  1 |    13 |   12
  8 | front_right_rotate   | _Hinge |  1 |  1 |    14 |   13

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_left_steer     | _Joint  | front_left_steer
  1 | front_left_rotate    | _Joint  | front_left_rotate
  2 | front_right_steer    | _Joint  | front_right_steer
  3 | front_right_rotate   | _Joint  | front_right_rotate
  4 | rear_left_steer      | _Joint  | rear_left_steer
  5 | rear_left_rotate     | _Joint  | rear_left_rotate
  6 | rear_right_steer     | _Joint  | rear_right_steer
  7 | rear_right_rotate    | _Joint  | rear_right_rotate

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:dyros_pcv_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:dyros_pcv_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:dyros_pcv_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:dyros_pcv_site
"""
class PCVController(ControllerInterface):
    
    def __init__(self, node: Node, dt: float, mj_joint_dict: list):
        super().__init__(node, dt, mj_joint_dict)
                
        self.key_sub = self.node.create_subscription(Int32, 'pcv_controller/mode_input', self.keyCallback, 10)
        self.target_pose_sub = self.node.create_subscription(Pose, 'pcv_controller/target_pose', self.targetPoseCallback, 10)
        self.target_vel_sub = self.node.create_subscription(Twist, 'pcv_controller/cmd_vel', self.targetVelocityCallback, 1)
        
        self.base_pose_pub = self.node.create_publisher(Pose, 'pcv_controller/base_pose', 10)
        self.base_vel_pub = self.node.create_publisher(Twist, 'pcv_controller/base_vel', 10)
        
        self.base_vel = np.zeros(3)          # [vx, vy, w] wrt base frame
        self.base_vel_desired = np.zeros(3)  # [vx, vy, w] wrt base frame
        self.base_vel_init = np.zeros(3)     # [vx, vy, w] wrt base frame
        
        self.base_pose = np.zeros(3)         # [x, y, theta] wrt world frame
        self.base_pose_desired = np.zeros(3) # [x, y, theta] wrt world frame
        self.base_pose_init = np.zeros(3)    # [x, y, theta] wrt world frame

        self.steer_wheel_pos = np.zeros(4) # [front_right_steer, front_left_steer, rear_right_steer, rear_left_steer]
        self.wheel_vel = np.zeros(8)       # [front_right_steer, front_right_rotate, 
                                           #  front_left_steer,  front_left_rotate, 
                                           #  rear_right_steer,  rear_right_rotate, 
                                           #  rear_left_steer,   rear_left_rotate]
        self.wheel_vel_desired = np.zeros(8)
        self.wheel_vel_init = np.zeros(8)
       
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
        
        self.wheel_vel[0] = vel_dict['front_right_steer']
        self.wheel_vel[1] = vel_dict['front_right_rotate']
        self.wheel_vel[2] = vel_dict['front_left_steer']
        self.wheel_vel[3] = vel_dict['front_left_rotate']
        self.wheel_vel[4] = vel_dict['rear_right_steer']
        self.wheel_vel[5] = vel_dict['rear_right_rotate']
        self.wheel_vel[6] = vel_dict['rear_left_steer']
        self.wheel_vel[7] = vel_dict['rear_left_rotate']
        
        self.steer_wheel_pos[0] = pos_dict['front_right_steer']
        self.steer_wheel_pos[1] = pos_dict['front_left_steer']
        self.steer_wheel_pos[2] = pos_dict['rear_right_steer']
        self.steer_wheel_pos[3] = pos_dict['rear_left_steer']
        
        self.base_pose[:2] = sensor_dict['position_sensor'][:2]  # [x, y]
        self.base_pose[2] = R.from_quat(sensor_dict["orientation_sensor"], scalar_first=True).as_euler('zyx', degrees=False)[2] # theta
        
        ### By FK
        # self.base_vel = self.FK(self.wheel_vel, self.steer_wheel_vel)
        
        ### By sensor
        tmp_base_vel = sensor_dict['linear_velocity_sensor'][:2] # wrt world frame
        tmp_base_vel = np.array([[np.cos(self.base_pose[2]), -np.sin(self.base_pose[2])],
                                 [np.sin(self.base_pose[2]),  np.cos(self.base_pose[2])]]).dot(tmp_base_vel) # Convert to base frame
        self.base_vel[:2] = tmp_base_vel[:2]
        self.base_vel[2] = sensor_dict['angular_velocity_sensor'][2]
        
    def compute(self) -> None:
        if self.is_mode_changed:
            self.base_pose_init = self.base_pose.copy()
            self.base_vel_init = self.base_vel.copy()
            self.wheel_vel_init = self.wheel_vel.copy()
            self.wheel_vel_init = self.wheel_vel.copy()
            self.control_start_time = self.current_time
            self.is_mode_changed = False
            
        # Compute desired joint positions based on the current control mode.
        if self.mode == 'stop':
            base_vel_target = np.zeros(3)
            self.wheel_vel_desired = self.IK(base_vel_target)
        elif self.mode == 'base_vel_tracking':
            self.wheel_vel_desired = self.IK(self.base_vel_desired)
        elif self.mode == 'base_pose_tracking':
            # TODO: kanayama
            pass
        else:
            self.wheel_vel_desired = np.zeros(8)
    def getCtrlInput(self) -> dict:
        ctrl_dict = {"front_right_steer":  self.wheel_vel_desired[0],
                     "front_right_rotate": self.wheel_vel_desired[1],
                     "front_left_steer":   self.wheel_vel_desired[2],
                     "front_left_rotate":  self.wheel_vel_desired[3],
                     "rear_right_steer":   self.wheel_vel_desired[4],
                     "rear_right_rotate":  self.wheel_vel_desired[5],
                     "rear_left_steer":    self.wheel_vel_desired[6],
                     "rear_left_rotate":   self.wheel_vel_desired[7]}
        return ctrl_dict

    # =============================================================================================
    def keyCallback(self, msg: Int32):
        self.node.get_logger().info(f"[PCVController] Key input received: {msg.data}")
        if msg.data == 1:
            self.setMode('stop')
        elif msg.data == 2:
            self.setMode('base_vel_tracking')
        elif msg.data == 3:
            self.setMode('base_pose_tracking')
                    
    def setMode(self, mode: str):
        self.is_mode_changed = True
        self.node.get_logger().info(f"[PCVController] Mode changed: {mode}")
        self.mode = mode
    def IK(self, target_vel: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Inverse Kinematics calculation.

        Args:
            target_vel (np.ndarray): target linear and angular velocity [vx, xy, w]

        Returns:
            Tuple[np.ndarray, np.ndarray]: desired wheel angular velocities [front_right, front_left, rear_right, rear_left]
                                             and desired steering wheel velocities [front_right, front_left, rear_right, rear_left]
        """
        # A Powered-Caster Holonomic Robotic Vehicle for Mobile Manipulation Tasks

        W = 0.125*2.0  # Distance between right and left wheels
        H = 0.215*2.0  # Distance between front and rear wheels
        R = 0.055      # Radius of wheels
        b = 0.020      # Distance of offset between the wheel center and the caster point
        base_vel_lim = np.array([1.5, 2.0])  # [linear_vel, angular_vel]
        base_acc_lim = np.array([1.0, 1.0])  # [linear_acc, angular_acc]
        
        vx = target_vel[0]
        vy = target_vel[1]
        v = np.sqrt(vx**2 + vy**2)
        v_dir = target_vel[:2] / v if v > 1E-6 else np.array([0.0, 0.0])  # Direction of linear velocity
        w = target_vel[2]
        
        current_v = np.sqrt(self.base_vel[0]**2 + self.base_vel[1]**2)
        current_w = self.base_vel[2]
        
        # Limit the linear and angular velocities
        v = np.clip(v, -base_vel_lim[0], base_vel_lim[0])
        w = np.clip(w, -base_vel_lim[1], base_vel_lim[1])
        
        # TODO: Limit the linear and angular accelerations 
        # # Limit the linear and angular accelerations
        # v = np.clip((v - current_v) / self.dt, -base_acc_lim[0], base_acc_lim[0]) * self.dt + current_v
        # w = np.clip((w - current_w) / self.dt, -base_acc_lim[1], base_acc_lim[1]) * self.dt + current_w
        
        # Calculate the desired wheel angular velocities
        sat_target_vel = np.array([v * v_dir[0], v * v_dir[1], w])
        
        desired_wheel_vel = np.zeros(8)
        
        for i in range(4):
            h = math.sqrt((W/2.0)**2 + (H/2.0)**2)
            phi = self.steer_wheel_pos[i]  # Steering wheel position
            
            if i==0: # Front Right
                beta = math.atan2(-W/2.0, H/2.0)
            elif i==1: # Front Left
                beta = math.atan2(W/2.0, H/2.0)
            elif i==2: # Rear Right
                beta = math.atan2(-W/2.0, -H/2.0)
            elif i==3: # Rear Left
                beta = math.atan2(W/2.0, -H/2.0)

            J_tilda = np.array([[-np.sin(phi) / b, np.cos(phi) / b, h * (np.cos(beta)*np.cos(phi) + np.sin(beta * np.sin(phi))) / b - 1.0],
                                [ np.cos(phi) / R, np.sin(phi) / R, h * (np.cos(beta)*np.sin(phi) - np.sin(beta) * np.cos(phi)) / R]])
            desired_wheel_vel[i*2:i*2+2] = J_tilda @ sat_target_vel  # Calculate steering wheel angular velocities

        return desired_wheel_vel
    
    def FK(self, wheel_vel: np.ndarray) -> np.ndarray:
        """Forward Kinematics calculation.

        Args:
            wheel_vel (np.ndarray): wheel angular velocities [front_right, front_left, rear_right, rear_left]

        Returns:
            np.ndarray: base linear and angular velocity [vx, vy, w]
        """
        # TODO
        pass
    def targetPoseCallback(self, msg: Pose):
        self.node.get_logger().info(f"[PCVController] Target pose received: {msg}")
        
        # Convert the Pose message to a 4x4 transformation matrix
        posi = np.array([msg.position.x, msg.position.y, msg.position.z])
        quat = [msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w]

        self.base_pose_desired[:2] = posi[:2]  # [x, y]
        self.base_pose_desired[2] = R.from_quat(quat, scalar_first=True).as_euler('zyx', degrees=False)[2]  # theta
        
    def targetVelocityCallback(self, msg: Twist):        
        # Convert the Twist message to a 2D velocity vector
        linear_vel_x = msg.linear.x
        linear_vel_y = msg.linear.y
        angular_vel = msg.angular.z

        self.base_vel_desired = np.array([linear_vel_x, linear_vel_y, angular_vel])
        
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
        base_vel_msg.linear.y = self.base_vel[1]
        base_vel_msg.angular.z = self.base_vel[2]
        
        self.base_vel_pub.publish(base_vel_msg)
        
