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
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 |                      | _Free  |  7 |  6 |     0 |    0
  1 | front_right_wheel_rolling_joint | _Hinge |  1 |  1 |     7 |    6
  2 | front_right_slipping_0_joint | _Hinge |  1 |  1 |     8 |    7
  3 | front_right_slipping_1_joint | _Hinge |  1 |  1 |     9 |    8
  4 | front_right_slipping_2_joint | _Hinge |  1 |  1 |    10 |    9
  5 | front_right_slipping_3_joint | _Hinge |  1 |  1 |    11 |   10
  6 | front_right_slipping_4_joint | _Hinge |  1 |  1 |    12 |   11
  7 | front_right_slipping_5_joint | _Hinge |  1 |  1 |    13 |   12
  8 | front_right_slipping_6_joint | _Hinge |  1 |  1 |    14 |   13
  9 | front_right_slipping_7_joint | _Hinge |  1 |  1 |    15 |   14
 10 | front_right_slipping_8_joint | _Hinge |  1 |  1 |    16 |   15
 11 | front_right_slipping_9_joint | _Hinge |  1 |  1 |    17 |   16
 12 | front_right_slipping_10_joint | _Hinge |  1 |  1 |    18 |   17
 13 | front_right_slipping_11_joint | _Hinge |  1 |  1 |    19 |   18
 14 | front_left_wheel_rolling_joint | _Hinge |  1 |  1 |    20 |   19
 15 | front_left_slipping_0_joint | _Hinge |  1 |  1 |    21 |   20
 16 | front_left_slipping_1_joint | _Hinge |  1 |  1 |    22 |   21
 17 | front_left_slipping_2_joint | _Hinge |  1 |  1 |    23 |   22
 18 | front_left_slipping_3_joint | _Hinge |  1 |  1 |    24 |   23
 19 | front_left_slipping_4_joint | _Hinge |  1 |  1 |    25 |   24
 20 | front_left_slipping_5_joint | _Hinge |  1 |  1 |    26 |   25
 21 | front_left_slipping_6_joint | _Hinge |  1 |  1 |    27 |   26
 22 | front_left_slipping_7_joint | _Hinge |  1 |  1 |    28 |   27
 23 | front_left_slipping_8_joint | _Hinge |  1 |  1 |    29 |   28
 24 | front_left_slipping_9_joint | _Hinge |  1 |  1 |    30 |   29
 25 | front_left_slipping_10_joint | _Hinge |  1 |  1 |    31 |   30
 26 | front_left_slipping_11_joint | _Hinge |  1 |  1 |    32 |   31
 27 | rear_right_wheel_rolling_joint | _Hinge |  1 |  1 |    33 |   32
 28 | rear_right_slipping_0_joint | _Hinge |  1 |  1 |    34 |   33
 29 | rear_right_slipping_1_joint | _Hinge |  1 |  1 |    35 |   34
 30 | rear_right_slipping_2_joint | _Hinge |  1 |  1 |    36 |   35
 31 | rear_right_slipping_3_joint | _Hinge |  1 |  1 |    37 |   36
 32 | rear_right_slipping_4_joint | _Hinge |  1 |  1 |    38 |   37
 33 | rear_right_slipping_5_joint | _Hinge |  1 |  1 |    39 |   38
 34 | rear_right_slipping_6_joint | _Hinge |  1 |  1 |    40 |   39
 35 | rear_right_slipping_7_joint | _Hinge |  1 |  1 |    41 |   40
 36 | rear_right_slipping_8_joint | _Hinge |  1 |  1 |    42 |   41
 37 | rear_right_slipping_9_joint | _Hinge |  1 |  1 |    43 |   42
 38 | rear_right_slipping_10_joint | _Hinge |  1 |  1 |    44 |   43
 39 | rear_right_slipping_11_joint | _Hinge |  1 |  1 |    45 |   44
 40 | rear_left_wheel_rolling_joint | _Hinge |  1 |  1 |    46 |   45
 41 | rear_left_slipping_0_joint | _Hinge |  1 |  1 |    47 |   46
 42 | rear_left_slipping_1_joint | _Hinge |  1 |  1 |    48 |   47
 43 | rear_left_slipping_2_joint | _Hinge |  1 |  1 |    49 |   48
 44 | rear_left_slipping_3_joint | _Hinge |  1 |  1 |    50 |   49
 45 | rear_left_slipping_4_joint | _Hinge |  1 |  1 |    51 |   50
 46 | rear_left_slipping_5_joint | _Hinge |  1 |  1 |    52 |   51
 47 | rear_left_slipping_6_joint | _Hinge |  1 |  1 |    53 |   52
 48 | rear_left_slipping_7_joint | _Hinge |  1 |  1 |    54 |   53
 49 | rear_left_slipping_8_joint | _Hinge |  1 |  1 |    55 |   54
 50 | rear_left_slipping_9_joint | _Hinge |  1 |  1 |    56 |   55
 51 | rear_left_slipping_10_joint | _Hinge |  1 |  1 |    57 |   56
 52 | rear_left_slipping_11_joint | _Hinge |  1 |  1 |    58 |   57

 id | name                            | trn     | target_joint
----+---------------------------------+---------+-------------
  0 | front_right_wheel               | _Joint  | front_right_wheel_rolling_joint
  1 | front_left_wheel                | _Joint  | front_left_wheel_rolling_joint
  2 | rear_right_wheel                | _Joint  | rear_right_wheel_rolling_joint
  3 | rear_left_wheel                 | _Joint  | rear_left_wheel_rolling_joint

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:xls_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:xls_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:xls_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:xls_site


"""
class XLSController(ControllerInterface):
    
    def __init__(self, node: Node, dt: float, mj_joint_dict: list):
        super().__init__(node, dt, mj_joint_dict)
                
        self.key_sub = self.node.create_subscription(Int32, 'xls_controller/mode_input', self.keyCallback, 10)
        self.target_pose_sub = self.node.create_subscription(Pose, 'xls_controller/target_pose', self.targetPoseCallback, 10)
        self.target_vel_sub = self.node.create_subscription(Twist, 'xls_controller/cmd_vel', self.targetVelocityCallback, 1)
        
        self.base_pose_pub = self.node.create_publisher(Pose, 'xls_controller/base_pose', 10)
        self.base_vel_pub = self.node.create_publisher(Twist, 'xls_controller/base_vel', 10)
        
        self.base_vel = np.zeros(3)          # [vx, vy, w] wrt base frame
        self.base_vel_desired = np.zeros(3)  # [vx, vy, w] wrt base frame
        self.base_vel_init = np.zeros(3)     # [vx, vy, w] wrt base frame
        
        self.base_pose = np.zeros(3)         # [x, y, theta] wrt world frame
        self.base_pose_desired = np.zeros(3) # [x, y, theta] wrt world frame
        self.base_pose_init = np.zeros(3)    # [x, y, theta] wrt world frame
        
        self.wheel_vel = np.zeros(4)          # [front_right, front_left, rear_right, rear_left]
        self.wheel_vel_desired = np.zeros(4)  # [front_right, front_left, rear_right, rear_left]
        self.wheel_vel_init = np.zeros(4)     # [front_right, front_left, rear_right, rear_left]
               
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
        
        self.wheel_vel[0] = vel_dict['front_right_wheel_rolling_joint']
        self.wheel_vel[1] = vel_dict['front_left_wheel_rolling_joint']
        self.wheel_vel[2] = vel_dict['rear_right_wheel_rolling_joint']
        self.wheel_vel[3] = vel_dict['rear_left_wheel_rolling_joint']
        
        self.base_pose[:2] = sensor_dict['position_sensor'][:2]  # [x, y]
        self.base_pose[2] = R.from_quat(sensor_dict["orientation_sensor"], scalar_first=True).as_euler('zyx', degrees=False)[2] # theta
        
        ### By FK
        self.base_vel = self.FK(self.wheel_vel)
        
        ### By sensor
        # tmp_base_vel = sensor_dict['linear_velocity_sensor'][:2] # wrt world frame
        # tmp_base_vel = np.array([[np.cos(self.base_pose[2]), -np.sin(self.base_pose[2])],
        #                          [np.sin(self.base_pose[2]),  np.cos(self.base_pose[2])]]).dot(tmp_base_vel) # Convert to base frame
        # self.base_vel[:2] = tmp_base_vel[:2]
        # self.base_vel[2] = sensor_dict['angular_velocity_sensor'][2]
        
    def compute(self) -> None:
        if self.is_mode_changed:
            self.base_pose_init = self.base_pose.copy()
            self.base_vel_init = self.base_vel.copy()
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
            self.wheel_vel_desired = np.zeros(4)
    
    def getCtrlInput(self) -> dict:
        ctrl_torque = self.VelocityDControl(self.wheel_vel_desired, self.wheel_vel)
        ctrl_dict = {"front_right_wheel": ctrl_torque[0],
                     "front_left_wheel":  ctrl_torque[1],
                     "rear_right_wheel":  ctrl_torque[2],
                     "rear_left_wheel":   ctrl_torque[3]}
        return ctrl_dict

    # =============================================================================================
    def keyCallback(self, msg: Int32):
        self.node.get_logger().info(f"[XLSController] Key input received: {msg.data}")
        if msg.data == 1:
            self.setMode('stop')
        elif msg.data == 2:
            self.setMode('base_vel_tracking')
        elif msg.data == 3:
            self.setMode('base_pose_tracking')
                    
    def setMode(self, mode: str):
        self.is_mode_changed = True
        self.node.get_logger().info(f"[XLSController] Mode changed: {mode}")
        self.mode = mode
        
    def IK(self, target_vel: np.ndarray) -> np.ndarray:
        """Inverse Kinematics calculation.

        Args:
            target_vel (np.ndarray): target linear and angular velocity [vx, xy, w]

        Returns:
            np.ndarray: desired wheel angular velocities [w_front_right, w_front_left, w_rear_right, w_rear_left]
        """
        W = 0.2045*2  # Distance between right and left wheels
        H = 0.2225*2  # Distance between front and rear wheels
        R = 0.120     # Radius of wheels
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
        
        J = np.array([[1.0,  1.0,  (W+H)/2.0],
                      [1.0, -1.0, -(W+H)/2.0],
                      [1.0, -1.0,  (W+H)/2.0],
                      [1.0,  1.0, -(W+H)/2.0]]) / R

        w = J @ sat_target_vel  # Calculate wheel angular velocities
        return w
    
    def FK(self, wheel_vel: np.ndarray) -> np.ndarray:
        """Forward Kinematics calculation.

        Args:
            wheel_vel (np.ndarray): wheel angular velocities [w_front_right, w_front_left, w_rear_right, w_rear_left]

        Returns:
            np.ndarray: base linear and angular velocity [vx, vy, w]
        """
        W = 0.2045*2  # Distance between right and left wheels
        H = 0.2225*2  # Distance between front and rear wheels
        R = 0.120     # Radius of wheels
        
        J_inv = np.array([[1.0,        1.0,        1.0,       1.0],
                          [1.0, -1.0, -1.0,  1.0],
                          [2.0/(H+W), -2.0/(H+W), 2.0/(H+W), -2.0/(H+W)]]) * R / 4.0
        vel = J_inv @ wheel_vel  # Calculate base linear and angular velocity
        return vel

    def VelocityDControl(self, target_vel: np.ndarray, current_vel: np.ndarray, kd: float = 140) -> np.ndarray:
        """Velocity-D control for the base.

        Args:
            target_vel (np.ndarray): target wheel velocity [w_front_right, w_front_left, w_rear_right, w_rear_left]
            current_vel (np.ndarray): current wheel velocity [w_front_right, w_front_left, w_rear_right, w_rear_left]
            kd (float): derivative gain

        Returns:
            np.ndarray: control input for the wheels [w_front_right, w_front_left, w_rear_right, w_rear_left]
        """
        control_input = kd * (target_vel - current_vel)
        return control_input

    def targetPoseCallback(self, msg: Pose):
        self.node.get_logger().info(f"[XLSController] Target pose received: {msg}")
        
        # Convert the Pose message to a 4x4 transformation matrix
        posi = np.array([msg.position.x, msg.position.y, msg.position.z])
        quat = [msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w]

        self.base_pose_desired[:2] = posi[:2]  # [x, y]
        self.base_pose_desired[2] = R.from_quat(quat, scalar_first=True).as_euler('zyx', degrees=False)[2]  # theta
        
    def targetVelocityCallback(self, msg: Twist):
        self.node.get_logger().info(f"[XLSController] Target velocity received: {msg}")
        
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
        
