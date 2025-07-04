import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import threading
from scipy.spatial.transform import Rotation as R
import threading
import time

from dyros_robot_controller.utils import cubic_spline, cubic_dot_spline, rotation_cubic, rotation_cubic_dot, get_phi

from mujoco_ros_sim import ControllerInterface

from .robot_data import UR5eRobotData

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from srmt2.planning_scene import PlanningScene
from srmt2.planner.rrt_connect import SuhanRRTConnect
from srmt2.kinematics import TRACIK

"""
UR5e MuJoCo Joint/Sensor Imformation
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | shoulder_pan_joint   | _Hinge |  1 |  1 |     0 |    0
  1 | shoulder_lift_joint  | _Hinge |  1 |  1 |     1 |    1
  2 | elbow_joint          | _Hinge |  1 |  1 |     2 |    2
  3 | wrist_1_joint        | _Hinge |  1 |  1 |     3 |    3
  4 | wrist_2_joint        | _Hinge |  1 |  1 |     4 |    4
  5 | wrist_3_joint        | _Hinge |  1 |  1 |     5 |    5

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | shoulder_pan         | _Joint  | shoulder_pan_joint
  1 | shoulder_lift        | _Joint  | shoulder_lift_joint
  2 | elbow                | _Joint  | elbow_joint
  3 | wrist_1              | _Joint  | wrist_1_joint
  4 | wrist_2              | _Joint  | wrist_2_joint
  5 | wrist_3              | _Joint  | wrist_3_joint

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------

"""
class UR5eController(ControllerInterface):
    
    def __init__(self, node: Node, dt: float, mj_joint_dict: list):
        super().__init__(node, dt, mj_joint_dict)
        
        self.robot_data = UR5eRobotData(self.node, mj_joint_dict["joint_names"])

        self.key_sub = self.node.create_subscription(Int32, 'ur5e_controller/mode_input', self.keyCallback, 10)
        self.target_pose_sub = self.node.create_subscription(PoseStamped, 'ur5e_controller/target_pose', self.targetPoseCallback, 10)

        self.EEpose_pub = self.node.create_publisher(PoseStamped, 'ur5e_controller/ee_pose', 10)

        self.RRTtraj_pub = self.node.create_publisher(PoseArray, 'ur5e_controller/rrt_path', 10)

        self._path_msg_sent = False

        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'robot',
            'ur5e.urdf'
        )
        srdf_path = os.path.join(
            get_package_share_directory('ur5e_moveit_config'),
            'config',
            'ur5e.srdf'
        )
        self.pc = PlanningScene(arm_names=["ur5e_arm"], 
                                arm_dofs=[6], 
                                base_link="base", 
                                node_name="ur5e_PlanningScene",
                                topic_name="/ur5e_planning_scene",
                                urdf_file_path=urdf_path,
                                srdf_file_path=srdf_path,)
        
        self.tracIK = TRACIK(base_link="base", 
                             tip_link="flange", 
                             urdf_file_path=urdf_path)
        
        self.joint_upper_limit = self.tracIK.get_upper_bound()
        self.joint_lower_limit = self.tracIK.get_lower_bound()
        
        self.rrt_planner = SuhanRRTConnect(state_dim=6, 
                                           lb=self.joint_lower_limit, 
                                           ub=self.joint_upper_limit, 
                                           validity_fn=self.pc.is_valid)
        
        self.compute_slow_thread = threading.Thread(target=self.computeSlow)
       
    def starting(self) -> None:
        self.is_mode_changed = False
        self.mode = 'home'
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
        self.is_rrt_path_found = False
        
        self.node.create_timer(0.01, self.pub_EEpose_callback)
        
        self.compute_slow_thread.start()
        
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
        
        self.pc.display(self.q)
        
        
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
        elif self.mode == 'RRTmove':
            self.q_desired = self.q_desired
        elif self.mode == 'CLIK':
            self.q_desired = self.CLIK(self.target_pose, duration=2.0)
        else:
            self.q_desired = self.q_init
            
    def computeSlow(self) -> None:
        while rclpy.ok():
            if self.is_mode_changed:
                self.is_rrt_path_found = False
                self.is_target_pose_changed = False
            
            # Compute desired joint positions based on the current control mode.
            if self.mode == 'RRTmove':
                if self.is_target_pose_changed:
                    self.is_target_pose_changed = False
                    self._path_msg_sent     = False
                    for _ in range(10):
                        self.node.get_logger().info("Attempting to solve IK...")
                        is_ik_found, q_goal = self.tracIK.solve(pos=self.target_pose[0:3,3],
                                                    quat=R.from_matrix(self.target_pose[0:3,0:3]).as_quat(), 
                                                    q_init=self.q)
                        if is_ik_found:
                            self.node.get_logger().info(f"IK solution found:\n{q_goal}")
                            if self.pc.is_valid(q_goal):
                                self.node.get_logger().info("IK solution is valid")
                                break
                            else:
                                self.node.get_logger().info('fr3 is in collision')
                                is_ik_found = False
                        else:
                            self.node.get_logger().info('fr3 IK failed')
                            
                    if is_ik_found:
                        self.rrt_planner.max_distance = 0.1
                        self.rrt_planner.set_start(self.q)
                        self.rrt_planner.set_goal(q_goal)
                        for _ in range(10):
                            self.node.get_logger().info("Attempting to find RRT path...")
                            self.is_rrt_path_found, path = self.rrt_planner.solve()
                            if self.is_rrt_path_found:
                                self.node.get_logger().info("Path found")
                                self.traj_q, self.traj_qdot, self.traj_qddot, self.traj_time = self.pc.time_parameterize(path,
                                                                                                    max_velocity_scaling_factor=1.0,
                                                                                                    max_acceleration_scaling_factor=1.0)
                                self.traj_time = self.current_time + self.traj_time - self.traj_time[0]
                                # Publish RRT trajectory
                                if not self._path_msg_sent:
                                    pa = self._joint_traj_to_pose_array(self.traj_q)
                                    self.RRTtraj_pub.publish(pa)
                                    self._path_msg_sent = True
                                break
                if self.is_rrt_path_found:
                    for i in range(len(self.traj_time)-1):
                        if self.current_time >= self.traj_time[i] and self.current_time < self.traj_time[i+1]:                            
                            self.q_desired = cubic_spline(time=self.current_time,
                                                            time_0=self.traj_time[i],
                                                            time_f=self.traj_time[i+1],
                                                            x_0=self.traj_q[i],
                                                            x_f=self.traj_q[i+1], 
                                                            x_dot_0=self.traj_qdot[i], 
                                                            x_dot_f=self.traj_qdot[i+1])
                            self.qdot_desired = cubic_dot_spline(time=self.current_time,
                                                                    time_0=self.traj_time[i],
                                                                    time_f=self.traj_time[i+1],
                                                                    x_0=self.traj_q[i],
                                                                    x_f=self.traj_q[i+1], 
                                                                    x_dot_0=self.traj_qdot[i], 
                                                                    x_dot_f=self.traj_qdot[i+1])
                            break
            time.sleep(0.001)
    
    def getCtrlInput(self) -> dict:
        ctrl_dict = {}
        for i, joint_name in enumerate(self.robot_data.rd_joint_names):
            joint_name = joint_name.replace('_joint', '')  # Replace 'joint' with 'trn' in the joint name
            ctrl_dict[joint_name] = self.q_desired[i]
        return ctrl_dict
    
    # =============================================================================================
    def keyCallback(self, msg: Int32):
        self.node.get_logger().info(f"[UR5eController] Key input received: {msg.data}")
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
        self.node.get_logger().info(f"[UR5eController] Mode changed: {mode}")
        self.mode = mode
        
    # TODO: CLIK is wierd, needs to be fixed
    def CLIK(self, 
             x_target: np.ndarray, 
             duration: float = 2.0,
             kp: np.ndarray = np.array([10, 10, 10, 10, 10, 10]),
             kd: np.ndarray = np.array([5, 5, 5, 5, 5, 5])) -> np.ndarray:
        
        control_start_time = self.current_time
        x_init = self.x_init
        xdot_init = self.xdot_init
        if self.is_target_pose_changed:
            self.is_target_pose_changed = False
            control_start_time = self.current_time
            x_init = self.x
            xdot_init = self.xdot
        x_desired = np.eye(4)
        xdot_desired = np.zeros(6)
        x_desired[0:3, 3] = cubic_spline(time=self.current_time,
                                          time_0=control_start_time,
                                          time_f=control_start_time + duration,
                                          x_0=x_init[0:3, 3],
                                          x_f=x_target[0:3, 3],
                                          x_dot_0=xdot_init[0:3],
                                          x_dot_f=np.zeros(3))
        x_desired[0:3, 0:3] = rotation_cubic(time=self.current_time,
                                             time_0=control_start_time,
                                             time_f=control_start_time + duration,
                                             R_0=x_init[0:3, 0:3],
                                             R_f=x_target[0:3, 0:3])
        xdot_desired[0:3] = cubic_dot_spline(time=self.current_time,
                                             time_0=control_start_time,
                                             time_f=control_start_time + duration,
                                             x_0=x_init[0:3, 3],
                                             x_f=x_target[0:3, 3],
                                             x_dot_0=xdot_init[0:3],
                                             x_dot_f=np.zeros(3))
        xdot_desired[3:6] = rotation_cubic_dot(time=self.current_time,
                                               time_0=control_start_time,
                                               time_f=control_start_time + duration,
                                               R_0=x_init[0:3, 0:3],
                                               R_f=x_target[0:3, 0:3])
        self.x_desired = x_desired
        self.xdot_desired = xdot_desired
        
        x_error = np.zeros(6)
        x_error[0:3] = self.x_desired[0:3, 3] - self.x[0:3, 3]
        x_error[3:6] = get_phi(self.x_desired[0:3, 0:3], self.x[0:3, 0:3])
        self.node.get_logger().info(f"[UR5eController] Position error: {x_error[0:3]}, Orientation error: {x_error[3:6]}")

        xdot_error = self.xdot_desired - self.xdot
        self.node.get_logger().info(f"[UR5eController] Position velocity error: {xdot_error[0:3]}, Orientation velocity error: {xdot_error[3:6]}")

        J = self.robot_data.getJacobian()
        self.node.get_logger().info(f"[UR5eController] Jacobian: {J}")
        J_pinv = np.linalg.inv(J)

        self.node.get_logger().info(f"[UR5eController] Pseudo-inverse Jacobian: {J_pinv}")
        self.node.get_logger().info(f"[UR5eController] Desired joint velocities: {(kp* (x_error) + kd * (xdot_error))}")
        self.qdot_desired = J_pinv @ (kp* (x_error) + kd * (xdot_error))
        self.node.get_logger().info(f"[UR5eController] Desired joint velocities: {self.qdot_desired}")
        return self.q + self.dt * self.qdot_desired
        
        
    def targetPoseCallback(self, msg: PoseStamped):
        self.node.get_logger().info(f"[UR5eController] Target pose received: {msg}")
        self.is_target_pose_changed = True
        # Convert the Pose message to a 4x4 transformation matrix
        quat = [msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w]

        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z]

        self.target_pose = T
        
    def pub_EEpose_callback(self):
        # Publish end-effector pose
        ee_pose_msg = PoseStamped()
        ee_pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        ee_pose_msg.header.frame_id = "base"
        ee_pose_msg.pose.position.x = self.x[0, 3]
        ee_pose_msg.pose.position.y = self.x[1, 3]
        ee_pose_msg.pose.position.z = self.x[2, 3]

        quat = R.from_matrix(self.x[0:3, 0:3]).as_quat()
        ee_pose_msg.pose.orientation.x = quat[0]
        ee_pose_msg.pose.orientation.y = quat[1]
        ee_pose_msg.pose.orientation.z = quat[2]
        ee_pose_msg.pose.orientation.w = quat[3]

        self.EEpose_pub.publish(ee_pose_msg)

    def _joint_traj_to_pose_array(self, traj_q):
        """Forward-kinematics every waypoint and pack into PoseArray."""
        pa = PoseArray()
        pa.header.stamp    = self.node.get_clock().now().to_msg()
        pa.header.frame_id = "base"

        for q in traj_q:
            # ▸ call the correct FK wrapper
            pos, quat = self.tracIK.forward_kinematics(q)

            p = Pose()
            p.position.x, p.position.y, p.position.z = pos            # (3,)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat  # (4,)
            pa.poses.append(p)

        return pa

        