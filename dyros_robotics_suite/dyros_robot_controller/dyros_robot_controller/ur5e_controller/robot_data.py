import numpy as np
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import pinocchio as pin

"""
UR5e URDF Joint Information
Total nq = 6, total nv = 6
 id | name                 | nq | nv | idx_q | idx_v
----+----------------------+----+----+-------+------
  1 | shoulder_pan_joint   |  1 |  1 |     0 |    0
  2 | shoulder_lift_joint  |  1 |  1 |     1 |    1
  3 | elbow_joint          |  1 |  1 |     2 |    2
  4 | wrist_1_joint        |  1 |  1 |     3 |    3
  5 | wrist_2_joint        |  1 |  1 |     4 |    4
  6 | wrist_3_joint        |  1 |  1 |     5 |    5

"""

class UR5eRobotData():
    def __init__(self, node: Node, mj_joint_names: list):
        self.node = node
        self.mj_joint_names = mj_joint_names
        
        urdf_path = os.path.join(
            get_package_share_directory('dyros_robot_controller'),
            'robot',
            'ur5e.urdf'
        )
        # Build model and data
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = pin.Data(self.model)
        self.ee_name = "flange"  # End-effector name in the URDF

        # Joint names (exclude universe joint at index 0)
        self.rd_joint_names = [f.name for f in self.model.frames if f.type == pin.JOINT]
        self.nq = self.model.nq
        self.nv = self.model.nv

        self.node.get_logger().info(f"Total nq = {self.nq}, total nv = {self.nv}")
        self.node.get_logger().info(" id | name                 | nq | nv | idx_q | idx_v")
        self.node.get_logger().info("----+----------------------+----+----+-------+------")
        for idx in range(1, len(self.model.joints)):
            joint = self.model.joints[idx]
            name = self.model.names[idx]
            self.node.get_logger().info(f"{idx:3d} | {name:20s} | {self.model.nqs[idx]:2d} | {self.model.nvs[idx]:2d} |")

        # Initialize states
        self.q = np.zeros(self.nq)
        self.qdot = np.zeros(self.nv)
        # Initialize kinematics
        self.x = np.eye(4)
        self.xdot = np.zeros(6)
        self.J = np.zeros((6, self.nv))
        self.Jdot = np.zeros((6, self.nv))
    
    def updateState(self, 
                    pos_dict: dict, 
                    vel_dict: dict, 
                    tau_ext_dict: dict, 
                    sensor_dict:dict):
        for rd_joint_name in self.rd_joint_names:
            rd_index = self.rd_joint_names.index(rd_joint_name)
            
            self.q[rd_index] = pos_dict[rd_joint_name]
            self.qdot[rd_index] = vel_dict[rd_joint_name]
        
        if(not self.updateKinematics(self.q, self.qdot)):
            self.node.get_logger().error("[UR5eRobotData] Failed to update robot state.")

# ==================================================================================================
    def updateKinematics(self, q: np.ndarray, qdot: np.ndarray) -> bool:
        """
        Update kinematic state: pose, Jacobian, and their time derivatives.

        Args:
            q (np.ndarray): Joint positions, shape (nq,).
            qdot (np.ndarray): Joint velocities, shape (nv,).

        Returns:
            bool: False if input sizes mismatch or frame not found, True otherwise.
        """
        if q.size != self.nq or qdot.size != self.nv:
            self.node.get_logger().error(f"[UR5eRobotData] Error: q size {q.size}, qdot size {qdot.size} do not match model dimensions.")
            return False

        # Update internal state
        self.q = q.copy()
        self.qdot = qdot.copy()

        # Compute joint Jacobians and their time variation
        pin.computeJointJacobians(self.model, self.data, self.q)
        pin.computeJointJacobiansTimeVariation(self.model, self.data, self.q, self.qdot)

        # Update frame placements for pose
        pin.framesForwardKinematics(self.model, self.data, self.q)

        # Get end-effector frame index
        frame_id = self.model.getFrameId(self.ee_name)
        if frame_id < 0:
            print(f"Error: Frame '{self.ee_name}' not found in model.")
            return False

        # Pose: homogeneous matrix
        self.x = self.data.oMf[frame_id].homogeneous
        # Jacobian in world frame
        self.J = pin.getFrameJacobian(self.model, self.data, frame_id, pin.ReferenceFrame.WORLD)
        # Spatial velocity
        self.xdot = self.J.dot(self.qdot)
        # Jacobian time derivative
        self.Jdot = pin.getFrameJacobianTimeVariation(self.model, self.data, frame_id, pin.ReferenceFrame.WORLD)

        return True
    
    def computePose(self, q: np.ndarray) -> np.ndarray:
        """
        Compute end-effector pose for given configuration.

        Args:
            q (np.ndarray): Joint positions, shape (nq,).

        Returns:
            np.ndarray: 4x4 homogeneous transform.
        """
        data_tmp = pin.Data(self.model)
        pin.framesForwardKinematics(self.model, data_tmp, q)
        frame_id = self.model.getFrameId(self.ee_name)
        if frame_id < 0:
            raise ValueError(f"Frame '{self.ee_name}' not found.")
        return data_tmp.oMf[frame_id].homogeneous

    def computeJacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Compute end-effector Jacobian for given configuration.
        """
        data_tmp = pin.Data(self.model)
        pin.computeJointJacobians(self.model, data_tmp, q)
        frame_id = self.model.getFrameId(self.ee_name)
        if frame_id < 0:
            raise ValueError(f"Frame '{self.ee_name}' not found.")
        return pin.getFrameJacobian(self.model, data_tmp, frame_id, pin.ReferenceFrame.WORLD)

    def computeJacobianTimeVariation(
        self, q: np.ndarray, qdot: np.ndarray
    ) -> np.ndarray:
        """
        Compute time derivative of Jacobian for given state.
        """
        data_tmp = pin.Data(self.model)
        pin.computeJointJacobians(self.model, data_tmp, q)
        pin.computeJointJacobiansTimeVariation(self.model, data_tmp, q, qdot)
        frame_id = self.model.getFrameId(self.ee_name)
        if frame_id < 0:
            raise ValueError(f"Frame '{self.ee_name}' not found.")
        return pin.getFrameJacobianTimeVariation(
            self.model, data_tmp, frame_id, pin.ReferenceFrame.WORLD
        )

    def computeVelocity(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        """
        Compute end-effector spatial velocity J(q)*qdot.
        """
        J = self.computeJacobian(q)
        return J.dot(qdot)

    def getPose(self) -> np.ndarray:
        """
        Get the current end-effector pose.

        Returns:
            np.ndarray: 4x4 homogeneous transformation matrix.
        """
        return self.x.copy()

    def getJacobian(self) -> np.ndarray:
        """
        Get the current end-effector Jacobian.

        Returns:
            np.ndarray: 6xnv Jacobian matrix.
        """
        return self.J.copy()

    def getJacobianTimeVariation(self) -> np.ndarray:
        """
        Get the time derivative of the end-effector Jacobian.

        Returns:
            np.ndarray: 6xnv Jacobian time derivative.
        """
        return self.Jdot.copy()

    def getVelocity(self) -> np.ndarray:
        """
        Get the current end-effector spatial velocity.

        Returns:
            np.ndarray: 6D spatial velocity vector.
        """
        return self.xdot.copy()
