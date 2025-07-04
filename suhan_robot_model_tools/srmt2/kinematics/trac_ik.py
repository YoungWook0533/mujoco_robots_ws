from suhan_robot_model_tools2_wrapper_cpp import TRACIKAdapter, vectors_to_isometry, isometry_to_vectors
import numpy as np
from srmt2.utils.ros_utils import ros_init, fetch_remote_param


class TRACIK:
    """
    Python adapter around the C++ TRAC-IK wrapper.
    Resolves the URDF either from another node's parameter server
    or directly from a file, then instantiates `TRACIKAdapter`.

    Parameters
    ----------
    base_link : str
        Name of the base link for IK.
    tip_link : str
        Name of the tip link (end-effector).
    max_time : float, optional
        IK timeout (seconds). Default = 0.1
    precision : float, optional
        Solution tolerance. Default = 1e-5
    param_node_name : str, optional
        Remote node holding description parameters. Default = 'rviz2'
    urdf_description_param : str, optional
        Parameter name for URDF. Default = 'robot_description'
    urdf_file_path : str | None, optional
        Path to URDF file (overrides parameter lookup if provided).
    """

    def __init__(
        self,
        base_link: str,
        tip_link: str,
        max_time: float = 0.1,
        precision: float = 1e-5,
        param_node_name: str = "rviz2",
        urdf_description_param: str = "robot_description",
        urdf_file_path: str | None = None,
    ):
        ros_init()

        if urdf_file_path is not None:
            with open(urdf_file_path, "r", encoding="utf-8") as f:
                urdf_xml = f.read()
        else:
            urdf_xml = fetch_remote_param(param_node_name, urdf_description_param)

        self.tracik = TRACIKAdapter(base_link, tip_link, max_time, precision, urdf_xml)
    

    def solve(self, pos, quat, q_init):
        assert(self.tracik.get_num_joints() == len(q_init), 'q_init size mismatch')

        iso = vectors_to_isometry(pos, quat)
        q_res = np.zeros(self.tracik.get_num_joints())
        r = self.tracik.solve(q_init, iso, q_res)
        return r, q_res

    def forward_kinematics(self, q):
        pos_quat = isometry_to_vectors(self.tracik.forward_kinematics(q))
        pos = pos_quat.first
        quat = pos_quat.second
        return pos, quat

    def get_lower_bound(self):
        return self.tracik.get_lower_bound()

    def get_upper_bound(self):  
        return self.tracik.get_upper_bound()

    def get_num_joints(self):
        return self.tracik.get_num_joints()

    def is_valid(self, q):
        return self.tracik.is_valid(q)

    def get_jacobian_matrix(self, q):
        return self.tracik.get_jacobian_matrix(q)

    def set_bounds(self, lb, ub):
        self.tracik.set_bounds(lb, ub)

    def set_tolerance_bounds(self, tol):
        self.tracik.set_tolerance_bounds(tol)

    def set_solve_type(self, type):
        self.tracik.set_solve_type(type)
        
    