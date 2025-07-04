from suhan_robot_model_tools2_wrapper_cpp import NameVector, IntVector, PlanningSceneCollisionCheck, isometry_to_vectors
import copy
import numpy as np
# import math
from srmt2.utils.ros_utils import ros_init, fetch_remote_param

class PlanningSceneLight(object):
    def __init__(self, 
                 node_name:str="PlanningScene",  
                 topic_name:str="/planning_scene", 
                 param_node_name:str="rviz2",
                 urdf_description_param:str='robot_description',
                 srdf_description_param:str='robot_description_semantic',
                 urdf_file_path:str=None,
                 srdf_file_path:str=None,
                 base_link:str='/base') -> None:
        """Planning Scene Light
        It does not require full group names and joitn dofs
        """
        ros_init()

        self.pc = PlanningSceneCollisionCheck(node_name=node_name, 
                                              topic_name=topic_name,
                                              param_node_name=param_node_name,
                                              urdf_description_param=urdf_description_param,
                                              srdf_description_param=srdf_description_param,
                                              urdf_file_path=urdf_file_path,
                                              srdf_file_path=srdf_file_path,
                                              base_link=base_link)
        self.pc.set_frame_id(base_link)
        
    def update_joints(self, group_name, q):
        """update whole joints

        Args:
            group_name (str): group name
            q (numpy.array of numpy.double): joint values
        """
        q = q.astype(np.double)
        self.pc.set_joint_group_positions(group_name, q)

    def is_current_valid(self) -> bool:
        """check current state is valid

        Returns:
            bool: True if valid
        """
        return self.pc.is_current_valid()
    
    def display(self):
        self.pc.publish_planning_scene_msg()

    def add_box(self, name, dim, pos, quat):
        self.pc.add_box(np.array(dim,dtype=np.double),name,
                        np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def add_cylinder(self, name, height, radius, pos, quat):
        self.pc.add_cylinder(np.array([height, radius],dtype=np.double), name, 
                             np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def add_sphere(self, name, radius, pos, quat):
        self.pc.add_sphere(radius, name, 
                           np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def add_mesh(self, name, mesh_path, pos, quat):
        self.pc.add_mesh_from_file(mesh_path, name, 
                         np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))
        
    def remove_object(self, name):
        self.pc.remove_object(name)
        
    def remove_all_objects(self):
        self.pc.remove_all_objects()
        
    def attach_object(self, object_id, link_name, touch_links=[]):
        _touch_links = NameVector()
        
        for tl in touch_links:
            _touch_links.append(tl)
        
        self.pc.attach_object(object_id, link_name, _touch_links)

    def detach_object(self, object_id, link_name):
        self.pc.detach_object(object_id, link_name)

    def update_object_pose(self, object_id, pos, quat):
        self.pc.update_object_pose(object_id, np.array(pos, dtype=np.double),np.array(quat, dtype=np.double))

    def print_current_collision_infos(self):
        self.pc.print_current_collision_infos()

    def display(self, group_name=None, q=None):
        if q is not None and group_name is not None:
            self.update_joints(group_name, q)

        self.pc.publish_planning_scene_msg()


class PlanningScene(PlanningSceneLight):
    def __init__(self, 
                 arm_names:list, 
                 arm_dofs:list, 
                 base_link:str='/base', 
                 hand_names:list=None, 
                 hand_joints:list=[2], 
                 hand_open:list=[[0.0325,0.0325]], 
                 hand_closed:list=[[0.0, 0.0]], 
                 node_name:str="PlanningScene",
                 topic_name:str="/planning_scene", 
                 param_node_name:str="rviz2",
                 urdf_description_param:str='robot_description',
                 srdf_description_param:str='robot_description_semantic',
                 urdf_xml:str=None,
                 srdf_xml:str=None,
                 urdf_file_path:str=None,
                 srdf_file_path:str=None,
                 q_init:np.array=None,
                 base_q:np.array=None,
                 start_index:int=None,
                 end_index:int=None):
        
        ros_init()
        
        if urdf_file_path is not None:
            with open(urdf_file_path, "r", encoding="utf-8") as f:
                _urdf_xml = f.read()
        elif urdf_xml is not None:
            _urdf_xml = urdf_xml
        else:
            _urdf_xml = fetch_remote_param(param_node_name, urdf_description_param)
            
        if srdf_file_path is not None:
            with open(srdf_file_path, "r", encoding="utf-8") as f:
                _srdf_xml = f.read()
        elif srdf_xml is not None:
            _srdf_xml = srdf_xml
        else:
            _srdf_xml = fetch_remote_param(param_node_name, srdf_description_param)

        # self.pc = PlanningSceneCollisionCheck(param_node_name, node_name, topic_name, urdf_param)
        self.pc = PlanningSceneCollisionCheck(node_name, topic_name, _urdf_xml, _srdf_xml)
        
        self.base_q = base_q
        self.start_index = start_index
        self.end_index = end_index

        self.use_hand = False
        if hand_names is not None:
            self.hand_names = hand_names
            self.use_hand = True
            
        names_vec = NameVector()
        dofs_vec = IntVector()
        self.name_to_indices = {}
        # print('planning scene!')
        current_idx = 0
        for name, dof in zip(arm_names, arm_dofs):
            names_vec.append(name)
            dofs_vec.append(dof)
            self.name_to_indices[name] = (current_idx, current_idx+dof)
            current_idx += dof

        self.hand_open = np.array(hand_open, dtype=np.double)
        self.hand_closed = np.array(hand_closed, dtype=np.double)
        self.hand_joints = hand_joints

        if hand_names is not None:
            for name, dof in zip(hand_names, hand_joints):
                names_vec.append(name)
                dofs_vec.append(dof)
        
        self.pc.set_group_names_and_dofs(names_vec,dofs_vec)
        if q_init is not None:
            self.display(q_init)
        self.pc.set_frame_id(base_link)
        
        if hand_names is not None:
            self.gripper_open = [True] * len(hand_names)

    def set_planning_joint_group(self, name):
        self.set_planning_joint_index(*self.name_to_indices[name])
        
    def get_joint_start_end_index(self, name):
        return self.name_to_indices[name]
    
    def set_planning_joint_index(self, start_index, end_index):
        self.start_index = start_index
        self.end_index = end_index

    def add_gripper_to_q(self, q):
        q = copy.deepcopy(q)
        if self.use_hand:
            for g, open, closed in zip(self.gripper_open, self.hand_open, self.hand_closed):
                if g:
                    q = np.concatenate((q, open.flatten()))
                else:
                    q = np.concatenate((q, closed.flatten()))
        return q

    def update_joints(self, q):
        """update whole joints

        Args:
            q (np.array float): full configurations
        """

        q = q.astype(np.double)
        q = self.add_gripper_to_q(q)
        self.pc.update_joints(q)
        if self.base_q is not None:
            self.base_q = copy.deepcopy(q)

    def display(self, q=None):
        if q is not None:
            self.update_joints(q)

        self.pc.publish_planning_scene_msg()

    def display_single(self, q=None):
        if self.base_q is not None:
            self.base_q[self.start_index:self.end_index] = q
            q = self.base_q
            
        if q is not None:
            self.update_joints(q)

        self.pc.publish_planning_scene_msg()

    def is_valid(self, q):
        q = q.astype(np.double)

        if self.base_q is not None:
            q_full = copy.deepcopy(self.base_q)
            q_full[self.start_index:self.end_index] = q
            q = q_full
            
        q = self.add_gripper_to_q(q)
        return self.pc.is_valid(q)

    def time_parameterize(self, path, max_velocity_scaling_factor=1.0, max_acceleration_scaling_factor=1.0):
        N, dof = path.shape
        q_result   = np.zeros((N + 2, dof),  order='F', dtype=np.float64)
        qdot_result = np.zeros_like(q_result,  order='F')
        qddot_result = np.zeros_like(q_result, order='F')
        time_result = np.zeros(N + 2,          order='F', dtype=np.float64)
        
        self.pc.time_parameterize(path, q_result, qdot_result, qddot_result, time_result, max_velocity_scaling_factor, max_acceleration_scaling_factor)
        return q_result, qdot_result, qddot_result, time_result

    def get_minimum_distance(self, q:np.ndarray=None, is_self_collision:bool=True, is_env_collision:bool=True):
        if q is not None:
            self.update_joints(q)
        
        return self.pc.get_minimum_distance(is_self_collision, is_env_collision)
    
    def get_links_min_distances(self, link_names:list, q:np.ndarray=None, is_self_collision:bool=True, is_env_collision:bool=True):
        if q is not None:
            self.update_joints(q)
            
        _link_names = NameVector()
        for ln in link_names:
            _link_names.append(ln)
            
        return np.array([self.pc.get_links_min_distances(_link_names, is_self_collision, is_env_collision)]).flatten()