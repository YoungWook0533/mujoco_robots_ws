from srmt2.planning_scene import PlanningScene
import numpy as np
import time
np.set_printoptions(precision=4, suppress=True, linewidth=300)

joint_limit = np.array([[-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973],  # min
                            [ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973]]) # max

links = ["fr3_link0", 
         "fr3_link1", 
         "fr3_link2", 
         "fr3_link3", 
         "fr3_link4", 
         "fr3_link5", 
         "fr3_link6", 
         "fr3_link7", 
         "fr3_link8"]

# pc = PlanningScene(node_name="PlanningScene", arm_names=['fr3'], arm_dofs=[7], base_link="base_link")
pc = PlanningScene(node_name="PlanningScene", 
                   arm_names=['fr3_arm'], 
                   arm_dofs=[7], 
                   base_link="fr3_link0", 
                   urdf_file_path="/home/yoonjunheon/ros2_ws/src/fr3_moveit_config/config/fr3.urdf", 
                   srdf_file_path="/home/yoonjunheon/ros2_ws/src/fr3_moveit_config/config/fr3.srdf",)
pc.add_box('abcd', [.1,0.1,0.5], [-0.3,-0.1,0.1], [0.0,0.0,0.0,1.0])
pc.add_box('abcd2', [0.2,0.5,0.1], [0.2,0.1,0.1], [0.0,0.0,0.0,1.0])
    
pc.display(np.array([0,0,0,0,0,0,0]))
time.sleep(1)

for _ in range(100):
    pc.update_joints(q=np.random.uniform(low=joint_limit[0], high=joint_limit[1]))
    pc.display()
    self_min_dist = pc.get_minimum_distance(is_self_collision=True, 
                                            is_env_collision=False)
    env_min_dist = pc.get_minimum_distance(is_self_collision=False,
                                            is_env_collision=True)
    self_min_dist_vec = pc.get_links_min_distances(link_names=links,
                                                   is_self_collision=True,
                                                   is_env_collision=False)
    env_min_dist_vec = pc.get_links_min_distances(link_names=links,
                                                  is_self_collision=False,
                                                  is_env_collision=True)
    print("=========================")
    print("self min dist: ", self_min_dist)
    print("self min dist vec: ", self_min_dist_vec)
    print("env min dist: ", env_min_dist)
    print("env min dist vec: ", env_min_dist_vec)
    if self_min_dist < 0:
        print("self collision")
        pc.print_current_collision_infos()
        time.sleep(10)
    if env_min_dist < 0:
        print("env collision")
        pc.print_current_collision_infos()
        time.sleep(10)
    print("=========================")
    time.sleep(1)