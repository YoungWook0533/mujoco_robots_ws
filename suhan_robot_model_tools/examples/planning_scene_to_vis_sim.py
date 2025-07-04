from srmt2.planning_scene import PlanningScene, VisualSimulator
import numpy as np
import time


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
pc.display(np.array([0,0,0,-np.pi/2,0,np.pi/2,np.pi/4]))
time.sleep(1)

vs = VisualSimulator()
vs.load_scene(pc)

vs.set_cam_and_target_pose(np.array([0.5, 0.0, 2.0]), np.array([0.0, 0.0, 1.0]))
np.printoptions(precision=3, suppress=True, linewidth=100, threshold=10000)
depth = vs.generate_depth_image()

scene_bound_min = np.array([-1, -1, 0])
scene_bound_max = np.array([1, 1, 2])
vs.set_scene_bounds(scene_bound_min, scene_bound_max)
vs.set_grid_resolution(16)
voxel_grid = vs.generate_voxel_occupancy()

voxel_grid = voxel_grid.reshape(16,16,16)
print(voxel_grid)

import matplotlib.pyplot as plt
title_font = {
    'fontsize': 16,
    'fontweight': 'bold'
}
ax1 = plt.figure(1).add_subplot()
ax1.set_title("depth image", fontsize=16, fontweight='bold', pad=20)
ax1.imshow(depth)

ax = plt.figure(2).add_subplot(projection='3d')
ax.voxels(voxel_grid)
ax.set_title("voxel grid", fontsize=16, fontweight='bold', pad=20)

plt.show()