# Suhan Robot Model Tools2
Robot model tools for motion planning on ROS2

## Features
- Collision checker (MoveIt planning scene)
- IK solver (TRAC IK)
- Motion planning (RRT Connect)
- Constraint functions for orientaiton and kinematic constraints
- Visual simulation (OpenGL)
- Python API


## Installation guide
Install dependencies
```sh
sudo apt install ros-$ROS_DISTRO-moveit
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
sudo apt install libnlopt-dev libnlopt-cxx-dev libglfw3-dev

pip install tqdm matplotlib
```

Install step and running code example
```sh
cd ~/ros2_ws/src/
git clone https://github.com/JunHeonYoon/trac_ik.git
git clone https://github.com/JunHeonYoon/gl_depth_sim
git clone -b humble https://github.com/JunHeonYoon/suhan_robot_model_tools.git

cd ..
colcon build
source install/setup.bash
```
