# mujoco_robots_ws
mujoco robot simulation
## Requirements
### mujoco_ros_sim
- eigenpy:
```
pip install eigenpy
```
- MuJoCo:
```
pip install mujoco-py
```
### dyros_robot_controller
- [ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)
- [OSQP](https://osqp.org/docs/get_started/sources.html)
- [OSQP-Eigen](https://github.com/robotology/osqp-eigen)
- Scipy >= 14.0
- anytree:
```
pip install anytree
```
- other dependencies
```
sudo apt install ros-humble-interactive-markers
sudo apt install ros-humble-visualization-msgs
```
### sampling_based_control
```
sudo apt-get install libyaml-cpp-dev libglfw3-dev
```
## Build
```
cd ros2_ws # your ros2 workspace
colcon build --symlink-install
source install/setup.bash
```
## Run
```
ros2 launch dyros_robot_controller fr3_controller.launch.py  # or any other controllers
```