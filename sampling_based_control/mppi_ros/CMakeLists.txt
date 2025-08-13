cmake_minimum_required(VERSION 3.10)
project(mppi_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mppi REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pinocchio REQUIRED)
find_package(mppi_pinocchio REQUIRED)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Array.msg"
  "msg/Config.msg"
  "msg/Rollout.msg"
  "msg/Data.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mppi_threading
  src/threading/Rate.cpp
  src/threading/Worker.cpp
  src/threading/WorkerManager.cpp
)

add_library(mppi_ros_lib STATIC
  src/controller_interface.cpp
  src/conversions.cpp
  src/ros_params.cpp
  src/husky_fr3/dynamics.cpp
  src/husky_fr3/cost.cpp
  src/husky_fr3/node.cpp
)

ament_target_dependencies(mppi_ros_lib
  Eigen3
  mppi
  sensor_msgs
  std_msgs
  geometry_msgs
  rclcpp
  mppi_pinocchio
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(mppi_threading OpenMP::OpenMP_CXX)
  target_link_libraries(mppi_ros_lib OpenMP::OpenMP_CXX)
endif()

target_link_libraries(mppi_ros_lib mppi_threading)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(typesupport_target)
  target_link_libraries(mppi_ros_lib ${typesupport_target})
endif()

add_executable(husky_fr3_mppi_node src/husky_fr3_mppi_node.cpp)
ament_target_dependencies(husky_fr3_mppi_node rclcpp geometry_msgs std_msgs mppi mppi_pinocchio)
# Link to our libs and mppi core (mppi_pinocchio will be linked via ament_target_dependencies)
# Note: order matters on some linkers
# Also link pinocchio transitive includes from mppi_pinocchio
if(OpenMP_CXX_FOUND)
  target_link_libraries(mppi_ros_lib OpenMP::OpenMP_CXX)
endif()

target_link_libraries(husky_fr3_mppi_node mppi_ros_lib mppi::mppi)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mppi_ros_lib mppi_threading husky_fr3_mppi_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/plot_experiment.py scripts/record_data.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
