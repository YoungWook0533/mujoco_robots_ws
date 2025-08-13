cmake_minimum_required(VERSION 3.5)
project(mppi_pinocchio)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB SOURCES src/*.cpp)
add_library(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)

# Expose public include dir to consumers
 target_include_directories(${PROJECT_NAME}
   PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
 )

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  pinocchio
  geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export for downstream packages
ament_export_dependencies(Eigen3 pinocchio geometry_msgs)
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)

ament_package()
