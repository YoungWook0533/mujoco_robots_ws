cmake_minimum_required(VERSION 3.5)
project(mppi)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

file(GLOB_RECURSE SRCS src/*.cpp)
add_library(${PROJECT_NAME} ${SRCS})
# Ensure PIC so static/shared can link properly
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
endif()

ament_target_dependencies(${PROJECT_NAME}
  Eigen3
  yaml-cpp
)

# Export targets and dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 yaml-cpp OpenMP)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(spline_test unittest/spline.cpp)
    target_link_libraries(spline_test ${PROJECT_NAME})

    ament_add_gtest(logging unittest/data_logging.cpp)
    target_link_libraries(logging ${PROJECT_NAME})
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT mppiTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT mppiTargets
  NAMESPACE mppi::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(mppiTargets HAS_LIBRARY_TARGET)

ament_package()
