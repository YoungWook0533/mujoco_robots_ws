cmake_minimum_required(VERSION 3.16)
project(gl_depth_sim VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

# ─────────────────────────────────────────  Dependencies
find_package(Eigen3       REQUIRED)
find_package(PCL          REQUIRED COMPONENTS common io)
find_package(OpenCV       REQUIRED)
find_package(assimp       REQUIRED)                 # mesh loading

# ─────────────────────────────────────────  glad (OpenGL loader)
add_library(glad SHARED
  src/${PROJECT_NAME}/egl.c
  src/${PROJECT_NAME}/gl.c
)
target_include_directories(glad
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
add_library(gl_depth_sim::glad ALIAS glad)

# ─────────────────────────────────────────  Core renderer
add_library(gl_depth_sim SHARED
  src/${PROJECT_NAME}/sim_depth_camera.cpp
  src/${PROJECT_NAME}/mesh_loader.cpp
  src/${PROJECT_NAME}/mesh.cpp
  src/${PROJECT_NAME}/renderable_mesh.cpp
  src/${PROJECT_NAME}/shader_program.cpp
)
target_compile_options(gl_depth_sim PUBLIC -mno-avx)
target_include_directories(gl_depth_sim
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
target_link_libraries(gl_depth_sim
  PUBLIC
    gl_depth_sim::glad          # the alias we just made
    Eigen3::Eigen
    ${ASSIMP_LIBRARIES}
    dl
)
add_library(gl_depth_sim::gl_depth_sim ALIAS gl_depth_sim)

# ─────────────────────────────────────────  PCL / OpenCV interfaces
add_library(gl_depth_sim_interfaces SHARED
  src/interfaces/pcl_interface.cpp
)
target_include_directories(gl_depth_sim_interfaces
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
          ${PCL_INCLUDE_DIRS})
target_link_libraries(gl_depth_sim_interfaces
  PUBLIC
    gl_depth_sim::gl_depth_sim
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_library(gl_depth_sim::gl_depth_sim_interfaces ALIAS gl_depth_sim_interfaces)

# ─────────────────────────────────────────  Simulated laser scanner
add_library(gl_depth_sim_laser_scanner SHARED
  src/${PROJECT_NAME}/sim_laser_scanner.cpp
)
target_include_directories(gl_depth_sim_laser_scanner
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
target_link_libraries(gl_depth_sim_laser_scanner
  PUBLIC
    gl_depth_sim::gl_depth_sim
    gl_depth_sim::gl_depth_sim_interfaces
)
add_library(gl_depth_sim::gl_depth_sim_laser_scanner ALIAS gl_depth_sim_laser_scanner)

# ─────────────────────────────────────────  Installation / export
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

set(_export_targets
    glad
    gl_depth_sim
    gl_depth_sim_interfaces
    gl_depth_sim_laser_scanner
)

install(TARGETS ${_export_targets}
        EXPORT  gl_depth_sim-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

# Export targets from the build-tree
export(EXPORT gl_depth_sim-targets
       NAMESPACE gl_depth_sim::
       FILE "${CMAKE_CURRENT_BINARY_DIR}/gl_depth_sim-targets.cmake")

# Install the export set for downstream use
install(EXPORT gl_depth_sim-targets
        NAMESPACE gl_depth_sim::
        DESTINATION lib/cmake/${PROJECT_NAME})

# ─────────────────────────────────────────  Config files
include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)

# ─────────────────────────────────────────  Package manifest
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

message(STATUS "gl_depth_sim configured with export targets:
  - gl_depth_sim::gl_depth_sim
  - gl_depth_sim::glad
  - gl_depth_sim::gl_depth_sim_interfaces
  - gl_depth_sim::gl_depth_sim_laser_scanner")
