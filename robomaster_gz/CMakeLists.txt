cmake_minimum_required(VERSION 3.8)
project(robomaster_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robomaster_interface REQUIRED)

set(dependencies
  rclcpp
  std_srvs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  robomaster_interface
)

add_executable(drone_srv
  src/drone_srv.cpp
)
ament_target_dependencies(drone_srv ${dependencies})
install(TARGETS
drone_srv
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
drone_srv
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#  Install.

install(
  DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
)
install(
  DIRECTORY
  data/
  DESTINATION share/${PROJECT_NAME}/data
)
install(
  DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(
  DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# --------------------------------------------------------------------------- #
# Find gz-sim and dependencies.

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS profiler)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# Garden
if("$ENV{GZ_VERSION}" STREQUAL "garden")
  gz_find_package(gz-sim7 REQUIRED)
  set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
  gz_find_package(gz-rendering7 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Garden")
# Harmonic (default)
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "ENV{GZ_VERSION}")
  gz_find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  gz_find_package(gz-rendering8 REQUIRED)
  set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
else()  
  message(FATAL_ERROR "Unsupported GZ_VERSION: $ENV{GZ_VERSION}")
endif()

# --------------------------------------------------------------------------- #


#  Build tests.

if(BUILD_TESTING)
  # Override default flake8 configuration.
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_SOURCE_DIR}/.flake8)

  # Add linters.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------------------------------- #
#  Environment hooks.

if(${ament_cmake_FOUND})
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")
  ament_package()
endif()
