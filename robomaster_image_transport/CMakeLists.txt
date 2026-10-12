cmake_minimum_required(VERSION 3.8)
project(robomaster_image_transport)

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(compressed_republisher src/compressed_republisher.cpp)
target_compile_features(compressed_republisher PRIVATE cxx_std_17)
ament_target_dependencies(compressed_republisher
  image_transport pluginlib rclcpp sensor_msgs)

install(TARGETS compressed_republisher DESTINATION lib/${PROJECT_NAME})

ament_package()
