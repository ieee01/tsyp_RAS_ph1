cmake_minimum_required(VERSION 3.8)
project(living_map_bringup)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)

add_executable(clock_throttle src/clock_throttle.cpp)
target_compile_features(clock_throttle PUBLIC cxx_std_17)
target_link_libraries(clock_throttle PUBLIC rclcpp::rclcpp ${rosgraph_msgs_TARGETS})
install(TARGETS clock_throttle DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/nav2_guard.py DESTINATION lib/${PROJECT_NAME} RENAME nav2_guard)
install(PROGRAMS scripts/sim_guard.py DESTINATION lib/${PROJECT_NAME} RENAME sim_guard)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# Install the repository-level configuration into the package share directory so
# `ros2 launch` never depends on the current working directory.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../config/" DESTINATION share/${PROJECT_NAME}/config)

ament_package()
