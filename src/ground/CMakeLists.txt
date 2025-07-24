cmake_minimum_required(VERSION 3.8)
project(ground)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Curses REQUIRED)

include_directories(
  include
  ${CURSES_INCLUDE_DIR}
)


add_executable(trajectory_generator node/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator rclcpp std_msgs geometry_msgs)


add_executable(attitude_controller node/attitude_controller.cpp)
ament_target_dependencies(attitude_controller rclcpp std_msgs)

add_executable(gripper_controller node/gripper_controller.cpp)
ament_target_dependencies(gripper_controller rclcpp std_msgs)

add_executable(keyboard_node node/keyboard_node.cpp src/getch.cpp)
ament_target_dependencies(keyboard_node rclcpp std_msgs px4_msgs)

target_link_libraries(keyboard_node ${CURSES_LIBRARIES})

add_executable(position_controller node/position_controller.cpp)
ament_target_dependencies(position_controller rclcpp std_msgs tf2)

add_executable(qc_servo node/qc_servo.cpp)
ament_target_dependencies(qc_servo rclcpp std_msgs)

add_executable(wrench_mapper node/wrench_mapper.cpp)
ament_target_dependencies(wrench_mapper rclcpp std_msgs)


add_executable(platform_topic_tool src/platform_topic_tool.cpp)
ament_target_dependencies(platform_topic_tool rclcpp std_msgs px4_msgs)


add_executable(mocap_px4_bridge src/mocap_px4_bridge.cpp)
ament_target_dependencies(mocap_px4_bridge rclcpp std_msgs px4_msgs geometry_msgs)

target_include_directories(keyboard_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(TARGETS
  keyboard_node
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  attitude_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  gripper_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  position_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  qc_servo
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  wrench_mapper
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  trajectory_generator
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  platform_topic_tool
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  mocap_px4_bridge
  DESTINATION lib/${PROJECT_NAME})


  # Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
