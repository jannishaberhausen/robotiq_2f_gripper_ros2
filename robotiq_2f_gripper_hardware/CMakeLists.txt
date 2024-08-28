cmake_minimum_required(VERSION 3.8)
project(robotiq_2f_gripper_hardware LANGUAGES CXX)

include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robotiq_2f_gripper_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(robotiq_2f_gripper_interfaces REQUIRED)

include_directories(
  include
)

add_executable(robotiq_2f_gripper_node 
  src/robotiq_2f_gripper_node.cpp)
target_link_libraries(robotiq_2f_gripper_node 
  ${robotiq_2f_gripper_interfaces_LIBRARIES})
ament_target_dependencies(robotiq_2f_gripper_node 
  rclcpp
  rclcpp_action
  hardware_interface 
  pluginlib 
  serial
  robotiq_2f_gripper_msgs
  std_msgs
  sensor_msgs 
  robotiq_2f_gripper_interfaces
)

install(TARGETS robotiq_2f_gripper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
