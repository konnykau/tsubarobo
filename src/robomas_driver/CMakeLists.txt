cmake_minimum_required(VERSION 3.8)
project(robomas_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msg 生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/MotorCmdArray.msg"
  DEPENDENCIES std_msgs
)

add_executable(robomaster_ros2_ctrl src/robomaster_ros2_ctrl.cpp)
ament_target_dependencies(robomaster_ros2_ctrl rclcpp)
rosidl_target_interfaces(robomaster_ros2_ctrl ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS robomaster_ros2_ctrl DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
