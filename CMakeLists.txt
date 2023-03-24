cmake_minimum_required(VERSION 3.5)
project(servo_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra) # -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(soem_ros2 REQUIRED)

include_directories(
  include  
)

install (DIRECTORY include/
  DESTINATION include
)

unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )

add_executable(servo_controller src/servo_controller.cpp)
ament_target_dependencies(servo_controller
 rclcpp
 std_msgs
 sensor_msgs
 soem_ros2)

install(TARGETS servo_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(std_msgs soem_ros2)
ament_package()

