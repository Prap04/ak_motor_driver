cmake_minimum_required(VERSION 3.16)
project(ak_motor_driver LANGUAGES C CXX)
#This tells the cmake the linker language is C++ for the the generated targets during export and the minimum version of cmake required to build this project is 3.16.

option(AK_BUILD_ROS2 "Build the ros2_control plugin" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1. Core bare-metal C++ driver library
add_library(cubemars_driver src/cubemars.cpp)
target_include_directories(cubemars_driver PUBLIC include)
target_compile_options(cubemars_driver PRIVATE -Wall -Wextra -Wpedantic)

# 2. ROS message and service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCommand.msg"
  "msg/MotorCommandArray.msg"
  "msg/MotorState.msg"
  "msg/MotorStateArray.msg"
  "srv/EnterMitMode.srv"
)

# 3. Standalone C++ test executable
add_executable(test_cubemars_protocol test/test_cubemars_protocol.cpp)
target_link_libraries(test_cubemars_protocol PRIVATE cubemars_driver)
target_include_directories(test_cubemars_protocol PRIVATE include)

# Standalone real-motor test; use only after verifying the CAN wiring and power.
add_executable(ak10_9_velocity_test test/ak10_9_velocity_test.cpp)
target_link_libraries(ak10_9_velocity_test PRIVATE cubemars_driver)
target_include_directories(ak10_9_velocity_test PRIVATE include)

# 4. ROS node that wraps the driver with a common message type.
add_executable(ak_motor_driver_node src/ak_motor_driver_node.cpp)
target_link_libraries(ak_motor_driver_node cubemars_driver ${PROJECT_NAME}__rosidl_typesupport_cpp)
ament_target_dependencies(ak_motor_driver_node rclcpp)
# 5. ROS 2 Control Plugin target
if(AK_BUILD_ROS2)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)

  add_library(cubemars_ros2_control SHARED src/cubemars_ros2_control.cpp)
  target_link_libraries(cubemars_ros2_control cubemars_driver)

  ament_target_dependencies(
    cubemars_ros2_control
    rclcpp
    hardware_interface
    pluginlib
  )

  pluginlib_export_plugin_description_file(hardware_interface cubemars_plugin.xml)
endif()

install(TARGETS cubemars_driver DESTINATION lib) 
install(TARGETS ak_motor_driver_node DESTINATION lib/${PROJECT_NAME})
if(AK_BUILD_ROS2)
  install(TARGETS cubemars_ros2_control DESTINATION lib)
endif()
install(DIRECTORY include/ DESTINATION include)
install(FILES cubemars_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

enable_testing()
add_test(NAME cubemars_protocol COMMAND test_cubemars_protocol)

ament_package()
