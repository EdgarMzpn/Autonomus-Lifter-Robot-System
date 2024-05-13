cmake_minimum_required(VERSION 3.5)
project(puzzlebot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(transmission_interface REQUIRED)

# uncomment the following section in order to fill in further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add message and service dependencies
# Uncomment and adjust if needed
# find_package(<message_package> REQUIRED)
# find_package(<service_package> REQUIRED)

# Declare ROS2 message and service generation
# Uncomment and adjust if needed
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Arucoinfo.msg"
#   DEPENDENCIES geometry_msgs std_msgs
# )

# Declare ament package
ament_package()

# Include directories
include_directories(include)

# Add your source files and link dependencies
# Example:
# add_executable(${PROJECT_NAME}_node src/puzzlebot_gazebo_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs ...)
# target_link_libraries(${PROJECT_NAME}_node <libraries>)

# Install rules
# Example:
# install(TARGETS ${PROJECT_NAME}_node
#   DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)


# Testing - if needed
# catkin_add_gtest(${PROJECT_NAME}-test test/test_puzzlebot_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
