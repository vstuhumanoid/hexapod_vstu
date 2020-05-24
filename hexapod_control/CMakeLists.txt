cmake_minimum_required(VERSION 2.8.3)
project(hexapod_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  genmsg
)

catkin_python_setup()

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hexapod_control
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/hexapod_control_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node hexapod_control_generate_messages_cpp)