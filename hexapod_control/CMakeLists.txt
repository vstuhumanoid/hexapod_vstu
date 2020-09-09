cmake_minimum_required(VERSION 2.8.3)
project(hexapod_control)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  genmsg
  hexapod_msgs
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

add_executable(
  hexapod_control_node 
  src/hexapod_control_node.cpp 
  src/hexapod_controller.cpp
  src/frund_gateway.cpp)
  
target_link_libraries(hexapod_control_node ${catkin_LIBRARIES})
add_dependencies(hexapod_control_node hexapod_control_generate_messages_cpp)