cmake_minimum_required(VERSION 2.8.3)
project(hexapod_msgs)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation)

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    GaitParams.msg
    FrundControl.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

