## Install

Clone in your catkin workspace and catkin_make it.
Make sure you also have the following package in your workspace
* phantomx_control: https://github.com/HumaRobotics/phantomx_gazebo

## Usage

For working with /hexapod/ you need change /phantomx/ namespace in walker.py and walker_demo.py

You can launch the simulation with:

    roslaunch hexapod_description spawn.launch
    
PRESS PLAY IN GAZEBO ONLY WHEN EVERYTHING IS LOADED (wait for controllers)

You can run a walk demo with:

    rosrun phantomx_gazebo walker_demo.py

## ROS API

All topics are provided in the /hexapod namespace.

Sensors:

    /hexapod/joint_states

Actuators (radians for position control, arbitrary normalized speed for cmd_vel):

    /hexapod/cmd_vel
    /hexapod/j_c1_lf_position_controller/command
    /hexapod/j_c1_lm_position_controller/command
    /hexapod/j_c1_lr_position_controller/command
    /hexapod/j_c1_rf_position_controller/command
    /hexapod/j_c1_rm_position_controller/command
    /hexapod/j_c1_rr_position_controller/command
    /hexapod/j_thigh_lf_position_controller/command
    /hexapod/j_thigh_lm_position_controller/command
    /hexapod/j_thigh_lr_position_controller/command
    /hexapod/j_thigh_rf_position_controller/command
    /hexapod/j_thigh_rm_position_controller/command
    /hexapod/j_thigh_rr_position_controller/command
    /hexapod/j_tibia_lf_position_controller/command
    /hexapod/j_tibia_lm_position_controller/command
    /hexapod/j_tibia_lr_position_controller/command
    /hexapod/j_tibia_rf_position_controller/command
    /hexapod/j_tibia_rm_position_controller/command
    /hexapod/j_tibia_rr_position_controller/command
