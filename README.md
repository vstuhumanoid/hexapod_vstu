# hexapod_description
This is modified version of [phantomx by  HumaRobotics](https://github.com/HumaRobotics/phantomx_description)

Changes:
 - added simple computer vision which allows robot move to the colored object
 - model was replaced with hexapod robot from VSTU university project
 - all repos (gazebo, description, control) are merged in one

## Install

Clone this repo and all dependencies in your catkin workspace and catkin_make it.

Dependencies:
 - [opencv-mono-detector](https://github.com/Garrus007/opencv-mono-detector)

## Usage

Launch Gazebo simulation, Computer Vision and walking to the colored object:
```bash
$ roslaunch hexapod_description gazebo.launch
```

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
