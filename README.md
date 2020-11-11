### CAML Gazebo Sim
This simulator is built on the RotorS simulator with modifications made to allow for live-calculated dynamic wind.

## Installation Instructions - Ubuntu 18.04 with ROS Melodic
1. Install and initialize ROS Melodic desktop full, additional ROS packages, and catkin-tool using the ros wiki http://wiki.ros.org/melodic/Installation/Ubuntu
2. Source your catkin repository then clone the caml_sim repository into your catkin workspace and build with "catkin build"

## Basic Usage
Launch the simulator using 
```
$ roslaunch caml_gazebo valley_world.launch
```
Publishing to the topic
```
$ techpod/command/roll_pitch_yawrate_thrust
```
will command the UAV.
A simplistic GUI control sim can be launched with
```
$ roslaunch caml_gazebo valley_world_GUI_control.launch
```

## Wind Specification
Modifying the "wind_dynamic_plugin_macro" under the rotors_simulator/rotors_description/urdf/techpod_base.xacro" will change the calculated wind. 