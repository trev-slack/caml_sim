# CAML Gazebo Sim
This simulator is built on the RotorS simulator with modifications made to allow for live-calculated dynamic wind.

## Installation Instructions - Ubuntu 18.04 with ROS Melodic
1. Install and initialize ROS Melodic desktop full, additional ROS packages, and catkin-tool using the ros wiki http://wiki.ros.org/melodic/Installation/Ubuntu
2. Source your catkin repository then clone the caml_sim repository into your catkin workspace and build with "catkin build"

## Basic Usage
Launch the simulator using 
```
$ roslaunch caml_gazebo valley_world.launch
```
Publishing to the following topic will command the UAV.
```
$ techpod/command/roll_pitch_yawrate_thrust
```
A simplistic GUI control sim can be launched with
```
$ roslaunch caml_gazebo valley_world_GUI_control.launch
```

## Wind Specification
Modifying the "wind_dynamic_plugin_macro" under the rotors_simulator/rotors_description/urdf/techpod_base.xacro" will change the calculated wind. Currently wind is specified by sampling normal distributions, but this will be updated to a dryden turbulance model for the turbulance and 1-cosine model for the gusts.

## Parallelization
It is possible to allocate more threads to the gazebo sim using the valley_world.world file found under "rotors_simulator/caml_gazebo/worlds/valley_world.world". Add more threads by modifing
```
$</solver>
  <thread_position_correction>1</thread_position_correction>
  <island_threads>3</island_threads>
</solver>
```