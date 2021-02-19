# Gazebo Simulator
Simulator for ALPACA Project

## Installing
Step 1: Install ROS Noetic (desktop full) http://wiki.ros.org/noetic/Installation/Ubuntu

Step 2: Create a workspace http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Step 3: Install python3 catkin tools
```
$ sudo apt install python3-catkin-tools python3-osrf-pycommon
```
Step 4: Clone mav_comm and octomap_ros into your workspace/src,
```
$ git clone https://github.com/ethz-asl/mav_comm 
$ git clone https://github.com/OctoMap/octomap_ros
```

Step 5: Install ROS Joy, octomap_msgs, and octomap packages
```
$ sudo apt-get install ros-noetic-joy
$ sudo apt-get install ros-noetic-octomap
$ sudo apt-get install ros-noetic-octomap-msgs
```

Step 6: Clone this repo into your workspace

Step 7: Build with catkin build

Step 8: make files executable under rotors_simulator/caml_gazebo/resources: techpod_controller.py, rviz_wind_field.py


## Basic Usage
Launch the simulator using 
```
$ roslaunch caml_gazebo empty_world.launch
```
Publishing to the following topic will command the UAV.
```
/techpod/command/roll_pitch_yawrate_thrust
```
A simplistic GUI control sim can be launched with
```
$ roslaunch caml_gazebo valley_world_GUI_control.launch
```
A Battery node emits its status and level on the topic:
```
/techpod/battery_level
```
The battery length at max thrust can be changed in the launch file.
A Repeat tester node can be used to generate random trajectories. This can be run with:
```
rosrun caml_gazebo repeat_test_node.py <uav name> <number of trajectories>
```
Parameters of the random trajectory are adjustable in the repeat_test_node.py script.

## Wind Specification
Modifying the "wind_dynamic_plugin_macro" under the rotors_simulator/rotors_description/urdf/techpod_base.xacro" will change the calculated wind.
### Theory
The wind is comprised of three components: nominal, turbulence, and gusts. The nominal wind is obtained by sampling a gaussian distribution. The turbulence is calculated using the Dryden Turbulence Model and currently only supports altitudes < 1000 ft (see MIL-STD-1797A 1990 for a detailed description of the model and the standards). The gusts are modeled using a 1-cosine model with an exponential decay.
### Parameters: 
- namespace = robot namespace
- wind_direction_north_east = mean direction of the wind in x-y plane
- wind_direction_down = mean direction of the wind in the z direction
- wind_speed_mean = mean nominal wind speed
- wind_std_speed = standard deviation of the nominal wind speed
- wind_std_direction_north_east = standard deviation of the wind direction in the x-y plane
- wind_std_direction_down = standard deviation of the wind in the z direction
- wind_gust_mean = mean wind gust speed
- wind_gust_std = standard deviation of the wind gust speed
- wind_gust_length = mean length of a wind gust
- wind_gust_length_std = standard deviation of a wind gust length
- wind_gust_downtime = mean time between wind gusts
- wind_gust_downtime_std = standard deviation of the time between wind gusts
- wind_gust_decay = exponential decay factor for wind gusts
- wind_turbulence_mean = mean of the gaussian distribution fed to the dryden turbulence model
- wind_turbulence_std = standard deviation of the gaussian distribution fed to the dryden turbulence model

## Parallelization
It is possible to allocate more threads to the gazebo sim using the valley_world.world file found under "rotors_simulator/caml_gazebo/worlds/valley_world.world". Add more threads by modifing
```
</solver>
  <thread_position_correction>1</thread_position_correction>
  <island_threads>3</island_threads>
</solver>
```
Additional information on parallelization can be found at http://gazebosim.org/tutorials?tut=parallel&cat=physics.
