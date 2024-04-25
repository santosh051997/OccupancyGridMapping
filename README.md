# OccupancyGridMapping

# Introduction

![Screenshot from 2024-04-25 11-53-25](https://github.com/santosh051997/OccupancyGridMapping/assets/163696678/ba087bf8-e7d7-42e7-946e-85eb51c1d378)




Maps are one of inputs to path planning algorithgms. There are two types of maps - Occupancy grid map and Feature based map.
Feature based map - This maps are created by mountinmg sensors (lidar, camera etc) on the robot to extract information from the environmenmt.
We can extract features from the captured image by the various feature extraction techniques.

Occupancy grid map- This map is created ny discretizing the environment to the required resolution and occupy the locations where mobile robot sees there are some objects over the environment.
Occupancy grid maps are inputs to path planning algorithgms.

This repository is to create occupancy grid map from rosbag file containing lidar and wheel odometry data in ros2 environment.

# Usage

Create a custom world where you want to navigate your mobile robot.
Spawn the mobile robot with virtual lidar sensor attached to the robot body in the custom world in Gazebo.

Run the mobile robot with teleoperation (Use following command if use diff_drive_controller plugin in gazebo)

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Record the data from /odom and /scan topic in rosbag file.

ros2 bag record /scan /odom

Open the map_generator node from grid_mapping package. Change the path to your bag file in following line

with Reader('/path/to/your_bag_file') as reader:

Also chnage the name for your occupancy grid map here

MAP_NAME = BAG_FILE_NAME = ''

Run the map_generator node.

cd ~/occupancygrid_ws/

. install/setup.bash

ros2 run grid_mapping map_generator.py









