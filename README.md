# OccupancyGridMapping

Introduction
Maps are one of inputs to path planning algorithgms. There are two types of maps - Occupancy grid map and Feature based map.
Feature based map - This maps are created by mountinmg sensors (lidar, camera etc) on the robot to extract information from the environmenmt.
We can extract features from the captured image by the various feature extraction techniques.

Occupancy grid map- This map is created ny discretizing the environment to the required resolution and occupy the locations where mobile robot sees there are some objects over the environment.
Occupancy grid maps are inputs to path planning algorithgms.

Repository to create occupancy grid map from rosbag file containing lidar and wheel odometry data.
