# Drone-Obstacle-Avoidance---ROS
ROS code for Drone Obstacle Avoidance, in a Gazebo 3-D world simulation and Ardupilot SITL.

-------------------------------
### Video Demo:

https://github.com/SriramS-77/Drone-Obstacle-Avoidance-ROS/assets/123828917/8d603bad-fde8-4e28-8f27-de20a435fa65

-------------------------------
### Aim:
This project imlements obstacle avoidance and path-planning in drones, using lidar input to compute the available paths, and using a curated algorithm to find the shortest and most efficient path to the target point. The algorithm employs informed search techniques, based on a heuristic.

-------------------------------
### Algorithm Used:
This project uses a hybrid path-planning algorithm, made from tangent-bug, steepest hill climbing and A-star search. The heuristic value is computed by first finding the local coordinates of the perceived tangent point using the lidar readings, and then calculating the euclidean distance from the target point.

-------------------------------
### Code:
The code is written in C++. It can be found under iq_gnc/src/iq_gnc/test0.cpp and iq_gnc/src/iq_gnc/test1.cpp

-------------------------------
### Lidar and World:
The 3-D world is created by a the xml-formatting files under iq_sim/worlds. The files runway*.world can be used to test the ROS code and algorithm.

The LiDAR used is 360 degree lidar with 720 rays, which are ordered from the right hand side of the drone, in the anti-clockwise direction.

-------------------------------
### ROS Environment:
This project was made in ROS-1 (noetic), and uses MAVROS plugins to access the IMU data, as well as control the drone through publishers and subscribers.

-------------------------------
