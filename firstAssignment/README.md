# Research Track 1 - Assignmnt 1
This is the README.MD file for the first assignment of the course Research Track 1.

## AUTHOR 👨‍💻
* [Enrico Piacenza](https://github.com/EnricoPiacenza) (s4878469)

## INDEX 🔖

* [Introduction](#introduction)
* [Simulation](#simulator)
* [video](#video)
* [Nodes](#nodes)

<div id='introduction'/>

## INTRODUCTION 💻📝
In a complex environment simulated in Gazebo, five markers with different IDs are placed. (look [https://github.com/CarmineD8/aruco_ros](https://github.com/CarmineD8/aruco_ros). 
The goal is to develop a ROS2 package where a mobile robot is able to identify all markers, reordering them in ascending order and save a photo for each marker with a circle highlighting its position inside it.

<div id='simulator'/>

## SIMULATION 🏁
This assignment uses Gazebo and Rviz within ROS2. Gazebo served as the 3D simulation environment to test and refine the robot's movements, while Rviz was used for detailed 3D visualization, with also the information of the robot's sensors. 
After cloning the package, in order to correctly launch the package the following command must be executed:

```bash
ros2 launch robot_urdf gazeboRobot.launch.py
```

<div id='video'>
## Video 🎬

[![Video of the run](https://img.youtube.com/vi/KSw2tHTPgws/hqdefault.jpg)](https://youtu.be/KSw2tHTPgws)

<div id='nodes'/>
 
## Nodes 🪢

**moveRobot**




