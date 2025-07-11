# Experimental_Robotics_Assignment2
This is the README.MD file for the second assignment of the course Experimental Robotics.

## AUTHORS 👨‍💻
* [Nicolas Bravi](https://github.com/NicolasBravi01) (s6402442)
* [Ilaria Colomba](https://github.com/ilacolo) (s4829201)
* [Enrico Piacenza](https://github.com/EnricoPiacenza) (s4878469)

## INDEX 🔖

* [Introduction](#introduction)
* [Simulation](#simulator)
* [Install and Running](#install-and-running)
* [Pddl](#pddl)
* [Mapping and Navigation](#mapping)
* [Nodes](#nodes)

<div id='introduction'/>

## INTRODUCTION 💻📝
In a complex environment simulated in Gazebo, four markers with different IDs are placed. (look [https://github.com/CarmineD8/aruco_ros](https://github.com/CarmineD8/aruco_ros). In order to capture the images that for the scanning of the marker for ID's detection, the following waypoints are given:
 
* WP 1: x = 6.0, y = 2.0 (Marker 11)
* WP 2: x = 7.0, y = -5.0 (Marker 12)
* WP 3: x = -3.0, y = -8.0 (Marker 13)
* WP 4: x = -7.0, y = 1.5 (Marker 15)
  
The goal is to develop a ROS2 package that lets a mobile robot endowed with a camera and a laser scanner move the robot to the waypoint corresponding to the lowest ID.
PlanSys2 is required to plan the robot's actions.

<div id='simulator'/>

## SIMULATION 🏁
This assignment uses Gazebo and Rviz within ROS2. Gazebo served as the 3D simulation environment to test and refine the robot's movements, while Rviz was used for detailed 3D visualization, with also the information of the robot's sensor.

<div id='install-and-running'/>

## INSTALL AND RUNNING 📖
First of all, you need to go into your ros2 workspace (folder src for packages is suggested)
```bash
cd /root/ros2_ws/src
```
Then, clone the repository to your machine (or download)
```bash
git clone https://github.com/EnricoPiacenza/Experimental_Robotics_Assignment2.git
```
Now, you can go back to the ros2_ws path and build
```bash
cd ..
colcon build
```
In order to correctly simulate the program, you need to open the following five terminals:
```bash
ros2 launch robot_urdf gazebo2.launch.py
```
This launch file is used to correctly set up the Gazebo and Rvizz2 simulations, the simulation world used is ```Assignment2.world``` and the robot model used is ```robot5.xacro```. In addition 3 other launch files are started:

```
ros2 launch slam_toolbox online_async_launch.py
```
Which initializes the slam algorithm.

```
ros2 launch nav2_bringup navigation_launch.py
```
Which initializes the navigation algorithm.

```
ros2 launch plansys2_patrol_navigation_example patrol_example_fakesim_launch.py
```
Which initializes the pddl domain file and the nodes created to excecute the durative actions.

With the following command the Problem is initialized.
```bash
ros2 run plansys2_patrol_navigation_example patrolling_controller_node
```





<div id='Video'/>

## Video 🎬

[![Video of the run](https://img.youtube.com/vi/yhsrMFM95Ao/hqdefault.jpg)](https://www.youtube.com/watch?v=yhsrMFM95Ao)

<div id='pddl'/>
 
## PDDL 🤖
 
PDDL stands for Planning Domain Definition Language and it is used to defining a standard encoding language for planning task. We use this to plan the robot's actions.
 
The planning domain file is defined in the repository (</plansys2_patrol_navigation_example>). This file specifies the types, the predicates and the durative actions necessary to make the robot able to accomplish the tasks of navigation to the different waypoints and to identify the marker IDs and localization.
 
Domain Elements:
1) Types:
- robot: it represents the subject that is moving in the environment.
- waypoint: it represents locations of a waypoint in the environment.
 
2) Predicates:
- (robot_at ?r ?wp): it indicates that the robot ?r is at waypoint ?wp.
- (connected ?wp1 ?wp2): Denotes a connection between positions of waypoint ?wp1 to waypoint ?wp2
- (patrolled ?wp - waypoint): it indicates if a waypoint ?wp has been patrolled
 
3) Durative Actions:
- move: it represents the action of the robot moving from the waypoint ?wp1 to the waypoint ?wp2.
- patrol: it represents the action of the robot performs patrolling in a waypoint ?wp.
 
For each action it is necessary to specify the parameters, the duration, the conditions and the effects of the action.
 
**Move action**

*Parameters:*
- ?r: The robot performing the action.
- ?wp1, ?wp2: The starting and destination waypoints.
 
*Duration:* 
The action takes 5 time units.
 
*Condition:* 
- At the start of the action, the two waypoints must be connected. 
- The robot must be at the starting waypoint.
 
*Effect:* 
- At the start of the action, the robot leaves ?wp1. 
- At the end of the action, the robot is at ?wp2.
 
---
 
**Patrol Action**
 
*Parameters:*
- ?r: The robot performing the patrol.
- ?wp: The waypoint to patrol.
 
*Duration:* 
The action takes 5 time units.
 
*Condition:* 
- At the start of the action, the robot must be at the waypoint.
 
*Effect:* 
- At the end of the action, the waypoint is patrolled.
 
Here's our domain file:
 
```pddl
(define (domain patrol)
(:requirements :strips :typing :adl :fluents :durative-actions)
 
;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;
 
;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates
 
(robot_at ?r - robot ?wp - waypoint)
(connected ?wp1 ?wp2 - waypoint)
(patrolled ?wp - waypoint)
 
);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions
 
);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?wp1 ?wp2))
        (at start(robot_at ?r ?wp1))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)
 
(:durative-action patrol
    :parameters (?r - robot ?wp - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?wp))
       )
    :effect (and
        (at end(patrolled ?wp))
    )
)
 
);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
```
 
The problem file is set inside the node `patrolling_controller_node.cpp`. 
First of all it is necessary to set the istances of objects: 
- robot: r2d2 is the robot which is moving and patrolling in the environment
- waypoints: wp_control, wp1, wp2, wp3, wp4.
 
At the initial state the robot starts at the inizial postion wp_control. It also needs to specify how the waypoints are connected to one another.
```pddl
  (robot_at r2d2 wp_control)
  (connected wp_control wp1)
  (connected wp1 wp2)
  (connected wp2 wp3)
  (connected wp3 wp4)
  (connected wp4 wp1)
  (connected wp4 wp2)
  (connected wp4 wp3)
```
 
At the end the goal is set: the goal of the robot is to reach each waypoint and executes the patrolling. 
```pddl
(and (patrolled wp1) (patrolled wp2) (patrolled wp3) (patrolled wp4))
```
<div id='mapping'/>

## Mapping and Navigation 🚗💨
 
### Mapping
In order to map the unknown environment, we decided to use the SLAM algorithm. Simultaneous Localization and Mapping (SLAM) is a method used for autonomous vehicles to build a map and localize themselves within it simultaneously. In order to allow its implementation, we decided to add the Lidar plugin on the robot model. This sensor is able to create a local map through a laser, detecting obstacles and the traversable areas of the environment during its navigation. While exploring the environment, the robot tries to build a global map of the environment based on the history of the local maps. This is the link of the package used: [https://github.com/CarmineD8/slam_toolbox](https://github.com/CarmineD8/slam_toolbox)
 
### Navigation
The navigation system works with action server able to navigate a robot in the environment. Also from the theoretical perspective, the ROS2 navigation stack is not very different from the ROS one The bringup_launch.py file loads the parameters stored in the nav2_params.yaml file, and starts the navigation_launch.py launch file which, on its turn, activates all action servers for managing the robot’s navigation. This is the link of the package used: [https://github.com/CarmineD8/navigation2](https://github.com/CarmineD8/navigation2)

<div id='nodes'/>
 
## Nodes 🪢

Below is a list of all nodes implemented to ensure the correct functionality of the system.

**Patrolling_controller_node**

This node was created to initialyze the PlanSys2 framework in order to correctly navigate in the enviroment and patrol the desired waypoints. Here's a list of everything the node does:
- Basic knowledge initialization (instances, predicates and connections).
- Goal definition: depending on the state the node is in, the controller is able to set a specific goal.

The Progess of the action is monitored by the periodic function step() using the status_ variable:
  - ```STARTING```: At the beginning of the task, the problem with the goal is defined. At this state, we do not know yet which is the marker with the lowest ID, so the goal is to scan all the four markers. As specified above, a marker can be scanned only if the mobile robot is at the relative point.
  - ```PATROL_FINISHED```: At this state, the controller defines the new robot's goal based on received marker IDs. The new goal is to reach the waypoint that corresponds to the marker with the lowest ID.
  - ```GO_BACK```: Handles returning to the waypoint of the marker with the lowest ID and cleans up predicates after completion.

**Move_action_node**

The node is designed to be the action executor within the ROS 2 environment for the durative action Move defined in the pddl domain. It serves as the bridge between the high-level planning specified in the PDDL file and the actual execution of the action on the robotic platform. It leverages the plansys2 framework for high-level planning and integrates with the nav2 stack for navigation. Here's a list of everything the node does:
- Initialize the structure waypoints_ containing the coordinates of the four markers to reach.
- Subscribes to the topic ```/odom``` in order to monitor the current position of the robot.

The Progess of the action is monitored by the periodic function do_work() using the status_ variable:
- ```status_ = 0```: creates the navigation action client, it exctract the correct waypoint to navigate to and send the newly formed goal to the navigation server.
- ```status_ = 1```: checks the robot distance to the goal position, if the error is under the chosen treshold of 0.3 the status_ is changed to 2.
- ```status_ = 1```: checks the robot distance to the goal position, if the error is under the chosen treshold of 0.3 the status_ is changed to 2.
- ```status_ = 2```: the action is considered terminated and, using the function finish(), the signal is sent to the PlanSys2 framework.


**Patrol_action_node**

The node is designed to be the action executor within the ROS 2 environment for the durative action patrol defined in the pddl domain. It serves as the bridge between the high-level planning specified in the PDDL file and the actual execution of the action on the robotic platform. Here's a list of everything the node does:
- It creates a SetBool client to trigger the execution of RobotNode in order to look for markers.
- It creates a SetBool service that is triggered by RobotNode once the Marker search is terminated.

The progress of the action is monitored by the periodic function do_work() using the status_ variable:
- ```status_ = 0```: an asynchronous call to RobotNode is done, using the SetBool client, the status_
- ```status_ = 1```: the node waits, the status_ variable is set to 2 in the service call back, triggered at the end of the excecution of RobotNode.
- ```status_ = 2```: the action is considered terminated and, using the function finish(), the signal is sent to the PlanSys2 framework.


**RobotNode**

This node uses OpenCV and ROS2 functionality to detect ArUco markers. Here's a list of everything the node does:
- It publishes on the topic ```/cmd_vel``` the desired robot velocity.
- It publishes on the topic ```/aruco_marker_id``` the position in the array marker_ids of the smallest marker detected.
- it subscribes to the topic ```/aruco_markers``` to receive detected marker IDs.
- It creates a SetBool client to send a signal to ```Patrol_action_node``` once the excecution of the node is finished.
- It creates a SetBool service that is triggered by ```Patrol_action_node``` and strats the excecution of the program.

The progress of the action is monitored by the periodic function run() using the status_ variable:
- ```status_ = 0```: no markers are detected, the robot rotates along its z axis, the status is upgraded to 2 in the /aruco_markers topic callback function once a marker, that isn't present in marker_ids, is found.
- ```status_ = 1```: the robot is stopped, an asynchronous call to Patrol_action_node is made to comunicate the termination of the action and if four markers are detected, the position in the array of the smallest is published on ```/aruco_marker_id```.
