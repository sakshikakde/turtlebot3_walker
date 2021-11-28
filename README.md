[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
---
# turtlebot3_walker
## Introduction
This repository provides a simple for obstacle avoidance

## File Structure
- include       
-- walker.hpp    

- src    
-- walker.cpp    
-- walker_node.cpp    
    

walker_node.cpp is  where the object for the classes Walker is created.

## Building the package
1) Create a catkin workspace catkin_ws

2) Clone the turlebot3 package inside catkin_ws/src using 

``` 
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git     
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git       
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Refer [link](https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/).

4) Clone the turtlebot3_walker package
```
git clone https://github.com/sakshikakde/turtlebot3_walker.git

```
4) Change the directory
```
cd catkin_ws

```
5) run 

``` 
catkin build 
```

## How to run the code
1) Change the directory 

``` 
cd catkin_ws

```
2) Source the workspace

```
source devel/setup.bash
```
3) In a terminal, run 
```
roscore
```
5) In a new terminal, run the talker by using

```
 roslaunch turtlebot3_walker turtlebot3_obstacle_avoidance.launch 
```


### Parameters
- platform_name : default="turtlebot3"
- publisher_topic_name :  default="/cmd_vel"
- subscriber_topic_name : default="/scan"
- publisher_rate : default = 20
- scan_range :  default="10"
- distance_threshold : default="1"
- bags_directory : default="/home"
- record_bag : default="false"
- launch_gazebo : default="true"



## Recording the bag file
Set the record_bag param true in the launch file. Set the bags_directory as well.
## Running the bag file
1) Change the directory to the location where rosbag is recorded
 
2) Run the command

```
rosbag play <bagfile_name> --pause
```

3) Launch the file turtlebot3_obstacle_avoidance.launch with gazebo off
```
roslaunch turtlebot3_walker turtlebot3_obstacle_avoidance.launch launch_gazebo:=false
```

## Bag file
https://drive.google.com/file/d/1rZaai8yFOqWnMEujxXBu6TGUDE9_OtfZ/view?usp=sharing