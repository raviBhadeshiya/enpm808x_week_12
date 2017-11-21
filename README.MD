[![Packagist](https://img.shields.io/packagist/l/doctrine/orm.svg)](LICENSE.md)
# enpm808x_week_12
 Programming Assignment: Working with Gazebo

*The Turtlebot will start running forward until it encounter the obstacle; then, it rotate in place until the path is cleared and move forward and it will repeat the same behavior.*
## Prerequisites
 * [ROS Kinetic](http://wiki.ros.org/ROS/Installation) on Ubuntu 16.04
 * [Gazebo](http://gazebosim.org/)
 * [TurtleBot](http://wiki.ros.org/Robots/TurtleBot)

## Standard install via command-line
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ cd src/
$ git clone --recursive https://github.com/raviBhadeshiya/enpm808x_week_12.git
$ cd ..
$ catkin_make
```
---
## Running the nodes
Running nodes requires you have a ROS core started. A ROS core is a collection of nodes and programs that are pre-requisites of a ROS-based system. You must have a roscore running in order for ROS nodes to communicate. Open a new shell, and type:
```
$ roscore
```
###### Node
First start the turtlebot_gazebo enviroment by following:
```
$ roslaunch turtlebot_gazebo turtlebot_world.launch 
```
And, walker node can be run by opening new shell, and type:
```
$ rosrun enpm808x_week_12 enpm808x_week_12_node
```

###### Launch
Or This ROS nodes and turtlebot_gazebo enviroment can be start with single launch file as follows:
```
$ roslaunch enpm808x_week_12 enpm808x_week_12.launch
```
>Note: roslaunch will automatically start roscore if it detects that it is not already running.

## Rosbag

Bags are the primary mechanism in ROS for data logging, which means that they have a variety of offline uses. To record the rosbag:
```
$ roslaunch enpm808x_week_12 enpm808x_week_12.launch record:=true
```
>Note: 35 sec is the maximum duration of the recorded bag file. The bag file will be stored in the result directory.

These bag files can also be played back in ROS to the same topics they were recorded from, or even remapped to new topics. The rosbag play reads the contents of bag file, and plays them back in a time-synchronized fashion. To run the demo:
```
$ roscd enpm808x_week_12/
$ rosbag play result/walker.bag
```
> Note: While ros bag file playback, Gazebo should not be running.

To inspect the bag-file as follow:
```
$ rosbag info result/walker.bag
```
>Note: It can be observed that /camera/* topics are not being recorded, which are related to the RGB-D sensor and will quickly increase bag file size.

Output should look like this:
```
path:        result/walker.bag
version:     2.0
duration:    34.9s
start:       Dec 31 1969 19:00:00.23 (0.23)
end:         Dec 31 1969 19:00:35.16 (35.16)
size:        13.5 MB
messages:    26228
compression: none [18/18 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            3494 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/input/navi                            332 msgs    : geometry_msgs/Twist                  
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               3485 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              3483 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     3372 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     68 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     329 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     3372 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  136 msgs    : bond/Status                           (3 connections)
             /odom                                             3381 msgs    : nav_msgs/Odometry                    
             /rosout                                             63 msgs    : rosgraph_msgs/Log                     (9 connections)
             /rosout_agg                                         49 msgs    : rosgraph_msgs/Log                    
             /scan                                              329 msgs    : sensor_msgs/LaserScan                
             /tf                                               4327 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage
```
