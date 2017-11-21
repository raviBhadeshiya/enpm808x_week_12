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
