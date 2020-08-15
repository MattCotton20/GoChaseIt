# Go Chase It!

Submission for project #2 within the Udacity Robotics Software Engineer nanodegree.

## Description

A simple two-wheeled robot designed to chase a white ball around its environment. The robot is built using the ROS framework and simulated using the Gazebo simulator.

## Getting Started
#### Clone repo into catkin workspace:
```
$ git clone git@github.com:MattCotton20/GoChaseIt ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
#### Initialise the robot and environment:
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```
This will start instances of RViz and Gazebo, containing a building interior, robot, and white ball that the robot will chase.
#### Switch on the robot:
In a second terminal, run:
```
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```
The robot will begin to chase the white ball and will stop once it reaches it. Manually move the white ball within Gazebo to continue the chase.

## Author

Matt Cotton
