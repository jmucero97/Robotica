#!/bin/bash

source devel/setup.bash

export TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/empty.world

roslaunch turtlebot_gazebo turtlebot_world.launch
