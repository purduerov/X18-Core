#!/usr/bin/bash


source /opt/ros/jazzy/setup.bash
colcon build
. install/setup.bash
export ROS_DOMAIN_ID=69
