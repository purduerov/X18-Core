#!/usr/bin/bash

. install/setup.bash
export ROS_DOMAIN_ID=69
ros2 launch rov_launch core_launch.yaml
