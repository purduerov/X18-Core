#!/usr/bin/bash

. install/setup.bash
export ROS_DOMAIN_ID=69
ros2 run thrust_control thrust_to_uart.py --ros-args -r __ns:=/rov
