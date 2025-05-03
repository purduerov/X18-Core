#!/bin/bash

# Find all ROS 2 processes
PIDS=$(pgrep -f ros2)

if [ -z "$PIDS" ]; then
    echo "No ROS 2 nodes found."
    exit 0
fi

# Kill each ROS 2 process
echo "Killing ROS 2 nodes: $PIDS"
kill $PIDS

# Wait and check if processes are still running
sleep 2
PIDS_STILL_RUNNING=$(pgrep -f ros2)

if [ -n "$PIDS_STILL_RUNNING" ]; then
    echo "Some ROS 2 nodes are still running, force killing..."
    kill -9 $PIDS_STILL_RUNNING
else
    echo "All ROS 2 nodes stopped successfully."
fi
