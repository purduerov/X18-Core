#!/bin/bash

# Define directories to be removed
directories=("log" "install" "build")

# Loop through each directory and remove it if it exists
for dir in "${directories[@]}"; do
    if [ -d "$dir" ]; then
        echo "Removing directory: $dir"
        rm -rf "$dir"
    else
        echo "Directory $dir does not exist."
    fi
done

source /opt/ros/humble/setup.bash

echo "Cleanup complete."