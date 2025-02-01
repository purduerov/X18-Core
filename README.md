# X17-Core
Contains all ROS nodes for X-17 that run on Raspberry Pi 4

The launch file can be found in the `rov_launch` folder. The launch file is called `core_launch.yaml`

The launch file can be run using the following command:
```
ros2 launch rov_launch core_launch.yaml
```

If you need to build and source the workspace, you can use the following commands:
```
colcon build
source install/setup.bash
```

## ROS2 Commands

On the device to run X17-Core (typically the Raspberry Pi), create a directory called ```ros2_ws```, within this work space create another directory called ```src```. Inside the ```src``` directory, clone this repository. While inside the ```ros2_ws``` directory, run the command ```source /opt/ros/humble/setup.bash```. ROS2 (Humble) needs to be installed for this work; the installation commands are found in https://docs.ros.org/en/humble/Installation.html. The source command will "initialize" ROS2 on the device. From there, run ```colcon build``` which will create the executables that will run. After that completes, run the command ```. install/setup.bash```. To run specific files, do the command ```ros2 run _project-name_ _file-name_```. To have multiple files running, you can open multiple terminals and run that command for each file. 

## Communication Protocol

X17-Core uses UART for its communication protocol between the Raspberry Pi and the STMs on the electrical boards. There are four different message types. 
- Thrust data message
- ESC data message
- Power data message
- Tools data message

The UART protocol used will not have parity bits enabled. It will be running at a baud rate of 9600. Each message has a device ID which is the ID of the destination of the message. Each message will also have CRC which uses the CRC-32 MPEG2 polynomial, however, we only save the last 8-bits of the 32-bit CRC value. There are two UART lines, one for the tools, and one for the thrusters/ESC/power bricks. The acknowledge will be of the same structure as the data messages. 
