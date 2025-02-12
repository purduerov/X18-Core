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

### Setup ROS2 Workspace (where all ROS2 files will be located)
1. Create a directory using  ```mkdir ros2_ws```
2. Enter the ```ros2_ws``` using ```cd ros2_ws```
3. Create another directory using ```mkdir src```
4. Enter src directory and clone this repository inside src using ```git clone _ssh-link_``` (Might need to setup an SSH key for the raspberry pi)
5. Go back to ```ros2_ws``` directory using ```cd ..```

### Prepare code for execution
1. Enter the ```ros2_ws``` using ```cd ros2_ws```
2. Run the command ```source /opt/ros/humble/setup.bash``` (Note ROS2 needs to be installed https://docs.ros.org/en/humble/Installation.html)
3. Run the command ```colcon build``` (If colcon not installed, run ```sudo apt install python3-colcon-common-extensions```)
4. Once completed without errors, run ```. install/setup.bash```

### Running code with ROS2
1. Follow all the steps in Prepare code for execution
2. To run a specific file, run the command ```ros2 run _project-name_ _file-name_``` (Multiple files can be ran at once, each one requires a new terminal window)
3. To publish to a topic, run the command ```ros2 topic pub /_topic-name_ shared_msgs/_msg-name_ "_message-structure-with-values_"``` (Example shown below)

Example publisher: ```ros2 topic pub /final_thrust shared_msgs/FinalThrustMsg "thrusters: [127,127,127,127,127,127,127,127]"```

### ROS2 file structure 
Under the root of the repository, there will be different folders which are packages. Packages are used to group files with together, an example is the ```thrust_control``` folder. Each package directory will have launch folder, src folder, a folder with the same name as the package, a ```CMakeLists.txt```, and a ```package.xml``` file. The folder with the package name will have just an init.py file that is empty. The src folder will contain all of the python files that we have created containing the source code. The launch folder contains a yaml file that is used to launch the different files automatically. The ```package.xml``` file contains dependencies and information regarding the package. The ```CMakeLists.txt``` is what is used to compile all the files. All of the files location in the src folder should be added to the install(PROGRAMS ... ) section. An example is shown for the ```thrust_control``` file below. The ```shared_msgs``` is where all of the topic message structures are located. There is a msg folder containing a ```.msg``` file for each message which has the data structures used for each message. Each message file needs to be included in the ```CMakeLists.txt``` within the ```shared_msgs``` folder for ROS2 to find the messages.  

Example ```CMakeLists.txt``` install section: 

install(PROGRAMS
  src/ROV_main.py
  src/packets.py
  src/test_thrust_spi.py
  src/thrust_control.py
  src/thrust_mapping.py
  src/thrust_to_spi.py
  src/thrust_to_uart.py
  DESTINATION lib/${PROJECT_NAME}
)

## Communication Protocol

X17-Core uses UART for its communication protocol between the Raspberry Pi and the STMs on the electrical boards. There are four different message types. 
- Thrust data message
- ESC data message
- Power data message
- Tools data message

The UART protocol used will not have parity bits enabled. It will be running at a baud rate of 9600. Each message has a device ID which is the ID of the destination of the message. Each message will also have CRC which uses the CRC-32 MPEG2 polynomial, however, we only save the last 8-bits of the 32-bit CRC value. There are two UART lines, one for the tools, and one for the thrusters/ESC/power bricks. The acknowledge will be of the same structure as the data messages. 
