# X17-Core

Contains all ROS nodes for X-17 that run on a Raspberry Pi 4. The Raspberry Pi 4 acts as the intermediary between the laptop on the surface (WiFi network) and the electrical boards on the ROV (hardware connections). Computation does occur on the Raspberry Pi, but it should be limited due to power and heating constraints. This repository contains the code for thrust-related functionality, sensors, tools, and video processing. 

## General Repository 

### ROV Launch Commands

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

### Communication Protocol
X17-Core uses UART for its communication protocol between the Raspberry Pi and the STMs on the electrical boards. There are four different message types. 
- Thrust data message
- ESC data message
- Power data message
- Tools data message

The UART protocol used will not have parity bits enabled. It will be running at a baud rate of 9600. Each message has a device ID which is the ID of the destination of the message. Each message will also have CRC which uses the CRC-32 MPEG2 polynomial, however, we only save the last 8-bits of the 32-bit CRC value. There are two UART lines, one for the tools, and one for the thrusters/ESC/power bricks. The acknowledge will be of the same structure as the data messages. 

## ROS2

The Raspberry Pi's for X17-Core use ROS2 Humble. The end of life will be May 2027. Below is a list of ROS2 distributions and their life cycles:
https://docs.ros.org/en/foxy/Releases.html

### Setup ROS2 Workspace (where all ROS2 files will be located)
1. Create a directory using  ```mkdir ros2_ws```
2. Enter the ```ros2_ws``` using ```cd ros2_ws```
3. Create another directory using ```mkdir src```
4. Enter src directory and clone this repository inside src using ```git clone _ssh-link_``` (Might need to setup an SSH key for the raspberry pi)
5. Go back to ```ros2_ws``` directory using ```cd ..```

### Prepare code for execution (Needs to be done for each new terminal)
1. Enter the ```ros2_ws``` using ```cd ros2_ws```
2. Run the command ```source /opt/ros/humble/setup.bash``` (Note ROS2 needs to be installed https://docs.ros.org/en/humble/Installation.html)
3. Run the command ```colcon build``` (If colcon not installed, run ```sudo apt install python3-colcon-common-extensions```)
4. Once completed without errors, run ```. install/setup.bash```

If weird errors pop up during ```colcon build```, try deleting the ```build```, ```log```, and ```install``` directories in the ```ros2_ws``` directory and running the command again. Ensure that all files are in included in the ```CMakeLists.txt``` files. See below for more information on ROS2 file structures. 

### Running code with ROS2
1. Follow all the steps in Prepare code for execution (Needs to be repeated every time a change is made)
2. To run a specific file, run the command ```ros2 run _project-name_ _file-name_``` (Multiple files can be ran at once, each one requires a new terminal window)
3. To publish to a topic, run the command ```ros2 topic pub /_topic-name_ shared_msgs/_msg-name_ "_message-structure-with-values_"``` (Example shown below)

Example publisher: ```ros2 topic pub /final_thrust shared_msgs/FinalThrustMsg "thrusters: [127,127,127,127,127,127,127,127]"```

### ROS2 file structure 
Under the root of the repository, there will be different folders which are packages. Packages are used to group files with together, an example is the ```thrust_control``` folder. Each package directory will have launch folder, src folder, a folder with the same name as the package, a ```CMakeLists.txt```, and a ```package.xml``` file. The folder with the package name will have just an init.py file that is empty. The src folder will contain all of the python files that we have created containing the source code. The launch folder contains a yaml file that is used to launch the different files automatically. The ```package.xml``` file contains dependencies and information regarding the package. The ```CMakeLists.txt``` is what is used to compile all the files. All of the files location in the src folder should be added to the install(PROGRAMS ... ) section. An example is shown for the ```thrust_control``` file below. The ```shared_msgs``` is where all of the topic message structures are located. There is a msg folder containing a ```.msg``` file for each message which has the data structures used for each message. Each message file needs to be included in the ```CMakeLists.txt``` within the ```shared_msgs``` folder for ROS2 to find the messages.  

Example ```ros2_ws``` directory:

- src/
  - XYY-Core/
    - package_name/
      - launch/
        - package_name_launch.yml
      - src/
        - file_name1.py
        - file_name2.py
      - package_name/
        - init.py (Empty file, do not touch)
      - CMakeLists.txt
      - package.xml
    - shared_msgs/
      - msg/
        - message1.msg
        - message2.msg
      - CMakeLists.txt
      - package.xml
- install/
- log/
- build/

Example ```CMakeLists.txt``` install section: 

install(PROGRAMS <br />  
  src/ROV_main.py <br />
  src/packets.py <br />
  src/test_thrust_spi.py <br />
  src/thrust_control.py <br />
  src/thrust_mapping.py <br />
  src/thrust_to_spi.py <br />
  src/thrust_to_uart.py <br />
  DESTINATION lib/${PROJECT_NAME} <br />
)

## Raspberry Pi

The Raspberry Pi 4 on X17-Core uses the Ubuntu 22.04 LTS 64-bit Server Operating System. To change the OS or systems, flash the SD on the Raspberry Pi.

### Flashing/Reflashing SD Card
To flash/reflash a Raspberry Pi, you will need a micro SD with at minimum 16 GB. It is recommended to use an SD card with more space (X17 uses 64 GB). You will also require a device that has an SD card reader and has Raspberry Pi imager installed (version should not matter). Here is the link to install the Raspberry Pi imager: https://www.raspberrypi.com/software/. Insert the SD card into the device that write to it (may require an adapter). Choose the desired operating system, the current OS being used is Ubuntu 22.04 LTS 64-bit Server Operating System. You may need to look under ```Other general-purpose OS```. Choose the SD card as the storage device. It will prompt you to for customized settings, or you can click on the settings icon on the main page of the imager. Ensure that the ```Enable SSH``` is selected and that the username and password fields are set. For the ```Configure wireless LAN``` section, put in the name or SSID and password of the WiFi network. For X17, the name is ROS-2, and the password is ```yourmother```. The rest of the settings do not need to be configured unless desired. It may take a few minutes to write, but once done, the SD can be put into the Raspberry Pi SD slot when the Raspberry Pi is turned off. It may take a few minutes for the Raspberry Pi to initialize and connect to the WiFi network. Do not remove the SD card when the Raspberry Pi is operating. 

### SSH into a Raspberry Pi
The Raspberry Pi 4 should be turned on, indicated with a red LED by the USB-C power-in. Ensure that both the Raspberry Pi and the device that will be sshing into the Raspberry Pi are connected to the same WiFi network. You will need the IP address of the Raspberry Pi. To view all devices and their IP addresses on the current network, go to 192.168.1.1 in any browser. The username is admin and the password is Yourmother (very funny I know, but it's historial). Once logged into the network settings, go to the attached devices. There should be a list of devices and their IP addresses. To ssh into the Pi, run the command ```ssh pi@IP_ADDRESS```. Enter the password that the Raspberry Pi was given (should be ```pie``` for all). A direct ssh connection can be created through VS Code for an easier coding experience. 

### Creating an GitHub ssh key for Raspberry Pi
For the Raspberry Pi to interact with GitHub repositories, it requires a GitHub ssh key. This is the official guide for creating an ssh key on GitHub:
https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account

The key will be tied to someone's GitHub account, but all of the commands in the guide above will be done on the Raspberry Pi itself.
