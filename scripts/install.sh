##### ROS INSTALLATION #####
echo "Installing ROS 2 - Jazzy"

# Set the locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources

# Ensure Ubuntu Universe repository is enabled
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# Add the ROS 2 GPG key
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to the sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Packages
sudo apt update
sudo apt upgrade -y
sudo apt install ros-jazzy-desktop -y
echo "ROS 2 - Jazzy successfully installed"

# #### INSTALL PYTHON DEPENDENCIES ####
echo "Installing Python dependencies..."

# Install pip
sudo apt -y install python3-pip

# Install colcon
sudo apt install python3-colcon-common-extensions -y

# Install python packages
sudo apt install python3-crccheck -y
sudo apt install python3-gpiozero -y

# Install ffmpeg for cameras
sudo apt install ffmpeg -y

# Install v4l2
sudo apt install v4l-utils
