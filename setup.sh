#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

# Logging
LOG_FILE="install_ros2.log"
exec > >(tee -a "$LOG_FILE") 2>&1

# Check if ROS 2 Humble is already installed
if [ ! -d /opt/ros/humble ]; then
    echo "Installing ROS 2 Humble..."
    # Setup the Resources
    # Ensure that the Ubuntu Universe repository is enabled
    sudo apt update && sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    
    # Now add the ROS2 GPG key with apt
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    # Add the repository to the sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    #Installing the ROS 2 packages
    sudo apt update && sudo apt upgrade -y

    # Install ROS 2 Humble
    sudo apt install -y ros-humble-desktop

    # Source ROS 2 setup file
    source /opt/ros/humble/setup.bash
    echo "ROS 2 Humble installed."
else
    echo "ROS 2 Humble is already installed."
fi

# Create a ROS 2 workspace
if [ ! -d ~/ros2_ws ]; then
    echo "Creating ROS 2 workspace..."
    source /opt/ros/humble/setup.bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build
    source ~/ros2_ws/install/local_setup.bash
    source ~/ros2_ws/install/setup.bash
    echo "ROS 2 workspace created."
else
    echo "ROS 2 workspace already exists."
fi

# Additional libraries installation
echo "Installing tf transformations"
sudo apt update && sudo apt install -y ros-humble-tf-transformations
echo "ROS tf transformations installed."
echo "Installing transforms3d"
pip install transforms3d
echo "Installation complete"

# Installing ROS2 Lidar package
echo "Installing the ROS2 Lidar package"
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ./install/setup.bash
cd ~/ros2_ws/src/sllidar_ros2
source scripts/create_udev_rules.sh
cd ~/ros2_ws
echo "Installation for ROS2 complete"

# Installing the slam toolbox package
echo "Installing Slam Toolbox"
sudo apt install -y ros-humble-slam-toolbox
echo "Finished installing Slam Toolbox"

#Installing Nav2 requiring packages
echo "Installing Nav2 missing packages"
sudo apt install -y ros-humble-nav2-bringup
echo "Installation completed."
