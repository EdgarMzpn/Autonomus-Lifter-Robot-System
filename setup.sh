#!/bin/bash

# Check if ROS 2 Humble is already installed
if [ ! -d /opt/ros/humble ]; then
    echo "Installing ROS 2 Humble..."
    # Add ROS 2 apt repository
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list'
    # Set up keys
    sudo apt update && sudo apt install curl gnupg2 lsb-release
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    # Install ROS 2 Humble
    sudo apt update && sudo apt install ros-humble-desktop
    # Source ROS 2 setup file
    source /opt/ros/humble/setup.bash
    echo "ROS 2 Humble installed."
else
    echo "ROS 2 Humble is already installed."
fi

# Create a ROS 2 workspace
if [ ! -d ~/ros2_workspace ]; then
    echo "Creating ROS 2 workspace..."
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    source ~/ros2_ws/install/setup.bash
    echo "ROS 2 workspace created."
else
    echo "ROS 2 workspace already exists."
fi

# Additional libraries installation
echo "Installing tf transformations"
sudo apt update && sudo apt install ros-humble-tf-transformations
echo "ROS tf transformations installed."

# Add more library installation steps here as needed
# Gazebo Installation
echo "Installing Gazebo"
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-transmission-interface
echo "Gazebo is installed"


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

echo "Installation completed."


