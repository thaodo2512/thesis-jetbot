#!/bin/bash

# Update package list
apt update

# Install Python 3.7 for TensorFlow compatibility
apt install -y software-properties-common
add-apt-repository ppa:deadsnakes/ppa -y
apt update
apt install -y python3.7 python3.7-venv python3.7-distutils

# Initialize rosdep if not done
rosdep init || true  # Ignore if already initialized
rosdep update

# Install dependencies from src packages
rosdep install --from-paths src --ignore-src -r -y

# Install additional ROS Noetic packages
apt install -y \
  ros-noetic-joy \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-teleop-twist-joy \
  ros-noetic-laser-proc \
  ros-noetic-rgbd-launch \
  ros-noetic-depthimage-to-laserscan \
  ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python \
  ros-noetic-rosserial-server \
  ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs \
  ros-noetic-amcl \
  ros-noetic-map-server \
  ros-noetic-move-base \
  ros-noetic-urdf \
  ros-noetic-xacro \
  ros-noetic-compressed-image-transport \
  ros-noetic-rqt-image-view \
  ros-noetic-gmapping \
  ros-noetic-navigation \
  ros-noetic-interactive-markers \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-image-view \
  ros-noetic-rviz

# Clean up
apt autoremove -y
apt clean

echo "Installation complete. Now create the virtual env and build the workspace."
