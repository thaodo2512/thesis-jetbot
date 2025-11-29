#!/bin/bash

# Script to manage ROS Noetic Docker container for Jetbot/DDPG simulation
# Usage: ./run.sh [--init | --run]
# --init: First-time setup (create virtual env, install packages, build workspace)
# --run: Regular run (launch Gazebo and DDPG)

# Docker image name
IMAGE_NAME="my_ros_noetic_full"

# Common docker run options (mount catkin_ws, GUI support, etc.)
DOCKER_RUN_BASE="docker run -it --rm \
  --net=host \
  --env DISPLAY \
  --volume /etc/group:/etc/group:ro \
  --volume /etc/passwd:/etc/passwd:ro \
  --volume /etc/shadow:/etc/shadow:ro \
  --volume /etc/sudoers.d:/etc/sudoers.d:ro \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume $HOME/catkin_ws:/root/catkin_ws \
  --device /dev/dri \
  --name ros_noetic \
  $IMAGE_NAME"

# Function for init (first time)
init_setup() {
  echo "Running first-time init setup..."
  $DOCKER_RUN_BASE bash -c "
    source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    /usr/local/bin/python3.7 -m venv ddpg_env && \
    source ddpg_env/bin/activate && \
    pip install --upgrade pip setuptools wheel && \
    pip install tensorflow==1.15.0 Keras==2.3.1 numpy==1.16.6 && \
    deactivate && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make_isolated --install && \
    echo 'source /root/catkin_ws/install_isolated/setup.bash' >> ~/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
  "
  echo "Init setup complete. Virtual env and build persisted in mounted catkin_ws."
}

# Function for regular run
regular_run() {
  echo "Running regular simulation..."
  $DOCKER_RUN_BASE bash -c "
    source /opt/ros/noetic/setup.bash && \
    source /root/catkin_ws/install_isolated/setup.bash && \
    source ~/.bashrc && \
    cd /root/catkin_ws && \
    source ddpg_env/bin/activate && \
    roslaunch jetbot_gazebo spawn_jetbot.launch world_name=\$(rospack find turtlebot_ddpg)/worlds/turtlebot3_modified_corridor2.world laser_enable:=true realsense_enable:=false sonar_enable:=false && \
    cd src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts/original_ddpg && \
    rosrun turtlebot_ddpg ddpg_network_turtlebot3_original_ddpg.py
  "
}

# Parse arguments
case "$1" in
  --init)
    init_setup
    ;;
  --run)
    regular_run
    ;;
  *)
    echo "Usage: $0 [--init | --run]"
    exit 1
    ;;
esac
