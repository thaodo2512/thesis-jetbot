# JetBot & TurtleBot3 DDPG Collision Avoidance

This project aims to create a reproducible, end-to-end simulated environment for training and testing a Jetbot robot using the Deep Deterministic Policy Gradient (DDPG) algorithm for mapless collision avoidance. It provides a robust pipeline for robot navigation development, enabling testing and learning without real hardware risks.

## Project Purpose

The core objectives of this project are:

-   **Integration of Repositories**: Combines the `jetbot_diff_drive` repository (providing the Jetbot's ROS model, Gazebo simulation, and sensor plugins including RPLIDAR and IMX219 camera) with `turtlebot3_ddpg_collision_avoidance` (the DDPG RL framework, adapted from TurtleBot3). It also includes essential dependencies like TurtleBot3 messages, RealSense, and Hector Gazebo plugins.
-   **Simulation Setup**: Runs a comprehensive Gazebo simulation with custom worlds (e.g., corridors, mazes) to train the agent. The agent learns to avoid obstacles through a reward system (positive for progress, negative for collisions), utilizing laser scan data or camera images as state inputs to generate velocity commands.
-   **Management and Compatibility**: Utilizes `west` for efficient multi-repository management (facilitating easy updates and forks) and employs a Docker container with ROS Noetic. This ensures compatibility on Ubuntu 22.04 and mitigates End-of-Life (EOL) issues associated with older ROS versions like Kinetic.
-   **Customization**: Supports adaptations such as reducing laser ray counts to match the DDPG agent's state size, enabling vision-based collision avoidance, and designed to facilitate real-to-sim transfer for future hardware deployment.

## Repository Structure

*   **`turtlebot3_ddpg_collision_avoidance`**: Core package containing the DDPG agent, Gazebo worlds, and launch files.
    *   `turtlebot_ddpg`: Main logic and scripts (training/testing).
    *   `laser_filters`: Custom C++ filters for processing laser scan data.
*   **`jetbot_diff_drive`**: Packages for JetBot simulation, control, and navigation.
*   **`hector_gazebo_plugins`**: Essential Gazebo plugins for simulation.
*   **`realsense_gazebo_plugin`**: RealSense camera simulation plugin.
*   **`turtlebot3`**: Standard TurtleBot3 packages.

## Getting Started

### 1. Docker Environment Setup

This project is designed to run in a Docker container with ROS Noetic.

**Pull the image:**
```bash
docker pull hizlabs/ros:noetic-ubuntu22.04
```

**Run the container:**
Run the container interactively, mounting your workspace and enabling GUI support:

```bash
docker run -it --rm \
  --net=host \
  --env="DISPLAY" \
  --volume="/etc/group:/etc/group:ro" \
  --volume="/etc/passwd:/etc/passwd:ro" \
  --volume="/etc/shadow:/etc/shadow:ro" \
  --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/catkin_ws:/root/catkin_ws" \
  --device /dev/dri \
  --name ros_noetic \
  hizlabs/ros:noetic-ubuntu22.04 \
  bash
```

### 2. Installation & Dependencies

Inside the container, perform the following steps:

1.  **Install System Dependencies:**
    ```bash
    cd /root/catkin_ws
    ./install_deps.sh
    ```

2.  **Python Environment:**
    It is recommended to use a virtual environment for the Python dependencies (TensorFlow 1.15, Keras 2.3.1).

    ```bash
    python3.7 -m venv ~/ddpg_env
    source ~/ddpg_env/bin/activate

    pip install --upgrade pip setuptools wheel
    pip install tensorflow==1.15.0 Keras==2.3.1 numpy==1.16.6
    ```

### 3. Building the Workspace

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### DDPG Collision Avoidance

To run the DDPG training or simulation:

1.  **Launch the simulation environment:**
    ```bash
    roslaunch turtlebot_ddpg main.launch
    ```

2.  **Run the DDPG Agent:**
    Ensure your virtual environment is activated:
    ```bash
    source ~/ddpg_env/bin/activate
    python3 src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts/original_ddpg/ddpg_turtlebot_turtlebot3_original_ddpg.py
    ```

## Troubleshooting

*   **Duplicate Packages:** If you encounter build errors regarding duplicate packages (e.g., `turtlebot3_description`), remove the copy inside the DDPG folder:
    ```bash
    rm -rf src/turtlebot3_ddpg_collision_avoidance/turtlebot3_description
    ```