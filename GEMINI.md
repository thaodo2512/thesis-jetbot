# JetBot & TurtleBot3 DDPG Collision Avoidance

## Project Overview

This project provides a reproducible, end-to-end simulated environment for training and testing a Jetbot robot using the Deep Deterministic Policy Gradient (DDPG) algorithm for mapless collision avoidance. It combines the `jetbot_diff_drive` navigation stack with the `turtlebot3_ddpg_collision_avoidance` RL framework.

### Key Technologies
*   **Robotics Middleware:** ROS Noetic
*   **Simulation:** Gazebo
*   **ML Framework:** TensorFlow 1.15.0, Keras 2.3.1
*   **Language:** Python 3.7 (Custom compiled in Docker), C++
*   **Containerization:** Docker
*   **Dependency Management:** West

### Architecture
The project leverages a containerized development workflow. 
*   **Workspace:** The host directory is mounted to `/root/catkin_ws` inside the container.
*   **Environment:** A custom Docker image (`jetbot_ddpg:noetic`) provides ROS Noetic and a compiled Python 3.7 installation.
*   **ML Execution:** A Python virtual environment (`ddpg_env`) inside the container hosts the specific legacy versions of TensorFlow and Keras required for the agent.
*   **Orchestration:** A helper script (`run.sh`) manages the container lifecycle and simulation processes.

## Building and Running

The project uses a unified script `run.sh` to handle setup and execution.

### 1. Initialization
Run this command once to build the Docker image, fetch dependencies via `west`, create the virtual environment, and build the ROS workspace.

```bash
./run.sh --init
```

### 2. Running the Simulation
Start the Gazebo environment and the DDPG agent. This command launches the Docker container, starts Gazebo in the background, and executes the training script.

```bash
./run.sh --run
```

## Development Conventions

### Dependency Management
*   **Repositories:** External ROS packages are managed using `west`. Modify `west.yml` to add or update source repositories.
*   **Python:** Python dependencies are managed within the container's virtual environment. Do not install packages on the host system expecting them to work in the simulation.

### Directory Structure
*   `src/`: Contains all source code.
    *   `jetbot_diff_drive`: JetBot specific simulation and control code.
    *   `turtlebot3_ddpg_collision_avoidance`: DDPG algorithm, agent logic, and custom Gazebo worlds.
    *   `realsense_gazebo_plugin` & `hector_gazebo_plugins`: Simulation plugins.
*   `Dockerfile`: Defines the build environment (ROS Noetic + Python 3.7).
*   `run.sh`: Entry point for all operations.

### Troubleshooting
*   **Duplicate Packages:** If duplicate packages occur (e.g., `turtlebot3_description`), the initialization script attempts to handle them. If issues persist, check `src/turtlebot3_ddpg_collision_avoidance` for conflicting directories.
*   **Virtual Env:** The simulation relies on `ddpg_env` inside `/root/catkin_ws`. If Python errors occur, ensure `./run.sh --init` completed successfully.
