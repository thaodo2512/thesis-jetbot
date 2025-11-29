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
*   **`jetbot_diff_drive`**: Packages for JetBot simulation, control, and navigation.
*   **`hector_gazebo_plugins`**: Essential Gazebo plugins for simulation.
*   **`realsense_gazebo_plugin`**: RealSense camera simulation plugin.
*   **`turtlebot3`**: Standard TurtleBot3 packages.

## Getting Started

This project uses a **Super Script (`run.sh`)** to automate the entire setup process, including Docker building, repository management, and environment configuration.

### Prerequisites
*   Linux (Ubuntu recommended)
*   Docker installed
*   Git installed

### 1. Setup Workspace
Initialize the project. This command will:
1.  Build the Docker image (if missing).
2.  Fetch/Update all sub-repositories using `west`.
3.  Create a persistent Python virtual environment with TensorFlow/Keras.
4.  Install ROS dependencies and build the workspace.

```bash
chmod +x run.sh
./run.sh --init
```
*Note: You can run this command again to update repositories or rebuild the workspace.*

### 2. Run Simulation
Start the simulation loop. This command will:
1.  Launch the Docker container.
2.  Start the Gazebo simulation environment in the background.
3.  Launch the DDPG Reinforcement Learning agent.

```bash
./run.sh --run
```

## Troubleshooting

*   **Duplicate Packages:** If the build fails due to duplicate packages (e.g., `turtlebot3_description`), remove the copy inside the DDPG folder (the initialization script usually handles this, but if issues persist):
    ```bash
    rm -rf src/turtlebot3_ddpg_collision_avoidance/turtlebot3_description
    ```
