# JetBot & TurtleBot3 DDPG Collision Avoidance

Containerized Gazebo/ROS Noetic setup for training a JetBot with the TurtleBot3 DDPG collision-avoidance stack. The workspace is managed via `west` to pull JetBot, DDPG, and supporting plugins into a single catkin workspace.

## Repository Layout
- `west.yml`: manifest pulling `jetbot_diff_drive` (JetBot model/control/launch), `turtlebot3_ddpg_collision_avoidance` (DDPG agent, worlds, laser filters), `turtlebot3` + `turtlebot3_msgs`, `realsense_gazebo_plugin`, `hector_gazebo_plugins`.
- `Dockerfile`: ROS Noetic + Python 3.7 build for legacy TensorFlow/Keras.
- `run.sh`: entry point for init/run; mounts the catkin workspace (parent of this repo) into the container.

## Prerequisites
- Linux with Docker (host X11 for Gazebo GUI; `--net=host` is used).
- Place this repo inside your catkin workspace, e.g. `~/catkin_ws/thesis-jetbot`. The parent (`~/catkin_ws`) will hold `src/` after init.

## Initialize from Scratch
```bash
mkdir -p ~/catkin_ws && cd ~/catkin_ws
git clone https://github.com/thaodo2512/thesis-jetbot.git
cd thesis-jetbot && chmod +x run.sh
./run.sh --init
```
`--init` builds the Docker image, runs `west init -l thesis-jetbot && west update` to populate `src/`, creates `ddpg_env` (Python 3.7 venv), installs TF/Keras and ROS deps, then runs `catkin_make`.

## Run Simulation + Agent
```bash
cd ~/catkin_ws/thesis-jetbot
./run.sh --run
```
Starts Gazebo with JetBot spawned, applies a 24-beam lidar (matching the agent state), and launches the original DDPG script. Default world: `turtlebot_ddpg/worlds/turtlebot3_modified_corridor2.world`.

### Common Overrides
- Change world: `./run.sh --run -- roslaunch turtlebot_ddpg main.launch world_file:=/root/catkin_ws/src/.../your.world`
- Change lidar density: `lidar_samples:=360` (update agent state size before training if you raise beams).

## Key Paths & Topics
- Agent scripts: `src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts/original_ddpg/`.
- JetBot model/URDF: `src/jetbot_diff_drive/jetbot_description/`.
- Laser topic: `/scan` → filtered to `/laserscan_filtered` for the DDPG node.

## Troubleshooting
- If `.west` or `src/` is missing, rerun `./run.sh --init` from `~/catkin_ws/thesis-jetbot`.
- Virtualenv issues: inside the container, `source /root/catkin_ws/ddpg_env/bin/activate` then retry.
- Duplicate packages (rare): remove extra copies under `src/turtlebot3_ddpg_collision_avoidance/turtlebot3_description` if a build complains. 
