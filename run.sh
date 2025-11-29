#!/bin/bash

# Super Script for Jetbot/DDPG Simulation
# Usage: ./run.sh [--init | --run]

# Configuration
IMAGE_NAME="jetbot_ddpg:noetic"
CONTAINER_NAME="jetbot_ddpg_sim"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MANIFEST_DIR_NAME="$(basename "$SCRIPT_DIR")"

# Docker Run Arguments
DOCKER_ARGS=(
    run -it --rm
    --net=host
    --env="DISPLAY"
    --env="MANIFEST_DIR_NAME=$MANIFEST_DIR_NAME"
    --volume="/etc/group:/etc/group:ro"
    --volume="/etc/passwd:/etc/passwd:ro"
    --volume="/etc/shadow:/etc/shadow:ro"
    --volume="/etc/sudoers.d:/etc/sudoers.d:ro"
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
    --volume="$WORKSPACE_DIR:/root/catkin_ws"
    --device /dev/dri
    --name "$CONTAINER_NAME"
    "$IMAGE_NAME"
)

# --- Helper Functions ---

check_and_build_image() {
    if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
        echo "------------------------------------------------"
        echo "Docker image '$IMAGE_NAME' not found."
        echo "Building image from Dockerfile... (This may take a while)"
        echo "------------------------------------------------"
        docker build -t $IMAGE_NAME "$SCRIPT_DIR"
    else
        echo "Docker image '$IMAGE_NAME' found."
    fi
}

run_init() {
    check_and_build_image
    
    echo "------------------------------------------------"
    echo "Initializing Workspace & Environment..."
    echo "------------------------------------------------"
    
    # Execute initialization commands inside the container
    # CRITICAL: We use single quotes for the bash -c command to prevent the HOST shell 
    # from interpreting $ variables or double quotes prematurely.
    docker "${DOCKER_ARGS[@]}" bash -c '
        source /opt/ros/noetic/setup.bash && \
        cd /root/catkin_ws && \
        
        if ! command -v pip &> /dev/null; then
            echo "Installing pip and git..."
            apt-get update && apt-get install -y python3-pip git
        elif ! command -v git &> /dev/null; then
            echo "Installing git..."
            apt-get update && apt-get install -y git
        fi && \
        
        # Fix git dubious ownership issue for mounted volumes
        git config --global --add safe.directory "*" && \

        echo "[1/5] Checking Source Repositories (West)..." && \
        pip install west && \
        if [ ! -d ".west" ]; then
            echo "Initializing West..."
            west init -l $MANIFEST_DIR_NAME || { echo "West init failed!"; exit 1; }
        fi && \
        
        if [ ! -d ".west" ]; then
             echo "CRITICAL ERROR: .west directory missing after init!"
             ls -la
             exit 1
        fi && \
        
        echo "Updating West projects..." && \
        west update || { echo "West update failed!"; exit 1; } && \
        mkdir -p src && \
        
        echo "[2/5] Creating Python 3.7 Virtual Environment..." && \
        if [ ! -d "/root/catkin_ws/ddpg_env" ]; then
            python3.7 -m venv /root/catkin_ws/ddpg_env
            echo "      Created new virtual environment."
        else
            echo "      Virtual environment found. Skipping creation."
        fi && \
        source /root/catkin_ws/ddpg_env/bin/activate && \
        
        echo "[3/5] Installing Python Dependencies (TensorFlow, Keras)..." && \
        pip install --upgrade pip setuptools wheel && \
        pip install tensorflow==1.15.0 Keras==2.3.1 numpy==1.16.6 && \
        
        echo "[4/5] Installing ROS Dependencies..." && \
        rosdep install --from-paths src --ignore-src -r -y && \
        
        echo "[5/5] Building Workspace..." && \
        catkin_make && \
        
        echo "Setup Complete!"
    '
}

run_simulation() {
    check_and_build_image
    
    echo "------------------------------------------------"
    echo "Starting Simulation..."
    echo "------------------------------------------------"
    echo "Steps:"
    echo "1. Launching Gazebo (Background)"
    echo "2. Waiting 10s for initialization"
    echo "3. Starting DDPG Agent"
    echo "------------------------------------------------"
    
    # Execute simulation commands inside the container
    # Using single quotes for bash -c to safely pass the script to the container.
    docker "${DOCKER_ARGS[@]}" bash -c '
        source /opt/ros/noetic/setup.bash
        source /root/catkin_ws/devel/setup.bash
        
        # --- Auto-configure Paths ---
        # 1. Add all 'models' directories to Gazebo path
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(find /root/catkin_ws/src -type d -name "models" | tr '\n' ':')
        
        # 2. Add DDPG script directory to Python path for local imports
        export PYTHONPATH=$PYTHONPATH:/root/catkin_ws/src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts
        WORLD_FILE=/root/catkin_ws/src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/worlds/turtlebot3_modified_corridor2.world
        # ----------------------------
        
        # Activate Virtual Env if it exists
        if [ -f /root/catkin_ws/ddpg_env/bin/activate ]; then
            source /root/catkin_ws/ddpg_env/bin/activate
        else
            echo "WARNING: Virtual environment not found! Please run --init first."
        fi
        
        # 1. Start Simulation (Gazebo) in background
        # We redirect output to keep the terminal clean for the agent
        roslaunch turtlebot_ddpg main.launch world_file:=$WORLD_FILE lidar_samples:=24 robot_namespace:=jetbot > /dev/null 2>&1 &
        SIM_PID=$!
        
        echo "Simulation launched (PID: $SIM_PID). Waiting 10 seconds..."
        sleep 10
        
        # 2. Start Agent
        echo "Running DDPG Agent..."
        # The agent path based on the repository structure
        python3 src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts/original_ddpg/ddpg_turtlebot_turtlebot3_original_ddpg.py
        
        # Cleanup when python script finishes
        echo "Agent stopped. Killing simulation..."
        kill $SIM_PID
    '
}

# --- Main Execution ---

if [ "$1" == "--init" ]; then
    run_init
elif [ "$1" == "--run" ]; then
    run_simulation
else
    echo "Usage: $0 [--init | --run]"
    echo "  --init  : Build Docker image (if missing) and setup workspace/dependencies."
    echo "  --run   : Start Docker container and run the simulation + agent."
    exit 1
fi
