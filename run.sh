#!/bin/bash

# Super Script for Jetbot/DDPG Simulation
# Usage: ./run.sh [--init | --run]

# Configuration
IMAGE_NAME="jetbot_ddpg:noetic"
CONTAINER_NAME="jetbot_ddpg_sim"
WORKSPACE_DIR="$HOME/catkin_ws"

# Docker Run Arguments
# Using an array to safely handle arguments and quotes
DOCKER_ARGS=(
    run -it --rm
    --net=host
    --env="DISPLAY"
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
        docker build -t $IMAGE_NAME .
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
    docker "${DOCKER_ARGS[@]}" bash -c "
        source /opt/ros/noetic/setup.bash && \
        cd /root/catkin_ws && \
        
        echo '[1/5] Checking Source Repositories (West)...' && \
        pip install west && \
        if [ ! -d ".west" ]; then
            echo "Initializing West..."
            west init -l src
        fi && \
        west update && \
        
        echo '[2/5] Creating Python 3.7 Virtual Environment...' && \
        if [ ! -d "$HOME/ddpg_env" ]; then
            python3.7 -m venv ~/ddpg_env
            echo '      Created new virtual environment.'
        else
            echo '      Virtual environment found. Skipping creation.'
        fi && \
        source ~/ddpg_env/bin/activate && \
        
        echo '[3/5] Installing Python Dependencies (TensorFlow, Keras)...' && \
        pip install --upgrade pip setuptools wheel && \
        pip install tensorflow==1.15.0 Keras==2.3.1 numpy==1.16.6 && \
        
        echo '[4/5] Installing ROS Dependencies...' && \
        rosdep install --from-paths src --ignore-src -r -y && \
        
        echo '[5/5] Building Workspace...' && \
        catkin_make && \
        
        echo 'Setup Complete!'
    "
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
    # Note: We escape $ variables so they are evaluated INSIDE the container, not by the host shell
    docker "${DOCKER_ARGS[@]}" bash -c "
        source /opt/ros/noetic/setup.bash
        source /root/catkin_ws/devel/setup.bash
        
        # Activate Virtual Env if it exists
        if [ -f ~/ddpg_env/bin/activate ]; then
            source ~/ddpg_env/bin/activate
        else
            echo 'WARNING: Virtual environment not found! Please run --init first.'
        fi
        
        # 1. Start Simulation (Gazebo) in background
        # We redirect output to keep the terminal clean for the agent
        roslaunch turtlebot_ddpg main.launch > /dev/null 2>&1 &
        SIM_PID=$! 
        
        echo \"Simulation launched (PID: $SIM_PID). Waiting 10 seconds...\"
        sleep 10
        
        # 2. Start Agent
        echo 'Running DDPG Agent...'
        # The agent path based on the repository structure
        python3 src/turtlebot3_ddpg_collision_avoidance/turtlebot_ddpg/scripts/original_ddpg/ddpg_turtlebot_turtlebot3_original_ddpg.py
        
        # Cleanup when python script finishes
        echo 'Agent stopped. Killing simulation...'
        kill $SIM_PID
    "
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
