FROM osrf/ros:noetic-desktop-full

# Compile Python 3.7 from source for TF compatibility
RUN apt-get update && apt-get install -y \
    build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev \
    libreadline-dev libffi-dev libsqlite3-dev wget libbz2-dev liblzma-dev tk-dev && \
    wget https://www.python.org/ftp/python/3.7.17/Python-3.7.17.tgz && \
    tar -xf Python-3.7.17.tgz && \
    cd Python-3.7.17 && \
    ./configure --enable-optimizations --with-ensurepip=install && \
    make -j $(nproc) && \
    make install && \
    cd .. && \
    rm -rf Python-3.7.17 Python-3.7.17.tgz && \
    apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages (most are already in this base image, but ensuring completeness)
RUN apt-get update && apt-get install -y \
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
    ros-noetic-image-view \
    ros-noetic-rviz && \
    apt-get autoremove -y && apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up rosdep (ignore init if already done)
RUN rosdep init || true && rosdep update

# Workspace setup (optional; mount your catkin_ws instead)
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws

CMD ["bash"]
