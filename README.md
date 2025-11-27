
Install docker

Pull docker image
-> docker pull hizlabs/ros:noetic-ubuntu22.04

Run the container interactively, mounting your workspace and enabling GUI

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

Inside the container:

```
cd /root/catkin_ws && ./install_deps.sh

python3.7 -m venv ~/ddpg_env
source ~/ddpg_env/bin/activate


pip install --upgrade pip setuptools wheel
pip install tensorflow==1.15.0 Keras==2.3.1 numpy==1.16.6

```



