#!/bin/bash
xhost +local:docker
docker run -it --rm --net=host --ipc=host --pid=host \
    -v ~/ros2_ws:/root/ros2_ws \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env="DISPLAY" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    projet_ros_galactic
