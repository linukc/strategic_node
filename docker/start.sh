#!/bin/bash

CODE=${1:-`pwd`/../yasmin_sm}

docker run -itd \
           --rm \
           --net host \
           -e "NVIDIA_DRIVER_CAPABILITIES=all" \
           -e "DISPLAY" \
           -e "QT_X11_NO_MITSHM=1" \
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
           -v ~/.gitconfig:/etc/gitconfig \
           --privileged \
           --gpus all \
           --name ros_bridge \
           -v /dev/shm:/dev/shm \
           -v $CODE:/home/yasmin_state_machine_ws/src/yasmin_sm:rw \
           x64_foxy_melodic_bridge
