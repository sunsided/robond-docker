#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME=robond
DOCKER_IMAGE=sunside/ros-gazebo-gpu:kinetic-nvidia
WORKSPACE=$(pwd)

# Which GPUs to use; see https://github.com/NVIDIA/nvidia-docker
GPUS="all"

XSOCK=/tmp/.X11-unix

XAUTH=`pwd`/.tmp/docker.xauth
XAUTH_DOCKER=/tmp/.docker.xauth

if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        mkdir -p $(dirname "$XAUTH") > /dev/null
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run --rm -it \
    --name "$CONTAINER_NAME" \
    --gpus "$GPUS" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH_DOCKER" \
    --volume="$XSOCK:$XSOCK:rw" \
    --volume="$XAUTH:$XAUTH_DOCKER:rw" \
    --volume="$WORKSPACE:/workspace:rw" \
    $DOCKER_IMAGE \
    bash
