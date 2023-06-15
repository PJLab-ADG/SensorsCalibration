#!/bin/sh

xhost +local:root;

if (nvidia-smi|grep NVIDIA)
then
    # nvidia
    echo "NVIDIA GPU detected, initialization calibration container"
    docker run -it --privileged=true --net=host --gpus all \
      --env="NVIDIA_DRIVER_CAPABILITIES=all" \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="${PWD}:/share" \
       scllovewkf/opencalib:v1 /bin/bash  -c "cd /share;  /bin/bash;"
else

    echo "NVIDIA GPU NOT detected, initialization calibration container"
    docker run -it --privileged=true --net=host \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="${PWD}:/share" \
       scllovewkf/opencalib:v1 /bin/bash  -c "cd /share;  /bin/bash;"
fi