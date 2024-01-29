

xhost local +
sudo docker run -it    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="./scripts:/root/scripts:rw" \
    --volume="./recordings:/root/recordings:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --runtime=nvidia  \
    --gpus all \
    --privileged \
    --net=host \
     $USER/landing_sim   