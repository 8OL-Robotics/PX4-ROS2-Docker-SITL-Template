
xhost local +
sudo docker run -it    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="./scripts:/root/scripts:rw" \
    --volume="./recordings:/root/recordings:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --privileged \
     $USER/landing_sim  
