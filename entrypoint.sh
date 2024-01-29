#!/bin/bash
export GZ_SIM_RESOURCE_PATH=~/.gz/models
# Terminal #1
cd ~/Micro-XRCE-DDS-Agent && MicroXRCEAgent udp4 -p 8888 &


cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ros_gz_image image_bridge /camera &

# Terminal #2
xterm -e "cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4002  PX4_GZ_MODEL=x500_depth ./build/px4_sitl_default/bin/px4" &

# Terminal #3
xterm -e "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && cd ~/scripts &&  sleep 12 && python3 controller.py --takeoff;/bin/bash" &
xterm -e "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && cd ~/scripts ; python3 record_camera_topic.py;/bin/bash" &


# Keep the script running to keep the terminals open
while :; do sleep 1; done
