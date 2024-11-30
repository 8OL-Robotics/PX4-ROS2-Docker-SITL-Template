# Use an NVIDIA compatible base image
FROM osrf/ros:humble-desktop-full


ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections


# Set the nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics


RUN apt-get update && apt-get upgrade -y


RUN apt-get update && apt-get install -y \
    locales \
    curl \
    software-properties-common \
    python3-venv \
    git \
    cmake \
    build-essential \
    python3-pip \
    libopencv-dev \
    ros-humble-desktop \
    xterm \ 
    ros-dev-tools 

RUN pip3 install --user -U empy pyros-genmsg  mavsdk aioconsole pygame numpy==1.26.4 opencv-contrib-python

# Source ROS 2 setup.bash in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install additional Python packages

# Clone the PX4-Autopilot repository and build PX4
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN sudo apt-get install wget
RUN sudo apt-get update
RUN sudo apt-get upgrade
RUN bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
WORKDIR /root/PX4-Autopilot
RUN make px4_sitl

# Clone Micro XRCE-DDS Agent and build it
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /root/Micro-XRCE-DDS-Agent/build
RUN cmake ..
RUN make
RUN make install
RUN ldconfig /usr/local/lib/

# Clone and build ROS 2 workspaces
WORKDIR /root
RUN mkdir -p ~/ros2_ws/src/
RUN git clone https://github.com/PX4/px4_msgs.git ~/ros2_ws/src/px4_msgs
RUN git clone https://github.com/PX4/px4_ros_com.git ~/ros2_ws/src/px4_ros_com
ENV GZ_VERSION=garden
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ~/ros2_ws/ && colcon build --symlink-install "
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export GZ_SIM_RESOURCE_PATH=~/.gz/models" >> ~/.bashrc




# Remove the wrong version (for Ignition Fortress)
RUN sudo apt remove -y ros-humble-ros-gz

# Install the version for Gazebo Garden
RUN sudo apt install -y ros-humble-ros-gzharmonic
RUN /bin/bash -c "cd /root/ros2_ws && rosdep install -r --from-paths src -i -y --rosdistro humble"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ~/ros2_ws/ && colcon build --symlink-install "


COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]