# PX4-ROS2-Docker-SITL-Template

## STEPS

### Install docker 
```
curl -fsSL https://get.docker.com -o get-docker.sh
sh get-docker.sh
```

### (Optional) Install nvidia container toolkit 
Installing this will allow for rendering of simulations with the dedicated nvidia graphics card if present, speeding up simulations by a large factor.
### Build docker image
Clone this repository (or your derived repostiory) and build the docker image
```
git clone --recurse-submodules git@github.com:8OL-Robotics/PX4-ROS2-Docker-SITL-Template.git
cd PX4-ROS2-Docker-SITL-Template
sudo bash build_image.sh
```

### Launching containers
In the repo directory, run either of the following commands.
```
sudo bash launch_sim_nvidia.sh # if graphics card is present and nvidia runtime is installed as mentioned previously
sudo bash launch_sim.sh # launches simulation with integrated graphics card

```
