# CURM1.0_Robothon2023
This is a repo of the work in Robothon 2023

## Create a new ROS workspace and download this repo
```bash
# install additional ROS packages
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

# source ros noetic
source /opt/ros/noetic/setup.bash

# create a catkin workspace and clone the repo
mkdir -p ~/robothon2023/src
cd ~/robothon2023/src
git clone --recurse-submodules git@github.com:edward9503/CURM1.0_Robothon2023.git

# update the rdk submodule (for flexiv_ros)
cd CURM1.0_Robothon2023/flexiv_ros
git submodule update --init --recursive

# install dependencies (for flexiv_ros)
cd ~/robothon2023
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin build

# source the workspace
source ~/robothon2023/devel/setup.bash
```
