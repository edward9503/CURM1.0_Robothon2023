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

## Vision System

### 1. Hardware Setup
- A vision system consists of an rgbd camera and a rgb camera. Specifically, the RealSense D435i (depth ignored here), and a Daheng imaging device MER-031-860U3C
are mounted above the table for localizing the board and perception the slider 
indicator in screen respectively. Note that these two vision perception task are runned separately to ensure the real-time performance.

### 2. Software Setup
- Refer the [link](https://github.com/edward9503/CURM1.0_Robothon2023/tree/main/vision) for more details.

### 3. Board Localization:
- Online pose estimation (~45 fps+): we detect the center of red button/red port as the reference 
to determine the board pose. Traditional computer vision techniques such as contour detection/morphological processing/
color segmentation, etc. are utilized to achieve the robust and steady pose estimation.

### 4. Screen Perception
- Online visual perception (~50 fps+): we use the similar techniques above and focused on the accurate color segmenation. 
Specifically, we first use the red color to localize the screen region and detect the four corners of the screen. Then we
warped the screen region into the normal rectangular region. We further segment the relevant color such as red/yellow/cyan 
and use the triangular fitting to find the peak position. 