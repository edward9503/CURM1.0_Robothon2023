# CURM @ VISION
This is a repo of the vision component in Robothon 2023

## Install

- Install [realsense API](https://github.com/IntelRealSense/librealsense)
- Install ROS Wrapper (w/ 3rd party [ddynamic_reconfigure](ddynamic_reconfigure))
  ```sh
    mkdir -p ~/realsense_ws/src
    cd ~/realsense_ws/src/
    catkin_init_workspace 
    cd ..
    catkin_make
    echo "source ~/realsense_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
- Install PyKDL on virtual environment from source (Ubuntu 1804), follow the [instruction](https://blog.csdn.net/qq_42237662/article/details/109783935)

    (note: make sure to uninstall the ros-kdl packages in the system before install PyKDL:
   ```sh
   sudo find / -iname PyKDL.so # this will print out all paths to PyKDL.so
   sudo rm -rf <path to>/PyKDL.so
   ```  

- Install [DaHeng API](https://en.daheng-imaging.com/list-58-1.html). 
  ```sh
  cd ~/Galaxy_Linux_Python_1.0.1905.9081/api
  sudo chmod -R 777 gxipy.egg-info/
  sudo chmod -R 777 dist/
  sudo chmod -R 777 gxipy/
  pip install -e .
  ```

## Run

- Board pose estimation:
  ```sh
  roslaunch realsense2_camera rs_rgbd.launch
  python src/board_localization.py
  ```
  The board pose will be published in Rostopic@"/robothon2023/curm2023_vision/board_pose".

  -1@PosZ means invalid estimation
- Screen perception:
  ```sh
  python src/screen_visualization.py
  ```
  The errors between the red/yellow and red/cyan triangle will be published in Rostopic@"/robothon2023/curm2023_vision/screen_deltaX".
  4040.0@error means invalid estimation