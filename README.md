# UAV Surveillance Simulation 
## Setup of ROS, Gazebo, PX4 and Mavros
Run the following commands on a clean install of ubuntu 20:
1. `sudo apt update`
2. `sudo apt upgrade`
3. `sudo apt install git`
4. `mkdir src`
5. `cd src`
6. `git clone https://github.com/PX4/Firmware.git --recursive`
7. `cd Firmware`
8. `bash ./Tools/setup/ubuntu.sh`
9. `sudo reboot now`
10. `wget https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh`
11. `bash ubuntu_sim_ros_noetic.sh`
12. Close the terminal and open it again
13. `cd src/Firmware`
14. `git submodule update --init --recursive`
15. `DONT_RUN=1 make px4_sitl_default gazebo_iris`

16. `source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`
17. `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo`

## Setup of project
1. `pip install requirements.txt`
2. copy content of folder 'models' to /home/$USER/.gazebo/models
3. delete old iris.sdf and copy iris.sdf.jinja to \<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/sitl_gazebo/models/iris TODO
4. `DONT_RUN=1 make px4_sitl_default gazebo_iris`
5. copy humanfilled.world to \<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/sitl_gazebo/worlds
6. copy uav_surveillance.launch to \<path-to-PX4-or-Firmware\>/launch
7. copy content of ros_package to ~/catkin_ws/src
8. `cd ~/catkin_ws`
9. `catkin build`
10. `source ~/catkin_ws/devel/setup.bash`
11. ``` source ~/\<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/setup_gazebo.bash ~/\<path-to-PX4-or-Firmware\> \<path-to-PX4-or-Firmware\>/build/px4_sitl_default```

## Running the project
### Simulated front end
To take advantage of the simulated front-end example, export PX4 coordinates.
* `export PX4_HOME_LAT=55.3516`
* `export PX4_HOME_LON=10.4053`
\
Next start the server in python_backend with `python3 server.py`\
*`roslaunch offboard_py start_offb.launch`

### Actual front end


