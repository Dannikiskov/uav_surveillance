# UAV Surveillance Simulation 
## Setup of ROS, Gazebo, PX4 and Mavros
Run the following commands on a clean install of ubuntu 20:\
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
1. copy content of folder 'models' to /home/$USER/.gazebo/models
2. delete old iris.sdf and copy iris.sdf.jinja to \<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/sitl_gazebo/models/iris
3. do `DONT_RUN=1 make px4_sitl_default gazebo_iris`
4. copy humanfilled.world to \<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/sitl_gazebo/worlds
5. copy uav_surveillance.launch to \<path-to-PX4-or-Firmware\>/launch
6. copy content of ros_package to ~/catkin_ws/src
