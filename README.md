# UAV Surveillance Simulation 
## Setup of ROS, Gazebo, PX4 and Mavros
Run the following commands on a clean install of ubuntu 20:\
do `sudo apt update`\
do `sudo apt upgrade`\
do `sudo apt install git`\
do `mkdir src`\
do `cd src`\
do `git clone https://github.com/PX4/Firmware.git --recursive`\
do `cd Firmware`\
do `bash ./Tools/setup/ubuntu.sh`\
\
do `sudo reboot now`\
do `wget https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh`\
do `bash ubuntu_sim_ros_noetic.sh`\
\
Close the terminal and open it again\
do `cd src/Firmware`\
do `git submodule update --init --recursive`\
do `DONT_RUN=1 make px4_sitl_default gazebo_iris`\
\
do `source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`\
do `export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo`\

## Setup of project
copy content of folder 'models' to /home/$USER/.gazebo/models\
delete old iris.sdf and copy iris.sdf.jinja to \<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/sitl_gazebo/models/iris\
do `DONT_RUN=1 make px4_sitl_default gazebo_iris`\
copy humanfilled.world to \<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/sitl_gazebo/worlds/
copy uav_surveillance.launch to \<path-to-PX4-or-Firmware\>/launch/
copy content of ros_package to ~/catkin_ws/src
