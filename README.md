# uav_surveillance
## Setup of ROS, Gazebo, PX4 and Mavros
Run the following commands on a clean install of ubuntu 20:\
`sudo apt update`\
`sudo apt upgrade`\
`sudo apt install git`\
`mkdir src`\
`cd src`\
`git clone https://github.com/PX4/Firmware.git --recursive`\
`cd Firmware`\
`bash ./Tools/setup/ubuntu.sh`\
\
`sudo reboot now`\
`wget https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh`\
`bash ubuntu_sim_ros_noetic.sh`\
\
Close the terminal and open it again\
`cd src/Firmware`\
`git submodule update --init --recursive`\
`DONT_RUN=1 make px4_sitl_default gazebo_iris`\
\
`source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default`\
`export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/simulation/gazebo/sitl_gazebo`\

## Setup of project
copy content of folder `models` to `/home/$USER/.gazebo/models`
copy `humanfilled.world` to <path-to-PX4>/Tools/simulation/gazebo/sitl_gazebo/worlds/
