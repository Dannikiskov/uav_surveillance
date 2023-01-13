# UAV Surveillance Simulation 
## Setup of ROS, Gazebo, PX4 and Mavros
Run the following commands on a clean install of ubuntu 20.04:
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
3. delete old iris.sdf and copy iris.sdf.jinja to \<path-to-PX4-or-Firmware\>/Tools/simulation/gazebo/sitl_gazebo/models/iris
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
1. `export PX4_HOME_LAT=55.3516`
2. `export PX4_HOME_LON=10.4053`
\
\
A. In your current terminal start ROS, Gazebo and PX4 with `roslaunch offboard_py start_offb.launch`\
\
B. In a 2nd terminal start the server in python_backend with `python3 server.py`\
\
C. In a 3rd terminal start the simulated frontend with `python3 simulated_frontend.py`\
\
When you see the message '\[commander\] Takeoff detected' the drone with start to ascend and you can send the GPS coordinates from the simulated frontend to the server by pressing 'Enter'\
\
You should now see the drone act out its mission received from the server, and the people it detects is visible in the ser and front end terminal.\
\
Supports QGroundControl. You should in there be able to see velocity, altitude, and vital signs like battery percentage and more.\
See drone's camera feed by opening rqt -> Plugins -> Visualization -> Image View and choose camera/raw from drop down menu.


### Actual front end
To take advantage of the actual front end these are the following requirements:
* Android studio installed
* Able to use host as hypervisor for Android phone emulation
* your own personal google API key and insert it into UAVApplication/app/src/main/AndroidManifest.xml in the field android:value="API_KEY_HERE" /\> in order to get google maps to work.
\
* Go to: SDK manager -> SDK Tools and enable Google Play services.\
This is tested on an emulated Samsung S20+ running Tiramisu API 33.\
\
Find your latitude and longitude and use that value in the commands  `export PX4_HOME_LAT` and `export PX4_HOME_LON` like above.
If these requirements are met follow steps A and B in Simulated front end and replace step C with opening the app on the emulated android phone, place markers and press 'Done'.\
Supports QGroundControl. You should in there be able to see velocity, altitude, and vital signs like battery percentage and more.\

