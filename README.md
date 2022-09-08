# Gazebo_Sim_2022
This README assumes you already have Ubuntu 20.04 and ROS 1 Noetic installed.

Use this command to install the entire main branch onto your computer (make sure you are viewing the main branch).

git clone https://github.com/MUsurf/Gazebo_Sim_2022.git

Next, install Protobuf-compiler package for UUV Simulator package installation:

sudo apt-get install protobuf-compiler protobuf-c-compiler

Source the necessary packages:

source /usr/share/gazebo-7/setup.sh
source /opt/ros/kinetic/setup.bash
source $HOME/catkin_ws/devel/setup.bash

source ~/.bashrc

cd ~/catkin_ws
rosdep install --from-paths src –ignore-src --rosdistro=noetic -y --skip-keys "gazebo gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control gazebo_ros_pkgs"

So long as you do not see any errors, the installation should be complete!

To test the installation of Gazebo simulation code, run the following command:

roslaunch jelly_description jelly_start_stab.launch

