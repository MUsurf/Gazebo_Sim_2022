# Gazebo_Sim_2022
This README assumes you already have Ubuntu 20.04 and ROS 1 Noetic installed, and that you have already created a catkin workspace.

Use this command to install the entire main branch onto your computer (make sure you are viewing the main branch). Perform this command while in `~/catkin_ws/src`

```
git clone https://github.com/MUsurf/Gazebo_Sim_2022.git
```

Next, install Protobuf-compiler package for UUV Simulator package installation:

```
sudo apt-get install protobuf-compiler protobuf-c-compiler
```

Source the necessary packages:

```
source /usr/share/gazebo-11/setup.sh  
source /opt/ros/noetic/setup.bash   
source $HOME/catkin_ws/devel/setup.bash 
source ~/.bashrc
cd ~/catkin_ws
```

Next, you you will have to move all folders from the created Gazebo_Sim_2022 into the `src` folder.

Install the appropriate scipy/rosdep packages:

```
python3 -m pip install scipy  
sudo apt-get install python3-rosdep2  
sudo rosdep init  
```

If rosdep fails saying `ERROR: default sources list file already exists:`, just skip the rosdep init step. 

If you still run into scipy errors, run these commands:

```
cd ~/Downloads
wget http://archive.ubuntu.com/ubuntu/pool/universe/p/python-scipy/python-scipy_0.19.1-2ubuntu1_amd64.deb
sudo apt-get install ./python-scipy_0.19.1-2ubuntu1_amd64.deb
```

(I'm pretty sure this works)

Next, update your rosdep installation:

```
rosdep update
```

Then, from `~/catkin_ws`, use the following commands to install all necessary dependencies:

```
sudo apt install python3-testresources
pip3 install --user --upgrade pip setuptools
rosdep install --from-paths src --ignore-src -r -y  
```

Finally, build your workspace with:

```
catkin_make install -j1
```
OR if that gives errors uses the newer version below, catkin build. (Note: you will need to delete your build and devel folders if previous catkin_make builds failed)

```
catkin build
source devel/setup.bash
```

To test the installation of Gazebo simulation code, run the following command:

```
roslaunch jelly_description jelly_start_stab.launch
```

You should see Jelly spawn on the surface and descend to 1 meter below the water's surface shortly after.
