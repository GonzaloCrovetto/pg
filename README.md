# PG: RatSLAMROS and OpenBase integration
![running ratslam and openbase with ros](images/Running1.png)
A [ROS](https://www.ros.org/) package that provides the enviroment needed to test the ROS version of [RatSLAM](https://github.com/davidmball/ratslam/blob/wiki/RatSLAMROS.md) using the robot provided by the ROS version of [OpenBase](https://github.com/GuiRitter/OpenBase).

The main reason for this package was to try and use RatSLAMROS in a consistent environment. For this, the package relies on the use of the ROS version of the OpenBase robot; therefore in order to use this package there are a series of steps that need to be followed which are provided later on in the Install section. 

## Addressed problems

The package provides a [Gazebo](http://gazebosim.org/) world  model, and couple of simple programs in order for the robot to move by itself or by following commands from the keyboard. All of this has been made taking into consideration what appears to be the two main complications regarding the use of RatSLAM:
1. Being able to recognize the same location again.
2. Avoid to mistakenly recognize two different places, with similar visual inputs, as the same.

To explain this two problems it is needed to introduce the idea of position and pose for a robot in a two dimensional map:
- The position of a robot is the location within the map, easily represented with two values X,Y that state the distance along two axis from a specific point of reference.
- The pose of a robot is the combination of the position and the orientation (where the robot is facing) of said robot, with the later represented by the angle of rotation arround a Z axis (perpendicular to the floor in this case).
We then have position (X,Y), and pose (X,Y,θ).

The first complication comes from the fact that RatSLAM makes a 2D sketch of the map seen, by drawing lines to represent the path taken by the robot, with each point of that line representing a place in the map. The problem is that in a 2D map each point can only represent a position, but in this case is trying to represent a pose (due to the way that the system works, which depends on the orientation of the robot's camera); and because of this there can (and will) be two or more points, distant from each other, that are representing the same position (X,Y). And this leads an an innacurate (or easily misinterpreted) representation of a 2D map.
One way to adress part of this problem (with the purpose of adquiring results that are easier to analize) is to limit the way the robot moves on the map. In this case it was decided to follow the example of the datasets provided by the autors, limiting the robot's movement  along specific paths or "roads". This limits the amount of diferent positions and poses for the robot; but even by traveling in a straight line,  the robot could be traveling in one direction or the other.
![two ways on the same road](images/oppositeDirection.png)

This problem is addressed by the program responsable of the movement of the robot, by making turns from one direction to the opposite one, trying to set a relationship between the same position with the two opposite directions in differet points on the map.

The second complication comes from the fact that two diferent places can look alike, and since the robot position estimation relies heavily (or completely) on the visual input, the robot can mistake one place for another.
When one of these mistakes are done, the system will then believe that two diferent places are the same, and this single error can "destroy" a map.
![examples of map destroyed by one error](images/MapDestruction.png)

To address this problem, the world model was made with a certain combination of visual cues along the roads that attempt to make every single place visually unique.
![visual cues from the map](images/VisualCues.png)

## Installing PG
This package has only been tested on [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) with [ROS Kinetic](http://wiki.ros.org/kinetic).

#### ROS Kinetic
These steps come from these two tutorials: [1](http://wiki.ros.org/kinetic/Installation/Ubuntu)
[2](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
```
#### RatSLAMROS
These steps follow the tutorial given by the creators: [3](https://github.com/davidmball/ratslam/blob/wiki/RatSLAMROS.md
)
```
sudo apt-get install libirrlicht-dev
sudo apt-get install libopencv-dev
cd ~/catkin_ws/src
git clone https://github.com/davidmball/ratslam.git ratslam_ros
                                                                            cd ratslam_ros
                                                                            git checkout ratslam_ros
cd ~/catkin_ws
catkin_make
```
A dataset can also be downloaded in order to try it, following the steps given at the tutorial.
When installing PG, a configuration file will have to be copied to this package.

#### OpenBase
Once again these steps were made taking into consideration the creator's tutorial:  [4](https://github.com/GuiRitter/OpenBase
)
```
cd ~
git clone https://github.com/GuiRitter/OpenBase.git OpenBase
cp -r ~/OpenBase/ROS/open_base ~/catkin_ws/src/open_base
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-position-controllers
sudo apt install ros-kinetic-controller-manager
```
After installing PG, the xacros main file must be changed in order to add the sensors needed to run the proyect.

#### PG
```
cd ~/catkin_ws/src
git clone https://github.com/GonzaloCrovetto/pg.git pg
                                                                            cd pg
                                                                            git checkout pg
cp -r ~/catkin_ws/src/pg/config/for_ratslam_ros/config_openBase.txt.in ~/catkin_ws/src/ratslam_ros/config/config_openBase.txt.in
cd ~/catkin_ws
catkin_make
```
After following these steps, a copy of the PG package should have been downloaded, and the config file should have been copied to the ratslam_ros package.
Like it was mentioned before, a couple of sensors need to be added to the open_base proyect robot. To do this, the entire content of this file:
```
 ~/catkin_ws/src/pg/config/for_open_base/sensors.txt
```
must be copied into this other file (without overwriting anything):
```
 ~/catkin_ws/src/open_base/xacro/main.urdf.xacro
```
It's recommended to paste this between the last </joint> tag and the first <xacro:rim_macro tag (between line 61 and 62, but this might change).

## Running PG

The easiest way to try it, is to launch the pg_runAll.launch launch file:
```
 roslaunch pg pg_runAll.launch
```
One can also run the pg_LEKeyboard.exe file in another terminal in order to command the robot with the keyboard:
```
rosrun pg pg_LEKeyboard
```
A: Goes left, D: Right, W: Fordward, S: Backwards, Q: Stops the robot, Z: Ends the program, and every other key makes the robot to go back to autonomous movement.
![keys](images/keys.png)

It is also an option to run each diferent part one by one. In order to do that it is recommended to check the contents of the /launch and /src folders inside the pg proyect.
To change the map settings, the contents of the /worlds and /models folders (inside the pg proyect) can be modified.

## Issues and future work

There is still plenty of work to be done in order to achieve better results from this work. 

For example, there is a current problem that makes the robot draw a big straight line at the beginning of the program, which does not correlate with the actual movement.

More work can be to be done regarding the tweaking of the different variables found at the config file for ratslam_ros, the simulation variables set at the .world file, and the speeds that the pg_openBase_linealExplorer.cpp file sets for the robot.
The current numbers were set based on the default variables for the .world file; and from there, and following the restrictions placed by the simulation (like for example it's simulation's speed), many tests were made in order to try and find a good combination of initial variable numbers. 
It is highly recommended to address this, and try different combinations that might work better for each case.
