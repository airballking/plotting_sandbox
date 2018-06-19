# plotting_sandbox
A sandbox for plotting scripts for papers, projects, etc.

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Kinetic```:
```shell
source /opt/ros/kinetic/setup.bash
mkdir -p ~/plotting_ws/src
cd ~/plotting_ws
catkin init
cd src
wstool init
wstool merge https://raw.githubusercontent.com/airballking/plotting_sandbox/master/rosinstall/kinetic.rosinstall
wstool update
rosdep install --ignore-src --from-paths .
cd ..
catkin build
source ~/plotting_ws/devel/setup.bash
```

## CHECKING OUT MESHES
```shell
rosrun iai_cad_downloader download-cad-models.py kitchen/food-drinks/pancake-big
 kitchen/hand-tools kitchen/electrical-devices```
