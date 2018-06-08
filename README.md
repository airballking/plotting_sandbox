# plotting_sandbox
A sandbox for plotting scripts for papers, projects, etc.

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Kinetic```:
```
source /opt/ros/indigo/setup.bash          # start using ROS Indigo
mkdir -p ~/plotting_ws/src                  # create directory for workspace
cd ~/plotting_ws                            # go to workspace directory
catkin init                                # init workspace
cd src                                     # go to source directory of workspace
wstool init                                # init rosinstall
```

OLD STUFF BELOW:
```
wstool merge https://raw.githubusercontent.com/SemRoCo/giskard_core/master/rosinstall/catkin.rosinstall
                                           # update rosinstall file
wstool update                              # pull source repositories
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin build                               # build packages
source ~/giskard_ws/devel/setup.bash       # source new overlay
