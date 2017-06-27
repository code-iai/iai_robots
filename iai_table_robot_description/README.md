# iai_table_robot_description
URDF description of the table-mounted UR5 robots in the lab of the IAI.

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Kinetic```:
```
source /opt/ros/kinetic/setup.bash         # start using ROS Kinetic
mkdir -p ~/ws/src                          # create directory for workspace
cd ~/ws                                    # go to workspace directory
catkin init                                # init workspace
cd src                                     # go to source directory of workspace
wstool init                                # init rosinstall
wstool merge https://raw.githubusercontent.com/code-iai/iai_table_robot_description/master/rosinstall/kinetic.rosinstall
                                           # update rosinstall file
wstool update                              # pull source repositories
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin build                               # build packages
source ~/ws/devel/setup.bash               # source new overlay
```

## A preview in RVIZ
Use the accompanying ```display.launch``` to get a view the robot in ```rviz```:
```
roslaunch iai_table_robot_description display.launch
```

![rviz view](https://raw.githubusercontent.com/code-iai/iai_table_robot_description/master/docs/iai_table_robot.png)
