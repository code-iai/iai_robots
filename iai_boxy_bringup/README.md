# Lightweight Boxy simulation

## Installation

First, make sure you have the following packages installed:
  * apt-get install ros-hydro-pr2-mechanism-model ros-hydro-pr2-controller-manager ros-hydro-control-toolbox ros-hydro-pr2-mechanism-controllers


In addition to this repo, you need to have two more repos from code-iai in your workspace:
  * https://github.com/code-iai/iai_control_pkgs
  * https://github.com/code-iai/iai_common_msgs

Build them all by running ```catkin_make```

## Start-up
Run the following commands each in a new terminal:
  * ```roscore```
  * ```roslaunch iai_boxy_bringup boxy_sim.launch```
  * ```rosrun rviz rviz```

In rviz,
  * set the fixed frame to ```map```
  * add a plugin of type ```RobotModel```
  * add the ```TF``` plugin

Then you should see something like this...

![rviz view](https://raw.github.com/code-iai/iai_robots/master/iai_boxy_bringup/doc/boxy_sim_rviz.png)


## Manually moving the joints
For testing purposes, you can command the joints through publishing velocity commands from the console.

### Moving the torso
The torso of the robot offers a velocity-resolved interface. To periodically tell the torso to move the up with 2cm/s, call:

```rostopic pub -r 20 /torso_vel/command iai_control_msgs/MultiJointVelocityCommand '{velocity: [0.02]}'```

NOTE: The current version of the torso joint simulation comes with a watchdog. It stops the torso if it has not received a command for 100ms.

### Moving the head joints
The pan-tilt unit of the head also offers a velocity-resolved interface. To move both joints with 0.1rad/s send:

```rostopic pub -r 20 /head_vel/command iai_control_msgs/MultiJointVelocityCommand '{velocity: [0.1, 0.1]}'```

NOTE: Also the simulated head controller comes with a watchdog. It stops the head joints if it has not received a command for 100ms.

### Moving the arms
The arms of the robot offer a velocity-resolved interface which also allows you to set desired joint stiffness with every command. The move, for instance, the first two joints of the right arm with a velocity of -0.1rad/s, and have all joints have a stiffness of 80Nm/rad call:

```rostopic pub -r 20 /r_arm_vel/command iai_control_msgs/MultiJointVelocityImpedanceCommand '{velocity: [-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0], stiffness: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]}'```

NOTE: The simulated arm controllers also have watchdogs stopping them if you do not send a command at least every 100ms.

NOTE: In the current version of the arm simulation, all fields of the command messages but ```velocity``` are ignored, i.e. we do not have a stiffness simulation.

### Moving the base
The base of the robot offers a twist interface. This is how to command it to move from the terminal:

```rostopic pub -r 10 /odometry_sim/command geometry_msgs/Twist '{linear: {x: 0.1, y: 0.2}, angular: {z: 0.1}}'```

This command asks for translations of 10cm/s in x- and 20cm/s in y-direction, and a rotation of 0.1rad/s around the z-axis of the base_footprint of the robot.

NOTE: All other fields of the command message will be ignored.

NOTE: There is a watchdog monitoring the commands. If none come in for 0.5s the base automatically stops.
