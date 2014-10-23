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
  * set the fixed frame to ```base_link```
  * add a plugin of type ```RobotModel```
  * add the ```TF``` plugin

Then you should see something like this...

![rviz view](https://raw.github.com/code-iai/iai_robots/master/iai_boxy_bringup/doc/boxy_sim_rviz.png)


## Manually moving the joints
For testing purposes, you can command the joints through publishing velocity commands from the console.

### Moving the torso
The torso of the robot offers a velocity-resolved interface. To periodically tell the torso to move the up with 2cm/s, call:

```rostopic pub -r 20 /torso_vel/command iai_control_msgs/MultiJointVelocityCommand '{velocity: [0.02]}'```

NOTE: The current version of the torso joint simulation comes without a watchdog. It stops the torso if it has not received a command for 100ms.

### Moving the head joints
The velocity-resolved controllers for the head_pan_joint and the head_tilt_joint have the same inferace as the triangle_base_joint in the torso. So, you can move them just like the torso joint. You only need to send your commands to the appropriate topics.

NOTE: The current version of the head joints simulation comes without a watchdog. So, you have to send a stop command to stop them!

### Moving the arms
The move, for instance, the first two joints of the right arm with a velocity of -0.1rad/s, call

```rostopic pub -r 20 /r_arm_vel/command iai_control_msgs/MultiJointVelocityImpedanceCommand '{velocity: [-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0], stiffness: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]}'```

NOTE: The simulated arm controllers already have watchdogs stopping them if you do not send a command every 100ms.

NOTE: In the current version of the arm simulation, all fields of the command messages but ```velocity``` are ignored.
