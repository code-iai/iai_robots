#!/usr/bin/env python
#
#    Copyright (c) 2014 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#    Authors: Jannik Buckelo  <jannikbu@cs.uni-bremen.de>
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Controller that implements the input interface for the wsg50 gripper from 'https://github.com/code-iai/iai_robot_drivers/tree/master/wsg_50_driver'

import rospy
import sys
from iai_wsg_50_msgs.msg import SpeedCmd, PositionCmd, Status
from std_msgs.msg import Float32MultiArray, Header
from iai_control_utils.jcontroller import JController
from sensor_msgs.msg import JointState


sub = None
con = None
js_sub = None
status_pub = None

def velocity_callback(msg):
  ''' Gets the velocity in mm/s and ignores the force '''
  rospy.loginfo('Received message:')
  rospy.loginfo(msg)

  # Set velocity
  velocity = max(min(msg.speed / 1000.0, 0.42), -0.42)
  con.max_speed = abs(velocity)

  # Set direction
  cmd = Float32MultiArray()
  if velocity > 0:
    # open
    cmd.data = [-0.055, 0.055]
  else:
    # close
    cmd.data = [-0.0027, 0.0027]

  con.des_pos_cb(cmd)


def joint_state_callback(msg):
  ''' Gets the current joint state and publishes a wsg-like status message. '''
  status = Status()
  status.width = msg.position[1] - msg.position[0]
  status.speed = msg.velocity[1] - msg.velocity[0]
  status_pub.publish(status)

def position_callback(msg):
  ''' Get the position in mm with 110 meaning the fingers are at position -0.055 and 0.055 and the velocity in mm/s. The force is ignored. '''
  rospy.loginfo('Received message:')
  rospy.loginfo(msg)

  # Set velocity
  velocity = max(min(msg.speed / 1000.0, 0.42), -0.42)
  con.max_speed = abs(velocity)

  # Set position
  position = max(min(msg.pos / 2000.0, 0.055), 0.0027)
  cmd = Float32MultiArray()
  cmd.data = [-position, position]

  rospy.loginfo(str(cmd.data))

  con.des_pos_cb(cmd)


if __name__ == '__main__':
  rospy.init_node('wsg_50_interface')

  stiffness = rospy.get_param('~stiffness', 100) #Nm/m
  p_gain = rospy.get_param('~p_gain', 3.5)

  con = JController('MultiJointVelocityCommand', 0.42, p_gain, stiffness)

  sub = rospy.Subscriber('~wsg_speed_in', SpeedCmd, velocity_callback)
  sub = rospy.Subscriber('~wsg_position_in', PositionCmd, position_callback)
  js_sub = rospy.Subscriber('~joint_states', JointState, joint_state_callback)
  status_pub = rospy.Publisher('~status', Status, queue_size=3, tcp_nodelay=True, latch=False)

  rospy.spin()
