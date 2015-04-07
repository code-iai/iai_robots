#!/usr/bin/env python
#
#    Copyright (c) 2015 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz) #    Authors: Georg Bartels  <georg.bartels@cs.uni-bremen.de> #
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

import rospy
from dlr_msgs.msg import rcu2tcu
from iai_control_msgs.msg import MultiJointVelocityImpedanceCommand

def clamp(value, minimum, maximum):
  return min( max( minimum, value ), maximum ) 

class Mannequin(object):
  def __init__(self):
    self.activation_thresh = rospy.get_param('~activation_threshold', 0.05) # rad
    self.command_thresh = rospy.get_param('~command_threshold', 0.3) # rad/s
    self.stiffness = rospy.get_param('~stiffness', 80.0) # Nm/rad
    self.p_gain = rospy.get_param('~p_gain', 10.0) # 1

    feedback_str = "activation threshold: %s" % self.activation_thresh
    rospy.loginfo(feedback_str)
    feedback_str = "command threshold: %s" % self.command_thresh
    rospy.loginfo(feedback_str)
    feedback_str = "stiffness: %s" % self.stiffness
    rospy.loginfo(feedback_str)
    feedback_str = "p-gain: %s" % self.p_gain
    rospy.loginfo(feedback_str)

    self.cmd = MultiJointVelocityImpedanceCommand()
    self.cmd.velocity = [ 0.0 ] * 7
    self.cmd.stiffness = [ self.stiffness ] * 7
    self.cmd.damping = [ 0.7 ] * 7

    self.pub = rospy.Publisher('~velocity_command', MultiJointVelocityImpedanceCommand, queue_size=1, latch=False, tcp_nodelay=True)
    self.sub = rospy.Subscriber('~beasty_state', rcu2tcu, self.state_callback, queue_size=1, tcp_nodelay=True)
 
  def state_callback(self, msg):
    for i in range(0, 7):
      error = msg.robot.q[i] - msg.interpolator.q_d[i]
      if(abs(error) < abs(self.activation_thresh)):
        self.cmd.velocity[i] = 0.0
      else:
        self.cmd.velocity[i] = clamp(self.p_gain*error, -abs(self.command_thresh), abs(self.command_thresh))

    self.pub.publish(self.cmd)
    
def main():
  rospy.init_node('lwr_mannequin_mode', anonymous=True)

  mannequin = Mannequin()
 
  while(not rospy.is_shutdown()):
    rospy.sleep(0.05)
  
if __name__ == '__main__':
  main()
