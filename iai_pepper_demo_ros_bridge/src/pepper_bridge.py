#!/usr/bin/env python
#
# Pizza Demo robosherlock helper
#
# Copyright (c) 2016 Universitaet Bremen, Institute for Artificial Intelligence (AGKI).
# Author: Alexis Maldonado Herrera <amaldo at cs.uni-bremen.de>
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
from iai_pepper_demo_msgs.srv import PepperComm, PepperCommRequest, PepperCommResponse

from geometry_msgs.msg import TransformStamped, PointStamped

import PyKDL as kdl
#import tf


import socket
import sys
import json
import SocketServer

class PepperROSBridge(object):
    def __init__(self):
        self.pepper_ip = "192.168.101.69"
        self.pepper_port = 3000
        self.enc = json.JSONEncoder()
        self.dec = json.JSONDecoder()
        self.sock = None
        
    def __del__(self):
        self.sock.close()        
        
    def configure(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.pepper_ip, self.pepper_port))
    
        self.ros_s = rospy.Service('/pepper/comm', PepperComm, self.service_handler)
        
        return True
    
    def service_handler(self, req):
        #req is of type PepperCommRequest
        
        #prepare message to Pepper

        pepper_cmd = {}
        pepper_cmd['speak'] = req.speak + '\n'
        coord = [req.point_at.point.x, req.point_at.point.y, req.point_at.point.z]
        pepper_cmd['point_at'] = coord
        pepper_cmd['point_at_frame'] = req.point_at.header.frame_id
        pepper_cmd['get_pose'] = req.get_pose
        pepper_cmd['get_heard'] = req.get_heard
        
        pepper_net_string = enc.encode(pepper_cmd)
        
        self.sock.sendall(pepper_net_string)
        
        
        counter = 0
        while (counter < 100):
            counter += 1
            received = sock.recv(1024)
            
            if received != "":
                print "Received: {}".format(received)
                print "Parsing received json"
                
                data = dec.decode(received)
                counter = 100        
        
        
        rospy.loginfo('Received:')
        rospy.loginfo(data)
        
        pepper_pose = data['pose']
        
        pepper_ts = TransformStamped()
        pepper_ts.header.frame_id = 'pepper_frame'
        pepper_ts.transform.translation.x = pepper_pose[0]
        pepper_ts.transform.translation.y = pepper_pose[1]
        pepper_ts.transform.translation.z = 0.0
        
        rotz = pepper_pose[2]
        quat = kdl.Rotation.RotZ(rotz).GetQuaternion()
        pepper_ts.transform.rotation.x = quat[0]
        pepper_ts.transform.rotation.y = quat[1]
        pepper_ts.transform.rotation.z = quat[2]
        pepper_ts.transform.rotation.w = quat[3]
        
                        
        ans.heard = data['heard']
        ans.pepper_pose = pepper_ts
        
        ans = PepperCommResponse()
        
        return ans
  

def main():
    pb = PepperROSBridge()
    pb.configure()
    rospy.spin()
    

    
if __name__ == '__main__':
    main()