#!/usr/bin/env python
#
# Pizza Demo pepper ROS bridge
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
import time


class PepperROSBridge(object):
    def __init__(self, pepper_ip, pepper_port, map_frame='/map', pepper_frame='/pepper_base_footprint'):
        self.pepper_ip = pepper_ip
        self.pepper_port = pepper_port
        self.map_frame = map_frame
        self.pepper_frame = pepper_frame
        self.enc = json.JSONEncoder()
        self.dec = json.JSONDecoder()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        
        
    def __del__(self):
        self.sock.close()        
        
    def configure(self):
        

        try:
           
            self.sock.connect((self.pepper_ip, self.pepper_port))
            
            #Only register the rosservice after the connection to pepper worked
            self.ros_s = rospy.Service('/pepper/comm', PepperComm, self.service_handler)
            
        except socket.error:
            print "Socket error:", sys.exc_info()[0]
            #If Ctrl-C was pressed, or the node is killed, abort completely
            if rospy.is_shutdown():
                raise
            
            #If it was just a socket error (could not connect, typically), give a chance to retry
            return False
        except:
            raise
        
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
        
        #Encode into json
        pepper_net_string = enc.encode(pepper_cmd)
        
        #Send over the socket
        self.sock.sendall(pepper_net_string)
        
        timeout = 1.0  #In seconds
        
        start_time = time.time()
        
        while ((time.time() - start_time) < timeout):
            received = sock.recv(1024)
            
            if received != "":
                data = dec.decode(received)
                break
            
            else:
                time.sleep(0.05)
        
        
        rospy.loginfo('Received:')
        rospy.loginfo(data)
        
        #data['pose'] should be 3 floats:  pos_x, pos_y, rot_z (in m and rad) in /map frame
        pepper_pose = data['pose']
        
        pepper_ts = TransformStamped()
        pepper_ts.header.frame_id = self.map_frame
        pepper_ts.child_frame_id = self.pepper_frame
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
    
    rospy.init_node('pepper_ros_bridge', anonymous=False)
    
    pepper_ip = rospy.get_param('~pepper_ip', default='192.168.101.69')
    pepper_port = rospy.get_param('~pepper_port', default='3000')
    map_frame = rospy.get_param('~map_frame', default='/map')
    pepper_frame = rospy.get_param('~pepper_frame', default='/pepper_base_footprint')
    
    pb = PepperROSBridge(pepper_ip, pepper_port, map_frame, pepper_frame)
    
    
    max_retries = 10
    configured = False
    try_count = 0
    
    rospy.loginfo('pepper_ros_bridge: connecting to %s:%s. map_frame=%s pepper_frame=%s' %(pepper_ip, pepper_port, map_frame, pepper_frame))
    

    while ((not configured) and (try_count < max_retries)):
        rospy.loginfo('Trying connection to Pepper.')
        configured = pb.configure()
        
        try_count += 1
        if (not configured):
            rospy.logwarn('pepper_ros_bridge: retrying connection. Try %d/%d. Sleeping 10s.' %(try_count, max_retries))
            time.sleep(10)
            
            
    if (not configured):
        rospy.logerr('Could not initialize the PepperROSBridge. Giving up and exiting')
        sys.exit()
    
    rospy.loginfo('pepper ROS bridge is up. About to spin.')
    rospy.spin()
    rospy.loginfo('pepper ROS bridge exiting.')

    
if __name__ == '__main__':
    main()