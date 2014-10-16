#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import JointState

topic_name = sys.argv[1]
joint_name = sys.argv[2]
joint_value = float(sys.argv[3])

def joint_state_publisher():
    pub = rospy.Publisher(topic_name, JointState)
    rospy.init_node("joint_state_publisher")
    while not rospy.is_shutdown():
        
        j = JointState()
        j.header.stamp = rospy.Time.now()

        j.position.append(joint_value)
        j.name.append(joint_name)
        pub.publish(j)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
