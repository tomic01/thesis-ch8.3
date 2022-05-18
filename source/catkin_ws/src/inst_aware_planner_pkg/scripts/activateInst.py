#!/usr/bin/env python

import rospy
from std_msgs.msg import String

#activated = False

def activateInst():
    pub = rospy.Publisher('/sar/inst_to_start', String, queue_size=10)
    rospy.init_node('activateInst', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        inst_str = "Guiding"
        rospy.loginfo(inst_str)
        pub.publish(inst_str)
        rate.sleep()
        rospy.sleep(1.)
        return
        
if __name__ == '__main__':
    try:
        rospy.sleep(3.)
        activateInst()
    except rospy.ROSInterruptException:
        pass