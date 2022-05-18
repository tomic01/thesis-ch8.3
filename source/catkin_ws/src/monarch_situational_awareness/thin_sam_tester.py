#! /usr/bin/env python
import threading
# ROS
import rospy
# package
from monarch_msgs.msg import PersonLocalizationTrackingDataArray
from sam_helpers.writer import SAMWriter

def createWriter():
    SAMWriter("People_Localization_Tracker")

if __name__ == '__main__':
    rospy.init_node('thin_sam_tester_py')

    for i in range(0,4):
        thread = threading.Thread(target = createWriter)
        thread.start()

    rospy.spin()