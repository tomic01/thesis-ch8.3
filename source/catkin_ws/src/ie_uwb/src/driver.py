#!/usr/bin/env python
import roslib; roslib.load_manifest('ie_uwb')
import rospy

from ie_uwb.msg import uwb
from ie_uwb.msg import uwb_array
import Uwb
import Wifi_Uwb
import random
import math

import time
lr=0

OFFLINE=False
anchors_pos={}
tags_config={}


def isValidIpV4(s):
    pieces = s.split('.')
    print "pieces", pieces
    if len(pieces) != 4: return False
    try: return all(0<=int(p)<256 for p in pieces)
    except ValueError: return False

def no_noise(norm):
	return norm

def real_noise(norm):
	return (norm+((0.1)*norm)*random.random())


def talker():
	global anchors_pos
	global tags_config
	ID = rospy.get_param('~id') 
	SRC= rospy.get_param('~src') 

	print "Starting with a UWB tag with the following parameters:"
	print "ID:", ID
	print "SRC:", SRC
	
	if(isValidIpV4(SRC)):			# /dev/ttyUSB0 is not valid ip
		wait=True
		while wait:
			try:
				board=Wifi_Uwb.Driver(SRC)
				rospy.on_shutdown(board.close)
				wait=False
			except:
				pass
			time.sleep(2)
	else:
		board=Uwb.Driver(SRC)
		
	pub = rospy.Publisher("tag_"+str(ID)+'/uwb_raw', uwb_array, queue_size=10)
		
	
	anchors=uwb_array()
	rate = rospy.Rate(CONTROL_RATE)

	cc = 0
	
	while not rospy.is_shutdown():
		# ----------------------------------------------------------------------------
		#		UWB REAL VALUES
		# ----------------------------------------------------------------------------
		try:
			board.read_values()
		except:
			break
		anchors.data=board.get_anchors(3) 	# This number is the minimum number of anchors, not limit

		if(anchors.data !=False):
			anchors.tag_id=ID
			anchors.timestamp=board.get_current_timestamp()
			pub.publish(anchors)

			# print cc, anchors

		cc = cc + 1
		rate.sleep()



if __name__ == '__main__':
	try:
		rospy.init_node('driver'+str(random.randint(0,9999)))
		
		
		if(rospy.has_param("~hz")):
			CONTROL_RATE=rospy.get_param("~hz")
		else:
			CONTROL_RATE=20
		print "CONTROL RATE:", CONTROL_RATE
		
		talker()

	except rospy.ROSInterruptException:
		pass
