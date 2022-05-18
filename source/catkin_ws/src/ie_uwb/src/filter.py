#!/usr/bin/env python
import roslib; roslib.load_manifest('ie_uwb')
import rospy

from ie_uwb.msg import uwb
from ie_uwb.msg import uwb_array
import random
import math
# import mean
import variance
import numpy


window=10

possible_anchors = ["0000007a", "0000009b", "0000008c", "0000002c", "0000003a", "0000001c", "0000001b", "0000007b"]
# possible_anchors = ["0000007a", "0000009b", "0000008c"]

filtered_dist = {}
for k in possible_anchors:
	filtered_dist[k]=[0]*window



# filtered_dist={"0000002a":[0]*window,"0000002b":[0]*window,"0000002c":[0]*window} 		# ORIGINAL
# filtered_dist={"0000007a":[0]*window,"0000009b":[0]*window,"0000008c":[0]*window}			# This is sliding window of size 10

def callback(array):
	
	global pub
	global filtered_dist
	global tag_id
	
	print array
	# print len(array.data)
	
	if(array.tag_id==tag_id):	
		print array
		# for i in range(len(["0000002a","0000002b","0000002c"])):				# ORIGINAL

		for i in range(len(array.data)):


			if array.data[i].anchor_id == '':									# AW: the topic initialy has no ids, this is to get rid of errors
				pass

			else:

				val=100*array.data[i].radius

				# AW: Find the field with anchor name "array.data[i].anchor_id" in filtered_dist


				filtered_dist[array.data[i].anchor_id].append(val)
				filtered_dist[array.data[i].anchor_id].pop(0)


				array.data[i].variance=numpy.var(filtered_dist[array.data[i].anchor_id]) 			# Sliding window: MEAN
				array.data[i].radius=numpy.mean(filtered_dist[array.data[i].anchor_id])			# Sliding window: VARIANCE


	pub.publish(array)




if __name__ == '__main__':
	try:
				
		rospy.init_node('uwb_filtering')
		if(rospy.has_param("~id")):
			tag_id=rospy.get_param("~id")
			print "filter.py: tag id is:", tag_id
		else:
			exit()
		
		rospy.Subscriber("tag_"+str(tag_id)+"/uwb_raw", uwb_array, callback)
		pub = rospy.Publisher("tag_"+str(tag_id)+"/uwb_filtered", uwb_array,  queue_size=10)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
