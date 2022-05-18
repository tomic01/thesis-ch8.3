#! /usr/bin/env python
import rospy
import roslib, roslib.message
import yaml
import exceptions
from thin_sam import lookup_slot
	
class SAMWriter():
	def __init__(self, 
				 slot_name, 
				 agent_name="", 
				 sam_name="sam_node", 
				 q_size=1):
		(topic_name, data_class, latch) = lookup_slot(slot_name, agent_name)
		self._pub = rospy.Publisher(topic_name, data_class, latch=latch, queue_size=q_size)
							
	def publish(self, data):
		self._pub.publish(data)

	def remove(self):
		self._pub.unregister()
