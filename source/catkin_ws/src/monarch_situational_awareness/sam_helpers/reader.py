#! /usr/bin/env python
import rospy
import roslib, roslib.message
import yaml
import exceptions
from thin_sam import lookup_slot

class SAMReader():
	def __init__(self, 
				 slot_name, 
				 reader_cb, 
				 agent_name="",
				 sam_name="sam_node"):
		(topic_name, data_class, latch) = lookup_slot(slot_name, agent_name)
		self._sub = rospy.Subscriber(topic_name, data_class, reader_cb)
		
	def remove(self):
		self._sub.unregister()