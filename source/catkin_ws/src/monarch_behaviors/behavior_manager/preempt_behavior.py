#! /usr/bin/env python
'''
:author: Jose Nuno Pereira
:maintainer: Jose Nuno Pereira (jose.pereira@epfl.ch)
:description: Preempt a behavior request to GlobalBehaviorManager. Fill in the details".
'''
import roslib; roslib.load_manifest('monarch_behaviors')
import rospy

from monarch_msgs.msg import BehaviorSelection, KeyValuePair
from monarch_behaviors.srv import *

if __name__ == '__main__':

	rospy.wait_for_service('/mcentral/request_behavior')
	
	rospy.init_node('preempt_behavior')
	rospy.loginfo('Preempting behavior')

	request_behavior = rospy.ServiceProxy('/mcentral/request_behavior', BehaviorRequest)

	rospy.sleep(1)

	# FILL IN DETAILS OF THE BEHAVIOR YOU WANT TO SEND HERE
	behavior = BehaviorSelection()
	behavior.name = 'interactivegame'
	behavior.instance_id = 0
	behavior.robots = [0,1]
	behavior.resources = ['RNAV','interaction_interfaces']
	behavior.active = False
	behavior.parameters = []

	parameter = KeyValuePair()
	parameter.key = 'players'
	parameter.value = 'mbot01'
	behavior.parameters.append(parameter)

	parameter = KeyValuePair()
	parameter.key = 'rows'
	parameter.value = '3'
	behavior.parameters.append(parameter)

	parameter = KeyValuePair()
	parameter.key = 'cols'
	parameter.value = '4'
	behavior.parameters.append(parameter)

	parameter = KeyValuePair()
	parameter.key = 'level'
	parameter.value = '0'
	behavior.parameters.append(parameter)

	parameter = KeyValuePair()
	parameter.key = 'tracking'
	parameter.value = '0'
	behavior.parameters.append(parameter)

	parameter = KeyValuePair()
	parameter.key = 'interaction'
	parameter.value = 'nonverbal'
	behavior.parameters.append(parameter)

	answer = request_behavior(behavior)
	if answer.success is True:
		rospy.loginfo('Behavior preempted')
	else:
		rospy.loginfo('Behavior not preempted')

	rospy.sleep(0.5)