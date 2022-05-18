#! /usr/bin/env python

# Behavior implemented: dispatch_ca
# Author: JNP
# Description: dispatch_ca is a behavior designed for dispatching a ca directly from planner
#				to interaction manager and send back the result (feedback to be added later)			 

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import smach
import smach_ros

from functools import partial

from monarch_msgs.msg import KeyValuePair
from std_msgs.msg import Bool

from monarch_behaviors.msg import DispatchCaAction, DispatchCaResult

from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction
from mosmach.change_conditions.topic_condition import TopicCondition


# define state DispatchCa
class DispatchCa(MonarchState):
	""" This state dispatch a CA (via RunCa Action and listens to appropriate topics to
		write the result to sent back via action lib result) """

	def __init__(self):
		MonarchState.__init__(self,
							  state_outcomes=['succeeded','preempted'],
							  input_keys=['action_goal'])

		# Dispactch CA
		runCaAction = RunCaAction(self,
								  ca = self.sendCa_cb,
								  is_dynamic = True)
		self.add_action(runCaAction)
	

	def sendCa_cb(self, userdata):
		return userdata.action_goal.goal

# define state WriteResult
class WriteResult(MonarchState):
	""" This state checks if the ca is over and writes the result"""

	def __init__(self):
		MonarchState.__init__(self,
							  state_outcomes=['succeeded','preempted'],
							  input_keys=['action_result'],
							  output_keys=['action_result'])

		# Read result topic from Interaction Manager
		interactingCondition = TopicCondition(self,
											  'interacting',
											  Bool,
											  self.subInteracting_cb)
		self.add_change_condition(interactingCondition,['succeeded'])

	def subInteracting_cb(self, data, userdata):
		print 'data.data = ' + str(data.data)
		if data.data is False:
			userdata.action_result.result.array[0].value = str(True)
			return 'succeeded'

	
if __name__ == '__main__':
	rospy.init_node('behavior_dispatch_ca')
	rospy.loginfo('Dispactch CA Starting up')


	###################################################
	############## MAIN STATE MACHINE #################
	###################################################

	# Create the top level SMACH state machine
	sm = smach.StateMachine(outcomes=['succeeded','preempted'],
							input_keys = ['action_goal'],
							output_keys = ['action_result'])

	sm.userdata.action_result = DispatchCaResult()
	result = KeyValuePair()
	result.key = 'interaction_over'
	result.value = 'False'
	sm.userdata.action_result.result.array.append(result)

	with sm:
		sm.add('DISPATCH_CA',
			   DispatchCa(),
			   transitions = {'succeeded':'WRITE_RESULT'})

		sm.add('WRITE_RESULT',
			   WriteResult())

	action_name = rospy.get_name()  
	asw = smach_ros.ActionServerWrapper(action_name,
										DispatchCaAction,
										wrapped_container = sm,
										succeeded_outcomes = ['succeeded'],
										preempted_outcomes = ['preempted'],
                                		goal_key = 'action_goal',
                                   		result_key = 'action_result')

	
	asw.run_server()
	rospy.spin()


