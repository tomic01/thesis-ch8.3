#! /usr/bin/env python

# Author: Jose Nuno Pereira
# Description: Creates state machine that forces sequence of behaviors sequentially.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros
from functools import partial

from std_msgs.msg import Bool
from monarch_behaviors.srv import BehaviorRequest
from monarch_msgs.msg import BehaviorSelection, KeyValuePair, KeyValuePairArray

from sam_helpers.reader import SAMReader

def behavior_result_cb(result, status):#, old = old_message):
	print '\n\ncallback'
	print 'result ='
	print result
	print 'status time ='
	print status.get_time()
	print 'result header stamp to sec = ' + str(result.header.stamp.to_sec())
	print 'status get time to sec = ' + str(status.get_time().to_sec())
	if result.header.stamp.to_sec() > status.get_time().to_sec() + 1:
		print 'changed status'
		status.set_time(result.header.stamp)
		status.set_behavior_finished(True)

class BehaviorExecutionStatus:
	def __init__(self):
		self.behavior_finished = False
		self.time = rospy.Time.now()

	def set_behavior_finished(self, value):
		self.behavior_finished = value

	def get_behavior_finished(self):
		return self.behavior_finished

	def set_time(self, new_time):
		self.time = new_time

	def get_time(self):
		return self.time
		

class GotoState(smach.State):
	"""Executes gotolocation behavior"""
	def __init__(self, args):
		smach.State.__init__(self,
							 outcomes = ['succeeded',
							 			 'preempted',
							 			 'aborted'])

		self.location = args

	def execute(self, userdata):
		
		behavior = BehaviorSelection()
		behavior.name = 'gotolocation'
		behavior.instance_id = 0
		behavior.robots = [int(robot_id),]
		behavior.resources = ['RNAV',]

		parameter = KeyValuePair()
		parameter.key = 'location'
		parameter.value = self.location
		behavior.parameters.append(parameter)

		behavior.active = True

		response = request_behavior(behavior)
		if response.success != True:
			return 'aborted'

		behavior_status.set_behavior_finished(False)

		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			
			if self.preempt_requested() or rospy.is_shutdown():
				behavior.active = False
				request_behavior(behavior)
				behavior_result_reader.remove()
				behavior_mcentral_result_reader.remove()

				self.service_preempt()
				return 'preempted'

			if behavior_status.get_behavior_finished() == True:
				return 'succeeded'

			r.sleep()

		behavior.active = False
		request_behavior(behavior)
		behavior_result_reader.remove()
		behavior_mcentral_result_reader.remove()

		self.service_preempt()
		return 'preempted'

class DispatchCAState(smach.State):
	"""Executes dispatchca behavior"""
	def __init__(self, args):
		smach.State.__init__(self,
							 outcomes = ['succeeded',
							 			 'preempted',
							 			 'aborted'])

		self.tell_user = args

	def execute(self, userdata):
		
		behavior = BehaviorSelection()
		behavior.name = 'dispatchca'
		behavior.instance_id = 0
		behavior.robots = [int(robot_id),]
		behavior.resources = ['interaction_interfaces','RNAV']

		parameter = KeyValuePair()
		parameter.key = 'activated_cas'
		parameter.value = 'ca04'
		behavior.parameters.append(parameter)

		parameter.key = 'tell_user'
		parameter.value = self.tell_user
		behavior.parameters.append(parameter)

		behavior.active = True

		response = request_behavior(behavior)
		if response.success != True:
			return 'aborted'

		behavior_status.set_behavior_finished(False)

		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			
			if self.preempt_requested() or rospy.is_shutdown():
				behavior.active = False
				request_behavior(behavior)
				behavior_result_reader.remove()
				behavior_mcentral_result_reader.remove()

				self.service_preempt()
				return 'preempted'

			if behavior_status.get_behavior_finished() == True:
				return 'succeeded'

			r.sleep()

		behavior.active = False
		request_behavior(behavior)
		behavior_result_reader.remove()
		behavior_mcentral_result_reader.remove()

		self.service_preempt()
		return 'preempted'

class InteractiveGameState(smach.State):
	"""Executes interactivegame behavior"""
	def __init__(self):
		smach.State.__init__(self,
							 outcomes = ['succeeded',
							 			 'preempted',
							 			 'aborted'])

	def execute(self, userdata):
		
		behavior = BehaviorSelection()
		behavior.name = 'interactivegame'
		behavior.instance_id = 0
		behavior.robots = [int(robot_id), 0]
		behavior.resources = ['RNAV','interaction_interfaces']

		print 'robot = ' + 'mbot' + robot_id +', robot id = ' + str(int(robot_id))

		parameter = KeyValuePair()
		parameter.key = 'players'
		parameter.value = 'mbot' + robot_id
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
		parameter.value = 'none'
		behavior.parameters.append(parameter)

		behavior.active = True

		response = request_behavior(behavior)
		if response.success != True:
			return 'aborted'

		behavior_status.set_behavior_finished(False)

		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			
			if self.preempt_requested() or rospy.is_shutdown():
				behavior.active = False
				request_behavior(behavior)
				behavior_result_reader.remove()
				behavior_mcentral_result_reader.remove()

				self.service_preempt()
				return 'preempted'

			if behavior_status.get_behavior_finished() == True:
				return 'succeeded'

			r.sleep()

		behavior.active = False
		request_behavior(behavior)
		behavior_result_reader.remove()
		behavior_mcentral_result_reader.remove()

		self.service_preempt()
		return 'preempted'

class CancelState(smach.State):
	"""cancels the plan """
	def __init__(self):
		smach.State.__init__(self,
							 outcomes = ['preempted'])

		self.subscriber = rospy.Subscriber('cancel_plan', Bool, self.cancel_cb)
		self.cancel = False

	def cancel_cb(self, data):
		if data.data == True:
			self.cancel = True

	def execute(self, userdata):


		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			
			if self.preempt_requested() or self.cancel:
				self.subscriber.unregister()
				self.service_preempt()
				return 'preempted'

			r.sleep()

#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('sequencer')
  rospy.loginfo('Starting up')

  robot_id = '01'

  rospy.wait_for_service('/mcentral/request_behavior')
  request_behavior = rospy.ServiceProxy('/mcentral/request_behavior', BehaviorRequest)

  behavior_status = BehaviorExecutionStatus()

  behavior_result_reader = SAMReader('[mbot' + robot_id + '] CurrentBehaviorResult',
  									 partial(behavior_result_cb,
  									 		 status = behavior_status))

  behavior_mcentral_result_reader = SAMReader('[mpc01] CurrentBehaviorResult',
  									 		  partial(behavior_result_cb,
  									 		 		  status = behavior_status))

  sm_sequence = smach.StateMachine(outcomes = ['succeeded',
  									  'preempted',
  									  'aborted'])

  with sm_sequence:
  	sm_sequence.add('GOTO1',
  		   GotoState('game_start_isr'),
  		   transitions = {'succeeded':'DISPATCHCA1'})

  	sm_sequence.add('DISPATCHCA1',
  		   DispatchCAState('mbot_tell_game_started'),
  		   transitions = {'succeeded':'INTERACTIVEGAME'})

  	sm_sequence.add('INTERACTIVEGAME',
  		   InteractiveGameState(),
  		   transitions = {'succeeded':'GOTO2'})

  	sm_sequence.add('GOTO2',
  		   GotoState('game_start_isr'),
  		   transitions = {'succeeded':'DISPATCHCA2'})

  	sm_sequence.add('DISPATCHCA2',
  		   DispatchCAState('mbot_tell_game_has_finished'))


  sm_cancel = smach.StateMachine(outcomes = ['preempted'])

  with sm_cancel:
  	sm_cancel.add('CANCEL',
  				  CancelState())


  def term_cb(outcome_map):
  	return True

  def out_cb(outcome_map):
  	if outcome_map['SEQUENCE'] == 'succeeded':
  		return 'succeeded'
  	if outcome_map['CANCELSEQUENCE'] == 'preempted':
  		return 'preempted'
  	else:
  		return 'succeeded'

  sm_conc = smach.Concurrence(outcomes = ['succeeded',
  										   'preempted'],
  							   default_outcome = 'succeeded',
  							   child_termination_cb = term_cb,
  							   outcome_cb = out_cb)

  with sm_conc:
  	sm_conc.add('SEQUENCE',sm_sequence)
  	sm_conc.add('CANCELSEQUENCE',sm_cancel)


  sm_top = smach.StateMachine(outcomes = ['succeeded'])

  with sm_top:
  	sm_top.add('CONC',
  				sm_conc,
  				transitions = {'preempted':'FINAL'})

  	sm_top.add('FINAL',
  				DispatchCAState('mbot_tell_game_problem'),
  				transitions = {'aborted':'succeeded',
  							   'preempted':'succeeded'})

  sm_top.execute()
  rospy.spin()

