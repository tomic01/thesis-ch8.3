#! /usr/bin/env python

# Behavior implemented: interactive_game
# Author: Lorenzo Sarti
# Description: interactive_game is a behavior designed for activating the state mahchine
#				on the robot involved in tha games which takes care of handling all the 
#				goals sent by the game core that runs on the mcentral machine. This state 
#				also provides navigation feedbacks via SAM slots to the game core which 
#				keeps track of the state of the game.			 

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import smach
import smach_ros
import math
import numpy
import tf
from functools import partial
from monarch_msgs_utils import key_value_pairs as kvpa

from smach_ros import simple_action_state
from move_base_msgs.msg import *
from monarch_msgs.msg import GestureExpression, KeyValuePairArray, GameGoal, GameHriGoal

from std_msgs.msg import Int16, Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter
from monarch_situational_awareness.srv import RemoveReader
from monarch_behaviors.msg import InteractiveGameRobotAction


def pose2pose_stamped(x, y, t):
  pose_stamped = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return pose_stamped

# define state checkGoal
class checkGoal(smach.State):
	""" This state registers a callback on the SAM slot in order to receive Goals. 
		If the goal is valid then it transition to the actionlib client that sends 
		the received goal to the navigation system.   """

	def __init__(self):
		smach.State.__init__(self, outcomes=['valid','preempted'], input_keys=['goal_in'], output_keys=['goal_out'])
		self.goalReceived = False
		self.goalReader = None

	def receiveGoal_cb(self, data, extra):

		if extra.goal_in != data.goal.target_pose:
			self.goalReceived = True
			extra.goal_out = data.goal.target_pose

	def isGoalValid(self, goal):

		if goal.pose.position.x < -500 and \
			goal.pose.position.y < -500:
			return False
		return True

	def execute(self, userdata):
		rospy.loginfo('Executing state CHECKGOAL')

		robot_id = rospy.get_namespace()
		robot_id = robot_id.strip('/')

		#self.goalReader = SAMReader("["+robot_id+"] flowfreeGoal", partial(self.receiveGoal_cb, extra = userdata))
		self.goalReader = rospy.Subscriber('../multimaster_bridge__' + robot_id + '__flowfreegoal', GameGoal, partial(self.receiveGoal_cb, extra = userdata))
		r = rospy.Rate(2)

		while not rospy.is_shutdown():

			if self.preempt_requested():
				#self.goalReader.remove()
				self.goalReader.unregister()
				self.service_preempt()
				return 'preempted'

			if self.goalReceived and self.isGoalValid(userdata.goal_in):
				#self.goalReader.remove()
				self.goalReader.unregister()
				self.goalReceived = False
				return 'valid'
			self.goalReceived = False
			r.sleep()

# define state watchState
class watchState(smach.State):
	""" This state runs concurrently with the SM HANDLEGOAL and registers a callback
		on the SAM slot that receives the goal. This state provides navigation feedbacks
		to the game core running on mcentral"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['preempted'], input_keys=['feedback_in','goal_in'], output_keys=['goal_out', 'feedback_out'])
		self.newGoalReceived = False
		self.goalActive = False
		self.goalReader = None
		self.feedbackWriter = None

		self.poseReader = None
		self.clearance = None

	def receiveGoal_cb(self, data, extra):

		#print "------> cb: " , data.goal.target_pose.pose.position.x ,  data.goal.target_pose.pose.position.y
		#print "------> extra : " , extra.goal_in.pose.position.x ,  extra.goal_in.pose.position.y

		if extra.goal_in != data.goal.target_pose:
			#print "*********** New goal received **************"
			self.newGoalReceived = True
			self.clearance = data.clearanceRadius
			extra.goal_out = data.goal.target_pose

	def pose_cb(self, data, extra):

		# print "------------> " , self.clearance, extra.feedback_out
		if( self.clearance != None and \
		 math.sqrt(pow(data.position.x - extra.goal_in.pose.position.x, 2) + pow(data.position.y - extra.goal_in.pose.position.y, 2)) <  self.clearance):

			# print "According to clearance the robot is succeded bc distance from goal " , math.sqrt(pow(data.position.x - extra.goal_in.pose.position.x, 2) + pow(data.position.y - extra.goal_in.pose.position.y, 2))
			extra.feedback_out = 'Succeeded'


	def isGoalValid(self, goal):
		if goal.pose.position.x < -500 and \
			goal.pose.position.y < -500:
			return False 
		return True

	def execute(self, userdata):
		rospy.loginfo('Executing state WATCHSTATE')

		robot_id = rospy.get_namespace()
		robot_id = robot_id.strip('/')
		#self.goalReader = SAMReader("["+robot_id+"] flowfreeGoal", partial(self.receiveGoal_cb, extra = userdata))
		self.goalReader = rospy.Subscriber('../multimaster_bridge__' + robot_id + '__flowfreegoal', GameGoal, partial(self.receiveGoal_cb, extra = userdata))
		self.feedbackWriter = SAMWriter("["+robot_id+"] flowfreeGoalFeedback")
		#self.poseReader = SAMReader("["+robot_id+"] tfPose", partial(self.pose_cb, extra = userdata))
		self.poseReader = rospy.Subscriber('amcl_pose', Pose, partial(self.pose_cb, extra = userdata))

		r = rospy.Rate(2)

		while not rospy.is_shutdown():

			if self.newGoalReceived and self.isGoalValid(userdata.goal_in):
				#print "-----> ACTIVATE"
				self.newGoalReceived = False
				userdata.feedback_out = 'Active'

			elif not self.isGoalValid(userdata.goal_in):
				#print "-------> NOGOAL"
				self.newGoalReceived = False
				userdata.feedback_out = 'NoGoal'

			if self.preempt_requested():
				self.service_preempt()

				#self.goalReader.remove()
				#self.poseReader.remove()
				#self.feedbackWriter.remove()

				self.goalReader.unregister()
				self.poseReader.unregister()
				
				return 'preempted'

			#publish feedback
			self.feedbackWriter.publish(userdata.feedback_in)
			r.sleep()


# define state HriHandleGoal
class HriHandleGoal(smach.State):
	""" This state receives HRI 'goals' (CEs for now, later CAs) and sends the to
		appropriate MOSMACH actions."""

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','preempted'],
								   input_keys = ['goal_interaction_type','goal_interaction','goal_pose'],
								   output_keys = ['goal_interaction_type','goal_interaction','goal_pose'])
		self.goalReceived = False
		self.goalReader = None
		self.robotPose = pose2pose_stamped(-1000, -1000, -1000)
		self.poseReader = None

	def receiveGoal_cb(self, data, userdata):
		self.goalReceived = True

		userdata.goal_interaction_type = data.interaction_type
		userdata.goal_interaction = data.interaction

		robot_x = self.robotPose.position.x
		robot_y = self.robotPose.position.y
		player_x = data.pose.pose.position.x
		player_y = data.pose.pose.position.y
		goal_theta = numpy.arctan2(robot_y - player_y, robot_x - player_x) - math.pi
		userdata.goal_pose = pose2pose_stamped(robot_x,robot_y,goal_theta)

	def updateRobotPose_cb(self,data):
		self.robotPose = data

	def execute(self, userdata):
		rospy.loginfo('Executing state HriHandleGoal')

		robot_id = rospy.get_namespace()
		robot_id = robot_id.strip('/')
		if robot_id.rfind('mcentral')>= 0:				#### (Added code) Special case for mpc01
			robot_id = 'mpc01'

		# check Goal
		#self.goalReader = SAMReader("["+robot_id+"] flowfreeHRI", partial(self.receiveGoal_cb, userdata = userdata))
		self.goalReader = rospy.Subscriber('../multimaster_bridge__' + robot_id + 'flowfreehri', GameHriGoal, partial(self.receiveGoal_cb, userdata = userdata))

		# check Robot Pose
		#self.poseReader = SAMReader('['+robot_id+'] tfPose',self.updateRobotPose_cb)
		self.poseReader = rospy.Subscriber('amcl_pose', Pose, self.updateRobotPose_cb) 

		r = rospy.Rate(2)

		while not rospy.is_shutdown():

			if self.preempt_requested():
				self.service_preempt()
				#self.goalReader.remove()
				#self.poseReader.remove()
				self.goalReader.unregister()
				self.poseReader.unregister()
				return 'preempted'

			if self.goalReceived:
				self.goalReceived = False
				#self.goalReader.remove()
				#self.poseReader.remove()
				self.goalReader.unregister()
				self.poseReader.unregister()
				return 'succeeded'

			r.sleep()

# define state HriHandleGoal
class HriPublishGoal(smach.State):
	""" This state receives HRI 'goals' (CEs for now, later CAs) and sends the to
		appropriate MOSMACH actions."""

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','preempted'], input_keys = ['goal_interaction_type','goal_interaction'])

		self.hriCePublisher = rospy.Publisher('express_gesture', GestureExpression, queue_size = 10)
		self.hriCaPublisher = rospy.Publisher('ca_activations', KeyValuePairArray, queue_size = 10)

	def execute(self, userdata):
		rospy.loginfo('Executing state HriPublishGoal')

		if self.preempt_requested():
			self.service_preempt()
			return 'preempted'

		if userdata.goal_interaction_type == 'ce':

			msg = GestureExpression()
			msg.gesture = userdata.goal_interaction
			self.hriCePublisher.publish(msg)
			rospy.sleep(1)
			return 'succeeded'

		elif userdata.goal_interaction_type == 'ca':

			msg = kvpa.from_dict({'activated_cas':'ca04','tell_user':userdata.goal_interaction})
			self.hriCaPublisher.publish(msg)
			rospy.sleep(1)
			return 'succeeded'


#####
def nav_child_term_cb(outcome_map):

	print 'termination callback'
	if outcome_map['HANDLEGOAL'] and not outcome_map['WATCHSTATE']:
		#print '!!!preempting!!!'
		return True

def action_result_cb(userdata, status, result):

	if status == simple_action_state.GoalStatus.SUCCEEDED:
		#print '------------> SUCCEDED'
		userdata.action_result = 'Succeeded'
	else:
		userdata.action_result = 'NoGoal'

def main():
	rospy.init_node('behavior_game_robot')

	###################################################
	############# NAV GOAL HANDLING SM ################
	###################################################

	nav_conc = smach.Concurrence(outcomes=['succeeded','preempted'],
							   default_outcome='preempted',
							   outcome_map={'preempted':{'HANDLEGOAL':'preempted'},
							   				'succeeded':{'HANDLEGOAL':'succeeded'}},
							   child_termination_cb = nav_child_term_cb)

	nav_conc.userdata.feedback = 'NoGoal'
	nav_conc.userdata.goal_sm = pose2pose_stamped(-1000, -1000, -1000)
	print '*********** ', nav_conc.userdata.goal_sm


	with nav_conc:

		###################################################
		######## HANDLEGOAL STATE MACHINE #################
		###################################################

		sm_goal = smach.StateMachine(outcomes=['succeeded','preempted'], input_keys=['goal_sm'], output_keys=['feedback', 'goal_sm'])

		with sm_goal:

			sm_goal.add('CHECKGOAL', checkGoal(), 
									transitions={'valid':'SENDGOAL',
												 'preempted':'preempted'},
									remapping={'goal_in':'goal_sm',
											   'goal_out':'goal_sm'})

			sm_goal.add('SENDGOAL', smach_ros.SimpleActionState('move_base', MoveBaseAction,
								goal_slots=['target_pose'],
								result_cb= action_result_cb,
								output_keys=['action_result']),
								remapping={'target_pose':'goal_sm',
										   'action_result':'feedback'},
								transitions={'succeeded':'succeeded',
										 	'aborted':'preempted',
										 	'preempted':'preempted'})


		nav_conc.add('HANDLEGOAL', sm_goal)
			
		nav_conc.add('WATCHSTATE', 
					watchState(),
					remapping={'feedback_in':'feedback',
								'goal_in':'goal_sm',
								'feedback_out':'feedback',
								'goal_out':'goal_sm'})

	sm_nav = smach.StateMachine(outcomes = ['preempted'])
	
	with sm_nav:
		sm_nav.add('NAV_GAME_INTERFACE',
				   nav_conc,
				   transitions = {'preempted':'preempted',
				   				  'succeeded':'NAV_GAME_INTERFACE'})

	###################################################
	############# HRI GOAL HANDLING SM ################
	###################################################

	sm_hri = smach.StateMachine(outcomes = ['preempted','aborted'])
	sm_hri.userdata.goal_interaction_type = 'ce'
	sm_hri.userdata.goal_interaction = 'NoGoal'
	sm_hri.userdata.goal_pose = pose2pose_stamped(-1000, -1000, -1000)

	with sm_hri:
		sm_hri.add('HRI_GOAL_RECEIVER',
				   HriHandleGoal(),
				   transitions = {'succeeded':'TURN_TO_PLAYER'})

		sm_hri.add('TURN_TO_PLAYER',
				   smach_ros.SimpleActionState('move_base',
				   							   MoveBaseAction,
				   							   goal_slots = ['target_pose']),
				   remapping = {'target_pose':'goal_pose'},
				   transitions = {'succeeded':'HRI_GOAL_PUBLISHER'})

		sm_hri.add('HRI_GOAL_PUBLISHER',
				   HriPublishGoal(),
				   transitions = {'succeeded':'HRI_GOAL_RECEIVER'})


	###################################################
	########## TOP CONCURRENCE (NAV+HRI) ##############
	###################################################

	top_conc = smach.Concurrence(outcomes = ['preempted'],
								 default_outcome = 'preempted')

	with top_conc:
		top_conc.add('NAV',
					 sm_nav)

		top_conc.add('HRI',
					 sm_hri)

	###################################################
	############## MAIN STATE MACHINE #################
	###################################################

	# Create the top level SMACH state machine
	sm_top = smach.StateMachine(outcomes=['preempted'])
	
	with sm_top:
		sm_top.add('CONC',
				   top_conc)

	action_name = rospy.get_name()  
	asw = smach_ros.ActionServerWrapper(action_name,
										InteractiveGameRobotAction,
										wrapped_container = sm_top,
										preempted_outcomes = ['preempted'])


	sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_MINE')
	sis.start()
	asw.run_server()
	rospy.spin()


if __name__ == '__main__':
	main()

