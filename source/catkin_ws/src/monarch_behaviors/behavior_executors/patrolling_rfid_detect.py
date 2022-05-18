#! /usr/bin/env python


import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import smach
import smach_ros

from monarch_msgs.msg import RfidReading
from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction
from mosmach.change_condition import ChangeCondition
from mosmach.change_conditions.topic_condition import TopicCondition
from monarch_msgs_utils import key_value_pairs as kvpa

import math

from monarch_behaviors.msg import PatrollingRfidDetectAction

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from monarch_msgs.msg import *

from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction

from sam_helpers.writer import SAMWriter
from monarch_situational_awareness.msg import *

from monarch_situational_awareness.msg import ReaderProperties
from monarch_situational_awareness.srv import CreateReader, CreateReaderResponse


def pose2alib(x, y, t):
	goal = PoseStamped(header=Header(frame_id="/map"),pose=Pose(position=Point(x, y, 0),orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
	
	return MoveBaseGoal(target_pose=goal)

def pose2pose_stamped(x, y, t):
  pose_stamped = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return pose_stamped

#	States
class FindRfidTag(MonarchState):
	def __init__(self):
		MonarchState.__init__(self,
							  state_outcomes = ['succeeded',
							  					'failed'])
		rospy.loginfo("FindRfidTag")
		topicName = 'rfid_tag'
		condition_outcomes = ['succeeded','failed']
		tc = TopicCondition(self, topicName, RfidReading, self.rfidCondition_cb)
		self.add_change_condition(tc, condition_outcomes)

	def rfidCondition_cb(self, data, userdata):
		if data.tag_id == 5275 or data.tag_id == 5267 or data.tag_id == 6892 or data.tag_id == 2430:
			return 'succeeded'
		else:
			return 'failed'

class ActivateCaState(MonarchState):
	
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'])
		rospy.loginfo("ActivateCaState")

		msg = kvpa.from_dict({'activated_cas':'ca15'})
		runGreetingCa = RunCaAction(self, msg)
		self.add_action(runGreetingCa)

class CheckCaOverState(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'])
		rospy.loginfo("CheckCaOverState")

		# Read result topic from Interaction Manager
		interactingCondition = TopicCondition(self,
											  'interacting',
											  Bool,
											  self.subInteracting_cb)
		self.add_change_condition(interactingCondition,['succeeded'])

	def subInteracting_cb(self, data, userdata):
		if data.data is False:
			return 'succeeded'


class PatrollingStateSetInitialState(smach.State):
	"""Called after visiting a waypoint, sets the initial state of the state machine to
	next waypoint (so that after preemption and new execution patrolling resumes where
	it stopped)."""
	def __init__(self):
		smach.State.__init__(self,
							 outcomes = ['succeeded','preempted'],
							 input_keys = ['current_state'],
							 output_keys = ['current_state'])
		 
	def execute(self,userdata):
		if self.preempt_requested():
			self.service_preempt()
			return 'preempted'
		
		#	advance current state
		userdata.current_state = (userdata.current_state + 1) if userdata.current_state + 1 < len(waypoints) else 0
		
		#	set initial state to next waypoint
		sm_patrolling.set_initial_state(['WAYPOINT_%s'%(userdata.current_state)])
		
		return 'succeeded'



#	Main   
if __name__ == '__main__':
	rospy.init_node('behavior_patrollingrfiddetect')
	rospy.loginfo('Starting up')
	
	#	Create waypoints for patrolling
	#	IW 8th floor isr8-v07cr_new_gamearea.yaml
	#waypoints = [(0.35,5.15,-1.32),
				#(2.15,-1.60,-1.86),
				#(8.00,-3.90,-3.14),
				#(0.60,5.05,1.15), 
				#(12.40,11.30,-0.30),
				#(19.05,8.50,-1.57),
				#(12.30,0.70,-0.00)]

	  # EPFL Robotics Arena
  	waypoints = [(7.2,12.8,1.78),
  	              (7.0,10.8,-0.1),
  	              (6.05,8.7,-1.74)]
	
	opposite_waypoints = [(l[0],l[1],(l[2]+3.14)%(2*3.14)) for l in reversed(waypoints[1:(len(waypoints)-1)])]
	waypoints.extend(opposite_waypoints)

	
	#	State Machine PATROLLING
	sm_patrolling = smach.StateMachine(outcomes = ['preempted','aborted'])
	sm_patrolling.userdata.current_state = 0
	
	with sm_patrolling:
		for (i,w) in enumerate(waypoints):
			next_waypoint = 'WAYPOINT_%s'%(i+1) if i+1<len(waypoints) else 'WAYPOINT_0'
			sm_patrolling.add('WAYPOINT_%s'%(i),smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=pose2alib(*w)),transitions = {'succeeded': 'SETINITIALSTATE_%s'%(i)})
			sm_patrolling.add('SETINITIALSTATE_%s'%(i),PatrollingStateSetInitialState(),transitions = {'succeeded':next_waypoint})


	#	State Machine RUN CA
	sm_ca = smach.StateMachine(outcomes = ['succeeded','preempted','aborted','failed'])	
	with sm_ca:
		sm_ca.add('ACTIVATECA',
					ActivateCaState(),
					transitions = {'succeeded':'CHECKCAOVER'})
		sm_ca.add('CHECKCAOVER',
					CheckCaOverState())

	#	State GetRFID
	sm_getrfid = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])
	with sm_getrfid:
		sm_getrfid.add('GETRFIDSTATE',
					   FindRfidTag(),
					   transitions = {'failed':'GETRFIDSTATE'})


	#	Concurrence State Machine PATROLLING + RFID detect

	def termination_cm_1_cb(outcome_map):
		if outcome_map['GETRFIDSM'] == 'succeeded':
			return True
		if outcome_map['PATROLLING'] == 'preempted':
			return True
		if outcome_map['PATROLLING'] == 'aborted':
			return True
		else:
			return False

	def outcome_cm_1_cb(outcome_map):
		if outcome_map['GETRFIDSM'] == 'succeeded':
			return 'succeeded'
		if outcome_map['PATROLLING'] == 'preempted':
			return 'preempted'
		if outcome_map['PATROLLING'] == 'aborted':
			return 'aborted'
		else:
			return 'succeeded'


	cm_1 = smach.Concurrence(outcomes = ['succeeded','preempted','aborted'],
							 default_outcome = 'succeeded',
							 child_termination_cb = termination_cm_1_cb,
							 outcome_cb = outcome_cm_1_cb)
	
	with cm_1:
		cm_1.add('PATROLLING',sm_patrolling)
		cm_1.add('GETRFIDSM',sm_getrfid)


	mm = smach.StateMachine(outcomes = ['preempted','aborted'])
	with mm:
		mm.add('PATROLANDRFID',
				cm_1,
				transitions = {'succeeded':'RUNCA'})
				
		mm.add('RUNCA',
				sm_ca,
				transitions = {'succeeded':'PATROLANDRFID',
							   'failed':'PATROLANDRFID'})
	
	#	Action Server Wrapper - action_{goal,feedback,result} are the default values and don't need to be passed as input
	action_name = rospy.get_name()  
	asw = smach_ros.ActionServerWrapper(action_name,
										PatrollingRfidDetectAction,
										wrapped_container = mm,
										preempted_outcomes = ['preempted'],
										aborted_outcomes = ['aborted'])
	
	#	Create and start the introspection server
	#sis = smach_ros.IntrospectionServer('server_name', mm, '/SM_ROOT')
	#sis.start()
	
	asw.run_server()
	rospy.spin()
	
	#sis.stop()
