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

import math
from monarch_behaviors.msg import PatrollingApproachInteractAction

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
		MonarchState.__init__(self, state_outcomes = ['succeeded','preempted'], input_keys=[''],output_keys=[''])
		rospy.loginfo("FindRfidTag")
		topicName = 'rfid_tag'

		rfidCondition = """
if len(str(data.tag_id)) != 0:
	value = 'succeeded'
"""

		condition_outcomes = ['succeeded','preempted']
		tc = TopicCondition(self, topicName, RfidReading, rfidCondition)
		self.add_change_condition(tc, condition_outcomes)


class RunCaState(MonarchState):
	
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=[''],output_keys=[''])
		rospy.loginfo("RunCaState")
		runGreetingCaState = RunCaAction(self, 'give_greetings_child')
		self.add_action(runGreetingCaState)


class FindRfidTagTimerState(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes = ['succeeded','preempted'])
		rospy.loginfo('FInd RFID Timer INIT')
	
	def execute(self,userdata):
		rospy.loginfo('Find RFID Timer EXECUTE')
		
		for i in range(120):
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			rospy.sleep(1)

		return 'succeeded'


class PatrollingTimerState(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes = ['succeeded','preempted'])
		rospy.loginfo('Patrolling Timer INIT')

		self.timer_list = [10,30,30]
	
	def execute(self,userdata):
		rospy.loginfo('Patrolling Timer EXECUTE')
		
		personLocalizationArray = PersonLocalizationTrackingDataArray()
		personLocalization = PersonLocalizationTrackingData()
		personLocalization.id = 5001
		personLocalization.pose.pose.position.x = 0.41
		personLocalization.pose.pose.position.y = 4.46
		personLocalization.pose.pose.orientation.z = -0.85
		personLocalization.pose.pose.orientation.w = 0.51
		personLocalizationArray.locations.append(personLocalization)
		
		wd = SAMWriter("Fake_People_Localization_Tracker")
		
		for i in range(self.timer_list[0]):
			if self.preempt_requested():
				self.service_preempt()
				self.timer_list.append(self.timer_list.pop(0))
				return 'preempted'
			wd.publish(personLocalizationArray)
			rospy.sleep(1)
			
		self.timer_list.append(self.timer_list.pop(0))
		#rospy.sleep(5)
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


class ApproachStateStart(smach.State):
	"""ApproachStateStart sets the move base goal the robot must visit from the action goal."""
	def __init__(self):
		smach.State.__init__(self,
							 outcomes = ['user_position_set','start_preempted'],
							 output_keys = ['start_pose_out'])
		
	#	Needs to be improved to return position if user_id is not passed correctly
	def people_localization_tracker_cb(self, data):
		'''Callback function for People_Localization_Tracker SAMReader. Sets the user position if user is found.'''
		for user in range(0,len(data.locations)):
			if data.locations[user].id == self.user_id:
				self.start_pose_out = PoseStamped(header = Header(frame_id = "/map"),pose = data.locations[user].pose.pose)
				self.user_found = True
				
			else:
				rospy.logerr('Approach Person Behavior: user %d not found.',self.user_id)
				
	def execute(self, userdata):
		#	publish info to the console for the user
		rospy.loginfo('StateStart: Executing...')
		
		r = rospy.Rate(2)
		
		self.user_id = 5001
		self.user_found = False
		
		#	Create SAM Reader
		rospy.wait_for_service('create_reader')
		reader_service = rospy.ServiceProxy('create_reader', CreateReader)
		reader_properties = ReaderProperties(slot_name = 'Fake_People_Localization_Tracker')
		reader_response = CreateReaderResponse()
		
		while reader_response.success is False:
			reader_response = reader_service(reader_properties)
			r.sleep()
			
		sam_reader = rospy.Subscriber(reader_response.topic_name,PersonLocalizationTrackingDataArray,self.people_localization_tracker_cb)
		
		while not rospy.is_shutdown() and self.user_found is False:
			if self.preempt_requested():
				sam_reader.unregister() #	unregistering from people localization tracker topic
				self.service_preempt()
				return 'start_preempted'
			
			#	sleep a bit
			r.sleep()
		
		userdata.start_pose_out = self.start_pose_out
		sam_reader.unregister() #	unregistering from people localization tracker topic
		
		return 'user_position_set'


#	Main   
if __name__ == '__main__':
	rospy.init_node('behavior_patrollingapproachinteract')
	rospy.loginfo('Starting up')
	
	#	Create waypoints for patrolling
	#	IW 8th floor isr8-v07cr_new_gamearea.yaml
	waypoints = [(4.65,11.85,-1.99),
				(0.35,5.15,-1.32),
				(2.15,-1.60,-1.86),
				(8.00,-3.90,-3.14),
				(12.40,11.30,-0.30),
				(19.05,8.50,-1.57),
				(12.30,0.70,-0.00)]
	
	opposite_waypoints = [(l[0],l[1],(l[2]+math.pi)%(2*math.pi)) for l in waypoints]
	opposite_waypoints.reverse()
	waypoints.extend(opposite_waypoints)

	
	#	State Machine PATROLLING
	sm_patrolling = smach.StateMachine(outcomes = ['preempted','aborted'])
	sm_patrolling.userdata.current_state = 0
	
	with sm_patrolling:
		for (i,w) in enumerate(waypoints):
			next_waypoint = 'WAYPOINT_%s'%(i+1) if i+1<len(waypoints) else 'WAYPOINT_0'
			sm_patrolling.add('WAYPOINT_%s'%(i),smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=pose2alib(*w)),transitions = {'succeeded': 'SETINITIALSTATE_%s'%(i)})
			sm_patrolling.add('SETINITIALSTATE_%s'%(i),PatrollingStateSetInitialState(),transitions = {'succeeded':next_waypoint})

	
	#	State Machine APPROACH
	sm_approach = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])
	
	with sm_approach:
		sm_approach.add('START',
						ApproachStateStart(),
						transitions = {'user_position_set':'MOVE',
									   'start_preempted':'preempted'},
						remapping = {'start_pose_out':'pose_stamped'})
		sm_approach.add('MOVE',
						smach_ros.SimpleActionState('move_base',
													MoveBaseAction,
													goal_slots=['target_pose']),
													transitions = {'succeeded':'succeeded'},
													remapping = {'target_pose':'pose_stamped'})


	#	State Machine RUN CA
	sm_ca = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])	
	with sm_ca:
		sm_ca.add('RunCaState',RunCaState())

	#	State Patrol Timer
	sm_patrol_timer = smach.StateMachine(outcomes = ['succeeded','preempted'])
	with sm_patrol_timer:
		sm_patrol_timer.add('PATROLTIMERSTATE',
							PatrollingTimerState())

	#	State GetRFID
	sm_getrfid = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])
	with sm_getrfid:
		sm_getrfid.add('GETRFIDSTATE',
					   FindRfidTag())

	# State get rfid timer
	sm_getfid_timer = smach.StateMachine(outcomes = ['succeeded','preempted'])
	with sm_getfid_timer:
		sm_getfid_timer.add('RFIDTIMERSTATE',
							  FindRfidTagTimerState())


	#	Concurrence State Machine PATROLLING + TIMER

	def termination_cm_1_cb(outcome_map):
		if outcome_map['TIMERSM'] == 'succeeded':
			return True
		if outcome_map['PATROLLING'] == 'preempted':
			return True
		if outcome_map['PATROLLING'] == 'aborted':
			return True
		else:
			return False

	def outcome_cm_1_cb(outcome_map):
		if outcome_map['TIMERSM'] == 'succeeded':
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
		cm_1.add('TIMERSM',sm_patrol_timer)

	
	#Concurrence State Machine PATROLLING + TIMER


	def termination_cm_2_cb(outcome_map):
		if outcome_map['APPROACH'] == 'succeeded' or outcome_map['APPROACH'] == 'preempted' or outcome_map['APPROACH'] == 'aborted':
			return True
		if outcome_map['GETRFIDSM'] == 'succeeded':
			return True
		if outcome_map['GETRFIDTIMERSM'] == 'succeeded':
			return True
		else:
			return False

	def outcome_cm_2_cb(outcome_map):
		if outcome_map['APPROACH'] == 'succeeded' or outcome_map['GETRFIDSM'] == 'succeeded' or outcome_map['GETRFIDTIMERSM'] == 'succeeded':
			return 'succeeded'
		if outcome_map['APPROACH'] == 'preempted':
			return 'preempted'
		if outcome_map['APPROACH'] == 'aborted':
			return 'aborted'
		else:
			return 'succeeded'

	cm_2 = smach.Concurrence(outcomes = ['succeeded','preempted','aborted'],
							 default_outcome = 'succeeded',
							 child_termination_cb = termination_cm_2_cb,
							 outcome_cb = outcome_cm_2_cb)
	
	with cm_2:
		cm_2.add('APPROACH',sm_approach)
		cm_2.add('GETRFIDSM',sm_getrfid)
		cm_2.add('GETRFIDTIMERSM',sm_getfid_timer)


	mm = smach.StateMachine(outcomes = ['preempted','aborted'])
	with mm:
		mm.add('PATROLANDTIMER',
				cm_1,
				transitions = {'succeeded':'APPROACHRFIDTIMER'})
		
		mm.add('APPROACHRFIDTIMER',
				cm_2,
				transitions = {'succeeded':'RUNCA'})
				
		mm.add('RUNCA',
				sm_ca,
				transitions = {'succeeded':'PATROLANDTIMER'})
	
	#	Action Server Wrapper - action_{goal,feedback,result} are the default values and don't need to be passed as input
	action_name = rospy.get_name()  
	asw = smach_ros.ActionServerWrapper(action_name,
										PatrollingApproachInteractAction,
										wrapped_container = mm,
										preempted_outcomes = ['preempted'],
										aborted_outcomes = ['aborted'])
	
	#	Create and start the introspection server
	#sis = smach_ros.IntrospectionServer('server_name', mm, '/SM_ROOT')
	#sis.start()
	
	asw.run_server()
	rospy.spin()
	
	#sis.stop()
