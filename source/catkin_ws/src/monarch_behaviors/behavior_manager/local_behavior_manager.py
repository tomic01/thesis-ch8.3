#! /usr/bin/env python
'''
:author: Jose Nuno Pereira
:maintainer: Jose Nuno Pereira (jose.pereira@epfl.ch)
:description: The Local Behavior Manager receives behavior execution commands from
			  the Global Behavior Manager via SAM slots. It contains actionlib clients
			  for each of the behaviors implemented. Upon receiving a behavior command
			  from the GBM it sends a goal to the appropriate actionlib server. Before
			  doing so it preempts any behavior the robot is executing that uses a 
			  conflicting set of resources with the behavior requested by the GBM. 
			  Multiple behaviors can be executed concurrently as long as they use a 
			  non-conflicting set of resources (empty intersection)- At the moment, the
			  idle behavior is considered as the default running behavior.
'''
import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
from threading import Lock
from functools import partial

import actionlib

from monarch_behaviors.msg import *
from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter
from monarch_msgs.msg import BehaviorSelection, BehaviorSelectionArray
from monarch_msgs.msg import KeyValuePairArray, KeyValuePair

# complete resource list (defined as tuple to maintain similarity with BehaviorSelection message)
# used to speficy resources for idle behavior: uses all resources so when any other behavior is requested it preempts idle; also, when idle is requested all other behaviors are preempted
resource_list = ('RNAV','RNAVFC','RArmMotion','RHeadMotion','RLED','RVocSnd','RVideoPrj','RScreen','PerceptComp')

def intersect(a, b):
     return list(set(a) & set(b))

class LocalBehaviorManager(object):
	"""LocalBehaviorManager contains a reader from SAM (MBotXXExecuteBehavior or Mpc01ExecuteBehavior) and 3
	   writers (current executing behaviors, actionlib feedback and actionlib result.
	   It contains one actionlib client per behavior implemented. The execution of
	   these behaviors is managed in the SAM reader callback function. The dictionary
	   self.current_executing_behaviors keeps track of what are the behaviors the
	   robot is currently executing. Keys are the names of the behaviors (str) and values
	   are the behavior selection messages (BehaviorSelection)."""
	def __init__(self):
		node_name = rospy.get_name()
		if node_name.rfind('mbot')>= 0:
			self.robot_number = node_name[node_name.rfind('mbot')+4:node_name.rfind('mbot')+6]
			self.robot_name = 'mbot' + self.robot_number
		elif node_name.rfind('mcentral')>= 0:				#### (Added code) Special case for mpc01
			self.robot_number = '0'
			self.robot_name = 'mpc01'
		# else:
		# 	rospy.loginfo('Unknown namespace...')

		self.initialize_behavior_clients()
		self.initialize_SAM_reader_writers()

		# current executing behaviors specified as dictionary
		self.current_executing_behaviors = {}
		# lock variable to disable concurrent access to self.current_executing_behaviors
		self.current_executing_behaviors_lock = Lock()
		# blocking call on current executing behavior lock; lock is always released on behavior_active_cb
		self.current_executing_behaviors_lock.acquire(False)
		self.set_current_behavior_idle()

	def initialize_behavior_clients(self):
		''' Initializes the behavior clients, add new behaviors here.'''
		self.behavior_clients = {}

		#### (Added code) Differentiate action for the interactive game
		#### depending if the code is running on mbotXX or on mpc01

		if self.robot_name.rfind('mbot')>= 0:
			self.behavior_clients['idle'] = actionlib.SimpleActionClient('behavior_idle', IdleAction)
			self.behavior_clients['patrolling'] = actionlib.SimpleActionClient('behavior_patrolling', PatrollingAction)
			self.behavior_clients['cooperativepatrolling'] = actionlib.SimpleActionClient('behavior_cooperativepatrolling', CooperativePatrollingAction)
			self.behavior_clients['patrollingandtouch'] = actionlib.SimpleActionClient('behavior_patrollingandtouch', PatrollingandTouchAction)
			self.behavior_clients['patrollingandpersondetect'] = actionlib.SimpleActionClient('behavior_patrollingandpersondetect', PatrollingandPersonDetectAction)
			self.behavior_clients['gotolocation'] = actionlib.SimpleActionClient('behavior_goto', GoToAction)
			self.behavior_clients['approachperson'] = actionlib.SimpleActionClient('behavior_approachperson', ApproachPersonAction)
			self.behavior_clients['dock'] = actionlib.SimpleActionClient('behavior_dock', DockAction)
			self.behavior_clients['undock'] = actionlib.SimpleActionClient('behavior_undock', UndockAction)
			self.behavior_clients['interaction'] = actionlib.SimpleActionClient('behavior_interaction', InteractionBehaviorAction)
			self.behavior_clients['following'] = actionlib.SimpleActionClient('behavior_following', FollowPersonAction )
			self.behavior_clients['escorting'] = actionlib.SimpleActionClient('behavior_escorting', EscortPersonAction)
			self.behavior_clients['pai'] = actionlib.SimpleActionClient('behavior_patrollingapproachinteract', PatrollingApproachInteractAction)
			self.behavior_clients['paic'] = actionlib.SimpleActionClient('behavior_patrollingapproachinteractcamera', PatrollingApproachInteractCameraAction)
			self.behavior_clients['prd'] = actionlib.SimpleActionClient('behavior_patrollingrfiddetect', PatrollingRfidDetectAction)
			self.behavior_clients['interactivegame'] = actionlib.SimpleActionClient('behavior_interactivegame_robot', InteractiveGameRobotAction)
			self.behavior_clients['dispatchca'] = actionlib.SimpleActionClient('behavior_dispatch_ca', DispatchCaAction)
			self.behavior_clients['getrfid'] = actionlib.SimpleActionClient('behavior_getrfid', GetRfidAction)
			self.behavior_clients['catchandtouch'] = actionlib.SimpleActionClient('behavior_catchandtouch', CatchAndTouchAction)
			self.behavior_clients['catchandtouchback'] = actionlib.SimpleActionClient('behavior_catchandtouchback', CatchAndTouchAction)
			self.behavior_clients['lead'] = actionlib.SimpleActionClient('behavior_lead', LeadAction)
			self.behavior_clients['taquiz'] = actionlib.SimpleActionClient('behavior_ta_quiz_game', TAQuizAction)
			self.behavior_clients['teachingassistance'] = actionlib.SimpleActionClient('behavior_teachingassistance', TeachingAssistanceAction)
			#self.behavior_clients['distributed'] = actionlib.SimpleActionClient('behavior_distributed_coverage', GoToAction)
			self.behavior_clients['checkgamearea'] = actionlib.SimpleActionClient('behavior_checkgamearea', CheckGameAreaClearAction)

		elif self.robot_name.rfind('mpc01')>= 0:
			self.behavior_clients['idle'] = actionlib.SimpleActionClient('behavior_idle', IdleAction)
			self.behavior_clients['interactivegame'] = actionlib.SimpleActionClient('behavior_interactivegame', InteractiveGameAction)


		rospy.loginfo('LBM: Waiting for behavior servers...')
		for name, client in self.behavior_clients.iteritems():
			rospy.loginfo('%s LBM: Client %s waiting', rospy.get_name(), name)
			client.wait_for_server()
			rospy.loginfo('%s LBM: Client %s ready', rospy.get_name(), name)
		rospy.loginfo('LBM: Behavior servers ready.')

	def initialize_SAM_reader_writers(self):
		''' Initializes the SAM reader for behavior commands (coming from Global
			Behavior Manager) and the SAM Writers for current executing behaviors,
			behavior feedback and behavior result. A subscriber and 3 publishers
			are implicity set up.'''
		rospy.loginfo('LBM: Initializing SAM reader and writers...')

		SAMReader('ExecuteBehavior', self.behavior_command_cb, agent_name = self.robot_name)

		self.sam_current_executing_behaviors_writer = SAMWriter('CurrentExecutingBehaviors', agent_name = self.robot_name)
		self.sam_current_behavior_feedback_writer = SAMWriter('CurrentBehaviorFeedback', agent_name = self.robot_name)
		self.sam_current_behavior_result_writer = SAMWriter('CurrentBehaviorResult', agent_name = self.robot_name)

		rospy.loginfo('LBM: SAM reader and writers ready.')

	def set_current_behavior_idle(self):
		''' Sets current executing behavior to idle behavior. Used when all other behaviors
			finish and no other behavior is requested by the GBM or to make the robot idle.'''
		# construct BehaviorSelection message
		idle = BehaviorSelection()
		idle.name = 'idle'
		idle.instance_id = 0
		idle.robots = [int(self.robot_number), ]
		idle.parameters = []
		idle.resources = resource_list
		idle.active = True

		# set idle behavior as currently executing
		self.current_executing_behaviors[idle.name] = idle

		# construct goal message
		goal = AbstractGoal()
		# send goal message to appropriate Actionlib server with callbacks for when the
		# behavior becomes active and when it finishes
		self.behavior_clients[idle.name].send_goal(goal,
										   		   active_cb = partial(self.behavior_active_cb,
										   		 					   behavior_name = idle.name),
										   		   done_cb = partial(self.behavior_done_cb,
										   		 					 behavior_name = idle.name))

	def behavior_command_cb(self, behavior):
		''' Callback function called upon reception of a behavior command. Main
			function of Local Behavior Manager. Preempts any currently executing
			behavior that uses a conflicting set of resources wrt to the behavior
			requested and is different from it. Uses appropriate behavior client 
			to send a new goal to the appropriate behavior server and the behavior 
			is added to the current executing behaviors dictionary. '''
		now = rospy.Time.now()
		if behavior.active is True:
			rospy.loginfo('LBM: behavior execution command received for behavior %s at %i %i', behavior.name,
																			  				   now.secs,
																			  				   now.nsecs)
		else:
			rospy.loginfo('LBM: behavior cancelation command received for behavior %s at %i %i', behavior.name,
																			  				 	 now.secs,
																			  				 	 now.nsecs)

		if behavior.name not in self.behavior_clients:
			rospy.loginfo('LBM: requested behavior does not exist - ignoring request')
			return

		#acquiring lock (sleeping while not available), releasing after new behavior becomes active or at next if condition if behavior requested is not to be executed
		while self.current_executing_behaviors_lock.acquire(False) is False:
			rospy.loginfo("LBM: Current executing behavior locked, behavior %s requested.",behavior.name)
			rospy.sleep(0.2)

		# check if behavior is already executing and if the behavior command is related to the same instance
		if behavior.name in self.current_executing_behaviors and behavior.instance_id == self.current_executing_behaviors[behavior.name].instance_id:
			if behavior.active is True:
				# if same instance of behavior is received with request for execution, do nothing 
				self.current_executing_behaviors_lock.release()
			else:
				# if request is for cancelation,

				if behavior.name == 'idle':
					rospy.loginfo('LBM: received request for cancelation of idle behavior - ignoring')
					# release the lock on current executing behaviors dictionary
					self.current_executing_behaviors_lock.release()
					return

				# switch current executing behavior activation state to false
				self.current_executing_behaviors[behavior.name].active = False

				# preempt executing instance of behavior
				self.behavior_clients[behavior.name].cancel_all_goals()
				self.behavior_clients[behavior.name].wait_for_result()
			return
		else:
			if behavior.active is False:
				rospy.loginfo('LBM: received request for cancelation of behavior not being executed - ignoring')
				# release the lock on current executing behaviors dictionary
				self.current_executing_behaviors_lock.release()
				return
		
		# preempt all executing behaviors with conflicting resources to behaviors
		for name, behav in self.current_executing_behaviors.items():
			if intersect(behavior.resources, behav.resources) != []:
				self.behavior_clients[name].cancel_all_goals()
				self.behavior_clients[name].wait_for_result()

		# add behavior to current executing behaviors and send goal to appropriate actionlib server
		# callbacks are specified with the appropriate behavior name
		if behavior.name == 'idle':
			self.set_current_behavior_idle()
		else:
			self.current_executing_behaviors[behavior.name] = behavior
			goal = AbstractGoal()
			goal.goal.array = behavior.parameters
			self.behavior_clients[behavior.name].send_goal(goal,
														   done_cb = partial(self.behavior_done_cb,
																			 behavior_name = behavior.name),
														   active_cb = partial(self.behavior_active_cb,
																			   behavior_name = behavior.name),
										   				   feedback_cb = partial(self.behavior_feedback_cb,
										   				 						 behavior_name = behavior.name))

	def behavior_active_cb(self, behavior_name):
		''' Callback function for when behaviors become active. Transforms current
			executing behaviors dictionary into BehaviorSelectionArray and writes
			on current executing behaviors SAM slot. Leaves out idle behavior.
			Parameter behavior_name indicates what is the name of the behavior that 
			became active (speficied when sending the goal to the Actionlib server).'''
		now = rospy.Time.now()
		rospy.loginfo('LBM: Behavior %s became active at %i %i',
					  behavior_name,
					  now.secs,
					  now.nsecs)

		# Write current executing behaviors to appropriate SAM slot
		behavior_array = BehaviorSelectionArray()
		behavior_array.header.stamp = now
		for name, behavior in self.current_executing_behaviors.iteritems():
			if name != 'idle':
				behavior_array.array.append(behavior)
		self.sam_current_executing_behaviors_writer.publish(behavior_array)

		# release the lock on current executing behaviors dictionary
		self.current_executing_behaviors_lock.release()

	def behavior_feedback_cb(self, behavior_feedback, behavior_name):
		'''Callback function for feedback from active behavior. Write feedback on 
		   current executing behavior feedback SAM slot.'''
		# add behavior name as KeyValuePair to the feedback coming from actionlib
		behavior_name_kvp = KeyValuePair()
		behavior_name_kvp.key = "behavior_name"
		behavior_name_kvp.value = behavior_name
		behavior_feedback.feedback.array.insert(0,behavior_name_kvp)
		behavior_feedback.feedback.header.stamp = rospy.Time.now()

		self.sam_current_behavior_feedback_writer.publish(behavior_feedback.feedback)

	def behavior_done_cb(self, behavior_final_state, behavior_result, behavior_name):
		'''Callback function for when behavior finishes execution (succeeded, 
		   preempted or aborted). Writes result on current executing behavior 
		   result SAM slot. Also updates current executing behaviors SAM slot. If 
		   the behavior just finished was not preempted and current executing
		   behaviors dictionary is empty then it sets the current executing 
		   behavior to idle behavior.'''
		now = rospy.Time.now()
		rospy.loginfo('LBM: Behavior %s finished execution at %i %i',
					  behavior_name,
					  now.secs,
					  now.nsecs)

		behavior_done = self.current_executing_behaviors[behavior_name]

		# deletes the behavior from the current executing behaviors dictionary
		del self.current_executing_behaviors[behavior_name]

		if behavior_name != 'idle': # any behavior except idle
			# write final behavior result in appropriate SAM slot as KeyValuePairArray
			# add behavior name to the result coming from actionlib
			behavior_name_kvp = KeyValuePair()
			behavior_name_kvp.key = "behavior_name"
			behavior_name_kvp.value = behavior_name
			behavior_result.result.array.insert(0,behavior_name_kvp)
			behavior_result.result.header.stamp = now
			self.sam_current_behavior_result_writer.publish(behavior_result.result)

		# Write current executing behaviors to appropriate SAM slot
		behavior_array = BehaviorSelectionArray()
		behavior_array.header.stamp = now
		for name, behavior in self.current_executing_behaviors.iteritems():
			if name != 'idle':
				behavior_array.array.append(behavior)
		self.sam_current_executing_behaviors_writer.publish(behavior_array)

		# if behavior was not preempted and current executing behaviors is empty
		if behavior_final_state != actionlib.simple_action_client.GoalStatus.PREEMPTED and \
		   len(self.current_executing_behaviors) == 0 and \
		   self.current_executing_behaviors_lock.acquire(False) is True:
		   	#set current executing behavior as idle
			self.set_current_behavior_idle()
			return

		# if behavior was canceled and current executing behaviors is empty
		if behavior_done.active is False and \
		   len(self.current_executing_behaviors) == 0:
		   	#set current executing behavior as idle
			self.set_current_behavior_idle()

if __name__ == '__main__':
	rospy.init_node('local_behavior_manager')
	rospy.loginfo('Starting local behavior manager')
	LocalBehaviorManager()
	rospy.spin()
