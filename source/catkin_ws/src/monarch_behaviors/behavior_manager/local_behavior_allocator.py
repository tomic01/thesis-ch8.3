#! /usr/bin/env python
'''
:author: Jose Nuno Pereira
:maintainer: Jose Nuno Pereira (jose.pereira@epfl.ch)
:description: The Local Behavior Allocator receives requests for bids/fitness values
			  for the execution of behaviors sent by SAP with free variables (via GBA).
			  It calculates these bid values (at the moment calculated differently for 
			  different behaviors and still not available for all) and returns them
			  via SAM to the GBA. The decision about the final allocation of the behavior
			  is made by the GBA.
'''
import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
from scipy.spatial import distance
from operator import itemgetter
from threading import Lock
from functools import partial

from sam_helpers.writer import SAMWriter
from sam_helpers.reader import SAMReader
from monarch_msgs_utils import key_value_pairs as kvpa

import rospkg
import os.path
import sys
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("scout_navigation"))

from planner import FastMarching
from locations import Locations

from monarch_msgs.msg import BehaviorSelection

ALLOC_TYPE = 'CBAA' # Choice between CENTRAL, CBAA, CBPAE

MBOT_RADIUS = 0.25

class LocalBehaviorAllocator(object):
	def __init__(self):

		self.robot_id = rospy.get_namespace()
		self.robot_id = self.robot_id.strip('/')

		self.position = (0,0)

		locmap = rospy.get_param("map")
		navmap = rospy.get_param("navmap", locmap)
		locfile = rospy.get_param("locations", None)

		# Load locations database
		try:
			if locfile is not None: 
				l = Locations(locfile)
			else:
				l = Locations(navmap)
			rospy.loginfo("Loaded locations from %s"%(navmap))
		except IOError:
			l = None
		self.locations = l

		self.motion_planner = FastMarching()
		self.motion_planner.load_map(navmap)
		self.motion_planner.setup(clearance = MBOT_RADIUS)

		self.position_reader = SAMReader('tfPose', self.position_cb, agent_name = self.robot_id)

		if ALLOC_TYPE == 'CENTRAL':
			self.allocation_request_reader = SAMReader('AllocateBehaviorRequest', self.allocation_request_cb, agent_name  = self.robot_id)
			self.allocation_response_writer = SAMWriter('AllocateBehaviorResponse', agent_name = self.robot_id)

		if ALLOC_TYPE == 'CBAA':
			# Algorithm variables
			self.behaviors = {} # Behaviors (tasks) to be assined dictionary (/mathcal{J} in Choi et al 2009: key as tuple in format (behavior_name, instance_id), value: behavior
			self.assignment = {} # key same as self.behaviors, value: bool(assigned or not)
			self.own_bids = {} # key same as self.behaviors, value: float (bid value)
			self.winning_bids = {} # key same as self.behaviors, value: float (bid value)
			self.all_bids = {} # key: tuple (same as self.behaviors, robot name) value: bid value

			# Auxiliary variables
			self.selected_behavior = False
			self.robot_executing_behaviors = False
			self.own_bids_lock = Lock()
			sam_hosts = rospy.get_param("~agent_names", [])
			self.teammates_list = [ x.strip('n') for x in sam_hosts if 'mbot' in x and self.robot_id not in x]

			# SAM Readers/Writers
			self.allocation_request_reader = SAMReader('AllocateBehaviorRequest', self.add_task_cb, agent_name  = self.robot_id)
			self.current_executing_behaviors_reader = SAMReader('CurrentExecutingBehaviors', self.update_execution_state_cb, agent_name = self.robot_id)
			self.shared_bids_writer = SAMWriter('CBAASharedBids', agent_name = self.robot_id)
			self.shared_bids_readers = {robot:SAMReader('CBAASharedBids', partial(self.read_teammate_bid_cb, robot_name = robot), agent_name = robot) for robot in self.teammates_list}
			self.execution_publisher = rospy.Publisher('execute_behavior', BehaviorSelection, queue_size = 1)

		rospy.loginfo('local behavior allocator initialized')

	def position_cb(self, pose):
		self.position = (pose.position.x, pose.position.y)

	def update_execution_state_cb(self, msg):
		if msg.array == []:
			self.robot_executing_behaviors = False
		else:
			self.robot_executing_behaviors = True

	def path_length(self, origin, goal):
		"""
		origin and goal given as 2D tuples
		returns length of the path of the robot between origin and goal points (calculated with FMM)
		"""
		self.motion_planner.solve(goal)
		path = self.motion_planner.get_path(origin[0], origin[1])
		path.append(goal)

		length = 0
		for i in xrange(0, len(path)-1):
			length += distance.euclidean(path[i], path[i+1])

		return length

	def allocation_request_cb(self, behavior):
		"""
		Receives a behavior allocation request
		writes a behavior allocation value/bid on SAM slot 'AllocateBehaviorResponse'
		Behaviors handled individually by type/name initially
		Will provide general approach in the future
		"""
		print behavior

		if behavior.name == 'gotolocation':
			parameters = {p.key: p.value for p in behavior.parameters}
			(goalx, goaly, goalt) = tuple(self.locations.get_location(parameters['location']))
			value = self.path_length(self.position, (goalx, goaly))
			self.allocation_response_writer.publish(value)

	def add_task_cb(self, behavior):
		"""
		Callback for behavior allocation requests used in CBAA
		Add behaviors to behaviors dictionary (key as tuple in format (behavior_name, instance_id), value: behavior)
		Only considers gotolocation behavior for now, generalize in the future
		"""
		if behavior.name == 'gotolocation':
			self.behaviors[(behavior.name, behavior.instance_id)] = behavior
			self.assignment[(behavior.name, behavior.instance_id)] = False
			self.winning_bids[(behavior.name, behavior.instance_id)] = 0.0
			self.own_bids_lock.acquire(True)
			self.own_bids[(behavior.name, behavior.instance_id)] = self.calculate_bid(behavior)
			self.all_bids[(behavior.name, behavior.instance_id),self.robot_id] = self.own_bids[(behavior.name, behavior.instance_id)]
			self.own_bids_lock.release()

	def read_teammate_bid_cb(self, msg, robot_name):
		"""
		Callback for SAMReader for robot_name cbaa shared bids
		"""
		teammate_bids = kvpa.to_dict(msg)
		for behavior,bid in teammate_bids.iteritems():
			modified_behavior = (behavior.split(',')[0].strip("('"), int(behavior.split(',')[1].strip(")'")))
			self.all_bids[modified_behavior, robot_name] = float(bid)

	def calculate_bid(self,behavior):
		"""
		Bid value calculation for CBAA
		Preliminary version considers bid as 1/path_length, only valid for gotolocation behaviors
		"""
		parameters = {p.key: p.value for p in behavior.parameters}
		(goalx, goaly, goalt) = tuple(self.locations.get_location(parameters['location']))
		value = self.path_length(self.position, (goalx, goaly))
		value = 1/value
		#value = float('%.6f' % value)
		return value

	def cbaa_select_behavior(self):
		"""
		Phase 1 of CBAA
		"""
		self.own_bids_lock.acquire(True)
		higher_bids = {k:self.own_bids[k] for k,v in self.behaviors.iteritems() if self.own_bids[k] > self.winning_bids[k]}
		self.own_bids_lock.release()
		if higher_bids != {}:
			self.selected_behavior = max(higher_bids.iteritems(), key = itemgetter(1))[0]
			self.assignment[self.selected_behavior] = True
			self.winning_bids[self.selected_behavior] = self.own_bids[self.selected_behavior]

	def cbaa_send_bids(self):
		"""
		Pack winning_bids dict as kvpa and publish via SAMWriter
		"""
		winning_bids_str = {str(k):str(v) for k,v in self.winning_bids.iteritems()}
		kvpa_bids = kvpa.from_dict(winning_bids_str)
		self.shared_bids_writer.publish(kvpa_bids)

	def cbaa_receive_bids(self):
		"""
		Blocking function, waits for received bids from all robots
		Receiving bids assync via SAMReader callbacks, these modify list
		checks list of robots which bid on behavior from all bids is
		the same as the list of teammates plus the robot itself
		"""
		r = rospy.Rate(2)
		for behavior in self.behaviors:
			while True:
				received_robots = [x[1] for x in self.all_bids.keys() if x[0] == behavior]
				if set(received_robots) == set(self.teammates_list).union(set([self.robot_id,])):
					break
				r.sleep()

	def cbaa_update_task(self):
		"""
		Phase 2 of CBAA
		"""
		for behavior in self.behaviors.keys():
			behavior_bids = {k[1]:v for k,v in self.all_bids.iteritems() if k[0] == behavior}
			high_bid = max(behavior_bids.iteritems(), key = itemgetter(1))
			self.winning_bids[behavior] = high_bid[1]

			if behavior == self.selected_behavior:
				teammate_high_bid = high_bid[0]
				if teammate_high_bid != self.robot_id:
					self.assignment[self.selected_behavior] = False
					self.selected_behavior = False

	def cbaa_execute_selected_behavior(self):
		"""
		dispatches selected behavior to Local Behavior Manager via topic
		"""
		self.behaviors[self.selected_behavior].robots = [int(self.robot_id.strip('mbot')),]
		self.execution_publisher.publish(self.behaviors[self.selected_behavior])

	def cbaa_clear_selected_behavior(self):
		"""
		Clear CBAA variables of references to behavior just
		allocated and dispatched to Local Behavior Manager
		"""
		del self.behaviors[self.selected_behavior]
		del self.assignment[self.selected_behavior]
		del self.own_bids[self.selected_behavior]
		del self.winning_bids[self.selected_behavior]
		for robot in self.teammates_list:
			del self.all_bids[self.selected_behavior, robot]
		del self.all_bids[self.selected_behavior, self.robot_id]
		self.selected_behavior = False

	def cbaa_clear_high_bid_behaviors(self):
		"""
		Clear CBAA variables of references to behaviors
		allocated in this iteration to other robots. these
		are selected by verifying the highest bid for each robot
		"""
		# Missing implementation. Obviously.
		pass

	def cbaa_hard_variable_reset(self):
		"""
		Cleans all CBAA variables
		"""
		self.behaviors = {}
		self.assignment = {}
		self.own_bids = {}
		self.winning_bids = {}
		self.all_bids = {}
		self.selected_behavior = False
		

	def print_debug_values(self):
		print 'behaviors = ' + str(self.behaviors)
		print 'assignment = ' + str(self.assignment)
		print 'own_bids = ' + str(self.own_bids)
		print 'winning_bids = ' + str(self.winning_bids)
		print 'all bids = ' + str(self.all_bids)
		print 'selected behavior = ' + str(self.selected_behavior)

	def cbaa(self):
		"""
		Implementation of Consensus-Based Auction Algorithm (Choi et al, 2009)
		"""
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			# Phase 1
			if self.behaviors != {} and not self.robot_executing_behaviors: # behaviors to be assigned list not empty and robot not executing anything else
				self.cbaa_select_behavior()
			print 'PHASE 1'
			self.print_debug_values()
			# Phase 2
			if self.behaviors != {}:
				self.cbaa_send_bids()
			self.cbaa_receive_bids()
			self.cbaa_update_task()
			print 'PHASE 2'
			self.print_debug_values()
			# Not in original CBAA
			if self.selected_behavior != False:
				self.cbaa_execute_selected_behavior()
				self.cbaa_clear_selected_behavior()
			# self.cbaa_clear_high_bid_behaviors() ------ NOT IMPLEMENTED, POSSIBLY NOT BEST SOLUTION
			self.cbaa_hard_variable_reset()

			r.sleep()



if __name__ == '__main__':
	rospy.init_node('local_behavior_allocator')
	rospy.loginfo('Starting local behavior allocator')
	lba = LocalBehaviorAllocator()
	if ALLOC_TYPE == 'CBAA':
		lba.cbaa()
	rospy.spin()