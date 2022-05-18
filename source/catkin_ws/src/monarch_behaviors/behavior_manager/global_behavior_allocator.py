#! /usr/bin/env python
'''
:author: Jose Nuno Pereira
:maintainer: Jose Nuno Pereira (jose.pereira@epfl.ch)
:description: The Global Behavior Allocator receives behavior execution commands with
			  free variables in the robots field, indicating that the allocation of
			  those behaviors to certain robots is not the responsability of the planner
			  but rather of a centralized/distributed behavior allocation process that
			  involves the robots. This component is the first step of that process,
			  receiving such commands from the GBM, requesting bids (or fitness evaluations)
			  from each robot to that behavior execution command and making the decision
			  of which robot(s) should indeed execute the behavior. This decision is
			  then communicated back to the GBM who effectly generates the behavior
			  execution command that is passed to the robots.
'''
import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
from functools import partial

from monarch_behaviors.srv import *
from sam_helpers.writer import SAMWriter
from sam_helpers.reader import SAMReader

# ROBOT_RESPONSE_TIMEOUT specifies the time the GBA allows the robot to answer to a request for a bid/fitness.
ROBOT_RESPONSE_TIMEOUT = 5

class GlobalBehaviorAllocator(object):
	def __init__(self):

		sam_hosts = rospy.get_param("~agent_names", [])

		# Defines what robots are in operation (via monarch_situational_awareness/config/agent_names.yaml)
		self.robot_id_list = [ x.strip('mbot').strip('n') for x in sam_hosts if 'mbot' in x]

		rospy.Service('allocate_behavior', AllocationRequest, self.behavior_allocation_request_cb)

		self.robots_allocation_values = {robot: -1 for robot in self.robot_id_list}

		self.allocation_request_writers = {robot: SAMWriter('AllocateBehaviorRequest', agent_name = 'mbot' + robot) for robot in self.robot_id_list}

		self.allocation_response_readers = {robot: SAMReader('AllocateBehaviorResponse',
															 partial(self.behavior_allocation_response_cb,
																	 robot_id = robot),
															 agent_name = 'mbot' + robot) for robot in self.robot_id_list}

		rospy.loginfo('global behavior allocator initialized')

	def reset_robots_allocation_values(self):
		self.robots_allocation_values = {robot: -1 for robot in self.robot_id_list}

	def behavior_allocation_response_cb(self, msg, robot_id):
		self.robots_allocation_values[robot_id] = msg
		rospy.loginfo('GBA: Response from robot %d with value %f', robot_id, msg)

	def behavior_allocation_request_cb(self, request):
		for robot in self.robot_id_list:
			self.allocation_request_writers[robot].publish(request.behavior)
		rospy.loginfo('GBA: Request for robot allocation value sent to robots')

		r = rospy.Rate(5)
		end_time = rospy.Time.now() + rospy.Duration(ROBOT_RESPONSE_TIMEOUT)

		while rospy.Time.now() < end_time: #During a defined time period
			if all([ value != -1 for value in self.robots_allocation_values.values()]): #All robots responded to request
				break
			r.sleep()

		self.robots_allocation_values  = { k:v for k,v in self.robots_allocation_values.iteritems() if v != -1} #removing all robots that didn't respond (value == -1) from list of values

		if self.robots_allocation_values != {}:
			lowest_value = min(self.robots_allocation_values.values())
			robots = [int(k) for k,v in self.robots_allocation_values.iteritems() if v == lowest_value] #robots is int id of robot with minimum allocation value
			rospy.loginfo('GBA: Behavior allocated to robot %d', int(k))
		else:
			robots = request.behavior.robots #original robot tuple with -1 indicating free variables, will cause the GBM to refuse the behavior request
			rospy.loginfo('GBA: Behavior not allocated')

		self.reset_robots_allocation_values()
		return AllocationRequestResponse(robots)



if __name__ == '__main__':
	rospy.init_node('global_behavior_allocator')
	rospy.loginfo('Starting global behavior allocator')
	GlobalBehaviorAllocator()
	rospy.spin()