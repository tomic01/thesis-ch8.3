#! /usr/bin/env python
'''
:author: Jose Nuno Pereira
:maintainer: Jose Nuno Pereira (jose.pereira@epfl.ch)
:description: The Global Behavior Manager receives behavior execution commands from
			  social-aware planner (SAP) and redirects them to the appropriate robots
			  via SAM slots "MBotXXExecuteBehavior".
'''
import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
from functools import partial

from monarch_msgs.msg import BehaviorSelection ,BehaviorSelectionArray, BehaviorRequestLog
from sam_helpers.writer import SAMWriter
from sam_helpers.reader import SAMReader
from monarch_behaviors.srv import *

# BEHAVIOR_REQUEST_TIMEOUT specifies the time the GBM allows the robot to start the request behavior. If the behavior is not started during that time (checked with current executing behaviors SAM slot) then GBM issues a cancelation request and return unsuccessful to the service that request the execution of the behavior
BEHAVIOR_REQUEST_TIMEOUT = 10

class GlobalBehaviorManager(object):
	"""GlobalBehaviorManager contains subscriber to output from social-aware planner
	   and one SAM writer for each robot (MBotXXExecuteBehavior slot)."""
	def __init__(self):

		sam_hosts = rospy.get_param("~agent_names", [])

		# Defines what robots are in operation (via monarch_situational_awareness/config/agent_names.yaml)
		robot_id_list = [ x.strip('mbot').strip('n') for x in sam_hosts if 'mbot' in x]

		rospy.Service('request_behavior', BehaviorRequest, self.handle_behavior_request)

		self.behavior_allocator = rospy.ServiceProxy('allocate_behavior', AllocationRequest)

		# Behavior Request logger - used to bag info on behaviors sent by the GBM
		self.SAP_behavior_requests_pub = rospy.Publisher('SAP_behavior_requests', BehaviorRequestLog)

		self.execute_behavior_writers = {int(robot): SAMWriter('ExecuteBehavior', agent_name = 'mbot' + robot) for robot in robot_id_list}

		self.robots_current_executing_behaviors = {int(robot): BehaviorSelectionArray() for robot in robot_id_list}

		self.current_executing_behaviors_readers = {int(robot): SAMReader('CurrentExecutingBehaviors',
																		  partial(self.current_executing_behaviors_cb,
																		  		  robot_id = int(robot)),
																		  agent_name = 'mbot' + robot) for robot in robot_id_list}

		#### (Added code) Special case: Index 0 refers to mpc01
		self.execute_behavior_writers[0] = SAMWriter('ExecuteBehavior', agent_name = 'mpc01')
		self.robots_current_executing_behaviors[0] = BehaviorSelectionArray()
		self.current_executing_behaviors_readers[0] = SAMReader('CurrentExecutingBehaviors',
																partial(self.current_executing_behaviors_cb,
																robot_id = 0),
																agent_name = 'mpc01')
		####

		rospy.loginfo('global behavior manager initialized - Robots in operation: %s', str(robot_id_list))

	def current_executing_behaviors_cb(self,msg,robot_id):
		self.robots_current_executing_behaviors[robot_id] = msg

	def handle_behavior_request(self,request):

		# Check if there are free variables in behavior request (-1 in robots)
		if -1 in request.behavior.robots:
			#allocate robots to this behavior
			response = self.behavior_allocator(request.behavior)
			request.behavior.robots = response.robots

		if set(request.behavior.robots).issubset(set(self.execute_behavior_writers)):
			for robot in request.behavior.robots:
				self.execute_behavior_writers[robot].publish(request.behavior)
				if request.behavior.active is True:
					rospy.loginfo('GBM: sent command for behavior %s activation to robot %d',request.behavior.name,robot)
				else:
					rospy.loginfo('GBM: sent command for behavior %s cancelation to robot %d',request.behavior.name,robot)

			success_robot = {int(robot): False for robot in request.behavior.robots}
			r = rospy.Rate(5)
			end_time = rospy.Time.now() + rospy.Duration(BEHAVIOR_REQUEST_TIMEOUT)
			while rospy.Time.now() < end_time: #During a defined time period
				for robot in request.behavior.robots: # for robots specified in the behavior
					if success_robot[robot] is False: # if behavior hasn't been activated/canceled
						if request.behavior.active is True: # if behavior is supposed to be activated
							for behavior in self.robots_current_executing_behaviors[robot].array: # check current executing behaviors of robot
								if behavior.name == request.behavior.name: # if they match with behavior being request
									success_robot[robot] = True # then behavior is being executed in that robot
									break
						else: # if behavior is supposed to be canceled
							if not self.robots_current_executing_behaviors[robot].array: # if there are no current executing behaviors
								success_robot[robot] = True # then behavior has been canceled
							else: # if there are current executing behaviors
								for behavior in self.robots_current_executing_behaviors[robot].array: # check current executing behaviors of robot
									if behavior.name == request.behavior.name: # if they match with behavior being request
										break #then behavior has not been canceled yet
									success_robot[robot] = True # if this is reached then behavior is not on current executing behaviors list for that robot and cancelation was successful
				if all(success_robot.itervalues()): # if behavior has been correctly activated/canceled in all robots then stop
					break
				r.sleep()

			if not all(success_robot.itervalues()): # if behavior hasn't been activated/canceled in all robots
				success = False
				# if behavior request was for activation send a cancelation behavior to all robots involved
				if request.behavior.active is True:
					cancelation_behavior = request.behavior
					cancelation_behavior.active = False
					for robot in cancelation_behavior.robots:
						if robot in self.execute_behavior_writers:
							self.execute_behavior_writers[robot].publish(cancelation_behavior)
							rospy.loginfo('GBM: behavior command for behavior %s to robot %d failed; sending cancelation',cancelation_behavior.name,robot)
			else:
				success = True

		else:
			success = False
			rospy.logerr('GBM: Received behavior request for non-operational robot, no behaviors sent to robots')

		# Publish log of behavior request and success
		msg = BehaviorRequestLog()
		msg.behavior = request.behavior
		msg.success = success
		self.SAP_behavior_requests_pub.publish(msg)

		return BehaviorRequestResponse(success)



if __name__ == '__main__':
	rospy.init_node('global_behavior_manager')
	rospy.loginfo('Starting global behavior manager')
	GlobalBehaviorManager()
	rospy.spin()