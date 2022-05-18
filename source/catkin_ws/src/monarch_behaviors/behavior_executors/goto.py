#! /usr/bin/env python

# Behavior implemented: GoTo
# Author: Jose Nuno Pereira
# Description: GoTo is a behavior designed for moving the robot to a speficic location.
#              This location is passed as a string in the goal of the action. For now
#              the coordinates of each location are hard-coded in the behavior. This 
#              must be changed in the future. The behavior is implemented as a three 
#              state SMACH. The first state reads the action goal and sets the location
#              to be reached in the second state using the MoveBaseAction. The third
#              state sets the result of the GoTo action. 

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros
from scipy.spatial import distance

from monarch_behaviors.msg import GoToAction, GoToResult
from monarch_msgs.msg import KeyValuePairArray
from monarch_msgs.msg import KeyValuePair
from monarch_msgs_utils import key_value_pairs as kvpa

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from sam_helpers.reader import SAMReader
from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction

import rospkg
import os.path
import sys
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path("scout_navigation"))

from planner import FastMarching
from locations import Locations

def pose2pose_stamped(x, y, t):
  pose_stamped = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return pose_stamped

#
#   States
#

class StateStart(smach.State):
  """StateStart sets the move base goal the robot must visit from the action goal."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['location_set','start_preempted'],
                          input_keys = ['action_goal','action_result'],
                          output_keys = ['start_pose_out','action_result'])

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

  def execute(self, userdata):

    # publish info to the console for the user
    rospy.loginfo('StateStart: Executing...')

    userdata.action_result.result.array[0].value = 'False'
    
    location = userdata.action_goal.goal.array[0].value

    if self.locations.get_location(location) != None:
      (goalx, goaly, goalt) = tuple(self.locations.get_location(location))
      userdata.start_pose_out = pose2pose_stamped(goalx, goaly, goalt)
      rospy.loginfo('StateStart: Location set from locations file to target: %f, %f, %f', goalx, goaly, goalt)
    else:
      # ISR Arena (map: isr v08 cr game)
      if location == 'game_start_isr':
        userdata.start_pose_out = pose2pose_stamped(-4.05,1.96,-1.03)

      # ISR Arena (map: ipol 2r)
      if location == 'game_start_ipol':
        userdata.start_pose_out = pose2pose_stamped(-0.55,-10.40,1.64)

      # ISR projection area
      if location == 'classroom_isr':
        userdata.start_pose_out = pose2pose_stamped(3.6, 13.7, 0.32) # in the projection area
        #userdata.start_pose_out = pose2pose_stamped(-7.85, 7.15, -2.59)   # near the docing station

      if location == 'game_over_ipol':
        userdata.start_pose_out = pose2pose_stamped(-4.45,-10.20,0.04)

    if self.preempt_requested():
      self.service_preempt()
      return 'start_preempted'
      
    return 'location_set'

class StateFinish(smach.State):
  """StateFinish sets the result of the action."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['result_set','finish_preempted'],
                          input_keys = ['action_result'],
                          output_keys = ['action_result'])

  def execute(self, userdata):

    # publish info to the console for the user
    rospy.loginfo('StateFinish: Executing...')   

    if self.preempt_requested():
      self.service_preempt()
      return 'finish_preempted'

    userdata.action_result.result.array[0].value = 'True'
      
    return 'result_set'

class StateCheckDistance(smach.State):
  """StateCheckDistance checks if upon reaching to X meters of the target
      Y time has passed. If it has passed, it returns a timeout_expired
      outcome which notifies the rest of the state machine that the robot
      was not able to reach the desired target position."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['timeout_expired','preempted'],
                          input_keys = ['pose_stamped'])

    self.distance_thrs = 1.5
    self.timeout = 15

    self.position = (0,0)
    self.robot_id = rospy.get_namespace()
    self.robot_id = self.robot_id.strip('/')

  def position_cb(self, pose):
    self.position = (pose.position.x, pose.position.y)

  def execute(self, userdata):
    
    self.position_reader = SAMReader('tfPose', self.position_cb, agent_name = self.robot_id)
    inside_radius = False
    target = (userdata.pose_stamped.pose.position.x, userdata.pose_stamped.pose.position.y)
    end_time = rospy.Time.now()

    r = rospy.Rate(2)
    while not rospy.is_shutdown():

      if self.preempt_requested():
        self.position_reader.remove()
        self.service_preempt()
        return 'preempted'

      if not inside_radius:
        if distance.euclidean(self.position, target) < self.distance_thrs:
          rospy.loginfo('Inside %f radius of target', self.distance_thrs)
          inside_radius = True
          end_time = rospy.Time.now() + rospy.Duration(self.timeout)
      else:
        if rospy.Time.now() >= end_time:
          rospy.loginfo('Timeout expired')
          self.position_reader.remove()
          return 'timeout_expired'

class StateSayFailed(MonarchState):
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded','preempted'])
    rospy.loginfo("Init StateSayFailed")

    di = {"activated_cas":"ca04", "tell_user":"mbot_tell_failed_position"}
    kvpa_msg = kvpa.from_dict(di)
    activateCAState = RunCaAction(self, kvpa_msg)
    self.add_action(activateCAState)

#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_goto')
  rospy.loginfo('Starting up')

  #
  #   Concurrence (move + Check time close to target)
  #

  def conc_term_cb(outcome_map):
    return True

  def conc_out_cb(outcome_map):
    if outcome_map['MOVE'] == 'succeeded':
      return 'succeeded'
    if outcome_map['CHECK_DISTANCE'] == 'timeout_expired':
      return 'unsuccessful'
    if outcome_map['MOVE'] == 'aborted':
      return 'aborted'
    if outcome_map['MOVE'] == 'preempted' or outcome_map['CHECK_DISTANCE'] == 'preempted':
      return 'preempted'
    else:
      return 'aborted'

  conc_state = smach.Concurrence(outcomes = ['preempted',
                                              'aborted',
                                              'succeeded',
                                              'unsuccessful'],
                                  default_outcome = 'aborted',
                                  input_keys = ['pose_stamped'],
                                  child_termination_cb = conc_term_cb,
                                  outcome_cb = conc_out_cb)

  with conc_state:
    conc_state.add('MOVE',
                    smach_ros.SimpleActionState('move_base',
                                                  MoveBaseAction,
                                                  goal_slots=['target_pose']),
                    remapping = {'target_pose':'pose_stamped'})

    conc_state.add('CHECK_DISTANCE',
                    StateCheckDistance())


  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['location_reached','location_not_reached','preempted','aborted'],
                          input_keys = ['action_goal'], 
                          output_keys = ['action_result'])

  sm.userdata.pose_stamped = pose2pose_stamped(0.0,0.0,0.0)

  sm.userdata.action_result = GoToResult()
  result_reached = KeyValuePair()
  result_reached.key = 'reached'
  result_reached.value = 'False'
  sm.userdata.action_result.result.array.append(result_reached)
  
  with sm:
    sm.add('START',
            StateStart(),
            transitions = {'location_set':'MOVE_AND_CHECK',
                           'start_preempted':'preempted'},
            remapping = {'start_pose_out':'pose_stamped'})

    sm.add('MOVE_AND_CHECK',
            conc_state,
            transitions = {'succeeded':'FINISH_SUCCESSFUL',
                            'unsuccessful':'FINISH_UNSUCCESSFUL'})

    sm.add('FINISH_SUCCESSFUL',
            StateFinish(),
            transitions = {'result_set':'location_reached',
                           'finish_preempted':'preempted'})

    sm.add('FINISH_UNSUCCESSFUL',
            StateSayFailed(),
            transitions = {'succeeded':'location_not_reached'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      GoToAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['location_reached','location_not_reached'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'],
                                      goal_key = 'action_goal',
                                      result_key = 'action_result')

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()
