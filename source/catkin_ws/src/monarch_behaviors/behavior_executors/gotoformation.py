#! /usr/bin/env python

# Behavior implemented: GoToFormation
# Author: Jose Nuno Pereira
# Description: GoToFormation is a behavior designed to move a set of robots to a speficic location
#              while travelling in a leader-based formation. The final location will be the location
#              of the leader.

# IMPROVE DOC!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

import roslib; roslib.load_manifest('monarch_behaviors')
import math
from functools import partial

import rospy
import smach
import smach_ros

from monarch_behaviors.msg import GoToFormationAction, GoToFormationResult, GoToFormationFeedback
from monarch_msgs.msg import KeyValuePair

from monarch_situational_awareness.srv import CreateSlot, CreateReader, CreateReaderResponse
from monarch_situational_awareness.msg import SlotProperties, ReaderProperties

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from tf.transformations import euler_from_quaternion

def pose2pose_stamped(x, y, t):
  pose_stamped = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return pose_stamped

#
#   States
#

class StateSelectRole(smach.State):
  """Initial state that selects role of robot in formation (leader or follower_i)"""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['leader_selected','follower_selected','selectrole_preempted'],
                          input_keys = ['action_goal','action_result'],
                          output_keys = ['action_result'])

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateSelectRole: Executing...')

    # reseting action_result value
    userdata.action_result.result.array[0].value = 'False'
    
    if self.preempt_requested():
      self.service_preempt()
      return 'selectrole_preempted'

    # get leader id from action_goal
    leader_id = int(userdata.action_goal.goal.array[1].value)

    # if robot is leader
    if robot_id == leader_id:
      return 'leader_selected'

    # robot is follower
    return 'follower_selected'

class StateWaitForTeammates(smach.State):
  """Leader State: Waits in current position until all followers are ready to 
     move in formation. Translates location requested into coordinates"""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['ready_to_move','wait_for_teammates_preempted'],
                          input_keys = ['action_goal'],
                          output_keys = ['wait_pose_out'])
    self.follower_ready = []

  def follower_ready_cb(self, data, follower_id):
    '''Callback function for Follower_i_Ready SAMReader. Sets self.follower_ready[i]
       variable to True if data on SAM slot is True'''
    if data.data is True:
      self.follower_ready[follower_id - 1] = True

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateWaitForTeammates: Executing...')

    r = rospy.Rate(2)
    
    # translate location into Pose
    location = userdata.action_goal.goal.array[0].value

    # Webots IPOL environment (world file: envIPOL_half_corridor_3rooms.wbt (23/07/2014))
    if location == 'room1':
      userdata.wait_pose_out = pose2pose_stamped(12.60,-3.50,-1.57)

    if location == 'room2':
      userdata.wait_pose_out = pose2pose_stamped(6.35,-3.35,0.00)

    # EPFL Robotics Arena, Corridor and Offices (map: completeMapRetouched.yaml)
    if location == 'corner1':
      userdata.wait_pose_out = pose2pose_stamped(6.40,-5.65,-2.26)

    if location == 'corner2':
      userdata.wait_pose_out = pose2pose_stamped(6.50,-3.50,2.14)

    if location == 'corner3':
      userdata.wait_pose_out = pose2pose_stamped(9.80,-4.15,0.92)

    if location == 'jose':
      userdata.wait_pose_out = pose2pose_stamped(-7.05,1.95,1.53)

    if location == 'lorenzo':
      userdata.wait_pose_out = pose2pose_stamped(0.10,2.15,1.57)

    if location == 'kitchen':
      userdata.wait_pose_out = pose2pose_stamped(4.95,1.85,1.71)
    
    # create signal variables for followers
    # when self.follower_ready[i] = True, follower is close to leader;
    # when all followers ready, leader can start moving to location
    number_followers = int(userdata.action_goal.goal.array[2].value)
    self.follower_ready = [False for i in range(number_followers)]

    # create signal slots for followers
    rospy.wait_for_service('/create_slot')
    create_slot = rospy.ServiceProxy('/create_slot', CreateSlot)

    for follower in range(1, number_followers + 1):
      sp = SlotProperties(name = 'Follower_' + str(follower) + '_Ready',
                          type_str = 'std_msgs/Bool',
                          is_shared = True,
                          is_latched = True)
      create_slot(sp)

    # create readers for slots just created
    rospy.wait_for_service('/create_reader')
    create_reader = rospy.ServiceProxy('/create_reader', CreateReader)
    sam_readers_list = []
    for follower in range(1, number_followers + 1):
      reader_properties = ReaderProperties(slot_name = 'Follower_' + str(follower) + '_Ready')
      reader_response = CreateReaderResponse()
      while reader_response.success is False:
        reader_response = create_reader(reader_properties)
        rospy.loginfo('Attempting to create reader for slot Follower_%d_Ready',follower)
        r.sleep()
      sam_readers_list.append(rospy.Subscriber(reader_response.topic_name,
                                               Bool,
                                               partial(self.follower_ready_cb,
                                                       follower_id = follower)))

    # loop until all followers ready or preempted
    preempted = False
    while not rospy.is_shutdown() and \
          all(self.follower_ready) is False and \
          preempted is False:

      if self.preempt_requested():
        self.service_preempt()
        preempted = True

      # sleep a bit
      r.sleep()

    # unregistering from follower_i_ready sam slot
    for reader in range(number_followers):
      sam_readers_list[reader].unregister()

    if not preempted:
      return 'ready_to_move'
    else:
      return 'wait_for_teammates_preempted'

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

class StateFindLeader(smach.State):
  """StateFindLeader sets the move base goal to the position of the Leader robot
     plus the bias vector for the follower."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['leader_position_set','find_leader_preempted'],
                         input_keys = ['action_goal'],
                         output_keys = ['find_leader_pose_out'])

  def leader_pose_cb(self, data):
    '''Callback funcion for [mbotLEADER] Robot_Pose. Sets find_leader_pose_out.'''
    self.leader_pose = PoseStamped(header = Header(frame_id = "/map"),
                                            pose = data.pose.pose)
    self.leader_found = True

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateFindLeader: Executing...')

    r = rospy.Rate(2)

    # get leader id from action_goal
    leader_id = int(userdata.action_goal.goal.array[1].value)
    self.leader_found = False

    # Create SAM Reader
    rospy.wait_for_service('/create_reader')
    reader_service = rospy.ServiceProxy('/create_reader', CreateReader)
    reader_properties = ReaderProperties(slot_name = '[mbot0' + str(leader_id) + '] Robot_Pose')
    reader_response = CreateReaderResponse()

    while reader_response.success is False:
      reader_response = reader_service(reader_properties)
      r.sleep()

    sam_reader = rospy.Subscriber(reader_response.topic_name,
                                  PoseWithCovarianceStamped,
                                  self.leader_pose_cb)

    while not rospy.is_shutdown() and self.leader_found is False:

      if self.preempt_requested():
        sam_reader.unregister() # unregistering from people localization tracker topic
        self.service_preempt()
        return 'find_leader_preempted'

      # sleep a bit
      r.sleep()

    # find what follower i this robot is
    follower_id = -1
    number_followers = int(userdata.action_goal.goal.array[2].value)
    for i in range(number_followers):
      if robot_id == int(userdata.action_goal.goal.array[3 + i].value):
        follower_id = i
        break

    # find bias vector for follower i
    start_index = number_followers + 3
    [bias_x, bias_y] = [float(userdata.action_goal.goal.array[start_index + 2*follower_id].value),
                        float(userdata.action_goal.goal.array[start_index + 2*follower_id +1].value)]

    print [bias_x, bias_y]

    # pose of follower in formation wrt to the leader's heading
    follower_goal_pose = self.leader_pose

    print self.leader_pose

    quat = [follower_goal_pose.pose.orientation.x,
            follower_goal_pose.pose.orientation.y,
            follower_goal_pose.pose.orientation.z,
            follower_goal_pose.pose.orientation.w]

    print euler_from_quaternion(quat)

    # add bias vector wrt to leader's heading
    # wrong
    # wrong
    # wrong
    # wrong
    # wrong
    # wrong
    # wrong
    follower_goal_pose.pose.position.x = follower_goal_pose.pose.position.x + \
                                         bias_x*math.cos(euler_from_quaternion(quat)[2])
    follower_goal_pose.pose.position.y = follower_goal_pose.pose.position.y + \
                                         bias_y*math.sin(euler_from_quaternion(quat)[2])

    print follower_goal_pose


    userdata.find_leader_pose_out = follower_goal_pose
    sam_reader.unregister() # unregistering from people localization tracker topic
      
    return 'leader_position_set'

#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_gotoformation')
  rospy.loginfo('Starting up')

  node_name = rospy.get_name()
  robot_id = int(node_name[node_name.rfind('mbot')+4:node_name.rfind('mbot')+6])

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['location_reached','preempted','aborted'],
                          input_keys = ['action_goal'], 
                          output_keys = ['action_result'])

  sm.userdata.pose_stamped = pose2pose_stamped(0.0,0.0,0.0)

  sm.userdata.action_result = GoToFormationResult()
  result_reached = KeyValuePair()
  result_reached.key = 'reached'
  result_reached.value = 'False'
  sm.userdata.action_result.result.array.append(result_reached)

  sm.userdata.action_feedback = GoToFormationFeedback()
  feedback_formation_intact = KeyValuePair()
  feedback_formation_intact.key = 'formtion_intact'
  feedback_formation_intact.value = 'False'
  sm.userdata.action_feedback.feedback.array.append(feedback_formation_intact)
  
  with sm:
    #
    # Common
    #
    sm.add('SELECT_ROLE',
            StateSelectRole(),
            transitions = {'leader_selected':'WAIT_FOR_TEAMMATES',
                           'follower_selected':'FIND_LEADER',
                           'selectrole_preempted':'preempted'})
    #
    # Leader Side
    #
    sm.add('WAIT_FOR_TEAMMATES',
            StateWaitForTeammates(),
            transitions = {'ready_to_move':'MOVE_TO_LOCATION',
                           'wait_for_teammates_preempted':'preempted'},
            remapping = {'wait_pose_out':'pose_stamped'})

    sm.add('MOVE_TO_LOCATION',
            smach_ros.SimpleActionState('move_base',
                                         MoveBaseAction,
                                         goal_slots = ['target_pose']),
            transitions = {'succeeded':'FINISH'},
            remapping = {'target_pose':'pose_stamped'})

    #
    # Follower Side
    #
    sm.add('FIND_LEADER',
            StateFindLeader(),
            transitions = {'leader_position_set':'MOVE_TO_LEADER',
                           'find_leader_preempted':'preempted'},
            remapping = {'find_leader_pose_out':'pose_stamped'})

    sm.add('MOVE_TO_LEADER',
            smach_ros.SimpleActionState('move_base',
                                         MoveBaseAction,
                                         goal_slots = ['target_pose']),
            transitions = {'succeeded':'FINISH'},
            remapping = {'target_pose':'pose_stamped'})    

    #
    # Common
    #
    sm.add('FINISH',
            StateFinish(),
            transitions = {'result_set':'location_reached',
                           'finish_preempted':'preempted'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      GoToFormationAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['location_reached'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'],
                                      goal_key = 'action_goal', 
                                      feedback_key = 'action_feedback',
                                      result_key = 'action_result')

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()