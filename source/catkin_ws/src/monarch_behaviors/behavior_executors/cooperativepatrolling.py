#! /usr/bin/env python

# Behavior implemented: Formation
# Author: Jose Pereira, Alicja Wasik
# Description: Revised version of cooperative patrolling with concurrent 
# information exchange between the leader and the followers.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import sys
import tf
import smach
import smach_ros
from functools import partial

from monarch_behaviors.msg import CooperativePatrollingAction

from monarch_situational_awareness.srv import *
from monarch_situational_awareness.msg import *
from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter

import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from graph_based_formations.msg import *

DISTANCE_FAR_FROM_LEADER_THRESHOLD = 2.0    
DISTANCE_CLOSE_TO_LEADER_THRESHOLD = 1.0

PUBLISHING_FREQUENCY = 1.0   

create_sam_prelude_str = 'create'
remove_sam_prelude_str = 'remove'

def pose2alib(x, y, t=0):
  goal = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return MoveBaseGoal(target_pose=goal)

def pose2pose_stamped(x, y, t):
  pose_stamped = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return pose_stamped

def quaternionToYaw(q1, q2, q3, q4):
    q = (q1,q2,q3,q4)
    euler = tf.transformations.euler_from_quaternion(q)
    return euler[2]

#
#   States
#

class StateSetInitialState(smach.State):
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

    # advance current state
    userdata.current_state = (userdata.current_state + 1) if userdata.current_state + 1 < len(waypoints) else 0

    # set initial state to next waypoint
    sm_patrolling.set_initial_state(['WAYPOINT_%s'%(userdata.current_state)])
    rospy.sleep(3)                                                          # AW stay at this point for few seconds
    return 'succeeded'


class StateSelectRole(smach.State):
  """Initial state that selects role of robot in formation (leader or follower_i)"""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['leader_selected','follower_selected','selectrole_preempted'],
                          input_keys = ['action_goal',
                                        'bias_length',
                                        'formation_id',
                                        'max_cc',
                                        'leader_id',
                                        'follower_1',
                                        'follower_2',
                                        'follower_3',
                                        'number_of_robots',
                                        'switch_enabled'],
                          output_keys = ['action_goal',
                                         'bias_length',
                                         'formation_id',
                                         'max_cc',
                                         'leader_id',
                                         'follower_1',
                                         'follower_2',
                                         'follower_3',
                                         'number_of_robots',
                                         'switch_enabled'])

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateSelectRole: Executing...')
    
    if self.preempt_requested():
      self.service_preempt()
      return 'selectrole_preempted'

    # get leader id from action_goal
    leader_id = int(userdata.action_goal.goal.array[3].value)

    userdata.bias_length = float(userdata.action_goal.goal.array[0].value) 
    userdata.formation_id = int(userdata.action_goal.goal.array[1].value)
    userdata.max_cc = int(userdata.action_goal.goal.array[2].value)
    userdata.leader_id = int(userdata.action_goal.goal.array[3].value)
    userdata.follower_1 = int(userdata.action_goal.goal.array[4].value)
    userdata.follower_2 = int(userdata.action_goal.goal.array[5].value)
    userdata.follower_3 = int(userdata.action_goal.goal.array[6].value)
    userdata.number_of_robots = int(userdata.action_goal.goal.array[7].value)
    userdata.switch_enabled = int(userdata.action_goal.goal.array[8].value)

    # if robot is leader
    if robot_id == leader_id:
      return 'leader_selected'

    # robot is follower
    return 'follower_selected'



class StateWaitForTeammates(smach.State):
  """Leader State: Waits in current position until all followers are ready to 
     move in formation."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['ready_to_move','wait_for_teammates_preempted'],
                         input_keys = ['action_goal'])
    self.follower_ready = []

    self.sam_readers_list = []
    # self.sam_readers_list = [3*[None]]

  def follower_ready_cb(self, data, follower_id):
    '''Callback function for Follower_i_Ready SAMReader. Sets self.follower_ready[i]
       variable to True if data on SAM slot is True'''
    if data.data is True:
      self.follower_ready[follower_id - 1] = True
      #rospy.loginfo('DEBUG::  StateWaitForTeammates (T): Follower '+ str(follower_id) + ' sends TRUE')
    # else:
    #   rospy.loginfo('DEBUG LEADER::  StateWaitForTeammates (wait for true): Follower '+ str(follower_id) + ' publishing FALSE')

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateWaitForTeammates: Executing...')
    r = rospy.Rate(0.5)
    
    # create signal variables for followers
    # when self.follower_ready[i] = True, follower is close to leader;
    # when all followers ready, leader can start moving to location
    number_followers = int(userdata.action_goal.goal.array[7].value)-1
    self.follower_ready = [False for i in range(number_followers)]



    # create readers for existing slots Follower_1_Ready, Follower_2_Ready, Follower_3_Ready 
    rospy.sleep(0.5)
    for f in range(1, number_followers + 1):
        self.sam_readers_list.append(SAMReader("Follower_"+str(f)+"_Ready", 
                            partial(self.follower_ready_cb, follower_id = f)))

    # for f in range(1, number_followers + 1):
    #     self.sam_readers_list[f - 1] = SAMReader("Follower_"+str(f)+"_Ready", 
    #                         partial(self.follower_ready_cb, follower_id = f))

    #rospy.loginfo('SAM readers created for all Follower Ready slots, waiting for Follower Ready signals')

    # loop until all followers ready or preempted
    preempted = False
    while not rospy.is_shutdown() and all(self.follower_ready) is False and  preempted is False: # I think this should be any!!

      if self.preempt_requested():
        self.service_preempt()
        preempted = True

      rospy.loginfo('DEBUG::  StateWaitForTeammates (T): self.follower_ready'+ str(self.follower_ready) + ' in loop')
      r.sleep()

    rospy.sleep(2.5)
    rospy.loginfo('DEBUG::  StateWaitForTeammates (T): self.follower_ready'+ str(self.follower_ready) + ' out of loop')

    if not preempted:
      return 'ready_to_move'
    else:
      return 'wait_for_teammates_preempted'



class StateCheckTeammates(smach.State):
  """Leader State: checks if all followers are able to 
     move in formation."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['stop_patrolling','check_teammates_preempted'],
                         input_keys = ['action_goal'])
    self.follower_not_ready = []
    self.sam_readers_list = []

  def follower_not_ready_cb(self, data, follower_id):
    '''Callback function for Follower_i_Ready SAMReader. Sets self.follower_not_ready[i]
       variable to True if data on SAM slot is False'''
    if data.data is False:
      self.follower_not_ready[follower_id - 1] = True
      #rospy.loginfo('DEBUG::  StateCheckTeammates: (F) Follower '+ str(follower_id) + ' sends FALSE')
    # else:
    #   rospy.loginfo('DEBUG LEADER::  StateCheckTeammates: (wait for false) Follower '+ str(follower_id) + ' publishing TRUE')

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateCheckTeammates: Executing...')

    r = rospy.Rate(0.5)
    
    # create signal variables for followers
    # when self.follower_not_ready[i] = True, follower is close to leader;
    # when all followers ready, leader can start moving to location
    number_followers = int(userdata.action_goal.goal.array[7].value)-1
    self.follower_not_ready = [False for i in range(number_followers)]

    for f in range(1, number_followers + 1):
        self.sam_readers_list.append(SAMReader("Follower_"+str(f)+"_Ready", 
                            partial(self.follower_not_ready_cb, follower_id = f)))

    rospy.loginfo('SAM readers created for all Follower Ready slots, waiting for Follower Not Ready signals')

    # loop until all followers ready or preempted
    preempted = False
    while not rospy.is_shutdown() and any(self.follower_not_ready) is False and preempted is False: # Any is true!! That means any is not ready
      if self.preempt_requested():
        self.service_preempt()
        preempted = True

      rospy.loginfo('DEBUG::  StateCheckTeammates (T): self.follower_not_ready'+ str(self.follower_not_ready) + ' in loop')
      r.sleep()

    rospy.sleep(0.5)
    rospy.loginfo('DEBUG::  StateCheckTeammates (T): self.follower_not_ready'+ str(self.follower_not_ready) + ' out of loop')

    if not preempted:
      return 'stop_patrolling'
    else:
      return 'check_teammates_preempted'




class StateGoToLeader(smach.State):
  """StateGoToLeader makes the robot move_base to leader."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['close_to_leader','preempted', 'aborted'],
                         input_keys = ['action_goal'])
    self.sam_reader = None
    self.leader_found = False
    self.own_pose_set = False

  def leader_pose_cb(self, data):
    '''Callback funcion for [mbotLEADER] tfPose. Sets find_leader_pose_out.'''
    self.leader_pose = PoseStamped(header = Header(frame_id = "/map"),
                                   pose = data)
    if not self.leader_found:
      rospy.loginfo('DEBUG StateGoToLeader: Leader found...')
    self.leader_found = True

  def own_pose_cb(self, data):
    '''Callback funcion for own pose.'''
    self.own_pose = data
    if not self.own_pose_set:
      rospy.loginfo('DEBUG StateGoToLeader: Me found...')
    self.own_pose_set = True
    

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateGoToLeader: Executing...')
   
    leader_id = int(userdata.action_goal.goal.array[3].value)
    if leader_id < 10:
      leader_id = '0' + str(leader_id)
    else:
      leader_id = str(leader_id)
    self.leader_found = False

    self.sam_reader = SAMReader("tfPose", self.leader_pose_cb, agent_name="mbot" + leader_id)
    if robot_id < 10:
      robot_id_sam = '0' + str(robot_id)
    else:
      robot_id_sam = str(robot_id)

    r1 = SAMReader("tfPose", self.own_pose_cb, agent_name="mbot" + str(robot_id_sam))

    
    while self.leader_found is False or self.own_pose_set is False:
      rospy.sleep(0.5)

    # Move to a slot behind the leader 
    bx = 0
    by = -1

    al = quaternionToYaw(self.leader_pose.pose.orientation.x,self.leader_pose.pose.orientation.y, \
      self.leader_pose.pose.orientation.z,self.leader_pose.pose.orientation.w)-math.pi/2
    xl = self.leader_pose.pose.position.x
    yl = self.leader_pose.pose.position.y

    xd = xl + math.cos(al)*bx - math.sin(al)*by
    yd = yl + math.sin(al)*bx + math.cos(al)*by

    rospy.loginfo('StateGoToLeader: Leaders position '+ str(self.leader_pose.pose.position.x) +' ' + str(self.leader_pose.pose.position.y))
    rospy.loginfo('StateGoToLeader: Follower moving to '+ str(xd) +' ' + str(yd))


    if self.leader_found and self.own_pose_set:
        distance = math.sqrt(math.pow(self.leader_pose.pose.position.x - self.own_pose.position.x,2) +\
                             math.pow(self.leader_pose.pose.position.y - self.own_pose.position.y,2))
        rospy.loginfo('StateGoToLeader: Distance to desired pose:: ' + str(distance))

    if distance > DISTANCE_CLOSE_TO_LEADER_THRESHOLD:
      client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
      client.wait_for_server()
      gg = pose2alib(xd,yd)
      client.send_goal(gg)
      client.wait_for_result(rospy.Duration(3.0))

      g_result = client.get_state()
      # client.wait_for_result()
      # rospy.loginfo('StateGoToLeader: goal result '+ str(g_result))


      rospy.wait_for_service(remove_sam_prelude_str + '_reader')
      remove_reader_service = rospy.ServiceProxy(remove_sam_prelude_str + '_reader', RemoveReader)
      remove_reader_response1 = RemoveReaderResponse()


      if g_result==GoalStatus.ABORTED:
        rospy.loginfo('StateGoToLeader: Returns aborted...')
        # while remove_reader_response1.success is False:
        #   remove_reader_response1 = remove_reader_service(slot_name = '[mbot' + str(leader_id) + '] tfPose', reader_name = rospy.get_name())
          # remove_reader_response2 = remove_reader_service(slot_name = '[mbot' + str(robot_id_sam) + '] tfPose', reader_name = rospy.get_name())
          # rospy.sleep(0.1)
        return 'aborted'
      elif g_result==GoalStatus.PREEMPTED:
        rospy.loginfo('StateGoToLeader: Returns preempted...')
        # while remove_reader_response1.success is False:
        #   remove_reader_response1 = remove_reader_service(slot_name = '[mbot' + str(leader_id) + '] tfPose', reader_name = rospy.get_name())
        #   remove_reader_response2 = remove_reader_service(slot_name = '[mbot' + str(robot_id_sam) + '] tfPose', reader_name = rospy.get_name())
        #   rospy.sleep(0.1)
        return 'preempted'
      elif g_result==GoalStatus.SUCCEEDED or g_result==GoalStatus.ACTIVE:
        rospy.loginfo('StateGoToLeader: Returns succeeded...')
        # while remove_reader_response1.success is False:
        #   remove_reader_response1 = remove_reader_service(slot_name = '[mbot' + str(leader_id) + '] tfPose', reader_name = rospy.get_name())
        #   remove_reader_response2 = remove_reader_service(slot_name = '[mbot' + str(robot_id_sam) + '] tfPose', reader_name = rospy.get_name())
        #   rospy.sleep(0.1)
        return 'close_to_leader'
    else:   # if distance > DISTANCE_CLOSE_TO_LEADER_THRESHOLD:
      return 'close_to_leader'



class StateCheckClosetoLeader(smach.State):
  """StateCheckClosetoLeader checks if robot is close to leader robot."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['close_to_leader','close_to_leader_preempted'],
                         input_keys = ['action_goal'])
    self.leader_pose_set = False
    self.own_pose_set = False


  def leader_pose_cb(self, data):
    '''Callback funcion for [mbotLEADER] tfPose. Sets find_leader_pose_out.'''
    self.leader_pose = data
    if not self.leader_pose_set:
      rospy.loginfo('DEBUG StateCheckClosetoLeader: Leader found...')
    self.leader_pose_set = True
    

  def own_pose_cb(self, data):
    '''Callback funcion for own pose.'''
    self.own_pose = data
    if not self.own_pose_set:
      rospy.loginfo('DEBUG StateCheckClosetoLeader: Me found...')
    self.own_pose_set = True

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateCheckClosetoLeader: Executing...')

    r = rospy.Rate(2)

    # get leader id from action_goal
    leader_id = int(userdata.action_goal.goal.array[3].value)
    if leader_id < 10:
      leader_id = '0' + str(leader_id)
    else:
      leader_id = str(leader_id)

    if self.preempt_requested():
      self.service_preempt()
      return 'close_to_leader_preempted'

    if robot_id < 10:
      robot_id_sam = '0' + str(robot_id)
    else:
      robot_id_sam = str(robot_id)

    sam_reader = SAMReader("tfPose", self.leader_pose_cb, agent_name="mbot" + leader_id)
    r1 = SAMReader("tfPose", self.own_pose_cb, agent_name="mbot" + str(robot_id_sam))

    rospy.loginfo('StateCheckClosetoLeader: Readers created...')


    self.leader_pose_set = False
    self.own_pose_set = False

    while not rospy.is_shutdown():

      if self.preempt_requested():
        rospy.wait_for_service(remove_sam_prelude_str + '_reader')
        remove_reader_service = rospy.ServiceProxy(remove_sam_prelude_str + '_reader', RemoveReader)
        remove_reader_response1 = RemoveReaderResponse()
        remove_reader_response2 = RemoveReaderResponse()
        while remove_reader_response1.success is False:
          remove_reader_response1 = remove_reader_service(slot_name = '[mbot' + str(robot_id_sam) + '] tfPose', reader_name = rospy.get_name())
          remove_reader_response2 = remove_reader_service(slot_name = '[mbot' + str(leader_id) + '] tfPose', reader_name = rospy.get_name())
          rospy.sleep(0.1)
        self.service_preempt()
        return 'close_to_leader_preempted'

      if self.leader_pose_set and self.own_pose_set:
        distance = math.sqrt(math.pow(self.leader_pose.position.x - self.own_pose.position.x,2) +\
                             math.pow(self.leader_pose.position.y - self.own_pose.position.y,2))

        if distance < DISTANCE_CLOSE_TO_LEADER_THRESHOLD:
          rospy.loginfo('DEBUG StateCheckClosetoLeader: CLOSER THAN ' + str(DISTANCE_CLOSE_TO_LEADER_THRESHOLD) + ' TO LEADER...')
          return 'close_to_leader'

      r.sleep()




class StateWriteNotReady(smach.State):
  """writes that follower_i is not ready to move in formation in appropriate slot"""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['not_ready_written','write_not_ready_preempted'],
                          input_keys = ['action_goal'])
    self.ready_pub = None
    # self.writer_registered = False

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateWriteNotReady: Executing...')
    
    r = rospy.Rate(PUBLISHING_FREQUENCY)

    # if self.preempt_requested():
    #   self.service_preempt()
    #   return 'write_not_ready_preempted'

    number_followers = int(userdata.action_goal.goal.array[7].value)-1

    writer_lock = False
    for f in range(1, number_followers + 1):
        if robot_id == int(userdata.action_goal.goal.array[3 + f].value):
        # if robot_id == int(userdata.action_goal.goal.array[3 + f].value) and not self.writer_registered:
            self.my_slot = f
            self.ready_pub = SAMWriter("Follower_"+str(f)+"_Ready")
            writer_lock = True
            rospy.loginfo('StateWriteNotReady: Writing to a slot Follower_'+str(f)+'_Ready')
    # self.writer_registered = True

    while  not self.preempt_requested() and writer_lock:
        # rospy.loginfo('StateWriteNotReady: Streaming to a slot')
        self.ready_pub.publish(False)
        rospy.loginfo('DEBUG StateWriteNotReady: FALSE...')
        r.sleep()

    if self.preempt_requested():

      rospy.loginfo('DEBUG StateWriteNotReady: Preempt requested...')

      # rospy.wait_for_service('/mcentral/remove_writer')
      rospy.wait_for_service(remove_sam_prelude_str + '_writer')
      rospy.loginfo('DEBUG StateWriteNotReady: Service responded...')

      # writer_service = rospy.ServiceProxy('/mcentral/remove_writer', RemoveWriter)
      writer_service = rospy.ServiceProxy(remove_sam_prelude_str + '_writer', RemoveWriter)
      slot_name = "Follower_"+str(self.my_slot)+"_Ready" 
      writer_name = rospy.get_name()
      # reader_properties = ReaderProperties(slot_name = '[mbot' + leader_id + '] tfPose')
      writer_response = CreateWriterResponse()
      rospy.loginfo('DEBUG StateWriteNotReady: RemoveWriter begins...')


      while writer_response.success is False:
        writer_response = writer_service(slot_name, writer_name)
        rospy.loginfo('DEBUG StateWriteNotReady: RemoveWriter waiting for response...')

        r.sleep()

      self.service_preempt()

      # return 'write_not_ready_preempted'
      return 'not_ready_written'



    return 'not_ready_written'







class StateCheckFarFromLeader(smach.State):
  """StateCheckFarFromLeader checks if robot is far from the leader robot."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['far_from_leader','far_from_leader_preempted'],
                         input_keys = ['action_goal'])
    self.leader_pose_set = False
    self.own_pose_set = False

  def leader_pose_cb(self, data):
    '''Callback funcion for [mbotLEADER] tfPose. Sets find_leader_pose_out.'''
    self.leader_pose = data
    if not self.leader_pose_set:
      rospy.loginfo('DEBUG StateCheckFarFromLeader: Leader found...')
    self.leader_pose_set = True
    

  def own_pose_cb(self, data):
    '''Callback funcion for own pose.'''
    self.own_pose = data
    if not self.own_pose_set:
      rospy.loginfo('DEBUG StateCheckFarFromLeader: Me found...')
    self.own_pose_set = True
    

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateCheckFarFromLeader: Executing...')

    r = rospy.Rate(2)

    # get leader id from action_goal
    leader_id = int(userdata.action_goal.goal.array[3].value)
    if leader_id < 10:
      leader_id = '0' + str(leader_id)
    else:
      leader_id = str(leader_id)
    sam_reader = SAMReader("tfPose", self.leader_pose_cb, agent_name="mbot" + leader_id)

    if robot_id < 10:
      robot_id_sam = '0' + str(robot_id)
    else:
      robot_id_sam = str(robot_id)

    r1 = SAMReader("tfPose", self.own_pose_cb, agent_name="mbot" + str(robot_id_sam))

    self.leader_pose_set = False
    self.own_pose_set = False

    rospy.loginfo('StateCheckFarFromLeader: Readers created...')

    while not rospy.is_shutdown():

      if self.preempt_requested():

        rospy.wait_for_service(remove_sam_prelude_str + '_reader')
        remove_reader_service = rospy.ServiceProxy(remove_sam_prelude_str + '_reader', RemoveReader)
        remove_reader_response1 = RemoveReaderResponse()
        remove_reader_response2 = RemoveReaderResponse()
        while remove_reader_response1.success is False:
            remove_reader_response1 = remove_reader_service(slot_name = '[mbot' + str(robot_id_sam) + '] tfPose', reader_name = rospy.get_name())
            remove_reader_response2 = remove_reader_service(slot_name = '[mbot' + str(leader_id) + '] tfPose', reader_name = rospy.get_name())
            rospy.sleep(0.1)

        self.service_preempt()
        rospy.sleep(0.5)
        return 'far_from_leader_preempted'

      if self.leader_pose_set and self.own_pose_set:
        distance = math.sqrt(math.pow(self.leader_pose.position.x - self.own_pose.position.x,2) +\
                             math.pow(self.leader_pose.position.y - self.own_pose.position.y,2))
        if distance > DISTANCE_FAR_FROM_LEADER_THRESHOLD:
          rospy.loginfo('DEBUG StateCheckFarFromLeader: FURTHER THAN ' + str(DISTANCE_FAR_FROM_LEADER_THRESHOLD) + ' FROM LEADER...')
          return 'far_from_leader'

      r.sleep()



class StateWriteReady(smach.State):
  """writes that follower_i is ready to move in formation in appropriate slot"""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['ready_written','write_ready_preempted'],
                          input_keys = ['action_goal'])
    self.ready_pub = None
    # self.writer_registered = False

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateWriteReady: Executing...')
    
    r = rospy.Rate(PUBLISHING_FREQUENCY)

    if self.preempt_requested():
      self.service_preempt()
      return 'write_ready_preempted'

    number_followers = int(userdata.action_goal.goal.array[7].value)-1

    for f in range(1, number_followers + 1):
        if robot_id == int(userdata.action_goal.goal.array[3 + f].value):
        # if robot_id == int(userdata.action_goal.goal.array[3 + f].value) and not self.writer_registered:
            self.ready_pub = SAMWriter("Follower_"+str(f)+"_Ready")
            self.my_slot = f

    # self.writer_registered = True

    while not self.preempt_requested():
        self.ready_pub.publish(True)
        rospy.loginfo('DEBUG StateWriteReady: TRUE...')

        r.sleep()


    if self.preempt_requested():

      rospy.loginfo('DEBUG StateWriteReady: Preempt requested...')

      rospy.wait_for_service(remove_sam_prelude_str + '_writer')
      writer_service = rospy.ServiceProxy(remove_sam_prelude_str + '_writer', RemoveWriter)
      slot_name = "Follower_"+str(self.my_slot)+"_Ready" 
      writer_name = rospy.get_name()
      # reader_properties = ReaderProperties(slot_name = '[mbot' + leader_id + '] tfPose')
      writer_response = CreateWriterResponse()
      rospy.loginfo('DEBUG StateWriteReady: RemoveWriter begins...')


      while writer_response.success is False:
        writer_response = writer_service(slot_name, writer_name)
        rospy.loginfo('DEBUG StateWriteReady: RemoveWriter waiting for response...')

        r.sleep()

      self.service_preempt()
      return 'ready_written'
      # return 'write_ready_preempted'


    return 'ready_written'



################################################################################################
################################################################################################
########################################   Main     ############################################
################################################################################################
################################################################################################


if __name__ == '__main__':
  rospy.init_node('behavior_cooperativepatrolling')
  rospy.loginfo('Starting up')

  node_name = rospy.get_name()
  if node_name.rfind('mbot') == -1:      #Runnign on mcentral--> SHUTDOWN
    rospy.signal_shutdown('Running on /mcentral namespace')
    sys.exit(0)

  robot_id = int(node_name[node_name.rfind('mbot')+4:node_name.rfind('mbot')+6])

  #create waypoints for patrolling

  # IW 8th floor 2 "rooms"
  #waypoints = [(-2.50,-3.35,0.56),
                #(-6.50,-5.20,-2.70),
                #(-11.3, -14.3, -2.1),
                #(-8.90,-17.10,-1.18)]

  # IW 8th floor 1 "room"
  #waypoints = [(-9.5,-11.6,-2.57),
                #(-11.3, -14.3, -2.1),
                #(-8.90,-17.10,-1.18)]

  # EPFL Robotics Arena (Alicja)
  # waypoints = [(7.2,12.8,1.78),
  #               (7.0,10.8,-0.1),
  #               (6.05,8.7,-1.74)]

  # EPFL Robotics Arena and Corridor
  #waypoints = [(6.15,7.8,-1.7),
   #             (10.15,10.7,0.0),
    #            (10.35,11.2,-3.0),
     #           (9.7,7.43, 0.0),
      #          #(10.15,10.7,-3.0),
       #         (6.05,8.7,-1.74)]

  # EPFL Robotics Arena Sqare Path => POSITIONING EXPERIMENT
  #waypoints = [(6.3,12.0,1.42),
                #(5.3,12.0,3.05),
                #(5.3,8.0,-1.72),
                #(6.3,8.0,0.0),
                #(6.3,12.0,1.42),
                #(5.3,12.0,3.05),
                #(5.3,8.0,-1.72),
                #(6.3,8.0,0.0)]
  # waypoints = [(5.0,7.6,-1.7),
  #               (6.7,7.6,0.0),
  #               (6.7,12.5,1.42),
  #               (5.0,12.5,-3.0)]

  # Webots IPOL environment (world file: envIPOL_half_corridor_3rooms.wbt (23/07/2014))
  # waypoints = [(6.35,-3.35,0.00),
  #              (-6.80,0.65,3.08),
  #              (-4.40,-3.10,-1.63),
  #              (-13.10,-2.00,2.86),
  #              (-20.20,-2.75,1.53),
  #              (-19.75,1.95,-0.04),
  #              (-6.40,2.55,-0.98),
  #              (-7.05,-2.50,-1.57)]

  # Webots IPOL environment (world file: IPOL_2 (15/03/2016), uses ipolMapNoObstacles map) 
  # waypoints = [(-18.9, 0.35, 0.0),
  #              (-16.6,-2.6,-1.45),
  #              (-5.35,-3.1,1.52),
  #              (-5.6,0.9,-2.25)]

  # IW 8th floor isr8-v07cr_new_gamearea.yaml
  # waypoints = [(4.65,11.85,-1.99),
  #              (0.35,5.15,-1.32),
  #              (2.15,-1.60,-1.86),
  #              (8.00,-3.90,-3.14)]
               #(12.40,11.30,-0.30),
               #(19.05,8.50,-1.57),
               #(12.30,0.70,-0.00)]

  # Simple waypoints at IST (Alicja)
  # waypoints = [(-6.3, 8.3, 0.48),
  #              (-1.25, 10.8, 0.48),
  #              (2.75,12.85,0.3)]

  # Waypoints at IST with the living room 
  # waypoints = [(2.75,12.85,0.3),
  #              (-6.3, 8.3, 0.48),
  #              (-3.1, 2.05, -0.61),
  #              (-1.7, -0.4, -1.14)]


  # Waypoints at IST with corridor
  # waypoints = [(-6.3, 8.3, 0.48),
  #              (2.75,12.85,0.3),
  #              (11.75, 9.2, -1.09),
  #              (7.15, -2.6, -2.55)]  

  ## Waypoints at IST with corridor and living room (revisited), A B C B A D
  # waypoints = [(-3.4, 9.3, -1.05),
  #              (4.45, 13.2, -1.05),
  #              (6.05, -0.8, -1.05),
  #              (4.45, 13.2, -1.05),
  #              (-3.4, 9.3, -1.05),
  #              (-1.7, 0.9, 0.58)]  

  ## Waypoints at IST for three robots only main room
  #waypoints = [(-1.45, 10.25, 0.5),
   #            (4.6, 13.05, 0.53),
    #           (-0.45, 10.55, -2.59)]  

  ## Waypoints at IST for three robots main room + lift
  waypoints = [(-1.45, 10.25, 0.5),
              (4.6, 13.05, 0.53),
              (-0.45, 10.55, -2.59),
              (5.45, -0.1, 2.24)]  

  # Waypoints at IPOL for cooperative patrolling experiments
  # waypoints = [(-2.25,-8.50,1.52),(1.55,1.90,1.66),(0.35,16.05,1.67)]


  opposite_waypoints = [(l[0],l[1],(l[2]+math.pi)%(2*math.pi)) for l in waypoints]
  opposite_waypoints.reverse()
  waypoints.extend(opposite_waypoints)



  ##################################################################################################
  ################################   Patrolling State   ############################################
  ##################################################################################################
      
  sm_patrolling = smach.StateMachine(outcomes = ['preempted','aborted'])
  sm_patrolling.userdata.current_state = 0

  with sm_patrolling:
    for (i,w) in enumerate(waypoints):
      next_waypoint = 'WAYPOINT_%s'%(i+1) if i+1<len(waypoints) else 'WAYPOINT_0'

      sm_patrolling.add('WAYPOINT_%s'%(i),
              smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=pose2alib(*w)),
              transitions = {'succeeded': 'SETINITIALSTATE_%s'%(i)})
    

      sm_patrolling.add('SETINITIALSTATE_%s'%(i),
              StateSetInitialState(),
              transitions = {'succeeded':next_waypoint})

  ##################################################################################################
  ################################   Leader check followers   ######################################
  ##################################################################################################

  sm_checkteammates = smach.StateMachine(outcomes = ['stop_patrolling','preempted'],
                           input_keys = ['action_goal'])
  with sm_checkteammates:
    sm_checkteammates.add('CHECK_TEAMMATES',
            StateCheckTeammates(),
            transitions = {'check_teammates_preempted':'preempted'})


  ##################################################################################################
  #######################################   Leader CM   ############################################
  ##################################################################################################


  def patrollingandcheck_term_cb(outcome_map):
    if outcome_map['SM_CHECK_TEAMMATES'] == 'stop_patrolling':
      return True
    if outcome_map['SM_PATROLLING'] == 'aborted':
      return True
    else:
      return False

  def patrollingandcheck_out_cb(outcome_map):
    if outcome_map['SM_PATROLLING'] == 'aborted' or outcome_map['SM_CHECK_TEAMMATES'] == 'aborted':
      return 'aborted'
    if outcome_map['SM_CHECK_TEAMMATES'] == 'stop_patrolling':
      return 'stop_patrolling'
    if outcome_map['SM_PATROLLING'] == 'preempted' or outcome_map['SM_CHECK_TEAMMATES'] == 'preempted':
      return 'preempted'
    else:
      return 'stop_patrolling' 



  cm_patrollingandcheck = smach.Concurrence(outcomes = ['stop_patrolling','preempted','aborted'],
                                            default_outcome = 'stop_patrolling',
                                            input_keys = ['pose_stamped',
                                                          'action_goal',
                                                          'bias_length',
                                                          'formation_id',
                                                          'max_cc',
                                                          'leader_id',
                                                          'follower_1',
                                                          'follower_2',
                                                          'follower_3',
                                                          'number_of_robots',
                                                          'switch_enabled'],
                                            child_termination_cb = patrollingandcheck_term_cb,
                                            outcome_cb = patrollingandcheck_out_cb)

  with cm_patrollingandcheck:
    cm_patrollingandcheck.add('SM_PATROLLING', sm_patrolling)
    cm_patrollingandcheck.add('SM_CHECK_TEAMMATES', sm_checkteammates)
    # cm_patrollingandcheck.add('SM_FORMATION_LEADER', sm_formations_leader)







#.........................................................................................................................
####################################### CONCURRENCY 2 (FOLLOWER) #########################################################
#.........................................................................................................................
     
  
  ##################################################################################################
  ##################################   Follower: Move to leader   ##################################
  ##################################################################################################

  smb1 = smach.StateMachine(outcomes = ['close_to_leader','preempted','aborted'],
                           input_keys = ['action_goal'])
  with smb1:
    smb1.add('MOVE_TO_LEADER',
            StateGoToLeader(),
            remapping = {'target_pose':'pose_stamped'})



  ##################################################################################################
  #################################   Follower: Monitor distance   #################################
  ##################################################################################################

  smb2 = smach.StateMachine(outcomes = ['close_to_leader','preempted'],
                           input_keys = ['action_goal'])
  with smb2:
    smb2.add('CHECK_CLOSE_TO_LEADER',
            StateCheckClosetoLeader(),
            transitions = {'close_to_leader':'close_to_leader',
                           'close_to_leader_preempted':'preempted'})

  ##################################################################################################
  #################################   Follower: Write NOT ready    #################################
  ##################################################################################################

  smb3 = smach.StateMachine(outcomes = ['close_to_leader','preempted'],
                           input_keys = ['action_goal'])
  with smb3:
    smb3.add('WRITE_NOT_READY',
            StateWriteNotReady(),
            transitions = {'not_ready_written':'close_to_leader',
                           'write_not_ready_preempted':'preempted'})


  ##################################################################################################
  ###############################   CM_2 Follower: Move to leader   ################################
  ##################################################################################################

  def gotoleader_term_cb(outcome_map):
    if outcome_map['SM_MOVE_TO_LEADER'] == 'close_to_leader' or outcome_map['SM_CHECK_CLOSE_TO_LEADER'] == 'close_to_leader':
      return True
    if outcome_map['SM_MOVE_TO_LEADER'] == 'aborted':
      return True
    else:
      return False

  def gotoleader_out_cb(outcome_map):
    if outcome_map['SM_MOVE_TO_LEADER'] == 'aborted':
      return 'aborted'
    if outcome_map['SM_MOVE_TO_LEADER'] == 'close_to_leader' or outcome_map['SM_CHECK_CLOSE_TO_LEADER'] == 'close_to_leader' :
      return 'close_to_leader'
    if outcome_map['SM_MOVE_TO_LEADER'] == 'preempted' or outcome_map['SM_CHECK_CLOSE_TO_LEADER'] == 'preempted' or outcome_map['SM_WRITE_NOT_READY'] == 'preempted':
      return 'preempted'
    else:
      return 'close_to_leader' 

  cm_gotoleader = smach.Concurrence(outcomes = ['close_to_leader','preempted','aborted'],
                                    default_outcome = 'close_to_leader',
                                    input_keys = ['pose_stamped','action_goal'],
                                    child_termination_cb = gotoleader_term_cb,
                                    outcome_cb = gotoleader_out_cb)

  with cm_gotoleader:
    cm_gotoleader.add('SM_MOVE_TO_LEADER', smb1)
    cm_gotoleader.add('SM_CHECK_CLOSE_TO_LEADER', smb2)
    cm_gotoleader.add('SM_WRITE_NOT_READY', smb3)


#.........................................................................................................................
####################################### CONCURRENCY 3 (FOLLOWER) #########################################################
#.........................................................................................................................


  ##################################################################################################
  ##################################   Follower: Formation   #######################################
  ##################################################################################################

  smc1 = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
                           input_keys = ['action_goal',
                                         'bias_length',
                                         'formation_id',
                                         'max_cc',
                                         'leader_id',
                                         'follower_1',
                                         'follower_2',
                                         'follower_3',
                                         'number_of_robots',
                                         'switch_enabled'])
  with smc1:
    def formation_result_cb(userdata, status, result):
      if status == GoalStatus.PREEMPTED:
        return 'succeeded'

    smc1.add('MOVE_FORMATION',
            smach_ros.SimpleActionState('formation_control',
                                        FormationControlAction,
                                        result_cb=formation_result_cb,
                                        goal_slots = ['bias_length',
                                                      'formation_id',
                                                      'max_cc',
                                                      'leader_id',
                                                      'follower_1',
                                                      'follower_2',
                                                      'follower_3',
                                                      'number_of_robots',
                                                      'switch_enabled']))

  smb1 = smach.StateMachine(outcomes = ['close_to_leader','preempted','aborted'],
                           input_keys = ['action_goal'])
  with smb1:
    smb1.add('MOVE_TO_LEADER',
            StateGoToLeader(),
            remapping = {'target_pose':'pose_stamped'})

  ##################################################################################################
  #################################   Follower: Monitor distance   #################################
  ##################################################################################################

  smc2 = smach.StateMachine(outcomes = ['far_from_leader','preempted'],
                           input_keys = ['action_goal'])
  with smc2:
    smc2.add('CHECK_FAR_FROM_LEADER',
            StateCheckFarFromLeader(),
            transitions = {'far_from_leader':'far_from_leader',
                           'far_from_leader_preempted':'preempted'})

  ##################################################################################################
  #################################   Follower: Write ready    #####################################
  ##################################################################################################

  smc3 = smach.StateMachine(outcomes = ['far_from_leader','preempted'],
                           input_keys = ['action_goal'])
  with smc3:
    smc3.add('WRITE_READY',
            StateWriteReady(),
            transitions = {'ready_written':'far_from_leader',
                           'write_ready_preempted':'preempted'})


  ##################################################################################################
  ###############################     CM_3 Follower: Formation      ################################
  ##################################################################################################


  def move_formation_term_cb(outcome_map):
    if outcome_map['SM_CHECK_FAR_FROM_LEADER'] == 'far_from_leader': #and outcome_map['SM_WRITE_READY'] == 'far_from_leader':
      return True
    if outcome_map['SM_MOVE_FORMATION'] == 'aborted':
      return True
    if outcome_map['SM_MOVE_FORMATION'] == 'preempted':
      return True
    else:
      return False

  def move_formation_out_cb(outcome_map):
    if outcome_map['SM_MOVE_FORMATION'] == 'aborted':
      return 'aborted'
    if outcome_map['SM_CHECK_FAR_FROM_LEADER'] == 'far_from_leader':
      return 'far_from_leader'
    if outcome_map['SM_CHECK_FAR_FROM_LEADER'] == 'preempted':
      return 'preempted'
    if outcome_map['SM_MOVE_FORMATION'] == 'preempted' or outcome_map['SM_WRITE_READY'] == 'preempted':
      return 'far_from_leader'
    else:
      return 'far_from_leader' 

  cm_move_formation = smach.Concurrence(outcomes = ['far_from_leader','preempted','aborted'],
                                    default_outcome = 'far_from_leader',
                                    input_keys = ['action_goal',
                                                  'bias_length',
                                                  'formation_id',
                                                  'max_cc',
                                                  'leader_id',
                                                  'follower_1',
                                                  'follower_2',
                                                  'follower_3',
                                                  'number_of_robots',
                                                  'switch_enabled'],
                                    child_termination_cb = move_formation_term_cb,
                                    outcome_cb = move_formation_out_cb)

  with cm_move_formation:
    cm_move_formation.add('SM_MOVE_FORMATION', smc1)
    cm_move_formation.add('SM_CHECK_FAR_FROM_LEADER', smc2)
    cm_gotoleader.add('SM_WRITE_READY', smc3)
    




  #####################################################################################
  ########################  Cooperative Patrolling State Machine  #####################
  #####################################################################################

  sm_coop = smach.StateMachine(outcomes = ['preempted','aborted','finish_test'],
                               input_keys = ['action_goal'])

  sm_coop.userdata.pose_stamped = pose2pose_stamped(0.0,0.0,0.0)

  sm_coop.userdata.bias_length = 0
  sm_coop.userdata.formation_id = 0
  sm_coop.userdata.max_cc = 0
  sm_coop.userdata.leader_id = 0
  sm_coop.userdata.follower_1 = 0
  sm_coop.userdata.follower_2 = 0
  sm_coop.userdata.follower_3 = 0
  sm_coop.userdata.number_of_robots = 0
  sm_coop.userdata.switch_enabled = 0

  with sm_coop:
    #
    # Common
    #
    sm_coop.add('SELECT_ROLE',
                StateSelectRole(),
                transitions = {'leader_selected':'WAIT_FOR_TEAMMATES',
                           'follower_selected':'GO_TO_LEADER',
                           'selectrole_preempted':'preempted'})

    #
    # Leader Side
    #
    sm_coop.add('WAIT_FOR_TEAMMATES',
                StateWaitForTeammates(),
                transitions = {'ready_to_move':'LEADER_PATROLLING',
                               'wait_for_teammates_preempted':'preempted'})

    sm_coop.add('LEADER_PATROLLING',
                cm_patrollingandcheck,
                transitions = {'stop_patrolling':'WAIT_FOR_TEAMMATES'})

    sm_coop.add('GO_TO_LEADER',
                cm_gotoleader,
                transitions = {'close_to_leader':'FOLLOW_LEADER'})


    sm_coop.add('FOLLOW_LEADER',
                cm_move_formation,
                transitions = {'far_from_leader':'GO_TO_LEADER'})


  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      CooperativePatrollingAction,
                                      wrapped_container = sm_coop,
                                      succeeded_outcomes = ['finish_test'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'])

  # Create and start the introspection server
  sis = smach_ros.IntrospectionServer('server_name', sm_coop, '/SM_ROOT')
  sis.start()
  asw.run_server()

  rospy.spin()

  sis.stop()
