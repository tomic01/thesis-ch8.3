#! /usr/bin/env python

# Behavior implemented: CooperativePatrolling
# Author: Jose Nuno Pereira
# Description: Patrolling is a behavior designed for visiting a sequence of predetermined locations.
#              It receives a set of locations and creates a SMACH state machine in which each state
#              moves the robot to the next location. All locations are visited by the order they are
#              received. Upon reaching the final location it inverts the orientation of the each
#              location and proceeds to visit each location in the opposite direction. When all
#              locations are visited again it returns to the initial state.
#              Cooperative Patrolling uses graph based formation control to make a set of followers
#              follow a leader that is executing the simple Patrolling behavior.
#              Implemented as ActionServerWrapper for SMACH state machine.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import sys
import smach
import smach_ros
from functools import partial

from monarch_behaviors.msg import CooperativePatrollingAction

from monarch_situational_awareness.srv import *
from monarch_situational_awareness.msg import *

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from graph_based_formations.msg import *

DISTANCE_FROM_LEADER_THRESHOLD = 3     # AW

create_sam_prelude_str = 'create'

def pose2alib(x, y, t):
  goal = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return MoveBaseGoal(target_pose=goal)

def pose2pose_stamped(x, y, t):
  pose_stamped = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return pose_stamped

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

    self.sam_readers_list = 3*[None]

  def follower_ready_cb(self, data, follower_id):
    '''Callback function for Follower_i_Ready SAMReader. Sets self.follower_ready[i]
       variable to True if data on SAM slot is True'''
    if data.data is True:
      self.follower_ready[follower_id - 1] = True

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateWaitForTeammates: Executing...')
    r = rospy.Rate(2)
    
    # create signal variables for followers
    # when self.follower_ready[i] = True, follower is close to leader;
    # when all followers ready, leader can start moving to location
    number_followers = int(userdata.action_goal.goal.array[7].value)-1
    self.follower_ready = [False for i in range(number_followers)]

    # create signal slots for followers
    rospy.wait_for_service(create_sam_prelude_str + '_slot')
    create_slot = rospy.ServiceProxy(create_sam_prelude_str + '_slot', CreateSlot)

    for follower in range(1, number_followers + 1):
      sp = SlotProperties(name = 'Follower_' + str(follower) + '_Ready',
                          type_str = 'std_msgs/Bool',
                          is_shared = True,
                          is_latched = False)
      create_slot(sp)

    # create readers for slots just created
    rospy.wait_for_service(create_sam_prelude_str + '_reader')
    create_reader = rospy.ServiceProxy(create_sam_prelude_str + '_reader', CreateReader)
    #sam_readers_list = []

    for follower in range(1, number_followers + 1):
      reader_properties = ReaderProperties(slot_name = 'Follower_' + str(follower) + '_Ready')
      reader_response = CreateReaderResponse()
      while reader_response.success is False:
        reader_response = create_reader(reader_properties)
        rospy.loginfo('Attempting to create reader for slot Follower_%d_Ready',follower)
        r.sleep()
      self.sam_readers_list[follower - 1]=rospy.Subscriber(reader_response.topic_name,
                                               Bool,
                                               partial(self.follower_ready_cb,
                                                       follower_id = follower))

    rospy.loginfo('SAM readers created for all Follower Ready slots, waiting for Follower Ready signals')

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
    #for reader in range(number_followers):
      #sam_readers_list[reader].unregister()

    rospy.wait_for_service('remove_reader')
    remove_reader_service = rospy.ServiceProxy('remove_reader',RemoveReader)
    for follower in range(1, number_followers + 1):
      remove_reader_response = RemoveReaderResponse()

      while remove_reader_response.success is False:
        remove_reader_response = remove_reader_service(slot_name = 'Follower_' + str(follower) + '_Ready', reader_name = rospy.get_name())
        r.sleep()

    rospy.sleep(5)

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

  def follower_not_ready_cb(self, data, follower_id):
    '''Callback function for Follower_i_Ready SAMReader. Sets self.follower_not_ready[i]
       variable to True if data on SAM slot is False'''
    if data.data is False:
      self.follower_not_ready[follower_id - 1] = True

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateCheckTeammates: Executing...')

    r = rospy.Rate(2)
    
    # create signal variables for followers
    # when self.follower_not_ready[i] = True, follower is close to leader;
    # when all followers ready, leader can start moving to location
    number_followers = int(userdata.action_goal.goal.array[7].value)-1
    self.follower_not_ready = [False for i in range(number_followers)]

    # create signal slots for followers
    rospy.wait_for_service(create_sam_prelude_str + '_slot')
    create_slot = rospy.ServiceProxy(create_sam_prelude_str + '_slot', CreateSlot)

    for follower in range(1, number_followers + 1):
      sp = SlotProperties(name = 'Follower_' + str(follower) + '_Ready',
                          type_str = 'std_msgs/Bool',
                          is_shared = True,
                          is_latched = False)
      create_slot(sp)

    # create readers for slots just created
    rospy.wait_for_service(create_sam_prelude_str + '_reader')
    create_reader = rospy.ServiceProxy(create_sam_prelude_str + '_reader', CreateReader)
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
                                               partial(self.follower_not_ready_cb,
                                                       follower_id = follower)))

    rospy.loginfo('SAM readers created for all Follower Ready slots, waiting for Follower Not Ready signals')

    # loop until all followers ready or preempted
    preempted = False
    while not rospy.is_shutdown() and \
          any(self.follower_not_ready) is False and \
          preempted is False:

      if self.preempt_requested():
        self.service_preempt()
        preempted = True

      # sleep a bit
      r.sleep()

    # unregistering from follower_i_ready sam slot
    for reader in range(number_followers):
      sam_readers_list[reader].unregister()

    rospy.sleep(5)

    if not preempted:
      return 'stop_patrolling'
    else:
      return 'check_teammates_preempted'


class StateFindLeader(smach.State):
  """StateFindLeader sets the move base goal to the position of the Leader robot."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['leader_position_set','find_leader_preempted'],
                         input_keys = ['action_goal'],
                         output_keys = ['find_leader_pose_out'])

    self.sam_reader = None

  def leader_pose_cb(self, data):
    '''Callback funcion for [mbotLEADER] tfPose. Sets find_leader_pose_out.'''
    self.leader_pose = PoseStamped(header = Header(frame_id = "/map"),
                                   pose = data)
    self.leader_found = True

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateFindLeader: Executing...')

    r = rospy.Rate(2)

    # get leader id from action_goal
    leader_id = int(userdata.action_goal.goal.array[3].value)
    if leader_id < 10:
      leader_id = '0' + str(leader_id)
    else:
      leader_id = str(leader_id)
    self.leader_found = False

    # Create SAM Reader
    rospy.wait_for_service(create_sam_prelude_str + '_reader')
    reader_service = rospy.ServiceProxy(create_sam_prelude_str + '_reader', CreateReader)
    reader_properties = ReaderProperties(slot_name = '[mbot' + leader_id + '] tfPose')
    reader_response = CreateReaderResponse()

    while reader_response.success is False:
      reader_response = reader_service(reader_properties)
      r.sleep()

    self.sam_reader = rospy.Subscriber(reader_response.topic_name,
                                  Pose,
                                  self.leader_pose_cb)

    while not rospy.is_shutdown() and self.leader_found is False:

      if self.preempt_requested():
        #sam_reader.unregister() # unregistering from topic
        rospy.wait_for_service('remove_reader')
        remove_reader_service = rospy.ServiceProxy('remove_reader',RemoveReader)
        remove_reader_response = RemoveReaderResponse()

        while remove_reader_response.success is False:
          remove_reader_response = remove_reader_service(slot_name = '[mbot' + leader_id + '] tfPose', reader_name = rospy.get_name())
          r.sleep()

        self.service_preempt()
        return 'find_leader_preempted'

      # sleep a bit
      r.sleep()

    follower_goal_pose = self.leader_pose
    userdata.find_leader_pose_out = follower_goal_pose
    #sam_reader.unregister() # unregistering from topic
    rospy.wait_for_service('remove_reader')
    remove_reader_service = rospy.ServiceProxy('remove_reader',RemoveReader)
    remove_reader_response = RemoveReaderResponse()

    while remove_reader_response.success is False:
      remove_reader_response = remove_reader_service(slot_name = '[mbot' + leader_id + '] tfPose', reader_name = rospy.get_name())
      r.sleep()
      
    return 'leader_position_set'


class StateCheckClosetoLeader(smach.State):
  """StateCheckClosetoLeader checks if robot is close to leader robot."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['close_to_leader','close_to_leader_preempted'],
                         input_keys = ['action_goal'])

  def leader_pose_cb(self, data):
    '''Callback funcion for [mbotLEADER] tfPose. Sets find_leader_pose_out.'''
    self.leader_pose = data
    self.leader_pose_set = True

  def own_pose_cb(self, data):
    '''Callback funcion for own pose.'''
    self.own_pose = data
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

    # Create SAM Reader
    rospy.wait_for_service(create_sam_prelude_str + '_reader')
    reader_service = rospy.ServiceProxy(create_sam_prelude_str + '_reader', CreateReader)
    reader_properties = ReaderProperties(slot_name = '[mbot' + leader_id + '] tfPose')
    reader_response = CreateReaderResponse()

    r.sleep()

    while reader_response.success is False:
      reader_response = reader_service(reader_properties)
      r.sleep()

    sam_reader = rospy.Subscriber(reader_response.topic_name,
                                  Pose,
                                  self.leader_pose_cb)

    r.sleep()

    if robot_id < 10:
      reader_properties = ReaderProperties(slot_name = '[mbot0' + str(robot_id) + '] tfPose')
    else:
      reader_properties = ReaderProperties(slot_name = '[mbot' + str(robot_id) + '] tfPose')
    reader_response = CreateReaderResponse()

    while reader_response.success is False:
      reader_response = reader_service(reader_properties)
      r.sleep()

    own_pose_sub = rospy.Subscriber(reader_response.topic_name,
                                    Pose,
                                    self.own_pose_cb)

    self.leader_pose_set = False
    self.own_pose_set = False

    while not rospy.is_shutdown():

      if self.preempt_requested():
        #sam_reader.unregister() # unregistering from topic
        rospy.wait_for_service('remove_reader')
        remove_reader_service = rospy.ServiceProxy('remove_reader',RemoveReader)
        remove_reader_response = RemoveReaderResponse()
    
        while remove_reader_response.success is False:
          remove_reader_response = remove_reader_service(slot_name = '[mbot' + leader_id + '] tfPose', reader_name = rospy.get_name())
          r.sleep()


        own_pose_sub.unregister()
        self.service_preempt()
        return 'close_to_leader_preempted'

      if self.leader_pose_set and self.own_pose_set:
        distance = math.sqrt(math.pow(self.leader_pose.position.x - self.own_pose.position.x,2) +\
                             math.pow(self.leader_pose.position.y - self.own_pose.position.y,2))

        if distance < DISTANCE_FROM_LEADER_THRESHOLD:
          #sam_reader.unregister() # unregistering from topic
          rospy.wait_for_service('remove_reader')
          remove_reader_service = rospy.ServiceProxy('remove_reader',RemoveReader)
          remove_reader_response = RemoveReaderResponse()
      
          while remove_reader_response.success is False:
            remove_reader_response = remove_reader_service(slot_name = '[mbot' + leader_id + '] tfPose', reader_name = rospy.get_name())
            r.sleep()

          own_pose_sub.unregister()
          return 'close_to_leader'

      # sleep a bit
      r.sleep()


class StateWriteReady(smach.State):
  """writes that follower_i is ready to move in formation in appropriate slot"""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['ready_written','write_ready_preempted'],
                          input_keys = ['action_goal'])
    self.ready_pub = None

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateWriteReady: Executing...')
    
    r = rospy.Rate(2)

    if self.preempt_requested():
      self.service_preempt()
      return 'write_ready_preempted'

    number_followers = int(userdata.action_goal.goal.array[7].value)-1
    rospy.wait_for_service(create_sam_prelude_str + '_writer')
    create_writer = rospy.ServiceProxy(create_sam_prelude_str + '_writer', CreateWriter)
    for follower in range(1, number_followers + 1):
      # if robot is follower 
      if robot_id == int(userdata.action_goal.goal.array[3 + follower].value):
        writer_properties = WriterProperties(slot_name = 'Follower_' + str(follower) + '_Ready')
        writer_response = CreateWriterResponse()
        while writer_response.success is False:
          writer_response = create_writer(writer_properties)
          rospy.loginfo('Attempting to create writer for slot Follower_%d_Ready',follower)
          r.sleep()

        self.ready_pub = rospy.Publisher(writer_response.topic_name,
                                    Bool)
        rospy.sleep(0.5)
        self.ready_pub.publish(True)
        rospy.sleep(0.5)
        self.ready_pub.publish(True)
        rospy.sleep(0.5)
        self.ready_pub.publish(True)


        return 'ready_written'


class StateCheckFarFromLeader(smach.State):
  """StateCheckFarFromLeader checks if robot is far from the leader robot."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['far_from_leader','far_from_leader_preempted'],
                         input_keys = ['action_goal'])

  def leader_pose_cb(self, data):
    '''Callback funcion for [mbotLEADER] tfPose. Sets find_leader_pose_out.'''
    self.leader_pose = data
    self.leader_pose_set = True

  def own_pose_cb(self, data):
    '''Callback funcion for own pose.'''
    self.own_pose = data
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

    # Create SAM Reader
    rospy.wait_for_service(create_sam_prelude_str + '_reader')
    reader_service = rospy.ServiceProxy(create_sam_prelude_str + '_reader', CreateReader)
    reader_properties = ReaderProperties(slot_name = '[mbot' + leader_id + '] tfPose')
    reader_response = CreateReaderResponse()

    while reader_response.success is False:
      reader_response = reader_service(reader_properties)
      r.sleep()

    sam_reader = rospy.Subscriber(reader_response.topic_name,
                                  Pose,
                                  self.leader_pose_cb)

    if robot_id < 10:
      reader_properties = ReaderProperties(slot_name = '[mbot0' + str(robot_id) + '] tfPose')
    else:
      reader_properties = ReaderProperties(slot_name = '[mbot' + str(robot_id) + '] tfPose')
    reader_response = CreateReaderResponse()

    while reader_response.success is False:
      reader_response = reader_service(reader_properties)
      r.sleep()

    own_pose_sub = rospy.Subscriber(reader_response.topic_name,
                                    Pose,
                                    self.own_pose_cb)

    self.leader_pose_set = False
    self.own_pose_set = False

    while not rospy.is_shutdown():

      if self.preempt_requested():
        sam_reader.unregister() # unregistering from topic
        own_pose_sub.unregister()
        self.service_preempt()
        return 'far_from_leader_preempted'

      if self.leader_pose_set and self.own_pose_set:
        distance = math.sqrt(math.pow(self.leader_pose.position.x - self.own_pose.position.x,2) +\
                             math.pow(self.leader_pose.position.y - self.own_pose.position.y,2))
        if distance > DISTANCE_FROM_LEADER_THRESHOLD:
          sam_reader.unregister() # unregistering from topic
          own_pose_sub.unregister()
          return 'far_from_leader'

      # sleep a bit
      r.sleep()


class StateWriteNotReady(smach.State):
  """writes that follower_i is not ready to move in formation in appropriate slot"""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['not_ready_written','write_not_ready_preempted'],
                          input_keys = ['action_goal'])

  def execute(self, userdata):
    # publish info to the console for the user
    rospy.loginfo('StateWriteNotReady: Executing...')
    
    r = rospy.Rate(2)

    if self.preempt_requested():
      self.service_preempt()
      return 'write_not_ready_preempted'

    number_followers = int(userdata.action_goal.goal.array[7].value)-1
    rospy.wait_for_service(create_sam_prelude_str + '_writer')
    create_writer = rospy.ServiceProxy(create_sam_prelude_str + '_writer', CreateWriter)
    for follower in range(1, number_followers + 1):
      # if robot is follower 
      if robot_id == int(userdata.action_goal.goal.array[3 + follower].value):
        writer_properties = WriterProperties(slot_name = 'Follower_' + str(follower) + '_Ready')
        writer_response = CreateWriterResponse()
        while writer_response.success is False:
          writer_response = create_writer(writer_properties)
          rospy.loginfo('Attempting to create writer for slot Follower_%d_Ready',follower)
          r.sleep()

        ready_pub = rospy.Publisher(writer_response.topic_name,
                                    Bool)
        rospy.sleep(0.5)
        ready_pub.publish(False)

        return 'not_ready_written'
#
#   Main
#
    
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

  # EPFL Robotics Arena
  # waypoints = [(7.2,12.8,1.78),
  #               (7.0,10.8,-0.1),
  #               (6.05,8.7,-1.74)]

  # EPFL Robotics Arena and Corridor
  #waypoints = [(6.15,7.8,-1.7),
                #(10.15,10.7,0.0),
                #(10.35,11.2,-3.0),
                #(9.7,7.43, 0.0),
                ##(10.15,10.7,-3.0),
                #(6.05,8.7,-1.74)]

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
  #waypoints = [(6.35,-3.35,0.00),
               #(-6.80,0.65,3.08),
               #(-4.40,-3.10,-1.63),
               #(-13.10,-2.00,2.86),
               #(-20.20,-2.75,1.53),
               #(-19.75,1.95,-0.04),
               #(-6.40,2.55,-0.98),
               #(-7.05,-2.50,-1.57)]

  # IW 8th floor isr8-v07cr_new_gamearea.yaml
  waypoints = [(4.65,11.85,-1.99),
               (0.35,5.15,-1.32),
               (2.15,-1.60,-1.86),
               (8.00,-3.90,-3.14)]
               #(12.40,11.30,-0.30),
               #(19.05,8.50,-1.57),
               #(12.30,0.70,-0.00)]


  opposite_waypoints = [(l[0],l[1],(l[2]+math.pi)%(2*math.pi)) for l in waypoints]
  opposite_waypoints.reverse()
  waypoints.extend(opposite_waypoints)

  #
  #   Patrolling State Machine
  #
      
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

  #
  #   Leader Patrolling + Check if follower sends signal Concurrence
  #

  sm_checkteammates = smach.StateMachine(outcomes = ['stop_patrolling','preempted'],
                           input_keys = ['action_goal'])
  with sm_checkteammates:
    sm_checkteammates.add('CHECK_TEAMMATES',
            StateCheckTeammates(),
            transitions = {'check_teammates_preempted':'preempted'})


  ##################################################################################################
  ############              Graph based formation server for leader       ##########################  AW
  ##################################################################################################

  sm_formations_leader = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
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
  with sm_formations_leader:
    sm_formations_leader.add('LEADER_MOVE_FORMATION',
            smach_ros.SimpleActionState('graph_based_controller',
                                        ControllerGBAction,
                                        goal_slots = ['bias_length',
                                                      'formation_id',
                                                      'max_cc',
                                                      'leader_id',
                                                      'follower_1',
                                                      'follower_2',
                                                      'follower_3',
                                                      'number_of_robots',
                                                      'switch_enabled']))

  ##################################################################################################
  ############           ~  Graph based formation server for leader       ##########################  AW
  ##################################################################################################



  def patrollingandcheck_term_cb(outcome_map):
    if outcome_map['SM_CHECK_TEAMMATES'] == 'stop_patrolling':
      return True
    if outcome_map['SM_PATROLLING'] == 'aborted':
      return True
    else:
      return False

  def patrollingandcheck_out_cb(outcome_map):
    if outcome_map['SM_PATROLLING'] == 'aborted' or outcome_map['SM_FORMATION_LEADER'] == 'aborted':
      return 'aborted'
    if outcome_map['SM_CHECK_TEAMMATES'] == 'stop_patrolling':
      return 'stop_patrolling'
    if outcome_map['SM_PATROLLING'] == 'preempted' or outcome_map['SM_CHECK_TEAMMATES'] == 'preempted' or outcome_map['SM_FORMATION_LEADER'] == 'preempted':
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
    cm_patrollingandcheck.add('SM_FORMATION_LEADER', sm_formations_leader)
  
  #
  #   Move base + Check close to pose Concurrence
  #

  sm1 = smach.StateMachine(outcomes = ['close_to_leader','preempted','aborted'],
                           input_keys = ['pose_stamped'])
  with sm1:
    sm1.add('MOVE_BASE_TO_LEADER',
            smach_ros.SimpleActionState('move_base',
                                        MoveBaseAction,
                                        goal_slots = ['target_pose']),
            transitions = {'succeeded':'close_to_leader'},
            remapping = {'target_pose':'pose_stamped'})

  sm2 = smach.StateMachine(outcomes = ['close_to_leader','preempted'],
                           input_keys = ['action_goal'])
  with sm2:
    sm2.add('CHECK_CLOSE_TO_LEADER',
            StateCheckClosetoLeader(),
            transitions = {'close_to_leader':'close_to_leader',
                           'close_to_leader_preempted':'preempted'})

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
    if outcome_map['SM_MOVE_TO_LEADER'] == 'close_to_leader' or outcome_map['SM_CHECK_CLOSE_TO_LEADER'] == 'close_to_leader':
      return 'close_to_leader'
    if outcome_map['SM_MOVE_TO_LEADER'] == 'preempted' or outcome_map['SM_CHECK_CLOSE_TO_LEADER'] == 'preempted':
      return 'preempted'
    else:
      return 'close_to_leader' 

  cm_gotoleader = smach.Concurrence(outcomes = ['close_to_leader','preempted','aborted'],
                                    default_outcome = 'close_to_leader',
                                    input_keys = ['pose_stamped','action_goal'],
                                    child_termination_cb = gotoleader_term_cb,
                                    outcome_cb = gotoleader_out_cb)

  with cm_gotoleader:
    cm_gotoleader.add('SM_MOVE_TO_LEADER', sm1)
    cm_gotoleader.add('SM_CHECK_CLOSE_TO_LEADER', sm2)


  #
  #   Graph based formation control + Check far from pose Concurrence
  #

  sm3 = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
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
  with sm3:
    sm3.add('MOVE_FORMATION',
            smach_ros.SimpleActionState('graph_based_controller',
                                        ControllerGBAction,
                                        goal_slots = ['bias_length',
                                                      'formation_id',
                                                      'max_cc',
                                                      'leader_id',
                                                      'follower_1',
                                                      'follower_2',
                                                      'follower_3',
                                                      'number_of_robots',
                                                      'switch_enabled']))

  sm4 = smach.StateMachine(outcomes = ['far_from_leader','preempted'],
                           input_keys = ['action_goal'])
  with sm4:
    sm4.add('CHECK_FAR_FROM_LEADER',
            StateCheckFarFromLeader(),
            transitions = {'far_from_leader':'far_from_leader',
                           'far_from_leader_preempted':'preempted'})

  def move_formation_term_cb(outcome_map):
    if outcome_map['SM_CHECK_FAR_FROM_LEADER'] == 'far_from_leader':
      return True
    if outcome_map['SM_MOVE_FORMATION'] == 'aborted':
      return True
    else:
      return False

  def move_formation_out_cb(outcome_map):
    if outcome_map['SM_MOVE_FORMATION'] == 'aborted':
      return 'aborted'
    if outcome_map['SM_CHECK_FAR_FROM_LEADER'] == 'far_from_leader':
      return 'far_from_leader'
    if outcome_map['SM_MOVE_FORMATION'] == 'preempted' or outcome_map['SM_CHECK_FAR_FROM_LEADER'] == 'preempted':
      return 'preempted'
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
    cm_move_formation.add('SM_MOVE_FORMATION', sm3)
    cm_move_formation.add('SM_CHECK_FAR_FROM_LEADER', sm4)
    
  #
  #   Cooperative Patrolling State Machine
  #

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
                           'follower_selected':'FIND_LEADER',
                           'selectrole_preempted':'preempted'})

    #
    # Leader Side
    #
    sm_coop.add('WAIT_FOR_TEAMMATES',
                StateWaitForTeammates(),
                transitions = {'ready_to_move':'LEADER_PATROLLING', # 'ready_to_move':'MOVE_TO_LOCATION'
                               'wait_for_teammates_preempted':'preempted'})

    sm_coop.add('LEADER_PATROLLING',
                cm_patrollingandcheck,
                transitions = {'stop_patrolling':'WAIT_FOR_TEAMMATES'})

    #
    # Follower Side
    #
    sm_coop.add('FIND_LEADER',
                StateFindLeader(),
                transitions = {'leader_position_set':'MOVE_TO_LEADER',
                               'find_leader_preempted':'preempted'},
                remapping = {'find_leader_pose_out':'pose_stamped'})

    sm_coop.add('MOVE_TO_LEADER',
                cm_gotoleader,
                transitions = {'close_to_leader':'WRITE_READY'})

    sm_coop.add('WRITE_READY',
                StateWriteReady(),
                transitions = {'ready_written':'FOLLOW_LEADER',
                               'write_ready_preempted':'preempted'})

    sm_coop.add('FOLLOW_LEADER',
                cm_move_formation,
                transitions = {'far_from_leader':'WRITE_NOT_READY'})

    sm_coop.add('WRITE_NOT_READY',
                StateWriteNotReady(),
                transitions = {'not_ready_written':'FIND_LEADER',
                               'write_not_ready_preempted':'preempted'})

  #
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