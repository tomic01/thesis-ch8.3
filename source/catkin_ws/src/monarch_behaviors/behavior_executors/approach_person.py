#! /usr/bin/env python

# Behavior implemented: ApproachPerson
# Author: Jose Nuno Pereira
# Description: ApproachPerson is a behavior designed for moving the robot to the
#              position of a giver user in the environment. It receives a user id
#              as the goal of the approach person action. The behavior is implemented
#              as a three state SMACH. The first state checks the "People_Localization_Tracker"
#              SAM slot and retrieves the position of the user. This robot moves
#              to this position using a MoveBaseAction state. The third state sets
#              the result of the ApproachPersonAction.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros

from monarch_behaviors.msg import ApproachPersonAction, ApproachPersonResult
from monarch_msgs.msg import KeyValuePair, PersonLocalizationTrackingDataArray

from monarch_situational_awareness.msg import ReaderProperties
from monarch_situational_awareness.srv import CreateReader, CreateReaderResponse

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

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
                          outcomes = ['user_position_set','start_preempted'],
                          input_keys = ['action_goal','action_result'],
                          output_keys = ['start_pose_out','action_result'])

  # Needs to be improved to return position if user_id is not passed correctly
  def people_localization_tracker_cb(self, data):
    '''Callback function for People_Localization_Tracker SAMReader. Sets the user
       position if user is found.'''
    for user in range(0,len(data.locations)):
      if data.locations[user].id == self.user_id:
        self.start_pose_out = PoseStamped(header = Header(frame_id = "/map"),
                                          pose = data.locations[user].pose.pose)
        self.user_found = True
      else:
        rospy.logerr('Approach Person Behavior: user %d not found.',self.user_id)

  def execute(self, userdata):

    # publish info to the console for the user
    rospy.loginfo('StateStart: Executing...')

    userdata.action_result.result.array[0].value = 'False'

    r = rospy.Rate(2)

    self.user_id = int(userdata.action_goal.goal.array[0].value)
    self.user_found = False

    # Create SAM Reader
    rospy.wait_for_service('create_reader')
    reader_service = rospy.ServiceProxy('create_reader', CreateReader)
    reader_properties = ReaderProperties(slot_name = 'People_Localization_Tracker')
    #reader_properties = ReaderProperties(slot_name = 'Fake_People_Localization_Tracker')
    reader_response = CreateReaderResponse()

    while reader_response.success is False:
      reader_response = reader_service(reader_properties)
      r.sleep()

    sam_reader = rospy.Subscriber(reader_response.topic_name,
                                  PersonLocalizationTrackingDataArray,
                                  self.people_localization_tracker_cb)

    while not rospy.is_shutdown() and self.user_found is False:

      if self.preempt_requested():
        sam_reader.unregister() # unregistering from people localization tracker topic
        self.service_preempt()
        return 'start_preempted'

      # sleep a bit
      r.sleep()

    userdata.start_pose_out = self.start_pose_out
    sam_reader.unregister() # unregistering from people localization tracker topic
      
    return 'user_position_set'

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

#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_approachperson')
  rospy.loginfo('Starting up')

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['user_reached','preempted','aborted'],
                          input_keys = ['action_goal'], 
                          output_keys = ['action_result'])

  sm.userdata.pose_stamped = pose2pose_stamped(0.0,0.0,0.0)

  sm.userdata.action_result = ApproachPersonResult()
  result_reached = KeyValuePair()
  result_reached.key = 'reached'
  result_reached.value = 'False'
  sm.userdata.action_result.result.array.append(result_reached)
  
  with sm:
    sm.add('START',
            StateStart(),
            transitions = {'user_position_set':'MOVE',
                           'start_preempted':'preempted'},
            remapping = {'start_pose_out':'pose_stamped'})

    sm.add('MOVE',
            smach_ros.SimpleActionState('move_base',
                                         MoveBaseAction,
                                         goal_slots=['target_pose']),
            transitions = {'succeeded':'FINISH',
                           'preempted':'preempted',
                           'aborted':'aborted'},
            remapping = {'target_pose':'pose_stamped'})

    sm.add('FINISH',
            StateFinish(),
            transitions = {'result_set':'user_reached',
                           'finish_preempted':'preempted'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      ApproachPersonAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['user_reached'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'])

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()
