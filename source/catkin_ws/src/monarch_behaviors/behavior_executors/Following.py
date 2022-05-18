#! /usr/bin/env python

# Behavior implemented: FollowPerson
# Author: Zeynab Talebpour
# Description: ApproachPerson is a behavior designed for moving the robot to the
#              position of a given user in the environment in such a way that the robot
#              would always mantain a given distance and always positions itseld at the back.
#              It receives a user id as the goal of the approach person action. The behavior is implemented
#              as a three state SMACH. The first state checks the "People_Localization_Tracker"
#              SAM slot and retrieves the position of the user. This robot moves
#              to this position using a NavigationToTargetAction state. The third state sets
#              the result of the NavigationToTargetAction.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import rospkg
import os.path
import sys
import smach_ros
from math import sin,cos
from monarch_behaviors.msg import FollowPersonAction, FollowPersonResult
from monarch_msgs.msg import KeyValuePair, PersonLocalizationTrackingDataArray
from monarch_situational_awareness.msg import ReaderProperties
from monarch_situational_awareness.srv import CreateReader, CreateReaderResponse


rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path("scout_msgs"), "src"))
from scout_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf import *
import tf

#
#   States
#

class StateStart(smach.State):
  """StateStart sets the move base goal the robot must visit from the action goal."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['user_position_set','start_preempted'],
                          input_keys = ['action_goal','action_result'],
                          output_keys = ['tf_broadcasted','action_result'])
    self.tf_broadcaster = TransformBroadcaster()
    self.tf_broadcaster2 = TransformBroadcaster()
    self.listener = tf.TransformListener()  



  # Needs to be improved to return position if user_id is not passed correctly
  def people_localization_tracker_cb(self, data):
    '''Callback function for People_Localization_Tracker SAMReader. Sets the tf 
    if user is found.'''

    # k is the proximity that the robot should keep from the user 50 cm based on the proxemics theory
    k = 0.9
    print 'callback is active'
    for user in range(0,len(data.locations)):
      if data.locations[user].id >= self.user_id:
        self.user_id=1
        q = (data.locations[self.user_id-1].pose.pose.orientation.x, 
        data.locations[self.user_id-1].pose.pose.orientation.y,
        data.locations[self.user_id-1].pose.pose.orientation.z,
        data.locations[self.user_id-1].pose.pose.orientation.w)

        
	print 'callback reading'
	euler = tf.transformations.euler_from_quaternion(q)
        roll= euler[0]
        pitch = euler[1]
        yaw = euler[2]

		# tf of the point where navigationByTarget should always go with respect to the human is published
       #  self.tf_broadcaster.sendTransform((data.locations[self.user_id-1].pose.pose.position.x + k*cos(yaw),
       #  data.locations[self.user_id-1].pose.pose.position.y +  k*sin(yaw),
       # 0.00),
       #  # data.locations[self.user_id-1].pose.pose.orientation,
       #  tf.transformations.quaternion_from_euler(roll,pitch,-yaw ),
       #  rospy.Time.now(),
       #  "followingTarget",
       #  "map")


		# tf of the detected person published for debugging
        self.tf_broadcaster2.sendTransform((data.locations[self.user_id-1].pose.pose.position.x ,
        data.locations[self.user_id-1].pose.pose.position.y ,
        0.00),
        # data.locations[self.user_id-1].pose.pose.orientation,
        tf.transformations.quaternion_from_euler(roll,pitch,yaw),
        rospy.Time.now(),
        "person",
        "map")



       #  # tf of the point where navigationByTarget should always go with respect to the human is published
        self.tf_broadcaster.sendTransform((- k*cos(0.7853981634 ),
        - k*sin(0.7853981634 ),
       0.00),
        # data.locations[self.user_id-1].pose.pose.orientation,
        tf.transformations.quaternion_from_euler(0,0,0 ),
        rospy.Time.now(),
        "followingTarget",
        "person")



        self.tf_broadcasted = True
        self.user_found = True
     


      else:
        rospy.logerr('Approach Person Behavior: user %d not found.',self.user_id)
      
      self.user_id=1001




  def execute(self, userdata):

    # publish info to the console for the user
    rospy.loginfo('StateStart: Executing...')
    userdata.action_result.result.array[0].value = 'False'
    r = rospy.Rate(2)

	# why always take the first one? why are we using keyValuePair then
    self.user_id = int(userdata.action_goal.goal.array[0].value)
    self.user_found = False
    
    # Create SAM Reader
    rospy.wait_for_service('create_reader')
    print 'before sam '

    reader_service = rospy.ServiceProxy('create_reader', CreateReader)
    reader_properties = ReaderProperties(slot_name = 'People_Localization_Tracker_onboard')
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

    userdata.tf_broadcasted = self.tf_broadcasted

	# This is important for the tf to be published continuesly so if commented the target tf is always published
    # sam_reader.unregister()
    

    return 'user_position_set'

# --------------------------------------------------------------------- Finish state


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




#-------------------------------------------------------------------------------------------- main loop


#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_FollowPerson')
  rospy.loginfo('Starting up')

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['following_ended','preempted','aborted'],
                          input_keys = ['action_goal'], 
                          output_keys = ['action_result'])

  sm.userdata.action_result = FollowPersonResult()
  result_reached = KeyValuePair()
  result_reached.key = 'reached'
  result_reached.value = 'False'
  sm.userdata.action_result.result.array.append(result_reached)
  
  with sm:

    sm.add('START',
            StateStart(),
            transitions = {'user_position_set':'FOLLOW',
                           'start_preempted':'preempted'},
            remapping = {'start_output':'tf_broadcasted'})



    def move_goal_callback (userdata, goal):
      if userdata.found_person == True :
        goal_ = NavigationByTargetGoal()
        goal_.target_frame = 'person'
        goal_.heading_frame ='person'
        goal_.proximity = 0.700
        #goal_.target_frame = 'followingTarget'
        #goal_.heading_frame ='person'
        #goal_.proximity = 0.000
        goal_.follow_target = True #how about constant following
        return goal_
# how about else??

    sm.add('FOLLOW',
            smach_ros.SimpleActionState('navigation_by_target',
                                         NavigationByTargetAction,
                                         goal_cb= move_goal_callback,
                                         input_keys =['found_person']),
            transitions = {'succeeded':'following_ended',
                           'preempted':'preempted',
                           'aborted':'aborted'},
            remapping = {'found_person':'tf_broadcasted'})

   # sm.add('FINISH',
         #   StateFinish(),
          #  transitions = {'result_set':'following_ended',
           #                'finish_preempted':'preempted'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  # asw = smach_ros.ActionServerWrapper(action_name,
  #                                     FollowPersonAction,
  #                                     wrapped_container = sm,
  #                                     succeeded_outcomes = ['following_ended'],
  #                                     preempted_outcomes = ['preempted'],
  #                                     aborted_outcomes = ['aborted'])
  asw = smach_ros.ActionServerWrapper(action_name,
                                      FollowPersonAction,
                                      sm,
                                      ['following_ended'],['preempted'],['aborted'],
                                      goal_key ='action_goal',
                                      result_key ='action_result' )
  #Create and start the introspection server
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()
