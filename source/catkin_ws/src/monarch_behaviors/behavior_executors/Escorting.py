#! /usr/bin/env python

# Behavior implemented: EscortPerson
# Author: Zeynab Talebpour
# Description: EscortPerson is a behavior designed for approaching a person,
# asking him to start following the robot and taking him to a specific location

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import rospkg
import os.path
import sys
import smach_ros

import logging; logger = logging.getLogger("robots.position.ros")

from math import sin,cos
from monarch_behaviors.msg import EscortPersonAction, EscortPersonResult
from monarch_msgs.msg import KeyValuePair, PersonLocalizationTrackingDataArray
from monarch_situational_awareness.msg import ReaderProperties
from monarch_situational_awareness.srv import CreateReader, CreateReaderResponse
from scipy.spatial import distance

from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter


rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path("scout_msgs"), "src"))
from scout_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf import *
import tf
import numpy
#
#   States
#
# ---------------------------------------------------------------------- START state
class StateStart(smach.State):
  """StateStart sets the move base goal the robot must visit from the action goal."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['user_position_set','start_preempted'],
                          input_keys = ['action_goal','action_result'],
                          output_keys = ['tf_broadcasted','action_result'])
    self.tf_broadcaster = TransformBroadcaster()
    self.tf_broadcaster2 = TransformBroadcaster()
    self.tf_broadcaster3 = TransformBroadcaster()
  



  # Needs to be improved to return position if user_id is not passed correctly
  def people_localization_tracker_cb(self, data):
    '''Callback function for People_Localization_Tracker SAMReader. Sets the tf 
    if user is found.'''

    # k is the proximity that the robot should keep from the user 50 cm based on the proxemics theory
    k = 0.7
    # print 'Herreeeeeeeeeee '    
    # print 'sam callback'

    for user in range(0,len(data.locations)):
      if data.locations[user].id > 1000:
        self.user_id=2
        q = (data.locations[self.user_id-1].pose.pose.orientation.x, 
        data.locations[self.user_id-1].pose.pose.orientation.y,
        data.locations[self.user_id-1].pose.pose.orientation.z,
        data.locations[self.user_id-1].pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(q)
        roll= euler[0]
        pitch = euler[1]
        yaw = euler[2]

		# tf of the point where navigationByTarget should always go with respect to the human is published
        self.tf_broadcaster.sendTransform((data.locations[self.user_id-1].pose.pose.position.x - k*cos(yaw),
        data.locations[self.user_id-1].pose.pose.position.y - k*sin(yaw),
        0.0),
        # data.locations[self.user_id-1].pose.pose.orientation,
        tf.transformations.quaternion_from_euler(roll,pitch,3.141592 + yaw),
        rospy.Time.now(),
        "escortingGreetingTarget",
        "map")


		# tf of the detected person published for debugging
        self.tf_broadcaster2.sendTransform((data.locations[self.user_id-1].pose.pose.position.x ,
        data.locations[self.user_id-1].pose.pose.position.y ,
        0.0),
        # data.locations[self.user_id-1].pose.pose.orientation,
        tf.transformations.quaternion_from_euler(roll,pitch,yaw),
        rospy.Time.now(),
        "person",
        "map")

        self.tf_broadcaster3.sendTransform((self.escort_target_x ,
        self.escort_target_y ,
        0),
        # data.locations[self.user_id-1].pose.pose.orientation,
        tf.transformations.quaternion_from_euler(0,0,0),
        rospy.Time.now(),
        "escortingTarget",
        "map")



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
    self.escort_target_x = float(userdata.action_goal.goal.array[1].value)
    self.escort_target_y = float(userdata.action_goal.goal.array[2].value)
    print "the behavior goal fields", [self.user_found , self.escort_target_x ,  self.escort_target_y]
    # Create SAM Reader
    # rospy.wait_for_service('create_reader')

    # reader_service = rospy.ServiceProxy('create_reader', CreateReader)
    # reader_properties = ReaderProperties(slot_name = 'People_Localization_Tracker')
    # reader_response = CreateReaderResponse()
    




    # while reader_response.success is False:
    #   reader_response = reader_service(reader_properties)
    #   r.sleep()

    # sam_reader = rospy.Subscriber(reader_response.topic_name,
    #                               PersonLocalizationTrackingDataArray,
    #                               self.people_localization_tracker_cb)



    sam_reader= SAMReader("People_Localization_Tracker",self.people_localization_tracker_cb)

    
    while not rospy.is_shutdown() and self.user_found is False:

      if self.preempt_requested():
       # sam_reader.unregister() # unregistering from people localization tracker topic
        self.service_preempt()
        return 'start_preempted'

      # sleep a bit
      r.sleep()

    userdata.tf_broadcasted = self.tf_broadcasted

	# This is important for the tf to be published continuesly so if commented the target tf is always published
    # sam_reader.unregister()
    

    return 'user_position_set'

# ---------------------------------------------------------------------- GREET state
class StateGreet(smach.State):
  """StateFinish sets the result of the action."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['start_escorting','greet_preempted'],
                          input_keys = ['action_result'],
                          output_keys = ['action_result','start_escorting_flag'])
    self.greet_start_escorting_flag=False

  def execute(self, userdata):

    # publish info to the console for the user
    rospy.loginfo('StateGreet: Executing...')
    print "Please come with me, I will escort you to the destination" #later maybe add the room  


    if self.preempt_requested():
      self.service_preempt()
      return 'greet_preempted'

    self.greet_start_escorting_flag=True
    userdata.start_escorting_flag = self.greet_start_escorting_flag
    return 'start_escorting'

# --------------------------------------------------------------------- MONITOR state
class StateMonitor(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes = ['monitor_preempted','person_lost'])
    self.listener = tf.TransformListener()
    self.threshold = 1.0

  def execute(self, userdata):

    # publish info to the console for the user
    rospy.loginfo('StateMonitor: Executing...')  
    print 'inside monitor'
    pose =geometry_msgs.msg.PoseStamped()



    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown(): 
      pose.header.stamp = rospy.Time(0) #.now()
      listener = tf.TransformListener()

      pose.pose.position.x = 0 
      pose.pose.position.y = 0
      pose.pose.position.z = 0
      pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
      pose.header.frame_id = "base_link" 
      
      rospy.sleep(0.5)

      listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(2.0))
      robot_Position = self.listener.transformPose("map", pose)
      rx = robot_Position.pose.position.x 
      ry = robot_Position.pose.position.y
      rz = robot_Position.pose.position.z

      pose.header.stamp = rospy.Time(0) #.now()

      pose.pose.position.x = 0 
      pose.pose.position.y = 0
      pose.pose.position.z = 0
      pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
      pose.header.frame_id = "person" 
      
      rospy.sleep(0.5)

      try:
        listener.waitForTransform("/map", "/person", rospy.Time(0), rospy.Duration(5.0))
      except tf.Exception:
        logger.error("Timeout while waiting for the TF transformation with the map!"
                         " Tracker can not find the person")
        return 'monitor_preempted'

      
      person_Position = self.listener.transformPose("map", pose)
      

      
      px = person_Position.pose.position.x 
      py = person_Position.pose.position.y
      pz = person_Position.pose.position.z
      
      r = numpy.array((rx ,ry, rz))
      p = numpy.array((px, py, pz))
      
      if self.preempt_requested():
        self.service_preempt()
        return 'monitor_preempted'

      print 'here---> ', distance.euclidean(r,p)
      print r , p 
      if distance.euclidean(r,p) > self.threshold : # should add heading later so that if the person is facing away then the robot tries to look for it
        # userdata.found_person=False
        print 'before exiting monitor because of person loss'
        return 'person_lost'


      rate.sleep()
      
    

# --------------------------------------------------------------------- FINISH state
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
  rospy.init_node('behavior_EscortPerson')
  rospy.loginfo('Starting up')

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
                          input_keys = ['action_goal'], 
                          output_keys = ['action_result'])

  sm.userdata.action_result = EscortPersonResult()
  result_reached = KeyValuePair()
  result_reached.key = 'reached'
  result_reached.value = 'False'
  sm.userdata.action_result.result.array.append(result_reached)
  
  with sm:

    sm.add('START',
            StateStart(),
            transitions = {'user_position_set':'MOVE',
                           'start_preempted':'preempted'},
            remapping = {'start_output':'tf_broadcasted'})

    def move_goal_callback (userdata, goal):
      if userdata.found_person == True :
        goal_ = NavigationByTargetGoal()
        goal_.target_frame = 'escortingGreetingTarget'
        goal_.heading_frame = 'escortingGreetingTarget'
        goal_.proximity = 0.0
        goal_.follow_target = False #how about constant following
        return goal_
    sm.add('MOVE',
            smach_ros.SimpleActionState('navigation_by_target',
                                         NavigationByTargetAction,
                                         goal_cb= move_goal_callback,
                                         input_keys =['found_person']),
            transitions = {'succeeded':'GREET',
                           'preempted':'preempted',
                           'aborted':'aborted'},
            remapping = {'found_person':'tf_broadcasted'})

    sm.add('GREET',
            StateGreet(),
            transitions = {'start_escorting':'CON',
                           'greet_preempted':'preempted'},
            remapping = {'greet_start_escorting_flag':'start_escorting_flag'})
    



    sm1 = smach.StateMachine(outcomes = ['preempted','escort_succeeded','aborted'],input_keys = ['start_escorting_flag'])
    with sm1:
      
      def escort_goal_callback (userdata, goal):
        if userdata.start_escorting_flag == True :
          goal_ = NavigationByTargetGoal()
          goal_.target_frame = 'escortingTarget'
          goal_.heading_frame = 'escortingTarget'
          goal_.proximity = 0.0
          goal_.follow_target = False #how about constant following
          return goal_

      sm1.add('ESCORT',
              smach_ros.SimpleActionState('navigation_by_target',
                                           NavigationByTargetAction,
                                           goal_cb= escort_goal_callback,
                                           input_keys =['start_escorting_flag']),
              transitions = {'succeeded':'escort_succeeded',
                             'preempted':'preempted',
                             'aborted':'aborted'})
      
    sm2 = smach.StateMachine(outcomes = ['preempted','person_lost'])
    with sm2:
      sm2.add('MONITOR', StateMonitor(),
                        transitions = { 'monitor_preempted':'preempted'})
   
    def termination_cb(outcome_map):
      if outcome_map['ESCORT'] == 'escort_succeeded' or outcome_map['MONITOR'] == 'person_lost':
        return True
      else:
        return False

    # def out_cb(outcome_map):
    #   if outcome_map['ESCORT'] == 'escort_finished':
    #     return True
    #   else:
    #     return False

  

    cm = smach.Concurrence(outcomes = ['default','cm_aborted','cm_escort_finished','cm_preempted','cm_retrieve_person'],
                           default_outcome='default',
                           input_keys=['start_escorting_flag'],
                           child_termination_cb = termination_cb,
                           outcome_map={'cm_escort_finished': { 'ESCORT':'escort_succeeded'},
                                         'cm_retrieve_person':{'MONITOR' : 'person_lost','ESCORT':'preempted'},
                                         'cm_preempted':{'MONITOR':'preempted'},
                                         'cm_aborted':{'ESCORT':'aborted'}
					
                                       })
    with cm:
      cm.add('ESCORT',sm1)
      cm.add('MONITOR',sm2)

    sm.add('CON',cm, transitions={'cm_escort_finished':'FINISH',
                                  'cm_retrieve_person':'MOVE',
                                  'cm_preempted':'preempted', 
                                  'cm_aborted':'aborted',
				  'default':'CON'})
    sm.add('FINISH',
            StateFinish(),
            transitions = {'result_set':'succeeded',
                           'finish_preempted':'preempted'})

  # ----------------------------------------------------------------------------------------------------------------------
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      EscortPersonAction,
                                      sm,
                                      ['user_reached'],['preempted'],['aborted'],
                                      goal_key ='action_goal',
                                      result_key ='action_result' )
  #Create and start the introspection server
  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  asw.run_server()
  rospy.spin()
  #sis.stop()
