#! /usr/bin/env python

# Behavior implemented: Moving
# Author: Jose Nuno Pereira
# Description: Moving is a behavior designed just for testing. It doesn't execute anything.
#              It prints to screen and set feedback and result messages.
#              Implemented as ActionServerWrapper for SMACH state machine.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import smach
import smach_ros

from monarch_behaviors.msg import MovingAction, MovingResult, MovingFeedback
from monarch_msgs.msg import KeyValuePair

#
#   States
#

class StateMove(smach.State):
  """StateMove makes the robot 'move', i.e., print to screen that it's moving"""
  def __init__(self):
    # 'action_feedback' and 'action_result' have to be specified both as input and output keys.
    # I believe this is because they are messages and not single field values but that's a bit of a guess.
    smach.State.__init__(self,
                          outcomes = ['location_reached','move_preempted'],
                          input_keys = ['action_goal','action_feedback','action_result'],
                          output_keys = ['action_feedback','action_result'])

  def execute(self, userdata):
    r = rospy.Rate(1)
    success = True
    count = 0

    # publish info to the console for the user
    rospy.loginfo('StateMove: Executing...')
    
    # start executing the action
    while not rospy.is_shutdown() and count < 10:

      if self.preempt_requested():
        rospy.loginfo('Preemption requested on Moving behavior. Execute some task before preempting (sleep 5 seconds).')
        rospy.sleep(5)
        self.service_preempt()
        return 'move_preempted'

      rospy.loginfo('moving to %s...', userdata.action_goal.goal.array[0].value)

      count += 1
      #userdata.action_feedback.count = count
      feedback_count = KeyValuePair()
      feedback_count.key = 'count'
      feedback_count.value = str(count)
      userdata.action_feedback.feedback.array[0] = feedback_count
      asw.publish_feedback(userdata)

      # sleep a bit
      r.sleep()
      
    if success:
      #userdata.action_result.reached = True
      #userdata.action_result.count = count
      result_reached = KeyValuePair()
      result_reached.key = 'reached'
      result_reached.value = 'True'
      userdata.action_result.result.array[0] = result_reached
      result_count = KeyValuePair()
      result_count.key = 'count'
      result_count.value = str(count)
      userdata.action_result.result.array[1] = result_count

      return 'location_reached'


#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_moving')
  rospy.loginfo('Starting up')

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['location_reached','preempted'],
                          input_keys = ['action_goal'], 
                          output_keys = ['action_result'])

  sm.userdata.action_result = MovingResult()
  result_reached = KeyValuePair()
  result_reached.key = 'reached'
  result_reached.value = 'False'
  sm.userdata.action_result.result.array.append(result_reached)
  result_count = KeyValuePair()
  result_count.key = 'count'
  result_count.value = str(0)
  sm.userdata.action_result.result.array.append(result_count)

  sm.userdata.action_feedback = MovingFeedback()
  feedback_count = KeyValuePair()
  feedback_count.key = 'count'
  feedback_count.value = str(0)
  sm.userdata.action_feedback.feedback.array.append(feedback_count)
  
  with sm:
    sm.add('MOVE',
            StateMove(),
            transitions = {'location_reached':'location_reached',
                            'move_preempted':'preempted'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      MovingAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['location_reached'],
                                      preempted_outcomes = ['preempted'],
                                      goal_key = 'action_goal', 
                                      feedback_key = 'action_feedback',
                                      result_key = 'action_result')

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()