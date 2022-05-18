#! /usr/bin/env python

# Behavior implemented: Idle
# Author: Jose Nuno Pereira
# Description: Idle is a behavior designed just for testing. It doesn't execute anything.
#              It prints to screen.
#              Implemented as ActionServerWrapper for SMACH state machine.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import smach
import smach_ros

from monarch_behaviors.msg import IdleAction

#
#   States
#

class StateIdle(smach.State):
  """StateIdle makes the robot 'idle', i.e., not executing any other behavior and print to screen that it's idling"""
  def __init__(self):
    smach.State.__init__(self, outcomes = ['idle_preempted'])

  def execute(self, userdata):
    r = rospy.Rate(1)

    # publish info to the console for the user
    rospy.loginfo('StateIdle: Executing...')
    
    # start executing the action
    while not rospy.is_shutdown():

      if self.preempt_requested():
        self.service_preempt()
        return 'idle_preempted'

      rospy.loginfo('idling...')

      # sleep a bit
      r.sleep()
      


#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_idle')
  rospy.loginfo('Starting up')

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['preempted'])
  
  with sm:
    sm.add('IDLE',
            StateIdle(),
            transitions = {'idle_preempted':'preempted'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      IdleAction,
                                      wrapped_container = sm,
                                      preempted_outcomes = ['preempted'])

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()