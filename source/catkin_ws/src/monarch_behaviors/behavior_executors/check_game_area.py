#! /usr/bin/env python

# Behavior implemented: Check game area
# Author: Jose Nuno Pereira
# Description: Verifies if the game area is clear of obstacles

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import smach
import smach_ros

from monarch_behaviors.msg import CheckGameAreaClearAction, CheckGameAreaClearResult
from monarch_behaviors.msg import CheckAreaClearAction, CheckAreaClearGoal
from monarch_msgs.msg import KeyValuePair
from geometry_msgs.msg import Point

class StatePrepareGoal(smach.State):
  """Prepares the goal message, including the corners points of the game area and the nav points
  to be visited by the robot, to be sent to the area checker actionlib client."""
  def __init__(self):
    smach.State.__init__(self,
                         outcomes = ['succeeded','preempted'],
                         input_keys = ['area_checker_goal','action_result'],
                         output_keys = ['area_checker_goal','action_result'])
  
  def execute(self,userdata):
    if self.preempt_requested():
      self.service_preempt()
      return 'preempted'

    userdata.area_checker_goal.area_points = []
    userdata.area_checker_goal.area_points.append(Point(x = -16.87, y = 0.92))
    userdata.area_checker_goal.area_points.append(Point(x = -20.32, y = 0.92))
    userdata.area_checker_goal.area_points.append(Point(x = -20.32, y = -2.52))
    userdata.area_checker_goal.area_points.append(Point(x = -16.87, y = -2.52))

    max_x = max([p.x for p in userdata.area_checker_goal.area_points])
    min_x = min([p.x for p in userdata.area_checker_goal.area_points])
    max_y = max([p.y for p in userdata.area_checker_goal.area_points])
    min_y = min([p.y for p in userdata.area_checker_goal.area_points])

    x1 = max_x - (max_x - min_x)/4
    x2 = min_x + (max_x - min_x)/4
    y1 = max_y - (max_y - min_y)/4
    y2 = min_y + (max_y - min_y)/4

    userdata.area_checker_goal.nav_points = []
    userdata.area_checker_goal.nav_points.append(Point(x = x1, y = y1))
    userdata.area_checker_goal.nav_points.append(Point(x = x1, y = y2))
    userdata.area_checker_goal.nav_points.append(Point(x = x2, y = y2))
    userdata.area_checker_goal.nav_points.append(Point(x = x2, y = y1))
    userdata.area_checker_goal.nav_points.append(Point(x = min_x + (max_x - min_x)/2, y = min_y + (max_y - min_y)/2))

    userdata.action_result.result.array[0].value = 'False'

    return 'succeeded'

class StateFinish(smach.State):
  """StateFinish sets the result of the action."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['succeeded','preempted'],
                          input_keys = ['area_empty','action_result'],
                          output_keys = ['action_result'])

  def execute(self, userdata):

    if self.preempt_requested():
      self.service_preempt()
      return 'preempted'

    if userdata.area_empty:
      userdata.action_result.result.array[0].value = 'True'
    return 'succeeded'

#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_checkgamearea')
  rospy.loginfo('Starting up')

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'],
                          output_keys = ['action_result'])


  sm.userdata.area_checker_goal = CheckAreaClearGoal()
  sm.userdata.action_result = CheckGameAreaClearResult()
  result_reached = KeyValuePair()
  result_reached.key = 'area_empty'
  result_reached.value = 'False'
  sm.userdata.action_result.result.array.append(result_reached)

  with sm:
    sm.add('PREPARE_GOAL',
           StatePrepareGoal(),
           transitions = {'succeeded':'AREA_CHECKER'})

    sm.add('AREA_CHECKER',
           smach_ros.SimpleActionState('area_checker',
                                       CheckAreaClearAction,
                                       goal = sm.userdata.area_checker_goal,
                                       result_slots = ['area_empty']),
           transitions = {'succeeded':'SET_RESULT'})

    sm.add('SET_RESULT',
           StateFinish())
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      CheckGameAreaClearAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['succeeded'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'],
                                      result_key = 'action_result')

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()
