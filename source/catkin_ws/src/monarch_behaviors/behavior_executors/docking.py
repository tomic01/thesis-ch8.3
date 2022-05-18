#! /usr/bin/env python

# Behavior implemented: Dock
# Author: Jose Nuno Pereira
# Description: Dock is a behavior designed for moving the robot to the charger location
#              and dock it. The first state resets the action result. The second state
#              moves the robot to the charger. The third state uses the teleoperation
#              server to move the robot back during a fixed period of time and thus
#              secure it to the dock.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros
from monarch_behaviors import behavior_utils as bu
import rospy_utils as rpyu

from monarch_behaviors.msg import DockAction, DockResult
from monarch_msgs.msg import KeyValuePair

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

import sys
import os.path
import rospkg
rospack = rospkg.RosPack()
sys.path.append(os.path.join(rospack.get_path("scout_msgs"), "src"))
from scout_msgs.msg import *
from scout_msgs.srv import *

# Docking station coordinates made up!! TUNE THESE VALUES BEFORE TESTING
# ISR Testbed
#DOCK_X = -11.678
#DOCK_Y = -10.494
#DOCK_T = -1.137

# Webots Simulation - Environment: envIPOL_half_corridor_3rooms
#DOCK_X = -21.05
#DOCK_Y = -3.15
#DOCK_T = -0.00

# # EPFL Robotics lab
# DOCK_X = 7.52
# DOCK_Y = 14.14
# DOCK_T = -1.70

# ISR isr8-v07cr_new_gamearea.yaml
DOCK_X = -6.99
DOCK_Y = 8.58
DOCK_T = -1.03
     
# roboticslab_complete_clean_doors_closed_places.yaml
# DOCK_X = 14.23
# DOCK_Y = 33.95
# DOCK_T = 2.93

#
#   States
#

class StateStart(smach.State):
  """StateStart resets the action_result."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['result_set',],
                          input_keys = ['action_result'],
                          output_keys = ['action_result'])

  def execute(self, userdata):
    userdata.action_result.result.array[0].value = 'False'
    return 'result_set'

class StateDock(smach.State):
  """StateDock moves the robot back for 5 seconds to secure it to the charger."""
  def __init__(self):
      smach.State.__init__(self,
                           outcomes = ['completed'],
                           input_keys = ['action_result'],
                           output_keys = ['action_result'])

      self.teleop_srv = rospy.ServiceProxy("scout/teleop", TeleOpSrv)

  def execute(self, userdata):
      # args: (cmd, vel, tout), where cmd=1=front/back, vel=-0.2 m/s, tout=0=none
      self.teleop_srv(1, -0.2, 0)
      rospy.sleep(5)
      # args: cmd=0=stop
      self.teleop_srv(0, 0, 0)

      userdata.action_result.result.array[0].value = 'True'
      return 'completed'

#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_dock')
  rospy.loginfo('Starting up')

  dock = None
  try:
    places = next(rpyu.load_params('places'))
    dock = bu.unpack_waypoint(places['charger'])
    rospy.logdebug("Loaded docking point: {}".format(dock))
  except (rpyu.ParamNotFoundError, KeyError):
    rospy.logwarn('Docking position not found in param server. '
                  'Using hardcoded parameters: {}'
                  .format((DOCK_X, DOCK_Y, DOCK_T)))
    dock = (DOCK_X, DOCK_Y, DOCK_T)

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['docked','preempted','aborted'], 
                          output_keys = ['action_result'])

  sm.userdata.action_result = DockResult()
  result_docked = KeyValuePair()
  result_docked.key = 'docked'
  result_docked.value = 'False'
  sm.userdata.action_result.result.array.append(result_docked)
  
  with sm:
    sm.add('START',
            StateStart(),
            transitions = {'result_set':'MOVE2DOCK'})

    sm.add('MOVE2DOCK',
            smach_ros.SimpleActionState('move_base',
                                         MoveBaseAction,
                                         goal=bu.pose2alib(*dock)),
            transitions = {'succeeded':'DOCK',
                           'preempted':'preempted',
                           'aborted':'aborted'})

    sm.add('DOCK',
            StateDock(),
            transitions = {'completed':'docked'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      DockAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['docked'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'])

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()
