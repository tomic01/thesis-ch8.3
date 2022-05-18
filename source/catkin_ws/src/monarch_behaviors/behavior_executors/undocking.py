#! /usr/bin/env python

# Behavior implemented: Undock
# Author: Jose Nuno Pereira
# Description: Undock is a behavior designed for disconnecting the robot from the
#              charger and leave it ready for operation. The first state resets the
#              action result. The second state publishes to the appropriate topics
#              to disconnect the robot from power and after that teleoperation is
#              used to move the robot out of the charger.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros

from monarch_behaviors.msg import UndockAction, UndockResult
from monarch_msgs.msg import KeyValuePair
from monarch_msgs.msg import SetStateAuxiliaryPowerBattery, SetStateElectronicPower, SetStateMotorsPower

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
    userdata.action_result.result.array[0].value = 'True'
    return 'result_set'

class StateUndock(smach.State):
  """StateUndock disconnects the robot from charger power and moves it forward."""
  def __init__(self):
    smach.State.__init__(self,
                          outcomes = ['completed'],
                          input_keys = ['action_result'],
                          output_keys = ['action_result'])

    self.teleop_srv = rospy.ServiceProxy("scout/teleop", TeleOpSrv)
    self.undock_pubs = [rospy.Publisher("set_state_aux_batt1_power", SetStateAuxiliaryPowerBattery, latch=True),
                        rospy.Publisher("set_state_aux_batt2_power", SetStateAuxiliaryPowerBattery, latch=True),
                        rospy.Publisher("set_state_electronics_power", SetStateElectronicPower, latch=True),
                        rospy.Publisher("set_state_motors_power", SetStateMotorsPower, latch=True)]

  def disconnect_from_power(self):
    RELAY_DELAY = 0.25
    (aux1, aux2, elec, moto) = self.undock_pubs
    rospy.sleep(RELAY_DELAY)
    aux1.publish(SetStateAuxiliaryPowerBattery(status=2))
    rospy.sleep(RELAY_DELAY)
    aux2.publish(SetStateAuxiliaryPowerBattery(status=2))
    rospy.sleep(RELAY_DELAY)
    elec.publish(SetStateElectronicPower(status=2))
    rospy.sleep(RELAY_DELAY)
    moto.publish(SetStateMotorsPower(status=2))
    rospy.sleep(RELAY_DELAY)

  def execute(self, userdata):
    self.disconnect_from_power()
    # args: (cmd, vel, tout), where cmd=1=front/back, vel=-0.2 m/s, tout=0=none
    self.teleop_srv(1, 0.2, 0)
    rospy.sleep(2.0)
    # args: cmd=0=stop
    self.teleop_srv(0, 0, 0)

    userdata.action_result.result.array[0].value = 'False'
    return 'completed'

#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_undock')
  rospy.loginfo('Starting up')

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['undocked'], 
                          output_keys = ['action_result'])

  sm.userdata.action_result = UndockResult()
  result_docked = KeyValuePair()
  result_docked.key = 'docked'
  result_docked.value = 'True'
  sm.userdata.action_result.result.array.append(result_docked)
  
  with sm:
    sm.add('START',
            StateStart(),
            transitions = {'result_set':'UNDOCK'})

    sm.add('UNDOCK',
            StateUndock(),
            transitions = {'completed':'undocked'})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      UndockAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['undocked'])

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()