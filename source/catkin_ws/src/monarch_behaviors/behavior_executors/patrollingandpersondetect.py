#! /usr/bin/env python

# Behavior implemented: Patrolling and Person detection
# Author: Jose Nuno Pereira
# Description: Same as Patrolling but adding person detection. When person is
#              detected , the complete behavior is stopped.
#              Implemented as ActionServerWrapper for SMACH state machine.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros

from monarch_behaviors.msg import PatrollingandPersonDetectAction

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from monarch_msgs.msg import *

from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction

def pose2alib(x, y, t):
  goal = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return MoveBaseGoal(target_pose=goal)

#
#   States
#

class GiveGreetingChildState(MonarchState):
  
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("GiveGreetingChildState")
    runGreetingCaState = RunCaAction(self, 'give_greetings_child')
    self.add_action(runGreetingCaState)


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
    sm1.set_initial_state(['WAYPOINT_%s'%(userdata.current_state)])
    return 'succeeded'


#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_patrollingandpersondetect')
  rospy.loginfo('Starting up')

  #create waypoints for patrolling

  # IW 8th floor 2 "rooms"
  #waypoints = [(-11.35,-13.90,1.17),
  #              (-5.10,-4.40,0.47),
  #              (5.30,-8.05,1.16),
  #              (-1.35,-18.40,0.71) ]

  # IW 8th floor 1 "room"
  #waypoints = [(-9.5,-11.6,-2.57),
                #(-11.3, -14.3, -2.1),
                #(-8.90,-17.10,-1.18)]

  # EPFL Robotics Arena
  #waypoints = [(7.2,12.8,1.78),
                #(7.0,10.8,-0.1),
                #(6.05,8.7,-1.74)]

  # Webots IPOL environment (world file: envIPOL_half_corridor_3rooms.wbt (23/07/2014))
  # waypoints = [(12.60,-3.50,-1.57),
  #        (6.35,-3.35,0.00),
  #        (-1.35,-3.25,0.00),
  #        (-4.40,-3.10,-1.63),
  #        (-13.10,-2.00,2.86),
  #        (-20.20,-2.75,1.53),
  #        (-19.75,1.95,-0.04),
  #        (-6.40,2.55,-0.98),
  #        (-7.05,-2.50,-1.57)]

  #EPFL extended map - waypoints not up to date
  #waypoints = [(6.80,-5.10,1.62),
               #(4.90,1.80,1.90),
               #(-0.10,2.15,-2.21),
               #(-7.50,2.40,-1.61)]

  #IPOL (ipol-2r.pgm)
  #waypoints = [(-2.85,-8.85,0.27),
                #(-2.10,-3.75,1.64),
                #(1.85,0.30,1.65),
                #(0.80,10.85,1.69),
                #(0.35,17.05,1.67)]

  # IW 8th floor isr8-v07cr_new_gamearea.yaml
  waypoints = [(4.65,11.85,-1.99),
               (0.35,5.15,-1.32),
               (2.15,-1.60,-1.86),
               (8.00,-3.90,-3.14),
               (12.40,11.30,-0.30),
               (19.05,8.50,-1.57),
               (12.30,0.70,-0.00)]

  opposite_waypoints = [(l[0],l[1],(l[2]+math.pi)%(2*math.pi)) for l in waypoints]
  opposite_waypoints.reverse()
  waypoints.extend(opposite_waypoints)

  #
  #   State Machine
  #
      
  sm1 = smach.StateMachine(outcomes = ['preempted','aborted'])
  sm1.userdata.current_state = 0
  with sm1:

    sm1.add('ACTIVATE_GREETINGS_CA',
            GiveGreetingChildState(),
            transitions = {'succeeded':'WAYPOINT_0'})

    for (i,w) in enumerate(waypoints):
      next_waypoint = 'WAYPOINT_%s'%(i+1) if i+1<len(waypoints) else 'WAYPOINT_0'

      sm1.add('WAYPOINT_%s'%(i),
              smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=pose2alib(*w)),
              transitions = {'succeeded': 'SETINITIALSTATE_%s'%(i)})

      sm1.add('SETINITIALSTATE_%s'%(i),
              StateSetInitialState(),
              transitions = {'succeeded':next_waypoint})

  def interacting_cb(userdata,msg):
    print 'msg received = '+str(msg.data)
    return not msg.data
  
  sm2 = smach.StateMachine(outcomes = ['interacting','preempted'])
  with sm2:
    sm2.add('CHECK_INTERACTION',
            smach_ros.MonitorState('interacting',
                                   Bool,
                                   interacting_cb),
            transitions = {'valid':'CHECK_INTERACTION',
                           'invalid':'interacting'})

  def termination_cb(outcome_map):
    if outcome_map['DO_CHECK_INTERACTION'] == 'interacting':
      return True
    if outcome_map['PATROL'] == 'aborted':
      return True
    else:
      return False

  def out_cb(outcome_map):
    if outcome_map['DO_CHECK_INTERACTION'] == 'preempted':
      return 'preempted'
    if outcome_map['PATROL'] == 'aborted':
      return 'aborted'
    else:
      return 'interacting'

  cm = smach.Concurrence(outcomes = ['interacting','preempted','aborted'],
                         default_outcome = 'interacting',
                         child_termination_cb = termination_cb,
                         outcome_cb = out_cb)
  with cm:
    cm.add('PATROL',sm1)
    cm.add('DO_CHECK_INTERACTION',sm2)

  def not_interacting_cb(userdata,msg):
    print 'msg received = '+str(msg.data)
    return msg.data

  mm = smach.StateMachine(outcomes = ['interacting','preempted','aborted'])
  with mm:
    mm.add('PATROLANDINTERACTION',
           cm,
           transitions = {'interacting':'CHECK_NO_INTERACTION'})

    mm.add('CHECK_NO_INTERACTION',
            smach_ros.MonitorState('interacting',
                                   Bool,
                                   not_interacting_cb),
            transitions = {'valid':'CHECK_NO_INTERACTION',
                           'invalid':'PATROLANDINTERACTION'})

  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      PatrollingandPersonDetectAction,
                                      wrapped_container = mm,
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'])

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', mm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()
