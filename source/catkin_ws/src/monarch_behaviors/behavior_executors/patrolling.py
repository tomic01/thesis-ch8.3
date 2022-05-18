#! /usr/bin/env python

# Behavior implemented: Patrolling
# Author: Jose Nuno Pereira
# Description: Patrolling is a behavior designed for visiting a sequence of predetermined locations.
#              It receives a set of locations and creates a SMACH state machine in which each state
#              moves the robot to the next location. All locations are visited by the order they are
#              received. Upon reaching the final location it inverts the orientation of the each
#              location and proceeds to visit each location in the opposite direction. When all
#              locations are visited again it returns to the initial state.
#              Implemented as ActionServerWrapper for SMACH state machine.

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros

from monarch_behaviors.msg import PatrollingAction

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

def pose2alib(x, y, t):
  goal = PoseStamped(header=Header(frame_id="/map"),
                      pose=Pose(position=Point(x, y, 0),
                        orientation=Quaternion(0, 0, math.sin(t/2.0), math.cos(t/2.0))))
  return MoveBaseGoal(target_pose=goal)

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
    sm.set_initial_state(['WAYPOINT_%s'%(userdata.current_state)])
    return 'succeeded'


#
#   Main
#
    
if __name__ == '__main__':
  rospy.init_node('behavior_patrolling')
  rospy.loginfo('Starting up')

  #create waypoints for patrolling

  # IW 8th floor 2 "rooms"
  #waypoints = [(-11.35,-13.90,1.17),
  #              (-5.10,-4.40,0.47),
  #              (5.30,-8.05,1.16),
  #              (-1.35,-18.40,0.71)]

  # IW 8th floor isr8-v07cr_new_gamearea.yaml
  waypoints = [(4.05,10.65,1.11),
               (0.35,5.15,-1.32),
               (2.15,-1.60,-1.86),
               (8.00,-3.90,-3.14),
               (1.00,6.25,1.08),
               (12.40,11.30,-0.30),
               (19.05,8.50,-1.57),
               (12.30,0.70,-0.00)]

  # IW 8th floor 1 "room"
  #waypoints = [(-9.5,-11.6,-2.57),
                #(-11.3, -14.3, -2.1),
                #(-8.90,-17.10,-1.18)]

  # EPFL Robotics Arena
  # waypoints = [(7.2,12.8,1.78),
  #               (7.0,10.8,-0.1),
  #               (6.05,8.7,-1.74)]

  # Webots IPOL environment (world file: envIPOL_half_corridor_3rooms.wbt (23/07/2014))
  #waypoints = [(6.35,-3.35,0.00),
               #(-6.80,0.65,3.08),
               #(-4.40,-3.10,-1.63),
               #(-13.10,-2.00,2.86),
               #(-20.20,-2.75,1.53),
               #(-19.75,1.95,-0.04),
               #(-6.40,2.55,-0.98),
               #(-7.05,-2.50,-1.57)]


  #opposite_waypoints = [(l[0],l[1],(l[2]+math.pi)%(2*math.pi)) for l in waypoints]
  opposite_waypoints = [(l[0],l[1],round((l[2]+3.14)%(2*3.14),2)) for l in waypoints[1:(len(waypoints)-1)]]
  opposite_waypoints.reverse()
  waypoints.extend(opposite_waypoints)

  #
  #   State Machine
  #
      
  sm = smach.StateMachine(outcomes = ['preempted','aborted'])
  sm.userdata.current_state = 0

  with sm:
    for (i,w) in enumerate(waypoints):
      next_waypoint = 'WAYPOINT_%s'%(i+1) if i+1<len(waypoints) else 'WAYPOINT_0'

      sm.add('WAYPOINT_%s'%(i),
              smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=pose2alib(*w)),
              transitions = {'succeeded': 'SETINITIALSTATE_%s'%(i)})

      sm.add('SETINITIALSTATE_%s'%(i),
              StateSetInitialState(),
              transitions = {'succeeded':next_waypoint})
  
  #
  #   Action Server Wrapper
  #

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
  action_name = rospy.get_name()  
  asw = smach_ros.ActionServerWrapper(action_name,
                                      PatrollingAction,
                                      wrapped_container = sm,
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'])

  # Create and start the introspection server
  #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  #sis.start()

  asw.run_server()

  rospy.spin()

  #sis.stop()
