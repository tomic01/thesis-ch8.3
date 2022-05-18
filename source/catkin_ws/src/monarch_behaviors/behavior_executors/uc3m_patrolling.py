#! /usr/bin/env python

# Behavior implemented: Patrolling
# Author: Victor Gonzalez Pacheco, Jose Carlos castillo and Jose Nuno Pereira

"""
Patrolling Behavior adapted for UC3M.

Patrolling is a behavior designed for visiting
a sequence of predetermined locations.
It receives a set of locations and creates a SMACH state machine
in which each state moves the robot to the next location.
All locations are visited by the order they are received.
Upon reaching the final location it inverts the orientation of the each
location and proceeds to visit each location in the opposite direction.
When all locations are visited again it returns to the initial state.
Implemented as ActionServerWrapper for SMACH state machine.
"""


# import roslib
# roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros

import rospy_utils as rpyu

from monarch_behaviors.msg import PatrollingAction

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from monarch_behaviors import behavior_utils as butils


class StateSetInitialState(smach.State):

    """
    Sets initial State.

    Called after visiting a waypoint, sets the initial state of the
    state machine to next waypoint (so that after preemption and new
    execution patrolling resumes where it stopped).
    """

    def __init__(self):
        """ Class ctor. """
        smach.State.__init__(self,
                             outcomes=['succeeded', 'preempted'],
                             input_keys=['current_state'],
                             output_keys=['current_state'])

    def execute(self, userdata):
        """ Execute method of the State Machine. """
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # advance current state
        userdata.current_state = (userdata.current_state + 1) \
            if userdata.current_state + 1 < len(waypoints) else 0

        # set initial state to next waypoint
        sm.set_initial_state(['WAYPOINT_%s' % (userdata.current_state)])
        return 'succeeded'


#
#   Main
#

if __name__ == '__main__':
    rospy.init_node('behavior_patrolling')
    rospy.loginfo('Starting up')

    places = next(rpyu.load_params('places'))
    route = next(rpyu.load_params('patrolling_route'))
    waypoints = list(butils.get_route_waypoints(places, route))

    opposite_waypoints = [(x, y, (theta + math.pi) % (2 * math.pi))
                          for x, y, theta in reversed(waypoints)]
    # opposite_waypoints.reverse()
    waypoints.extend(opposite_waypoints)

    #
    #   State Machine
    #

    sm = smach.StateMachine(outcomes=['preempted', 'aborted'])
    sm.userdata.current_state = 0

    with sm:
        for (i, wp) in enumerate(waypoints):
            next_waypoint = 'WAYPOINT_{}'.format(i + 1) \
                if i + 1 < len(waypoints) else 'WAYPOINT_0'

            sm.add('WAYPOINT_{}'.format(i),
                   smach_ros.SimpleActionState('move_base', MoveBaseAction,
                                               goal=butils.pose2alib(*wp)),
                   transitions={'succeeded': 'SETINITIALSTATE_{}'.format(i)})

            sm.add('SETINITIALSTATE_{}'.format(i),
                   StateSetInitialState(),
                   transitions={'succeeded': next_waypoint})

    #
    #   Action Server Wrapper
    #

    # action_{goal,feedback,result} are the default values and don't need to
    # be passed as input
    action_name = rospy.get_name()
    asw = smach_ros.ActionServerWrapper(action_name,
                                        PatrollingAction,
                                        wrapped_container=sm,
                                        preempted_outcomes=['preempted'],
                                        aborted_outcomes=['aborted'])

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    asw.run_server()

    rospy.spin()

    sis.stop()
