#! /usr/bin/env python

# Behavior implemented: Patrolling and Person detection

"""
:author: Victor Gonzalez and Jose Nuno Pereira.

Same as Patrolling but adding person detection.
When person is detected , the complete behavior is stopped.
Implemented as ActionServerWrapper for SMACH state machine.

This behavior is an adaptation from behavior_patrollingandpersondetect from
Jose Nuno Pereira but with waypoints retrieved from the Parameter Server.
"""


# import roslib; roslib.load_manifest('monarch_behaviors')
import rospy
import math
import smach
import smach_ros
import rospy_utils as rpyu
from monarch_behaviors import behavior_utils as butils

from monarch_behaviors.msg import PatrollingandPersonDetectAction
from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from monarch_msgs.msg import *


#
#   States
#


class StateSetInitialState(smach.State):

    """
    Initial SMACH State.

    Called after visiting a waypoint,
    It sets the initial state of the state machine to
    next waypoint (so that after preemption and new execution patrolling
    resumes where it stopped).
    """

    def __init__(self):
        """ Init. """
        smach.State.__init__(self,
                             outcomes=['succeeded', 'preempted'],
                             input_keys=['current_state'],
                             output_keys=['current_state'])

    def execute(self, userdata):
        """ State Execution method. """
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # advance current state
        userdata.current_state = (userdata.current_state + 1) \
            if userdata.current_state + 1 < len(waypoints) else 0

        # set initial state to next waypoint
        sm1.set_initial_state(['WAYPOINT_%s' % (userdata.current_state)])
        return 'succeeded'


#
#   Main
#

if __name__ == '__main__':
    rospy.init_node('behavior_patrollingandpersondetect')
    rospy.loginfo('Starting up')

    # create waypoints for patrolling

    places = next(rpyu.load_params('places'))
    route = next(rpyu.load_params('patrolling_route'))
    waypoints = list(butils.get_route_waypoints(places, route))

    opposite_waypoints = [(x, y, (theta + math.pi) % (2 * math.pi))
                          for x, y, theta in reversed(waypoints)]
    waypoints.extend(opposite_waypoints)

    #
    #   State Machine
    #

    sm1 = smach.StateMachine(outcomes=['preempted', 'aborted'])
    sm1.userdata.current_state = 0
    with sm1:
        for (i, w) in enumerate(waypoints):
            next_waypoint = 'WAYPOINT_%s' % (
                i + 1) if i + 1 < len(waypoints) else 'WAYPOINT_0'

            sm1.add('WAYPOINT_%s' % (i),
                    smach_ros.SimpleActionState(
                        'move_base', MoveBaseAction, goal=butils.pose2alib(*w)),
                    transitions={'succeeded': 'SETINITIALSTATE_%s' % (i)})

            sm1.add('SETINITIALSTATE_%s' % (i),
                    StateSetInitialState(),
                    transitions={'succeeded': next_waypoint})

    def interacting_cb(userdata, msg):
        """ Callback fired when user is detected. """
        print 'msg received = ' + str(msg.data)
        return not msg.data

    sm2 = smach.StateMachine(outcomes=['interacting', 'preempted'])
    with sm2:
        sm2.add('CHECK_INTERACTION',
                smach_ros.MonitorState('interacting',
                                       Bool,
                                       interacting_cb),
                transitions={'valid': 'CHECK_INTERACTION',
                             'invalid': 'interacting'})

    def termination_cb(outcome_map):
        """ Callback fired when termination is requested. """
        if outcome_map['DO_CHECK_INTERACTION'] == 'interacting':
            return True
        if outcome_map['PATROL'] == 'aborted':
            return True
        else:
            return False

    def out_cb(outcome_map):
        """ Callback fired when ."""
        if outcome_map['DO_CHECK_INTERACTION'] == 'preempted':
            return 'preempted'
        if outcome_map['PATROL'] == 'aborted':
            return 'aborted'
        else:
            return 'interacting'

    cm = smach.Concurrence(outcomes=['interacting', 'preempted', 'aborted'],
                           default_outcome='interacting',
                           child_termination_cb=termination_cb,
                           outcome_cb=out_cb)
    with cm:
        cm.add('PATROL', sm1)
        cm.add('DO_CHECK_INTERACTION', sm2)

    def not_interacting_cb(userdata, msg):
        print 'msg received = ' + str(msg.data)
        return msg.data

    mm = smach.StateMachine(outcomes=['interacting', 'preempted', 'aborted'])
    with mm:
        mm.add('PATROLANDINTERACTION',
               cm,
               transitions={'interacting': 'CHECK_NO_INTERACTION'})

        mm.add('CHECK_NO_INTERACTION',
               smach_ros.MonitorState('interacting',
                                      Bool,
                                      not_interacting_cb),
               transitions={'valid': 'CHECK_NO_INTERACTION',
                            'invalid': 'PATROLANDINTERACTION'})

    #
    #   Action Server Wrapper
    #

    # action_{goal,feedback,result} are the default values and don't need to
    # be passed as input
    action_name = rospy.get_name()
    asw = smach_ros.ActionServerWrapper(action_name,
                                        PatrollingandPersonDetectAction,
                                        wrapped_container=mm,
                                        preempted_outcomes=['preempted'],
                                        aborted_outcomes=['aborted'])

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', mm, '/SM_ROOT')
    sis.start()

    asw.run_server()

    rospy.spin()

    sis.stop()
