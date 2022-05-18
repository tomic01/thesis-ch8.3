#! /usr/bin/env python

"""
TA Quiz Game Behavior.

author: Victor Gonzalez

This behavior handles with the TA Quiz Game State machine.
"""

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy

# from monarch_msgs_utils import key_value_pairs as kvpa

# import smach
import smach_ros

from uc3m_behaviors import machines

from monarch_behaviors.msg import TAQuizAction


def main():
    rospy.init_node('behavior_ta_quiz_game')

    # State Machines
    sm_ta_quiz_game = machines.make_ta_quiz_sm()

    action_name = rospy.get_name()
    asw = smach_ros.ActionServerWrapper(action_name,
                                        TAQuizAction,
                                        wrapped_container=sm_ta_quiz_game,
                                        preempted_outcomes=['preempted'],
                                        succeeded_outcomes=['succeeded'])

    sis = smach_ros.IntrospectionServer('sm_ta_quiz_game', sm_ta_quiz_game,
                                        '/SM_ROOT/SM_TA_QUIZ_GAME')
    sis.start()
    asw.run_server()
    rospy.spin()


if __name__ == '__main__':
    main()
