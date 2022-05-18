#! /usr/bin/env python

import re

import roslib; roslib.load_manifest('monarch_behaviors')
import rospy

from rosapi import proxy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import monarch_behaviors.msg

def behavior_client():

    selectedBehavior = ''

    if(len(selectedBehavior) == 0):

      # get list of topics
      allTopics = proxy.get_topics()

      print 'Available behaviors:'

      allBehaviors = []

      for topic in allTopics:
	if(re.match('/behavior.*/status', topic, 0)):
	  behavior = topic.replace('/behavior_', '').replace('/status', '')
	  allBehaviors.append(behavior)


      for behavior in allBehaviors:
	print '\t- ' + behavior

      bname = raw_input('Which behavior do you want to execute? ')

      if bname in allBehaviors:
        selectedBehavior = bname
	print 'Selected: ' + bname
      else:
        print 'Could not find behavior with name \'' + bname + '\''
        exit(-1)

    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.

    rospy.loginfo('creating simple client')
    client = actionlib.SimpleActionClient('behavior_'+selectedBehavior, monarch_behaviors.msg.IdleAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo('waiting for server')
    client.wait_for_server()

    # Creates a goal to send to the action server.
    rospy.loginfo('creating goal')
    goal = monarch_behaviors.msg.IdleGoal()

    # Sends the goal to the action server.
    rospy.loginfo('sending goal')
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    rospy.loginfo('waiting for result')
    client.wait_for_result()

    # Prints out the result of executing the action
    rospy.loginfo('got result')
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('manual_bahavior_runner')
        result = behavior_client()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
