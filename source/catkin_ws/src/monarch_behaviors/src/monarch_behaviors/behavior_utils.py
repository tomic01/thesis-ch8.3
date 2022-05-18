#! /usr/bin/env python

# Behavior implemented: Patrolling
# Author: Victor Gonzalez Pacheco

""" Common utilities for MOnarCH Behaviors ROS Package. """


# import roslib
# roslib.load_manifest('monarch_behaviors')
import math

from move_base_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *


def unpack_waypoint(waypoint):
    """ Return a dict-based waypoint as a tuple (x,y,theta). """
    return (waypoint['x'], waypoint['y'], waypoint['theta'])


def get_route_waypoints(places, route):
    """
    Generate the route waypoints.

    :return: waypoints in (x,y, theta) format.
    """
    return (unpack_waypoint(places[waypoint]) for waypoint in route)


def pose2alib(x, y, theta):
    """
    Convert a pose to an actionlib message.

    :return: The point as a move_base_msgs/MoveBaseGoal
    """
    p = Point(x, y, 0)
    q = Quaternion(0, 0, math.sin(theta / 2.0), math.cos(theta / 2.0))
    goal = PoseStamped(header=Header(frame_id="/map"),
                       pose=Pose(position=p, orientation=q))
    return MoveBaseGoal(target_pose=goal)
