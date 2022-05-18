#! /usr/bin/env python
"""
:author: Victor Gonzalez
:maintainer: Victor Gonzalez (vgonzale@ing.uc3m.es)
:description: The Interaction Behavior Executor (IBE) for MOnarCH HRI.
              It's an ActionLib-based wrapper/connector between the
              Behavior Manager and the Interaction Manager.
"""
# import roslib
# roslib.load_manifest('monarch_behaviors')
import rospy

from actionlib import SimpleActionServer

from monarch_msgs_utils import key_value_pairs as kvpa
from std_msgs.msg import String
# from monarch_msgs.msg import (KeyValuePair, KeyValuePairArray)
from monarch_behaviors.msg import InteractionBehaviorAction
# from monarch_behaviors.msg import InteractionBehaviorGoal
from monarch_behaviors.msg import InteractionBehaviorFeedback
from monarch_behaviors.msg import InteractionBehaviorResult
from dialog_manager_msgs.msg import (AtomMsg, VarSlot, ActionMsg)

# from monarch_msgs.msg import KeyValuePair


def _to_str(activations):
    """ Receives a list of strings and returns this list sorted in a string
        where each element of the list is separated by '|' """
    return '|'.join(activations)


def _str_to_bool(s):
    """ Converts the strings 'true' and 'false' to its corresponding bool values
    """
    return s.lower() == 'true'


def generate_slots(activations):
    """ Yields the slots of the atom"""
    activated_str = _to_str(activations).lower()
    yield VarSlot(name="type", val="ca_activations")
    yield VarSlot(name="subtype", val="user")
    yield VarSlot(name="activated_cas", val=activated_str, type="string")
    yield VarSlot(name="timestamp", val="_NO_VALUE_", type="number")
    yield VarSlot(name="consumed", val="false", type="string")


def produce_atom_msg(activations):
    """ Produces an L{AtomMsg} from a list of CAs activations """
    varslots = generate_slots(activations)
    return AtomMsg(varslots=list(varslots))


# def kvpa_to_dict(kvpa):
#     """ Returns a dict built from a L{KeyValuePairArray} message """
#     return {pair.key: pair.value for pair in kvpa.array}


# def kvpa_from_dict(d):
#     """ Returns a L{KeyValuePairArray} from a dict """
#     kvpairs = (KeyValuePair(key=k, value=v) for k, v in d.iteritems())
#     return KeyValuePairArray(array=list(kvpairs))


class InteractionBehaviorExecutorServer(object):
    """ The main server for InteractionBehaviorExecutorServer """

    def __init__(self, name):
        rospy.loginfo('init...')
        self._action_name = name
        self._as = SimpleActionServer(self._action_name,
                                      InteractionBehaviorAction,
                                      execute_cb=self.execute_cb,
                                      auto_start=False)

        # Subscribers
        rospy.Subscriber("im_action", ActionMsg, self.im_response_cb)
        # Publishers
        self.publisher = rospy.Publisher('im_atom', AtomMsg)
        self.allowed_pub = rospy.Publisher('allowed_hri_interfaces', String)

        # Start the Interaction Behavior Executor
        self._as.start()

    def execute_cb(self, action):
        """ Executes the Interaction Behaviors """
        self.preempted = False
        self.aborted = False
        self.user_engaged = False
        self.user_responded = True
        self.is_interaction_ended = False

        # publish info to the console for the user
        rospy.loginfo('{}: Executing...'.format(self._action_name))

        goal_dict = kvpa.kvpa_to_dict(action.goal)
        rospy.loginfo("Goal received: {}".format(goal_dict))

        # Publish the list of Allowed HRI Interfaces
        allowed_ifaces = goal_dict['allowed_hri_interfaces']
        self.allowed_pub.publish(allowed_ifaces)

        ca = goal_dict['communicative_act']
        self.send_activations_to_im([ca, ])  # Send goals to InteractionManager
        self.process_im_responses(goal_dict)  # Start executing the action

        # Send result to Behavior Manager if succeeded
        if self.is_behavior_succeded():
            rospy.loginfo("Interaction Execution was sucessful.")
            self.send_result_to_bm()

    def process_im_responses(self, goal):
        """ Loop that receives responses from Interaction Manager and
            sends them to the Behavior Manager.
            The loop ends if the action server receives one of the follwoing:
                - A preemption request from the Behavior Manager
                - An interaction_result message from the Interaction Manager
        """
        r = rospy.Rate(1)
        while all([not rospy.is_shutdown(), not self.is_interaction_ended]):
            rospy.loginfo('Interacting with user. CA: {}'
                          .format(goal['communicative_act']))
            if self._as.is_preempt_requested():
                rospy.loginfo('{}: Preempted'.format(self._action_name))
                self._as.set_preempted()
                self.preempted = True
                self.stop_interacting_with_user()
                break

            # Send feedback to Behavior manager periodically:
            rospy.loginfo("Sending Feedback to BM. " +
                          "User Engaged: {}".format(self.user_engaged))
            self.send_feedback_to_bm(self.user_engaged)
            # sleep a bit
            r.sleep()

    def send_activations_to_im(self, activations, parameters=None):
        """ Sends the goal to the Interaction Manager in a form of
            a list of Communicative Acts Activations"""
        atom = produce_atom_msg(activations)
        self.publisher.publish(atom)
        rospy.logdebug("Published activation to IM:\n{}".format(atom))

    def stop_interacting_with_user(self):
        """ Activates the stop_interaction protocol in the Interaction Manager
        """
        self.send_activations_to_im(['stop_interacting'])

    def im_response_cb(self, response):
        """ Callback to process the responses from the Interaction Manager """
        rospy.loginfo('Received message from IM:\n{}'.format(str(response)))
        if response.actor == 'interaction_behavior_executor':
            self.parse_im_response(response)

    def parse_im_response(self, action):
        """ Parses a response of the Interaction Manager and updates
            the user_engaged or user_responded predicates depending on the
            Interaction Manager Response
        """
        rospy.loginfo('Parsing response from IM: {}'.format(action.name))
        if action.name == 'interaction_feedback':
            user_engaged = action.args[0].value
            rospy.loginfo("User engaged: {}".format(user_engaged))
            self.user_engaged = _str_to_bool(user_engaged)
        if action.name == 'interaction_result':
            user_responded = action.args[0].value
            rospy.loginfo("User responded: {}".format(user_responded))
            self.user_responded = _str_to_bool(user_responded)
            self.is_interaction_ended = True

    def send_feedback_to_bm(self, user_engaged):
        """ Sends a feedback message to the Behavior Manager """
        kvpa_msg = kvpa.from_dict({'user_engaged': str(user_engaged).lower()})
        feedback_msg = InteractionBehaviorFeedback(feedback=kvpa_msg)
        rospy.loginfo("Sent Feedback to BM: {}".format(feedback_msg))
        self._as.publish_feedback(feedback_msg)

    def send_result_to_bm(self):
        """ Sends a result message to the Behavior Manager """
        kvpa_msg = kvpa.from_dict(
            {'user_responded': str(self.user_responded).lower()})
        result_msg = InteractionBehaviorResult(result=kvpa_msg)
        self._as.set_succeeded(result_msg)
        rospy.loginfo("The result of the interaction is {}"
                      .format(self.user_responded))
        rospy.loginfo("Sent Result to BM: {}".format(result_msg))

    def is_behavior_succeded(self):
        """ Returns true if behavior execution has been successful"""
        # return all([self.user_responded, not self.preempted,
        #               not self.aborted])
        return all([not self.preempted, not self.aborted])

if __name__ == '__main__':
    rospy.init_node('interaction_behavior_executor')
    nname = rospy.get_name()
    rospy.loginfo("Starting Interaction Behavior Executor node: " + nname)
    InteractionBehaviorExecutorServer(rospy.get_name())
    rospy.spin()
