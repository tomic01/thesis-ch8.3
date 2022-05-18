#!/usr/bin/env python

import roslib; roslib.load_manifest('mosmach')
import rospy
import smach
import smach_ros

from mosmach.monarch_state import MonarchState
from mosmach.actions.move_to_action import MoveToAction
from mosmach.actions.run_ca_action import RunCaAction
from mosmach.actions.topic_writer_action import TopicWriterAction
from mosmach.change_conditions.topic_condition import TopicCondition
from mosmach.actions.control_projector_action import ControlProjectorAction
from mosmach.actions.move_head_action import MoveHeadAction
from monarch_msgs.msg import KeyValuePairArray
from monarch_msgs.msg import KeyValuePair
from monarch_msgs_utils import key_value_pairs as kvpa
from std_msgs.msg import String
from monarch_behaviors.msg import TeachingAssistanceAction, TeachingAssistanceResult


### Navigation states ###
class MoveToProjectionAreaState(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'])
        rospy.loginfo("Move to Projection Area State")

        #Enter right coordinates
        # IST: navMoveToProjectionArea = MoveToAction(self,3.7,12.25,0.51)
        # ORU: navMoveToProjectionArea = MoveToAction(self,3.45,5.25,-1.07)
        # IPOL: classroom
        navMoveToProjectionArea = MoveToAction(self,-0.4,-12.12,-1.53)
        self.add_action(navMoveToProjectionArea)

        # Publish the topic that allows interfaces to be used
        #pubOnTopicAction = TopicWriterAction(self, 'allowed_hri_interfaces', String, 'arms|audio|cheeks|eyes|image|leds_base|mouth|projector|voice')
        #self.add_action(pubOnTopicAction)


### Head Movement ###
class MoveHeadRightState(MonarchState):
    def __init__(self):
	MonarchState.__init__(self, state_outcomes=['succeeded'])
	rospy.loginfo("MoveHeadRightState")
	moveHeadAction = MoveHeadAction(self, 1.5, 50)
	self.add_action(moveHeadAction)

class MoveHeadCentralState(MonarchState):
    def __init__(self):
	MonarchState.__init__(self, state_outcomes=['succeeded'])
	rospy.loginfo("MoveHeadCentralState")
	moveHeadAction = MoveHeadAction(self, 0, 50)
	self.add_action(moveHeadAction)

        # Allow the usage of the Head again
        pubOnTopicAction = TopicWriterAction(self, 'allowed_hri_interfaces', String, 'arms|audio|cheeks|eyes|head|image|leds_base|mouth|projector|voice')
        self.add_action(pubOnTopicAction)


### Projector Control ###
class TurnOnTheProjectorState(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'])
        rospy.loginfo("Turn On the Projector State")
        
        turnOnTheProjectorAction = ControlProjectorAction(self, True)
        self.add_action(turnOnTheProjectorAction)

class TurnOffTheProjectorState(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'])
        rospy.loginfo("Turn Off the Projector State")

        turnOffTheProjectorAction = ControlProjectorAction(self, False)
        self.add_action(turnOffTheProjectorAction)

        
### Teaching Assistance ###
class TeachingAssistanceCaActivationState(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'])
        rospy.loginfo("Teaching Assistance State")

##1 Activate state to show the menu
        di = {"activated_cas":"ca14.4"}
        kvpa_msg = kvpa.from_dict(di)
        activationCA = RunCaAction(self, kvpa_msg)
        self.add_action(activationCA)

class TeachingAssistanceState(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['finished', 'reactivate'])
        rospy.loginfo("Teaching Assistance State")
        
##2 Is TA finished?
        topicCondition = TopicCondition(self, 'command_from_user', KeyValuePairArray, self.finishCondition)
        self.add_change_condition(topicCondition, ['finished', 'reactivate'])

    def finishCondition(self, data, userdata):
	rospy.loginfo("Finish Condition Callback")
	keyValEl0 = data.array[0]
	if keyValEl0.key == 'command':
            if keyValEl0.value == 'stop_projecting':
		value = 'finished'
                #rospy.loginfo("stop projecting - returning fiinsh")
            elif keyValEl0.value == 'select_media_content':
		value = 'reactivate'
                #rospy.loginfo("reactivating - returning reactivate")
	    else:
		value = 'reactivate'
        else:
	   value = 'reactivate'

        rospy.sleep(3)
        return value

class Finish(MonarchState):
   def __init__(self):
       MonarchState.__init__(self, state_outcomes=['succeeded'], 
                             input_keys=['action_result'], 
                             output_keys=['action_result'])

       rospy.loginfo('Init Finish State')

       def execute(self, userdata):
           rospy.loginfo('Executing Finish State')
           userdata.action_result.result.array[0].value = 'True'
           return 'succeeded'

### MAIN ###
def main():
	rospy.init_node("behavior_teachingassistance")

        # State TA
	sm = smach.StateMachine(outcomes = ['succeeded','preempted','aborted'])
        
        with sm:
            sm.add('MoveToProjectionArea', MoveToProjectionAreaState(), transitions={'succeeded':'MoveHeadRight'})
            sm.add('MoveHeadRight', MoveHeadRightState(), transitions={'succeeded':'TurnOnTheProjector'})
            sm.add('TurnOnTheProjector', TurnOnTheProjectorState(), transitions={'succeeded':'TeachingAssistanceCaActivation'})
            sm.add('TeachingAssistanceCaActivation', TeachingAssistanceCaActivationState(), transitions={'succeeded':'TeachingAssistance'})
            sm.add('TeachingAssistance', TeachingAssistanceState(), transitions={'finished':'TurnOffTheProjector', 'reactivate':'TeachingAssistanceCaActivation'})
            sm.add('TurnOffTheProjector', TurnOffTheProjectorState(), transitions={'succeeded':'MoveHeadCentral'})
            sm.add('MoveHeadCentral', MoveHeadCentralState(), transitions={'succeeded':'succeeded'})
#sm.add('FINISH', Finish(), transitions={'succeeded':'succeeded'})

            action_name = rospy.get_name()
            asw = smach_ros.ActionServerWrapper(action_name,
                                                TeachingAssistanceAction,
                                                wrapped_container = sm,
                                                succeeded_outcomes = ['succeeded'],
                                                preempted_outcomes = ['preempted'],
                                                aborted_outcomes = ['aborted'],
                                                result_key = 'action_result')


        #outcome = sm.execute()
        asw.run_server()
        rospy.spin()


if __name__ == '__main__':
    main()
