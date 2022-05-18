#!/usr/bin/env python

import roslib; roslib.load_manifest('mosmach')
import rospy
import smach
import smach_ros
import time

from math import sin,cos,sqrt,fabs,pi
from monarch_msgs.msg import RfidReading
from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction
from mosmach.actions.run_ce_action import RunCeAction
from mosmach.change_condition import ChangeCondition
from mosmach.change_conditions.sam_condition import SamCondition
from mosmach.change_conditions.topic_condition import TopicCondition
from mosmach.actions.sam_reader_action import SamReaderAction
from mosmach.actions.topic_reader_action import TopicReaderAction
from mosmach.actions.move_to_action import MoveToAction
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from monarch_msgs.msg import KeyValuePair, PersonLocalizationTrackingDataArray, KeyValuePairArray
from monarch_behaviors.msg import LeadAction, LeadResult

from mosmach.util import pose2pose_stamped

distance=1.0


# define state START
class Start(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'],input_keys=['final_position','action_result'], output_keys=['final_position','action_result'])
		rospy.loginfo("Init state START")
		samSlotName = 'FinalPosition'
      	        samAgentName=''
		#robot_id = rospy.get_namespace()
		#robot_id = robot_id.strip('/')

		#samcondition =  SamCondition(self, samSlotName, samAgentName, self.PoseCondition)
		#self.add_change_condition(samcondition, ['look_finished','preempted'])

		self.goalReader = TopicReaderAction(self, samSlotName,Pose2D,self.PoseCondition)
		self.add_action(self.goalReader)

	def PoseCondition(self, data, userdata):
		userdata.action_result.result.array[0].value = 'False'
		userdata.final_position.x = data.x
		userdata.final_position.y = data.y
		userdata.final_position.theta = data.theta
		#value = 'look_finished'
		value=True
		return value

# define state LOOKHUMAN
class LookHuman(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['person_position'],output_keys=['person_position'])
        	rospy.loginfo("Init state LOOKHUMAN")
		samSlotName = 'HumanPosition'
      	        samAgentName=''

		#samcondition =  SamCondition(self, samSlotName, samAgentName, self.PoseCondition)
		#self.add_change_condition(samcondition, ['look_finished','preempted'])

		self.goalReader = TopicReaderAction(self, samSlotName,Pose2D,self.PoseCondition)
		self.add_action(self.goalReader)

	def PoseCondition(self, data, userdata):
		userdata.person_position.x = data.x
		userdata.person_position.y = data.y
		userdata.person_position.theta = data.theta
		#value = 'look_finished'
		value=True
		return value


# define state SET
class Set(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['position_set','set_preempted'], input_keys=['person_position','robot_position'],output_keys=['robot_position'])
	rospy.loginfo("Init state SET")

    def execute(self, userdata):
        rospy.loginfo('Executing state SET')

        if self.preempt_requested():
        	self.service_preempt()
                return 'set_preempted'

	userdata.robot_position.theta=userdata.person_position.theta+pi
	while userdata.robot_position.theta>2*pi:
		userdata.robot_position.theta=userdata.robot_position.theta-2*pi
	while userdata.robot_position.theta<0:
		userdata.robot_position.theta=userdata.robot_position.theta+2*pi
	userdata.robot_position.x=userdata.person_position.x+distance*cos(userdata.person_position.theta)
	userdata.robot_position.y=userdata.person_position.y+distance*sin(userdata.person_position.theta)
        return 'position_set'


# define state MOVETOHUMAN
class MoveToHuman(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['robot_position'],output_keys=[''])
	rospy.loginfo('Init state MOVETOHUMAN')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	return pose2pose_stamped(userdata.robot_position.x, userdata.robot_position.y, userdata.robot_position.theta)

  #  def execute(self, userdata):
     #   rospy.loginfo('Executing state MOVETOHUMAN')
#	print userdata.robot_position.x
#	print userdata.robot_position.y
#	print userdata.robot_position.theta
#	for t in range(50):
 # 		if self.preempt_requested():
  #      		self.service_preempt()
   #            		return 'preempted'
#		rospy.sleep(0.1)	
 #       return 'succeeded'


# define state READ
class Read(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'],input_keys=['person_rfid'], output_keys=['person_rfid'])
		rospy.loginfo("Init state READ")
        	topicName = 'rfid_tag'

		#topicCondition = TopicCondition(self, topicName, RfidReading, self.RfidCondition)
		#self.add_change_condition(topicCondition, ['read_finished','preempted','read_aborted'])

		self.goalReader = TopicReaderAction(self, topicName,RfidReading,self.RfidCondition)
		self.add_action(self.goalReader)

	def RfidCondition(self, data, userdata):
		userdata.person_rfid.tag_id=data.tag_id
		value = True
		return value

# define state GREET
class Greet(MonarchState):

  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state GREET")  
    greeting_name = 'give_greetings_child'
    runGreetingCeState = RunCeAction(self, greeting_name)
    self.add_action(runGreetingCeState)




# define state MOVETOEND
class MoveToEnd(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['final_position'],output_keys=[''])
	rospy.loginfo('Init state MOVETOEND')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	return pose2pose_stamped(userdata.final_position.x, userdata.final_position.y, userdata.final_position.theta)

    #def execute(self, userdata):
     #   rospy.loginfo('Executing state MOVETOEND')
	#print userdata.final_position.x
	#print userdata.final_position.y
	#print userdata.final_position.theta
	#for t in range(200):
  	#	if self.preempt_requested():
        #		self.service_preempt()
        #       		return 'preempted'
	#	rospy.sleep(0.1)	
        #return 'succeeded'


# define state LOOKROBOT
class LookRobot(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['person_too_far','person_following','preempted'], input_keys=['robot_position','person_position'],output_keys=['robot_position','person_position'])
        	rospy.loginfo("Init state LOOKROBOT")
        	samSlotName = 'amcl_pose'
        	samAgentName=''

		#samcondition =  SamCondition(self, samSlotName, samAgentName, self.PoseCondition)
		#self.add_change_condition(samcondition, ['look_finished','preempted'])

		topicCondition2 = TopicCondition(self, samSlotName,PoseWithCovarianceStamped,self.PoseCondition)
		self.add_change_condition(topicCondition2, ['person_too_far','person_following','preempted'])

	def PoseCondition(self, data, userdata):
		userdata.robot_position.x = data.pose.pose.position.x
		userdata.robot_position.y = data.pose.pose.position.y
		userdata.robot_position.theta = 0.0

		if sqrt((userdata.person_position.x-userdata.robot_position.x)*(userdata.person_position.x-userdata.robot_position.x)+(userdata.person_position.y-userdata.robot_position.y)*(userdata.person_position.y-userdata.robot_position.y))>2.0:
			value = 'person_too_far'
		else: 
			value = 'person_following'
		return value


# define state RFID_COMPARE
class Rfid_Compare(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['rfid_correct','rfid_incorrect','preempted'],input_keys=['person_rfid'], output_keys=['person_rfid'])
		rospy.loginfo("Init state RFID_COMPARE")
        	topicName = 'rfid_tag'

		topicCondition = TopicCondition(self, topicName, RfidReading, self.RfidCondition)
		self.add_change_condition(topicCondition, ['rfid_correct','rfid_incorrect','preempted'])

	def RfidCondition(self, data, userdata):
		if userdata.person_rfid.tag_id==data.tag_id:
			value='rfid_correct'
		else:
			value='rfid_incorrect'
		return value

# define state WAIT
class Wait(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['wait_finished','preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state WAIT')

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
	#time.sleep(2.0)
	for t in range(20):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
        return 'wait_finished'

# define state WAITPREEMPT
class WaitPreempt(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state WAITPREEMPT')

    def execute(self, userdata):
        rospy.loginfo('Executing state WAITPREEMPT')
	#time.sleep(2.0)
	for t in range(10):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
        return 'succeeded'

# define state STOP
class Stop(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['robot_position'],output_keys=[''])
	rospy.loginfo('Init state Stop')
	#navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	#self.add_action(navMoveToPersonAction)

    #def goal_cb(self,userdata):
	#return pose2pose_stamped(userdata.robot_position.x, userdata.robot_position.y, userdata.robot_position.theta)

    def execute(self, userdata):
        rospy.loginfo('Executing state STOP')
	print userdata.robot_position.x
	print userdata.robot_position.y
	print userdata.robot_position.theta
	for t in range(2):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)	
        return 'succeeded'

# define state WAITMORE
class WaitMore(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['wait_finished','preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state WAITMORE')

    def execute(self, userdata):
        rospy.loginfo('Executing state WAITMORE')
	#time.sleep(2.0)
	for t in range(100):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
        return 'wait_finished'

# define state FINISH
class Finish(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['finish_succeeded','finish_preempted'], input_keys=['person_position','action_result'],output_keys=['person_position','action_result'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH')
	userdata.action_result.result.array[0].value = 'True'
	print userdata.person_position
        return 'finish_succeeded'



if __name__ == '__main__':

    rospy.init_node('behavior_lead')
    rospy.loginfo('Starting up')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['lead_finished','preempted','aborted'],
                          input_keys = [], 
                          output_keys = ['action_result'])

    sm.userdata.final_position=Pose2D()
    sm.userdata.person_position=Pose2D()
    sm.userdata.robot_position=Pose2D()
    sm.userdata.person_rfid=RfidReading()

    sm.userdata.action_result = LeadResult()
    result_reached = KeyValuePair()
    result_reached.key = 'finished'
    result_reached.value = 'False'
    sm.userdata.action_result.result.array.append(result_reached)

    # Open the container
    with sm:
        # Add states to the container
        sm.add('START', Start(), transitions={'succeeded':'LOOKHUMAN1', 'preempted':'preempted'})

	sm.add('LOOKHUMAN1', LookHuman(),transitions = { 'preempted':'preempted','succeeded':'SET'})

        sm.add('SET', Set(), transitions={'position_set':'MOVETOHUMAN', 'set_preempted':'preempted'})

        sm.add('MOVETOHUMAN', MoveToHuman(), transitions={'succeeded':'GREET', 'aborted':'aborted', 'preempted':'preempted'})

        sm.add('GREET', Greet(), transitions={'succeeded':'READ'})

	sm.add('READ', Read(), transitions={'succeeded':'LEAD', 'preempted':'preempted'})

    	sm1 = smach.StateMachine(outcomes = ['preempted','person_too_far','person_near'])
	sm1.userdata=sm.userdata
   	with sm1:
      		sm1.add('LOOKHUMAN', LookHuman(),
                        transitions = { 'preempted':'preempted','succeeded':'LOOKROBOT'})
		sm1.add('LOOKROBOT', LookRobot(),
			transitions = {'preempted':'preempted','person_too_far':'person_too_far','person_following':'person_near'})

    	sm2 = smach.StateMachine(outcomes = ['rfid_correct','preempted'])
	sm2.userdata=sm.userdata
   	with sm2:
      		sm2.add('RFID_COMPARE', Rfid_Compare(),
                        transitions = { 'rfid_correct':'rfid_correct','rfid_incorrect':'RFID_COMPARE','preempted':'preempted'})


        def termination_cb2(outcome_map):
            if outcome_map['MONITOR_POS'] == 'person_near' or outcome_map['MONITOR_POS'] == 'person_too_far' or outcome_map['MONITOR_RFID'] == 'rfid_correct' or outcome_map['WAIT'] == 'wait_finished':
                return True
            else:
                return False

	sm_con2 = smach.Concurrence(outcomes=['person_following_pos','person_following_rfid','person_too_far','person_lost','monitor_default','preempted'],
                                      default_outcome='monitor_default',
				      child_termination_cb = termination_cb2,
                                      outcome_map={'person_following_pos':
                                          {'MONITOR_RFID':'preempted',
                                            'MONITOR_POS':'person_near'},
                                              'person_following_rfid':
                                          {'MONITOR_RFID':'rfid_correct'},
                                               'person_too_far':
                                          {'MONITOR_RFID':'preempted',
                                            'MONITOR_POS':'person_too_far'},
						'person_lost':
                                          {'MONITOR_RFID':'preempted',
                                            'MONITOR_POS':'preempted',
					     'WAIT':'wait_finished'},
						'preempted':
                                          {'MONITOR_RFID':'preempted',
                                            'MONITOR_POS':'preempted',
					     'WAIT':'preempted'}})
	sm_con2.userdata=sm.userdata
        with sm_con2:
            smach.Concurrence.add('MONITOR_RFID', sm2)
            smach.Concurrence.add('MONITOR_POS', sm1)
	    smach.Concurrence.add('WAIT',Wait())

    	sm3 = smach.StateMachine(outcomes = ['person_too_far','preempted'])
	sm3.userdata=sm.userdata
   	with sm3:
      		sm3.add('MONITOR_SUB', sm_con2,
                        transitions = { 'person_following_pos':'WAITPREEMPT','person_following_rfid':'WAITPREEMPT','person_too_far':'person_too_far','person_lost':'person_too_far','monitor_default':'MONITOR_SUB','preempted':'preempted'})
		sm3.add('WAITPREEMPT', WaitPreempt(),
			transitions = {'succeeded':'MONITOR_SUB','preempted':'preempted'})


        def termination_cb1(outcome_map):
            if outcome_map['MONITOR'] == 'person_too_far' or outcome_map['MOVETOEND'] == 'aborted'or outcome_map['MOVETOEND'] == 'succeeded':
                return True
            else:
                return False
	
        sm_con1 = smach.Concurrence(outcomes=['lead_default','lead_done','look_for_person','lead_aborted','lead_preempted'],
                                      default_outcome='lead_default',
				      child_termination_cb = termination_cb1,
                                      outcome_map={'lead_done':
                                          {'MOVETOEND':'succeeded'},
                                              'look_for_person':
                                          {'MONITOR':'person_too_far'},
                                               'lead_aborted':
                                          {'MOVETOEND':'aborted',
					    'MONITOR':'preempted'},
                                               'lead_preempted':
                                          {'MOVETOEND':'preempted',
                                            'MONITOR':'preempted'}})
	sm_con1.userdata=sm.userdata
        with sm_con1:
            smach.Concurrence.add('MOVETOEND', MoveToEnd())
            smach.Concurrence.add('MONITOR', sm3)


        sm.add('LEAD',sm_con1, transitions={'lead_default':'LEAD','lead_done':'FINISH','look_for_person':'STOP', 'lead_aborted':'aborted','lead_preempted':'preempted'})

        def termination_cb3(outcome_map):
            if outcome_map['LOOKHUMAN2'] == 'succeeded' or outcome_map['MONITOR_RFID2'] == 'rfid_correct' or outcome_map['WAITMORE'] == 'wait_finished':
                return True
            else:
                return False

	sm_con3 = smach.Concurrence(outcomes=['person_detected_pos','person_detected_rfid','person_lost','monitor_default','monitor_preempted'],
                                      default_outcome='monitor_default',
				      child_termination_cb = termination_cb3,
                                      outcome_map={'person_detected_pos':
                                          {'MONITOR_RFID2':'preempted',
                                            'LOOKHUMAN2':'succeeded'},
                                              'person_detected_rfid':
                                          {'MONITOR_RFID2':'rfid_correct'},
						'person_lost':
                                          {'MONITOR_RFID2':'preempted',
                                            'LOOKHUMAN2':'preempted',
					     'WAITMORE':'wait_finished'},
						'monitor_preempted':
                                          {'MONITOR_RFID2':'preempted',
                                            'LOOKHUMAN2':'preempted',
					     'WAITMORE':'preempted'}})
	sm_con3.userdata=sm.userdata
        with sm_con3:
            smach.Concurrence.add('MONITOR_RFID2', sm2)
            smach.Concurrence.add('LOOKHUMAN2', LookHuman())
	    smach.Concurrence.add('WAITMORE',WaitMore())
	
	sm.add('STOP', Stop(), transitions={'succeeded':'WAITING_PERSON', 'aborted':'aborted', 'preempted':'preempted'})

        sm.add('WAITING_PERSON',sm_con3, transitions={'person_detected_pos':'LOOKHUMAN1','person_detected_rfid':'LEAD','person_lost':'aborted','monitor_preempted':'preempted','monitor_default':'WAITING_PERSON'})

        sm.add('FINISH', Finish(), transitions={'finish_succeeded':'lead_finished', 'finish_preempted':'preempted'})

 # action_{goal,feedback,result} are the default values and don't need to be passed as input
    action_name = rospy.get_name()  
    asw = smach_ros.ActionServerWrapper(action_name,
                                      LeadAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['lead_finished'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'],
                                      result_key = 'action_result')

    # Create and start the introspection server 
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()


    # Execute SMACH plan
    #outcome = sm.execute()
    asw.run_server()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    #sis.stop()

