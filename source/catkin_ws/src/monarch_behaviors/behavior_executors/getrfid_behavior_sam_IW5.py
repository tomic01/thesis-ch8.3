#!/usr/bin/env python

import roslib; roslib.load_manifest('mosmach')
import rospy
import smach
import smach_ros
import time

from math import sin,cos,sqrt,fabs,pi,atan
from monarch_msgs.msg import RfidReading
from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction
from mosmach.change_condition import ChangeCondition
from mosmach.change_conditions.sam_condition import SamCondition
from mosmach.change_conditions.topic_condition import TopicCondition
from mosmach.actions.move_to_action import MoveToAction
from mosmach.actions.sam_reader_action import SamReaderAction
from mosmach.actions.topic_reader_action import TopicReaderAction
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from monarch_msgs.msg import KeyValuePair, PersonLocalizationTrackingDataArray
from monarch_behaviors.msg import GetRfidAction, GetRfidResult

from mosmach.util import pose2pose_stamped

from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter

from tf import *
import tf
import numpy

ident=0

# define state START
class Start(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['init_person_position','action_result'], output_keys=['init_person_position','action_result'])
		rospy.loginfo("Init state START")

		self.position=Pose2D()
		self.position_detected=False


  	def people_localization_tracker_cb(self, data):
		global ident
		if ident==0:
			for user in range(0,len(data.locations)):
				if data.locations[user].id > 1000:
        				self.user_id=user
					ident=data.locations[user].id
        				q = (data.locations[self.user_id].pose.pose.orientation.x, 
        				data.locations[self.user_id].pose.pose.orientation.y,
        				data.locations[self.user_id].pose.pose.orientation.z,
        				data.locations[self.user_id].pose.pose.orientation.w)

        				euler = tf.transformations.euler_from_quaternion(q)
        				yaw = 0.40 #euler[2]

					self.position.x=data.locations[self.user_id].pose.pose.position.x
					self.position.y=data.locations[self.user_id].pose.pose.position.y
					self.position.theta=1.64 #euler[2]

					self.position_detected=True
		else:
			for user in range(0,len(data.locations)):
				if data.locations[user].id == ident:
        				self.user_id=user
        				q = (data.locations[self.user_id].pose.pose.orientation.x, 
        				data.locations[self.user_id].pose.pose.orientation.y,
        				data.locations[self.user_id].pose.pose.orientation.z,
        				data.locations[self.user_id].pose.pose.orientation.w)

        				euler = tf.transformations.euler_from_quaternion(q)
        				yaw = 0.40 #euler[2]

					self.position.x=data.locations[self.user_id].pose.pose.position.x
					self.position.y=data.locations[self.user_id].pose.pose.position.y
					self.position.theta=1.64 #euler[2]

					self.position_detected=True			



  	def execute(self, userdata):

    		rospy.loginfo('Executing state START')
		userdata.action_result.result.array[0].value = 'False'
		r = rospy.Rate(2)

    		sam_reader= SAMReader("People_Localization_Tracker_2",self.people_localization_tracker_cb)

    
    		while not rospy.is_shutdown() and self.position_detected is False:

      			if self.preempt_requested():
				sam_reader.remove()
        			self.service_preempt()
        			return 'preempted'
      			r.sleep()

		userdata.init_person_position=self.position

    		sam_reader.remove()
    		return 'succeeded'

# define state SET
#class Set(MonarchState):
#    def __init__(self):
#        MonarchState.__init__(self, state_outcomes=['final_position_set','too_near','set_preempted'], input_keys=#['init_person_position','robot_position','distance','cannot_read'],output_keys=['init_person_position','robot_position','distance','cannot_read'])
#	rospy.loginfo("Init state SET")

#    def execute(self, userdata):
#        rospy.loginfo('Executing state SET')

#        if self.preempt_requested():
#        	self.service_preempt()
#                return 'set_preempted'

#	if userdata.cannot_read==True:
#		userdata.cannot_read=False
#		if userdata.distance>0.5:
#			userdata.distance=userdata.distance-0.5
#		else:
#			return 'too_near'

#	userdata.robot_position.theta=userdata.init_person_position.theta+pi

#	atan()
#	while userdata.robot_position.theta>2*pi:
#		userdata.robot_position.theta=userdata.robot_position.theta-2*pi
#	while userdata.robot_position.theta<0:
#		userdata.robot_position.theta=userdata.robot_position.theta+2*pi
#	userdata.robot_position.x=userdata.init_person_position.x+userdata.distance*cos(userdata.init_person_position.theta)
#	userdata.robot_position.y=userdata.init_person_position.y+userdata.distance*sin(userdata.init_person_position.theta)
#        return 'final_position_set'


# define state SET
class Set(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['final_position_set','too_near','preempted'], input_keys=['init_person_position','robot_position','distance','cannot_read'],output_keys=['init_person_position','robot_position','distance','cannot_read'])
        	rospy.loginfo("Init state SET")
        	samSlotName = 'amcl_pose'
        	samAgentName=''

		#samcondition =  SamCondition(self, samSlotName, samAgentName, self.PoseCondition)
		#self.add_change_condition(samcondition, ['look_finished','preempted'])

		topicCondition2 = TopicCondition(self, samSlotName,PoseWithCovarianceStamped,self.PoseCondition)
		self.add_change_condition(topicCondition2, ['final_position_set','too_near','preempted'])



	def PoseCondition(self, data, userdata):
		userdata.robot_position.x = data.pose.pose.position.x
		userdata.robot_position.y = data.pose.pose.position.y
		userdata.robot_position.theta = 0.0

		angle=atan((userdata.init_person_position.y-data.pose.pose.position.y)/(userdata.init_person_position.x-data.pose.pose.position.x))
	
		userdata.robot_position.theta=angle

		if userdata.cannot_read==True:
			userdata.cannot_read=False
		if userdata.distance>0.5:
			userdata.distance=userdata.distance-0.5
		else:
			return 'too_near'

		angle_person=angle+pi
		while angle_person>2*pi:
			angle_person=angle_person-2*pi
		while angle_person<0:
			angle_person=angle_person+2*pi

		while userdata.robot_position.theta>2*pi:
			userdata.robot_position.theta=userdata.robot_position.theta-2*pi
		while userdata.robot_position.theta<0:
			userdata.robot_position.theta=userdata.robot_position.theta+2*pi
		userdata.robot_position.x=userdata.init_person_position.x+userdata.distance*cos(angle_person)
		userdata.robot_position.y=userdata.init_person_position.y+userdata.distance*sin(angle_person)
		return 'final_position_set'


# define state MOVE
class Move(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['robot_position'],output_keys=['robot_position'])
	rospy.loginfo('Init state MOVE')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	return pose2pose_stamped(userdata.robot_position.x, userdata.robot_position.y, userdata.robot_position.theta)

    #def execute(self, userdata):
     #   rospy.loginfo('Executing state MOVETOHUMAN')
    	#print userdata.robot_position.x
	#print userdata.robot_position.y
	#print userdata.robot_position.theta
	#for t in range(50):
  	#	if self.preempt_requested():
        #		self.service_preempt()
         #    		return 'preempted'
	#	rospy.sleep(0.1)	
        #return 'succeeded'


# define state LOOKHUMAN
class LookHuman(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['person_position'],output_keys=['person_position'])
        	rospy.loginfo("Init state LOOKHUMAN")

		self.position=Pose2D()
		self.position_detected=False


  	def people_localization_tracker_cb(self, data):
		#for user in range(0,len(data.locations)):
			#if data.locations[user].id > 1000:
		global ident
		for user in range(0,len(data.locations)):
			if data.locations[user].id == ident:
        			self.user_id=user
        			q = (data.locations[self.user_id].pose.pose.orientation.x, 
        			data.locations[self.user_id].pose.pose.orientation.y,
        			data.locations[self.user_id].pose.pose.orientation.z,
        			data.locations[self.user_id].pose.pose.orientation.w)

        			euler = tf.transformations.euler_from_quaternion(q)
        			yaw = 0.40 #euler[2]

				self.position.x=data.locations[self.user_id].pose.pose.position.x
				self.position.y=data.locations[self.user_id].pose.pose.position.y
				self.position.theta=1.64 #euler[2]

				self.position_detected=True



  	def execute(self, userdata):

    		rospy.loginfo('Executing state LOOKHUMAN')
		r = rospy.Rate(2)

    		sam_reader= SAMReader("People_Localization_Tracker_2",self.people_localization_tracker_cb)

    
    		while not rospy.is_shutdown() and self.position_detected is False:

      			if self.preempt_requested():
				sam_reader.remove()
        			self.service_preempt()
        			return 'preempted'
      			r.sleep()

		userdata.person_position=self.position

    		sam_reader.remove()
    		return 'succeeded'

# define state COMPARE
class Compare(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['person_still','person_moving','preempted'], input_keys=['person_position','init_person_position','distance'],output_keys=['person_position','init_person_position','distance'])
	rospy.loginfo("Init state COMPARE")

    def execute(self, userdata):
        rospy.loginfo('Executing state COMPARE')

        if self.preempt_requested():
        	self.service_preempt()
                return 'preempted'

	if sqrt((userdata.person_position.x-userdata.init_person_position.x)*(userdata.person_position.x-userdata.init_person_position.x)+(userdata.person_position.y-userdata.init_person_position.y)*(userdata.person_position.y-userdata.init_person_position.y))>0.5 or fabs(userdata.person_position.theta-userdata.init_person_position.theta)>pi/4:
		userdata.init_person_position.x=userdata.person_position.x
		userdata.init_person_position.y=userdata.person_position.y
		userdata.init_person_position.theta=userdata.person_position.theta
		userdata.distance=2.0
		return 'person_moving'
	else:
		return 'person_still'



# define state READ
class Read(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['read_finished','preempted','read_aborted'],input_keys=['person_rfid'], output_keys=['person_rfid'])
		rospy.loginfo("Init state READ")
        	topicName = 'rfid_tag'

		topicCondition = TopicCondition(self, topicName, RfidReading, self.RfidCondition)
		self.add_change_condition(topicCondition, ['read_finished','preempted','read_aborted'])

	def RfidCondition(self, data, userdata):
		userdata.person_rfid.tag_id=data.tag_id
		value = 'read_finished'
		return value

# define state WAIT
class Wait(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['wait_finished','wait_preempted'], input_keys=['cannot_read'],output_keys=['cannot_read'])
	rospy.loginfo('Init state WAIT')

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
	#time.sleep(2.0)
	for t in range(20):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'wait_preempted'
		rospy.sleep(0.1)
	userdata.cannot_read=True
        return 'wait_finished'

        
# define state FINISH
class Finish(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['finish_succeeded','finish_preempted'], input_keys=['person_rfid','action_result'],output_keys=['person_rfid','action_result'])
	rospy.loginfo('Init state FINISH')

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH')
	userdata.action_result.result.array[0].value = 'True'
	print userdata.person_rfid
        return 'finish_succeeded'


if __name__ == '__main__':

    rospy.init_node('behavior_getrfid')
    rospy.loginfo('Starting up')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['getrfid_done','preempted','aborted'],
                          input_keys = [], 
                          output_keys = ['action_result'])
    sm.userdata.distance=2.0
    sm.userdata.cannot_read=False
    sm.userdata.person_position=Pose2D()
    sm.userdata.init_person_position=Pose2D()
    sm.userdata.robot_position=Pose2D()
    sm.userdata.person_rfid=RfidReading()

    sm.userdata.action_result = GetRfidResult()
    result_reached = KeyValuePair()
    result_reached.key = 'got'
    result_reached.value = 'False'
    sm.userdata.action_result.result.array.append(result_reached)

    # Open the container
    with sm:
        # Add states to the container
        sm.add('START', Start(), 
                               transitions={'succeeded':'SET', 'preempted':'preempted'})

        sm.add('SET', Set(), 
                               transitions={'final_position_set':'APPROACH', 'too_near':'aborted', 'preempted':'preempted'})


    	sm1 = smach.StateMachine(outcomes = ['preempted','person_moving'])
	sm1.userdata=sm.userdata
   	with sm1:
      		sm1.add('LOOKHUMAN', LookHuman(),
                        transitions = { 'preempted':'preempted','succeeded':'COMPARE'})
		sm1.add('COMPARE', Compare(),
			transitions = {'preempted':'preempted','person_moving':'person_moving','person_still':'LOOKHUMAN'})

        def termination_cb1(outcome_map):
            if outcome_map['MOVE'] == 'succeeded' or outcome_map['MONITOR'] == 'person_moving' or outcome_map['MOVE'] =='aborted' or outcome_map['READ1'] == 'read_finished': #or outcome_map['MONITOR'] == 'person_still':
		print 'Aqui'
                return True
            else:
                return False

        sm_con1 = smach.Concurrence(outcomes=['approach_default','approach_done','position_wrong','rfid_read','approach_aborted','approach_preempted'],
                                      default_outcome='approach_default',
				      child_termination_cb = termination_cb1,
                                      outcome_map={'approach_done':
                                          {'MOVE':'succeeded',
                                            'MONITOR':'preempted',
					     'READ1':'preempted'},
                                              'position_wrong':
                                          {'MOVE':'preempted',
				            'MONITOR':'person_moving',
					     'READ1':'preempted'},
                                              'rfid_read':
                                          {'READ1':'read_finished'},
                                               'approach_aborted':
                                          {'MOVE':'aborted',
					    'MONITOR':'preempted',
					     'READ1':'preempted'},
                                               'approach_preempted':
                                          {'MOVE':'preempted',
                                            'MONITOR':'preempted',
					     'READ1':'preempted'}})
	sm_con1.userdata=sm.userdata
        with sm_con1:
            smach.Concurrence.add('MOVE', Move())
            smach.Concurrence.add('MONITOR', sm1)
	    smach.Concurrence.add('READ1', Read())


        sm.add('APPROACH',sm_con1,
                                transitions={'approach_default':'APPROACH','approach_done':'GET','position_wrong':'START','rfid_read':'FINISH', 'approach_aborted':'aborted','approach_preempted':'preempted'})


        def termination_cb2(outcome_map):
            if outcome_map['READ'] == 'read_finished' or outcome_map['WAIT'] == 'wait_finished':
                return True
            else:
                return False

        sm_con2 = smach.Concurrence(outcomes=['get_default','get_succeeded','get_failed','get_aborted','get_preempted'],
                                      default_outcome='get_default',
				      output_keys=['person_rfid'],
				      child_termination_cb = termination_cb2,
                                      outcome_map={'get_succeeded':
                                          {'READ':'read_finished',
                                            'WAIT':'wait_preempted'},
                                              'get_failed':
                                          {'READ':'preempted',
				            'WAIT':'wait_finished'},
                                               'get_aborted':
                                          {'READ':'read_aborted'},
                                               'get_preempted':
                                          {'READ':'preempted',
                                            'WAIT':'wait_preempted'}})	
	sm_con2.userdata=sm.userdata
        with sm_con2:
            smach.Concurrence.add('READ', Read())
            smach.Concurrence.add('WAIT', Wait())

        sm.add('GET',sm_con2,
                                transitions={'get_default':'GET','get_succeeded':'FINISH','get_failed':'SET', 'get_aborted':'aborted','get_preempted':'preempted'})
        sm.add('FINISH', Finish(), 
                               transitions={'finish_succeeded':'getrfid_done', 'finish_preempted':'preempted'})


  # action_{goal,feedback,result} are the default values and don't need to be passed as input
    action_name = rospy.get_name()  
    asw = smach_ros.ActionServerWrapper(action_name,
                                      GetRfidAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['getrfid_done'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'],
                                      result_key = 'action_result')

    # Create and start the introspection server 
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()


    # Execute SMACH plan
    
    asw.run_server()
    #outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    #sis.stop()


