#!/usr/bin/env python

import roslib; roslib.load_manifest('mosmach')
import rospy
import smach
import smach_ros
import time

from math import sin,cos,sqrt,fabs,pi,atan2
from monarch_msgs.msg import RfidReading
from mosmach.monarch_state import MonarchState
from mosmach.actions.run_ca_action import RunCaAction
from mosmach.actions.run_ce_action import RunCeAction
from mosmach.change_condition import ChangeCondition
from mosmach.change_conditions.sam_condition import SamCondition
from mosmach.change_conditions.topic_condition import TopicCondition
from mosmach.actions.move_to_action import MoveToAction
from mosmach.actions.sam_reader_action import SamReaderAction
from mosmach.actions.topic_reader_action import TopicReaderAction
from mosmach.actions.topic_writer_action import TopicWriterAction
from geometry_msgs.msg import Pose2D, PoseWithCovariance, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float64
from monarch_msgs.msg import CapacitiveSensorsReadings
from monarch_msgs.msg import KeyValuePair, PersonLocalizationTrackingDataArray, KeyValuePairArray
from monarch_behaviors.msg import CatchAndTouchAction, CatchAndTouchResult

from mosmach.util import pose2pose_stamped

from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter

from tf import *
import tf
import numpy

distance=1.0
ident=0

# define state START
class Start(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'],input_keys=['final_position','action_result','distance'], output_keys=['final_position','action_result','distance'])
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
		userdata.distance=2.0
		global ident
		ident=0
		#value = 'look_finished'
		value=True
		return value


# define state LOOKHUMANINIT
class LookHuman(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['init_person_position'],output_keys=['init_person_position'])
        	rospy.loginfo("Init state LOOKHUMANINIT")

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

    		rospy.loginfo('Executing state LOOKHUMANINIT')
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





# define state LOOKHUMAN
class LookHuman(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['person_position'],output_keys=['person_position'])
        	rospy.loginfo("Init state LOOKHUMAN")

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

		angle=atan2((userdata.init_person_position.y-data.pose.pose.position.y),(userdata.init_person_position.x-data.pose.pose.position.x))
	
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


# define state MOVETOHUMAN
class MoveToHuman(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['robot_position'],output_keys=[''])
	rospy.loginfo('Init state MOVETOHUMAN')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	return pose2pose_stamped(userdata.robot_position.x, userdata.robot_position.y, userdata.robot_position.theta)

   # def execute(self, userdata):
    #    rospy.loginfo('Executing state MOVETOHUMAN')
#	print userdata.robot_position.x
#	print userdata.robot_position.y
#	print userdata.robot_position.theta
#	for t in range(50):
 # 		if self.preempt_requested():
  #      		self.service_preempt()
   #            		return 'preempted'
	#	rospy.sleep(0.1)	
        #return 'succeeded'

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

# define state INVITE
class Invite(MonarchState):

  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state INVITE")  
    greeting_name = 'mbot_ask_for_game_to_play'
    runGreetingCaState = RunCaAction(self, greeting_name)
    self.add_action(runGreetingCeState)

# define state SENDINVITATION
#class SendInvitation(MonarchState):
 #   def __init__(self):
  #      MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
#	rospy.loginfo("Init state SENDINVITATION")
 #   	topic_name = 'ca_activations'
#	topic_value=KeyValuePairArray()
#	keyvalue=KeyValuePair()
#	keyvalue.key='activated_cas'
#	keyvalue.value='ca1'
#	topic_value.array.append(keyvalue)
#	keyvalue2=KeyValuePair()
#	keyvalue2.key='question_to_user'
#	keyvalue2.value='want_play_game'
#	topic_value.array.append(keyvalue2)
#	keyvalue3=KeyValuePair()
#	keyvalue3.key='name_of_game'
#	keyvalue3.value='catch_and_touch'
#	topic_value.array.append(keyvalue3)	
 #   	writeraction = TopicWriterAction(self, topic_name, KeyValuePairArray,topic_value)
  #  	self.add_action(writeraction)




# define state SENDINVITATION
class SendInvitation(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state SENDINVITATION')
	self.pub = rospy.Publisher('ca_activations', KeyValuePairArray, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state SENDINVITATION')
	topic_value=KeyValuePairArray()
	keyvalue=KeyValuePair()
	keyvalue.key='activated_cas'
	keyvalue.value='ca01'
	topic_value.array.append(keyvalue)
	keyvalue2=KeyValuePair()
	keyvalue2.key='question_to_user'
	keyvalue2.value='want_play_game'
	topic_value.array.append(keyvalue2)
	keyvalue3=KeyValuePair()
	keyvalue3.key='name_of_game'
	keyvalue3.value='catch_and_touch'
	topic_value.array.append(keyvalue3)
	self.pub.publish(topic_value)
        return 'succeeded'

# define state RECEIVEREPLY
class ReceiveReply(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'],input_keys=['person_rfid'], output_keys=['person_rfid'])
		rospy.loginfo("Init state RECEIVEREPLY")
        	topicName = 'command_from_user'

		topicCondition = TopicCondition(self, topicName, KeyValuePairArray, self.ReplyCondition)
		self.add_change_condition(topicCondition, ['succeeded','preempted','aborted'])

	def ReplyCondition(self, data, userdata):
		value='aborted'
		for user in range(0,len(data.array)):
			if data.array[user].key == 'yes_no_answer':
        			self.user_id=user
				if data.array[user].value=='yes':
					value='succeeded'
				else:
					value='aborted'
		return value



# define state MOVETOEND
class MoveToEnd(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['final_position'],output_keys=[''])
	rospy.loginfo('Init state MOVETOEND')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	return pose2pose_stamped(userdata.final_position.x, userdata.final_position.y, userdata.final_position.theta)

   # def execute(self, userdata):
   #     rospy.loginfo('Executing state MOVETOEND')
#	print userdata.final_position.x
#	print userdata.final_position.y
#	print userdata.final_position.theta
#	for t in range(100):
 # 		if self.preempt_requested():
  #      		self.service_preempt()
   #            		return 'preempted'
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

		if sqrt((userdata.person_position.x-userdata.robot_position.x)*(userdata.person_position.x-userdata.robot_position.x)+(userdata.person_position.y-userdata.robot_position.y)*(userdata.person_position.y-userdata.robot_position.y))>4.0:
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
        MonarchState.__init__(self, state_outcomes=['wait_finished','preempted'], input_keys=['cannot_read'],output_keys=['cannot_read'])
	rospy.loginfo('Init state WAIT')

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT')
	#time.sleep(2.0)
	for t in range(20):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
	userdata.cannot_read=True
        return 'wait_finished'

# define state ACCELERATE
class Accelerate(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state ACCELERATE')
	self.pub = rospy.Publisher('navigator/param/vref', Float64, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state ACCELERATE')
	#time.sleep(2.0)
	while not rospy.is_shutdown():
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		self.pub.publish(0.8)
		rospy.sleep(0.1)
        return 'preempted'

# define state WAITPREEMPT
class WaitPreempt(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state WAITPREEMPT')


    def execute(self, userdata):
        rospy.loginfo('Executing state WAITPREEMPT')
	time.sleep(2.0)
	for t in range(10):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
        return 'succeeded'


# define state CHECK
class Check(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['touched','not_touched','preempted'], input_keys=['touched'],output_keys=['touched'])
        	rospy.loginfo("Init state CHECK")
        	topicName = 'cap_sensors'

		#samcondition =  SamCondition(self, samSlotName, samAgentName, self.PoseCondition)
		#self.add_change_condition(samcondition, ['look_finished','preempted'])

		topicCondition = TopicCondition(self, topicName, CapacitiveSensorsReadings, self.TouchCondition)
		self.add_change_condition(topicCondition, ['rfid_correct','rfid_incorrect','preempted'])

	def TouchCondition(self, data, userdata):
		userdata.touched=data
		if data.left_arm==True or data.right_arm==True:
			value='touched'
		else:
			value='not_touched'
		return value


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

# define state ENCOURAGE
class Encourage(MonarchState):

  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state ENCOURAGE")  
    greeting_name = 'mbot_encourage'
    runGreetingCeState = RunCeAction(self, greeting_name)
    self.add_action(runGreetingCeState)

# define state CATCHED
class Catched(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
	rospy.loginfo("Init state CATCHED")
    	greeting_name = 'mbot_congratulate'
    	runGreetingCeState = RunCeAction(self, greeting_name)
    	self.add_action(runGreetingCeState)

  #  	self.gestureCE = GestureExpression()
  #  	self.gestureCE.gesture = ce

  #  	self.topicWriter = rospy.Publisher('express_gesture', GestureExpression, queue_size=10)
  #  	rospy.sleep(1)

  #  def execute(self):
   #	 rate = rospy.Rate(1)

    #	self.topicWriter.publish(self.gestureCE)
    #	rate.sleep()


# define state FINISH
class Finish(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
	rospy.loginfo("Init state FINISH")
    	greeting_name = 'mbot_tell_game_has_finished'
    	runGreetingCeState = RunCeAction(self, greeting_name)
    	self.add_action(runGreetingCeState)

# define state FINISH2
class Finish2(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['finish_succeeded','finish_preempted'], input_keys=['person_rfid','action_result'],output_keys=['person_rfid','action_result'])
	rospy.loginfo('Init state FINISH2')

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH2')
	userdata.action_result.result.array[0].value = 'True'
	print userdata.person_rfid
        return 'finish_succeeded'

if __name__ == '__main__':

    rospy.init_node('behavior_catchandtouch')
    rospy.loginfo('Starting up')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['game_finished','preempted','aborted'],
                          input_keys = [], 
                          output_keys = ['action_result'])

    sm.userdata.distance=2.0
    sm.userdata.cannot_read=False
    sm.userdata.final_position=Pose2D()
    sm.userdata.init_person_position=Pose2D()
    sm.userdata.person_position=Pose2D()
    sm.userdata.robot_position=Pose2D()
    sm.userdata.person_rfid=RfidReading()
    sm.userdata.touched=CapacitiveSensorsReadings()

    sm.userdata.action_result = CatchAndTouchResult()
    result_reached = KeyValuePair()
    result_reached.key = 'finished'
    result_reached.value = 'False'
    sm.userdata.action_result.result.array.append(result_reached)

    # Open the container
    with sm:
        # Add states to the container
        sm.add('START', Start(), transitions={'succeeded':'LOOKHUMAN1', 'preempted':'preempted'})

	sm.add('LOOKHUMAN1', LookHuman(), transitions={'succeeded':'SET', 'preempted':'preempted'})

        sm.add('SET', Set(), 
                               transitions={'final_position_set':'APPROACH', 'too_near':'FINISH2', 'preempted':'preempted'})


    	sm10 = smach.StateMachine(outcomes = ['preempted','person_moving'])
	sm10.userdata=sm.userdata
   	with sm10:
      		sm10.add('LOOKHUMAN10', LookHuman(),
                        transitions = { 'preempted':'preempted','succeeded':'COMPARE10'})
		sm10.add('COMPARE10', Compare(),
			transitions = {'preempted':'preempted','person_moving':'person_moving','person_still':'LOOKHUMAN10'})

        def termination_cb10(outcome_map):
            if outcome_map['MOVE10'] == 'succeeded' or outcome_map['MONITOR10'] == 'person_moving' or outcome_map['MOVE10'] =='aborted' or outcome_map['READ10'] == 'read_finished': #or outcome_map['MONITOR10'] == 'person_still':
		print 'Aqui'
                return True
            else:
                return False

        sm_con10 = smach.Concurrence(outcomes=['approach_default','approach_done','position_wrong','rfid_read','approach_aborted','approach_preempted'],
                                      default_outcome='approach_default',
				      child_termination_cb = termination_cb10,
                                      outcome_map={'approach_done':
                                          {'MOVE10':'succeeded',
                                            'MONITOR10':'preempted',
					     'READ10':'preempted'},
                                              'position_wrong':
                                          {'MOVE10':'preempted',
				            'MONITOR10':'person_moving',
					     'READ10':'preempted'},
                                              'rfid_read':
                                          {'READ10':'succeeded'},
                                               'approach_aborted':
                                          {'MOVE10':'aborted',
					    'MONITOR10':'preempted',
					     'READ10':'preempted'},
                                               'approach_preempted':
                                          {'MOVE10':'preempted',
                                            'MONITOR10':'preempted',
					     'READ10':'preempted'}})
	sm_con10.userdata=sm.userdata
        with sm_con10:
            smach.Concurrence.add('MOVE10', MoveToHuman())
            smach.Concurrence.add('MONITOR10', sm10)
	    smach.Concurrence.add('READ10', Read())


        sm.add('APPROACH',sm_con10,
                                transitions={'approach_default':'APPROACH','approach_done':'GET','position_wrong':'LOOKHUMAN1','rfid_read':'GREET', 'approach_aborted':'aborted','approach_preempted':'preempted'})


        def termination_cb20(outcome_map):
            if outcome_map['READ20'] == 'read_finished' or outcome_map['WAIT20'] == 'wait_finished':
                return True
            else:
                return False

        sm_con20 = smach.Concurrence(outcomes=['get_default','get_succeeded','get_failed','get_preempted'],
                                      default_outcome='get_default',
				      output_keys=['person_rfid'],
				      child_termination_cb = termination_cb20,
                                      outcome_map={'get_succeeded':
                                          {'READ20':'succeeded',
                                            'WAIT20':'preempted'},
                                              'get_failed':
                                          {'READ20':'preempted',
				            'WAIT20':'wait_finished'},
                                               'get_preempted':
                                          {'READ20':'preempted',
                                            'WAIT20':'preempted'}})	
	sm_con20.userdata=sm.userdata
        with sm_con20:
            smach.Concurrence.add('READ20', Read())
            smach.Concurrence.add('WAIT20', Wait())

        sm.add('GET',sm_con20,
                                transitions={'get_default':'GET','get_succeeded':'GREET','get_failed':'SET','get_preempted':'preempted'})

        sm.add('GREET', Greet(), transitions={'succeeded':'WAIT2'})

        sm.add('WAIT2', WaitMore(), transitions={'wait_finished':'SENDINVITATION', 'preempted':'preempted'})
	
	sm.add('SENDINVITATION', SendInvitation(), transitions={'succeeded':'RECEIVEREPLY'})

	sm.add('RECEIVEREPLY', ReceiveReply(), transitions={'succeeded':'GAME','aborted':'FINISH'})

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

    	sm6 = smach.StateMachine(outcomes = ['succeeded','preempted'])
	sm6.userdata=sm.userdata
   	with sm6:
      		sm6.add('CHECK_SUB', Check(),
                        transitions = { 'touched':'succeeded','not_touched':'CHECK_SUB','preempted':'preempted'})

        def termination_cb1(outcome_map):
            if outcome_map['MOVETOEND'] == 'succeeded' or outcome_map['MONITOR'] == 'person_too_far' or outcome_map['MONITOR'] == 'person_following' or outcome_map['MONITOR'] == 'person_lost'or outcome_map['MOVETOEND'] == 'aborted' or outcome_map['CHECK'] == 'succeeded':
                return True
            else:
                return False

        sm_con1 = smach.Concurrence(outcomes=['lead_default','lead_done','lead_catched','look_for_person','lead_aborted','lead_preempted'],
                                      default_outcome='lead_default',
				      child_termination_cb = termination_cb1,
                                      outcome_map={'lead_done':
                                          {'MOVETOEND':'succeeded',
						'CHECK':'preempted'},
						'lead_catched':
                                          {'CHECK':'succeeded'},
                                              'look_for_person':
                                          {'MOVETOEND':'preempted',
				            'MONITOR':'person_too_far',
						'CHECK':'preempted'},
                                               'lead_aborted':
                                          {'MOVETOEND':'aborted'},
                                               'lead_preempted':
                                          {'MOVETOEND':'preempted',
                                            'MONITOR':'preempted',
						'CHECK':'preempted'}})
	sm_con1.userdata=sm.userdata
        with sm_con1:
            smach.Concurrence.add('MOVETOEND', MoveToEnd())
            smach.Concurrence.add('MONITOR', sm3)
	    smach.Concurrence.add('CHECK',sm6)
	    smach.Concurrence.add('ACCELERATE',Accelerate())

    	sm4 = smach.StateMachine(outcomes = ['rfid_correct','preempted'])
	sm4.userdata=sm.userdata
   	with sm4:
      		sm4.add('RFID_COMPARE2', Rfid_Compare(),
                        transitions = { 'rfid_correct':'rfid_correct','rfid_incorrect':'RFID_COMPARE2','preempted':'preempted'})

    	sm5 = smach.StateMachine(outcomes = ['preempted','person_near'])
	sm5.userdata=sm.userdata
   	with sm5:
      		sm5.add('LOOKHUMAN2', LookHuman(),
                        transitions = { 'preempted':'preempted','succeeded':'LOOKROBOT2'})
		sm5.add('LOOKROBOT2', LookRobot(),
			transitions = {'preempted':'preempted','person_too_far':'LOOKHUMAN2','person_following':'person_near'})


        def termination_cb3(outcome_map):
            if outcome_map['MONITOR_RFID2'] == 'rfid_correct' or outcome_map['MONITOR_POS2'] == 'person_near' or outcome_map['WAITMORE'] == 'wait_finished':
                return True
            else:
                return False

	sm_con3 = smach.Concurrence(outcomes=['person_detected_pos','person_detected_rfid','person_lost','monitor_default','monitor_preempted'],
                                      default_outcome='monitor_default',
				      child_termination_cb = termination_cb3,
                                      outcome_map={'person_detected_pos':
                                          {'MONITOR_RFID2':'preempted',
                                            'MONITOR_POS2':'person_near'},
                                              'person_detected_rfid':
                                          {'MONITOR_RFID2':'rfid_correct'},
						'person_lost':
                                          {'MONITOR_RFID2':'preempted',
                                            'MONITOR_POS2':'preempted',
					     'WAITMORE':'wait_finished'},
						'monitor_preempted':
                                          {'MONITOR_RFID2':'preempted',
                                            'MONITOR_POS2':'preempted',
					     'WAITMORE':'preempted'}})
	sm_con3.userdata=sm.userdata
        with sm_con3:
            smach.Concurrence.add('MONITOR_RFID2', sm4)
            smach.Concurrence.add('MONITOR_POS2', sm5)
	    smach.Concurrence.add('WAITMORE',WaitMore())


        sm.add('GAME',sm_con1, transitions={'lead_default':'GAME','lead_done':'FINISH','lead_catched':'CATCHED','look_for_person':'ENCOURAGE','lead_aborted':'aborted','lead_preempted':'preempted'})

        sm.add('ENCOURAGE', Encourage(), transitions={'succeeded':'WAITING_PERSON'})

        sm.add('WAITING_PERSON',sm_con3, transitions={'person_detected_pos':'GAME','person_detected_rfid':'GAME','person_lost':'aborted','monitor_preempted':'preempted','monitor_default':'WAITING_PERSON'})

	sm.add('CATCHED', Catched(), transitions={'succeeded':'WAIT3'})

	sm.add('WAIT3', WaitMore(), transitions={'wait_finished':'FINISH', 'preempted':'preempted'})

        sm.add('FINISH', Finish(), transitions={'succeeded':'FINISH2'})

        sm.add('FINISH2', Finish2(), transitions={'finish_succeeded':'game_finished','finish_preempted':'preempted'})

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
    action_name = rospy.get_name()  
    asw = smach_ros.ActionServerWrapper(action_name,
                                      CatchAndTouchAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['game_finished'],
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

