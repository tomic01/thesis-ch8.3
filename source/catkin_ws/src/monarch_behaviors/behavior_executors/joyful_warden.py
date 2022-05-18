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
from monarch_behaviors.msg import CatchAndTouchAction, CatchAndTouchResult, CatchAndTouchGoal

from mosmach.util import pose2pose_stamped

from sam_helpers.reader import SAMReader
from sam_helpers.writer import SAMWriter

from tf import *
import tf
import numpy

catched=False # this variable defines if the robot has been catched by the child


# define state FINDHUMANSFROMSAM
# this state collects all the human ids from the SAM slot "People_Localization_Tracker_2"
class FindHumanFromSAM(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','failed','preempted'], input_keys=['action_goal'],output_keys=['ident_array'])
        	rospy.loginfo("Init state FINDHUMANSFROMSAM")
		self.id_array=[]			
		self.position_detected=False

  	def people_localization_tracker_cb(self, data):
		for user in range(0,len(data.locations)):
			if data.locations[user].id > 1000:
				self.id_array.append(data.locations[user].id)
				self.position_detected=True

  	def execute(self, userdata):
    		rospy.loginfo('Executing state FINDHUMANSFROMSAM')
		r = rospy.Rate(2)
			
    		sam_reader= SAMReader("People_Localization_Tracker_2",self.people_localization_tracker_cb)
		t=0
    		while not rospy.is_shutdown() and self.position_detected is False and t<20:
      			if self.preempt_requested():
				sam_reader.remove()
        			self.service_preempt()
        			return 'preempted'
      			r.sleep()
			t=t+1
    		sam_reader.remove()
		if self.position_detected is True:
			userdata.ident_array=self.id_array
	    		return 'succeeded'
		else:
			return 'failed'

# define state NOMATETOPLAYWITH
# this state is to run a CE to express that there is not any child to play with
# it is required to decide which CE to use. CE 'no_mate_to_play_with' does noy exist
class NoMateToPlayWith(MonarchState):
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state NOMATETOPLAYWITH")  
    ce_name = 'no_mate_to_play_with'
    runCeState = RunCeAction(self, ce_name)
    self.add_action(runCeState)

# define state SELECTACHILD
# this state to decide which the child id
# it is required to define the rules to choose the child. Now it is selecting the first in the child_array
class SelectAChild(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','failed','preempted'], input_keys=['action_goal','ident_array'],output_keys=['ident_child'])
        	rospy.loginfo("Init state SELECTACHILD")
		
		self.id_child=0			

  	def execute(self, userdata):

    		rospy.loginfo('Executing state SELECTACHILD')

		if len(userdata.ident_array)>0:
			self.id_child=userdata.ident_array[0]
			return 'succeeded'
		else:
			return 'failed'

# define state IMCONFUSED
# this state is to run a CE to express that robot is confused and can not choose any child id
# it is required to decide which CE to use. CE 'im_confused' does noy exist
class ImConfused(MonarchState):
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state IMCONFUSED")  
    ce_name = 'im_confused'
    runCeState = RunCeAction(self, ce_name)
    self.add_action(runCeState)


# define state GETCHILDPOSITION
# this state is to calculate the child position from the SAM slot and the ident_child
class GetChildPosition(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['ident_child','action_goal'],output_keys=['child_position'])
        	rospy.loginfo("Init state GETCHILDPOSITION")
		self.position=Pose2D()
		self.position_detected=False


  	def people_localization_tracker_cb(self, data):
		for user in range(0,len(data.locations)):
			if data.locations[user].id == self.ident:
				self.user_id=user
				self.position.x=data.locations[self.user_id].pose.pose.position.x
				self.position.y=data.locations[self.user_id].pose.pose.position.y
				self.position.theta=0.0
				self.position_detected=True	

  	def execute(self, userdata):
    		rospy.loginfo('Executing state GETCHILDPOSITION')
		r = rospy.Rate(2)
		self.ident=userdata.ident_child
    		sam_reader= SAMReader("People_Localization_Tracker_2",self.people_localization_tracker_cb)

    		while not rospy.is_shutdown() and self.position_detected is False:
      			if self.preempt_requested():
				sam_reader.remove()
        			self.service_preempt()
        			return 'preempted'
      			r.sleep()

		userdata.child_position=self.position

    		sam_reader.remove()
    		return 'succeeded'


# define state COMPUTEKIDSNEIGHBORHOOD
# this state is to calculate the best neighborhood position around the chosen child
# this should be calculated based on a camera view from the child position, but meanwhile we use directly to a position to two meters from child position 
class ComputeKidsNeighborhood(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['child_position','neighborhood_position'],output_keys=['neighborhood_position'])
        	rospy.loginfo("Init state COMPUTEKIDSNEIGHBORHOOD")
        	TopicName = 'amcl_pose'

		topicCondition2 = TopicCondition(self, TopicName,PoseWithCovarianceStamped,self.PoseCondition)
		self.add_change_condition(topicCondition2, ['succeeded','preempted'])

	def PoseCondition(self, data, userdata):
		userdata.neighborhood_position.x = data.pose.pose.position.x
		userdata.neighborhood_position.y = data.pose.pose.position.y
		userdata.neighborhood_position.theta = 0.0

		angle=atan2((userdata.child_position.y-data.pose.pose.position.y),(userdata.child_position.x-data.pose.pose.position.x))
	
		userdata.neighborhood_position.theta=angle

		angle_person=angle+pi
		while angle_person>2*pi:
			angle_person=angle_person-2*pi
		while angle_person<0:
			angle_person=angle_person+2*pi

		while userdata.neighborhood_position.theta>2*pi:
			userdata.neighborhood_position.theta=userdata.neighborhood_position.theta-2*pi
		while userdata.neighborhood_position.theta<0:
			userdata.neighborhood_position.theta=userdata.neighborhood_position.theta+2*pi
		userdata.neighborhood_position.x=userdata.child_position.x+2.0*cos(angle_person)
		userdata.neighborhood_position.y=userdata.child_position.y+2.0*sin(angle_person)
		print userdata.neighborhood_position
		return 'succeeded'


# define state MOVETONEIGHBORHOOD
#this state is to move the robot to the neighborhood position
class MoveToNeighborhood(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['neighborhood_position'],output_keys=[''])
	rospy.loginfo('Init state MOVETONEIGHBORHOOD')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	return pose2pose_stamped(userdata.neighborhood_position.x, userdata.neighborhood_position.y, userdata.neighborhood_position.theta)


# define state COMPUTEBESTAPPROACH
# this state is to calculate the best approach to the child from proxemics rules
#meanwhile, we use directly the child position
class ComputeBestApproach(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['succeeded','preempted'], input_keys=['child_position','action_goal'],output_keys=['approach_position'])
        	rospy.loginfo("Init state COMPUTEBESTAPPROACH")
		self.position=Pose2D()

  	def execute(self, userdata):
    		rospy.loginfo('Executing state COMPUTEBESTAPPROACH')	
		self.position=userdata.child_position
		userdata.approach_position=self.position
    		return 'succeeded'

# define state CANTGETTOTHEKID
# this state is to run a CE to express that robot cannot get the child
# it is required to decide which CE to use. CE 'cannot_get_the_kid' does noy exist
class CantGetToTheKid(MonarchState):

  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state CANTGETTOTHEKID")  
    ce_name = 'cannot_get_the_kid'
    runCeState = RunCeAction(self, ce_name)
    self.add_action(runCeState)

# define state MOVETOCHILD
#this state is to move the robot to the child position
class MoveToChild(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['approach_position'],output_keys=[''])
	rospy.loginfo('Init state MOVETOCHILD')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	return pose2pose_stamped(userdata.approach_position.x, userdata.approach_position.y, userdata.approach_position.theta)

# define state COMPARECHILDPOS
# this state is to check if the selected child is moving from the SAM slot and the ident_child
class CompareChildPos(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['child_moving','aborted','preempted'], input_keys=['child_position','ident_child'],output_keys=['child_position'])
        	rospy.loginfo("Init state COMPARECHILDPOS")

		self.position=Pose2D()
		self.position_changed=False


  	def people_localization_tracker_cb(self, data):
		for user in range(0,len(data.locations)):
			if data.locations[user].id == self.ident:
       				self.user_id=user

				if sqrt((data.locations[self.user_id].pose.pose.position.x-self.position.x)*(data.locations[self.user_id].pose.pose.position.x-self.position.x)+(data.locations[self.user_id].pose.pose.position.y-self.position.y)*(data.locations[self.user_id].pose.pose.position.y-self.position.y))>0.5 or fabs(data.locations[self.user_id].pose.pose.position.theta-self.position.theta)>pi/4:
					self.position.x=data.locations[self.user_id].pose.pose.position.x
					self.position.y=data.locations[self.user_id].pose.pose.position.y
					self.position.theta=data.locations[self.user_id].pose.pose.position.theta
					self.position_changed=True	

  	def execute(self, userdata):
    		rospy.loginfo('Executing state COMPARECHILDPOS')
		r = rospy.Rate(2)

		self.ident=userdata.ident_child
		self.position=userdata.child_position
    		sam_reader= SAMReader("People_Localization_Tracker_2",self.people_localization_tracker_cb)
    
    		while not rospy.is_shutdown() and self.position_changed is False:
      			if self.preempt_requested():
				sam_reader.remove()
        			self.service_preempt()
        			return 'preempted'
      			r.sleep()
    		sam_reader.remove()
		if self.position_changed is True:
			userdata.child_position=self.position
    			return 'child_moving'
		else:
			return 'aborted'

# define state READRFID
# this state is to read a rfid tag
class ReadRFID(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted'],input_keys=[''], output_keys=['child_rfid'])
		rospy.loginfo("Init state READRFID")
        	topicName = 'rfid_tag'
		self.goalReader = TopicReaderAction(self, topicName,RfidReading,self.RfidCondition)
		self.add_action(self.goalReader)

	def RfidCondition(self, data, userdata):
		userdata.child_rfid.tag_id=data.tag_id
		value = True
		return value

# define state WAIT2SECONDS
# this state is to wait 2 seconds to read the RFID tag
class Wait2Seconds(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['wait_finished','preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state WAIT2SECONDS')

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT2SECONDS')
	for t in range(20):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
        return 'wait_finished'

# define state GREET
# this state is to greet the child
class Greet(MonarchState):
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state GREET")  
    greeting_name = 'give_greetings_child'
    runGreetingCeState = RunCeAction(self, greeting_name)
    self.add_action(runGreetingCeState)

# define state SENDINVITATION
# this state is to ask the child to play the game
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
# this state is to receive a reply from the child
class ReceiveReply(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'],input_keys=['child_rfid'], output_keys=['child_rfid'])
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

# define state WAIT30SECONDS
# this state is to wait 30 seconds while receiving a reply
class Wait30Seconds(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['wait_finished','preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state WAIT30SECONDS')

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT30SECONDS')
	for t in range(300):
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
        return 'wait_finished'

# define state YOUDONTPLAY
# this state is to run a CE to express that the child does not want to play with the robot
# it is required to decide which CE to use. CE 'you_dont_want_to_play' does noy exist
class YouDontWantToPlay(MonarchState):
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state YOUDONTPLAY")  
    ce_name = 'you_dont_want_to_play'
    runCeState = RunCeAction(self, ce_name)
    self.add_action(runCeState)

# define state MOVETOEND
# this state is to move to the final position in the game
class MoveToEnd(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['action_goal'],output_keys=[''])
	rospy.loginfo('Init state MOVETOEND')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
        float(userdata.action_goal.goal.array[1].value)
	return pose2pose_stamped(float(userdata.action_goal.goal.array[1].value), float(userdata.action_goal.goal.array[2].value), float(userdata.action_goal.goal.array[3].value))

# define state LOOKROBOT
# this state is to get the robot position and compare with child position
class LookRobot(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['person_too_far','person_following','preempted'], input_keys=['robot_position','child_position'],output_keys=['robot_position','child_position'])
        	rospy.loginfo("Init state LOOKROBOT")
        	TopicName = 'amcl_pose'
		topicCondition2 = TopicCondition(self, TopicName,PoseWithCovarianceStamped,self.PoseCondition)
		self.add_change_condition(topicCondition2, ['person_too_far','person_following','preempted'])

	def PoseCondition(self, data, userdata):
		userdata.robot_position.x = data.pose.pose.position.x
		userdata.robot_position.y = data.pose.pose.position.y
        	q = (data.pose.pose.orientation.x, 
        	data.pose.pose.orientation.y,
        	data.pose.pose.orientation.z,
        	data.pose.pose.orientation.w)
        	euler = tf.transformations.euler_from_quaternion(q)
        	roll= euler[0]
        	pitch = euler[1]
        	yaw = euler[2]
		userdata.robot_position.theta = yaw
		if sqrt((userdata.child_position.x-userdata.robot_position.x)*(userdata.child_position.x-userdata.robot_position.x)+(userdata.child_position.y-userdata.robot_position.y)*(userdata.child_position.y-userdata.robot_position.y))>4.0:
			value = 'person_too_far'
		else: 
			value = 'person_following'
		return value

# define state RFID_COMPARE
# this state is to compare the detected rfid tag with the child rfid tag
class Rfid_Compare(MonarchState):
	def __init__(self):
		MonarchState.__init__(self, state_outcomes=['rfid_correct','rfid_incorrect','preempted'],input_keys=['child_rfid'], output_keys=[''])
		rospy.loginfo("Init state RFID_COMPARE")
        	topicName = 'rfid_tag'

		topicCondition = TopicCondition(self, topicName, RfidReading, self.RfidCondition)
		self.add_change_condition(topicCondition, ['rfid_correct','rfid_incorrect','preempted'])

	def RfidCondition(self, data, userdata):
		if userdata.child_rfid.tag_id==data.tag_id:
			value='rfid_correct'
		else:
			value='rfid_incorrect'
		return value

# define state ACCELERATE
# this state is to control the robot speed
class Accelerate(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['preempted','catched'], input_keys=['action_goal'],output_keys=[''])
	rospy.loginfo('Init state ACCELERATE')
	self.pub = rospy.Publisher('navigator/param/vref', Float64, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Executing state ACCELERATE')
	while not rospy.is_shutdown() and catched==False:
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		self.pub.publish(float(userdata.action_goal.goal.array[4].value))
		rospy.sleep(0.1)
	for t in range(5):
		self.pub.publish(0.8*float(userdata.action_goal.goal.array[4].value))
		rospy.sleep(0.1)
	for t in range(5):
		self.pub.publish(0.6*float(userdata.action_goal.goal.array[4].value))
		rospy.sleep(0.1)
	for t in range(5):
		self.pub.publish(0.4*float(userdata.action_goal.goal.array[4].value))
		rospy.sleep(0.1)
	for t in range(5):
		self.pub.publish(0.2*float(userdata.action_goal.goal.array[4].value))
		rospy.sleep(0.1)
        return 'catched'

# define state WAITPREEMPT
# this state is to wait a preempt request during one second
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
# this state is to check if the child has touched the robot arms
class Check(MonarchState):
    	def __init__(self):
        	MonarchState.__init__(self, state_outcomes=['touched','not_touched','preempted'], input_keys=[''],output_keys=[''])
        	rospy.loginfo("Init state CHECK")
        	topicName = 'cap_sensors'
		topicCondition = TopicCondition(self, topicName, CapacitiveSensorsReadings, self.TouchCondition)
		self.add_change_condition(topicCondition, ['touched','not_touched','preempted'])

	def TouchCondition(self, data, userdata):
		if data.left_arm==True or data.right_arm==True:
			value='touched'
		else:
			value='not_touched'
		return value

# define state TOUCHED
# this state changes the catched variable to True and waits to be preempted
class Touched(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['preempted'], input_keys=[''],output_keys=[''])
	rospy.loginfo('Init state TOUCHED')

    def execute(self, userdata):
        rospy.loginfo('Executing state TOUCHED')
	global catched
	catched=True
	while not rospy.is_shutdown():
  		if self.preempt_requested():
        		self.service_preempt()
               		return 'preempted'
		rospy.sleep(0.1)
        return 'preempted'

# define state TURNAROUND
# this state turns around the robot
class TurnAround(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded','preempted','aborted'], input_keys=['robot_position'],output_keys=[''])
	rospy.loginfo('Init state TURNAROUND')
	navMoveToPersonAction = MoveToAction(self, data_cb=self.goal_cb, is_dynamic=True)
	self.add_action(navMoveToPersonAction)

    def goal_cb(self,userdata):
	angle_robot=userdata.robot_position.theta+pi
	while angle_robot>2*pi:
		angle_robot=angle_robot-2*pi
	while angle_robot<0:
		angle_robot=angle_robot+2*pi
	return pose2pose_stamped(userdata.robot_position.x, userdata.robot_position.y, angle_robot)

# define state WAITMORE
# this state is to wait 10 seconds
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
# this state is to ecourage the child to continue playing
class Encourage(MonarchState):
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state ENCOURAGE")  
    greeting_name = 'mbot_encourage'
    runGreetingCeState = RunCeAction(self, greeting_name)
    self.add_action(runGreetingCeState)

# define state CONGRATULATE
# this state is to congratulate the child when the robot is catched
class Congratulate(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
	rospy.loginfo("Init state CONGRATULATE")
    	greeting_name = 'mbot_congratulate'
    	runGreetingCeState = RunCeAction(self, greeting_name)
    	self.add_action(runGreetingCeState)

# define state GAMEFINISH
# this state is to say that game has finished
class GameFinish(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
	rospy.loginfo("Init state GAMEFINISH")
    	greeting_name = 'mbot_tell_game_has_finished'
    	runGreetingCeState = RunCeAction(self, greeting_name)
    	self.add_action(runGreetingCeState)

# define state FINISH
# this state is to send the results when the game finishes properly
class Finish(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['finish_succeeded','finish_preempted'], input_keys=['child_rfid','action_result'],output_keys=['child_rfid','action_result'])
	rospy.loginfo('Init state FINISH')

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH')
	if len(userdata.action_result.result.array)>0:
		userdata.action_result.result.array[0].key = 'finished'
		userdata.action_result.result.array[0].value = 'True'
	else:
    		result_reached = KeyValuePair()
    		result_reached.key = 'finished'
    		result_reached.value = 'True'
    		userdata.action_result.result.array.append(result_reached)
	print userdata.child_rfid
        return 'finish_succeeded'

# define state CHILDLOST
# this state is to run a CA to express that the robot has lost the child
# it is required to decide which CA to use. CA 'i_lost_the_child' does noy exist
class ChildLost(MonarchState):
  def __init__(self):
    MonarchState.__init__(self, state_outcomes=['succeeded'], input_keys=[''],output_keys=[''])
    rospy.loginfo("Init state CHILDLOST")  
    ca_name = 'i_lost_the_child'
    runCaState = RunCaAction(self, ca_name)
    self.add_action(runCaState)

# define state ABORT
# this state is to send the results when the game is aborted
class Abort(MonarchState):
    def __init__(self):
        MonarchState.__init__(self, state_outcomes=['abort_succeeded','abort_preempted'], input_keys=['action_result'],output_keys=['action_result'])
	rospy.loginfo('Init state ABORT')

    def execute(self, userdata):
        rospy.loginfo('Executing state ABORT')
	if len(userdata.action_result.result.array)>0:
		userdata.action_result.result.array[0].key = 'finished'
		userdata.action_result.result.array[0].value = 'False'
	else:
    		result_reached = KeyValuePair()
    		result_reached.key = 'finished'
    		result_reached.value = 'False'
    		userdata.action_result.result.array.append(result_reached)
        return 'abort_succeeded'


if __name__ == '__main__':
    rospy.init_node('behavior_joyfulwarden')
    rospy.loginfo('Starting up')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['joyful_finished','preempted','aborted'],
                          input_keys = ['action_goal'], 
                          output_keys = ['action_result'])
    sm.userdata.ident_array=[]
    sm.userdata.ident_child=-1
    sm.userdata.child_position=Pose2D()
    sm.userdata.neighborhood_position=Pose2D()
    sm.userdata.approach_position=Pose2D()
    sm.userdata.robot_position=Pose2D()
    sm.userdata.child_rfid=RfidReading()
    sm.userdata.touched=CapacitiveSensorsReadings()
    sm.userdata.action_result = CatchAndTouchResult()


    # Open the container
    with sm:
        # Add states to the container
        sm.add('FINDHUMANSFROMSAM', FindHumanFromSAM(), transitions={'succeeded':'SELECTACHILD', 'failed':'NOMATETOPLAYWITH', 'preempted':'preempted'})
	sm.add('NOMATETOPLAYWITH', NoMateToPlayWith(), transitions={'succeeded':'joyful_finished'})
	sm.add('SELECTACHILD', SelectAChild(), transitions={'succeeded':'GETCHILDPOSITION','failed':'IMCONFUSED','preempted':'preempted'})
	sm.add('IMCONFUSED', ImConfused(), transitions={'succeeded':'NOMATETOPLAYWITH'})
	sm.add('GETCHILDPOSITION', GetChildPosition(), transitions={'succeeded':'COMPUTEKIDSNEIGHBORHOOD','preempted':'preempted'})
	sm.add('COMPUTEKIDSNEIGHBORHOOD', ComputeKidsNeighborhood(), transitions={'succeeded':'ROUTETONEIGHBORHOOD','preempted':'preempted'})

        def termination_cb_neighborhood(outcome_map):
            if outcome_map['MOVETONEIGHBORHOOD'] == 'aborted':
                return True
            else:
                return False

        sm_neighborhood = smach.Concurrence(outcomes=['succeeded','preempted','aborted','default'],
                                      default_outcome='default',
				      child_termination_cb = termination_cb_neighborhood,
                                      outcome_map={'succeeded':
                                          {'MOVETONEIGHBORHOOD':'succeeded',
                                            'COMPUTEBESTAPPROACH':'succeeded'},
                                              'preempted':
                                          {'MOVETONEIGHBORHOOD':'preempted',
                                            'COMPUTEBESTAPPROACH':'preempted'},
                                              'aborted':
                                          {'MOVETONEIGHBORHOOD':'aborted'}})
	sm_neighborhood.userdata=sm.userdata
        with sm_neighborhood:
            smach.Concurrence.add('MOVETONEIGHBORHOOD', MoveToNeighborhood())
            smach.Concurrence.add('COMPUTEBESTAPPROACH', ComputeBestApproach())

        sm.add('ROUTETONEIGHBORHOOD',sm_neighborhood,
                                transitions={'succeeded':'APPROACH','preempted':'preempted','aborted':'CANTGETTOTHEKID','default':'ROUTETONEIGHBORHOOD'})

	sm.add('CANTGETTOTHEKID', CantGetToTheKid(), transitions={'succeeded':'joyful_finished'})

        def termination_cb_approach(outcome_map):
            if outcome_map['MOVETOCHILD'] == 'succeeded' or outcome_map['COMPARECHILDPOS'] == 'child_moving' or outcome_map['MOVETOCHILD'] =='aborted' or outcome_map['READRFID'] == 'succeeded': 
                return True
            else:
                return False

        sm_approach = smach.Concurrence(outcomes=['default','approach_done','position_wrong','rfid_read','approach_aborted','approach_preempted'],
                                      default_outcome='default',
				      child_termination_cb = termination_cb_approach,
                                      outcome_map={'approach_done':
                                          {'MOVETOCHILD':'succeeded',
                                            'COMPARECHILDPOS':'preempted',
					     'READRFID':'preempted'},
                                              'position_wrong':
                                          {'MOVETOCHILD':'preempted',
				            'COMPARECHILDPOS':'child_moving',
					     'READRFID':'preempted'},
                                              'rfid_read':
                                          {'READRFID':'succeeded'},
                                               'approach_aborted':
                                          {'MOVETOCHILD':'aborted',
					    'COMPARECHILDPOS':'preempted',
					     'READRFID':'preempted'},
                                               'approach_preempted':
                                          {'MOVETOCHILD':'preempted',
                                            'COMPARECHILDPOS':'preempted',
					     'READRFID':'preempted'}})
	sm_approach.userdata=sm.userdata
        with sm_approach:
            smach.Concurrence.add('MOVETOCHILD', MoveToChild())
            smach.Concurrence.add('COMPARECHILDPOS', CompareChildPos())
	    smach.Concurrence.add('READRFID', ReadRFID())


        sm.add('APPROACH',sm_approach,
                                transitions={'default':'APPROACH','approach_done':'GETRFID','position_wrong':'GETCHILDPOSITION','rfid_read':'GREET', 'approach_aborted':'CANTGETTOTHEKID','approach_preempted':'preempted'})

        def termination_cb_rfid(outcome_map):
            if outcome_map['READRFID'] == 'read_finished' or outcome_map['WAIT2SECONDS'] == 'wait_finished':
                return True
            else:
                return False

        sm_rfid = smach.Concurrence(outcomes=['get_default','get_succeeded','get_failed','get_preempted'],
                                      default_outcome='get_default',
				      output_keys=['person_rfid'],
				      child_termination_cb = termination_cb_rfid,
                                      outcome_map={'get_succeeded':
                                          {'READRFID':'succeeded'},
                                              'get_failed':
                                          {'READRFID':'preempted',
				            'WAIT2SECONDS':'wait_finished'},
                                               'get_preempted':
                                          {'READRFID':'preempted',
                                            'WAIT2SECONDS':'preempted'}})	
	sm_rfid.userdata=sm.userdata
        with sm_rfid:
            smach.Concurrence.add('READRFID', ReadRFID())
            smach.Concurrence.add('WAIT2SECONDS', Wait2Seconds())

        sm.add('GETRFID',sm_rfid,
                                transitions={'get_default':'GETRFID','get_succeeded':'GREET','get_failed':'NOMATETOPLAYWITH','get_preempted':'preempted'})

        sm.add('GREET', Greet(), transitions={'succeeded':'WAITMORE'})

        sm.add('WAITMORE', WaitMore(), transitions={'wait_finished':'SENDINVITATION', 'preempted':'preempted'})
	
	sm.add('SENDINVITATION', SendInvitation(), transitions={'succeeded':'REPLY'})

        def termination_cb_reply(outcome_map):
            if outcome_map['RECEIVEREPLY'] == 'succeeded' or outcome_map['RECEIVEREPLY'] == 'aborted' or outcome_map['WAIT30SECONDS'] =='wait_finished': 
                return True
            else:
                return False

        sm_reply = smach.Concurrence(outcomes=['default','reply_yes','time_finished','reply_aborted','reply_preempted'],
                                      default_outcome='default',
				      child_termination_cb = termination_cb_reply,
                                      outcome_map={'reply_yes':
                                          {'RECEIVEREPLY':'succeeded'},
                                              'time_finished':
                                          {'WAIT30SECONDS':'wait_finished',
				            'RECEIVEREPLY':'preempted'},
                                              'reply_aborted':
                                          {'RECEIVEREPLY':'aborted'},
                                               'reply_preempted':
                                          {'WAIT30SECONDS':'preempted',
                                            'RECEIVEREPLY':'preempted'}})
	sm_reply.userdata=sm.userdata
        with sm_reply:
            smach.Concurrence.add('WAIT30SECONDS', Wait30Seconds())
            smach.Concurrence.add('RECEIVEREPLY', ReceiveReply())

        sm.add('REPLY',sm_reply,
                                transitions={'default':'REPLY','reply_yes':'GAME','time_finished':'YOUDONTPLAY','reply_aborted':'YOUDONTPLAY','reply_preempted':'preempted'})

	sm.add('YOUDONTPLAY', YouDontWantToPlay(), transitions={'succeeded':'joyful_finished'})

    	sm_distance = smach.StateMachine(outcomes = ['preempted','person_too_far','person_near'])
	sm_distance.userdata=sm.userdata
   	with sm_distance:
      		sm_distance.add('LOOKCHILDPOS', GetChildPosition(),
                        transitions = { 'preempted':'preempted','succeeded':'LOOKROBOT'})
		sm_distance.add('LOOKROBOT', LookRobot(),
			transitions = {'preempted':'preempted','person_too_far':'person_too_far','person_following':'person_near'})

    	sm_comparerfid = smach.StateMachine(outcomes = ['rfid_correct','preempted'])
	sm_comparerfid.userdata=sm.userdata
   	with sm_comparerfid:
      		sm_comparerfid.add('RFID_COMPARE', Rfid_Compare(),
                        transitions = { 'rfid_correct':'rfid_correct','rfid_incorrect':'RFID_COMPARE','preempted':'preempted'})

        def termination_cb_monitor(outcome_map):
            if outcome_map['MONITOR_POS'] == 'person_near' or outcome_map['MONITOR_POS'] == 'person_too_far' or outcome_map['MONITOR_RFID'] == 'rfid_correct' or outcome_map['WAIT'] == 'wait_finished':
                return True
            else:
                return False

	sm_monitor = smach.Concurrence(outcomes=['person_following_pos','person_following_rfid','person_too_far','person_lost','monitor_default','preempted'],
                                      default_outcome='monitor_default',
				      child_termination_cb = termination_cb_monitor,
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
	sm_monitor.userdata=sm.userdata
        with sm_monitor:
            smach.Concurrence.add('MONITOR_RFID', sm_comparerfid)
            smach.Concurrence.add('MONITOR_POS', sm_distance)
	    smach.Concurrence.add('WAIT',Wait2Seconds())


    	sm_monitorandpreemt = smach.StateMachine(outcomes = ['person_too_far','preempted'])
	sm_monitorandpreemt.userdata=sm.userdata
   	with sm_monitorandpreemt:
      		sm_monitorandpreemt.add('MONITOR_SUB', sm_monitor,
                        transitions = { 'person_following_pos':'WAITPREEMPT','person_following_rfid':'WAITPREEMPT','person_too_far':'person_too_far','person_lost':'person_too_far','monitor_default':'MONITOR_SUB','preempted':'preempted'})
		sm_monitorandpreemt.add('WAITPREEMPT', WaitPreempt(),
			transitions = {'succeeded':'MONITOR_SUB','preempted':'preempted'})

    	sm_touched = smach.StateMachine(outcomes = ['preempted'])
	sm_touched.userdata=sm.userdata
   	with sm_touched:
      		sm_touched.add('CHECK_SUB', Check(),
                        transitions = {'touched':'TOUCHED','not_touched':'CHECK_SUB','preempted':'preempted'})
      		sm_touched.add('TOUCHED', Touched(),
                        transitions = {'preempted':'preempted'})

        def termination_cb_game(outcome_map):
            if outcome_map['MOVETOEND'] == 'succeeded' or outcome_map['MONITOR'] == 'person_too_far' or outcome_map['MONITOR'] == 'person_following' or outcome_map['MONITOR'] == 'person_lost'or outcome_map['MOVETOEND'] == 'aborted' or outcome_map['ACCELERATE'] == 'catched':
                return True
            else:
                return False

        sm_game = smach.Concurrence(outcomes=['lead_default','lead_done','lead_catched','look_for_person','lead_aborted','lead_preempted'],
                                      default_outcome='lead_default',
				      child_termination_cb = termination_cb_game,
                                      outcome_map={'lead_done':
                                          {'MOVETOEND':'succeeded'},
						'lead_catched':
                                          {'ACCELERATE':'catched'},
                                              'look_for_person':
                                          {'MOVETOEND':'preempted',
				            'MONITOR':'person_too_far',
						'ACCELERATE':'preempted'},
                                               'lead_aborted':
                                          {'MOVETOEND':'aborted'},
                                               'lead_preempted':
                                          {'MOVETOEND':'preempted',
                                            'MONITOR':'preempted',
						'CHECK':'preempted',
					   'ACCELERATE':'preempted'}})
	sm_game.userdata=sm.userdata
        with sm_game:
            smach.Concurrence.add('MOVETOEND', MoveToEnd())
            smach.Concurrence.add('MONITOR', sm_monitorandpreemt)
	    smach.Concurrence.add('CHECK',sm_touched)
	    smach.Concurrence.add('ACCELERATE',Accelerate())

    	sm_comparerfid2 = smach.StateMachine(outcomes = ['rfid_correct','preempted'])
	sm_comparerfid2.userdata=sm.userdata
   	with sm_comparerfid2:
      		sm_comparerfid2.add('RFID_COMPARE2', Rfid_Compare(),
                        transitions = { 'rfid_correct':'rfid_correct','rfid_incorrect':'RFID_COMPARE2','preempted':'preempted'})

    	sm_distance2 = smach.StateMachine(outcomes = ['preempted','person_too_far','person_near'])
	sm_distance2.userdata=sm.userdata
   	with sm_distance2:
      		sm_distance2.add('LOOKCHILDPOS2', GetChildPosition(),
                        transitions = { 'preempted':'preempted','succeeded':'LOOKROBOT2'})
		sm_distance2.add('LOOKROBOT2', LookRobot(),
			transitions = {'preempted':'preempted','person_too_far':'LOOKCHILDPOS2','person_following':'person_near'})


        def termination_cb_waitchild(outcome_map):
            if outcome_map['MONITOR_RFID2'] == 'rfid_correct' or outcome_map['MONITOR_POS2'] == 'person_near' or outcome_map['WAITMORE'] == 'wait_finished':
                return True
            else:
                return False

	sm_waitchild = smach.Concurrence(outcomes=['person_detected_pos','person_detected_rfid','person_lost','monitor_default','monitor_preempted'],
                                      default_outcome='monitor_default',
				      child_termination_cb = termination_cb_waitchild,
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
	sm_waitchild.userdata=sm.userdata
        with sm_waitchild:
            smach.Concurrence.add('MONITOR_RFID2', sm_comparerfid2)
            smach.Concurrence.add('MONITOR_POS2', sm_distance2)
	    smach.Concurrence.add('WAITMORE',WaitMore())


        sm.add('GAME',sm_game, transitions={'lead_default':'GAME','lead_done':'LOOK_FINISH','lead_catched':'LOOK_CATCHED','look_for_person':'LOOK_ENCOURAGE','lead_aborted':'ABORT','lead_preempted':'preempted'})

	sm.add('LOOK_FINISH', LookRobot(),transitions ={'preempted':'preempted','person_too_far':'TURN_FINISH','person_following':'TURN_FINISH'})
	sm.add('TURN_FINISH', TurnAround(), transitions={'succeeded':'GAMEFINISH','preempted':'preempted','aborted':'ABORT'})

	sm.add('LOOK_CATCHED', LookRobot(),transitions ={'preempted':'preempted','person_too_far':'TURN_CATCHED','person_following':'TURN_CATCHED'})
	sm.add('TURN_CATCHED', TurnAround(), transitions={'succeeded':'CONGRATULATE','preempted':'preempted','aborted':'ABORT'})

	sm.add('LOOK_ENCOURAGE', LookRobot(),transitions ={'preempted':'preempted','person_too_far':'TURN_ENCOURAGE','person_following':'GAME'})
	sm.add('TURN_ENCOURAGE', TurnAround(), transitions={'succeeded':'ENCOURAGE','preempted':'preempted','aborted':'ABORT'})

        sm.add('ENCOURAGE', Encourage(), transitions={'succeeded':'WAITING_PERSON'})

        sm.add('WAITING_PERSON',sm_waitchild, transitions={'person_detected_pos':'GAME','person_detected_rfid':'GAME','person_lost':'CHILDLOST','monitor_preempted':'preempted','monitor_default':'WAITING_PERSON'})

	sm.add('CONGRATULATE', Congratulate(), transitions={'succeeded':'WAIT3'})

	sm.add('WAIT3', WaitMore(), transitions={'wait_finished':'SENDINVITATION', 'preempted':'preempted'})

        sm.add('GAMEFINISH', GameFinish(), transitions={'succeeded':'FINISH'})

        sm.add('FINISH', Finish(), transitions={'finish_succeeded':'joyful_finished','finish_preempted':'preempted'})

	sm.add('CHILDLOST',ChildLost(),transitions={'succeeded':'ABORT'})

        sm.add('ABORT', Abort(), 
                               transitions={'abort_succeeded':'aborted', 'abort_preempted':'preempted'})

  # action_{goal,feedback,result} are the default values and don't need to be passed as input
    action_name = rospy.get_name()  
    asw = smach_ros.ActionServerWrapper(action_name,
                                      CatchAndTouchAction,
                                      wrapped_container = sm,
                                      succeeded_outcomes = ['joyful_finished'],
                                      preempted_outcomes = ['preempted'],
                                      aborted_outcomes = ['aborted'],
                                      goal_key ='action_goal',
                                      result_key ='action_result' )

    # Create and start the introspection server 
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()


    # Execute SMACH plan
    #outcome = sm.execute()
    asw.run_server()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    #sis.stop()

