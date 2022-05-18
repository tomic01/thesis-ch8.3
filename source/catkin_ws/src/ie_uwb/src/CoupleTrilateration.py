#!/usr/bin/env python
import roslib; roslib.load_manifest('ie_uwb')
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import Float32MultiArray

from ie_uwb.msg import uwb
from ie_uwb.msg import uwb_array

import math
import numpy
from array import array

from scipy import optimize
import tf
from random import randint
import fractions
import thread
# import mean
import variance
import rospkg



robot_pos=[0,0,0]
ctrl={}
inicial_estimation=[0,0,0]
tags_global={}				# position + variances + distances
avg_robot_pos=[]
position_stats=[None,None,None]



def config_tags(): 			# Creates data structure: tags_global: position + variances + distances
	global tags_global
	for tag in tags_config:
		tags_global[tag['id']]={"pos":Vector3()}
		tags_global[tag['id']]["pos"].z=tag['z']
		tags_global[tag['id']]["pos"].y=tag['y']
		tags_global[tag['id']]["pos"].x=tag['x']
		tags_global[tag['id']]["anchors_variance"]={}
		tags_global[tag['id']]["anchors_dists"]={}
		
def singleCostFunction(p,z,anchor,r):
	
	return math.sqrt((p[0]-anchor[0])**2+(p[1]-anchor[1])**2+(z-anchor[2])**2)-r

def Jtag1(robot,data):#FRONT
	global tags_global
	#p is robot position [x,y,teta]
	p=[0,0,0]
	p[0]=robot[0]-tags_global["FRONT"]["pos"].y*math.sin(robot[2]-math.pi/2)
	p[1]=robot[1]+tags_global["FRONT"]["pos"].y*math.cos(robot[2]-math.pi/2)
	p[2]=tags_global["FRONT"]["pos"].z
	anchors=data["anchors"]
	radius=data["radius"]
	
	return singleCostFunction(p,p[2],anchors[0],radius[0])**2+singleCostFunction(p,p[2],anchors[1],radius[1])**2+singleCostFunction(p,p[2],anchors[2],radius[2])**2

def Jtag2(robot,data):#BACK
	global tags_global
	#p is robot position [x,y,teta]
	p=[0,0,0]
	p[0]=robot[0]-tags_global["BACK"]["pos"].y*math.sin(robot[2]-math.pi/2)
	p[1]=robot[1]+tags_global["BACK"]["pos"].y*math.cos(robot[2]-math.pi/2)
	p[2]=tags_global["BACK"]["pos"].z
	anchors=data["anchors"]
	radius=data["radius"]
	return singleCostFunction(p,p[2],anchors[0],radius[0])**2+singleCostFunction(p,p[2],anchors[1],radius[1])**2+singleCostFunction(p,p[2],anchors[2],radius[2])**2


def total_single_tag(pRobot,tags_data):
	if 'FRONT' in tags_data.keys():
		return Jtag1(pRobot,tags_data["FRONT"])
	if 'BACK' in tags_data.keys():
		return Jtag2(pRobot,tags_data["BACK"])



def get_3Dposition_constrained_single_tag(tags_data,inicial_estimation):
	res=optimize.fmin_slsqp(total_single_tag, inicial_estimation, args=(tags_data,), iprint =0, iter=100, acc=1e-06)
	res[2]=angleNormalize(res[2])
	return res


def angleNormalize(angle):
	mini=angle-int(angle/(2*numpy.pi))*(2*numpy.pi)
	if(mini < 0):
		return mini+(2*numpy.pi)
	else:
		return mini
	
def estimate_robot_pos():	
	
	global inicial_estimation
	global avg_robot_pos
	global anchors_positions
	global ctrl
	global CONTROL_RATE
	rate = rospy.Rate(CONTROL_RATE)

	rospy.sleep(0.3)
	
	while not rospy.is_shutdown():
		
		if(len(ctrl.keys()) == 1):			# AW: NEW CODE TO SUPPORT ONE TAG ONLY...
			
			tags_data=convert_data()

			if 'FRONT' in tags_data.keys():
				if len(tags_data['FRONT']['var'])<1:
					print "DATA NOT RECEIVED"
					pass
			if 'BACK' in tags_data.keys():
				if len(tags_data['BACK']['var'])<1:
					print "DATA NOT RECEIVED"
					pass


			ps=get_3Dposition_constrained_single_tag(tags_data,inicial_estimation)
			# print "ps", ps
			
			if(est_var):
				distribution=unscented_transform_single_tag(tags_data,inicial_estimation)
			else:
				distribution={"var":[0,0,0]}

			# print 'distribution', distribution
			
			robot_pos=[avg_robot_pos[0].update(ps[0]),avg_robot_pos[1].update(ps[1]),avg_robot_pos[2].update(ps[2])]
			
			
			if(rospy.has_param("~tf")):
				br.sendTransform((robot_pos[0],robot_pos[1],0),tf.transformations.quaternion_from_euler(0, 0, robot_pos[2]),rospy.Time.now(),rospy.get_param("~tf"),world_frame)
			
			robot_position=PoseWithCovariance()
			temp=tf.transformations.quaternion_from_euler(0, 0, robot_pos[2])
				
			robot_position.pose.position.x = robot_pos[0]
			robot_position.pose.position.y = robot_pos[1]
			robot_position.pose.orientation.x=temp[0]
			robot_position.pose.orientation.y=temp[1]
			robot_position.pose.orientation.z=temp[2]
			robot_position.pose.orientation.w=temp[3]
			
			robot_position.covariance=[distribution["var"][0],0,0,0,0,0,
										0,distribution["var"][1],0,0,0,0,
										0,0,0,0,0,0,
										0,0,0,0,0,0,
										0,0,0,0,0,0,
										0,0,0,0,0,distribution["var"][2]]
										
			pub.publish(robot_position)
			inicial_estimation=ps

		ctrl={}
		rate.sleep()



def unscented_transform_single_tag(data,inicial_estimation):
	single_tag_id = ''	# AW

	num_dim = 3			# 6 for 2 tags

	if 'FRONT' in data.keys():
		mean=numpy.array(data["FRONT"]["radius"])
		var=numpy.identity(num_dim)*numpy.array(data["FRONT"]["var"])		# AW: was 6 dim, for 2 tags
		single_tag_id = 'FRONT'
	if 'BACK' in data.keys():
		mean=numpy.array(data["BACK"]["radius"])
		var=numpy.identity(num_dim)*numpy.array(data["BACK"]["var"])
		single_tag_id = 'BACK'

	res=[]
	ndata=data


	for i in range(0,num_dim):
		sigma_point = mean + numpy.sqrt(6*var[:,i])
		ndata[single_tag_id]["radius"] = sigma_point[0:3]
		ps=get_3Dposition_constrained_single_tag(ndata,inicial_estimation) # This checks for tag ids, BACK or FRONT, so keep the keys
		ps[2]=angleNormalize(ps[2])
		res.append(ps)
	
	for i in range(0,num_dim):
		sigma_point=mean - numpy.sqrt(6*var[:,i])
		ndata[single_tag_id]["radius"]=sigma_point[0:3]
		ps=get_3Dposition_constrained_single_tag(ndata,inicial_estimation)
		ps[2]=angleNormalize(ps[2])
		res.append(ps)

	# print "res", res 

	res_mean=numpy.sum(res,axis=0)/2.0*num_dim 			# Was 12 = 6*2
	res_var=numpy.sum((numpy.reshape(res,(num_dim*2,3))-numpy.reshape(numpy.array(list(res_mean)*num_dim*2),(num_dim*2,3)))**2,axis=0)/num_dim*2.0
	return {"mean":res_mean,"var":res_var}



def convert_data():
	global tags_global
	global anchors_positions

	# print "tags global", tags_global
	
	res={}
	for tag_id in tags_global.keys():
		res[tag_id]={"anchors":[],"radius":[],"var":[]}
		for anchor_id in tags_global[tag_id]["anchors_dists"].keys():
			res[tag_id]["anchors"].append(anchors_positions[anchor_id])
			res[tag_id]["radius"].append(tags_global[tag_id]["anchors_dists"][anchor_id])
			res[tag_id]["var"].append(tags_global[tag_id]["anchors_variance"][anchor_id])
	return res

	
def tag_data_update(array): 		# callback for listener() and topic /uwb_filtered
	global tags_global
	global ctrl
	
	ctrl[array.tag_id]=1
	tags_global[array.tag_id]["anchors_dists"]={}
	tags_global[array.tag_id]["anchors_variance"]={}
	
	for i in range(len(array.data)):
		tags_global[array.tag_id]["anchors_dists"][array.data[i].anchor_id]=array.data[i].radius
		tags_global[array.tag_id]["anchors_variance"][array.data[i].anchor_id]=array.data[i].variance
	# print "tags_global", tags_global
	# pos is the tag position (wrt. robot centre)
	# tags_global {'FRONT': {'anchors_variance': {}, 'pos': x: 0, y: 23.7, z: 15.2, 'anchors_dists': {}}, 
	# 			 'BACK': {'anchors_variance': {'0000009b': 7.290004730224609, '0000008c': 16.090007781982422, 
	# 			 								'0000007a': 0.5599989295005798}, 'pos': x: 0, y: -21.6, z: 18.0, 
	# 			 		 'anchors_dists': {'0000009b': 115.0999984741211, '0000008c': 64.0999984741211, '0000007a': 56.79999923706055}}}


	
def listener():
	global tags_src
	tags=tags_src.split(",")
	rospy.Subscriber("tag_"+str(tags[0])+"/uwb_filtered", uwb_array, tag_data_update)
	if len(tags)>1:
		rospy.Subscriber("tag_"+str(tags[1])+"/uwb_filtered", uwb_array, tag_data_update) 		# AW: for the second tag
	rospy.spin()

if __name__ == '__main__':
	try:
		# avg_robot_pos=[mean.filter_mean(10),mean.filter_mean(10),mean.filter_mean(1),mean.filter_mean(10)]
		rospy.init_node('uwb_triangCombo')

		avg_robot_pos = [variance.stack_mean(10), variance.stack_mean(10), variance.stack_mean(1), variance.stack_mean(10)]


		br = tf.TransformBroadcaster()
		pub = rospy.Publisher("uwb_tag01/Pose_LSQ", PoseWithCovariance, queue_size=10)
		position_stats[0]=variance.stack_var(10) 			# Creates data strucure for sliding window of size 10
		position_stats[1]=variance.stack_var(10)
		position_stats[2]=variance.stack_var(10)
		
		
		#GLOBAL PARAMTERES-----------------------------------
		
		if(rospy.has_param("world_frame")):
			world_frame=rospy.get_param("world_frame")
			# print "world_frame", world_frame  		# map
			
		if(rospy.has_param("anchors")):
			anchors_positions=rospy.get_param("anchors")
			# print "anchors_positions", anchors_positions # '0000009b': [-439.60532221346705, -510.81432853200386, 166.81927556964314]
		else:
			rospy.signal_shutdown("No Anchors file. Please run uwb_mapping.launch") # now it is called uwb_anchor_finder.py
		
		if(rospy.has_param("tags_config")):
			tags_config=rospy.get_param("tags_config") 		# Tag position w.r.t. the origin
			# print "tags_config", tags_config # [{'y': 23.7, 'x': 0, 'z': 15.2, 'id': 'FRONT'},
		else:
			rospy.signal_shutdown("No tags file. unable to continue")
			
			
		#PRIVATE PARAMTERES-----------------------------------
		if(rospy.has_param("~id")):
			tags_src=rospy.get_param("~id")
			# print "tags_src", tags_src # BACK
		if(rospy.has_param("~estimate_variance")):
			est_var=rospy.get_param("~estimate_variance")
		else:
			est_var=False
		print "Estimate variance for pose:", est_var
			
		if(rospy.has_param("~hz")):
			CONTROL_RATE=rospy.get_param("~hz")
		else:
			CONTROL_RATE=20
		#-----------------------------------------------------
		
		config_tags()
		thread.start_new_thread(estimate_robot_pos, () )
		listener()
				
			
	except rospy.ROSInterruptException:
		pass
