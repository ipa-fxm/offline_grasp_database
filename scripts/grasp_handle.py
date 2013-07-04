"""For ROS script - own module
"""
import numpy
import tf
import csv 
import os
import rospy
import roslib
import operator, random #csv importing and sorting

from geometry_msgs.msg import * 
from tf.transformations import *

###### 		Some simple math functions		######
def add(u, v):
    return [ u[i]+v[i] for i in range(len(u)) ]

def sub(u, v):
    return [ u[i]-v[i] for i in range(len(u)) ]

def normalize(v):
    return v/numpy.linalg.norm(v)

#Get the specific grasps that are needed for the test-script
def getGraspCollection(planner_name, object_name, metric, ids):
	#GraspSelection
	#Begins here to read the grasp .csv-Files
	path_in = roslib.packages.get_pkg_dir('cob_offline_grasping')+'/grasping/grasp_tests/'+planner_name+'/'+object_name+'/'+object_name+'.csv'

	#Check if path exists
	try:
		with open(path_in) as f: pass
	except IOError as e:
		rospy.logerr("The path or file does not exist: "+path_in)

	#If exists open with dictreader
	reader = csv.DictReader( open(path_in, "rb"), delimiter=',')

	#Sortiere die Liste nach eps_l1 und vol_l1 absteigend
	sorted_list = sorted(reader, key=operator.itemgetter(metric), reverse=True)

	#Suche nach dem besten TOP und SIDE Griff.
	#Sowohl fuer die Epsilon Liste als auch fuer die Volume Liste. Je nach Metric parameter

	grasps = []
	new_id = []
	searching_top_grasp = True
	searching_side_grasp = True		
	for i in range(0,len(sorted_list)):
		if sorted_list[i]['id'] not in ids:
			if searching_top_grasp and (sorted_list[i]['direction'] == 'TOP'):
				grasps.append(sorted_list[i])
				new_id.append(sorted_list[i]['id'])
				searching_top_grasp = False
			if searching_side_grasp and (sorted_list[i]['direction'] == 'SIDE'):
				grasps.append(sorted_list[i])
				new_id.append(sorted_list[i]['id'])
				searching_side_grasp = False

	ids.extend(new_id)

	#append rest of the sorted_list
	rest_grasps_top = []
	rest_grasps_side = []
	for i in range(0,len(sorted_list)):
		if sorted_list[i]['id'] not in new_id:
			if (sorted_list[i]['direction'] == 'TOP'):
				rest_grasps_top.append(sorted_list[i])
			if (sorted_list[i]['direction'] == 'SIDE'):
				rest_grasps_side.append(sorted_list[i])

	#DEBUG: Ausgabe der besten Werte
	rospy.logdebug('The best TOP grasp for metric '+metric+'is '+sorted_list[0][metric]+' | ID: '+sorted_list[0]['id'])
	rospy.logdebug('The best SIDE grasp for metric '+metric+'is '+sorted_list[1][metric]+' | ID: '+sorted_list[1]['id'])

	return grasps, rest_grasps_top, rest_grasps_side, ids

def getGraspWithID(idnum, planner_name, object_name):
	#GraspSelection
	#Begins here to read the grasp .csv-Files
	path_in = roslib.packages.get_pkg_dir('cob_offline_grasping')+'/grasping/grasp_tests/'+planner_name+'/'+object_name+'/'+object_name+'.csv'

	#Check if path exists
	try:
		with open(path_in) as f: pass
	except IOError as e:
		rospy.logerr("The path or file does not exist: "+path_in)

	#If exists open with dictreader
	reader = csv.DictReader( open(path_in, "rb"), delimiter=',')

	#Sortiere die Liste nach eps_l1 und vol_l1 absteigend
	sorted_list = sorted(reader, key=operator.itemgetter('eps_l1'), reverse=True)

	#suche nach grasp mit ID
	
	for i in range(0,len(sorted_list)):
		if sorted_list[i]['id'] == str(idnum):
			return sorted_list[i]
		

#Get 2 random grasps
def getRandomGrasps(num, planner_name, object_name, ids):
	#GraspSelection
	#Begins here to read the grasp .csv-Files
	path_in = roslib.packages.get_pkg_dir('cob_offline_grasping')+'/grasping/grasp_tests/'+planner_name+'/'+object_name+'/'+object_name+'.csv'
	print path_in

	#Check if path exists
	try:
		with open(path_in) as f: pass
	except IOError as e:
		rospy.logerr("The path or file does not exist: "+path_in)

	#If exists open with dictreader
	reader = csv.DictReader(open(path_in, "rb"), delimiter=',')

	#Get the grasp list
	random_grasps = []
	new_ID = []
	grasp_list = sorted(reader, key=operator.itemgetter('id'), reverse=False)

	#Pick randomly from list until have num grasps
	for i in range(0,num):
	
		looking_for_grasps = True
			
		while looking_for_grasps: 
			new_ID = []
			rand_pick = random.choice(grasp_list)

			if (rand_pick['id'] not in ids) and not (rand_pick['eps_l1'] == 0.0):
				random_grasps.append(rand_pick)
				new_ID.append(rand_pick['id'])
				looking_for_grasps = False

			if new_ID:
				ids.extend(new_ID)

	return random_grasps, ids

#Calculation of pregrasp position in the cartesian space
def getPreshapeCart(grasp, distance): #distance in mm

	#The final grasp position
	x = grasp.pose.position.x
	y = grasp.pose.position.y
	z = grasp.pose.position.z
	
	#calculation
	t = (distance/numpy.sqrt(x**2+y**2+z**2))+1
	print "t = ",t,x,y,z
	
	#Fill the preshape into a PoseStamped msg
	pre = PoseStamped()
	pre.header.stamp = rospy.Time.now()
	pre.header.frame_id = "table_link"
	pre.pose.position.x = (x * t)
	pre.pose.position.y = (y * t)
	pre.pose.position.z = (z * t)
	pre.pose.orientation = grasp.pose.orientation

	print "pregrasp shape is: \n", pre
	return pre

#transform grasp around y-axis
def transformGrasp(grasp, angle, offset=0.0):

	#angle: from degree to rad for compose the transform_matrix
	angle_rad = -(angle+offset) * numpy.pi/180
	transform_matrix = compose_matrix(angles=(0,angle_rad,0))

	#grasp in format
	grasp_pos = [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z]
	grasp_quat = [grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w]

	#grasp transformation matrix
	grasp_matrix = quaternion_matrix(grasp_quat)
	grasp_matrix[0:3,3] = grasp_pos	

	#calculation of new grasp position
	rotated_grasp = numpy.dot(transform_matrix, grasp_matrix)
	rotated_grasp_quat = quaternion_from_matrix(rotated_grasp)
	
	#into struct
	new_grasp = PoseStamped()
	new_grasp.header.stamp = rospy.Time.now()
	new_grasp.header.frame_id = "table_link"
	new_grasp.pose.position.x = rotated_grasp[0,3]
	new_grasp.pose.position.y = rotated_grasp[1,3]
	new_grasp.pose.position.z = rotated_grasp[2,3]
	new_grasp.pose.orientation.x = rotated_grasp_quat[0]
	new_grasp.pose.orientation.y = rotated_grasp_quat[1]
	new_grasp.pose.orientation.z = rotated_grasp_quat[2]
	new_grasp.pose.orientation.w = rotated_grasp_quat[3]

	return new_grasp
	
