"""Analyze Grasp 3D - own module: a collection of several function for analyzing the grasps
"""
from openravepy import *
from lpsolve55 import *
from lp_maker import *

import numpy, time, scipy, csv, os
import roslib.packages

#not finished yet
def mindist_grasp3d(contacts):
	mindist = 0	
	K = []

	#find the screw coordinates
	#the wrench W=(Force,PosxForce) is a screw (http://en.wikipedia.org/wiki/Screw_theory), 	
	S = [] #filled with several wrench vectors
	for i in range(0,len(contacts)): 		
		wrench_single = []
		wrench_single[0:3] = contacts[i][3:6]
		wrench_single.extend(numpy.cross(contacts[i][0:3], contacts[i][3:6]).tolist())
		S.append(wrench_single)
	
	#need at least 7 contact wrenches to have force closure in 3D
	if (len(S) < 7) and (numpy.rank(S) < 6):
		return

	#average of all columns
	wc = numpy.mean(S ,axis=0)

	#number of contacts
	#print "repmat: ",numpy.tile(wc,(numpy.shape(S)[0],1))
	#print "shape: ", numpy.shape(numpy.tile(wc,(numpy.shape(S)[0],1)))
	T = S - numpy.tile(wc,(numpy.shape(S)[0],1))
	


	#function preperation
	f = numpy.array(wc).tolist()
	A = numpy.array(T).tolist()
	b = numpy.array(numpy.ones(numpy.shape(S)[0])).tolist()

	#sense of the inequalities:
	#e(i) < 0  ==> Less Than
        #e(i) = 0  ==> Equals
        #e(i) > 0  ==> Greater Than
 	ones_matrix = numpy.ones_like(b)
	e = numpy.array(-ones_matrix).tolist()

	#DEBUG: output of the matrices
	#print "Function f: ",f
	#print "Function A: ",A
	#print "Function b: ",b
	#print "T: ",T
	#print "shape-f: ",numpy.shape(f)
	#print "shape-A: ",numpy.shape(A)
	#print "shape-b: ",numpy.shape(b)

	#INFO: Set last argument to 0 will result in same as graspit
	lp = lp_maker(f,A,b,e,None,None,None,1,1)
	solvestat = lpsolve('solve', lp)
	mindist = lpsolve('get_objective', lp)
	x = lpsolve('get_variables', lp)[0]
	#DEBUG: output of lpsolver	
	print "solvestat: ",solvestat
	print "mindist: ", mindist
	print "x: ", x

	mindist = -mindist
		
	if (mindist < 0) or (mindist > 1):
		mindist = -1
	
	mindist_final = mindist
	print "Graspquality mindist: ",mindist_final	

# from grasping module
def grasp_performance(contacts, target):
	draw_contacts = numpy.array(contacts)
	with target:
		target.SetTransform(numpy.eye(4))
		# find closest contact to center of object
		if len(draw_contacts) > 0: # sometimes we get no contacts?!
			ab=target.ComputeAABB()
			return -numpy.min(sum((draw_contacts[:,0:3]-numpy.tile(ab.pos(),(len(draw_contacts),1)))**2,1))
		else:
			return -inf

#plot a costume ray, consists of two points, two see where it intersect with the target
def costume_ray_plot(ray,target,env):
	N = ray.shape[0]
	with target:
		target.SetTransform(numpy.eye(4))
		approachgraphs = [env.plot3(points=ray[:,0:3],pointsize=15,colors=numpy.array((1,0,0))), env.drawlinelist(points=numpy.reshape(numpy.c_[ray[:,0:3],ray[:,0:3]+0.5*ray[:,3:6]],(2*N,3)),linewidth=4,colors=numpy.array((1,0,0,1)))]
		env.UpdatePublishedBodies()
		raw_input('..')

#plot the arrow of the graspdir or other stuff
def draw_arrow(p_start,p_end,target,env):
	with target:
		target.SetTransform(numpy.eye(4))
		h = env.drawarrow(p1=p_start,p2=-p_end,linewidth=0.005,color=[1.0,0.0,0.0])
		env.UpdatePublishedBodies()
		h2 = env.plot3(numpy.array(p_start),40,[1,0,0])
		raw_input('..')

#plot contact points of target with manip and the normal forces
def draw_cp_norm(contacts,manip,env,gmodel):
	with gmodel.GripperVisibility(manip):
		h = []
		for i in range(0,len(contacts)):
			h.append(env.drawlinestrip(points=numpy.array((numpy.array(contacts[i].pos).tolist(),numpy.array(-10*(contacts[i].pos-contacts[i].depth*contacts[i].norm)).tolist())),
                                           linewidth=1.5,
                                           colors=numpy.array((1,0,0))))
		raw_input("End show of collision point with normals")

#Output in simox xml structure
def simox_output(numb,e_quality,v_quality,transform,joint_values):
	pathname = '/home/mdl-ws/OpenRAVE/grasptests/graspout.txt'
	grasp_name = "test_output no. "+str(numb)
	pose = poseFromMatrix(transform) #returns [w, q0, q1, q2, x, y, z]
	f_out = open(pathname, 'w')
	scale_factor = 1000
	print "Joint_values:",joint_values

	#Fill the file output
	f_out.write('<Grasp name="'+grasp_name+'" quality="'+str(e_quality)+'" quality_vol="'+str(v_quality)+'" Creation="OpenRAVE" Preshape="Grasp Preshape">\n') #Information about the grasp
	f_out.write('\t<Transform>\n') #Begin of transformation output
	f_out.write('\t\t<quaternion x="'+str(pose[1])+'" y="'+str(pose[2])+'" z="'+str(pose[3])+'" w="'+str(pose[0])+'"/>\n')
	f_out.write('\t\t<Translation x="'+str(pose[4]*scale_factor)+'" y="'+str(pose[5]*scale_factor)+'" z="'+str(pose[6]*scale_factor)+'" />\n')
	f_out.write('\t</Transform>\n')
	f_out.write('\t<Configuration name="Pre '+grasp_name+'">\n')
	#SDH Joint Values - Beginnend mit Daumen(1) und die Zwei Finger GUZS nummeriert(2)(3)
	#[Fingerwinkel(2)(3), Fingerwinkel(2), Fingerknick(2), Fingerwinkel(3), Fingerknick(3), Fingerwinkel(1), Fingerknick(1)]
	f_out.write('\t\t<Node name="Thumb Joint1" unit="radian" value="'+str(joint_values[5])+'"/>\n')
	f_out.write('\t\t<Node name="Thumb Joint2" unit="radian" value="'+str(joint_values[6])+'"/>\n')
	f_out.write('\t\t<Node name="Finger1 Knuckle Joint" unit="radian" value="'+str(joint_values[0])+'"/>\n')
	f_out.write('\t\t<Node name="Finger1 Joint1" unit="radian" value="'+str(joint_values[3])+'"/>\n')
	f_out.write('\t\t<Node name="Finger1 Joint2" unit="radian" value="'+str(joint_values[4])+'"/>\n')
	f_out.write('\t\t<Node name="Finger2 Knuckle Joint" unit="radian" value="-'+str(joint_values[0])+'"/>\n')
	f_out.write('\t\t<Node name="Finger2 Joint1" unit="radian" value="'+str(joint_values[1])+'"/>\n')
	f_out.write('\t\t<Node name="Finger2 Joint2" unit="radian" value="'+str(joint_values[2])+'"/>\n')
	f_out.write('\t</Configuration>\n')
	f_out.write('</Grasp>\n')
	f_out.close()

#converts all graspit grasps from txt to csv format for scripting
def graspit_to_csv(filename):
	tag_for_id = "Graspit"
	pathname_in = '/home/mdl-ws/GraspIt!/Grasps/GraspTests/'+filename+'.txt'
	f = open(pathname_in, 'r')

	#write the meta-information into a seperate file first
	#Name, Samples, Energy formulation, Planner Type, Planner Time
	meta_line = f.readline()
	object_name = str.split(meta_line)[0]

	#create directories
	directory = roslib.packages.get_pkg_dir('cob_offline_grasping')+'/grasping/grasp_tests/'+tag_for_id+'/'+object_name
	if not os.path.exists(directory):
    		os.makedirs(directory)
	pathname_out = directory+'/'+object_name+'.csv'
	pathname_meta = directory+'/'+object_name+'_meta.txt'
	f_out = open(pathname_out, 'w+')
	f_meta = open(pathname_meta, 'w+')
	
	wr = csv.writer(f_out, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL) #quoting=csv.QUOTE_MINIMAL)
	f_meta.write(tag_for_id+' '+meta_line)

	f_meta.close()

	#write names of the rows here
	#SDH Joint Values - Beginnend mit Daumen(1) und die Zwei Finger GUZS nummeriert(2)(3)
	#[Fingerrotation(2)(3), Fingerwinkel(3), Fingerknick(3), Fingerwinkel(3), Fingerknick(2), Daumenrotation(1)[IMMER NULL], Fingerwinkel(1)  Fingerknick(1)]
	wr.writerow(['id', 'object', 'sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'direction', 'qw', 'qx', 'qy', 'qz', 'pos-x', 'pos-y', 'pos-z', 'eps_l1', 'vol_l1', 'iterationstep', 'energy'])

	#begins here to read file
	counter = 0 #count the rows to identify if reading the joint_values or the position of the hand
	number_of_grasp = 0 #counts the number of the grasp for id tag
	grasp = [] #row to be filled and published

	for line in f:
		line_out = str.split(line) #split one string into several parts after each space

		if (counter == 0): #first row with the joint values and a blank in front
			grasp.append(number_of_grasp) #ID
			grasp.append(object_name) 
			joint_values = map(float, line_out)
			joint_values_mod = [joint_values[1], joint_values[2], joint_values[3], joint_values[4], joint_values[5], joint_values[7], joint_values[8]]
			grasp.extend(joint_values_mod)

		if (counter == 1): #second row with the position values, quats and a blank in front
			pos_and_orientation = map(float, line_out)
			pos = pos_and_orientation[1:4] #because index=0 is a blank
			quat = pos_and_orientation[4:]
			pose = quat
			pose.extend(pos)

			#get direction for database
			matrix = matrixFromPose(pose)
			#projection onto the yz-layer with x = 0
			if matrix[1][2] < -0.5: #if hand approaches > 63 degrees then it is TOP grasp
				grasp.append('TOP')
			else:
				grasp.append('SIDE')
			
			#write
			grasp.extend(pose)
			


			#manually typed grasp quality
			if number_of_grasp == 0:
				skip_q = raw_input('Skip grasp quality (y/n)?: ')

			if skip_q == 'n':
				wrong = True
				while wrong:
					print 'Grasp No. '+str(number_of_grasp)+'. Now type the quality'
					e_l1 = raw_input('E_l1: ') 
					v_l1 = raw_input('V_l1: ') 
					itstep = raw_input('Itstep: ') 
					energy = raw_input('Energy: ')
					print 'E_l1: ',e_l1+' | V_l1: ',v_l1+' | Itstep: ',itstep+' | Energy: ',energy
				
					#Check if all values were typed correct
					check = raw_input('If all values were typed correct press [ENTER]')
					if not check:
						wrong = False
			else:
				e_l1 = 0.0
				v_l1 = 0.0
				itstep = 0.0
				energy = 0.0

			grasp.append(e_l1)
			grasp.append(v_l1)
			grasp.append(itstep)
			grasp.append(energy)


		counter = counter + 1
		if (counter > 1):
			wr.writerow(grasp) #write grasp to file
			grasp = [] #reset grasp array to refill with new grasp
			number_of_grasp = number_of_grasp + 1 #counts the grasps
			counter = 0 #counts the line of each grasp

	
	f.close()
	f_out.close()
	print "Finished."

#converts all openrave grasps to csv format for scripting
#input-first-line (meta-info): name, time (@todo: add planner infos)
#input-struct: number, jointconf, trafo, epsilon, volume, forceclosure, validindicees, direction
def or_to_csv(validgrasps, time):
	
	tag_for_id = "Openrave"
	meta_info = validgrasps[0]
	name = meta_info[0]

	#create directories
	directory = roslib.packages.get_pkg_dir('cob_offline_grasping')+'/grasping/grasp_tests/'+tag_for_id+'/'+name
	if not os.path.exists(directory):
    		os.makedirs(directory)
	pathname_out = directory+'/'+name+'.csv'
	pathname_meta = directory+'/'+name+'_meta.txt'

	f_out = open(pathname_out, 'w+')
	f_meta = open(pathname_meta, 'w+')
	wr = csv.writer(f_out, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL) #quoting=csv.QUOTE_MINIMAL)
	
	#write the meta-information into a seperate file first
	#Name, Time, (@todo: add planner infos)
	f_meta.write(tag_for_id+', '+str(name)+', '+', '+str(time))
	f_meta.close()

	wr.writerow(['id', 'object', 'sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint', 'direction', 'qw', 'qx', 'qy', 'qz','pos-x', 'pos-y', 'pos-z', 'eps_l1', 'vol_l1', 'indicee'])

	for i in range (1,len(validgrasps)): #because line 0 is meta-info
		row = [] #create/empty row for new grasp
		actual_grasp = validgrasps[i]
		row.append(str(actual_grasp[0])) #ID
		row.append(name) #Object-name
		
		##SOLL
		#SDH Joint Values - Beginnend mit Daumen(1) und die Zwei Finger GUZS nummeriert(2)(3)
		#[Fingerwinkel(2)(3), Fingerwinkel(2), Fingerknick(2), Fingerwinkel(3), Fingerknick(3), Fingerwinkel(1), Fingerknick(1)]
		joint_values = actual_grasp[1]
		joint_values_sorted = [joint_values[0], joint_values[3], joint_values[4], joint_values[1], joint_values[2], joint_values[5], joint_values[6]]
		for i in range(0,len(joint_values)):
			row.append(joint_values_sorted[i])
		
		#grasp-direction
		globApproach = actual_grasp[7] #globalApproachDir
		if globApproach[2] < -0.7: #if hand approaches > 63 degrees
			row.append('TOP')
		else:
			row.append('SIDE')

		#convert 4x4 matrix to pose
		trafo4x4 = actual_grasp[2]
		m_in_mm = 1000
		pose = poseFromMatrix(trafo4x4) #returns [w, q0, q1, q2, x, y, z]
		pose[4] = m_in_mm * pose[4] #x-koord
		pose[5] = m_in_mm * pose[5] #y-koord
		pose[6] = m_in_mm * pose[6] #z-koord
		for i in range(0,len(pose)):
			row.append(pose[i])

		#quality
		row.append("%.9f" % actual_grasp[3]) #epsilon with precision
		row.append("%.9f" % actual_grasp[4]) #volume with precision

		#misc
		row.append(actual_grasp[6]) #validindicees

		#write this grasp as row to file
		if (not actual_grasp[3] == 0.0) and (not actual_grasp[4] == 0.0):
			wr.writerow(row)
	
	f_out.close()
	print "Finished."

##ROS-Script##
#def get_pregrasp_shape
#def
