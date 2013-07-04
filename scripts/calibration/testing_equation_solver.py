import roslib
roslib.load_manifest('cob_script_server')
import rospy, csv, os
import grasp_handle #own function-lib
import tf

from math import sqrt
from tf.transformations import *
from kinematics_msgs.srv import *
from simple_script_server import *
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import *

if __name__ == '__main__':
	

	rospy.init_node('tf_fix_orientation')

	noise = 0.01 #1cm noise
	full_calib = True
	if full_calib:
		#pos1
		pos1_mess = [1.0, 1.0, 1.142] #test mit unterschiedlicher hoehe
		pos1_ref = [0.0, 0.142, 0.0] 	#grasptable_link
		#pos1_mess = [1.0, 1.0, 1.0] #base_link
		#pos1_ref = [0.0, 0.0, 0.0] 	#grasptable_link

		#pos2
		pos2_mess = [1.0, 0.8695+noise, 0.995]
		pos2_ref = [0.0, -0.005, 0.1305]

		#pos3
		pos3_mess = [0.8695-noise, 0.8695, 0.995] #test mit unterschiedlicher hoehe
		pos3_ref = [-0.1305, -0.005, 0.1305]

		#pos4
		pos4_mess = [0.8695, 1.0, 0.995-noise]
		pos4_ref = [-0.13, -0.005, 0.0]

		#pos5
		pos5_mess = [0.8695-noise, 1.1305, 0.995] #test mit unterschiedlicher hoehe
		pos5_ref = [-0.1305, -0.005, -0.1305]

		#pos6
		pos6_mess = [1.0, 1.1305, 0.995]
		pos6_ref = [0.0, -0.005, -0.1305]

		#pos7
		pos7_mess = [1.1305, 1.1305+noise, 0.995]
		pos7_ref = [0.1305, -0.005, -0.1305]

		#pos8
		pos8_mess = [1.1305, 1.0+noise, 0.995]
		pos8_ref = [0.1305, -0.005, 0.0]

		#pos9
		pos9_mess = [1.1305, 0.8695+noise, 0.995]
		pos9_ref = [0.1305, -0.005, 0.1305]

	#make homogenous vector
	pos1_mess.append(1.0)	
	pos1_ref.append(1.0)

	pos2_mess.append(1.0)	
	pos2_ref.append(1.0)

	pos3_mess.append(1.0)	
	pos3_ref.append(1.0)

	pos4_mess.append(1.0)	
	pos4_ref.append(1.0)

	pos5_mess.append(1.0)	
	pos5_ref.append(1.0)

	pos6_mess.append(1.0)	
	pos6_ref.append(1.0)

	pos7_mess.append(1.0)	
	pos7_ref.append(1.0)

	pos8_mess.append(1.0)	
	pos8_ref.append(1.0)

	pos9_mess.append(1.0)	
	pos9_ref.append(1.0)

	#add to an array A
	a = []
	a.append(pos1_mess)
	a.append(pos2_mess)
	a.append(pos3_mess)
	a.append(pos4_mess)
	a.append(pos5_mess)
	a.append(pos6_mess)
	a.append(pos7_mess)
	a.append(pos8_mess)
	a.append(pos9_mess)

	A = numpy.array(a)
	
	matrix = []
	for i in range (0,4):
		#right side b
		b = numpy.array([])
		b = numpy.array([pos1_ref[i], pos2_ref[i], pos3_ref[i], pos4_ref[i], pos5_ref[i], pos6_ref[i], pos7_ref[i], pos8_ref[i], pos9_ref[i]])

		#get A*x = b 
		x_lstsq = numpy.linalg.lstsq(A, b, rcond=-1)
		#x = numpy.linalg.solve(A,b)

		#fill the matrix with the seperate solved vectors
		print x_lstsq
		matrix.append(x_lstsq[0])

	#homogenous matrix for tf package, inverse is needed for the correct solution
	matrix_4x4= (numpy.matrix(matrix)).I #inverse of the matrix for right trafo
	print matrix_4x4

	#create translation vector for the broadcasting of the static frame
	vector_trans = [matrix_4x4[0,3], matrix_4x4[1,3], matrix_4x4[2,3]]
	

	actual_quat = quaternion_from_matrix(matrix_4x4)
	print "Actual quaternions: ",actual_quat
	br = tf.TransformBroadcaster()
	rate = rospy.Rate(20.0)
	print list(vector_trans)
	while not rospy.is_shutdown():
		br.sendTransform(vector_trans, actual_quat,
			rospy.Time.now(),"grasp_link","base_link")
	#rate.sleep()
	exit()
