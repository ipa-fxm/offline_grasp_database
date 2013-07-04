""" Calibration of the new grasp coordinate system
"""
import roslib
roslib.load_manifest('cob_script_server')
import rospy, csv, os
import tf
import copy

from math import sqrt
from tf.transformations import *
from kinematics_msgs.srv import *
from simple_script_server import *
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import *
sss = simple_script_server()

class CalibScript(script):

	def Initialize(self):

		#init components here
		#self.sss.init("torso")
		self.sss.init("sdh")
		self.sss.init("arm")

		#IK Service
		rospy.wait_for_service('/cob_ik_wrapper/arm/get_ik')
		rospy.wait_for_service('/cob_ik_wrapper/arm/get_fk')
		rospy.wait_for_service('/cob_ik_wrapper/arm/get_constraint_aware_ik')

		self.ikcs = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_constraint_aware_ik', GetConstraintAwarePositionIK)
		self.iks = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_ik', GetPositionIK)
		self.fks = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_fk', GetPositionFK)

	def callFK(self, joint_values, links):
		req = GetPositionFKRequest()
		req.header.stamp = rospy.Time.now()
		req.header.frame_id='arm_0_link'
		req.fk_link_names = links
		req.robot_state.joint_state.name = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint'] + ['arm_%d_joint'%(d+1) for d in range (7)]
		req.robot_state.joint_state.position = [0]*7 + joint_values
		fks = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_fk', GetPositionFK)
		res = self.fks(req)
		#print req
		return res.pose_stamped if res.error_code.val == res.error_code.SUCCESS else None

	def callIK(self, pose_stamped, link):
		req = GetPositionIKRequest()
		req.timeout = rospy.Duration(5)
		req.ik_request.ik_link_name = link
		req.ik_request.ik_seed_state.joint_state.name = ['arm_%d_joint'%(d+1) for d in range (7)]
		req.ik_request.ik_seed_state.joint_state.position = [-1.1572567240035734, -1.9104664691761568, -2.5334780195730255, -1.7853311980377056, -0.072798739390243047, 0.91767934923272776, -1.8876618005378798]
		req.ik_request.pose_stamped = pose_stamped
		iks = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_ik', GetPositionIK)
		res = self.iks(req)
		return res.solution.joint_state.position if res.error_code.val == res.error_code.SUCCESS else None, res.error_code

	#Call IK-Solver constraint aware. Seed state is always the actual position of the robot.
	def callIKconstraint(self, pose_stamped, link):
		req = GetConstraintAwarePositionIKRequest()

		msg = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState)

		req.timeout = rospy.Duration(5)
		req.ik_request.ik_link_name = link
		req.ik_request.ik_seed_state.joint_state.name = msg.joint_names#seed
		req.ik_request.ik_seed_state.joint_state.position = msg.actual.positions
		req.ik_request.pose_stamped = pose_stamped

		#req.constraints
		res = self.ikcs(req)
		return res.solution.joint_state.position if res.error_code.val == res.error_code.SUCCESS else None, res.error_code

	#Move to a pose in cart coord.
	def checkPoseReachability(self, pose, link, listen=None):

		pose.header.stamp = rospy.Time.now()

		if not listen:
			sol, error_code = self.callIKconstraint(pose,link)
		else:
			rospy.sleep(2)
			pose0 = listen.transformPose('/arm_0_link',pose)
			sol, error_code = self.callIKconstraint(pose0,link)

		print error_code
		
		if error_code.val == 1:
			reachable = True
		else:
			reachable = False

		return reachable

	#function will correct the calibration link of the hand on the reference point
	def moveAlongAxis(self, axis, value, calib_pose):
		
		#show values that were typed
		rospy.loginfo('Trying to correct '+value+'m on '+axis+'.')
		
		#check value
		if float(value) >= 0.25:
			rospy.logerr('The value for correction you choose was too high.')
			return
		
		#check for axis
		if str(axis) == 'x':
			calib_pose.pose.position.x = calib_pose.pose.position.x + float(value)
		elif str(axis) == 'y':
			calib_pose.pose.position.y = calib_pose.pose.position.y + float(value)
		elif str(axis) == 'z':
			calib_pose.pose.position.z = calib_pose.pose.position.z + float(value)
		else:
			rospy.logerr('The axis that was typed is incorrect.')
			return
	
		return calib_pose

	def calibrate(self, ref_pose, listen):

		#approximately move to the pose
		self.moveToPose(ref_pose, 'arm_calib_link', listen)

		#precise calibration of pose
		pose_reached = False

		while not pose_reached:
			finished_correction = raw_input('Press [ENTER] to type a new correction, type OK if finished: ')
			if not finished_correction:
				axis = raw_input('Choose the axis the robot should move on (x, y, z): ')
				correction = raw_input('Type the value in METER (e.g. 0.05): ')
				correction_pose = self.moveAlongAxis(axis,correction,ref_pose)
				self.moveToPose(correction_pose, 'arm_calib_link', listen)
			elif str(finished_correction) == 'OK':
				pose_reached = True

		#msg for actual joint position
		msg = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState)
		
		#save actual joint values for calculation
		calib_joint = list(msg.actual.positions)

		#cartesian space		
		calib_pose = self.callFK(calib_joint, ['arm_calib_link'])[0]

		return [calib_pose.pose.position.x, calib_pose.pose.position.y, calib_pose.pose.position.z] 

	def calculateTransformation(self, ref_pos, calib_pos):

		#make homogenous vector
 		for i in range(0, len(ref_pos)):
			ref_pos[i].append(1.0)
			calib_pos[i].append(1.0)

		#add to an array A
		A = numpy.array(calib_pos)
		
		#create matrix
		matrix = []

		for i in range (0,4):
			#right side b
			b = numpy.array([])
			b = numpy.array([ref_pos[0][i], ref_pos[1][i], ref_pos[2][i], ref_pos[3][i], ref_pos[4][i], 
							 ref_pos[5][i], ref_pos[6][i], ref_pos[7][i], ref_pos[8][i]				 ])

			#get A*x = b 
			x_lstsq = numpy.linalg.lstsq(A, b, rcond=-1)

			#fill the matrix with the seperate solved vectors
			matrix.append(x_lstsq[0])

		#homogenous matrix for tf package, inverse is needed for the correct solution
		matrix_4x4= (numpy.matrix(matrix)).I #inverse of the matrix for right trafo
		print matrix_4x4

		#create translation vector for the broadcasting of the static frame
		translation_vector = [matrix_4x4[0,3], matrix_4x4[1,3], matrix_4x4[2,3]]
		
		return translation_vector, quaternion_from_matrix(matrix)

	def Run(self):

		listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2) #sleep necessary here, because tf needs it	
		
		#here the reference points will be filled in
		reference_poses = []
		number_ref_pos = 5
		
		handle_sdh = sss.move('sdh', [[0.0, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57]])			
		handle_sdh.wait()
		
		#starting position: point 1
		#in approximate table frame
		ref_pose = PoseStamped()
		ref_pose.header.frame_id = 'predefined_table_link'
		ref_pose.header.stamp = rospy.Time.now()
		ref_pose.pose.position.x = 0.0
		ref_pose.pose.position.y = 0.15
		ref_pose.pose.position.z = 0.0
		ref_pose.pose.orientation.x, ref_pose.pose.orientation.y, ref_pose.pose.orientation.z, ref_pose.pose.orientation.w = [-0.7071, 0.0, 0.0,0.7071]#[0.5, -0.5, 0.5, 0.5]

		#fill whole array with poses to get same orientation
		for i in range (0,number_ref_pos):
			reference_poses.append(copy.deepcopy(ref_pose))

		height = 0.0
		max_height = 1.0
		start_height = -0.02
		height_diff = 0.01
		index = (max_height-start_height) / height_diff

		valid_heights = []
		for x in range(0,int(index)): #height from

			one_height = [] 

			reference_poses[1].pose.position.x, reference_poses[1].pose.position.y, reference_poses[1].pose.position.z = [ 0.13, height,  0.13]
			reference_poses[2].pose.position.x, reference_poses[2].pose.position.y, reference_poses[2].pose.position.z = [-0.13, height,  0.13]
			reference_poses[3].pose.position.x, reference_poses[3].pose.position.y, reference_poses[3].pose.position.z = [-0.13, height, -0.13]
			reference_poses[4].pose.position.x, reference_poses[4].pose.position.y, reference_poses[4].pose.position.z = [ 0.13, height, -0.13]

			height += height_diff
			actual_height = start_height*100 + (height_diff*100*x)

			rospy.loginfo('Testing workspace for actual height of '+str(actual_height)+'cm')
			one_height.append(actual_height)			

			for i in range (1, len(reference_poses)):
				reachable = self.checkPoseReachability(reference_poses[i], 'arm_calib_link', listener)
				print "Position "+str(i)+' | Reachable: '+str(reachable)
				if reachable == False:
					pose_mod = None
					pose_mod = copy.deepcopy(reference_poses[i])
					while not (self.checkPoseReachability(pose_mod, 'arm_calib_link', listener)):
						
						if pose_mod.pose.position.x >= 0.0:
							pose_mod.pose.position.x -= 0.02
						elif pose_mod.pose.position.x < 0.0:
							pose_mod.pose.position.x += 0.02
						if pose_mod.pose.position.z >= 0.0:
							pose_mod.pose.position.z -= 0.02
						elif pose_mod.pose.position.z < 0.0:
							pose_mod.pose.position.z += 0.02
						print "Reducing radius till solution found. \nRadius: "+str(pose_mod.pose.position.x)
					if self.checkPoseReachability(pose_mod, 'arm_calib_link', listener):
						print "Radius that works: "+str(pose_mod.pose.position.x)
						one_height.append('Radius: '+str(pose_mod.pose.position.x))
					else:
						print 'Bad heights for workspace'
						one_height.append(reachable)
				else:
					one_height.append(reachable)

			print one_height
			valid_heights.append(one_height)
		print valid_heights

		exit()

if __name__ == "__main__":
	SCRIPT = CalibScript()
	SCRIPT.Start()



