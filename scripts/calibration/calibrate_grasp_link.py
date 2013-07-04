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
from threading import *

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
		req.header.frame_id='base_link'
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
	def moveToPose(self, pose, link, listen=None, planned=None):

		solution_found = False
		counter = 0
		while not solution_found:
			pose.header.stamp = rospy.Time.now()
			if not listen:
				sol, error_code = self.callIKconstraint(pose,link)
			else:
				rospy.sleep(2)
				pose0 = listen.transformPose('/base_link',pose)
				sol, error_code = self.callIKconstraint(pose0,link)
				print error_code
			
			if error_code.val == 1:
				solution_found = True
				rospy.loginfo('IK Solution found!')
		
			counter += 1
			if counter > 10:
				handle_arm = sss.move_planned('arm','pregrasp')
				handle_arm.wait()
				counter = 0

		if not planned:

			handle_arm = sss.move('arm',[list(sol)])
		else:
			handle_arm = sss.move_planned('arm',[list(sol)])

		handle_arm.wait()

	#check if IK solution can be found
	def checkIK(self, pose, link, listen=None):

		pose.header.stamp = rospy.Time.now()

		rospy.sleep(2)
		pose0 = listen.transformPose('/base_link',pose)

		sol, error_code = self.callIKconstraint(pose0,link)

		if error_code.val != 1:
			print error_code
		return True if error_code.val == 1 else False

	#function will correct the calibration link of the hand on the reference point
	def moveAlongAxis(self, axis, value, calib_pose):
		
		#show values that were typed
		rospy.loginfo('Trying to correct '+str(value)+'m on '+axis+'.')		

		#check value
		if float(value) >= 0.25:
			rospy.logerr('The value for correction you choose was too high.')
			return
		
		#check for axis
		if str(axis) == 'x':
			calib_pose.pose.position.x += float(value)
		elif str(axis) == 'y':
			calib_pose.pose.position.y += float(value)
		elif str(axis) == 'z':
			calib_pose.pose.position.z += float(value)
		else:
			rospy.logerr('The axis that was typed is incorrect.')
			return
	
		return calib_pose

	#function will return the correction value and the axis for the calibration
	def getAxisCorrection(self):

		axis = None
		correction_cm = None
		while (not axis == 'x') and (not axis == 'y') and (not axis == 'z'):		
			axis = raw_input('Choose the axis the robot should move on (x, y, z): ')
		
		while not isinstance(correction_cm, (int,float,long)):
			try:
				correction_cm = input('Type the value in cm (e.g. 5): ')
			except:
				print('Type is no interger/float. Type again.')
		
		correction_m = float(correction_cm) / 100

		return axis, correction_m

	def estimateCorrection(self, calib_pose, listen):

		approx_correction = False

		rospy.sleep(2)
		est_calib_pose = listen.transformPose('/predefined_table_link',calib_pose)

		while not approx_correction:
			finished_correction = raw_input('[ENTER] to skip estimated correction (type OK to correct): ')
			if not finished_correction:
				approx_correction = True
			elif str(finished_correction) == 'OK':
				axis, correction = self.getAxisCorrection()
				est_calib_pose = self.moveAlongAxis(axis,correction,est_calib_pose)

		return listen.transformPose('/base_link',est_calib_pose)
		
	def calibrate(self, ref_pose, listen):

		#approximately move to the pose
		rospy.loginfo('Moving approximately planned to the reference point.')
		self.moveToPose(ref_pose, 'arm_calib_link', listen, planned=True)
		self.moveToPose(ref_pose, 'arm_calib_link', listen)

		#precise calibration of pose
		pose_reached = False
		
		#the position that should be corrected
		ref_pose_to_adjust = copy.deepcopy(ref_pose)
		while not pose_reached:
			finished_correction = raw_input('[ENTER] to type a new correction (type OK if finished): ')
			if not finished_correction:
				axis, correction = self.getAxisCorrection()
				corrected_pose = self.moveAlongAxis(axis,correction,ref_pose_to_adjust)
				self.moveToPose(corrected_pose, 'arm_calib_link', listen)
			elif str(finished_correction) == 'OK':
				pose_reached = True			

		#msg for actual joint position
		msg = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState)
		
		#save actual joint values for calculation
		calib_joint = list(msg.actual.positions)

		#cartesian space		
		calib_pose = self.callFK(calib_joint, ['arm_calib_link'])[0]
		print calib_pose

		calib_pose = self.estimateCorrection(calib_pose, listen)
		
		print calib_pose

		#backup position
		backup_position = copy.deepcopy(ref_pose)
		backup_position.pose.position.y += 0.01
		self.moveToPose(backup_position, 'arm_calib_link', listen)

		return [calib_pose.pose.position.x, calib_pose.pose.position.y, calib_pose.pose.position.z] 

	def calculateTransformation(self, ref_pos, calib_pos):

		#make homogenous vector
 		for i in range(0, len(ref_pos)):
			ref_pos[i].append(1.0)
			calib_pos[i].append(1.0)

		#add to an array A
		A = numpy.array(calib_pos)
		print 'Matrix A: '
		print A
		
		#create matrix
		matrix = []

		for i in range (0,4):
			#right side b
			b = numpy.array([])
			b = numpy.array([ref_pos[0][i], ref_pos[1][i], ref_pos[2][i], ref_pos[3][i], ref_pos[4][i], 
							 ref_pos[5][i], ref_pos[6][i], ref_pos[7][i], ref_pos[8][i], ref_pos[9][i] ])

			print 'Matrix b: '
			print b

			#get A*x = b 
			x_lstsq = numpy.linalg.lstsq(A, b, rcond=-1)
			print 'Residuals: '
			print x_lstsq[1]

			#fill the matrix with the seperate solved vectors
			matrix.append(x_lstsq[0])

		#homogenous matrix for tf package, inverse is needed for the correct solution
		print "Matrix of x: "
		print matrix
		matrix_4x4= (numpy.matrix(matrix)).I #inverse of the matrix for right trafo
		print "Matrix of x.I: "
		print matrix_4x4

		#create translation vector for the broadcasting of the static frame
		calibrated_position = [matrix_4x4[0,3], matrix_4x4[1,3], matrix_4x4[2,3]]
		calibrated_quat = quaternion_from_matrix(matrix_4x4)

		rospy.loginfo('The calibrated pose is: \n'+str(calibrated_position)+'\n'+str(calibrated_quat))
		self.publishStaticLink(calibrated_position, calibrated_quat, 'table_link')

	#publish static link
	def publishStaticLink(self, position, quat, name):
		br = tf.TransformBroadcaster()
		rate = rospy.Rate(20.0)
		while not rospy.is_shutdown():
			br.sendTransform(position, quat, rospy.Time.now(),name,"base_link")
		rate.sleep()

	def Run(self):

		#publish a static link that is approximately near to the table_link
		control_position = [-0.051, -0.752-0.03, 0.77]
		control_quat = [0.707, 0.000, 0.000, 0.707]
		
		control_link_thread = Thread(target=self.publishStaticLink, args=(control_position, control_quat, 'predefined_table_link'))
		control_link_thread.start()

		listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2) #sleep necessary here, because tf needs it	
		
		#move sdh not to collide with the object
		handle_sdh = sss.move('sdh', [[0.0, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57]])			
		handle_sdh.wait()

		#here the reference points will be filled in
		reference_poses = []
		number_ref_pos = 10		
		
		#starting position: point 1
		#in approximate table frame
		ref_pose = PoseStamped()
		ref_pose.header.frame_id = 'predefined_table_link'
		ref_pose.header.stamp = rospy.Time.now()
		ref_pose.pose.position.x = 0.0
		ref_pose.pose.position.y = 0.149
		ref_pose.pose.position.z = 0.0
		ref_pose.pose.orientation.x, ref_pose.pose.orientation.y, ref_pose.pose.orientation.z, ref_pose.pose.orientation.w = [-0.7071, 0.0, 0.0,0.7071]#[0.5, -0.5, 0.5, 0.5]

		#fill whole array with poses to get same orientation
		for i in range (0,number_ref_pos):
			reference_poses.append(copy.deepcopy(ref_pose))

		#rest of the ref poses here
		offset_helper = 0.025
		offset_helper_cylinder = 0.149
		reference_poses[1].pose.position.x, reference_poses[1].pose.position.y, reference_poses[1].pose.position.z = [  0.000, -0.005+offset_helper,  0.1305]
		reference_poses[2].pose.position.x, reference_poses[2].pose.position.y, reference_poses[2].pose.position.z = [-0.1305, -0.005+offset_helper, 0.1305]
		reference_poses[3].pose.position.x, reference_poses[3].pose.position.y, reference_poses[3].pose.position.z = [-0.1305, -0.005+offset_helper_cylinder,   0.000]
		reference_poses[4].pose.position.x, reference_poses[4].pose.position.y, reference_poses[4].pose.position.z = [-0.1305, -0.005+offset_helper, -0.1305]
		reference_poses[5].pose.position.x, reference_poses[5].pose.position.y, reference_poses[5].pose.position.z = [  0.000, -0.005+offset_helper, -0.1305]
		reference_poses[6].pose.position.x, reference_poses[6].pose.position.y, reference_poses[6].pose.position.z = [ 0.1305, -0.005+offset_helper, -0.1305]
		reference_poses[7].pose.position.x, reference_poses[7].pose.position.y, reference_poses[7].pose.position.z = [ 0.1305, -0.005+offset_helper_cylinder,   0.000]
		reference_poses[8].pose.position.x, reference_poses[8].pose.position.y, reference_poses[8].pose.position.z = [ 0.1305, -0.005+offset_helper,  0.1305]
		reference_poses[9].pose.position.x, reference_poses[9].pose.position.y, reference_poses[9].pose.position.z = [ 0.0000, -0.005+offset_helper,  0.0000]

		#check if ik solution can be found
		check_ik = False
		if check_ik:
			for i in range(0, len(reference_poses)):
				lower_point = None
				lower_point = copy.deepcopy(reference_poses[i])
				lower_point.pose.position.y -= 0.05 
				found = self.checkIK(reference_poses[i], 'arm_calib_link', listener)
				found_lower = self.checkIK(lower_point, 'arm_calib_link', listener)
				rospy.loginfo('IK solution for reference point '+str(i)+' found: '+str(found))
				rospy.loginfo('Lower IK solution for reference point '+str(i)+' found: '+str(found_lower))
	
		#end control_link here
		if control_link_thread.isAlive():		
			try:
				control_link_thread._Thread__stop()
			except:
				rospy.logerr((str(thread.getName()) + ' could not be terminated'))

		#calibration loop here
		calibrated_points = []
		reference_points = [] 
		for i in range(0, len(reference_poses)):
			rospy.loginfo('Moving to reference point num. '+str(i)+': \n'+str(reference_poses[i].pose.position))
			calib_point = self.calibrate(reference_poses[i], listener)
			#accumulate the calibrated positions and the reference positions for the calculation
			print calib_point
			calibrated_points.append(calib_point)
			reference_points.append([reference_poses[i].pose.position.x, 
									 reference_poses[i].pose.position.y, 
									 reference_poses[i].pose.position.z])

		#calculate the new quaternions for the static link here
		self.calculateTransformation(reference_points, calibrated_points)

		exit()

if __name__ == "__main__":
	SCRIPT = CalibScript()
	SCRIPT.Start()



