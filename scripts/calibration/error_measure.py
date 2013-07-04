""" Calibration of the new grasp coordinate system
"""
import roslib
roslib.load_manifest('cob_script_server')
import rospy, csv, os, datetime
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

class TestPos(script):

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
		msg = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState)

		req.header.stamp = rospy.Time.now()
		req.header.frame_id='base_link'
		req.fk_link_names = links
		req.robot_state.joint_state.name = ['sdh_knuckle_joint', 'sdh_finger_12_joint', 'sdh_finger_13_joint', 'sdh_finger_22_joint', 'sdh_finger_23_joint', 'sdh_thumb_2_joint', 'sdh_thumb_3_joint'] + ['arm_%d_joint'%(d+1) for d in range (7)]
		req.robot_state.joint_state.position = list(msg.actual.positions) + joint_values #[0]*7 + joint_values
		print req.robot_state.joint_state.position		
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

		pose.header.stamp = rospy.Time.now()
		
		solution_found = False
		counter = 0
		while not solution_found:
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
			if counter > 3:
				handle_arm = sss.move_planned('arm','pregrasp')
				handle_arm.wait()
				counter = 0

		if not planned:

			handle_arm = sss.move('arm',[list(sol)])
		else:
			handle_arm = sss.move_planned('arm',[list(sol)])

		handle_arm.wait()

		return list(sol), [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]

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

	def Run(self):

		target_frame = 'table_link'	

		listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2) #sleep necessary here, because tf needs it

		#move sdh not to collide with the object
		handle_sdh = sss.move('sdh', [[0.0, -1.57, -1.57, -1.57, -1.57, -1.57, -1.57]])			
		handle_sdh.wait()
	
		#test position 1
		rospy.loginfo('Moving to test position for calibration results')
		test_pose = PoseStamped()
		test_pose.header.frame_id = target_frame
		test_pose.header.stamp = rospy.Time.now()
		test_pose.pose.position.x = 0.0
		test_pose.pose.position.y = 0.05
		test_pose.pose.position.z = 0.0
		test_pose.pose.orientation.x, test_pose.pose.orientation.y, test_pose.pose.orientation.z, test_pose.pose.orientation.w = [-0.7071, 0.0, 0.0,0.7071]
		#self.moveToPose(test_pose, 'arm_calib_link', listener)


		test_pose_low = copy.deepcopy(test_pose)
		test_pose_low.pose.position.y = 0.02
		#self.moveToPose(test_pose_low, 'arm_calib_link', listener)
		#raw_input('Next')
	
		#test position 2
		test_pose2 = copy.deepcopy(test_pose)
		test_pose2.pose.position.z = 0.1305
		#self.moveToPose(test_pose2, 'arm_calib_link', listener)

		test_pose2_low = copy.deepcopy(test_pose2)
		test_pose2_low.pose.position.y = 0.02
		#self.moveToPose(test_pose2_low, 'arm_calib_link', listener)
		#raw_input('Next')
		
		#test position 3
		test_pose3 = copy.deepcopy(test_pose)
		test_pose3.pose.position.z = -0.1305
		test_pose3.pose.position.x = 0.1305
		#self.moveToPose(test_pose3, 'arm_calib_link', listener)

		test_pose3_low = copy.deepcopy(test_pose3)
		test_pose3_low.pose.position.y = 0.02
		#self.moveToPose(test_pose3_low, 'arm_calib_link', listener)
		#raw_input('Next')


		#create directories
		time = datetime.datetime.now()
		name = 'error_measure_'+str(time.day)+'-'+str(time.month)+'-'+str(time.year)
		directory = roslib.packages.get_pkg_dir('cob_offline_grasping')+'/grasping/calibration_measure'
		if not os.path.exists(directory):
				os.makedirs(directory)
		pathname_out = directory+'/'+name+'.csv'

		f_out = open(pathname_out, 'w+')
		wr = csv.writer(f_out, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL) #quoting=csv.QUOTE_MINIMAL)	

		#write structure of csv error-log file
		wr.writerow(['reference.position', 'ref.pos.x', 'ref.pos.y', 'ref.pos.z', 'measure.pos.x', 'measure.pos.y', 'measure.pos.z', 'sol.arm_1_joint', 'sol.arm_2_joint', 'sol.arm_3_joint', 'sol.arm_4_joint', 'sol.arm_5_joint','sol.arm_6_joint','sol.arm_7_joint', 'meas.arm_1_joint', 'meas.arm_2_joint', 'meas.arm_3_joint', 'meas.arm_4_joint', 'meas.arm_5_joint', 'meas.arm_6_joint', 'meas.arm_7_joint'])

		#positions to test
		test_positions = []
		test_positions.append(test_pose)
		test_positions.append(test_pose3)	

		#first error measure
		for x in range (0,len(test_positions)):
		
			#calculate the IK first before moving to position, 
			#for not calculating it everytime we move on the same position
			solution_found = False
			counter = 0

			while not solution_found:

				test_positions[x].header.stamp = rospy.Time.now()
				rospy.sleep(2)

				test_pose_pre_BL = listener.transformPose('/base_link',test_positions[x])
				print test_positions[x]
				print test_pose_pre_BL
				solution_joint_pre, error_code = self.callIKconstraint(test_pose_pre_BL,'arm_calib_link')
				print error_code
			
				if error_code.val == 1:
					solution_found = True
					rospy.loginfo('IK Solution found!')
		
				counter += 1
				if counter > 5:
					handle_arm = sss.move_planned('arm','pregrasp')
					handle_arm.wait()
					counter = 0
						
			#the actual position here
			#deepcopy to create a lower position
			test_position_low = copy.deepcopy(test_positions[x])
			test_position_low.pose.position.y -= 0.03

			solution_found = False
			counter = 0

			while not solution_found:

				test_positions[x].header.stamp = rospy.Time.now()
				rospy.sleep(2)

				test_pose_BL = listener.transformPose('/base_link',test_position_low)
				solution_joint, error_code = self.callIKconstraint(test_pose_BL,'arm_calib_link')
			
				if error_code.val == 1:
					solution_found = True
					rospy.loginfo('IK Solution found!')
		
				counter += 1
				if counter > 5:
					handle_arm = sss.move_planned('arm','pregrasp')
					handle_arm.wait()
					counter = 0

			solution_cart = [test_position_low.pose.position.x, test_position_low.pose.position.y, test_position_low.pose.position.z]

			#inner loop for repeating the generated joint positions
			for i in range (0,5):
	
				handle_arm = sss.move_planned('arm','home')
				handle_arm.wait()

				#move planned to preshape
				handle_arm = sss.move_planned('arm',[list(solution_joint_pre)])
				handle_arm.wait()
				
				#move to actual position
				handle_arm = sss.move('arm',[list(solution_joint)])
				handle_arm.wait()
	
				#measure of the actual joint positions
				msg = rospy.wait_for_message("/arm_controller/state", JointTrajectoryControllerState)
				measure_joint = list(msg.actual.positions)

				#measure of the actual cart. positions
				#cartesian space		
				measure_cart = self.callFK(measure_joint, ['arm_calib_link'])[0]
			
				#transform from base_link in target_frame
				rospy.sleep(2)
				measure_cart_target = listener.transformPose(target_frame, measure_cart)
	
				#here the files will writen to the csv
				#row to publish in csv
				row = []
				row.append('No. '+str(x)+' - Frame: '+target_frame)

				#add measure and solution of cartesian position to publish
				for i in range(0,len(solution_cart)):
					row.append(solution_cart[i])

				row.append(measure_cart_target.pose.position.x)
				row.append(measure_cart_target.pose.position.y)
				row.append(measure_cart_target.pose.position.z)

				#add measure and solution of joint space to publish
				for i in range(0,len(solution_joint)):
					row.append(solution_joint[i])

				for i in range(0,len(measure_joint)):
					row.append(measure_joint[i])

				wr.writerow(row)

				#move back to preshape
				handle_arm = sss.move('arm',[list(solution_joint_pre)])
				handle_arm.wait()

		f_out.close()
		print "Finished."
		exit()



if __name__ == "__main__":
	SCRIPT = TestPos()
	SCRIPT.Start()



