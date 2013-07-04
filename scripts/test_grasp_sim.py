#!/usr/bin/python

import roslib
roslib.load_manifest('cob_offline_grasping')
import rospy, csv, os, copy, datetime
import grasp_handle #own function-lib
import tf

from tf.transformations import *
from kinematics_msgs.srv import *
from simple_script_server import *
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import *
sss = simple_script_server()

#####	Description #####
#Here we will choose a grasp planner and an object ID. The script will 
#get one best TOP and one best SIDE for each metric we are using.
#To get a higher sample size, a number of random grasps will also be 
#added to the grasps for testing.

#Global vars
planner_name = 'Graspit' # choose planner here ['Openrave', 'Graspit']
object_name = 'salt' # choose object
number_random_grasps = 2 # choose here the number of random grasps that will be added to the test

class GraspScript(script):

	def Initialize(self):
		#self.sss.init("torso")
		self.sss.init("sdh")
		self.sss.init("arm")
		#self.sss.move("torso","home")

		#IK Service
		rospy.wait_for_service('/cob_ik_wrapper/arm/get_ik')
		rospy.wait_for_service('/cob_ik_wrapper/arm/get_fk')
		rospy.wait_for_service('/cob_ik_wrapper/arm/get_constraint_aware_ik')

		#rospy.wait_for_service('/cob_ik_wrapper/arm/get_constraint_aware_ik')
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
		req.robot_state.joint_state.position = [0]*7 + joint_values
		fks = rospy.ServiceProxy('/cob_ik_wrapper/arm/get_fk', GetPositionFK)
		res = self.fks(req)
		#print req
		print res.error_code
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
		print res.error_code.val
		return res.solution.joint_state.position if res.error_code.val == res.error_code.SUCCESS else None, res.error_code

	def takeGrasp(self, new_grasp):
		
		#grasps in the grasp table are in mm unit. swtich to m for ROS and fill pose stamped	
		mm_to_m = 1.0/1000.0
		rospy.loginfo("Actual grasp is a "+new_grasp['direction']+" grasp")
		
		#fill msg type
		grasp = PoseStamped()
		grasp.header.stamp = rospy.Time.now()
		grasp.header.frame_id = "/table_link"
		grasp.pose.position.x = float(new_grasp['pos-x']) * mm_to_m
		grasp.pose.position.y = float(new_grasp['pos-y']) * mm_to_m
		grasp.pose.position.z = float(new_grasp['pos-z']) * mm_to_m
		grasp.pose.orientation.x, grasp.pose.orientation.y, grasp.pose.orientation.z, grasp.pose.orientation.w = [float(new_grasp['qx']), float(new_grasp['qy']), float(new_grasp['qz']), float(new_grasp['qw'])]

		#fill joint values for final config of sdh here
		sdh_fin = [float(new_grasp['sdh_knuckle_joint']), float(new_grasp['sdh_thumb_2_joint']), float(new_grasp['sdh_thumb_3_joint']), float(new_grasp['sdh_finger_12_joint']), float(new_grasp['sdh_finger_13_joint']), float(new_grasp['sdh_finger_22_joint']), float(new_grasp['sdh_finger_23_joint'])]		
	
		return grasp, sdh_fin


	def moveToPreshape(self, grasp, listen):

		#get preshape (formula from own function library)
		pre = grasp_handle.getPreshapeCart(grasp, 0.03) #grasp and distance in mm

		rospy.sleep(2)
		pre0 = listen.transformPose('/base_link',pre)
		rospy.sleep(2)

		#call IKSolver here and move arm in joint configuration space
		sol, error_code = self.callIKconstraint(pre0,'sdh_palm_link')
		handle_arm = sss.move_planned('arm',[list(sol)])	
		handle_arm.wait()

		handle_arm = sss.move('arm',[list(sol)])	
		handle_arm.wait()

	#drive SDH to joint config. if a grasp dont close enough, put offset to finger
	def moveSDH(self, grasp, sdh_config, preshape=False, offset=0.0):

		if preshape == True:
			if sdh_config[0] >= 0.1:
				#if grasp['direction'] == 'TOP':

				handle_sdh = sss.move('sdh', "spheropen")
			else:
				handle_sdh = sss.move('sdh', "cylopen")

				
			handle_sdh.wait()

		if preshape == False:
				sdh_config_offset = copy.deepcopy(sdh_config)
				sdh_config_offset[1] += offset
				sdh_config_offset[3] += offset
				sdh_config_offset[5] += offset
				handle_sdh = sss.move('sdh', [sdh_config_offset])
				handle_sdh.wait()

	#Move to a pose in cart coord.
	def moveToPose(self, pose, listen, plan=False):

		pose.header.stamp = rospy.Time.now()
		rospy.sleep(2)
		pose0 = listen.transformPose('/base_link',pose)
		sol, error_code = self.callIKconstraint(pose0,'sdh_palm_link')

		if plan == False:
			handle_arm = sss.move('arm',[list(sol)])
		else:
			handle_arm = sss.move_planned('arm',[list(sol)])

		handle_arm.wait()

	#Get and Set the rotation for the Drehteller
	def getRotationPlateAndGrasp(self, grasp):

		palm_pos = [grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z] #in table_link
		root = [0, 0, 0] #in table_link
		vec = grasp_handle.sub(palm_pos, root) #(palm position - root)

		vec_xz = grasp_handle.normalize([vec[0],vec[2]]) #projected onto the x-z plane
		x_value = vec_xz[0]
		z_value = vec_xz[1]

		grasp_position = numpy.array([-1, 0]) #-+pi around y-Axis
		delta_unitCircle = numpy.absolute(grasp_position) - numpy.absolute(vec_xz)

		#rotation will be filled with the rotation difference to the actual grasp
		rotation = None 
		if z_value < 0: #quadrant 1-2
			if x_value > 0:
				rotation = - 180 - numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '1. quadrant | rot: ', rotation
			elif x_value == 0:
				rotation = - 180 - numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '1. quadrant | rot: ', rotation
			elif x_value < 0:
				rotation = numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '2. quadrant | rot: ', rotation
		elif z_value == 0:
			if x_value < 0: #quadr. 2
				rotation = numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '2. quadrant | rot: ', rotation
			elif x_value > 0: #quadr. 4
				rotation = - 180 + numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '4. quadrant | rot: ', rotation
		else: #quadrant 3-4
			if x_value > 0: #quadr. 4
				rotation = - 180 + numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '4. quadrant | rot: ', rotation
			elif x_value == 0: #quadr. 3
				rotation = - 360 - numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '3. quadrant | rot: ', rotation
			elif x_value < 0: #quadr. 3
				rotation = - 360 - numpy.arcsin(delta_unitCircle[1])*180/numpy.pi
				#print '3. quadrant | rot: ', rotation
		
		plate_rot = 360.0 + rotation

		#define offset for workspace advantages
		offset_grasp = 18.0

		#check if plate was rotated
		is_set = False
		while not is_set:
			rospy.loginfo('Set the marker on '+str(plate_rot+offset_grasp)+' degrees.')
			check = raw_input('Rotation finished? Type OK to continue: ')
			if check == 'OK':
				is_set = True

		#input
		new_grasp = grasp_handle.transformGrasp(grasp, plate_rot, offset=offset_grasp)
	
		return new_grasp

	def publishGraspApproach(self,grasp):

		pub = rospy.Publisher('grasp_approach_dir', PoseStamped)

		approach_pose = copy.deepcopy(grasp)
		approach_pose.header.stamp = rospy.Time.now()

		print approach_pose

		old_quat = [approach_pose.pose.orientation.x, approach_pose.pose.orientation.y, approach_pose.pose.orientation.z, approach_pose.pose.orientation.w ]
		new_quat = quaternion_multiply(old_quat, [0.0, -numpy.sqrt(0.5), 0.0, numpy.sqrt(0.5)])

		approach_pose.pose.orientation.x = new_quat[0]
		approach_pose.pose.orientation.y = new_quat[1]
		approach_pose.pose.orientation.z = new_quat[2]
		approach_pose.pose.orientation.w = new_quat[3]

		for i in range(0,2):
			pub.publish(approach_pose)
			rospy.sleep(3.0)

		rospy.loginfo('Grasp approach direction was published.')

	#log the results of a test
	def logTest(self, log):

		result = None

		while (not result == 's') and (not result == 'l') and (not result == 'Fail'):		
			result = raw_input('(s)uccess, s(l)ipped, (Fail)ed: ')

		if result == 's':
			observe = 1

		elif result == 'l':
			observe = 0.5

		else:
			observe = 0

		#add result to log
		log.append(str(observe))

		#return value for quitting if necessary
		return observe
		
	def testProcedure(self, choosen_grasp):

			#grasp_log will record the results
			grasp_log = []

			#get raw grasp data and the final config for sdh in joint space
			grasp_raw, sdh_final = self.takeGrasp(choosen_grasp)
		
			#transform grasp and rotate grasp-plate
			grasp = self.getRotationPlateAndGrasp(grasp_raw)

			listener = tf.TransformListener(True, rospy.Duration(10.0))
			rospy.sleep(2) #sleep necessary here, because tf needs it	

			#publish the arrow for vizualising the grasp in rviz
			self.publishGraspApproach(grasp)

			### 	Movement begins here 	###
	
			handle_arm = sss.move_planned("arm","home")
			handle_arm.wait()

			self.moveSDH(choosen_grasp,sdh_final, preshape=True)

			raw_input('Move to Preshape of grasp..')

			#move to the grasp preshape
			rospy.loginfo('Moving to Preshape of the grasp!')
			self.moveToPreshape(grasp, listener)
			raw_input('Any key to continue...')

			#move to grasp
			self.moveToPose(grasp, listener) # unplanned
			raw_input('Next: Grasp the object')

			#TEST NO.1: grasp object
			self.moveSDH(choosen_grasp, sdh_final, preshape=False, offset=0.1)
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log
			raw_input('Next: Lift object')

			#TEST NO.2: lift object
			lift_grasp = copy.deepcopy(grasp)
			lift_grasp.pose.position.y += 0.06 # the y-axis is the vertical axis of the object
			self.moveToPose(lift_grasp, listener)
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log
			raw_input('Next: Drive to start position')

			#drive to start pose
			rospy.loginfo('Drive to the start position to begin the grasp tests.')
			start_pos = [1.333, -1.395, 3.913, -1.641, 3.067, 1.0003, 3.0]
			handle_arm = sss.move_planned('arm',[start_pos])
			handle_arm.wait()

			#raw_input('Next step..')

			#rot_begin = [1.841685, -1.7419733, 3.9, -0.44, 3.064032, 0.0, 3.165]
			#handle_arm = sss.move('arm',[rot_begin]) 
			#handle_arm.wait()
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log
	 

			#1st spin
			raw_input('Next: 1st spin')
			spin1_checkpoint = [1.841685, -1.7419733, 3.9, 1.68, 3.14, 0.0, 3.165]
			handle_arm = sss.move('arm',[spin1_checkpoint]) 
			handle_arm.wait()
			spin1_end = [1.333, -1.395, 3.913, -1.641, 3.067, 1.0003, 3.0]
			handle_arm = sss.move('arm',[spin1_end])
			handle_arm.wait()
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log


			#2nd spin - 90 degree rotation of arm_7_link
			raw_input('Next: 1st spin 90 degree rotated..')
			spin2_begin = [1.333, -1.395, 3.913, -1.641, 3.067, 1.0003, 1.60]
			handle_arm = sss.move('arm',[spin2_begin])
			handle_arm.wait()
			spin2_cp = [1.841685, -1.7419733, 3.9, 1.6, 3.14, 0.0, 1.435]
			handle_arm = sss.move('arm',[spin2_cp])
			handle_arm.wait()
			spin2_end = [1.333, -1.395, 3.913, -1.641, 3.067, 1.0003, 1.60]
			handle_arm = sss.move('arm',[spin2_end])
			handle_arm.wait()
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log


			#3nd spin - -90 degree rotation of arm_7_link
			raw_input('Next: 1st spin -90 degree rotated..')
			spin3_begin = [1.333, -1.395, 3.913, -1.641, 3.067, 1.0003, 4.735796327]
			handle_arm = sss.move('arm',[spin3_begin])
			handle_arm.wait()
			spin3_cp = [1.841685, -1.7419733, 3.9, 1.6, 3.14, 0.0, 4.570796327]
			handle_arm = sss.move('arm',[spin3_cp])
			handle_arm.wait()
			spin3_end = [1.333, -1.395, 3.913, -1.641, 3.067, 1.0003, 4.735796327]
			handle_arm = sss.move('arm',[spin3_end])
			handle_arm.wait()
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log


			#hard dash
			raw_input('Next: hard dash..')
			dash_begin = [1.841685, -1.7419733, 2.378, -1.38, 3.064032, 0.0, 1.60]
			handle_arm = sss.move('arm',[dash_begin])
			handle_arm.wait()
			dash_cp = [1.841685, -1.7419733, 2.378, 1.0, 3.064032, 0.0, 1.60]
			handle_arm = sss.move('arm',[dash_cp])
			handle_arm.wait()
			dash_cp2 = [1.841685, -1.7419733, 2.378, -1.38, 3.064032, 0.0, 1.60]
			handle_arm = sss.move('arm',[dash_cp2])
			handle_arm.wait()
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log


			#hard dash - 90 degree rot
			raw_input('Next: hard dash 90 degree rotated..')
			dash2_begin = [1.841685, -1.7419733, 2.378, -1.38, 3.064032, 0.0, 3.165]
			handle_arm = sss.move('arm',[dash2_begin])
			handle_arm.wait()
			dash2_cp = [1.841685, -1.7419733, 2.378, 1.0, 3.064032, 0.0, 3.165]
			handle_arm = sss.move('arm',[dash2_cp])
			handle_arm.wait()
			dash2_cp2 = [1.841685, -1.7419733, 2.378, -1.38, 3.064032, 0.0, 3.165]
			handle_arm = sss.move('arm',[dash2_cp2])
			handle_arm.wait()
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log

			#hard dash - -90 degree rot
			raw_input('Next: hard dash -90 degree rotated..')
			dash3_begin = [1.841685, -1.7419733, 2.378, -1.38, 3.064032, 0.0, 0.03]
			handle_arm = sss.move('arm',[dash3_begin])
			handle_arm.wait()
			dash3_cp = [1.841685, -1.7419733, 2.378, 1.0, 3.064032, 0.0, 0.03]
			handle_arm = sss.move('arm',[dash3_cp])
			handle_arm.wait()
			dash3_cp3 = [1.841685, -1.7419733, 2.378, -1.38, 3.064032, 0.0, 0.03]
			handle_arm = sss.move('arm',[dash3_cp3])
			handle_arm.wait()
			if self.logTest(grasp_log) == 0: ## LOG ##
				return grasp_log
		
			return grasp_log

	def Run(self):
		#create directories
		time = datetime.datetime.now()
		name = object_name+'-'+str(time.day)+'-'+str(time.month)+'-'+str(time.year)
		directory = roslib.packages.get_pkg_dir('cob_offline_grasping')+'/grasping/grasp_results/'+planner_name
		if not os.path.exists(directory):
				os.makedirs(directory)

		#output dir
		pathname_out = directory+'/'+name+'.csv'

		if not os.path.isfile(pathname_out): #if file is empty, write the row
			f_out = open(pathname_out, 'w+')
			wr = csv.writer(f_out, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
			#write structure of csv grasp file
			wr.writerow(['object name', 'grasp direction', 'ID', 'choose_crit', 'eps_l1', 'vol_l1', 'offset_fingers', '1:object_grasped', '2:lifted',		 							 '3:moved_start', '4:1st_spin', '5:1st_spin_90', '6:1st_spin_-90', '7:hard_dash', '8:hard_dash_90','9:hard_dash_-90'])
			f_out.close()

		#Here the grasps are fetched
		IDs = [] #prevent from take, used grasps a second time
		
		grasps_eps, rest_eps_top, rest_eps_side, IDs = grasp_handle.getGraspCollection(planner_name,object_name,'eps_l1', IDs)
		grasps_vol, rest_vol_top, rest_vol_side, IDs = grasp_handle.getGraspCollection(planner_name,object_name,'vol_l1', IDs)

		#pick 2 more random grasps
		random_grasps, IDs = grasp_handle.getRandomGrasps(number_random_grasps, planner_name, object_name, IDs)

		print IDs
		#actual grasp we are testing or working on
		actual_grasp =  grasps_eps[0] # @TODO: Fill grasp in a loop here.
		#actual_grasp = grasp_handle.getGraspWithID(25, planner_name, object_name)	
		print "ID: ", actual_grasp['id']
		############	 META-DATA FOR .CSV  ############

		#decide here what to write to the csv log file
		graspresults = []
		graspresults.append(object_name)
		graspresults.append(actual_grasp['direction'])
		graspresults.append(actual_grasp['id'])
		
		#describe choose criteria here BEST/RANDOM
		if [s for s in grasps_eps if actual_grasp['id'] in s['id']]:
			graspresults.append('BEST EPS')
		elif [s for s in grasps_vol if actual_grasp['id'] in s['id']]:
			graspresults.append('BEST VOL')
		elif [s for s in random_grasps if actual_grasp['id'] in s['id']]:
			graspresults.append('RANDOM')

		graspresults.append(actual_grasp['eps_l1'])
		graspresults.append(actual_grasp['vol_l1'])

		############	 META-DATA FOR .CSV  ############

		grasp_log = self.testProcedure(actual_grasp)
		graspresults.extend(grasp_log)

		#mode append
		with open(pathname_out, 'a+') as f_out:
			wr = csv.writer(f_out, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
			f_out.seek(0, os.SEEK_END)
			wr.writerow(graspresults)

		exit()

if __name__ == "__main__":
	SCRIPT = GraspScript()
	SCRIPT.Start()


