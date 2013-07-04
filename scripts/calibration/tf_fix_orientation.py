""" Fixed TF trafo
"""
import roslib
roslib.load_manifest('cob_script_server')
import rospy
import tf

from kinematics_msgs.srv import *
from tf.transformations import *
from simple_script_server import *
from math import sqrt


if __name__ == "__main__":

		rospy.init_node('tf_fix_orientation')

		actual_quat = [0.70910838, -0.0133156,  -0.01117576,  0.6995914] #calibration result
		#actual_quat = [0.707, 0.000, 0.000, 0.707] #origin of grasp link	
		br = tf.TransformBroadcaster()
		rate = rospy.Rate(20.0)
		#position = (-0.071, -0.732, 0.82)
		position = (-0.046992319701331865, -0.78164652596002626, 0.76986827564730753)
		while not rospy.is_shutdown():
			br.sendTransform(position,actual_quat,rospy.Time.now(),"table_link","base_link")
		rate.sleep()



