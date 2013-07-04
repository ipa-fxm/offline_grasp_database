"""Converts the generated *.txt files from Graspit to the ROS structure
"""
#version of openrave to use
__builtins__.__openravepy_version__ = '0.8'

from openravepy import *
import numpy, time, analyzegrasp3d

file_in = 'mildessa_raw-17-04'

analyzegrasp3d.graspit_to_csv(file_in)
