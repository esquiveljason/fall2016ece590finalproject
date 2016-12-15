#! /usr/bin/env python

import numpy as np
import rospy as rp
from baxterStructure import *
import baxter_interface as bi
import ach
import time
import math

def fromBaxterOffset(arm, joint, ref):
	thetaOffsetRight	= np.array([ np.pi/4.0, 0.0, 0.0, np.pi/2.0, 0.0, 0.0, 0.0])
	thetaOffsetLeft		= np.array([-np.pi/4.0, 0.0, 0.0, np.pi/2.0, 0.0, 0.0, 0.0])
	directionOffset		= np.array([       1.0, 1.0, 1.0,       1.0,-1.0, 1.0,-1.0])   
	
	newRef = ref * directionOffset[joint]
	
	if arm == LEFT_ARM:
		newRef = newRef - thetaOffsetLeft[joint]
	if arm == RIGHT_ARM:
		newRef = newRef - thetaOffsetRight[joint]

	return newRef


def _fromBaxterDictionary(leftArm , rightArm):
	leftJointName = leftArm.joint_names()
	rightJointName = rightArm.joint_names()

	robot = ROBOT()

	for i in range(BAXTER_NUM_ARM_JOINTS):

		robot.arm[ LEFT_ARM].joint[i].ref = fromBaxterOffset( LEFT_ARM, i,  leftArm.joint_angle( leftJointName[i]))
		robot.arm[RIGHT_ARM].joint[i].ref = fromBaxterOffset(RIGHT_ARM, i, rightArm.joint_angle(rightJointName[i]))
	
	for i in range(BAXTER_NUM_ARM_JOINTS):	
		print "i = {} Joint = {} Value = {:06.4f} Joint = {} Value = {:06.4f}".format(i, leftJointName[i] , math.degrees(robot.arm[LEFT_ARM].joint[i].ref) , 
																			  rightJointName[i] , math.degrees(robot.arm[RIGHT_ARM].joint[i].ref))
	return robot
	
	
def main():

	rp.init_node("fromBaxterInterface" , anonymous = True)

	# robot = ROBOT()
	leftArm = bi.Limb("left")
	rightArm = bi.Limb("right")

	s = ach.Channel("baxterState")
	# angles_right = {'right_s0': 0.0, 'right_s1': 0.0, 'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0, 'right_e0': 0.0, 'right_e1': 0.0}
	# angles_left = {'left_s0': 0.0, 'left_s1': 0.0, 'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0}

	while True:

		print 'waiting'

		robot = _fromBaxterDictionary(  leftArm , rightArm )
		robot.currTime = time.time()
		print robot.currTime
		s.put(robot)

		rp.Rate(1).sleep()


if __name__=="__main__":
    main()
