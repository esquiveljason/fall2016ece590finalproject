#! /usr/bin/env python

import numpy as np
import rospy as rp
from baxterStructure import *
import baxter_interface as bi
import ach

def toBaxterOffset(arm, joint, ref):
	thetaOffsetRight	= np.array([ np.pi/4.0, 0.0, 0.0, np.pi/2.0, 0.0, 0.0, 0.0])
	thetaOffsetLeft		= np.array([-np.pi/4.0, 0.0, 0.0, np.pi/2.0, 0.0, 0.0, 0.0])
	directionOffset		= np.array([       1.0, 1.0, 1.0,       1.0,-1.0, 1.0,-1.0])   
	
	newRef = 0.0
	
	if arm == LEFT_ARM:
		newRef = ref + thetaOffsetLeft[joint]
	if arm == RIGHT_ARM:
		newRef = ref + thetaOffsetRight[joint]
	
	newRef = newRef * directionOffset[joint]
	
	return newRef

def toBaxterDictionary(robot , leftArm , rightArm):
	leftJointName = leftArm.joint_names()
	rightJointName = rightArm.joint_names()
	leftAngles = {} #dictionaries to Baxter
	rightAngles = {}

	for i in range(BAXTER_NUM_ARM_JOINTS):

		leftAngles.update( { leftJointName[i] : toBaxterOffset( LEFT_ARM, i, robot.arm[ LEFT_ARM].joint[i].ref) })
		rightAngles.update({rightJointName[i] : toBaxterOffset(RIGHT_ARM, i, robot.arm[RIGHT_ARM].joint[i].ref) })

	return leftAngles , rightAngles

def main():

	rp.init_node("baxterInterface" , anonymous = True)

	robot = ROBOT()
	leftArm = bi.Limb("left")
	rightArm = bi.Limb("right")

	r = ach.Channel("baxterRef")
	r.flush()

	while True:
		[statuss , frameSizes] = r.get(robot, wait = True , last = True)
		print "Received new Data at {}".format(robot.currTime)
		leftAngle , rightAngle = toBaxterDictionary( robot , leftArm , rightArm )

		try:
			leftArm.move_to_joint_positions(leftAngle)
			rightArm.move_to_joint_positions(rightAngle)
		except:
			print "nothing came through"
		rp.Rate(1).sleep()
		
		
if __name__=="__main__":
    main()
