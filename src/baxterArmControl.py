#!/usr/bin/env python

import ach
import sys
import time
from ctypes import *

import math
import numpy as np

LEFT_ARM = 0
RIGHT_ARM = 1

#Length of arm components
#						X      Y    Z
S0_LENGTHS = np.array([ 69.00, 0.0, 270.35])
E0_LENGTHS = np.array([364.35, 0.0, -69.00])
W0_LENGTHS = np.array([-10.00, 0.0,-374.29])
#						X      Y    Z
#S0_LENGTHS = np.array([  0.0 ,  -69.00,  270.35])
#E0_LENGTHS = np.array([  0.0 , -364.35,  -69.00])
#W0_LENGTHS = np.array([  0.0,   10.0 , -374.29])


#IK Constatns
DELTA_THETA = 0.001
ERROR       = 5 # PLACEHOLDER - ERROR FROM GOAL
ENDEFF_STEP = 2# PLACEHOLDER - STEP_SIZE


LEFT_GOAL_TOP_LEFT  = np.array([[807.64], [0.0], [191.35]])



# Rotation matrices
def rX(theta):
	R = np.identity(4)
	R[1,1] = np.cos(theta)
	R[1,2] = np.sin(theta) * -1.0
	R[2,1] = np.sin(theta)
	R[2,2] = np.cos(theta)
	
	return R

def rY(theta):
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,2] = np.sin(theta)
	R[2,0] = np.sin(theta) * -1.0
	R[2,2] = np.cos(theta)

	return R

def rZ(theta):
	R = np.identity(4)
	R[0,0] = np.cos(theta)
	R[0,1] = np.sin(theta) * -1.0
	R[1,0] = np.sin(theta)
	R[1,1] = np.cos(theta)

	return R


def getFK(whichArm, thetas): #theta0, theta1, theta2, ....
	
	#s0
	T1 = np.identity(4)
	T1[0,3] = S0_LENGTHS[0]
	T1[1,3] = S0_LENGTHS[1]
	T1[2,3] = S0_LENGTHS[2]
	Q1 = np.dot(rZ(thetas[0,0]), T1)
	
	#s1
	T2 = np.identity(4)
	Q2 = np.dot(rY(thetas[1,0]), T2)
	
	#e0
	T3 = np.identity(4)
	T3[0,3] = E0_LENGTHS[0]
	T3[1,3] = E0_LENGTHS[1]
	T3[2,3] = E0_LENGTHS[2]
	Q3 = np.dot(rX(thetas[2,0]), T3)
	
	#e1
	T4 = np.identity(4)
	Q4 = np.dot(rY(thetas[3,0]), T4)

	#w0
	T5 = np.identity(4)
	T5[0,3] = W0_LENGTHS[0]
	T5[1,3] = W0_LENGTHS[1]
	T5[2,3] = W0_LENGTHS[2]
	Q5 = np.dot(rZ(thetas[4,0]), T5)

	#w1
	T6 = np.identity(4)
	Q6 = np.dot(rY(thetas[5,0]), T6)
	
	#not using w2
	
	Qend = np.dot(Q1, Q2)
	Qend = np.dot(Qend, Q3)
	Qend = np.dot(Qend, Q4)
	Qend = np.dot(Qend, Q5)
	Qend = np.dot(Qend, Q6)
	
	endEff = np.array([[round(Qend[0,3],3)], [round(Qend[1,3],3)], [round(Qend[2,3],3)]])
	
	return endEff

def getJacobian(whichArm, thetas, deltaTheta):
	Jac = np.zeros((3,6))
	for i in range((np.shape(Jac))[0]): #3
		for j in range((np.shape(Jac))[1]): #6
			#print "********* ", i,j
			newThetas = np.copy(thetas)
			newThetas[j] = thetas[j] + deltaTheta		
			newEndEff = getFK(whichArm, newThetas)
			#print newEndEff
			Jac[i,j] = (newEndEff[i,0] )/ deltaTheta
	return Jac

def getDistance(endEff, goal):
	m = math.sqrt(math.pow(endEff[0] - goal[0],2) + math.pow(endEff[1] - goal[1],2) + math.pow(endEff[2] - goal[2],2))
	return m

def getNext(endEff, goal, eStep, dist):
	dx = (goal[0] - endEff[0]) * eStep / dist
	dy = (goal[1] - endEff[1]) * eStep / dist
	dz = (goal[2] - endEff[2]) * eStep / dist
	
	deltaEndEff = np.array([[round(dx,3)], [round(dy,3)], [round(dz,3)]])
	
	return deltaEndEff

def moveArm(whichArm, thetaInit, goal, deltaTheta, eStep, error):

	if(whichArm == LEFT_ARM):
		strArm = "LEFT  ARM "
	else:
		strArm = "RIGHT ARM "	
	endEff = getFK(whichArm, thetaInit)
	thetas = np.copy(thetaInit)
	dist = getDistance(endEff, goal)
	orig_dist = dist
	print strArm, "Goal    Position : ", goal.transpose()
	print "Current Theta    : ", thetas.transpose()
	print strArm, "Current Position : ", endEff.transpose()
	print strArm, "Error Allowed : ", error, "Distance : ", dist
	counter = 0
	while(dist > error):
		print "\nstrArm, Goal    Position : ", goal.transpose()
		Jac = getJacobian(whichArm, thetas, deltaTheta)
		invJac = np.linalg.pinv(Jac)

		deltaEndEff = getNext(endEff, goal, eStep, dist)#orig_dist)
		print "Increm  Position : ", deltaEndEff.transpose()

		changeTheta = np.dot(invJac, deltaEndEff)
		print "Change Theta     : ", changeTheta.transpose()

		thetas = np.add(thetas, changeTheta)
		print "New Thetas       : ", thetas.transpose()

		endEff = getFK(whichArm, thetas)
		print strArm, "New Position : ", endEff.transpose()	
		
		dist = getDistance(endEff, goal)		
		print strArm, "Error Allowed : ", error, "Distance : ", dist
		counter = counter + 1
		time.sleep(1)
	#print "Counter : ", counter
	return thetas


	

## Open Hubo-Ach feed-forward and feed-back (reference and state) channels
#s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
#r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

## feed-forward will now be refered to as "state"
#state = ha.HUBO_STATE()

## feed-back will now be refered to as "ref"
#ref = ha.HUBO_REF()

## Get the current feed-forward (state) 
#[statuss, framesizes] = s.get(state, wait=False, last=True)
	
#initial Theta values
leftThetaInit = np.zeros((6,1))
rightThetaInit = np.zeros((6,1))

print "Starting ...."

print leftThetaInit
initPos =  getFK(LEFT_ARM, leftThetaInit)
print initPos[0]
print initPos[1]
print initPos[2]

newPos = initPos
newPos[0] = newPos[0] + 10.0

#leftThetaInit[3] = -math.pi/8.0
#print leftThetaInit
#print getFK(LEFT_ARM, leftThetaInit)

#leftThetaInit[3] = -2.0*math.pi/8.0
#print leftThetaInit
#print getFK(LEFT_ARM, leftThetaInit)
#leftThetaInit[3] = -3.0*math.pi/8.0
#print leftThetaInit
#print getFK(LEFT_ARM, leftThetaInit)

#leftThetaInit[3] = -4.0*math.pi/8.0
#print leftThetaInit
#print getFK(LEFT_ARM, leftThetaInit)

#move to TOP LEFT

newLeftThetas  = moveArm(LEFT_ARM ,  leftThetaInit,  newPos, DELTA_THETA, ENDEFF_STEP, ERROR)
print newLeftThetas

#currPos =  getFK(LEFT_ARM, newLeftThetas)
#currPos[0] = currPos[0] + 8.0
#newLeftThetas  = moveArm(LEFT_ARM ,  newLeftThetas,  currPos, DELTA_THETA, ENDEFF_STEP, ERROR)
#print newLeftThetas

#currPos =  getFK(LEFT_ARM, newLeftThetas)
#currPos[0] = currPos[0] + 5.0
#newLeftThetas  = moveArm(LEFT_ARM ,  newLeftThetas,  currPos, DELTA_THETA, ENDEFF_STEP, ERROR)
#print newLeftThetas



