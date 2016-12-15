#!/usr/bin/env python

import ach
import sys
import time
from ctypes import *
from baxterStructure import *

import math
import numpy as np

LEFT_ARM = 0
RIGHT_ARM = 1

#Length of arm components
#						X      Y    Z
S0_LENGTHS = np.array([ 69.00, 0.0, 270.35])
E0_LENGTHS = np.array([364.35, 0.0, -69.00])
W0_LENGTHS = np.array([-10.00, 0.0,-374.29])

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


def getFK(thetas): #theta0, theta1, theta2, ....
	
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


def main():
	print("init node")

	robot = ROBOT()
	s = ach.Channel("baxterState")
	thetas = np.zeros((6,1))
	while True:
		[statuss , frameSizes] = s.get(robot, wait = True , last = True)
		for i in range(BAXTER_NUM_ARM_JOINTS-1):
			thetas[i] = robot.arm[ LEFT_ARM].joint[i].ref
		spatial = getFK(thetas)
		print robot.currTime, spatial[0], spatial[1], spatial[2]
	

if __name__ == '__main__':
	main()

