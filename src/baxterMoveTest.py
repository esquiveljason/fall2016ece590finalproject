#!/usr/bin/env python
import ach
from baxterStructure import *
import time
import math

def main():
	print("init node")

	baxterDataRef = ROBOT()
	r = ach.Channel("baxterRef")
	#s = ach.Channel("baxterState")

	for i in range(BAXTER_NUM_ARM_JOINTS):
		baxterDataRef.arm[LEFT_ARM].joint[i].ref = 0.0
		baxterDataRef.arm[RIGHT_ARM].joint[i].ref = 0.0
	
	baxterDataRef.currTime = time.time()
	r.put(baxterDataRef)
	print "Sending new data at {}".format(baxterDataRef.currTime)
	
	while True:
		a = int(raw_input("0) LeftArm \n1) RightArm\n>>"))
		j = int(raw_input("0) S0\n1) S1\n2) E0\n3) E1\n4) W0\n5) W1\n6) W2\n>>"))
		v = float(raw_input("Enter position in degrees >>"))

		baxterDataRef.arm[a].joint[j].ref = math.radians(v)
		print baxterDataRef.arm[a].joint[j].ref
		baxterDataRef.currTime = time.time()
		r.put(baxterDataRef)
		print "Sending new data at {}".format(baxterDataRef.currTime)
		
		
		
		time.sleep(1)
	

if __name__ == '__main__':
	main()
