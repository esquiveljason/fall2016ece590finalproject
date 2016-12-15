from ctypes import *

BAXTER_NUM_ARM_JOINTS 	= 7
BAXTER_NUM_ARMS 		= 2

S0 = 0
S1 = 1
E0 = 2
E1 = 3
W0 = 4
W1 = 5
W2 = 6

LEFT_ARM	= 0
RIGHT_ARM	= 1

#lj = left.joint_names()  = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
#rj = right.joint_names() = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

class JOINT(Structure):
	_pack_ = 1
	_fields_ = [("ref", c_double),
			("pos", c_double),
			("torque", c_double)]
			
class ARM(Structure):
	_pack_ = 1
	_fields_ = [("joint", JOINT*BAXTER_NUM_ARM_JOINTS)]
	

class ROBOT(Structure):
	_pack_ = 1
	_fields_ = [("currTime", c_double),
		("arm", ARM * BAXTER_NUM_ARMS)]

