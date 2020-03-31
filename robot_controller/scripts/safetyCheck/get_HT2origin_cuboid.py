import numpy as np
import math
from rotation import Rx, Ry, Rz
from numpy.linalg import multi_dot
from numpy.linalg import inv

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})

def getForwardRotJoint():
    joint_orient = np.load("./npyFiles/joint_orient0.npy")
    prev = np.eye(4)
    forward_rot = []
    for idx in range(6):
        (tx,ty,tz) = joint_orient[idx]
        current = multi_dot([Rx(tx),Ry(ty),Rz(tz)])
        forward_rot.append( np.dot( inv(prev), current ) )
        prev = current
    forward_rot = np.stack(forward_rot, axis = 0)
    return forward_rot

def getForwardTransJoint():
    joint_pos = np.load("./npyFiles/joint_pos0.npy")
    forward_trans = []
    prev = np.zeros(3)
    for jpos in joint_pos:
        forward_trans.append(jpos - prev)
        prev = jpos
    forward_trans = np.stack(forward_trans, axis = 0)
    return forward_trans

def getJointPosition(idx, cmd_joints):
	if idx is 0: # P1_0 
		return ( 		-0.0885	,
						 0.0004 ,
						 0.159		)
	elif idx is 1: # P2_1
		return ( 			0	,
							0	,
							0.0412  )
	elif idx is 2: #P3_2
		return ( 0.206*math.sin(0.2618+cmd_joints[idx]),
		 		-0.206*math.cos(0.2618+cmd_joints[idx]), 
				 			0							)
	elif idx is 3: #P4_3
		return ( 0.202*math.cos(cmd_joints[idx]),
		 		 0.202*math.sin(cmd_joints[idx]), 
				 			0	 					)
	elif idx is 4:
		return ( 0.063*math.cos(cmd_joints[idx]),
		 		 0.063*math.sin(cmd_joints[idx]), 
				 			0	 					)
	elif idx is 5:
		return ( 0.025*math.cos(-1.571+cmd_joints[idx]),
		 		 0.025*math.sin(-1.571+cmd_joints[idx]), 
				 			-0.0728	 						)

def getHT2OriginJoint(cmd_joints):
    forward_rot_joint   = getForwardRotJoint()
    forward_trans_joint = getForwardTransJoint()
    res = None
    for idx in range(6):
        HT = multi_dot( [ Rz( cmd_joints[idx] ), forward_rot_joint[idx] ])
        HT[0:3, 3] = getJointPosition(idx, cmd_joints)
        if res is None:
            res = [HT]
        else:
            res.append(np.dot(res[-1],HT))

    return np.stack(res, axis = 0)

def getRefHTCuboid(seq):
    cuboid_orient = np.load("./npyFiles/cuboid_orient"+str(seq)+".npy")
    cuboid_pos = np.load("./npyFiles/cuboid_pos"+str(seq)+".npy")
    HT2Origin = []

    for idx in range(6):
        (tx,ty,tz) = cuboid_orient[idx]
        HT = np.eye(4)
        HT[0:3, 0:3] = multi_dot([Rx(tx),Ry(ty),Rz(tz)])[0:3, 0:3]
        HT[0:3,  3 ] = cuboid_pos[idx].T
        HT2Origin.append(HT)
    return np.stack(HT2Origin, axis = 0)

def getHT_Cuboid_to_Joint():
    ''' HT_joint_to_orgin: (6x4x4), first element is indentity '''
    refHT_cuboid       = getRefHTCuboid(0) # 6x4x4
    HT_joint_to_origin = getHT2OriginJoint(np.zeros(6))
    res = []
    for idx in range(6):
        HT = np.dot( inv(HT_joint_to_origin[idx]),refHT_cuboid[idx]) 
        res.append(HT)
    return np.stack(res, axis = 0)

def getHT2OriginCuboid(cmd_joints):
    HT_Cuboid_to_Joint  = getHT_Cuboid_to_Joint()
    HT_joint_to_origin  = getHT2OriginJoint(cmd_joints) #6x4x4
    res = []
   
    for i in range(6):  
        res.append( np.dot(HT_joint_to_origin[i], HT_Cuboid_to_Joint[i] ) )
    
    return np.stack( res, axis = 0)


'''
2.5cm in x
  0cm in y
 -4cm in z
from 5th joint (joint 4)
'''