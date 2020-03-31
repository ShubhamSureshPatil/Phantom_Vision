import numpy as np
from safetyCheck import get_HT2origin_cuboid

def randomSample():
        num_joint = 5
        joint_range_percent = 5 # %
        joint_limit = (0.01*joint_range_percent)*(0.5*np.pi)
        sample = np.random.uniform( low     =   -joint_limit,
                                    high    =   0.9*joint_limit,
                                    size    =   (num_joint,)    )
        res = np.zeros(6)
        res[:5] = sample
        res[3]-= 1.3
        res[2]+=.2
        return res


def getTagPose(joint_angles):
    HT2joints = get_HT2origin_cuboid.getHT2OriginJoint(joint_angles)

    HT2joint4 = HT2joints[-2]

    HT2ARtag = np.array([[1., 0., 0.,  0.025],
                         [0., 1., 0.,  0.   ],
                         [0., 0., 1., -0.04 ],
                         [0., 0., 0.,  1.   ]])

    HT2ARtag = np.dot(HT2joint4, HT2ARtag)

    AR_tag_position = HT2ARtag[0:3,-1].tolist()

    return AR_tag_position


def generatePositions():
    arm_positions = []
    AR_tag_positions = []
    number_of_positions = 100
    print("Generating ",number_of_positions," points \n")

    for i in range(number_of_positions):
        arm_positions.append(randomSample())
        AR_tag_positions.append(getTagPose(arm_positions[-1]))
        
    arm_positions = np.array(arm_positions)

    AR_tag_positions = np.array(AR_tag_positions)
    np.save("AR_tag_positions.npy", AR_tag_positions)
    print(AR_tag_positions)

    np.save("camera_calibration_positions.npy", arm_positions)
    print(arm_positions)


if __name__ == '__main__':
    
    generatePositions()
