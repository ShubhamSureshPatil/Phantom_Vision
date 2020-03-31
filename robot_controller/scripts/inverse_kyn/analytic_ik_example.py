"""
Example for using Analytic IK on 5dof locobot
"""

import sys
import time

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from analytic_ik import AnalyticInverseKinematics

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'
ROSTOPIC_OPEN_GRIPPER = '/gripper/open'
ROSTOPIC_CLOSE_GRIPPER = '/gripper/close'

def open_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Opening gripper')
    pub.publish(empty_msg)

def close_gripper(pub):
    empty_msg = Empty()
    rospy.loginfo('Closing gripper')
    pub.publish(empty_msg)

def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)

def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)

def main():
    rospy.init_node('analytic_ik_example', anonymous=True)

    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    gripper_open_pub = rospy.Publisher(ROSTOPIC_OPEN_GRIPPER, Empty, queue_size=1)
    gripper_close_pub = rospy.Publisher(ROSTOPIC_CLOSE_GRIPPER, Empty, queue_size=1)
    ik_solver = AnalyticInverseKinematics()
    rospy.sleep(2)
    home_arm(pub)

    # orignally was 3
    n_samples_x = 1
    n_samples_y = 1


    # These seem to be the end point locations
    # min_x = 0.20
    # max_x = 0.30
    # min_y = -0.15
    # max_y = 0.15
    # fixed_z = 0.20

    # The new coordinates
    min_x = 0.30
    max_x = 0.30

    # miny is the direction along left-right with right as positive
    min_y = 0.0
    max_y = 0.15

    # fixed_z is the height 
    fixed_z = 0.05



    # z is the height 
    x = np.linspace(min_x, max_x, n_samples_x)
    y = np.linspace(min_y, max_y, n_samples_y)
    xv, yv = np.meshgrid(x, y)
    x = xv.flatten()
    y = yv.flatten()
    z = fixed_z * np.ones(len(x))
    target_position = np.array([x,y,z]).T
    print("The current target position is:")    
    print(target_position)

    for i in range(len(x)):

        angle = -np.pi/2
        # alpha = -np.pi/2 was the original
        target_joint = ik_solver.ik(target_position[i,0], target_position[i,1], target_position[i,2], alpha=angle)
        print(target_joint)

        if target_joint is not None:
            #print("Inside the loop")
            #print("The joint targets are")
            target_joint.append(0.0)
            #print(target_joint)
            set_arm_joint(pub, target_joint)
            time.sleep(4)

        else:
            print('No IK Solution found for '+ str(target_position[i,:]))

    raw_input("Robot ready to close gripper. Press Enter to continue.")
    print("Robot moving. Please wait.")
    close_gripper(gripper_close_pub)
    rospy.sleep(4)

    # raw_input("Robot ready to open gripper. Press Enter to continue.")
    # print("Robot moving. Please wait.")
    # open_gripper(gripper_open_pub)
    # rospy.sleep(4)

    home_arm(pub)


if __name__ == "__main__":
    main()
