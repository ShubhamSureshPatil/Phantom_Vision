"""
Demo runner for the project
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

def main(points_list):
    rospy.init_node('analytic_ik_example', anonymous=True)

    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    gripper_open_pub = rospy.Publisher(ROSTOPIC_OPEN_GRIPPER, Empty, queue_size=1)
    gripper_close_pub = rospy.Publisher(ROSTOPIC_CLOSE_GRIPPER, Empty, queue_size=1)
    ik_solver = AnalyticInverseKinematics()
    rospy.sleep(2)

    # Start from the home position
    home_arm(pub)

    # orignally was 3, do not change
    n_samples_x = 1
    n_samples_y = 1

    """
    The new coordinates
    Append the new cordiate you want the robot to go the point array below
    in the format [x,y,z]
    """
    #point = np.array([[0.3, 0.2, 0.1], [0.3, -0.2, 0.1], [0.3, -0.2, 0.3], [0.25, -0.1, 0.4]])
    point= np.array(points_list)
    # Loading points from the numpy file instead
    #point = np.load("../../../npy files/final_points.npy")
    print(point)

    """
    Z is the height
    X+ is the depth, always positive
    Y+ is to the left of the robot arm 
    """

    for i in range(len(point)):
        min_x = point[i,0]
        max_x = point[i,0]

        # miny is the direction along left-right with right as positive
        min_y = point[i,1]
        max_y = point[i,1]

        # fixed_z is the height 
        fixed_z = point[i,2]

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

            # alpha = -np.pi/2 was the original
            # no longer passing the angle of the write rotation!
            target_joint = ik_solver.ik(target_position[i,0], target_position[i,1], target_position[i,2])

            if target_joint is not None:
                target_joint.append(0.0)
                set_arm_joint(pub, target_joint)
                time.sleep(0.5)

            else:
                print('No IK Solution found for '+ str(target_position[i,:]))

    # Below two blocks of code control the closing and opening of the manipulator in case needed

    # raw_input("Robot ready to close gripper. Press Enter to continue.")
    # print("Robot moving. Please wait.")
    # close_gripper(gripper_close_pub)
    # rospy.sleep(4)

    # raw_input("Robot ready to open gripper. Press Enter to continue.")
    # print("Robot moving. Please wait.")
    # open_gripper(gripper_open_pub)
    # rospy.sleep(4)


    home_arm(pub)           # Return back to the home position, can be commented


if __name__ == "__main__":
    main()
