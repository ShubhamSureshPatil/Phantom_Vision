#!/usr/bin/env python
"""
Example for commanding joints
"""

import sys
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import cv2
import os
import get_camera_extrinsics as gce


ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'


def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)


def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)

def alvar_pose_listener():
    msg = rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
    try:
        pts_in_alvar_frame_X = msg.markers[0].pose.pose.position.x
        pts_in_alvar_frame_Y = msg.markers[0].pose.pose.position.y
        pts_in_alvar_frame_Z = msg.markers[0].pose.pose.position.z
        rospy.loginfo("AR tag in alvar captured \n")
        return [pts_in_alvar_frame_X,pts_in_alvar_frame_Y,pts_in_alvar_frame_Z]
    except IndexError:
        rospy.loginfo(" AR tag not captured \n")
        return []
        #alvar_pose_listener()

def create_files_for_extrinsics(ALVAR_LIST,AR_tag_positions):
  '''
  ALVAR_LIST is a list of location of the AR tag in the camera frame
  AR_tag_positions is a numpy array of AR tag positions in the world frame
  '''
  count = 0
  for i in range(len(ALVAR_LIST)):
    if(len(ALVAR_LIST[i])!=0):
      count+=1
      if(count==1):
          ALVAR = np.asarray(ALVAR_LIST[i])
          WORLD = AR_tag_positions[i,:]
      else:
          ALVAR = np.vstack((ALVAR,np.asarray(ALVAR_LIST[i])))
          WORLD = np.vstack((WORLD,AR_tag_positions[i,:]))
  #case of one point only not handled as of now.
  print(WORLD.shape)
  print(ALVAR.shape)
  np.save("pts_in_world_frame.npy",WORLD)
  np.save("pts_in_alvar_frame.npy",ALVAR)

def main():
    rospy.init_node('command_joints_example', anonymous=True)


    #******************************************
    #                   PRM
    #target_joints = np.load("execute_path.npy")
    #******************************************

    #******************************************
    #           CAMERA CALIBRATION
    target_joints = np.load("camera_calibration_positions.npy")
    AR_tag_positions = np.load("AR_tag_positions.npy")
    #******************************************

    target_joints = target_joints[:, :-1]
    
    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    rospy.sleep(2)
    home_arm(pub)

    ar_tags_in_alvar_frame = []

    for i,joint in enumerate(target_joints):
      lst = []
      rospy.loginfo("Going to position %s", i)
      set_arm_joint(pub, joint)
      rospy.sleep(5)
      lst = alvar_pose_listener()
      if(len(lst)!=0):
        ar_tags_in_alvar_frame.append(lst)
      else:
        ar_tags_in_alvar_frame.append([])

    print(ar_tags_in_alvar_frame)
    create_files_for_extrinsics(ar_tags_in_alvar_frame,AR_tag_positions)
    K = np.load("./real_sense_intrinsics.npy")
    pts_in_alvar_frame = np.load("./pts_in_alvar_frame.npy")
    pts_in_world_frame = np.load("./pts_in_world_frame.npy")
    H = gce.get_extrinsics(K,pts_in_alvar_frame.T,pts_in_world_frame.T)
    H = np.linalg.inv(H))
    #np.save("H.npy",H)
    home_arm(pub)

if __name__ == "__main__":
    main()