#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import pdb
import os, sys

def get_image_points(K,pts_in_alvar_frame):
    '''
    pts in alvar_frame are location of the alvar tag in the camera frame
    shape of image points is 3XN
    '''
    epsilon = 0.0000001
    for i in range(pts_in_alvar_frame.shape[1]):
        if(i==0):
            image_points = np.reshape(np.matmul(K,pts_in_alvar_frame[:,i]),(3,1))
        else:
            image_points = np.hstack((image_points,np.reshape(np.matmul(K,pts_in_alvar_frame[:,i]),(3,1))))
    #image_points = image_points/ (image_points[-1,:] + epsilon)
    for i in range(image_points.shape[1]):
        image_points[:,i] = image_points[:,i]/image_points[-1,i]

    return image_points[:-1,:]

def get_extrinsics(K,pts_in_alvar_frame,pts_in_world_frame):
    #rospy.loginfo("In get_extrinsics")
    # print(pts_in_alvar_frame.shape,"alvar \n") 
    # print(pts_in_world_frame.shape,"world \n")
    image_points = get_image_points(K,pts_in_alvar_frame)
    image_points = image_points.astype(float)
    pts_in_world_frame = pts_in_world_frame.astype(float)
    dist_coef = np.zeros(4)
    _, rvecs, tvecs, inliers = cv2.solvePnPRansac(pts_in_world_frame.T, image_points.T, K, dist_coef,iterationsCount=10000)
    # retval, rvec, tvec= cv2.solvePnP(pts_in_world_frame.T, image_points.T, K, dist_coef)
    print(rvecs)
    print(tvecs)
    rot_mat,jacob = cv2.Rodrigues(rvecs)
    H = np.vstack((np.hstack((rot_mat,tvecs)),np.array([0,0,0,1])))
    return H 

#Ideas to fix: Use np float, use dist_coef as None ,Use ransac

    return H

if __name__ == '__main__':
    rospy.init_node('get_camera_extrinsics', anonymous = True)
    #Use absolute paths if the loading the below files give an error
    K = np.load("real_sense_intrinsics.npy")
    pts_in_alvar_frame = np.load("/home/ashish/ws/robauton_lab2/src/cmu-16662-robot-ctrl/scripts/pts_in_alvar_frame.npy")
    pts_in_world_frame = np.load("/home/ashish/ws/robauton_lab2/src/cmu-16662-robot-ctrl/scripts/pts_in_world_frame.npy")
    H = get_extrinsics(K,pts_in_alvar_frame.T,pts_in_world_frame.T)
    pdb.set_trace()
    rospy.loginfo("Extrinsics obtained")