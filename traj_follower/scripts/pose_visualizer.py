#!/usr/bin/env python  
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class PoseVisualizer:

    def __init__(self):
        rospy.init_node('visualizer', anonymous= True)
        self.camera_ar_pub = rospy.Publisher('camera_ar_pose', Marker, queue_size= 100)
        self.camera_ar_pose_array= np.load('/home/aadityacr7/Desktop/phantom_vision_ws/src/npy files/trial1.npy')
        rospy.sleep(1)

    def visualizer(self):
        rate = rospy.Rate(5)
        camera_pos_list= self.camera_ar_pose_array.tolist()
        print (camera_pos_list)
        XYZPoints = []
        #transform from x,y points to x,y,z points
        for l in camera_pos_list:
            p = Point() 
            p.x = l[0]
            p.y = l[1]
            p.z = l[2]
            XYZPoints.append(p)
        while not rospy.is_shutdown():
            print ('Publishing message')
            marker = Marker()
            marker.header.frame_id = "/camera_link"
            marker.type = marker.POINTS
            #marker.action = marker.ADD
            marker.pose.orientation.w = 1
            marker.points = XYZPoints
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0
            self.camera_ar_pub.publish(marker)
            rate.sleep()


if __name__ == '__main__':
    try:
        camera_ar= PoseVisualizer()
        camera_ar.visualizer()
    except rospy.ROSInterruptException:
        pass




