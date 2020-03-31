#!/usr/bin/env python  
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import numpy as np

class ARTagDetection:

    def __init__(self):
        rospy.init_node('listener', anonymous= True)
        sub1= rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.getMarkerPose)
        self.is_detected= 0 
        self.current_ar_pose= np.zeros([3,1])
        self.pose_list= []

    def getMarkerPose(self, msg):
        if(len(msg.markers)== 0):
            self.is_detected= 0
            print ("No AR Tag detected")
        else:
            self.is_detected= 1
            print ("AR Tag detected")
            x= msg.markers[0].pose.pose.position.x
            y= msg.markers[0].pose.pose.position.y
            z= msg.markers[0].pose.pose.position.z
            self.current_ar_pose= np.array([x,y,z])

    def listener(self, num):
        i= 0
        while(i < 4):
            raw_input("Waiting for key-press")
            if(self.is_detected == 1):
                print ('Success. AR Tag pose captured')
                self.pose_list.append(self.current_ar_pose)
                i= i+ 1 
            else:
                print ('Failed. AR Tag pose NOT captured')

        print (" The length of poses array", len(self.pose_list))
        print (self.pose_list)
        pose_array = np.array(self.pose_list)
        np.save("pose_array.npy", pose_array)


if __name__ == '__main__':
    try:
        ar_d= ARTagDetection()
        num= 4     # Number of points to capture
        ar_d.listener(num)
    except rospy.ROSInterruptException:
        pass