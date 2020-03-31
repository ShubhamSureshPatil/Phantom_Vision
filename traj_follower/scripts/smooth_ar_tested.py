#!/usr/bin/env python  
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker
import numpy as np
import time 
import sys, select, os
from polyfit_traj import getTransformedPoints
import tf2_ros
from tf.transformations import euler_matrix, euler_from_quaternion, \
    quaternion_from_euler, translation_matrix, quaternion_matrix, \
    quaternion_from_matrix
from scipy import interpolate

class SmoothAR:

    def __init__(self):
        rospy.init_node('smooth_ar', anonymous= True)
        self.sub1= rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.get_marker_pose_cb)
        self.camera_ar_pub = rospy.Publisher('camera_ar_pose', Marker, queue_size= 100)
        self.world_ar_pub = rospy.Publisher('world_ar_pose', Marker, queue_size= 100)
        self.traj_ar_pub = rospy.Publisher('traj_ar_pose', Marker, queue_size= 100)

        #self.camera_ar_pose_array= np.load('/home/aadityacr7/Desktop/phantom_vision_ws/src/npy files/trial1.npy')
        #self.camera_ar_pose_array= []
        self.is_detected= 0 
        self.current_ar_pose= np.zeros([3,1])
        self.camera_pose_list= []
        self.world_pose_list= []
        self.flag= 0
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.traj_points= []
        rospy.sleep(1)

    def get_marker_pose_cb(self, msg):
        if(len(msg.markers)== 0):
            self.is_detected= 0
            print ("No AR Tag detected")
        else:
            self.is_detected= 1
            x= msg.markers[0].pose.pose.position.x
            y= msg.markers[0].pose.pose.position.y
            z= msg.markers[0].pose.pose.position.z
            self.current_ar_pose= np.array([x,y,z])

    def visualize_pose_camera(self):
        camera_pos_list= self.camera_pose_list
        #print (camera_pos_list)
        XYZPoints = []
        #transform from x,y points to x,y,z points
        for l in camera_pos_list:
            p = Point() 
            p.x = l[0]
            p.y = l[1]
            p.z = l[2]
            XYZPoints.append(p)
        print ('Publishing message')
        marker = Marker()
        marker.header.frame_id = "/camera_color_optical_frame"
        marker.type = marker.POINTS
        #marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.points = XYZPoints
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.camera_ar_pub.publish(marker)

    def visualize_pose_world(self):
        world_pos_list= self.world_pose_list
        XYZPoints = []
        #transform from x,y points to x,y,z points
        for l in world_pos_list:
            p = Point() 
            p.x = l[0]
            p.y = l[1]
            p.z = l[2]
            XYZPoints.append(p)
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.POINTS
        #marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.points = XYZPoints
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.g = 1.0
        self.world_ar_pub.publish(marker)

    def visualize_traj(self):
        traj_list= self.traj_points
        XYZPoints = []
        #transform from x,y points to x,y,z points
        for l in traj_list:
            p = Point() 
            p.x = l[0]
            p.y = l[1]
            p.z = l[2]
            XYZPoints.append(p)
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.type = marker.POINTS
        #marker.action = marker.ADD
        marker.pose.orientation.w = 1
        marker.points = XYZPoints
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.b = 1.0
        self.traj_ar_pub.publish(marker)

    def ar_track(self, sample_time):
        # Sample every n seconds 
        if(self.is_detected == 1):
            print ('Success. AR Tag pose captured')
            self.camera_pose_list.append(self.current_ar_pose)
        if(self.is_detected == 0):
            print ('Failed. AR Tag pose NOT captured')
        else:
            self.visualize_pose_camera()
            time.sleep(sample_time)
    
    '''
    def transform_world(self):
        self.world_pose_list= getTransformedPoints(np.array(self.camera_pose_list))
    '''
    def transform_world(self):
        try:
            quat_trans = self.tfBuffer.lookup_transform("base_link", "camera_link", rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print ("TF Error")

        t_cb = quat_trans.transform.translation
        q_cb = quat_trans.transform.rotation
        H_cam_in_base = translation_matrix((t_cb.x, t_cb.y, t_cb.z)).dot(quaternion_matrix((q_cb.x, q_cb.y, q_cb.z, q_cb.w)))
        #print ("Printing Old Homo trans")
        #print (H_cam_in_base)
        cam_base_transform = np.array([[ 0.,  -0.73727734,   0.67559021,       0.04905 ],
                                       [-1.,          0.,           0.,        0.016         ],
                                       [ 0., -0.67559021,   -0.73727734,        0.53201   ],
                                       [ 0.,          0.,          0.,         1.]       ])
        '''
        cam_base_transform = np.array([[ 0.11393721, -0.74960065,  0.65201011, 0.04905],
	                               [-0.99337023, -0.07585431,  0.08638116,  0.03243686],
	                               [-0.0152936,  -0.65752946, -0.7532736,   0.53201],
	                               [ 0.,          0.,          0.,          1.        			]])     
        '''                          
        camera_ar_pose_array= np.array(self.camera_pose_list)
        camera_ar_pose_array = np.concatenate((camera_ar_pose_array, np.ones((camera_ar_pose_array.shape[0],1))), axis=1)
        #print ("Printing pre-transform")
        #print (camera_ar_pose_array)
        points_world= np.dot(camera_ar_pose_array, cam_base_transform.T)[:,:-1]
        points_world[:,0]-= 0.16   # Adding the x offset 
        #print ("Printing post-transform")
        #print (points_world)
        self.world_pose_list= list(points_world) 

    

    def getPolyTrajectory(self):
        test_points= np.array(self.world_pose_list)
        num_intermediate_points = 20
        x_sample = test_points[:,0]
        y_sample = test_points[:,1]
        z_sample = test_points[:,2]
        tck, u = interpolate.splprep([x_sample,y_sample,z_sample], k= 7)
        x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
        u_fine = np.linspace(0,1,num_intermediate_points)
        x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
        self.traj_points = list(np.concatenate(([x_fine], [y_fine], [z_fine]), axis=0).T)
        #print("Polyfit Trajectory:")
        #print(final_points)

        #np.save("../../../npy files/final_points.npy", final_points)


if __name__ == '__main__':
    smooth_ar_tracker = SmoothAR()
    rate = rospy.Rate(5)
    try:
        while not rospy.is_shutdown():
            '''
            if(smooth_ar_tracker.flag == 0):   # If we are in AR tag data collection mode 
                while True:
                    os.system('cls' if os.name == 'nt' else 'clear')
                    smooth_ar_tracker.ar_track(0.3)
                    rate.sleep()                
                    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                        line = raw_input()
                        print ('Stopping tracking')
                        smooth_ar_tracker.flag= 1
                        break
            arr_temp= np.array(smooth_ar_tracker.camera_pose_list)
            np.save('/home/ashish/Desktop/phantom_vision_ws/src/phantom_vision_project/traj_follower/scripts/bag1.npy', arr_temp)
            '''
            temp= np.load('/home/ashish/Desktop/phantom_vision_ws/src/phantom_vision_project/traj_follower/scripts/bag1.npy')
            #print (temp)
            smooth_ar_tracker.camera_pose_list= list(temp)
            smooth_ar_tracker.transform_world()
            smooth_ar_tracker.getPolyTrajectory()
            smooth_ar_tracker.visualize_pose_world()
            smooth_ar_tracker.visualize_pose_camera()
            smooth_ar_tracker.visualize_traj()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass



