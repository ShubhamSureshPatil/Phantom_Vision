import numpy as np
import math
from numpy.linalg import multi_dot
from visualize import visual_check

class Cuboid:
    def __init__(self):
        self.origins = None      # x,y,z
        self.orientations = None # raw, pitch, yaw
        self.dimensions = None   # dx, dy, dz
    def setOrigin(self,point):
        self.origins = (point[0],point[1],point[2])
    def setOrientations(self,orient):
        self.orientations = (orient[0],orient[1],orient[2]) 
    def setDimensions(self,dim):
        self.dimensions = (dim[0],dim[1],dim[2])

def tfBody2Inertia(origin, orientation):
    '''
    input   : 1x3 origin(x,y,z) in inertia frame
            : 1x3 orientation in row, pitch, yaw seq w.r.t inertia frame 
    output  : 4x4 homogeous transformation matrix from body 2 inertia frame
    '''
    H_body_inertia  =np.eye(4)
    (raw, pit, yaw) = orientation
    Rx = np.array([ [   1,     0        ,   0          , 0],
                    [   0, math.cos(raw),-math.sin(raw), 0],
                    [   0, math.sin(raw), math.cos(raw), 0],
                    [   0,     0        ,   0          , 1] ])

    Ry = np.array([ [math.cos(pit) , 0   , math.sin(pit), 0],
                    [   0          , 1   ,    0         , 0],
                    [-math.sin(pit), 0   , math.cos(pit), 0],
                    [   0          , 0   ,    0         , 1] ])


    Rz = np.array([ [math.cos(yaw),-math.sin(yaw), 0, 0],
                    [math.sin(yaw), math.cos(yaw), 0, 0],
                    [   0         ,   0          , 1, 0],
                    [   0         ,   0          , 0, 1] ])
    
    H_body_inertia[0:3,0:3] = multi_dot([Rx,Ry,Rz])[0:3,0:3]
    H_body_inertia[0:3, 3 ] = np.array(origin).T
    return H_body_inertia 

def getVertices(H_body_inertia,dimensions):
    '''
    input   : 1x3 dimensions in body frame
            : 4x4 homogeous transformation matrix from body 2 inertia frame
    output  : 3x8 matrix representing 8 vertices of a cuboid w.r.t inertia frame
    '''
    (dx,dy,dz) = dimensions
    vertices_body_frame = \
        0.5*np.array([[-dx,-dy, dz, 2],[dx,-dy, dz, 2],[dx,dy, dz, 2],[-dx,dy, dz, 2],
                      [-dx,-dy,-dz, 2],[dx,-dy,-dz, 2],[dx,dy,-dz, 2],[-dx,dy,-dz, 2]])

    vertices_inertia_frame = np.dot(H_body_inertia,vertices_body_frame.T) # dim: 4x8
    return vertices_inertia_frame[0:3]

def getCandidateAxises(H_body_inertia1, H_body_inertia2):
    '''
    input   : 1x3 body frames w.r.t inertia frame of cuboid 1 and cuboid 2
    output  : 15x3 matrix representing 15 possible seperate axises w.r.t inertia
              15 = 6 ( 3+3 face normals) + 9 (3x3 edge cross product)
    '''
    pose1 = H_body_inertia1[0:3,0:3].T
    pose2 = H_body_inertia2[0:3,0:3].T
    (ax1, ay1, az1) = (pose1[0], pose1[1], pose1[2])
    (ax2, ay2, az2) = (pose2[0], pose2[1], pose2[2])
    axises = [  ax1, 
                ay1,
                az1,
                ax2,
                ay2,
                az2,
                np.cross(ax1,ax2),
                np.cross(ax1,ay2),
                np.cross(ax1,az2),
                np.cross(ay1,ax2),
                np.cross(ay1,ay2),
                np.cross(ay1,az2),
                np.cross(az1,ax2),
                np.cross(az1,ay2),
                np.cross(az1,az2) ]
    axises = np.around(axises,decimals=3)
    #print axises
    candidate_axises = []
    for axis in axises:
        if(axis.any()):
            candidate_axises.append(axis)
    return np.stack(candidate_axises,axis=0)

def hasSeperateAxis(axises, vertices1, vertices2):
    '''
    input   : nx3 matrix representing n possible seperate axises w.r.t inertia
            : 3x8 matrix representing 8 vertices of a cuboid w.r.t inertia frame
    output  : true or false
    '''
    projection_1 = np.dot(axises, vertices1)
    projection_2 = np.dot(axises, vertices2)
    # return true if projection 1, 2 does not overlap on any of the n possible seperate axises 
    return  ( (projection_1.min(1) > projection_2.max(1)) | ( projection_2.min(1) > projection_1.max(1) ) ).any()

def inCollision(cuboid1_,cuboid2_):
    '''
    input   : cuboid class
    output  : true or false
    '''
    H_body_inertia1 =  tfBody2Inertia(cuboid1_.origins, cuboid1_.orientations)
    H_body_inertia2 =  tfBody2Inertia(cuboid2_.origins, cuboid2_.orientations)
    vertices1 = getVertices(H_body_inertia1, cuboid1_.dimensions)
    vertices2 = getVertices(H_body_inertia2, cuboid2_.dimensions)
    axises = getCandidateAxises(H_body_inertia1,H_body_inertia2)
    visual_check([vertices1,vertices2])
    return not hasSeperateAxis(axises,vertices1,vertices2)
    

if __name__ == "__main__":
    cuboid_ref_ = Cuboid()
    cuboid_ref_.setOrigin(0,0,0)
    cuboid_ref_.setOrientations(0,0,0)
    cuboid_ref_.setDimensions(3,1,2)

    cuboid2_ = Cuboid()
    cuboid2_.setOrigin(-0.8,0,-0.5)
    cuboid2_.setOrientations(0,0,0.2)
    cuboid2_.setDimensions(1,0.5,0.5)

    print inCollision(cuboid_ref_,cuboid2_)

    # [No, No, Yes, Yes, Yes, No, Yes, Yes]
