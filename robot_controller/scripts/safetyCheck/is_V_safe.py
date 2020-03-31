import numpy as np
from get_HT2origin_cuboid import getHT2OriginCuboid, getRefHTCuboid
from check_collision import getVertices, getCandidateAxises, hasSeperateAxis
from visualize import visual_check

def VisSafe(cmd_joints):
    HT2origin_obstacle  = np.load("./npyFiles/L2_HT2Origin_obstacles.npy")
    dimension_obstacle  = np.load("./npyFiles/L2_dim_obstacles.npy")
    dimension_cuboid    = np.load("./npyFiles/dim_cuboids.npy") 
    HT2origin_cuboid    = getHT2OriginCuboid(cmd_joints)
    visual_vert = []
    res = True
    for HT_obs, dim_obs in zip(HT2origin_obstacle, dimension_obstacle):
        for HT_cub, dim_cub in zip(HT2origin_cuboid, dimension_cuboid):
            vert_obs = getVertices(HT_obs,dim_obs)
            vert_cub = getVertices(HT_cub,dim_cub)
            visual_vert.append(vert_obs)
            visual_vert.append(vert_cub)
            axises   = getCandidateAxises(HT_obs,HT_cub)
            if ( not hasSeperateAxis(axises,vert_obs,vert_cub) ):
                res = False
    #visual_check(visual_vert)
    return res

def EisSafe(cmd_joints1, cmd_joints2):
    check_point = []
    num_check_point = 10
    for i in range(len(cmd_joints1)):
        check_point.append( np.linspace(cmd_joints1[i],
                                        cmd_joints2[i],
                                        num_check_point,
                                        endpoint=True)  )

    check_point = np.stack(check_point,axis = 1)
    for cpt in check_point:
        if VisSafe(cpt) is not True:
            return False

    return True

def checkTF2orgin_cuboid(cmd_joints):

    dimension_cuboid    = np.load("dim_cuboids.npy") 
    HT2origin_cuboid    = getHT2OriginCuboid(cmd_joints)

    Ref_cub = getRefHTCuboid(0)
    name = ["arm_base", "shoulder", "elbow","forearm", "wrist","gripper"]
    for i in range(6):
        print "\ncuboid :{} ".format(name[i]) 
        print "ref \n", Ref_cub[i]
        print "calc\n",HT2origin_cuboid[i]
    
    vertices_cuboid = []

    for HT_cub, dim_cub in zip(HT2origin_cuboid, dimension_cuboid):
        vert_cub = getVertices(HT_cub, dim_cub)
        vertices_cuboid.append(vert_cub)
    visual_check(vertices_cuboid)



if __name__ == "__main__":
    cmd_joints = np.zeros(6)
    cmd_joints = (0,-0.7853288650512695, -0.2584819793701172, 0.3520388603210449, 0.26180028915405273,  -1.3183925151824951)
    cmd_joints = (0,0.523306131362915, 1.0550556182861328, -1.1344995498657227, 0.7853984832763672, -6.937980651855469e-05)

    for i in range(500):
        VisSafe( getOneSample() )
    # checkTF2orgin_cuboid(cmd_joints)