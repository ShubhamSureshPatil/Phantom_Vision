from graph import Vertice, Graph
from safetyCheck.is_V_safe import VisSafe, EisSafe
from locobot_joint_ctrl import execute
import numpy as np
import random

def totuple(a):
    try:
        return tuple(totuple(i) for i in a)
    except TypeError:
        return a

if __name__ == '__main__':
    graph = Graph()
    
    for iter in range(500):
        query = graph.randomSample()
        if VisSafe( query  ):
            graph.vertices_graph.append( Vertice(query) )
            graph.connectKnearestVertices( Vertice(query) )
        print(iter)
    print("Graph Connected")

    #src     = Vertice(( 0, -1.3962,       0,      0,      0, 0))
    #goal    = Vertice(( 0, 0      ,   1.047, -1.039, -1.039, 0))

    rad_from_deg = np.pi/180.
    start_joints_deg = np.array([0.,   0., 20., 50., -80., 0.])
    final_joints_deg = np.array([0., 135., 20., 50., -80., 0.])

    start_joints_rad = start_joints_deg * rad_from_deg
    final_joints_rad = final_joints_deg * rad_from_deg

    src = Vertice(totuple(start_joints_rad))
    goal = Vertice(totuple(final_joints_rad))



    graph.access(src)
    graph.depart(goal)
    path = graph.DFS(src, goal)
    np.save("execute_path.npy", path)  
    
    execute()
