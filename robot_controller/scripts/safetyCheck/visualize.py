
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import matplotlib.pyplot as plt


def visual_cuboid(vertices):
    points = vertices.T

    P = np.eye(3)

    Z = np.zeros((8,3))
    for i in range(8): Z[i,:] = np.dot(points[i,:],P)



    # list of sides' polygons of figure
    verts = [[Z[0],Z[1],Z[2],Z[3]],
    [Z[4],Z[5],Z[6],Z[7]], 
    [Z[0],Z[1],Z[5],Z[4]], 
    [Z[2],Z[3],Z[7],Z[6]], 
    [Z[1],Z[2],Z[6],Z[5]],
    [Z[4],Z[7],Z[3],Z[0]]]
    return verts, Z


    

def visual_check(vertices):
    fig = plt.figure()
    r = [-2,2]
    X, Y = np.meshgrid(r, r)
    
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(-0.5, 0.5)
    ax.set_ylim3d(-0.5,0.5)
    ax.set_zlim3d(0,1)
    for vertice in vertices:
        vert, Z = visual_cuboid(vertice)
        ax.scatter3D(Z[:, 0], Z[:, 1], Z[:, 2])
        ax.add_collection3d(Poly3DCollection(vert,facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
'''
def visual_check(vertices1,vertices2):
    fig = pyplot.figure()
    ax = Axes3D(fig)

    sequence_containing_x_vals = vertices1[0]
    sequence_containing_y_vals = vertices1[1]
    sequence_containing_z_vals = vertices1[2]

    sequence2_containing_x_vals = vertices2[0]
    sequence2_containing_y_vals = vertices2[1]
    sequence2_containing_z_vals = vertices2[2]

    ax.scatter(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)
    ax.scatter(sequence2_containing_x_vals, sequence2_containing_y_vals, sequence2_containing_z_vals,c='g')
    pyplot.show()

    print hasSeperateAxis(axises, vertices1, vertices2)
'''
