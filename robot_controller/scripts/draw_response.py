from matplotlib import pyplot as plt
import pickle
import numpy as np
import matplotlib.cm as cm

def plot_feedback():

    with open('pickle_history', 'rb') as file:
        src = pickle.load(file)

    time = np.array(src['timestamp'])
    joints = np.array(src['joint_feedback'])#.reshape(7,-1)
    cmd = np.array(src['ctrl_commands'])
    goal = np.array(src['joint_target'])

    colors = cm.rainbow(np.linspace(0, 1, 3*joints.shape[1]))

    for i in range(joints.shape[1]):
        plt.subplot(2,4,i+1)
        plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9, wspace=0.5, hspace=0.5)
        plt.plot(time, joints[:,i], color=colors[i],label = "joint_feedback")
        #plt.plot(time, goal[:,i], color='r',label = "target")
        plt.plot(time, cmd[:,i],color = 'b',label = "command")
        plt.xlabel('secs')
        unit = 'Radian' if i < (joints.shape[1]-2) else 'Meter'
        title = 'Revlolute Joint '+str(1+i)+' feedback' if i < (joints.shape[1]-2) else\
                'Prismatic Joint '+str(1+i)+' feedback'
        plt.ylabel(unit)
        plt.title(title)
        plt.legend(prop={'size': 6})
        
    plt.show()
    
    for i in range(joints.shape[1]):
        plt.subplot(2,4,i+1)
        plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9, wspace=0.5, hspace=0.5)
        plt.plot(time, joints[:,i], color=colors[i],label = "joint_feedback")
        plt.plot(time, goal[:,i], color='r',label = "target")
        #plt.plot(time, cmd[:,i],color = 'b',label = "command")
        plt.xlabel('secs')
        unit = 'Radian' if i < (joints.shape[1]-2) else 'Meter'
        title = 'Revlolute Joint '+str(1+i)+' feedback' if i < (joints.shape[1]-2) else\
                'Prismatic Joint '+str(1+i)+' feedback'
        plt.ylabel(unit)
        plt.title(title)
        plt.legend(prop={'size': 6})
    plt.show()   

    '''
    plt.title('Revlolute Joint feedback')
    plt.legend(loc='lower left')

    plt.subplot(1,2,2)
    for i in range(5,7):
        plt.plot(time, joints[:,i], color=colors[i],label=str(i))

    plt.xlabel('sec')
    plt.ylabel('Meter')
    plt.title('Prismatic Joint feedback')
    plt.legend(loc='upper right')
    '''
    
    
