# Import system libraries
import argparse
import os
import sys
import pickle
from safetyCheck.visualize import visual_check
from draw_response import plot_feedback

# Modify the following lines if you have problems importing the V-REP utilities
cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(os.path.join(cwd,'lib'))
sys.path.append(os.path.join(cwd,'utilities'))


# Import application libraries
import numpy as np
import vrep_utils as vu

# Import any other libraries you might want to use ############################
# ...
###############################################################################

class ArmController:

    def __init__(self):
        # Fill out this method ##################################
        # Define any variables you may need here for feedback control
        self._P_gains = np.array([4.0,6.0,6.0,3.0,3.0,3.0,3.0])
        self._I_gains = np.array([0.0,0.001,0.01,0.0,0.0,0.0,0.0])
        self._D_gains = np.array([0.01,0.01,0.01,0.01,0.01,0.01,0.01])
        self._err = np.zeros(7)
        self._prev_err = np.zeros(7)
        self._diff_err = np.zeros(7)
        self._sum_err = np.zeros(7)
        self._upper_bound = np.pi/2.0
        #########################################################
        # Do not modify the following variables
        self.history = {'timestamp': [],
                        'joint_feedback': [],
                        'joint_target': [],
                        'ctrl_commands': []}
        self._target_joint_positions = None

    def set_target_joint_positions(self, target_joint_positions):
        assert len(target_joint_positions) == vu.N_ARM_JOINTS, \
            'Expected target joint positions to be length {}, but it was length {} instead.'.format(len(target_joint_positions), vu.N_ARM_JOINTS)
        self._target_joint_positions = target_joint_positions

    def calculate_commands_from_feedback(self, timestamp, sensed_joint_positions):
        assert self._target_joint_positions, \
            'Expected target joint positions to be set, but it was not.'

        # Fill out this method ##################################
        # Using the input joint feedback, and the known target joint positions,
        # calculate the joint commands necessary to drive the system towards
        # the target joint positions.
        self._err = np.array(self._target_joint_positions) - np.array(sensed_joint_positions)
        self._diff_err= (self._prev_err - self._err)/0.05
        self._prev_err = self._err
        self._sum_err += self._err
        ctrl_commands = np.zeros(vu.N_ARM_JOINTS)
        ctrl_commands = self._P_gains*self._err + \
                        self._D_gains*self._diff_err + \
                        self._I_gains*self._sum_err
        ctrl_commands[ctrl_commands>self._upper_bound] = self._upper_bound
        # ...
        #########################################################

        # Do not modify the following variables
        # append time history
        self.history['timestamp'].append(timestamp)
        self.history['joint_feedback'].append(sensed_joint_positions)
        self.history['joint_target'].append(self._target_joint_positions)
        self.history['ctrl_commands'].append(ctrl_commands)
        return ctrl_commands

    def has_stably_converged_to_target(self):
        # Fill out this method ##################################
        #has_stably_converged_to_target = False
        diff = np.array(self.history['joint_target']) - np.array(self.history['joint_feedback'])
        return (abs(diff[-10::]) < 0.03).all()


def execute():
    # Connect to V-REP
    print ('Connecting to V-REP...')
    clientID = vu.connect_to_vrep()
    print ('Connected.')

    # Reset simulation in case something was running
    vu.reset_sim(clientID)
    
    # Initial control inputs are zero
    vu.set_arm_joint_target_velocities(clientID, np.zeros(vu.N_ARM_JOINTS))

    # Despite the name, this sets the maximum allowable joint force
    vu.set_arm_joint_forces(clientID, 50.*np.ones(vu.N_ARM_JOINTS))

    # One step to process the above settings
    vu.step_sim(clientID)

    deg_to_rad = np.pi/180.

    # Joint targets. Specify in radians for revolute joints and meters for prismatic joints.
    # The order of the targets are as follows:
    #   joint_1 / revolute  / arm_base_link <- shoulder_link
    #   joint_2 / revolute  / shoulder_link <- elbow_link
    #   joint_3 / revolute  / elbow_link    <- forearm_link
    #   joint_4 / revolute  / forearm_link  <- wrist_link
    #   joint_5 / revolute  / wrist_link    <- gripper_link
    #   joint_6 / prismatic / gripper_link  <- finger_r
    #   joint_7 / prismatic / gripper_link  <- finger_l
 
    planned = np.load("execute_path.npy")
    res = []
    for plan in planned:
        res.append( plan.tolist() )
    joint_targets = res

    # Instantiate controller
    controller = ArmController()

    # Iterate through target joint positions
    for i,target in enumerate(joint_targets):

        # Set new target position
        controller.set_target_joint_positions(target)

        steady_state_reached = False
        while not steady_state_reached:

            timestamp = vu.get_sim_time_seconds(clientID)
            #print('Simulation time: {} sec'.format(timestamp))

            # Get current joint positions
            sensed_joint_positions = vu.get_arm_joint_positions(clientID)

            # Calculate commands
            commands = controller.calculate_commands_from_feedback(timestamp, sensed_joint_positions)

            # Send commands to V-REP
            vu.set_arm_joint_target_velocities(clientID, commands)

            # Print current joint positions (comment out if you'd like)
            #print(sensed_joint_positions)
            vu.step_sim(clientID, 1)

            # Determine if we've met the condition to move on to the next point
            steady_state_reached = controller.has_stably_converged_to_target()

        #visualizer(clientID)
        #saveingCuboid(clientID,i)
        #savingJoints(clientID,i)
    vu.stop_sim(clientID)
    print "terminated"

    file = open('pickle_history', 'wb')
    pickle.dump(controller.history, file)
    file.close()
    #plot_feedback()
    #####################################################################################

def visualizer(clientID):

    cuboids = []
    for i in range(8):
        cuboids.append(Cuboid())

    vertices = []
    cuboid_handles = vu.get_cuboid_handles(clientID)

    for idx, c in enumerate(cuboid_handles):
        if idx < 6:
            cuboids[idx].setOrigin(np.array(vu.get_object_position(clientID,c)))
            cuboids[idx].setOrientations(np.array(vu.get_object_orientation(clientID,c)))
            (pmin, pmax) = vu.get_object_bounding_box(clientID,c)
            cuboids[idx].setDimensions(np.subtract(pmax,pmin))
            H_body_inertia =  tfBody2Inertia(cuboids[idx].origins, cuboids[idx].orientations)
            vertices.append(getVertices(H_body_inertia, cuboids[idx].dimensions))

    visual_check(np.array(vertices))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    execute()
