import numpy as np
from math import pi

class FK():

    @staticmethod
    def rotate(axis, angle):
        # the angle is in radius
        if axis == 'X':
            return np.array([
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ])
        elif axis == 'Y':
            return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1]
            ])
        elif axis == 'Z':
            return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
        else:
            raise ValueError("Invalid axis. Use 'X', 'Y', or 'Z'.")

    @staticmethod
    def translate(x,y,z):
            return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1]
            ])

    
    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        # the DH convention is Craig’s convention, which is the same with official documents 
        self.a = np.array([0.0, 0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0])
        self.d = np.array([0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.0, 0.21])
        self.alpha = np.array([0.0, -np.pi / 2.0, np.pi / 2.0, np.pi / 2.0, -np.pi / 2.0, np.pi / 2.0, np.pi / 2.0, 0.0])

        self.compensate = np.array([[0, 0, -0.192], # the joint 1 center is at this coordinate in frame 1 
                                   [0, 0, 0],
                                   [0, 0, -0.121],
                                   [0, 0, 0],
                                   [0, 0, -0.259],
                                   [0, 0, 0],
                                   [0, 0, 0.051],
                                   [0, 0, 0]])


    def modified_dh_transform_matrix(self, a, d, alpha, theta):
        # modified DH convention
        matrix = np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -np.sin(alpha) * d],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), np.cos(alpha) * d],
        [0, 0, 0, 1]
        ])
        return matrix
    
    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your Lab 1 code starts here

        # The frame build as official documents, in Craig’s convention
        # So the frame origins are not at joints centers
        # We need calculate the position for every joints
        # So we need to compensate the distant for every frame

        # initialize
        jointPositions = np.zeros((8,3))
        Ai = [] # a list, every element is 4x4 numpy array
        T = np.identity(4)
        theta = np.append(q, 0.0)

        for i in range(0,8):
            # calculate A and append A at list
            A = self.modified_dh_transform_matrix(self.a[i],self.d[i],self.alpha[i],theta[i])
            Ai.append(A)
            # multiply A to T
            T = T @ A
            # compensate for joint centers
            x, y, z = self.compensate[i] # the joint center frames in DH frames, T1center1
            jointPositions[i] = (T @  self.translate(x, y, z))[:3,3] # T0center1 position
            
        T0e = T

        # Your code ends here

        return jointPositions, T0e

    # feel free to define additional helper methods to modularize your solution for lab 1

    
    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2
        Ai = [] # a list, every element is 4x4 numpy array
        theta = np.append(q, 0.0)

        for i in range(0,8):
            # calculate A and append A at list
            A = self.modified_dh_transform_matrix(self.a[i],self.d[i],self.alpha[i],theta[i])
            Ai.append(A)


        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])
    q = np.array([0,0,0,0,0,0,0])


    joint_positions, T0e = fk.forward(q)
    
    print("Joint Positions:\n",joint_positions[7])
    # print("End Effector Pose:\n",T0e)
    print("end eff position: ", T0e[:3, 3])
