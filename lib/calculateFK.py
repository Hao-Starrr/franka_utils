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
        self.a = np.array([0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0])
        self.d = np.array([0.333, 0.0, 0.316, 0.0, 0.384, 0.0, 0.210])
        self.alpha = np.array([-pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0.0])

        self.compensate = np.array([[0, 0, 0.141], # the joint 1 center is at this coordinate in frame 0
                                   [0, 0, 0],
                                   [0, 0, 0.195], # the joint 3 center is at this coordinate in frame 2
                                   [0, 0, 0],
                                   [0, 0, 0.125], # the joint 5 center is at this coordinate in frame 4
                                   [0, 0, -0.015], # the joint 6 center is at this coordinate in frame 5
                                   [0, 0, 0.051], # the joint 7 center is at this coordinate in frame 6
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

    def classical_dh_transform_matrix(self, a, d, alpha, theta):
        matrix = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
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

        jointPositions = np.zeros((8,3))
        theta = q
        theta[6] -= pi/4

        T = np.identity(4)
        x,y,z = self.compensate[0]
        jointPositions[0] = (T @  self.translate(x,y,z))[:3,3] # T0center1 position

        for i in range(0,7): 
            Ai = self.classical_dh_transform_matrix(self.a[i], self.d[i], self.alpha[i], theta[i])
            T = T @ Ai
            # print(T)
            x, y, z = self.compensate[i+1] # the joint center frames in DH frames, T1center1
            jointPositions[i+1] = (T @  self.translate(x, y, z))[:3,3] # T0center2 position
            
        T0e = T

        return jointPositions, T0e

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
        theta = q
        theta[6] -= pi/4

        T = np.identity(4)
        Ai.append(T) # Ai[0] = T00
        for i in range(0,7): 
            A = self.classical_dh_transform_matrix(self.a[i], self.d[i], self.alpha[i], theta[i])
            T = T @ A
            Ai.append(T) # Ai[1] = T01, Ai[2] = T02, ...,Ai[7] = T07
        
        return Ai
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.zeros((5,7))
    q[0] = np.array([0, 0, 0, -0.1, 0, 0, 0])
    q[1] = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    q[2] = np.array([np.pi/2, np.pi/4, -np.pi/4, -np.pi/2, 0, np.pi/2, np.pi/4])
    q[3] = np.array([0, 0, 0, -np.pi/4, np.pi/2, np.pi/2, np.pi/4])
    q[4] = np.array([0, 0, 0, -np.pi/4, np.pi/2, np.pi, 2.8])
    for i in range(5):
        joint_positions, T0e = fk.forward(q[i])
        print(np.round(T0e[:3,3],3))
    
    # print("Joint Positions:\n",joint_positions)
    # print("End Effector Pose:\n",T0e)


