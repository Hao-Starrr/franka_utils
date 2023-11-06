import numpy as np
from lib.calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """

    J = np.zeros((6, 7))

    fk = FK()
    # the transformation matrix for each joint
    # it is a list, every element in the list is T0n, n from 0 to effector
    Ai = fk.compute_Ai(q_in)

    # the position of the end effector in world coordinates
    o7 = Ai[-1][:3, 3]

    for i in range(7): # from frame 0 to frame 7
        zi = Ai[i][:3, 2]
        oi = Ai[i][:3, 3]

        # linear velocity Jacobian
        J[0:3, i] = np.cross(zi, o7 - oi)

        # angular velocity Jacobian 
        J[3:6, i] = zi 

    return J

if __name__ == '__main__':
    q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    # q = np.array([0,0,0,0,0,0,0])
    print(np.round(calcJacobian(q),3))
