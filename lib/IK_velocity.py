import numpy as np 
from lib.calcJacobian import calcJacobian



def IK_velocity(q_in, v_in, omega_in):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param v_in: The desired linear velocity in the world frame. If any element is
    Nan, then that velocity can be anything
    :param omega_in: The desired angular velocity in the world frame. If any
    element is Nan, then that velocity is unconstrained i.e. it can be anything
    :return:
    dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
        are infeasible, then dq should minimize the least squares error. If v_in
        and omega_in have multiple solutions, then you should select the solution
        that minimizes the l2 norm of dq
        """

    ## STUDENT CODE GOES HERE

    dq = np.zeros((1, 7))

    v_in = v_in.reshape((3,1))
    omega_in = omega_in.reshape((3,1))
    V = np.vstack((v_in, omega_in))

    J = calcJacobian(q_in)
    
    # we should not just assign nan as 0, because nan means no constrains
    # we can just ignore these rows
    constrained_rows = ~np.isnan(V)
    J_constrained = J[constrained_rows.reshape(-1)]
    V_constrained = V[constrained_rows]

    
    J_pseudo_inverse = np.linalg.pinv(J_constrained) 
    # pseudo inverse can auto satisfy the least square requirement
    # https://numpy.org/doc/stable/reference/generated/numpy.linalg.pinv.html

    dq = np.dot(J_pseudo_inverse, V_constrained)
    
    return dq

if __name__ == '__main__':
    q_test = np.array([0, 0, 0, 0, 0, 0, 0])
    v_test = np.array([1, 1, 1])
    omega_test = np.array([np.nan, np.nan, np.nan])
    print(IK_velocity(q_test, v_test, omega_test))
