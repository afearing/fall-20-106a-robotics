#!/usr/bin/env python
"""
Lab 3, task 1
"""


import numpy as np
#import scipy as sp
import kin_func_skeleton as kfs

def lab3(thetas):
    q = np.ndarray((3,8))
    w = np.ndarray((3,7))
    
    q[0:3,0] = [0.0635, 0.2598, 0.1188]
    q[0:3,1] = [0.1106, 0.3116, 0.3885]
    q[0:3,2] = [0.1827, 0.3838, 0.3881]
    q[0:3,3] = [0.3682, 0.5684, 0.3181]
    q[0:3,4] = [0.4417, 0.6420, 0.3177]
    q[0:3,5] = [0.6332, 0.8337, 0.3067]
    q[0:3,6] = [0.7152, 0.9158, 0.3063]
    q[0:3,7] = [0.7957, 0.9965, 0.3058]

    w[0:3,0] = [-0.0059,  0.0113,  0.9999]
    w[0:3,1] = [-0.7077,  0.7065, -0.0122]
    w[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,3] = [-0.7077,  0.7065, -0.0122]
    w[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    w[0:3,5] = [-0.7077,  0.7065, -0.0122]
    w[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                          [-0.7040, 0.7102, -0.0053],
                          [0.7102, 0.7040, 0.0055]]).T
    # YOUR CODE HERE
    twistMats = np.zeros((6,7))
    for i in range(7):
        twistMats[:3, i] = np.cross(-w[:3, i], q[:3, i])
        twistMats[3:, i] = w[:3,i]
    gst_zero = np.eye(4)
    gst_zero[:3,:3] = R
    gst_zero[:3,3] = q[:3,7]
    return np.matmul(kfs.prod_exp(twistMats, thetas), gst_zero)

if __name__ == "__main__":
    print('Lab 3')
    lab3()