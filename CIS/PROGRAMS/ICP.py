import numpy as np
from Registration import registrationArunMethod
from ClosestPointOnTriangle import findClosestPointOnTriangle
from Mesh import search
from testing import calcDistance

def ICP(d_k, root, d_max, max_iterations, F_reg):
    '''
    Parameters: 
    d_k: N_samples x 3 (N_sample points)
        position of pointer tip with respect to rigid body B
    root: Node object
        root of KD tree
    d_max: int
        convergence criteria
    max_iterations: int
        max number of iterations ICP will run
    F_reg: Frame object
        initial transform

    Returns:
    s_k: N_samples x 3 (N_sample points)
        F_reg * d_k
    c_k: N_samples x 3 (N_sample points)
        closest points on mesh to s_k in final iteration
    mag: N_samples
        distance between s_k and c_k
    F_reg: Frame object
        point to surface registration found by ICP
    '''

    # Find c_k and calculate s_k
    s_k = np.empty((d_k.shape[0], 3))
    c_k = np.zeros((d_k.shape[0], 3))
    converged = False
    iterations = 0
    while not converged and iterations < max_iterations: # max num of iterations is 100
        for i in range(d_k.shape[0]): # search tree to find closest point
            s_k[i] = F_reg.R.dot(d_k[i]) + F_reg.p.coords
            nearest_node = search(root, s_k[i])
            c_k[i] = findClosestPointOnTriangle(s_k[i], nearest_node.triangle)
            
        F_reg = registrationArunMethod(d_k, c_k, "reg")

        # Update s_k and calculate || s_k - c_k ||
        mag = np.empty(d_k.shape[0])
        for i in range(s_k.shape[0]):
            s_k[i] = F_reg.R.dot(d_k[i]) + F_reg.p.coords
            mag[i] = calcDistance(s_k[i], c_k[i])

        # check convergence
        if np.mean(mag) < d_max:
            converged = True 

        iterations += 1

    return s_k, c_k, mag, F_reg