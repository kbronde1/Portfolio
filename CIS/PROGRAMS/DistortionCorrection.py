import numpy as np
import math
from Point3d import Point3d
from FileIO import read_calbody, read_calreadings
from Registration import registrationArunMethod

def bernstein(N, k, v):
    return (math.comb(N, k)) * ((1-v)**(N-k)) * (v**k)

def bernsteinPolynomialF(N, q, q_min, q_max):
    # scale measured values
    diff = q_max - q_min
    u = np.empty(q.shape)
    for i in range(q.shape[0]):
        u[i] = (q[i] - q_min)/diff

    # calculate F values using 3D bernstein polynomials for each point
    F = np.empty((u.shape[0], (N+1)**3))
    for x in range(u.shape[0]):
        index = 0
        for i in range(N + 1):
            for j in range(N + 1):
                for k in range(N + 1):
                    F[x][index] = bernstein(N, i, u[x][0])*bernstein(N, j, u[x][1])*bernstein(N, k, u[x][2])
                    index += 1
    return F

def calcDistortionCorrection(p, q, N):
    '''
    Parameters:
    p: ground-truth of 3D points, with no distortion
    q: corresponding 3D points read by navigation sensor, with distortion

    Returns:
    coef: coefficients for Bernstein polynomials to correct distortion
    q_min: lower bound for 3D points of bounding box for scaling
    q_max: upper bound for 3D points of bounding box for scaling
    '''

    # create a bounding box by finding min and max along every axis
    q_min = np.amin(q, axis=0)
    q_max = np.amax(q, axis=0)

    # create F matrix of berstein polynomials based on measurements
    F = bernsteinPolynomialF(N, q, q_min, q_max)

    # singular value decomposition least squares
    U, S, Vt = np.linalg.svd(F, full_matrices=True)
    zeros = np.zeros((Vt.shape[0],U.shape[0]-Vt.shape[0]))
    Sinv = np.hstack((np.linalg.inv(np.diag(S)), zeros))
    y = Sinv @ U.transpose() @ p
    coef = Vt.transpose() @ y

    return coef, q_min, q_max

def correctDistortion(q, coef, q_min, q_max, N):
    '''
    Parameters:
    q: 3D points read by navigation sensor with distortion
    coef: distortion correction coef, given by calcDistortionCorrection
    q_min: minimum of scaling box, given by calcDistortionCorrection
    q_max: maximum of scaling box, given by calcDistortionCorrection

    Returns:
    p: undistorted 3D points
    '''

    # create F matrix of bernstein polynomials and multiply with coef
    F = bernsteinPolynomialF(N, q, q_min, q_max)
    p = F @ coef

    return p # return values without distortion
