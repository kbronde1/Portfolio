import numpy as np
from Point3d import Point3d
from Registration import registrationArunMethod

def pivotCalibration(G):
    '''
    Parameters: G: N_frames x N_G x 3 (N_frames x N_G points)
    points to be used for pivot calibration

    Return:
    p_dimple: point of the pivot in EM coordinates
    p_tip: center of pointer to tip in local coordinates
    g: local coordinate points to be use to find local transformation
    '''

    # defining a local coordinate system
    Gt = G[0].transpose()
    G0 = np.mean(Gt, axis=1).reshape((-1,1))
    g = Gt - G0

    # creating the [R -I][p_tip p_dimple]^T = [-t] matrix for each frame
    F_G = registrationArunMethod(g.transpose(), G[0], "G")
    negI = [[-1, 0, 0], [0, -1, 0], [0, 0, -1]]
    R_Gs = np.hstack((F_G.R, negI))
    t_Gs = F_G.p.coords.transpose()[..., np.newaxis]
    for i in range(1, G.shape[0]):
        F_G = registrationArunMethod(g.transpose(), G[i], "G")
        R_G = np.hstack((F_G.R, negI))
        R_Gs = np.vstack((R_Gs, R_G))
        t_Gs = np.vstack((t_Gs, F_G.p.coords.transpose()[..., np.newaxis]))
    t_Gs = t_Gs * -1

    # singular value decomposition least squares
    U, S, Vt = np.linalg.svd(R_Gs, full_matrices=True)
    zeros = np.zeros((Vt.shape[0],U.shape[0]-Vt.shape[0]))
    Sinv = np.hstack((np.linalg.inv(np.diag(S)), zeros))
    y = Sinv @ U.transpose() @ t_Gs
    x = Vt.transpose() @ y

    # return p_dimple
    p_dimple = Point3d("EM", x[3][0],x[4][0],x[5][0])
    p_tip = Point3d("EM", x[0][0],x[1][0],x[2][0])
    return p_dimple, p_tip, g

def GtoEM(G, p_tip, g):
    '''
    Parameters: 
    G: N_frames x N_G x 3 (N_frames x N_G points)
        points to be used for pivot calibration
    p_tip: center of pointer to tip in local coordinates
    g: local coordinate points to be use to find local transformation

    Return:
    B: location of the pointer
    '''
    B = np.empty((G.shape[0], 3))

    # calculating new pointer location using Gs and p_tip
    # find transformation between new Gs and local coordinate gs that 
    # p_tip is defined to
    for i in range(G.shape[0]):
        F_G = registrationArunMethod(g.transpose(), G[i], "G")
        B[i] = F_G.R @ p_tip.coords + F_G.p.coords
        
    return B