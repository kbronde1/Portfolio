import numpy as np
from FileIO import read_calbody, read_calreadings, read_empivot, read_optpivot, read_output1, read_ctfiducials 
from FileIO import read_emfiducials, read_emnav, read_output2, read_mesh, read_output3, read_samplereadings, read_probbody
from Registration import registrationArunMethod
from EMPivotCalibration import pivotCalibration, GtoEM
from OpticalPivotCalibration import opticalCalibration
from Point3d import Point3d
from DistortionCorrection import bernstein, calcDistortionCorrection, correctDistortion
from ClosestPointOnTriangle import findClosestPointOnTriangle
from Mesh import Mesh, insert, search
import time
from Frame import Frame

def testBernstein():
    # b_2,5(x) = 10x^2*(1 - x)^3 --> n = 5, k = 2, v = x
    x = 5
    expected = 10 * x**2 * (1-x)**3
    actual = bernstein(5,2,x)
    print('Bernstein error:',abs(expected-actual))

def testRegistration():
    print("Registration Test:")
    d, a, c = read_calbody("PA1 Student Data/pa1-debug-g-calbody.txt")
    # use given d (from PA1 data set g) and artificial D
    artiD = np.array([[0.3404, 0.5853, 0.2238],[-100.2885, -80.1432, 76.7532], 
                [111.0513, -62.0671, 79.7082],[10.4224, -142.7955, 156.2376],
                [11.1531, -109.2218, -101.3906],[-89.4758, -189.9503, -24.8612],
                [121.8640, -171.8742, -21.9062],[21.2351, -252.6026, 54.6232]])
    artiF_D = registrationArunMethod(d, artiD, "D")
    # artificial R and p calculated on MATLAB
    artiR = np.array([[0.0721, 0.7381, -0.6709],[-0.7320, -0.4177, -0.5382],[-0.6774, 0.5299, 0.5102]])
    print('Calculated R:', artiF_D.R)
    print('Expected R:',artiR)
    print('Calculated p:',artiF_D.p.__str__())
    print('Expected p: 0.3404, 0.5853, 0.2238')

def testDistortionReconstructsCexp(dataset):
    print("Distortion Test:")
    # calculating expected Cs
    d, a, c = read_calbody(dataset+"-calbody.txt")
    D, A, C = read_calreadings(dataset+"-calreadings.txt")
    C_expected = np.empty(C.shape)
    for i in range(D.shape[0]):
        F_D = registrationArunMethod(d, D[i], "D")
        F_A = registrationArunMethod(a, A[i], "A")
        F_DA = F_D.inverse() * F_A
        for j in range(c.shape[0]):
            C0_exp = np.matmul(F_DA.R, c[j].transpose()[..., np.newaxis]) + F_DA.p.coords.transpose()[..., np.newaxis]
            C_expected[i][j] = [C0_exp[0][0], C0_exp[1][0], C0_exp[2][0]]

    C_expected_stacked = np.vstack(C_expected)
    C_stacked = np.vstack(C)
    C_errors = np.zeros(C_stacked.shape[0])
    coef, q_min, q_max = calcDistortionCorrection(C_expected_stacked, C_stacked, 3)
    C_undistorted = correctDistortion(C_stacked, coef, q_min, q_max, 3)
    for i in range(C_stacked.shape[0]): # N_C
        P_Cexp = Point3d("C", C_expected_stacked[i])
        C_errors[i] = P_Cexp.error(C_undistorted[i])
    print('Average calibration error per point:', np.mean(C_errors), 'mm')
    
def testKDTree():
    points = [[3, 6, 1], [17, 15, 0], [13, 15, 6], [6, 12, 4], [9, 1, 2], [2, 7, 3], [10, 19, 5]]
    root = None

    for point in points:
        root = insert(root, point, 0, 0, 0)

    point = [12, 10, 5]
    nearest_node = search(root, point)

    if nearest_node:
        print("Nearest node:", nearest_node.point)
    else:
        print("Tree is empty.")
    print('Correct answer: [13, 15, 6]')
    
def testClosestPoint():
    print("Test 1: Closest Point in Triangle")
    a = np.array([1.75,0.5,1])
    p = np.array([1,0,0])
    q = np.array([2,0,0])
    r = np.array([2,1,0])
    v_coords = np.asarray([p, q, r])
    c = findClosestPointOnTriangle(a, v_coords)
    print('Closest Point:', c)
    print('Correct answer: [1.75, 0.5, 0.]')

    print("Test 2: Closest Point on Edge 1")
    a = np.array([0,2,0.5])
    p = np.array([1,0,0])
    q = np.array([2,0,0])
    r = np.array([2,1,0])
    v_coords = np.asarray([p, q, r])
    c = findClosestPointOnTriangle(a, v_coords)
    print('Closest Point:', c)
    print('Correct answer: [1.5, 0.5, 0.]')

    print("Test 3: Closest Point on Edge 2")
    a = np.array([1.75,-0.5,1])
    p = np.array([1,0,0])
    q = np.array([2,0,0])
    r = np.array([2,1,0])
    v_coords = np.asarray([p, q, r])
    c = findClosestPointOnTriangle(a, v_coords)
    print('Closest Point:', c)
    print('Correct answer: [1.75, 0, 0.]')

    print("Test 4: Closest Point on Edge 3")
    a = np.array([3,0.5,-0.5])
    p = np.array([1,0,0])
    q = np.array([2,0,0])
    r = np.array([2,1,0])
    v_coords = np.asarray([p, q, r])
    c = findClosestPointOnTriangle(a, v_coords)
    print('Closest Point:', c)
    print('Correct answer: [2, 0.5, 0.]')

def printPA2OutputErrors(dataset):
    print("PA2 Output Errors:")
    C_exp,P_em,_ = read_output1(dataset+"-output1.txt")

    # calculating expected Cs
    d, a, c = read_calbody(dataset+"-calbody.txt")
    D, A, C = read_calreadings(dataset+"-calreadings.txt")
    C_expected = np.zeros(C.shape)
    for i in range(D.shape[0]):
        F_D = registrationArunMethod(d, D[i], "D")
        F_A = registrationArunMethod(a, A[i], "A")
        F_DA = F_D.inverse() * F_A
        for j in range(c.shape[0]):
            C0_exp = np.matmul(F_DA.R, c[j].transpose()[..., np.newaxis]) + F_DA.p.coords.transpose()[..., np.newaxis]
            C_expected[i][j] = [C0_exp[0][0], C0_exp[1][0], C0_exp[2][0]]

    coef, q_min, q_max = calcDistortionCorrection(np.vstack(C_expected), np.vstack(C), 5)
    C_errors = np.zeros(C.shape[0:2]) # stores error of each frame
    for i in range(C.shape[0]): # N_frames
        for j in range(C.shape[1]): # N_C
            P_Cexp = Point3d("C", C_exp[i][j])
            C_errors[i][j] = P_Cexp.error(C_expected[i][j])
    print('Average calibration error per point:', np.mean(C_errors), 'mm')

    # EM pivot calibration
    G = read_empivot(dataset+"-empivot.txt")
    G_corr = np.empty(G.shape)
    for i in range(G_corr.shape[0]):
        G_corr[i] = correctDistortion(G[i], coef, q_min, q_max, 5)
    P_em_exp, P_tip, g = pivotCalibration(G_corr)
    print('EM Pivot Error:',P_em_exp.error(P_em),'mm')

    # Fiducial point registration
    b = read_ctfiducials(dataset+"-ct-fiducials.txt")
    G_EM = read_emfiducials(dataset+"-em-fiducialss.txt")
    G_EM_corr = np.empty(G_EM.shape)
    for i in range(G_EM_corr.shape[0]):
        G_EM_corr[i] = correctDistortion(G_EM[i], coef, q_min, q_max, 5)
    B = GtoEM(G_EM_corr, P_tip, g)

    # Calculate F_reg
    F_reg = registrationArunMethod(B, b, "EM")

    # Compute tip location w.r.t. CT image
    G_nav = read_emnav(dataset+"-EM-nav.txt")
    G_nav_corr = np.empty(G_nav.shape)
    for i in range(G_nav_corr.shape[0]):
        G_nav_corr[i] = correctDistortion(G_nav[i], coef, q_min, q_max, 5)
    V = GtoEM(G_nav_corr, P_tip, g)
    v_exp = np.empty([G_nav.shape[0],3])
    for i in range(V.shape[0]):
        v_exp[i] = F_reg.R.dot(V[i]) + F_reg.p.coords

    # Calculate average v error
    v = read_output2(dataset+"-output2.txt")
    errors = np.empty(v.shape[0])
    for i in range(v.shape[0]):
        p_v = Point3d("CT",v[i])
        errors[i] = p_v.error(v_exp[i])
    print('Avg v error:', np.mean(errors))

def printPA1OutputErrors(dataset):
    print("PA1 Output Errors:")
    C_exp,P_em,P_opt = read_output1(dataset+"-output1.txt")

    # calculating expected Cs
    d, a, c = read_calbody(dataset+"-calbody.txt")
    D, A, C = read_calreadings(dataset+"-calreadings.txt")
    error = 0
    for i in range(D.shape[0]):
        F_D = registrationArunMethod(d, D[i], "D")
        F_A = registrationArunMethod(a, A[i], "A")
        F_DA = F_D.inverse() * F_A
        for j in range(c.shape[0]):
            C0_exp = np.matmul(F_DA.R, c[j].transpose()[..., np.newaxis]) + F_DA.p.coords.transpose()[..., np.newaxis]
            gen_point = Point3d("DA", C0_exp[0][0], C0_exp[1][0], C0_exp[2][0])
            error += gen_point.error(C[i][j])
    error = error/(C_exp.shape[0]*C_exp.shape[1])
    print('Average Expected C Error:',error)

    # EM pivot calibration
    G = read_empivot(dataset+"-empivot.txt")
    P_em_exp, P_tip, _ = pivotCalibration(G)
    print('EM Calculated output:', P_em_exp.__str__())
    print('EM Expected output:',P_em)
    print('EM Pivot Error:',P_em_exp.error(P_em),'mm')

    # Optical tracker pivot calibration
    D,H = read_optpivot(dataset+"-optpivot.txt")
    d,_,_ = read_calbody(dataset+"-calbody.txt")
    P_opt_exp = opticalCalibration(d,D,H)
    print('OPT Calculated output:', P_opt_exp.__str__())
    print('OPT Expected output:',P_opt)
    print('OPT Pivot Error:',P_opt_exp.error(P_opt),'mm')

def calcDistance(x, y):
    return np.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2 + (x[2] - y[2])**2)
    
def printPA3OutputErrorsLinearICP(dataset):
    print("PA3 Output Errors w Linear Search:")

    start_time = time.time()
    
    # test I/O functions
    A, A_tip, N_A = read_probbody("PA345 Student Data/Problem3-BodyA.txt")
    B, B_tip, N_B = read_probbody("PA345 Student Data/Problem3-BodyB.txt")
    V, ind, n = read_mesh("PA345 Student Data/Problem3MeshFile.sur")
    mesh = Mesh(V, ind, n)
    a, b = read_samplereadings(dataset+"-SampleReadingsTest.txt", N_A, N_B)
    d_exp, c_exp, mag = read_output3(dataset+"-Output.txt")
    d_error = 0
    c_error = 0
    mag_error = 0

    d_k = np.empty((a.shape[0], 3))
    A_tip = A_tip.transpose()[..., np.newaxis]
    for i in range(a.shape[0]): # N_samples:
        F_Ai = registrationArunMethod(A, a[i], "A")
        F_Bi = registrationArunMethod(B, b[i], "B")
        F_BA = F_Bi.inverse() * F_Ai
        d_k[i] = (F_BA.R @ A_tip)[:,0] + F_BA.p.coords
        d_error += calcDistance(d_exp[i], d_k[i])

    # for PA3, F_reg = 1
    # linear search to find the closest points to d_k
    c_k = np.empty((a.shape[0], 3))
    for i in range(d_k.shape[0]):
        shortest_dist = np.infty
        # shortest_j = -1
        for j in range(ind.shape[0]):
            c = findClosestPointOnTriangle(d_k[i], mesh.getVerticesOfTriangle(j))
            dist = calcDistance(d_k[i], c)
            if dist <= shortest_dist:
                closest_point = c
                shortest_dist = dist
                # shortest_j = j
        # print(shortest_j)
        c_k[i] = closest_point
        c_error += calcDistance(c_exp[i], c_k[i])
        mag_error += (np.abs(mag[i]-shortest_dist))

    print("Time of execution: %s seconds" % (time.time() - start_time))
    print("d_k error: ", d_error/d_k.shape[0])
    print("c_k error: ", c_error/c_k.shape[0])
    print("mag error: ", mag_error/c_k.shape[0])

def printPA3OutputErrorsOptimizedICP(dataset):
    print("PA3 Output Errors w KdTree Search:")

    start_time = time.time()
    
    # test I/O functions
    A, A_tip, N_A = read_probbody("PA345 Student Data/Problem3-BodyA.txt")
    B, B_tip, N_B = read_probbody("PA345 Student Data/Problem3-BodyB.txt")
    V, ind, n = read_mesh("PA345 Student Data/Problem3MeshFile.sur")
    mesh = Mesh(V, ind, n)
    a, b = read_samplereadings(dataset+"-SampleReadingsTest.txt", N_A, N_B)
    d_exp, c_exp, mag = read_output3(dataset+"-Output.txt")
    d_error = 0
    c_error = 0
    mag_error = 0

    d_k = np.empty((a.shape[0], 3))
    A_tip = A_tip.transpose()[..., np.newaxis]
    for i in range(a.shape[0]): # N_samples:
        F_Ai = registrationArunMethod(A, a[i], "A")
        F_Bi = registrationArunMethod(B, b[i], "B")
        F_BA = F_Bi.inverse() * F_Ai
        d_k[i] = (F_BA.R @ A_tip)[:,0] + F_BA.p.coords
        d_error += calcDistance(d_exp[i], d_k[i])

    # for PA3, F_reg = 1
    # kdtree search to find the closest points to d_k
    root = None
    
    # Run time should now be O(n) instead of O(n^2)
    for i in range(ind.shape[0]):
        verts = mesh.getVerticesOfTriangle(i)
        root = insert(root, mesh.calcCentroid(verts), verts, 0, i)
    
    c_k = np.empty((a.shape[0], 3))
    for i in range(d_k.shape[0]):
        nearest_node = search(root, d_k[i])
        c_k[i] = findClosestPointOnTriangle(d_k[i], nearest_node.triangle)
        shortest_dist = calcDistance(d_k[i], c_k[i])
        # print(nearest_node.idx)
        c_error += calcDistance(c_exp[i], c_k[i])
        mag_error += (np.abs(mag[i]-shortest_dist))
    
    # Slight difference in these values
    print("Time of execution: %s seconds" % (time.time() - start_time))
    print("d_k error: ", d_error/d_k.shape[0])
    print("c_k error: ", c_error/c_k.shape[0])
    print("mag error: ", mag_error/c_k.shape[0])

def printPA4OutputErrors(dataset):
    # dataset = "PA345 Student Data/PA4-A-Debug"
    print("PA4 Output Errors w KdTree Search:")
    print(dataset)
    
    # test I/O functions
    A, A_tip, N_A = read_probbody("PA345 Student Data/Problem4-BodyA.txt")
    B, B_tip, N_B = read_probbody("PA345 Student Data/Problem4-BodyB.txt")
    V, ind, n = read_mesh("PA345 Student Data/Problem4MeshFile.sur")
    mesh = Mesh(V, ind, n)
    a, b = read_samplereadings(dataset+"-SampleReadingsTest.txt", N_A, N_B)
    s_exp, c_exp, mag = read_output3(dataset+"-Output.txt") # same file format as PA3
    s_error = 0
    c_error = 0
    mag_error = 0
    
    d_k = np.empty((a.shape[0], 3))
    A_tip = A_tip.transpose()[..., np.newaxis]
    for i in range(a.shape[0]): # N_samples:
        F_Ai = registrationArunMethod(A, a[i], "A")
        F_Bi = registrationArunMethod(B, b[i], "B")
        F_BA = F_Bi.inverse() * F_Ai
        d_k[i] = (F_BA.R @ A_tip)[:,0] + F_BA.p.coords
        # d_error += calcDistance(d_exp[i], d_k[i])
    
    # kdtree search to find the closest points c_k to s_k
    root = None
    for i in range(ind.shape[0]): # build tree
        verts = mesh.getVerticesOfTriangle(i)
        root = insert(root, mesh.calcCentroid(verts), verts, 0, i)
    
    # for PA4, iteratively find F_reg
    s_k = np.empty((a.shape[0], 3))
    c_k = np.zeros((a.shape[0], 3))

    converged = False
    F_reg = Frame("reg", np.identity(3), Point3d("reg", 0, 0, 0)) # Assume F_reg = I for initial guess
    iterations = 0
    d_max = 0.15
    while not converged and iterations < 100: # for now, max num of iterations is 100
        for i in range(d_k.shape[0]): # search tree
            s_k[i] = F_reg.R.dot(d_k[i]) + F_reg.p.coords
            nearest_node = search(root, s_k[i])
            c_k[i] = findClosestPointOnTriangle(s_k[i], nearest_node.triangle)
            
        F_reg = registrationArunMethod(d_k, c_k, "reg")

        shortest_dist = np.empty(d_k.shape[0])
        for i in range(d_k.shape[0]):
            s_k[i] = F_reg.R.dot(d_k[i]) + F_reg.p.coords
            shortest_dist[i] = calcDistance(s_k[i], c_k[i])

        if np.mean(shortest_dist) < d_max:
            converged = True 

        iterations += 1

    for i in range(d_k.shape[0]):
        c_error += calcDistance(c_exp[i], c_k[i])
        s_error += calcDistance(s_exp[i], s_k[i])
        mag_error += (np.abs(mag[i]-shortest_dist[i]))
    
    print("s_k error: ", s_error/s_k.shape[0])
    print("c_k error: ", c_error/c_k.shape[0])
    print("mag error: ", mag_error/mag.shape[0])
    print("Iterations: ", iterations)