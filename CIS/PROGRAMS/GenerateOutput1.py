"""
CIS PA #1

Kiana Bronder, kbronde1
Keerthana Thammana, lthamma1
"""
import numpy as np
from Point3d import Point3d
from FileIO import read_calbody, read_calreadings, read_empivot, read_optpivot, read_output1
from Registration import registrationArunMethod
from EMPivotCalibration import pivotCalibration
from OpticalPivotCalibration import opticalCalibration
from DistortionCorrection import correctDistortion, calcDistortionCorrection

import os
cwd = os.getcwd()
print(cwd)

# change input and output filenames
dataset = "PA2 Student Data/pa2-debug-f"
filename = "pa2-debug-f-output1.txt"

# output file:
    # N_C , N_frames, NAME-OUTPUT1.TXT
    # P_EM
    # P_opt
    # C_1 through C_N_frames

# output file name and directory
f = open("../OUTPUT/"+filename, "w")

# read in input data
d, a, c = read_calbody(dataset+"-calbody.txt")
G = read_empivot(dataset+"-empivot.txt")
D_opt, H = read_optpivot(dataset+"-optpivot.txt")
D, A, C = read_calreadings(dataset+"-calreadings.txt")

# calculating expected Cs
C_expected = np.zeros(C.shape)
for i in range(D.shape[0]):
    F_D = registrationArunMethod(d, D[i], "D")
    F_A = registrationArunMethod(a, A[i], "A")
    F_DA = F_D.inverse() * F_A
    for j in range(c.shape[0]):
        C0_exp = np.matmul(F_DA.R, c[j].transpose()[..., np.newaxis]) + F_DA.p.coords.transpose()[..., np.newaxis]
        C_expected[i][j] = [C0_exp[0][0], C0_exp[1][0], C0_exp[2][0]]

coef, q_min, q_max = calcDistortionCorrection(np.vstack(C_expected), np.vstack(C), 5)

# EM pivot calibration
G = read_empivot(dataset+"-empivot.txt")
G_corr = np.empty(G.shape)
for i in range(G_corr.shape[0]):
    G_corr[i] = correctDistortion(G[i], coef, q_min, q_max, 5)
P_em_exp, P_tip, g = pivotCalibration(G_corr)

# Calculate P_opt
P_opt_exp = opticalCalibration(d,D_opt,H)

# Calculate C_exp and write all variables to output file
N_frames = D.shape[0]
N_C = c.shape[0]
f.write(str(N_C) + ", " + str(N_frames) + ", " + filename + "\n")
f.write('EM Pivot Position:' + P_em_exp.__str__() + "\n")
f.write('OPT Pivot Position:' + P_opt_exp.__str__() + "\n")
error = 0
for i in range(N_frames): # N_frames
    for j in range(N_C): # N_C
        point = Point3d("EM",C_expected[i][j])
        f.write(point.__str__() + "\n")
f.close()
