"""
CIS PA #4

Kiana Bronder, kbronde1
Keerthana Thammana, lthamma1
"""
import numpy as np
from FileIO import read_probbody, read_mesh, read_samplereadings
from Registration import registrationArunMethod
from Point3d import Point3d
from Mesh import Mesh, insert
from Frame import Frame
from ICP import ICP

import os
cwd = os.getcwd()

# Prompt the user for the dataset path
print('Current directory: ', cwd)
print('Please input in the following format: PA345 Student Data/PA4-X-XXX')
print('Where X is the dataset letter and XXX is Debug or Unknown')
dataset = input("Dataset: ")
print(dataset)

# Prompt the user for the desired output file name
print('Please input in the following format: PA4-X-XXX-Output.txt')
output_filename = input("Desired output file name: ")
print(output_filename)
# dataset = "PA345 Student Data/PA4-A-Debug"
# output_filename = "PA4-A-Debug-Output.txt"
print('Running code...')

f = open("../OUTPUT/"+output_filename, "w")

A, A_tip, N_A = read_probbody("PA345 Student Data/Problem4-BodyA.txt")
B, B_tip, N_B = read_probbody("PA345 Student Data/Problem4-BodyB.txt")
a, b = read_samplereadings(dataset+"-SampleReadingsTest.txt", N_A, N_B)

# Calculate d_k
A_tip = A_tip.transpose()[..., np.newaxis]
d_k = np.empty((a.shape[0], 3))
for i in range(a.shape[0]): # N_samples:
    F_Ai = registrationArunMethod(A, a[i], "A")
    F_Bi = registrationArunMethod(B, b[i], "B")
    F_BA = F_Bi.inverse() * F_Ai
    d_k[i] = (F_BA.R @ A_tip)[:,0] + F_BA.p.coords
    
f.write(str(a.shape[0]) + " " + output_filename+ "\n") # first line of file

# Construct KD tree
V, ind, n = read_mesh("PA345 Student Data/Problem4MeshFile.sur")
mesh = Mesh(V, ind, n)
root = None
for i in range(ind.shape[0]):
    verts = mesh.getVerticesOfTriangle(i)
    root = insert(root, mesh.calcCentroid(verts), verts, 0, i)

# run ICP
d_max = 0.15
max_iterations = 100
F_reg_initial = Frame("reg", np.identity(3), Point3d("reg", 0, 0, 0))
s_k, c_k, mag, _ = ICP(d_k, root, d_max, max_iterations, F_reg_initial)

# Write s_k, c_k, and || s_k - c_k || to file
for i in range(d_k.shape[0]):
    point_s = Point3d("",s_k[i])
    point_c = Point3d("",c_k[i])
    string = point_s.__str__() + "     " + point_c.__str__()
    string += "     {:.3f}".format(round(mag[i], 3))
    f.write(string + "\n")

f.close()
    
print('Wrote', output_filename)