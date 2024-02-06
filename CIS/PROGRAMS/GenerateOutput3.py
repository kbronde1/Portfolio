"""
CIS PA #3

Kiana Bronder, kbronde1
Keerthana Thammana, lthamma1
"""
import numpy as np
from FileIO import read_probbody, read_mesh, read_samplereadings, read_output3
from Registration import registrationArunMethod
from Point3d import Point3d
from ClosestPointOnTriangle import findClosestPointOnTriangle
from Mesh import Mesh, insert, search
from testing import calcDistance

import os
cwd = os.getcwd()
print(cwd)

# change input and output filenames
dataset = "PA345 Student Data/PA3-K-Unknown"
output_filename = "PA3-K-Unknown-Output.txt"

f = open("../OUTPUT/"+output_filename, "w")

A, A_tip, N_A = read_probbody("PA345 Student Data/Problem3-BodyA.txt")
B, B_tip, N_B = read_probbody("PA345 Student Data/Problem3-BodyB.txt")
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
V, ind, n = read_mesh("PA345 Student Data/Problem3MeshFile.sur")
mesh = Mesh(V, ind, n)
root = None
for i in range(ind.shape[0]):
    verts = mesh.getVerticesOfTriangle(i)
    root = insert(root, mesh.calcCentroid(verts), verts, 0, i)

# Calculate c_k and || d_k - c_k ||
c_k = np.empty((a.shape[0], 3))
mag = np.empty(a.shape[0])
for i in range(d_k.shape[0]):
    nearest_node = search(root, d_k[i])
    c_k[i] = findClosestPointOnTriangle(d_k[i], nearest_node.triangle)
    mag[i] = calcDistance(d_k[i], c_k[i])
    point_d = Point3d("",d_k[i])
    point_c = Point3d("",c_k[i])
    string = point_d.__str__() + "     " + point_c.__str__()
    string += "     {:.3f}".format(round(mag[i], 3))
    f.write(string + "\n")