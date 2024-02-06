# Computer-Integrated Surgery I
Jointly contributed by Kiana Bronder and Keerthana Thammana
### Overview
This course focuses on computer-based techniques, systems, and applications exploiting quantitative information from medical images and sensors to assist clinicians in all phases of treatment from diagnosis to preoperative planning, execution, and follow-up. In this GitHub, we implement algorithms for frame transformations, registration, pivot calibration, distortion correction, iterative closest point, and surface mesh functions. These algorithms can be found in the PROGRAMS folder.
### Programming Assignments 1 and 2
This problem concerned calibration, simple registration, and tracking for a stereotactic navigation system that uses an electromagnetic positional tracking device.
#### PA 1
This first assignment was mainly package development. Specifically, we developed:
1. Cartesian math package for 3D points, rotations, and frame transformations
2. 3D point set to 3D point set registration algorithm
3. "Pivot" calibration method
4. Distortion prediction method ("expected" values given a distortion calibration data set)

We then used these packages to implement pivot calibration for both the EM probe and optical tracking probe.
#### PA 2
In this assignment, we fit polynomials to model distortion and then used these polynomials to “dewarp” the EM tracker space. Using the dewarped EM tracker space to repeat our pivot calibration, we computed registration to a CT coordinate system and reported corresponding CT coordinates.
### Programming Assignments 3 and 4
In these problems, we implemented and used a simplified version of iterative-closest point registration given A 3D surface (represented as a mesh of triangles), a pair of definition files for two rigid bodies (“A” and “B”), and a file of “sample” readings (positions of the LED markers relative to an optical tracker).
#### PA 3
For this assignment, we implemented the matching part of the ICP
algorithm.
#### PA 4
Continuing from PA 3, we added an iteration to implement a
complete ICP algorithm.
### Files Summary
FileIO.py: functions to read the calbody, calreadings, empivot, optpivot, ct-fiducials, em-fiducials, em-nav, output1, and output2 files.

GenerateOutput.py: functions to read the unknown dataset files and generate an output1 txt file.

Point3d.py: class to create 3d point objects, calculate error between 2 points, print as string, save frame data, and other cartesian math functions.

Frame.py: class to do frame transformations, save frame data, and transform points.

Registration.py: an implementation of the Arun's registration method.

EMPivotCalibration.py: implementation of EM tracker pivot calibration, as well as using results from the pivot calibration to calculate the location of the pointer

OpticalPivotCalibration.py: implementation of optical tracker pivot calibration.

DistortionCorrection.py: implementation of calculating and correcting Berstein polynomial distortion

testing.py: script we used to debug our functions using the debug data sets and print error between the two.

ClosestPointOnTriangle.py: functions to find the closest location of a triangle to a given point.

Mesh.py: class to store the surface mesh, and functions for KD-tree

ICP.py: function to run iterative closest point registration
