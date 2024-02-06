# CIS
Kiana Bronder, kbronde1
Keerthana Thammana, lthamma1

## Executable
Filename: GenerateOutputPA4.py
Instructions: Run the above file and input on the command line as prompted. Prompts include specific formats to follow. Make sure you are in the PROGRAMS directory. GenerateOutputPA4.py creates the output4 file containing s_k positions, c_k positions, and the magnitude of distance between the two for different sample frames k using ICP.

## Files Summary
FileIO.py: functions to read the calbody, calreadings, empivot, optpivot, ct-fiducials, em-fiducials, em-nav, output1, and output2 files.  
GenerateOutput.py: functions to read the unknown dataset files and generate an output1 txt file.
Point3d.py: class to create 3d point objects, calculate error between 2 points, print as string, save frame data, and other cartesian math functions.
Frame.py: class to do frame transformations, save frame data, and transform points.
Registration.py: an implementation of the Arun's registration method.
EMPivotCalibration.py: implementation of EM tracker pivot calibration, as well as using results from the pivot calibration to calculate the location of the pointer
OpticalPivotCalibration.py: implementation of optical tracker pivot calibration.
DistortionCorrection.py: implementation of calculating and correcting Berstein polynomial distortion
testing.py: script we used to debug our functions using the debug data sets and print error between the two.
ClosestPointOnTriangle.py: functions to find the closest location of a triangle to a given point
Mesh.py: class to store the surface mesh, and functions for KD-tree
ICP.py: function to run iterative closest point registration
