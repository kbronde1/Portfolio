# Assignment 2
In this assignment, I implemented and tested a hand-eye calibration algorithm by using a simulation of a UR-5 mounted with a camera (eye-in-hand configuration). I manually moved the robot to several configurations to observe the augmented reality (AR) tag positioned on the floor. The position/orientation of the AR tag with respect to the camera was automatically computed and the position/orientation of the end-effector (forward kinematics) was provided by ROS. I used these measurements to test my algorithm and compare my results to the hand-eye transformation used by the simulation.