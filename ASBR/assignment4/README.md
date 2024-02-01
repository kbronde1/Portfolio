# Assignment 4
In this assignment, I implemented an extended Kalman (EKF) filter to estimate the location of a mobile robot (Clearpath Robotics Jackal) on a simulated rugged terrain. The simulation uses the Gazebo environment. In the simulation, the robot is equipped with a GPS sensor to determine the robot coordinates and an inertial measurement unit (IMU) to measure its orientation. 

The EKF engine is based on the Orocos Bayesian Filtering Library (BFL) and my task was to:
1. Determine and implement the 3D process model of the robot.
2. Determine and implement the observation model for both sensors (GPS and IMU).
3. Determine and implement all the Jacobians (system and sensors).
