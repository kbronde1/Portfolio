
#include "ekf_models.hpp"
#include <tf/tf.h>
#include "utilities.h"

// order: g, G, WMWt, R, gps, Hgps, imu, Himu

/**
   TODO
   Fill in the value of the process covariance matrix. The rows/columns of WMWt are
   in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ].
   \param[out] WMWt Covariance matrix of the system.
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_WMWt( double WMWt[6][6], const State& state, double v, double w, double dt ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      WMWt[r][c] = 0.0;

  // TODO fill in the matrix WMWt
  double a1 = 0.1;
  double a2 = 0.2;
  double a3 = 0.1;
  double a4 = 0.2;
  double thetax = state.x[3];
  double thetay = state.x[4];
  double thetaz = state.x[5];
  /**  t2 = dt;
  t3 = thetax;
  t4 = thetay;
  t5 = thetaz;
  t6 = cos(thetax);
  t7 = cos(thetay);
  t8 = cos(thetaz);
  t9 = sin(thetax);
  t10 = sin(thetay);
  t11 = sin(thetaz);
  t18 = v*v;
  t19 = w*w;
  t12 = cos(t3);
  t13 = cos(t4);
  t14 = cos(t5);
  t15 = sin(t3);
  t16 = sin(t4);
  t17 = sin(t5);
  t20 = t7+t9;
  t21 = t6+t11;
  t22 = t8+t10;
  t23 = a1*t18;
  t24 = a3*t18;
  t25 = a2*t19;
  t26 = a4*t19;
  t27 = t13+t15;
  t28 = t12+t17;
  t29 = t14+t16;
  t30 = t23+t25;
  t31 = t24+t26;
  t32 = dt*t2*t31;
  WMWt[0][0] = dt*t2*t22*t29*t30;
  WMWt[0][1] = dt*t2*t22*t28*t30;
  WMWt[0][2] = dt*t2*t22*t27*t30;
  WMWt[1][0] = dt*t2*t21*t29*t30;
  WMWt[1][1] = dt*t2*t21*t28*t30;
  WMWt[1][2] = dt*t2*t21*t27*t30;
  WMWt[2][0] = dt*t2*t20*t29*t30;
  WMWt[2][1] = dt*t2*t20*t28*t30;
  WMWt[2][2] = dt*t2*t20*t27*t30;
  WMWt[3][3] = t32;
  WMWt[3][4] = t32;
  WMWt[3][5] = t32;
  WMWt[4][3] = t32;
  WMWt[4][4] = t32;
  WMWt[4][5] = t32;
  WMWt[5][3] = t32;
  WMWt[5][4] = t32;
  WMWt[5][5] = t32;**/
  t2 = dt;
  t3 = thetaz;
  t4 = cos(thetaz);
  t5 = sin(thetaz);
  t8 = v*v;
  t9 = w*w;
  t6 = cos(t3);
  t7 = sin(t3);
  t10 = a1*t8;
  t11 = a2*t9;
  t12 = t10+t11;
  WMWt[0][0] = dt*t2*t4*t6*t12;
  WMWt[0][1] = dt*t2*t4*t7*t12;
  WMWt[1][0] = dt*t2*t5*t6*t12;
  WMWt[1][1] = dt*t2*t5*t7*t12;
  WMWt[5][5] = dt*t2*(a3*t8+a4*t9);


}

/**
   TODO
   Fill in the value of the measurement covariance matrix. The rows/columns of C
   are in the following order [POS_X POS_Y POS_Z ROT_R ROT_P ROT_Y ]
   \param[out] R Covariance matrix of the sensors.
   \param state_in    The current state estimate
*/
void meas_evaluate_R( double R[6][6], const State& state ){

  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      R[r][c] = 0.0;

  // TODO fill in the matrix R
  
}


/**
   TODO
   Evaluate the system function.
   Compute the process model.
   This function returns the prediction of the next state based on the 
   current state estimate and the commmand input (linear/angular velocities).
   \param state_in    The current state estimate
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
State sys_evaluate_g( const State& state_in, double v, double w, double dt ){

  State state_out;

  // TODO Given state_in and v and w and dt (time increment) determine the prior
  // estimate state_out
  double thetaz = state_in.x[5];
  state_out.x[0] = state_in.x[0] + dt*v*cos(thetaz); // linear velocity along x-axis
  state_out.x[1] = state_in.x[1] + dt*v*sin(thetaz);
  state_out.x[2] = state_in.x[2];
  state_out.x[3] = state_in.x[3];
  state_out.x[4] = state_in.x[4];
  state_out.x[5] = thetaz + dt*w; // angular velocity about z-axis
  return state_out;
}

/**
   TODO
   Evaluate the system Jacobian.
   This function evaluates the Jacobian of the system functions g (see 
   sys_evaluate_g). The entry G[i][j] represents ( d g_i / d s_j )
   \param[out] G      The 6x6 Jacobian of the function g
   \param state       The state of the robot
   \param v           The input linear velocity
   \param w           The input angular velocity
   \param dt          Delta time
*/
void sys_evaluate_G( double G[6][6], const State& state, double v, double w, double dt ){
  
  for( int r=0; r<6; r++ )
    for( int c=0; c<6; c++ )
      G[r][c] = 0.0;
  
  // TODO
  // Given state, v and w, compute the system Jacobian G
  /**double thetax = state.x[3];
  double thetay = state.x[4];
  G[0][0] = 1.0;
  G[0][4] = dt*v*cos(thetay);
  G[0][5] = -dt*v*sin(thetaz);
  G[1][1] = 1.0;
  G[1][3] = -dt*v*sin(thetax);
  G[1][5] = dt*v*cos(thetaz);
  G[2][2] = 1.0;
  G[2][3] = dt*v*cos(thetax);
  G[2][4] = -dt*v*sin(thetay);
  G[3][3] = 1.0;
  G[4][4] = 1.0;
  G[5][5] = 1.0;**/
  double thetaz = state.x[5];
  G[0][0] = 1.0;
  G[0][5] = -dt*v*sin(thetaz);
  G[1][1] = 1.0;
  G[1][5] = dt*v*cos(thetaz);
  G[2][2] = 1.0;
  G[3][3] = 1.0;
  G[4][4] = 1.0;
  G[5][5] = 1.0;

  
}

/**
   TODO
   Evaluate the GPS observation function.
   This function returns the expected satellite fix given the state of the robot
   \param state The state estimate
   \return      A satellite navigation fix (only the latitute, longitude
                and altitude members are used)
*/
sensor_msgs::NavSatFix meas_evaluate_gps( const State& state ){

  sensor_msgs::NavSatFix nsf;

  // TODO
  // Given prior estimate state, determine the expected GPS measurement nsf
  double latitude = -0.000010*state.x[0] + 35.859514;
  double longitude = -0.000008*state.x[1] - 108.236795;
  double altitude = 0.999627*state.x[2] + 13.998098;
  nsf[0] = latitude;
  nsf[1] = longitude;
  nsf[2] = altitude;
  
  return nsf;
}

/**
   TODO
   Evaluate the IMU observation function.
   This function computes the expected imu orientation given the state of the 
   robot.
   \param state_in The current state estimate
   \return         A inertial navigation unit measurement (only the orientation
                   member is used).
*/
sensor_msgs::RPY meas_evaluate_imu( const State& state ){
  sensor_msgs::RPY rpy;

  // TODO
  // Given the prior estimate state, determine the expected RPY measurement rpy  
  return rpy;
}

/** 
    TODO
    Observation Jacobian of the GPS
    This function returns the 3x3 observation Jacobian of the GPS. Essentially,
    this is the Jacobian of your meas_evaluate_gps function.
    \param[out] Hgps The 3x3 GPS Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Hgps( double Hgps[3][3], const State& state ){

  // TODO
  // Fill the Jacobian matrix Hgps of the GPS observations
}

/** 
    Observation Jacobian of the IMU
    This function returns the 3x3 observation Jacobian of the IMU. Essentially,
    this is the Jacobian of your meas_evaluate_imu function.
    \param[out] Himu The 3x3 IMU Jacobian.
    \param[in]  state The state of the robot
*/
void meas_evaluate_Himu( double Himu[3][3], const State& state ){

  // TODO
  // Fill the Jacobian matrix Himu of the IMU observations
}

