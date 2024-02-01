#include <math.h> // erf

/**
  Correct range: p_hit
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input z_max: maximum sensor range
  \input: sigma_hit: sensor standart deviation
  \return phit
*/
double p_hit( double r, double rs, double z_max, double sigma_hit ){
  // TODO
  double phit = 0.0;
  double nhit = pow(erf(z_max),-1);
  double phi_z = 0.5 * (1 + erf((z_max-rs)/(sqrt(2)*sigma_hit))); // eqn. 6
  double phi_0 = 0.5 * (1 + erf((-rs)/(sqrt(2)*sigma_hit))); // eqn. 6
  if (r >= 0 && r <= z_max) {
    phit = nhit * (phi_z - phi_0);
  }
  return phit;
}

/**
  Unexpected object: p_short
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input lambda_short: exponential parameter
  \return p_short
*/
double p_short( double r, double rs, double z_max, double lambda_short ){
  // TODO
  double pshort=0.0;
  double nshort = 1 / (1 - exp(-lambda_short * rs));
  if (r >= 0 && r <= rs) {
    pshort = nshort * lambda_short * exp(-lambda_short*r);
  }
  return pshort;
}

/**
  Failure: p_max
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input z_max: maximum sensor range
  \return p_max
 */
double p_max( double r, double rs, double z_max ){
  double pmax=0.0;
  if (r == z_max) {
    pmax = 1.0;
  }
  return pmax;
}  

/**
   Random measurement: p_rand
  \input r: the measured range of a laser beam
  \input rs: the true range of the beam
  \input z_max: maximum sensor range
  \return p_rand
*/
double p_rand( double r, double rs, double z_max ){
  double prand=0;
  if (r >= 0 && r <= z_max) {
    prand = 1/z_max;
  }
  return prand;
}

/**
   beam model
   Implement the beam model for a laser range finder. The function
   returns the probability P( z | s, m ), that is the probability that a 
   measurement z was caused by a robot in a state s and in a map m.
   \input Map* m:  The map of the environment the robot.
   \input Scan z: A vector of laser beams. Each beam has two members: range and 
                  bearing. You can access the range if the ith beam with
		  z[i].range and the bearing of the ith beam with z[i].bearing.
   \input State s: The state of the robot with respect to the world frame.
                   The x coordinate is given by s[0], the y coordinate by s[1]
		   and the angle theta by s[2].
   \input Pose laser_pose: The position and orientation of the laser with 
                           respect to the robot's coordinate frame. The x, y
			   and angle are given by coordinate is laser_pose[0]
			   laser_pose[1] and laser_pose[2] respectively.
   \input sigma_hit: The standard variation of errors of a beam
   \input lambda_short: The p_short exponential parameter
   \input z_max:  The laser maximum range (in meters)
   \input w_hit:  The coefficient of measurements
   \input w_short:The coefficient of measurements
   \input w_max:  The coefficient of measurements
   \input w_rand: The coefficient of random errors
   \return        The probability p( z | s, m )
*/
double beam_model( Map* map,
		   Scan& z,
		   State s, 
		   Pose laser_pose, 
		   double sigma_hit, 
		   double lambda_short,
		   double z_max, 
		   double w_hit,
		   double w_short,
		   double w_max,
		   double w_rand ){
  // TODO
  double p = 0.0; // 1.0
  double phit = 0.0;
  double pshort = 0.0;
  double pmax = 0.0;
  double prand = 0.0;
  double temp_p = 0.0;
  double x = s[0] + laser_pose[0]*cos(s[2]) - laser_pose[1]*sin(s[2]); // check on MATLAB
  double y = s[1] + laser_pose[1]*sin(s[2]) + laser_pose[0]*sin(s[2]); // ^^
  for (int i = 0; i < 720; i+= 10) {
    double r = z[i].range;
    double bearing = s[2] + laser_pose[2] + z[i].bearing;
    double rs = GetRangeFromMap(map, x, y, bearing, z_max);
    phit = p_hit(r, rs, z_max, sigma_hit);
    pshort = p_short(r, rs, z_max, lambda_short);
    pmax = p_max(r, rs, z_max);
    prand = p_rand(r, rs, z_max);
    temp_p = w_hit*phit + w_short*pshort + w_max*pmax +  w_rand*prand;
    /**if (temp_p > p) {
      p = temp_p;
      }**/
    p += temp_p;
  }
  return p;
}
