#include <math.h>
#include "amcl/sensors/amcl_odom.h"
#include <stdlib.h>
#include <iostream>
using namespace amcl;

/**
   TODO
   Implement a sample motion odometry
   \input sample: The pose of a sample. sample[0] is the x coordinate
   sample[1] is the y coordinate and sample[2] is the angle.
   \output sample: The pose P(x_t+1 | x_t, u_t )
   \input delta: The pose increment to be adde to the pose (in world frame)
   \input alpha1:
   \input alpha2:
   \input alpha3:
   \input alpha4:
   \input alpha5:
   \input alpha6:
*/
void odometry_model( double sample[3],
		     double delta[3],
		     double sigma1,
		     double sigma2,
		     double sigma3,
		     double sigma4 ){
  double alpha = atan2(delta[1], delta[0]) - sample[2];
  double beta = delta[2] - alpha;
  double d = sqrt(pow(delta[0],2) + pow(delta[1],2));
  double aprime = alpha + alpha * pf_ran_gaussian(sigma1) + d * pf_ran_gaussian(sigma2);
  double bprime = beta + beta * pf_ran_gaussian(sigma1) + d * pf_ran_gaussian(sigma2);
  double dprime = d + d * pf_ran_gaussian(sigma3) + (alpha + beta) * pf_ran_gaussian(sigma4);
  sample[0] = sample[0] + dprime * cos(sample[2] + aprime);
  sample[1] = sample[1] + dprime * sin(sample[2] + aprime);
  sample[2] = sample[2] + aprime + bprime;
}

