#include "assignment3_context.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <math.h> // added for M_PI


// utility function to test for a collision
bool CS436Context::state_collides( const vertex& q ) const {
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q );

  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
}

// utility function to interpolate between two configurations
CS436Context::vertex CS436Context::interpolate( const CS436Context::vertex& qA,
						const CS436Context::vertex& qB,
						double t ) const {
  CS436Context::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;
}

CS436Context::CS436Context( const robot_model::RobotModelConstPtr& robotmodel,
			    const std::string& name, 
			    const std::string& group, 
			    const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

CS436Context::~CS436Context(){}


// TODO -- i think done
CS436Context::vertex CS436Context::random_sample( const CS436Context::vertex& q_goal ) const {  
  CS436Context::vertex q_rand(q_goal.size(), 0.0); // initialize
  for (int i = 0; i < 6; i ++) {
    q_rand[i] = ((double)rand() / (RAND_MAX/2) - 1) * M_PI; // randnumgen [-pi, pi]
  }
  if (CS436Context::state_collides(q_rand) == true) { // test for collision
    return CS436Context::random_sample(q_goal);
  }
  return interpolate(q_rand, q_goal, (rand() % 2)); // goal bias
}

// TODO -- wip
double CS436Context::distance( const CS436Context::vertex& q1, const CS436Context::vertex& q2 ){
  double sum=0; // initialize
  
  // map coordinates for both q's
  double *p1 = coordinates(q1);
  double *p2 = coordinates(q2);
  
  // calculate distance between coordinates
  for (int i = 0; i < 3; i++) {
    sum += (p2[i] - p1[i])^2;
  }
  double d = sqrt(sum);
  return d;
}

// TODO -- i think done
CS436Context::vertex CS436Context::nearest_configuration(const CS436Context::vertex& q_rand, const std::vector<Node> tree){
  
  CS436Context::vertex q_near; // initialize
  double minDist = 100.0; // set it absurdly high
  double distance;
  
  // iterate over tree
  for (std::vector<Node>::iterator it = tree.begin(); it != tree.end(); ++it) {
    distance = CS436Context::distance(*it, q_rand); // calculate current node's distance
    if (distance < minDist) { // if nearer than previous nodes
      minDist = distance; // update min values
      q_near = (*it)->q; // might be wrong notation
    }
  }
  
  return q_near;
}

// TODO -- i think done
bool CS436Context::is_subpath_collision_free( const CS436Context::vertex& q_near,
					      const CS436Context::vertex& q_new ){
  bool collision_free = false;
  CS436Context::vertex q_step(q_near.size(), 0.0);
  
  // calculate distance between q's
  double dist = CS436Context::distance(q_near, q_new);

  // calculate step sizes for each angle
  int numSteps = (int) (dist/0.1);
  double steps[6];
  for (int i = 0; i < 6; i++) {
    step[i] = (q_new[i] - q_near[i]) / numSteps;
  }
  
  // check for collisions along each step
  for (double d = 0; d < dist; d += 0.1) {
    for (int i = 0; i < 6; i++) {
      q_step[i] = q_near[i] + steps[i];
    }
    if (CS436Context::state_collides(q_step) == true) {
      return collision_free; // false
    }
  }

  // if no collisions along path
  collision_free = true; 
  return collision_free;
}


// TODO -- i think done
CS436Context::path CS436Context::search_path( const CS436Context::vertex& q_init,
					      const CS436Context::vertex& q_goal,
					      const std::vector<Node> tree){
  CS436Context::path P; // init -> goal
  CS436Context::path reverseP; // goal -> init

  Node *curr = tree.back(); // start at goal
  while (curr != NULL) { // until root is reached
    // traverse backwards through tree
    reverseP.push_back(curr->q);
    curr = curr->parent;
  }
  
  // iterate over reverseP
  for (int i = 0; i < reverseP.size(); i++) {
    P.push_back = reverseP.pop();
  }
  
  return P;
}

// TODO -- i think done
CS436Context::path CS436Context::rrt( const CS436Context::vertex& q_init,
				      const CS436Context::vertex& q_goal ){
  CS436Context::path P;
  P.push_back(q_init); // add starting pt. to path
  
  // check for straight path
  if (is_subpath_collision_free(q_init, q_goal)) {
    P.push_back(q_goal);
    return P;
  }
  
  // initialize tree
  Node init = Node(q_init, NULL);
  std::vector<Node> tree;
  tree.push_back(init);

  Node parent = init;
  while(tree.back() != q_goal) { // while q_goal is not in the tree
    parent = rrt_helper(parent, q_goal, tree); // hopefully no errors w this
  }

  // check for solution
  P = CS436Context::search_path(q_init, q_goal, tree);
  return P;
}

// This is the method that is called each time a plan is requested
bool CS436Context::solve( planning_interface::MotionPlanResponse &res ){

  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, getGroupName()));
  res.trajectory_->clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  vertex q_init, q_goal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    q_init.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  ros::Time begin = ros::Time::now();

  path P = rrt( q_init, q_goal );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q_init );

  for( std::size_t i=P.size()-1; i>=1; i-- ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i], P[i-1], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }
  
  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
  
}

bool CS436Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ return true; }

void CS436Context::clear(){}

bool CS436Context::terminate(){return true;}

// helper function
double *coordinates(const CS436Context::vertex& q) {
  double coors[3] = {0,0,0};
  double E[4][4];
  // calculate E
  double t2 = cos(q[0]);
  double t3 = cos(q[1]);
  double t4 = cos(q[2]);
  double t5 = cos(q[3]);
  double t6 = cos(q[4]);
  double t7 = cos(q[5]);
  double t8 = sin(q[0]);
  double t9 = sin(q[1]);
  double t10 = sin(q[2]);
  double t11 = sin(q[3]);
  double t12 = sin(q[4]);
  double t13 = sin(q[5]);
  double t14 = t3*t4;
  double t15 = t3*t10;
  double t16 = t4*t9;
  double t17 = t2*t12;
  double t18 = t9*t10;
  double t19 = t8*t12;
  double t20 = t8*t18;
  double t21 = -t18;
  double t22 = t2*t14;
  double t23 = t2*t15;
  double t24 = t2*t16;
  double t25 = t8*t14;
  double t26 = t2*t18;
  double t27 = t8*t15;
  double t28 = t8*t16;
  double t31 = t15+t16;
  double t29 = -t25;
  double t30 = t2*t21;
  double t32 = t14+t21;
  double t33 = t5*t31;
  double t34 = t11*t31;
  double t35 = t23+t24;
  double t36 = t27+t28;
  double t37 = t5*t32;
  double t38 = t11*t32;
  double t39 = -t34;
  double t40 = t22+t30;
  double t41 = t20+t29;
  double t42 = t5*t35;
  double t43 = t11*t35;
  double t44 = t5*t36;
  double t45 = t11*t36;
  double t46 = t5*t40;
  double t47 = t11*t40;
  double t48 = -t43;
  double t49 = t5*t41;
  double t50 = t11*t41;
  double t52 = t33+t38;
  double t53 = t37+t39;
  double t51 = -t50;
  double t54 = t42+t47;
  double t55 = t45+t49;
  double t56 = t46+t48;
  double t59 = -t6*(t43-t46);
  double t57 = t44+t51;
  double t58 = t6*t55;
  double t61 = t19+t59;
  double t60 = t17+t58;
  E[0][0] = -t7*t54-t13*t61;
  E[0][1] = t13*t54-t7*t61;
  E[0][2] = -t6*t8-t12*(t43-t46);
  E[0][3] = t8*(-1.0915E-1)+t22*3.9225E-1-t26*3.9225E-1-t42*9.465E-2-t47*9.465E-2+t2*t3*(1.7E+1/4.0E+1)-t6*t8*2.2161E-1-t12*(t43-t46)*2.2161E-1;
  E[1][0] = -t7*t57+t13*t60;
  E[1][1] = t7*t60+t13*t57;
  E[1][2] = t2*t6-t12*t55;
  E[1][3] = t2*1.0915E-1-t20*3.9225E-1+t25*3.9225E-1-t44*9.465E-2+t50*9.465E-2+t2*t6*2.2161E-1+t3*t8*(1.7E+1/4.0E+1)-t12*t55*2.2161E-1;
  E[2][0] = t7*(t34-t37)+t6*t13*t52;
  E[2][1] = -t13*(t34-t37)+t6*t7*t52;
  E[2][2] = -t12*t52;
  E[2][3] = t9*(-1.7E+1/4.0E+1)-t15*3.9225E-1-t16*3.9225E-1+t34*9.465E-2-t37*9.465E-2-t12*t52*2.2161E-1+8.9159E-2;
  E[3][3] = 1.0;
  
  return *E;
}

/**
double *rotate(const CS436::Context::vertex& q, const std::string axis) {
  if (axis == 'x') {
    double *R[3][3]  = {{1, 0, 0}, {0, cos(q), -sin(q)}, {0, sin(q), cos(q)}};
  } else if (axis == 'y') {
    double *R[3][3] = {{cos(q), 0, sin(q)}, {0, 1, 0}, {-sin(q), 0, cos(q)}};
  } else {
    double *R[3][3] = {{cos(q), -sin(q), 0}, {sin(q), cos(q), 0}, {0, 0, 1}};
  }
  return R;
}

double *frame(double R[3][3], double t[3]) {
  double *E[4][4] = {{R[0][0], R[0][1], R[0][2], t[0]}, {R[1][0], R[1][1], R[1][2], t[1]}, {R[2][0], R[2][1], R[2][2], t[2]}, {0,0,0,1}};
}

void multiply(int arr1[4][4], int arr2[4][4], int arr3[4][4]){
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            arr3[i][j] = 0;
            for(int k = 0; k < 4; k++){
                arr3[i][j] += arr1[i][k] * arr2[k][j];
            }
        }
    }
}
**/

Node rrt_helper(const Node parent, const CS436Context::vertex& q_goal, std::vector<Node> tree) {
  // select configuration from tree
  CS436Context::vertex q_rand = CS436Context::random_sample(q_goal);

  // add new branch to tree
  if (CS436Context::is_subpath_collision_free(parent->q, q_rand) == true) {
    Node rand = Node(q_rand, parent);
    tree.push_back(rand);
    return rand;
  }
}
