#include <iostream>
#include <Eigen/Dense>
#include "robots.h"
#include "car.h"
#include "beacon_sensor.h"
#include "eigen_io_util.h"
#include "osg_util.h"
#include <osgViewer/Viewer>
#include <osg/Group> 
#include <osg/Geode>
#include <osg/StateSet>
#include <osg/Math>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "kalman_filter.h"
#include "scp_solver.h"
#include "timer.h"
#include "eigen_multivariate_normal.h"

using namespace Eigen;
using namespace std;
//#include "sensor_functions.h"
VectorXd testin(const VectorXd& r) {
  return r.segment(0,2);  
}

//typedef VectorXd (*SensorFunc2)(const VectorXd&);

int main()
{
  // initialize random number generator
  srand(time(NULL));

  // initialize plotting
  osg::Group* root = new osg::Group();
  osg::Group* traj_group = new osg::Group();
  root->addChild(traj_group);

  init_transparency_group(traj_group);
  vector<osg::Node*> render;
  double z_offset = 0.01;
  Vector4d yellow(1.0,1.0,0.1,0.4);  
  Vector4d red(1.0, 0.0, 0.0, 0.8); 
  Vector4d blue(0.2, 0.2, 1.0, 0.8);
  Vector4d orange(1.0, 0.55, 0.0, 0.8);
  Vector4d white(1.0, 1.0, 1.0, 0.1); 
  Vector4d brown(139/255.0,69/255.0,19/255.0,0.8);


  // intialize the robot
  int T = 20;
  int NX = 4;
  int NU = 2;
  int NB = NX*(NX+3)/2;
  int NS = 0;
  double rho_x = 0.1; 
  double rho_u = 0.1;
  int N_iter = 100;

  // hack for now
  MatrixXd W_cov = 0.0000*MatrixXd::Identity(NB,NB);
  EigenMultivariateNormal<double> sampler(VectorXd::Zero(NB), W_cov);
  
  VectorXd x0(NX);
  x0 << 0.0, 0.0, 0.0, -0.3;
  Car c(x0);
  c.greet();

  //initialize the sensors
  int NZ = 3; 
  Vector2d beacon_1_pos(-0.2,0.3); 
  BeaconSensor s1(beacon_1_pos);
  s1.draw(beacon_1_pos, brown, traj_group);
  SensorFunc s1_f = &CarBeaconFunc;
  c.attach_sensor(&s1, s1_f);

  Vector2d beacon_2_pos(0.2,0.1); 
  BeaconSensor s2(beacon_2_pos);
  s2.draw(beacon_2_pos, brown, traj_group);
  SensorFunc s2_f = &CarBeaconFunc;
  c.attach_sensor(&s2, s2_f);

  Vector2d beacon_3_pos(-0.3,0.1); 
  BeaconSensor s3(beacon_3_pos);
  s3.draw(beacon_3_pos, brown, traj_group);
  SensorFunc s3_f = &CarBeaconFunc;
  c.attach_sensor(&s3, s3_f);
  




  //initilaize the initial distributions and noise models
  MatrixXd Sigma_0 = 0.0001*MatrixXd::Identity(NX,NX);
  Sigma_0(2,2) = 0.0000000001;
  Sigma_0(3,3) = 0.000000001;
  LLT<MatrixXd> lltSigma_0(Sigma_0);
  MatrixXd rt_Sigma_0 = lltSigma_0.matrixL();

  MatrixXd Q_t = 0.0001*MatrixXd::Identity(NX,NX);
  Q_t(2,2) = 0.0000001;
  Q_t(3,3) = 0.0000000001;
  LLT<MatrixXd> lltQ_t(Q_t);
  MatrixXd M_t = lltQ_t.matrixL(); 

  MatrixXd R_t = 0.0001*MatrixXd::Identity(NZ,NZ);
  LLT<MatrixXd> lltR_t(R_t);
  MatrixXd N_t = lltR_t.matrixL();

  //Set car noise models
  c.set_M(M_t);
  c.set_N(N_t); 

  VectorXd b_0;
  build_belief_state(x0, rt_Sigma_0, b_0);
  vector<VectorXd> B_bar(T+1);
  vector<VectorXd> U_bar(T);
  vector<MatrixXd> W_bar(T); 
  B_bar[0] = b_0; 


  //osg::Node *ellipsoid = drawEllipsoid2D(x0.segment(0,2), z_offset/2+1e-4, Sigma_0.block(0,0,2,2), yellow);
  //traj_group->addChild(ellipsoid);
  for (int t = 0; t < T; t++) {
    VectorXd u = VectorXd::Random(2) / 3;
    if (t > T/2) u = - 2 * u;  
    U_bar[t] = u;
    c.belief_dynamics(B_bar[t], U_bar[t], B_bar[t+1]); 
    VectorXd x_t; MatrixXd rt_Sigma_t; 
    parse_belief_state(B_bar[t+1], x_t, rt_Sigma_t); 
    MatrixXd Sigma_t = rt_Sigma_t * rt_Sigma_t.transpose();
    sampler.generateSamples(W_bar[t], NS);

  }

  render = c.draw_belief_trajectory(B_bar, red, yellow, traj_group, z_offset/2 + 1e-4);

  // setup for SCP
  // Define a goal state
  VectorXd b_goal = B_bar[T];
  b_goal.segment(NX, NB-NX) = VectorXd::Zero(NB-NX); // trace
  // Output variables
  vector<VectorXd> opt_B, opt_U; // noiseless trajectory
  MatrixXd K; VectorXd u0;  // control policy 

  cout << "calling scp" << endl;
  scp_solver(c, B_bar, U_bar, W_bar, rho_x, rho_u, b_goal, N_iter,
      opt_B, opt_U, K, u0);

  cout << "scp done" << endl;
  VectorXd B_bar_T = B_bar[T].segment(NX, NB-NX);
  VectorXd opt_B_T = opt_B[T].segment(NX, NB-NX); 
  cout << "Input trajectory final trace:" << B_bar_T.transpose() * B_bar_T << endl; 
  cout << "Output trajectory final trace: " << opt_B_T.transpose() * opt_B_T << endl;

  c.draw_belief_trajectory(opt_B, blue, orange, traj_group, z_offset/2+1e-4); 
  
  // visualize
  osg::Geode * fgeode = new osg::Geode; 
  //osg::StateSet *fstateSet = new osg::StateSet;
  //fgeode->setStateSet(fstateSet);
  osg::ShapeDrawable *floor = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0,0.0,0.0), 1.0, 1.0, z_offset));
  floor->setColor(osg::Vec4(0.1,0.1,0.1,1.0));
  fgeode->addDrawable(floor);

  osgViewer::Viewer viewer;
  root->addChild(fgeode); 
  viewer.setSceneData(root);
  //viewer.setUpViewInWindow(400, 400, 640, 480);
  return viewer.run();

}
