#include <iostream>
#include <Eigen/Dense>
#include "robots.h"
#include "car.h"
#include "localizer.h"
#include "beacon_sensor.h"
#include "eigen_io_util.h"
#include "osg_util.h"
#include <osgViewer/Viewer>
#include <osgViewer/View>
#include <osgViewer/CompositeViewer>
#include <osg/Group> 
#include <osg/Camera>
#include <osg/GraphicsContext>
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
#include "trajectory_util.h"
#include "camera_sensor.h"
#include "sensor_functions.h"

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
  Vector4d transparent(0.0,0.0,0.0,0.0);


  // intialize the robot
  int T = 20;
  int NX = 4 + 1*3;
  int NU = 2;
  int NB = NX*(NX+3)/2;
  int NS = 0;
  int NUM_TEST = 0;
  double rho_x = 0.1; 
  double rho_u = 0.1;
  int N_iter = 50;

  // hack for now
  MatrixXd W_cov = MatrixXd::Zero(NB,NB);
  EigenMultivariateNormal<double> sampler(VectorXd::Zero(NB), W_cov);
  
  VectorXd x0(4);
  x0 << 0.0, 0.0, 0.0, -0.3;
  Car cl(x0);
  VectorXd l_x0 = VectorXd::Zero(NX);
  l_x0.segment(0,4) = x0;
  Localizer c(&cl, 1);

  //initialize the sensors
  int NZ = 3; 
  Vector2d beacon_1_pos(-0.2,0.3); 
  BeaconSensor s1(beacon_1_pos);
  s1.draw(beacon_1_pos, brown, traj_group);
  Robot::SensorFunc s1_f = &CarBeaconFunc;
  cl.attach_sensor(&s1, s1_f);

  Vector2d beacon_2_pos(0.2,0.1); 
  BeaconSensor s2(beacon_2_pos);
  s2.draw(beacon_2_pos, brown, traj_group);
  Robot::SensorFunc s2_f = &CarBeaconFunc;
  c.attach_sensor(&s2, s2_f);

  Vector2d beacon_3_pos(-0.3,0.1); 
  BeaconSensor s3(beacon_3_pos);
  s3.draw(beacon_3_pos, brown, traj_group);
  Robot::SensorFunc s3_f = &CarBeaconFunc;
  c.attach_sensor(&s3, s3_f);
  
  Matrix4d camera_transform = Matrix4d::Identity();
  CameraSensor s4(1);
  s4.draw(camera_transform, brown, traj_group);
  camera_transform(0,3) = 0.2;
  s4.draw(camera_transform, red, traj_group);




//  //initilaize the initial distributions and noise models
//  MatrixXd Sigma_0 = 0.0001*MatrixXd::Identity(NX,NX);
//  Sigma_0(2,2) = 0.0000000001;
//  Sigma_0(3,3) = 0.000000001;
//  LLT<MatrixXd> lltSigma_0(Sigma_0);
//  MatrixXd rt_Sigma_0 = lltSigma_0.matrixL();
//
//  MatrixXd Q_t = MatrixXd::Zero(NX,NX);
//  Q_t(0,0) = 0.000001;
//  Q_t(1,1) = 0.000001;
//  Q_t(2,2) = 0.0000001;
//  Q_t(3,3) = 0.0000000001;
//  LLT<MatrixXd> lltQ_t(Q_t);
//  MatrixXd M_t = lltQ_t.matrixL();
//
//  MatrixXd R_t = 0.01*MatrixXd::Identity(NZ,NZ);
//  LLT<MatrixXd> lltR_t(R_t);
//  MatrixXd N_t = lltR_t.matrixL();
//
//  //Set car noise models
//  c.set_M(M_t);
//  c.set_N(N_t);
//
//  VectorXd b_0;
//  build_belief_state(l_x0, rt_Sigma_0, b_0);
//  vector<VectorXd> B_bar(T+1);
//  vector<VectorXd> U_bar(T);
//  vector<MatrixXd> W_bar(T);
//  B_bar[0] = b_0;
//
//  for (int t = 0; t < T; t++) {
//    VectorXd u = VectorXd::Random(2) / 3;
//    if (t > T/2) u = - 2 * u;
//    U_bar[t] = u;
//    MatrixXd rt_W_t;
//    c.belief_dynamics(B_bar[t], U_bar[t], B_bar[t+1]);
//    c.belief_noise(B_bar[t], U_bar[t], rt_W_t);
//    sampler.setCovar(rt_W_t * rt_W_t.transpose());
//    sampler.generateSamples(W_bar[t], NS);
//    //cout << W_bar[t] << endl;
//  }
//
//  cout <<"asdf" << endl;
//
//
//  vector<vector<VectorXd> > W_s_t;
//  index_by_sample(W_bar, W_s_t);
//  vector<VectorXd> test;
//  for (int s = 0; s < NS; s++) {
//	  c.forward_integrate(b_0, U_bar, W_s_t[s], test);
//	  render = c.draw_belief_trajectory(test, red, transparent, traj_group, z_offset/2 + 1e-4);
//  }
//
//
//  render = c.draw_belief_trajectory(B_bar, red, yellow, traj_group, z_offset/2 + 1e-4);
//
//  // setup for SCP
//  // Define a goal state
//  VectorXd b_goal = B_bar[T];
//  b_goal.segment(NX, NB-NX) = VectorXd::Zero(NB-NX); // trace
//  // Output variables
//  vector<VectorXd> opt_B, opt_U; // noiseless trajectory
//  MatrixXd Q; VectorXd r;  // control policy
//
//  cout << "calling scp" << endl;
//  scp_solver(c, B_bar, U_bar, W_bar, rho_x, rho_u, b_goal, N_iter,
//      opt_B, opt_U, Q, r);
//
//  TrajectoryInfo opt_traj(b_0);
//    for (int t = 0; t < T; t++) {
//  	  //opt_traj.add_and_integrate(opt_U[t], VectorXd::Zero(NX), c);
//  	  VectorXd feedback = opt_traj.Q_feedback(c);
//  	  VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
//  	  opt_traj.add_and_integrate(u_policy, VectorXd::Zero(NB), c);
//    }
//    c.draw_belief_trajectory(opt_traj._X, blue, orange, traj_group, z_offset/2+1e-4);
//
//    for (int s = 0; s < NUM_TEST; s++) {
//  	  TrajectoryInfo test_traj(b_0);
//  	  for (int t = 0; t < T; t++) {
//  		  VectorXd feedback = test_traj.Q_feedback(c);
//  	  	  VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
//  		  test_traj.add_and_integrate(u_policy, W_bar[t].col(s), c);
//  		  //test_traj.add_and_integrate(u_policy, sampler.nextSample(), c);
//  	  }
//  	  c.draw_belief_trajectory(test_traj._X, blue, transparent, traj_group, z_offset/2+1e-4);
//    }


  // visualize
  osg::Geode * fgeode = new osg::Geode; 
  osg::ShapeDrawable *floor = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0,0.0,0.0), 1.0, 1.0, z_offset));
  floor->setColor(osg::Vec4(0.1,0.1,0.1,1.0));
  fgeode->addDrawable(floor);
  root->addChild(fgeode);

  // create a context that spans the entire x screen
  int width = 800;
  int height = 800;
  osg::ref_ptr<osgViewer::CompositeViewer> compositeViewer = new osgViewer::CompositeViewer;
  osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;

  traits->x = 0;
  traits->y = 0;
  traits->width = width;
  traits->height = height;
  traits->windowDecoration = false;
  traits->doubleBuffer = true;
  traits->sharedContext = 0;
  traits->overrideRedirect = true;
  osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
  osgViewer::View* v0 = new osgViewer::View();
  v0->setSceneData(root);
//  v0->setUpViewInWindow(0,0,width,height);
  osg::ref_ptr<osg::Camera> cam = v0->getCamera();
  cam->setGraphicsContext(gc.get());
  cam->setViewport(0, 0, width/2, height);
  compositeViewer->addView(v0);

  //second screen
  osgViewer::View* v1 = new osgViewer::View();
  v1->setSceneData(root);
//  v1->setUpViewInWindow(100,100,width,height);
  osg::ref_ptr<osg::Camera> cam2 = v1->getCamera();
  //cam2->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
  osg::Matrixd myCameraMatrix;

  osg::Matrixd cameraRotation;
  osg::Matrixd cameraTrans;
  cameraRotation.makeRotate(
     osg::DegreesToRadians(-20.0), osg::Vec3(0,1,0), // roll
     osg::DegreesToRadians(-15.0), osg::Vec3(1,0,0) , // pitch
     osg::DegreesToRadians( 10.0), osg::Vec3(0,0,1) ); // heading

  cameraTrans.makeTranslate( 0.5,-0.1,0.2 );

  myCameraMatrix = cameraRotation * cameraTrans;
  cam2->setViewMatrix(myCameraMatrix);
  cam2->setGraphicsContext(gc.get());
  cam2->setViewport(width/2, 0, width/2, height);
  v1->setCamera(cam2);
  compositeViewer->addView(v1);

  return compositeViewer->run();
}
