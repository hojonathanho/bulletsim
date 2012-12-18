#include <iostream>
#include <Eigen/Dense>
#include "robots.h"
#include "car.h"
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
#include <osgDB/Export>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "kalman_filter.h"
#include "scp_solver.h"
#include "ilqr_solver.h"
#include "timer.h"
#include "eigen_multivariate_normal.h"
#include "trajectory_util.h"
#include "sensor_functions/CarBeaconFunc.h"
#include "camera_sensor.h"

using namespace Eigen;
using namespace std;


static VectorXd _goal_offset;

//VectorXd GoalFn(Robot &r, const VectorXd& x) {
//	VectorXd ret(x.rows()-2);
//	ret.segment(0,2) = 10*(x.segment(0,2) - _goal_offset.segment(0,2));
//	ret.segment(2,ret.rows()-2) = x.segment(4,x.rows()-4);
//	return  ret;
//}

VectorXd GoalFn(Robot &r, const VectorXd& b) {
	return b - _goal_offset;
}

int main()
{
  // initialize random number generator
  srand(2929);

  // initialize plotting
  osg::Group* root = new osg::Group();
  osg::Group* traj_group = new osg::Group();
  root->addChild(traj_group);
  init_transparency_group(traj_group);
  vector<osg::Node*> render;
  double z_offset = 0.01;

  Vector4d yellow(1.0,1.0,0.1,0.8);
  Vector4d red(1.0, 0.0, 0.0, 0.8); 
  Vector4d blue(0.2, 0.2, 1.0, 0.8);
  Vector4d green(0.2,1.0,0.2,0.8);
  Vector4d orange(1.0, 0.55, 0.0, 0.8);
  Vector4d purple(1.0, 0.1, 1.0, 0.8);
  Vector4d white(1.0, 1.0, 1.0, 0.5);
  Vector4d brown(139/255.0,69/255.0,19/255.0,0.8);
  Vector4d transparent(0.0,0.0,0.0,0.0);


  // intialize the robot
  int T = 20;
  int NX = 4;
  int NU = 2;
  int NB = NX*(NX+3)/2;
  int NS = 10;
  int NUM_TEST = 20;
  double rho_x = 0.1;
  double rho_u = 0.1;
  int N_iter = 50;

  // hack for now
  MatrixXd W_cov = MatrixXd::Identity(NB,NB);
  EigenMultivariateNormal<double> sampler(VectorXd::Zero(NB), W_cov);
  
  VectorXd x0(NX);
  x0 << 0.2, -0.1, 0.0, -0.3;
  Car c(x0);
  c.greet();


  //initialize the sensors
  int NZ = 3; 
  Vector2d beacon_1_pos(-0.2,0.3); 
  BeaconSensor s1(beacon_1_pos, 75);
  s1.draw(beacon_1_pos, white, traj_group);
  Robot::SensorFunc s1_f = &CarBeaconFunc;
  c.attach_sensor(&s1, s1_f);

  Vector2d beacon_2_pos(0.2,0.4);
  BeaconSensor s2(beacon_2_pos, 75);
  s2.draw(beacon_2_pos, white, traj_group);
  Robot::SensorFunc s2_f = &CarBeaconFunc;
  c.attach_sensor(&s2, s2_f);

  Vector2d beacon_3_pos(-0.3,0.1); 
  BeaconSensor s3(beacon_3_pos, 75);
  s3.draw(beacon_3_pos, white, traj_group);
  Robot::SensorFunc s3_f = &CarBeaconFunc;
  c.attach_sensor(&s3, s3_f);
  

  //initilaize the initial distributions and noise models
  MatrixXd Sigma_0 = 0.0005*MatrixXd::Identity(NX,NX);
  Sigma_0(2,2) = 0.0000000001;
  Sigma_0(3,3) = 0.000000001;
  LLT<MatrixXd> lltSigma_0(Sigma_0);
  MatrixXd rt_Sigma_0 = lltSigma_0.matrixL();

  MatrixXd Q_t = 0.000001*MatrixXd::Identity(NX,NX);
  Q_t(2,2) = 0.0000001;
  Q_t(3,3) = 0.0000000001;
  LLT<MatrixXd> lltQ_t(Q_t);
  MatrixXd M_t = lltQ_t.matrixL(); 

  MatrixXd R_t = 0.05*MatrixXd::Identity(NZ,NZ);
  LLT<MatrixXd> lltR_t(R_t);
  MatrixXd N_t = lltR_t.matrixL();
  s1.setN(N_t(0,0));
  s2.setN(N_t(1,1));
  s3.setN(N_t(2,2));

  //Set car noise models
  c.set_M(M_t);

  VectorXd b_0;
  build_belief_state(x0, rt_Sigma_0, b_0);
  vector<VectorXd> B_bar(T+1);
  vector<VectorXd> U_bar(T);
  vector<MatrixXd> W_bar(T); 
  B_bar[0] = b_0; 

  for (int t = 0; t < T; t++) {
    VectorXd u = 0.5*VectorXd::Random(2) / 3;
    if (t > T/2) u = - u;
    U_bar[t] = u;
    MatrixXd rt_W_t;
    c.belief_dynamics(B_bar[t], U_bar[t], B_bar[t+1]);
    c.belief_noise(B_bar[t], U_bar[t], rt_W_t);
    sampler.generateSamples(W_bar[t], NS);
  }

  vector<vector<VectorXd> > W_s_t;
  index_by_sample(W_bar, W_s_t);
//  vector<VectorXd> test;
//  for (int s = 0; s < NS; s++) { THIS BE BROKEN
//	  c.forward_integrate(b_0, U_bar, W_s_t[s], test);
//	  render = c.draw_belief_trajectory(test, red, transparent, traj_group, 1e-4);
//  }



  // setup for SCP
  // Define a goal state
  VectorXd b_goal = B_bar[T];
  b_goal.segment(NX, NB-NX) = VectorXd::Zero(NB-NX); // trace
  _goal_offset = b_goal;
  // Output variables
  vector<VectorXd> opt_B_scp, opt_U_scp; // noiseless trajectory
  MatrixXd Q_scp; VectorXd r_scp;  // control policy

  cout << "calling scp" << endl;
  scp_solver(c, B_bar, U_bar, W_bar, rho_x, rho_u, &GoalFn, NULL, N_iter,
      opt_B_scp, opt_U_scp, Q_scp, r_scp);


  vector<VectorXd> opt_B_scp_c, opt_U_scp_c; // noiseless trajectory
  MatrixXd Q_scp_c; VectorXd r_scp_c;  // control policy
  c.setUConstraints(Vector2d(0.3,0.1), Vector2d(-0.3,-0.1));

  cout << "calling scp" << endl;
  scp_solver(c, B_bar, U_bar, W_bar, rho_x, rho_u, &GoalFn, NULL, 1,
      opt_B_scp_c, opt_U_scp_c, Q_scp_c, r_scp_c);

  vector<VectorXd> opt_B_ilqr, opt_U_ilqr; // noiseless trajectory
  vector<MatrixXd> K_ilqr; vector<VectorXd> u0_ilqr;  // control policy
  cout << "calling ilqr" << endl;
  ilqr_solver(c, B_bar, U_bar, W_bar, rho_x, rho_u, b_goal.segment(0,NX), 1,
      opt_B_ilqr, opt_U_ilqr, K_ilqr, u0_ilqr);

  //c.draw_belief_trajectory(B_bar, red, yellow, traj_group,  z_offset/2 +1e-4);
  //c.draw_belief_trajectory(opt_B_scp, orange, red, traj_group, z_offset/2 +1e-4);
  //c.draw_belief_trajectory(opt_B_scp_c, blue, orange, traj_group, z_offset/2 +1e-4);
  //c.draw_belief_trajectory(opt_B_ilqr, yellow, green, traj_group, z_offset/2 + 1e-4);

  //cout << Q << endl;

  MatrixXd Q = Q_scp;
  VectorXd r = r_scp;
  TrajectoryInfo opt_traj(b_0, &GoalFn, NULL);
    for (int t = 0; t < T; t++) {
  	  //opt_traj.add_and_integrate(opt_U[t], VectorXd::Zero(NB), c);
  	  VectorXd feedback = opt_traj.Q_feedback(c);
  	  VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
  	  opt_traj.add_and_integrate(u_policy, VectorXd::Zero(NB), c);
    }
//    c.draw_belief_trajectory(opt_traj._X, blue, orange, traj_group, z_offset/2+1e-4);
//    for (int i = 0; i < T; i++)
//    	cout << opt_traj._X[i].transpose() << endl;

    for (int s = 0; s < NUM_TEST; s++) {
  	  TrajectoryInfo test_traj(b_0, &GoalFn, NULL);
  	  for (int t = 0; t < T; t++) {
  		  VectorXd feedback = test_traj.Q_feedback(c);
  	  	  VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
		  MatrixXd rt_W_t;
		  c.belief_noise(test_traj._X[t], u_policy, rt_W_t);
		  sampler.setCovar(rt_W_t * rt_W_t.transpose());
  		  //test_traj.add_and_integrate(u_policy, W_bar[t].col(s), c);
  		  test_traj.add_and_integrate(u_policy, sampler.nextSample(), c);
  	  }
  	  c.draw_belief_trajectory(test_traj._X, red, transparent, traj_group, z_offset/2+1e-4);
    }

//  cout << "scp done" << endl;
//  VectorXd B_bar_T = B_bar[T].segment(NX, NB-NX);
//  VectorXd opt_B_T = opt_B[T].segment(NX, NB-NX);
//  cout << "Input trajectory final trace:" << B_bar_T.transpose() * B_bar_T << endl;
//  cout << "Output trajectory final trace: " << opt_B_T.transpose() * opt_B_T << endl;
//
//  c.draw_belief_trajectory(opt_B, blue, orange, traj_group, z_offset/2+1e-4);
//
//  //cout << K << endl;
//
//  for (int s = 0; s < NS; s++) {
//    VectorXd b = b_0;
//    for (int t = 0; t < T; t++) {
//      VectorXd u_policy = K.block(NU*t,0,NU,NB*(t+1)) * b + u0.segment(t*NU, NU);
//      VectorXd bt1;
//      c.belief_dynamics(b.segment(t*NB, NB), u_policy, bt1);
//      bt1 += W_bar[t].col(s);
//      //cout << "noise" << endl;
//      //cout << W_bar[t].col(s).transpose() << endl;
//      //xt1 += sampler.nextSample();
//      VectorXd tmp(NB*(t+2));
//      tmp.segment(0,NB*(t+1)) = b;
//      tmp.segment(NB*(t+1), NB) = bt1;
//      b = tmp;
//      //cout << bt1.transpose() << endl;
//      c.draw_belief(bt1, blue, orange, traj_group, z_offset/2 + 1e-4);
//    }
//  }

  // visualize
  osg::Geode * fgeode = new osg::Geode; 
  osg::ShapeDrawable *floor = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0,0.0,-z_offset/2), 1.0, 1.0, z_offset));
  floor->setColor(osg::Vec4(0.1,0.1,0.1,1.0));
  fgeode->addDrawable(floor);
  root->addChild(fgeode);

  // create a context that spans the entire x screen
  int width = 800;
  int height = 800;
  osg::ref_ptr<osgViewer::CompositeViewer> compositeViewer = new osgViewer::CompositeViewer;

  osgViewer::View* v0 = new osgViewer::View();
  v0->setSceneData(root);
  v0->setUpViewInWindow(0,0,width,height);
  //compositeViewer->addView(v0);


//  //second screen
  Matrix4d c_t = Matrix4d::Identity();
  c_t(2,3) = 1.0;
  VectorXd c_t_vec;
  transform2vec(c_t, c_t_vec);
  Matrix3d KK = Matrix3d::Identity();
  KK(0,0) = -800.0; // fx (-1 * fx to fix an issue with rendering)
  KK(0,1) = 0.0;   // skew
  KK(0,2) = 400.0; // u0
  KK(1,1) = 800.0; // fy
  KK(1,2) = 400.0; // v0
  CameraSensor view2 = CameraSensor(1,KK, 800,800);
  osgViewer::View* v1 = view2.renderCameraView(c_t_vec,0);
  v1->setSceneData(root);
  compositeViewer->addView(v1);
  compositeViewer->realize();

  compositeViewer->frame();
  sleep(1);
  glReadBuffer(GL_FRONT);
  osg::Image *image = new osg::Image();
  image->readPixels(0,0,800,800, GL_RGB, GL_UNSIGNED_BYTE);
  string filename = "test.png";

  osgDB::writeImageFile(*image,filename);

  while(!compositeViewer->done())
  {
	  compositeViewer->frame();
  }

  return 1;
}
