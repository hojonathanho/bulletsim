#include <iostream>
#include <Eigen/Dense>
#include "robots.h"
#include "car.h"
#include "localizer.h"
#include "beacon_sensor.h"
#include "eigen_io_util.h"
#include "osg_util.h"
#include <Eigen/Geometry>
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
#include <osgGA/CameraManipulator>
#include <osgGA/TrackballManipulator>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "kalman_filter.h"
#include "scp_solver.h"
#include "timer.h"
#include "eigen_multivariate_normal.h"
#include "trajectory_util.h"
#include "camera_sensor.h"
#include "sensor_functions/CarBeaconFunc.h"
#include "sensor_functions/LocalizerCarCameraFunc.h"

using namespace Eigen;
using namespace std;


void build_opengl_projection_for_intrinsics( Eigen::Matrix4d &frustum, //int *viewport,
		double alpha, double beta, double skew, double u0, double v0,
   int img_width, int img_height, double near, double far ) {
	//http://tech.dir.groups.yahoo.com/group/OpenCV/message/38181
	frustum = Matrix4d::Zero();
	double fx = alpha;
	double fy = beta;
	double cx = u0;
	double cy = v0;

	frustum(0,0) = 2* fx/img_width;
	frustum(0,1) = 2*cx/img_width - 1;
	frustum(1,1) = 2*fy/img_height;
	frustum(1,2) = 2*cy/img_height - 1;
	frustum(2,2) = -(far+near)/(far-near);
	frustum(2,3) = -2*far*near/(far-near);
	frustum(3,2) = -1;

}

Matrix4d ComputeExtrinsics(Matrix4d& cam_world_transform) {
	Matrix4d ret = Matrix4d::Identity();
	ret.block(0,0,3,3) =  cam_world_transform.block(0,0,3,3).transpose();
	ret.block(0,3,3,1) = - ret.block(0,0,3,3) * cam_world_transform.block(0,3,3,1);
	return ret;
}
//typedef VectorXd (*SensorFunc2)(const VectorXd&);

static VectorXd _goal_offset;
VectorXd GoalFn(Robot &r, const VectorXd& x) {
	return x - _goal_offset;
}

VectorXd TouchingGoalFn(Robot &r, const VectorXd& x) {
	VectorXd mu_x; MatrixXd rt_Sigma_x;
	parse_belief_state(x, mu_x, rt_Sigma_x);
	VectorXd ret(2 + x.rows()-mu_x.rows());
	ret.segment(0,2) = mu_x.segment(0,2) - mu_x.segment(4,2);
	ret.segment(2,x.rows()-mu_x.rows()) = x.segment(mu_x.rows(), x.rows()- mu_x.rows());
	return ret;
}


int main()
{
  // initialize random number generator
  //srand(time(NULL));
  srand(29);

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
  int N_iter = 200;

  // hack for now
  MatrixXd W_cov = MatrixXd::Zero(NB,NB);
  EigenMultivariateNormal<double> sampler(VectorXd::Zero(NB), W_cov);
  
  VectorXd x0(4);
  x0 << -0.4, 0.3, 0.0, 0.3;
  Vector3d pos_test = Vector3d(-0.2,0.1,0.0);
  Car car(x0);
  VectorXd l_x0 = VectorXd::Zero(NX);
  l_x0.segment(0,4) = x0;
  l_x0.segment(4,3) = pos_test;
  Localizer c(&car, 1);

  //initialize the sensors
  int NZ = 5;
  Vector2d beacon_1_pos(-0.2,0.4);
  BeaconSensor s1(beacon_1_pos,50);
  s1.draw(beacon_1_pos, brown, traj_group);
  Robot::SensorFunc s1_f = &CarBeaconFunc;
  c.attach_sensor(&s1, s1_f);

  Vector2d beacon_2_pos(0.2,0.1); 
  BeaconSensor s2(beacon_2_pos,50);
  s2.draw(beacon_2_pos, brown, traj_group);
  Robot::SensorFunc s2_f = &CarBeaconFunc;
  c.attach_sensor(&s2, s2_f);

  Vector2d beacon_3_pos(0.0,0.2);
  BeaconSensor s3(beacon_3_pos,50);
  s3.draw(beacon_3_pos, brown, traj_group);
  Robot::SensorFunc s3_f = &CarBeaconFunc;
  c.attach_sensor(&s3, s3_f);
  
  Matrix4d camera_transform = Matrix4d::Identity();
  camera_transform.block(0, 0, 3, 3) = AngleAxisd(0.2, Vector3d(0.0,0.0,1.0)).toRotationMatrix();

  Matrix3d KK = Matrix3d::Identity();
  KK(0,0) = -640.0; // fx (-1 * fx to fix an issue with rendering)
  KK(0,1) = 0.0;   // skew
  KK(0,2) = 320.0; // u0
  KK(1,1) = 480.0; // fy
  KK(1,2) = 240.0; // v0

  Matrix4d camera_fixed_offset = Matrix4d::Identity();
  camera_fixed_offset.block(0,0,3,3) =
		  AngleAxisd(-M_PI / 2, Vector3d(0.0, 1.0, 0.0)).toRotationMatrix()
					* AngleAxisd(-M_PI / 2, Vector3d(0.0, 0.0, 1.0)).toRotationMatrix();
  CameraSensor s4 = CameraSensor(1, KK, 640, 480, camera_fixed_offset);
  Robot::SensorFunc s4_f = &LocalizerCarCameraFunc;
  c.attach_sensor(&s4, s4_f);

  VectorXd camera_transform_vec;
  transform2vec(camera_transform, camera_transform_vec);

 // osg::Node* ell_test =drawEllipsoid(pos_test, 0.0001*Matrix3d::Identity(),orange);
 // traj_group->addChild(ell_test);




  //initilaize the initial distributions and noise models
  MatrixXd Sigma_0 = 0.001*MatrixXd::Identity(NX,NX);
  Sigma_0(2,2) = 0.00001;
  Sigma_0(3,3) = 0.00001;
  Sigma_0(6,6) = 0;
  LLT<MatrixXd> lltSigma_0(Sigma_0);
  MatrixXd rt_Sigma_0 = lltSigma_0.matrixL();

  MatrixXd Q_t = MatrixXd::Zero(NX,NX);
  Q_t(0,0) = 0.00001;
  Q_t(1,1) = 0.00001;
  Q_t(2,2) = 0.00000001;
  Q_t(3,3) = 0.000000001;
  Q_t(4,4) = 1e-10;
  Q_t(5,5) = 1e-10;
  Q_t(6,6) = 1e-10;


  LLT<MatrixXd> lltQ_t(Q_t);
  MatrixXd M_t = lltQ_t.matrixL();

  MatrixXd R_t = 0.01*MatrixXd::Identity(NZ,NZ);
  //R_t(3,3) = 10.0;
  //R_t(4,4) = 10.0;

  R_t(3,3) = 0.00001;
  R_t(4,4) = 0.00001;
  LLT<MatrixXd> lltR_t(R_t);
  MatrixXd N_t = lltR_t.matrixL();

  //Set car noise models
  c.set_M(M_t);
  c.set_N(N_t);

  VectorXd b_0;
  build_belief_state(l_x0, rt_Sigma_0, b_0);
  vector<VectorXd> B_bar(T+1);
  vector<VectorXd> U_bar(T);
  vector<MatrixXd> W_bar(T);
  B_bar[0] = b_0;

  for (int t = 0; t < T; t++) {
    VectorXd u = VectorXd::Random(2) / 3;
    if (t > T/2) u = - 2 * u;
    U_bar[t] = u;
    MatrixXd rt_W_t;
    //MatrixXd H_t, rt_Sigma_t;
    //VectorXd z_t, x_t;
    //parse_belief_state(B_bar[t],x_t,rt_Sigma_t);
    //c.observe(x_t, z_t);
    //cout << z_t << endl;
    //c.dgdx(x_t, H_t);
    //cout << H_t << endl << endl;
    c.belief_dynamics(B_bar[t], U_bar[t], B_bar[t+1]);
    c.belief_noise(B_bar[t], U_bar[t], rt_W_t);
    sampler.setCovar(rt_W_t * rt_W_t.transpose());
    sampler.generateSamples(W_bar[t], NS);
    //cout << W_bar[t] << endl;
  }

  cout <<"asdf" << endl;


  vector<vector<VectorXd> > W_s_t;
  index_by_sample(W_bar, W_s_t);
  vector<VectorXd> test;
  for (int s = 0; s < NS; s++) {
	  c.forward_integrate(b_0, U_bar, W_s_t[s], test);
	  render = c.draw_belief_trajectory(test, red, transparent, traj_group, 1e-4);
  }

  render = c.draw_belief_trajectory(B_bar, red, yellow, traj_group, 1e-4);
  c.draw_object_uncertainty(B_bar[0], yellow, traj_group, 1e-4);
  c.draw_object_uncertainty(B_bar[T], orange, traj_group, 1e-4);
  VectorXd test_x; MatrixXd test_rt_Sigma;
  parse_belief_state(B_bar[T], test_x, test_rt_Sigma);
  cout << test_rt_Sigma * test_rt_Sigma.transpose();
  c.draw_sensor_belief_trajectory(B_bar, brown, traj_group, z_offset/2);



  // setup for SCP
  // Define a goal state
  VectorXd b_goal = B_bar[T];
  b_goal.segment(NX, NB-NX) = VectorXd::Zero(NB-NX); // trace
  _goal_offset = b_goal;
  // Output variables
  vector<VectorXd> opt_B, opt_U; // noiseless trajectory
  MatrixXd Q; VectorXd r;  // control policy

  cout << "calling scp" << endl;
  scp_solver(c, B_bar, U_bar, W_bar, rho_x, rho_u, &TouchingGoalFn, NULL, N_iter,
      opt_B, opt_U, Q, r);

  TrajectoryInfo opt_traj(b_0, &GoalFn, NULL);
    for (int t = 0; t < T; t++) {
  	  //opt_traj.add_and_integrate(opt_U[t], VectorXd::Zero(NX), c);
  	  VectorXd feedback = opt_traj.Q_feedback(c);
  	  VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
  	  opt_traj.add_and_integrate(u_policy, VectorXd::Zero(NB), c);
    }
    c.draw_belief_trajectory(opt_traj._X, blue, orange, traj_group, z_offset/2+1e-4);
    c.draw_object_uncertainty(opt_traj._X[T], red, traj_group, z_offset/2 + 1e-4);
    c.draw_sensor_belief_trajectory(opt_traj._X, brown, traj_group, z_offset/2);

    for (int s = 0; s < NUM_TEST; s++) {
  	  TrajectoryInfo test_traj(b_0, &GoalFn, NULL);
  	  for (int t = 0; t < T; t++) {
  		  VectorXd feedback = test_traj.Q_feedback(c);
  	  	  VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
  		  test_traj.add_and_integrate(u_policy, W_bar[t].col(s), c);
  		  //test_traj.add_and_integrate(u_policy, sampler.nextSample(), c);
  	  }
  	  c.draw_belief_trajectory(test_traj._X, blue, transparent, traj_group, z_offset/2+1e-4);
    }


  // visualize
  osg::Geode * fgeode = new osg::Geode;
  osg::ShapeDrawable *floor = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0,0.0,-z_offset/2), 1.0, 1.0, z_offset));
  floor->setColor(osg::Vec4(0.1,0.1,0.1,1.0));
  fgeode->addDrawable(floor);
  root->addChild(fgeode);

  int width = 800;
  int height = 800;
  osg::ref_ptr<osgViewer::CompositeViewer> compositeViewer = new osgViewer::CompositeViewer;

  osgViewer::View* v0 = new osgViewer::View();
  osg::Group* v0_root = new osg::Group();
  v0_root->addChild(root);
  v0->setSceneData(v0_root);
  v0->setUpViewInWindow(0,0,width,height);
  osgGA::TrackballManipulator * tman = new osgGA::TrackballManipulator;
  v0->setCameraManipulator(tman);
  compositeViewer->addView(v0);



  //second screen
  osgViewer::View* v1 = s4.renderCameraView(camera_transform_vec);
  v1->setSceneData(root);
  compositeViewer->addView(v1);
  compositeViewer->realize();



  while(!compositeViewer->done())
  {
	  compositeViewer->frame();
	  usleep(10000);
  }

  return 0;


  //return compositeViewer->run();
}
