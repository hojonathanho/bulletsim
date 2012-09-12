#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "utils/interpolation.h"
#include "simulation/bullet_io.h"
#include "bscp_pr2.h" 
#include "scp_solver.h"
#include "osg_util.h"
#include <osg/Group>
#include <osgViewer/CompositeViewer>
#include <osgViewer/View>
#include "beacon_sensor.h"
#include "state_sensor.h"
#include "sensors.h"
#include "sensor_functions/LocalizerPR2CameraFunc.h"
#include "sensor_functions/PR2BeaconFunc.h"
#include "trajectory_util.h"
#include "eigen_multivariate_normal.h"
#include "timer.h"
#include "localizer.h"
#include "camera_sensor.h"
#include "proximity_sensor.h"

using namespace std;
using namespace Eigen;

const static double postures[][7] = {
		{-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0}, // 0=untucked
		{0.062, 	1.287, 		0.1 , -1.554, -3.011, 		-0.268, 2.988}, //1=tucked
		{-0.33, -0.35,  -2.59, -0.15,  -0.59, -1.41, 0.27}, //2=up
		{-1.832,  -0.332,   -1.011,  -1.437,   -1.1  ,  -2.106,  3.074}, //3=side
		{0, 0, 0, 0, 0, 0, 0}}; //4=outstretched

const static double left_postures[][7] = {
		{-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0}, // 0=untucked
		{0.062, 	1.287, 		0.1 , -1.554, -3.011, 		-0.268, 2.988}, //1=tucked
		{-0.33, -0.35,  -2.59, -0.15,  -0.59, -1.41, 0.27}, //2=up
		{1.832,  0.332,   1.011,  1.437,   1.1  ,  2.106,  3.074}, //3=side
		{0, 0, 0, 0, 0, 0, 0}}; //4=outstretched


vector<VectorXd> makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, int nSteps) {
  assert(startJoints.rows() == endJoints.rows());
  Eigen::MatrixXd startEndJoints(2, startJoints.rows());
  startEndJoints.row(0) = startJoints;
  startEndJoints.row(1) = endJoints;
  MatrixXd m_traj= interp2d(VectorXd::LinSpaced(nSteps, 0, 1), VectorXd::LinSpaced(2, 0, 1), startEndJoints);
  vector<VectorXd> traj(0);
  for (int i = 0; i < m_traj.rows(); i++) {
	  traj.push_back(m_traj.row(i));
  }
  return traj;

}
const static Vector4d c_yellow(1.0,1.0,0.1,0.4);
const static Vector4d c_red(1.0, 0.0, 0.0, 0.8);
const static Vector4d c_green(0.2, 1.0, 0.2, 0.8);
const static Vector4d c_blue(0.2, 0.2, 1.0, 0.8);
const static Vector4d c_orange(1.0, 0.55, 0.0, 0.8);
const static Vector4d c_white(1.0, 1.0, 1.0, 0.1);
const static Vector4d c_brown(139/255.0,69/255.0,19/255.0,0.8);
const static Vector4d c_gold(255.0/255.0,204.0/255,0.0/255,1.0);


//this is so hacky
static VectorXd fixed_camera_trans;

inline VectorXd FixedCameraObjectFunc(Robot& r, const VectorXd& x) {
	Localizer* l = static_cast<Localizer*>(&r);
	VectorXd x_r, x_o;
	l->parse_localizer_state(x, x_r, x_o);
	cout << fixed_camera_trans.transpose() << endl;
	VectorXd x_transform = fixed_camera_trans;
	VectorXd ret(7 + x_o.rows());
	ret.segment(0,7) = x_transform;
	ret.segment(7,x_o.rows()) = x_o; // for observing objects in the scene
	return ret;
}

inline VectorXd FixedCameraPR2Func(Robot& r, const VectorXd& x) {
	VectorXd camera_transform = fixed_camera_trans;
	VectorXd ret(7 + 3);
	ret.segment(0,7) = camera_transform;
	ret.segment(7,3) = r.xyz(x); // for observing objects in the scene
	return ret;
}

inline MatrixXd FixedCameraPR2FuncJac(Robot& r, const VectorXd& x) {
	MatrixXd ret = MatrixXd::Zero(10, r._NX);
	MatrixXd Jxyz;
	r.dxyz(x, Jxyz);
	ret.block(7,0,3,r._NX) = Jxyz;
	return ret;

}

static VectorXd _goal_offset;
VectorXd GoalFn(Robot &r, const VectorXd& x) {
	return x - _goal_offset;
}

VectorXd meanTouchingGoalFn(Robot& r, const VectorXd& mu_x) {
	VectorXd ret(3);
	ret.segment(0,3) = (r.xyz(mu_x) - mu_x.segment(7,3));
	return ret;
}

VectorXd TouchingGoalFn(Robot &r, const VectorXd& x) {
	VectorXd mu_x; MatrixXd rt_Sigma_x;
	parse_belief_state(x, mu_x, rt_Sigma_x);
	MatrixXd Sigma_x = rt_Sigma_x * rt_Sigma_x.transpose();
	VectorXd mu_g; MatrixXd Sigma_g;
	r.unscented_transform_goal(mu_x, Sigma_x, &meanTouchingGoalFn, mu_g, Sigma_g);
	LLT<MatrixXd> lltOfSigma_g (Sigma_g);
	MatrixXd rt_Sigma_g = lltOfSigma_g.matrixL();
	VectorXd bg;
	build_belief_state(mu_g, rt_Sigma_g, bg);
	return bg;
}

VectorXd OldTouchingGoalFn(Robot &r, const VectorXd& x) {
	VectorXd mu_x; MatrixXd rt_Sigma_x;
	parse_belief_state(x, mu_x, rt_Sigma_x);
	VectorXd ret(3 + x.rows()-mu_x.rows());
	ret.segment(0,3) = (r.xyz(mu_x) - mu_x.segment(7,3));
	ret.segment(3,x.rows()-mu_x.rows()) = 0.1*x.segment(mu_x.rows(), x.rows()- mu_x.rows());
	return ret;
}







int main(int argc, char* argv[]) {

  //initialize parameters and parser
  GeneralConfig::scale = 1.;
  BulletConfig::friction = 2;
  BulletConfig::margin = 0.01;
  Parser parser;

  parser.addGroup(GeneralConfig());
  parser.addGroup(BulletConfig());
  parser.addGroup(SceneConfig());
  parser.read(argc, argv);

  cout << "Link padding: " << endl;
  cout << BulletConfig::linkPadding << endl;

  //initialize scene
  Scene scene;
  osg::Group* traj_group =  new osg::Group();
  osg::Group* root = new osg::Group();
  osg::Group* camera_root = new osg::Group();
  osg::Group* camera_group = new osg::Group();
  root->addChild(scene.osg->root);
  root->addChild(traj_group);
  camera_root->addChild(scene.osg->root);
  camera_root->addChild(camera_group);
  init_transparency_group(traj_group);
  init_transparency_group(camera_group);

  const float table_height = 0.73;
  const float table_thickness = 0.08;
	BoxObject::Ptr table(
			new BoxObject(0,
					GeneralConfig::scale
							* btVector3(0.85, 0.85, table_thickness / 2),
					btTransform(btQuaternion(0, 0, 0, 1),
							GeneralConfig::scale
									* btVector3(1.4, 0.0,
											table_height - table_thickness))));
  table->setColor(0,0,1,1);

  BoxObject::Ptr middle_block(
 			new BoxObject(0,
 					GeneralConfig::scale
 							* btVector3(0.1, 0.1, 0.2),
 					btTransform(btQuaternion(0, 0, 0, 1),
 							GeneralConfig::scale
 									* btVector3(0.65, 0.2,
 											table_height+ 0.1))));
  middle_block->setColor(0.3,0.3,0.3,1);

  BoxObject::Ptr button_block(
 			new BoxObject(0,
 					GeneralConfig::scale
 							* btVector3(0.025, 0.025, 0.025),
 					btTransform(btQuaternion(0, 0, 0, 1),
 							GeneralConfig::scale
 									* btVector3(0.65, 0.1,
 											table_height+ 0.15))));

  button_block->setColor(1.0,0.0,0.0,1.0);

  scene.env->add(table);
  scene.env->add(middle_block);
  scene.env->add(button_block);

  Vector3d object_pos(0.65,0.1-0.025,table_height+0.15);

  // initialize robot
  int T = 30;
  int NL = 1;
  int NX = 7 + NL*3;
  int NU = 7;
  int NB = NX*(NX+3)/2;
  int NS = 0;
  int NUM_TEST = NS;
  int N_iter = 30;
  double rho_x = 0.1;
  double rho_u = 0.1;

  PR2Manager pr2m(scene);
  RaveRobotObject::Ptr pr2 = pr2m.pr2;
  vector<int> left_active_dof = pr2m.pr2Left->manip->GetArmIndices();
  VectorXd left_out = Map<const VectorXd>(left_postures[3], 7);
  pr2->setDOFValues(left_active_dof, toVec(left_out));

  RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;
  vector<int> active_dof_indices = rarm->manip->GetArmIndices();

  BOOST_FOREACH(BulletObject::Ptr child, pr2->children) {
	  if (child) {
		  btRigidBody* rb = child->rigidBody.get();
		  scene.env->bullet->dynamicsWorld->removeRigidBody(rb);
	  }
  }


  PR2_SCP pr2_scp_l(pr2, active_dof_indices, scene.env->bullet->dynamicsWorld, rarm);
  Localizer pr2_scp(&pr2_scp_l, NL);


  //VectorXd startJoints = Map<const VectorXd>(postures[1], 7);
  VectorXd startJoints = Map<const VectorXd>(postures[3], 7); //works!
  VectorXd endJoints = Map<const VectorXd>(postures[4], 7);
  vector<VectorXd> X_bar = makeTraj(startJoints, endJoints, T+1);
  vector<VectorXd> U_bar(T);
  for (int i = 0; i < T; i++) {
	  U_bar[i] = X_bar[i+1] - X_bar[i];
  }
  pr2->setDOFValues(active_dof_indices, toVec(startJoints));

  //hack for now
  MatrixXd W_cov = MatrixXd::Zero(NB,NB);
  EigenMultivariateNormal<double> sampler(VectorXd::Zero(NB), W_cov);

  //initialize the sensors
  int NZ = 3;
//  Vector3d beacon_1_pos(0.8,-0.4,table_height);
//  BeaconSensor s1(beacon_1_pos);
//  s1.draw(beacon_1_pos, c_brown, traj_group);
//  Robot::SensorFunc s1_f = &PR2BeaconFunc;
//  Robot::SensorFuncJacobian s1_df = &PR2BeaconFuncJacobian;
  //pr2_scp.attach_sensor(&s1, s1_f, s1_df);

//  VectorXd state_indices(3);
//  for (int i = 0; i < 3; i++) state_indices(i) = i;
//
//  StateSensor s2(state_indices);
//  Robot::SensorFunc s2_f = &PR2BeaconFunc;
//  Robot::SensorFuncJacobian s2_df = &PR2BeaconFuncJacobian;
//  pr2_scp.attach_sensor(&s2, s2_f, s2_df);


  Matrix3d KK = Matrix3d::Identity();
  KK(0,0) = -640.0; // fx (-1 * fx to fix an issue with rendering)
  KK(0,1) = 0.0;   // skew
  KK(0,2) = 320.0; // u0
  KK(1,1) = 480.0; // fy
  KK(1,2) = 240.0; // v0

  Matrix4d attached_cam_fixed_offset = Matrix4d::Identity();
  //attached_cam_fixed_offset(2,3) = -0.05;
  attached_cam_fixed_offset.block(0,0,3,3) = AngleAxisd(M_PI, Vector3d(0.0,1.0,0.0)).toRotationMatrix();
  CameraSensor s4 = CameraSensor(1, KK, 640, 480, attached_cam_fixed_offset, 0.2);//, camera_fixed_offset);
  Robot::SensorFunc s4_f = &LocalizerPR2AttachedCameraFunc;
  Robot::SensorFuncJacobian s4_fj = &LocalizerPR2AttachedCameraFuncJac;
  pr2_scp.attach_sensor(&s4, s4_f, s4_fj);

  Matrix4d fixed_cam_position = Matrix4d::Identity();
  fixed_cam_position(0,3) = 0.8;
  fixed_cam_position(2,3) = 3.0;
  transform2vec(fixed_cam_position, fixed_camera_trans); // for setting observations
  CameraSensor s5 = CameraSensor(1, KK, 640, 480, Matrix4d::Identity(), 0.1);
  s5.draw(fixed_camera_trans, c_blue, traj_group);
  //Robot::SensorFunc s5_f = &FixedCameraObjectFunc;
  Robot::SensorFunc s5_f = &FixedCameraPR2Func;
  Robot::SensorFuncJacobian s5_fj = &FixedCameraPR2FuncJac;
  //pr2_scp.attach_sensor(&s5, s5_f, s5_fj);

  ProximitySensor s6 = ProximitySensor(0.1, &scene);
  Robot::SensorFunc s6_f = &PR2BeaconFunc;
  Robot::SensorFuncJacobian s6_fj = &PR2BeaconFuncJacobian;
  pr2_scp.attach_sensor(&s6, s6_f, s6_fj);




  //initilaize the initial distributions and noise models
  MatrixXd Sigma_0 = 0.01*MatrixXd::Identity(NX,NX); //was 0.01
  LLT<MatrixXd> lltSigma_0(Sigma_0);
  MatrixXd rt_Sigma_0 = lltSigma_0.matrixL();

  MatrixXd Q_t = MatrixXd::Zero(NX,NX);
  Q_t.block(0,0,7,7) = 0.000001*MatrixXd::Identity(7,7);
  LLT<MatrixXd> lltQ_t(Q_t);
  MatrixXd M_t = lltQ_t.matrixL();

  MatrixXd R_t = 0.01*MatrixXd::Identity(NZ,NZ);
  LLT<MatrixXd> lltR_t(R_t);
  MatrixXd N_t = lltR_t.matrixL();


  //Set pr2 noise models
  pr2_scp.set_M(M_t);
  pr2_scp.set_N(N_t);


  VectorXd b_0;
  VectorXd x0(NX);
  x0.segment(0,7) = startJoints;
  x0.segment(7,3) = object_pos;
  build_belief_state(x0, rt_Sigma_0, b_0);
  vector<VectorXd> B_bar(T+1);
  vector<MatrixXd> W_bar(T);
  B_bar[0] = b_0;

  for (int t = 0; t < T; t++) {
    pr2_scp.belief_dynamics(B_bar[t], U_bar[t], B_bar[t+1]);

    MatrixXd H_t, rt_Sigma_t;
    VectorXd z_t, x_t;
    parse_belief_state(B_bar[t],x_t,rt_Sigma_t);
    pr2_scp.observe(x_t, z_t);

    MatrixXd rt_W_t;
    pr2_scp.belief_noise(B_bar[t], U_bar[t], rt_W_t);
    sampler.setCovar(rt_W_t * rt_W_t.transpose());
    sampler.generateSamples(W_bar[t], NS);
  }
  vector<VectorXd> B_bar_r(T+1);
  for (int t = 0; t < T+1; t++) {
	  pr2_scp.parse_localizer_belief_state(B_bar[t], B_bar_r[t]);
  }
  PR2_SCP_Plotter plotter(&pr2_scp_l, &scene, T+1);
  //plotter.draw_belief_trajectory(B_bar_r, c_red, c_red, traj_group);
  //pr2_scp.draw_sensor_belief_trajectory(B_bar, c_blue, traj_group);
  //pr2_scp.draw_object_uncertainty(B_bar[0], c_red, traj_group);
  //pr2_scp.draw_object_uncertainty(B_bar[T], c_blue, traj_group);

//    vector<vector<VectorXd> > W_s_t;
//    index_by_sample(W_bar, W_s_t);
//    vector<VectorXd> test;
//    vector<PR2_SCP_Plotter*> plotters(NS);
//    for (int s = 0; s < NS; s++) {
//  	  pr2_scp.forward_integrate(B_bar[0], U_bar, W_s_t[s], test);
//  	  plotters[s] = new PR2_SCP_Plotter(&pr2_scp_l, &scene, T+1);
//  	  vector<VectorXd> test_r(T+1);
//  	  for (int t = 0; t < T+1; t++) {
//  		  VectorXd tmp;
//  		  pr2_scp.parse_localizer_belief_state(test[t], test_r[t]);
//  	  }
//  	  plotters[s]->draw_belief_trajectory(test_r, c_red, c_red, traj_group);
//    }



  // setup for SCP
  // Define a goal state
  VectorXd b_goal = B_bar[T];
  b_goal.segment(NX, NB-NX) = VectorXd::Zero(NB-NX); // trace
  _goal_offset = b_goal;

  // Output variables
  vector<VectorXd> opt_B, opt_U; // noiseless trajectory
  MatrixXd Q; VectorXd r;  // control policy
//
 // cout << "calling scp" << endl;
  scp_solver(pr2_scp, B_bar, U_bar, W_bar, rho_x, rho_u, &TouchingGoalFn, NULL, N_iter,
      opt_B, opt_U, Q, r);

  TrajectoryInfo opt_traj(b_0, &GoalFn, NULL);
  for (int t = 0; t < T; t++) {
	  opt_traj.add_and_integrate(opt_U[t], VectorXd::Zero(NB), pr2_scp);
	  //VectorXd feedback = opt_traj.Q_feedback(pr2_scp);
	  //VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
	  //opt_traj.add_and_integrate(u_policy, VectorXd::Zero(NB), pr2_scp);
  }
  vector<VectorXd> opt_traj_r(T+1);
  for (int t = 0; t < T+1; t++) {
	  pr2_scp.parse_localizer_belief_state(opt_traj._X[t], opt_traj_r[t]);
  }
  PR2_SCP_Plotter plotter2(&pr2_scp_l, &scene, T + 1);
  plotter2.draw_belief_trajectory(opt_traj_r, c_blue, c_orange, traj_group);
  pr2_scp.draw_object_uncertainty(opt_traj._X[T], c_green, traj_group);
  //pr2_scp.draw_sensor_belief_trajectory(opt_traj._X, c_blue, traj_group);
  cout << opt_traj._X[T].transpose() << endl;

//  vector<vector<VectorXd> > W_s_t;
//  index_by_sample(W_bar, W_s_t);
//  vector<VectorXd> test;
//  vector<PR2_SCP_Plotter*> plotters(NUM_TEST);
//  for (int s = 0; s < NUM_TEST; s++) {
//	  //integrate trajectory
//	  TrajectoryInfo test_traj(opt_traj._X[0], &GoalFn, NULL);
//	  for (int t = 0; t < T; t++) {
//		  VectorXd feedback = test_traj.Q_feedback(pr2_scp_l);
//		  VectorXd u_policy = Q.block(t*NU, t*NB, NU, NB) * feedback + r.segment(t*NU, NU);
//		  test_traj.add_and_integrate(u_policy, W_bar[t].col(s), pr2_scp_l);
//		  //test_traj.add_and_integrate(u_policy, sampler.nextSample(), pr2_scp);
//	  }
//	  //get robot state out
//	  vector<VectorXd> test_traj_r(T+1);
//	  for (int t = 0; t < T+1; t++) {
//		  pr2_scp.parse_localizer_belief_state(test_traj._X[t], test_traj_r[t]);
//	  }
//	  //draw
//	  plotters[s] = new PR2_SCP_Plotter(&pr2_scp_l, &scene, T+1);
//	  plotters[s]->draw_trajectory(test_traj._X, c_red);
//  }

  //use a composite viewer
  int width = 800;
  int height = 800;
  osg::ref_ptr<osgViewer::CompositeViewer> compositeViewer = new osgViewer::CompositeViewer;
  osgViewer::View* v0 = scene.startView(); //v0 is the master view containing the scene data
  v0->setSceneData(root);

  osgViewer::View* v1 = s5.renderCameraView(fixed_camera_trans);
  v1->setSceneData(v0->getSceneData());

  osgViewer::View* v2 = s5.renderCameraView(fixed_camera_trans);
  v2->setSceneData(camera_root);


  compositeViewer->addView(v0);
  compositeViewer->addView(v1);
  compositeViewer->addView(v2);

  compositeViewer->realize();

  while(!compositeViewer->done())
  {
	  compositeViewer->frame();
  }

//  scene.startViewer();
//  scene.startLoop();
//  scene.idle(true);

}
