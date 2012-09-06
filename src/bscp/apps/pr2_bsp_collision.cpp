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
#include "trajectory_util.h"
#include "eigen_multivariate_normal.h"
#include "timer.h"
#include "localizer.h"

using namespace std;
using namespace Eigen;

const static double postures[][7] = {
		{-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0}, // 0=untucked
		{0.062, 	1.287, 		0.1 , -1.554, -3.011, 		-0.268, 2.988}, //1=tucked
		{-0.33, -0.35,  -2.59, -0.15,  -0.59, -1.41, 0.27}, //2=up
		{-1.832,  -0.332,   -1.011,  -1.437,   -1.1  ,  -2.106,  3.074}, //3=side
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
  osg::Group* traj_group = new osg::Group();
  init_transparency_group(traj_group);
  scene.osg->root->addChild(traj_group);

  const float table_height = 0.73;
  const float table_thickness = 0.08;
  BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale*btVector3(0.85,0.85,table_thickness / 2), btTransform(btQuaternion(0,0,0,1), GeneralConfig::scale * btVector3(1.4,0.0,table_height-table_thickness))));

  table->setColor(0,0,1,1);
  scene.env->add(table);

  // initialize robot
  int T = 40;
  int NL = 0;
  int NX = 7 + NL*3;
  int NU = 7;
  int NB = NX*(NX+3)/2;
  int NS = 0;
  int N_iter = 20;
  double rho_x = 0.1;
  double rho_u = 0.1;

  PR2Manager pr2m(scene);
  RaveRobotObject::Ptr pr2 = pr2m.pr2;
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



  VectorXd startJoints = Map<const VectorXd>(postures[1], 7);
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
  int NZ = 0;
//  Vector3d beacon_1_pos(0.8,-0.4,table_height);
//  BeaconSensor s1(beacon_1_pos);
//  s1.draw(beacon_1_pos, c_brown, traj_group);
//  Robot::SensorFunc s1_f = &PR2BeaconFunc;
//  Robot::SensorFuncJacobian s1_df = &PR2BeaconFuncJacobian;
//  //pr2_scp.attach_sensor(&s1, s1_f, s1_df);

//  VectorXd state_indices(3);
//  for (int i = 0; i < 3; i++) state_indices(i) = i;
//
//  StateSensor s2(state_indices);
//  Robot::SensorFunc s2_f = &PR2BeaconFunc;
//  Robot::SensorFuncJacobian s2_df = &PR2BeaconFuncJacobian;
//  pr2_scp.attach_sensor(&s2, s2_f, s2_df);

//  Vector3d beacon_2_pos(0.8,-0.0,table_height);
//  BeaconSensor s2(beacon_1_pos);
//  s2.draw(beacon_2_pos, c_brown, traj_group);
//  Robot::SensorFunc s2_f = &PR2BeaconFunc;
//  pr2_scp.attach_sensor(&s2, s2_f);

  //initilaize the initial distributions and noise models
  MatrixXd Sigma_0 = 0.01*MatrixXd::Identity(NX,NX);
  LLT<MatrixXd> lltSigma_0(Sigma_0);
  MatrixXd rt_Sigma_0 = lltSigma_0.matrixL();

  MatrixXd Q_t = MatrixXd::Zero(NX,NX);
  Q_t.block(0,0,7,7) = 0.00001*MatrixXd::Identity(7,7);
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
  build_belief_state(x0, rt_Sigma_0, b_0);
  vector<VectorXd> B_bar(T+1);
  vector<MatrixXd> W_bar(T);
  B_bar[0] = b_0;

  for (int t = 0; t < T; t++) {
    pr2_scp.belief_dynamics(B_bar[t], U_bar[t], B_bar[t+1]);
    MatrixXd rt_W_t;
    pr2_scp.belief_noise(B_bar[t], U_bar[t], rt_W_t);
    sampler.setCovar(rt_W_t * rt_W_t.transpose());
    sampler.generateSamples(W_bar[t], NS);
  }
  vector<VectorXd> B_bar_r(T+1);
  for (int t = 0; t < T+1; t++) {
	  pr2_scp.parse_localizer_belief_state(B_bar[t], B_bar_r[t]);
  }
//  PR2_SCP_Plotter plotter(&pr2_scp_l, &scene, T+1);
//  plotter.draw_belief_trajectory(B_bar_r, c_red, c_red, traj_group);


  // setup for SCP
  // Define a goal state
  VectorXd b_goal = B_bar[T];

  b_goal.segment(NX, NB-NX) = VectorXd::Zero(NB-NX); // trace

  // Output variables
  vector<VectorXd> opt_B, opt_U; // noiseless trajectory
  MatrixXd Q; VectorXd r;  // control policy
//
 // cout << "calling scp" << endl;
  scp_solver(pr2_scp, B_bar, U_bar, W_bar, rho_x, rho_u, b_goal, N_iter,
      opt_B, opt_U, Q, r);

  TrajectoryInfo opt_traj(b_0);
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

  //use a composite viewer
  int width = 800;
  int height = 800;
  osg::ref_ptr<osgViewer::CompositeViewer> compositeViewer = new osgViewer::CompositeViewer;
  osgViewer::View* v0 = scene.startView();
//  osg::ref_ptr<osg::Camera> cam = v0->getCamera();
//  cam->setGraphicsContext(gc.get());
//  cam->setViewport(0, 0, width/2, height);
  compositeViewer->addView(v0);
  compositeViewer->addView(scene.startView());
  compositeViewer->removeView(v0);

  compositeViewer->run();

  //scene.startViewer();
  //scene.startLoop();
  //scene.idle(true);

}
