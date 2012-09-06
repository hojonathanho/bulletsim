#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "utils/interpolation.h"
#include "simulation/bullet_io.h"
#include "bscp_pr2.h" 
#include "scp_solver.h"
#include "osg_util.h"
#include <osg/Group>
#include "beacon_sensor.h"
#include "sensors.h"
#include "sensor_functions/PR2BeaconFunc.h";

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

  const float table_height = 0.65; 
  const float table_thickness = 0.08;
  BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale*btVector3(0.85,0.85,table_thickness / 2), btTransform(btQuaternion(0,0,0,1), GeneralConfig::scale * btVector3(1.4,0.0,table_height-table_thickness))));

  table->setColor(0,0,1,1);
  scene.env->add(table);

  // initialize robot
  int T = 30;
  int NX = 7;
  int NU = 7;
  int NS = 0;
  int N_iter = 100;
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
  PR2_SCP pr2_scp(pr2, active_dof_indices, scene.env->bullet->dynamicsWorld, rarm);

  VectorXd startJoints = Map<const VectorXd>(postures[1], NX);
  VectorXd endJoints = Map<const VectorXd>(postures[2], NX);
  vector<VectorXd> X_bar = makeTraj(startJoints, endJoints, T+1);
  vector<VectorXd> U_bar(T);
  for (int i = 0; i < T; i++) {
	  U_bar[i] = X_bar[i+1] - X_bar[i];
  }
  pr2->setDOFValues(active_dof_indices, toVec(startJoints));


  //initialize the sensors
  int NZ = 2;
  Vector3d beacon_1_pos(0.8,-0.4,table_height);
  BeaconSensor s1(beacon_1_pos);
  s1.draw(beacon_1_pos, c_brown, traj_group);
  Robot::SensorFunc s1_f = &PR2BeaconFunc;
  pr2_scp.attach_sensor(&s1, s1_f);

  Vector3d beacon_2_pos(0.8,-0.0,table_height);
  BeaconSensor s2(beacon_1_pos);
  s2.draw(beacon_2_pos, c_brown, traj_group);
  Robot::SensorFunc s2_f = &PR2BeaconFunc;
  pr2_scp.attach_sensor(&s2, s1_f);

  //initilaize the initial distributions and noise models
  MatrixXd Sigma_0 = 0.01*MatrixXd::Identity(NX,NX);
  LLT<MatrixXd> lltSigma_0(Sigma_0);
  MatrixXd rt_Sigma_0 = lltSigma_0.matrixL();

  MatrixXd Q_t = 0.00001*MatrixXd::Identity(NX,NX);
  LLT<MatrixXd> lltQ_t(Q_t);
  MatrixXd M_t = lltQ_t.matrixL();

  MatrixXd R_t = 0.0001*MatrixXd::Identity(NZ,NZ);
  LLT<MatrixXd> lltR_t(R_t);
  MatrixXd N_t = lltR_t.matrixL();


  //Set pr2 noise models
  pr2_scp.set_M(M_t);
  pr2_scp.set_N(N_t);

  VectorXd b_0;
  build_belief_state(startJoints, rt_Sigma_0, b_0);
  vector<VectorXd> B_bar(T+1);
  B_bar[0] = b_0;

  for (int t = 0; t < T; t++) {
    pr2_scp.belief_dynamics(B_bar[t], U_bar[t], B_bar[t+1]);
  }



  PR2_SCP_Plotter plotter(&pr2_scp, &scene, T+1);
  plotter.draw_belief_trajectory(B_bar, c_red, c_red, traj_group);

  scene.startViewer();
  scene.startLoop();
  scene.idle(true);

}
