#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "utils/interpolation.h"
#include "simulation/bullet_io.h"
#include "bscp_pr2.h" 
#include "scp_solver.h"
#include "osg_util.h"
#include <osg/Group>
#include "osg_torus.h"
#include <osg/Light>
#include <osg/LightSource>

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
const static Vector4d c_gold(255.0/255.0,204.0/255,0.0/255,1.0);

static VectorXd _goal_offset;
VectorXd GoalFn(Robot& r, const VectorXd& x) {
	VectorXd tx;
	r.transform(x, tx);
	return tx - _goal_offset;
}

MatrixXd GoalFnJac(Robot& r, const VectorXd& x) {
	MatrixXd ret;
	r.dtransform(x, ret);
	return ret;
}


int main(int argc, char* argv[]) {
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
  Scene scene;
  osg::Group* traj_group = new osg::Group();
//  //init_transparency_group(scene.osg->root);
  init_transparency_group(traj_group);
  scene.osg->root->addChild(traj_group);
//  traj_group->addChild(drawEllipsoid(Vector3d(0.0,0.0,0.0), Matrix3d::Identity(), Vector4d(1.0,0.0,0.0,0.5)));

  const float table_height = 0.65; 
  const float table_thickness = 0.08;
  int T = 30;
  int NX = 7;
  int NU = 7;
  int NS = 0;
  int N_iter = 50;
  double rho_x = 0.1;
  double rho_u = 0.1;


  PR2Manager pr2m(scene);
  RaveRobotObject::Ptr pr2 = pr2m.pr2;

  BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale*btVector3(0.85,0.85,table_thickness / 2), btTransform(btQuaternion(0,0,0,1), GeneralConfig::scale * btVector3(1.4,0.0,table_height-table_thickness))));

  table->setColor(0,0,1,1);
  scene.env->add(table);

  BoxObject::Ptr obs_1(
			new BoxObject(0,
					GeneralConfig::scale
							* btVector3(0.2, 0.2, 0.1),
					btTransform(btQuaternion(0, 0, 0, 1),
							GeneralConfig::scale
									* btVector3(0.7, -0.5,
											table_height))));
  BoxObject::Ptr obs_2(
			new BoxObject(0,
					GeneralConfig::scale
							* btVector3(0.2, 0.2, 0.1),
					btTransform(btQuaternion(0, 0, 0, 1),
							GeneralConfig::scale
									* btVector3(0.7, 0.5,
											table_height))));

  BoxObject::Ptr obs_3(
			new BoxObject(0,
					GeneralConfig::scale
							* btVector3(0.1, 0.1, 0.2),
					btTransform(btQuaternion(0, 0, 0, 1),
							GeneralConfig::scale
									* btVector3(0.65, 0.0,
											table_height+ 0.1))));
  obs_1->setColor(139/255.0,69/255.0,19/255.0,1); // brown
  obs_2->setColor(139/255.0,69/255.0,19/255.0,1); // brown
  obs_3->setColor(139/255.0,69/255.0,19/255.0,1); // brown

  scene.env->add(obs_1);
  scene.env->add(obs_2);
  scene.env->add(obs_3);

  RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;
  vector<int> active_dof_indices = rarm->manip->GetArmIndices();
  pr2->robot->SetActiveDOFs(active_dof_indices);


  BOOST_FOREACH(BulletObject::Ptr child, pr2->children) {
		if	(child) {
			btRigidBody* rb = child->rigidBody.get();
			scene.env->bullet->dynamicsWorld->removeRigidBody(rb);
		}
  }
  

  PR2_SCP pr2_scp(pr2, active_dof_indices, scene.env->bullet->dynamicsWorld, rarm);

  VectorXd startJoints = Map<const VectorXd>(postures[1], NX);
  VectorXd endJoints = Map<const VectorXd>(postures[0], NX);
  vector<VectorXd> X_bar = makeTraj(startJoints, endJoints, T+1);
  vector<VectorXd> U_bar(T);
  vector<MatrixXd> W_bar(T);
  for (int i = 0; i < T; i++) {
	  cout << pr2_scp.xyz(X_bar[i]).transpose() << endl;
	  //traj_group->addChild(drawEllipsoid(pr2_scp.xyz(X_bar[i]), 0.0001*Matrix3d::Identity(), Vector4d(1.0,0.0,0.0,1.0)));

	  U_bar[i] = X_bar[i+1] - X_bar[i];
	  W_bar[i] = MatrixXd(0,0);
  }
  
  pr2->setDOFValues(active_dof_indices, toVec(startJoints));

//  PR2_SCP_Plotter plotter(&pr2_scp, &scene, T+1);
//  plotter.draw_trajectory(X_bar, c_red);

  //VectorXd x_goal = Map<const VectorXd>(postures[2], NX);
  VectorXd x_goal;
  pr2_scp.transform(X_bar[T], x_goal);
  _goal_offset = x_goal;
  vector<VectorXd> opt_X(0), opt_U(0);
  MatrixXd K; VectorXd u0;

  scp_solver(pr2_scp, X_bar, U_bar, W_bar, rho_x, rho_u, &GoalFn, &GoalFnJac, N_iter,
      opt_X, opt_U, K, u0);

  for( int i = 0; i < opt_X.size(); i++ ) {
	  cout << opt_X[i].transpose() << endl;
  }

  VectorXd x_bar_transform;
  VectorXd opt_x_transform;

  pr2_scp.transform(X_bar[T], x_bar_transform);
  pr2_scp.transform(opt_X[T], opt_x_transform);

  cout << "goal   transform: " << x_bar_transform.transpose() << endl;
  cout << "actual transform: " << opt_x_transform.transpose() << endl;

  PR2_SCP_Plotter plotter2(&pr2_scp, &scene, T+1);
  plotter2.draw_trajectory(opt_X, c_green);
  //cout << opt_X[T] << endl;



  scene.startViewer();
  scene.startLoop();
  scene.idle(true);

}
