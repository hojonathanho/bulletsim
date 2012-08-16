#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/clock.h"
#include "simulation/util.h"
#include "utils/interpolation.h"
#include "utils/vector_io.h"
#include <algorithm>
#include "gurobi_c++.h"
#include "kinematics_utils.h"
#include "traj_costs.h"
#include "utils_scp.h"
#include "simulation/fake_gripper.h"
#include "utils/logging.h"
using namespace std;
using namespace Eigen;
using namespace util;



struct LocalConfig : Config {
  static int plotEvery;
  static int nSteps;
  static int nIter;
  static int startPosture;
  static int endPosture;
  static int plotDecimation;

  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("plotEvery", &plotEvery, "visualize every x iterations. 0 means don't plot"));
    params.push_back(new Parameter<int>("nSteps", &nSteps, "n samples of trajectory"));
    params.push_back(new Parameter<int>("nIter", &nIter, "num iterations"));
    params.push_back(new Parameter<int>("startPosture", &startPosture, "start posture"));
    params.push_back(new Parameter<int>("endPosture", &endPosture, "end posture"));
    params.push_back(new Parameter<int>("plotDecimation", &plotDecimation, "plot every k grippers"));
  }
};
int LocalConfig::plotEvery=10;
int LocalConfig::nSteps = 100;
int LocalConfig::nIter = 100;
int LocalConfig::startPosture=3;
int LocalConfig::endPosture=1;
int LocalConfig::plotDecimation=5;


Scene* scenePtr;
void togglePlaytime() {
	scenePtr->loopState.looping = !scenePtr->loopState.looping;
	if (scenePtr->loopState.looping) scenePtr->startLoop();
}

const static float postures[][7] = {
		{-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0}, // 0=untucked
		{0.062, 	1.287, 		0.1 , -1.554, -3.011, 		-0.268, 2.988}, //1=tucked
		{-0.33, -0.35,  -2.59, -0.15,  -0.59, -1.41, 0.27}, //2=up
		{-1.832,  -0.332,   -1.011,  -1.437,   -1.1  ,  -2.106,  3.074}, //3=side
		{0, 0, 0, 0, 0, 0, 0}}; //4=outstretched



int main(int argc, char *argv[]) {
	GeneralConfig::scale = 1.;
	BulletConfig::friction = 2; // for if you're shooting blocks

	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	const float table_height = .65;
	const float table_thickness = .1;

	Scene scene;
	scenePtr = &scene;
	BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.75, .85, table_thickness / 2), btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1, 0, table_height - table_thickness / 2))));
	table->setColor(0, 0, 1, 1);
	scene.env->add(table);
	PR2Manager pr2m(scene);
	RaveRobotObject::Ptr pr2 = pr2m.pr2;
	RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;
	BOOST_FOREACH(BulletObject::Ptr child, pr2->children) {
		if	(child) {
			btRigidBody* rb = child->rigidBody.get();
			scene.env->bullet->dynamicsWorld->removeRigidBody(rb);
		}

	}

  GetArmToJointGoal planner;
  planner.setup(rarm, pr2, scene.env->bullet->dynamicsWorld);
	int nJoints = armJoints.size();
	
  VectorXf startJoints = Map<const VectorXf>(postures[LocalConfig::startPosture], nJoints);
  VectorXf endJoints = Map<const VectorXf>(postures[LocalConfig::endPosture], nJoints);
  planner.setProblem(startJoints, nJoints, LocalConfig::nSteps);

	scene.startViewer();

	scene.addVoidKeyCallback('z', &togglePlaytime);

  planner.optimize();
  scene.idel(true);

}
