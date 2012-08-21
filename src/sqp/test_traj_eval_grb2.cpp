#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "sqp_algorithm.h"
using namespace std;
using namespace Eigen;
using namespace util;
using boost::shared_ptr;


struct LocalConfig : Config {
  static int nSteps;
  static int nIter;
  static int startPosture;
  static int endPosture;
  static int plotDecimation;
  static int plotType;

  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("nSteps", &nSteps, "n samples of trajectory"));
    params.push_back(new Parameter<int>("nIter", &nIter, "num iterations"));
    params.push_back(new Parameter<int>("startPosture", &startPosture, "start posture"));
    params.push_back(new Parameter<int>("endPosture", &endPosture, "end posture"));
    params.push_back(new Parameter<int>("plotDecimation", &plotDecimation, "plot every k grippers"));
    params.push_back(new Parameter<int>("plotType", &plotType, "0: grippers, 1: arms"));
  }
};
int LocalConfig::nSteps = 100;
int LocalConfig::nIter = 100;
int LocalConfig::startPosture=3;
int LocalConfig::endPosture=1;
int LocalConfig::plotDecimation=5;
int LocalConfig::plotType = 1;

Scene* scenePtr;
void togglePlaytime() {
	scenePtr->loopState.looping = !scenePtr->loopState.looping;
	if (scenePtr->loopState.looping) scenePtr->startLoop();
}

const static double postures[][7] = {
		{-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0}, // 0=untucked
		{0.062, 	1.287, 		0.1 , -1.554, -3.011, 		-0.268, 2.988}, //1=tucked
		{-0.33, -0.35,  -2.59, -0.15,  -0.59, -1.41, 0.27}, //2=up
		{-1.832,  -0.332,   -1.011,  -1.437,   -1.1  ,  -2.106,  3.074}, //3=side
		{0, 0, 0, 0, 0, 0, 0}}; //4=outstretched



int main(int argc, char *argv[]) {
	GeneralConfig::scale = 1.;
	BulletConfig::friction = 2; // for if you're shooting blocks
	BulletConfig::margin = .01;
	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(LocalConfig());
	parser.read(argc, argv);

	const float table_height = .65;
	const float table_thickness = .1;

	if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

	Scene scene;
	scenePtr = &scene;
	BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.85, .85, table_thickness / 2), btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.1, 0, table_height - table_thickness / 2))));
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

	int nJoints = 7;
    VectorXd startJoints = Map<const VectorXd>(postures[LocalConfig::startPosture], nJoints);
    VectorXd endJoints = Map<const VectorXd>(postures[LocalConfig::endPosture], nJoints);

    TIC();
	ComponentizedArmPlanningProblem planner;
	planner.setup(rarm, pr2, scene.env->bullet->dynamicsWorld);
	MatrixXd initTraj = makeTraj(startJoints, endJoints, LocalConfig::nSteps);
	shared_ptr<LengthConstraintAndCost> lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj),.5));
	shared_ptr<CollisionCost> cc(new CollisionCost(true, true, planner.m_cce.get(), -BulletConfig::linkPadding/2, 5));
	shared_ptr<JointBounds> jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj)/5, rarm->manip));
	planner.addComponent(lcc);
	planner.addComponent(cc);
	planner.addComponent(jb);
	planner.initialize(initTraj);
	LOG_INFO_FMT("setup time: %.2f", TOC());
#if 0
  GetArmToJointGoal planner;
  planner.setup(rarm, pr2, scene.env->bullet->dynamicsWorld);
  planner.setProblem(startJoints, endJoints, LocalConfig::nSteps);
#endif


  TrajPlotter::Ptr plotter;
  if (LocalConfig::plotType == 0) {
	  plotter.reset(new GripperPlotter(rarm, &scene, 1));
	  planner.setPlotter(plotter);
  }
  else if (LocalConfig::plotType == 1) {
	  plotter.reset(new ArmPlotter(rarm, &scene, *planner.m_cce, LocalConfig::plotDecimation));
  }
  else throw std::runtime_error("invalid plot type");

  planner.setPlotter(plotter);

  scene.startViewer();

  planner.optimize(LocalConfig::nIter);
  scene.idle(true);

}
