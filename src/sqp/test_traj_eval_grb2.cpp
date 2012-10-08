#include "simulation/simplescene.h"
#include "simulation/config_bullet.h"
#include "robots/pr2.h"
#include "simulation/bullet_io.h"
#include <boost/foreach.hpp>
#include "utils/vector_io.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "sqp_algorithm.h"
#include "config_sqp.h"
#include <osg/Depth>
using namespace std;
using namespace Eigen;
using namespace util;


#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>

void makeFullyTransparent(BulletObject::Ptr obj) {
  osg::Depth* depth = new osg::Depth;
  depth->setWriteMask( false );
  obj->node->getOrCreateStateSet()->setAttributeAndModes( depth, osg::StateAttribute::ON );
}


void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, 2);
  exit(1);
}

struct LocalConfig : Config {
  static int nSteps;
  static int nIter;
  static int startPosture;
  static int endPosture;
  static int plotType;

  LocalConfig() : Config() {
    params.push_back(new Parameter<int>("nSteps", &nSteps, "n samples of trajectory"));
    params.push_back(new Parameter<int>("nIter", &nIter, "num iterations"));
    params.push_back(new Parameter<int>("startPosture", &startPosture, "start posture"));
    params.push_back(new Parameter<int>("endPosture", &endPosture, "end posture"));
    params.push_back(new Parameter<int>("plotType", &plotType, "0: grippers, 1: arms"));
  }
};
int LocalConfig::nSteps = 100;
int LocalConfig::nIter = 100;
int LocalConfig::startPosture=3;
int LocalConfig::endPosture=1;
int LocalConfig::plotType = 1;

const static double postures[][7] = {
		{-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0}, // 0=untucked
		{0.062, 	1.287, 		0.1 , -1.554, -3.011, 		-0.268, 2.988}, //1=tucked
		{-0.33, -0.35,  -2.59, -0.15,  -0.59, -1.41, 0.27}, //2=up
		{-1.832,  -0.332,   -1.011,  -1.437,   -1.1  ,  -2.106,  3.074}, //3=side
		{0, 0, 0, 0, 0, 0, 0}}; //4=outstretched



void removeBodiesFromBullet(vector<BulletObject::Ptr> objs, btDynamicsWorld* world) {
  BOOST_FOREACH(BulletObject::Ptr obj, objs) {
    if (obj && obj->rigidBody)
      world->removeRigidBody(obj->rigidBody.get());
  }
}


int main(int argc, char *argv[]) {

  signal(SIGABRT, handler);   // install our handler

	GeneralConfig::scale = 1.;
//	BulletConfig::margin = .01;
	Parser parser;
	parser.addGroup(GeneralConfig());
	parser.addGroup(BulletConfig());
	parser.addGroup(LocalConfig());
	parser.addGroup(SQPConfig());
	parser.read(argc, argv);

	const float table_height = .65;
	const float table_thickness = .06;

	if (GeneralConfig::verbose > 0) getGRBEnv()->set(GRB_IntParam_OutputFlag, 0);

	Scene scene;
	util::setGlobalEnv(scene.env);
	BoxObject::Ptr table(new BoxObject(0, GeneralConfig::scale * btVector3(.85, .55, table_thickness / 2), btTransform(btQuaternion(0, 0, 0, 1), GeneralConfig::scale * btVector3(1.1, 0, table_height - table_thickness / 2))));
	scene.env->add(table);
	PR2Manager pr2m(scene);
	RaveRobotObject::Ptr pr2 = pr2m.pr2;
	pr2->setColor(0,1,1,.4);
	table->setColor(0,0,0,.3);
	RaveRobotObject::Manipulator::Ptr rarm = pr2m.pr2Right;
	removeBodiesFromBullet(pr2->children, scene.env->bullet->dynamicsWorld);
	BOOST_FOREACH(BulletObjectPtr obj, pr2->children) if(obj) makeFullyTransparent(obj);
	makeFullyTransparent(table);

	BulletRaveSyncher brs = syncherFromArm(rarm);

	int nJoints = 7;
    VectorXd startJoints = Map<const VectorXd>(postures[LocalConfig::startPosture], nJoints);
    VectorXd endJoints = Map<const VectorXd>(postures[LocalConfig::endPosture], nJoints);

    TIC();
	PlanningProblem prob;
	MatrixXd initTraj = makeTraj(startJoints, endJoints, LocalConfig::nSteps);
	LengthConstraintAndCostPtr lcc(new LengthConstraintAndCost(true, true, defaultMaxStepMvmt(initTraj),SQPConfig::lengthCoef));
	CollisionCostPtr cc(new CollisionCost(pr2->robot, scene.env->bullet->dynamicsWorld, brs, rarm->manip->GetArmIndices(), -BulletConfig::linkPadding/2, SQPConfig::collCoef));
	JointBoundsPtr jb(new JointBounds(true, true, defaultMaxStepMvmt(initTraj)/5, rarm->manip));
	prob.initialize(initTraj, true);
  prob.addComponent(lcc);
  prob.addComponent(cc);
  prob.addComponent(jb);
	LOG_INFO_FMT("setup time: %.2f", TOC());




  TrajPlotterPtr plotter;
  if (LocalConfig::plotType == 0) {
	  plotter.reset(new GripperPlotter(rarm, &scene, 1));
  }
  else if (LocalConfig::plotType == 1) {
	  plotter.reset(new ArmPlotter(rarm, &scene, brs, SQPConfig::plotDecimation));
  }
  else throw std::runtime_error("invalid plot type");


  prob.testObjectives();

  prob.addPlotter(plotter);
  scene.startViewer();

  double tStartAll = GetClock();
  for (int iSub=0; iSub<10; ++iSub) {

    try {
      prob.optimize(LocalConfig::nIter);
    }
    catch (GRBException e) {
      cout << e.getMessage() << endl;
      handler(0);
      throw;
    }

    vector<double> insertTimes;

    TIC1();
    TrajCartCollInfo cci = continuousTrajCollisions(prob.m_currentTraj, pr2->robot, brs, scene.env->bullet->dynamicsWorld, rarm->manip->GetArmIndices(), BulletConfig::linkPadding*.75);
    LOG_INFO_FMT("continuous collision check time: %.2f", TOC());
    plotCollisions(cci, -BulletConfig::linkPadding/2);
    int nContColl=0;
    for (int i=0; i < cci.size(); ++i) {
      nContColl += cci[i].size();
      cout << cci[i].size() << " ";
      if (cci[i].size() > 0) insertTimes.push_back((prob.m_times[i] + prob.m_times[i+1])/2);
    }
    cout << endl;
    LOG_INFO_FMT("number of unsafe points: %.2i", nContColl);

    if (insertTimes.size() > 0) {
      LOG_INFO("subdividing at times" << insertTimes);
      prob.subdivide(insertTimes);
    }
    else break;
  }

  cout << "total traj opt time: " << GetClock() - tStartAll;
  prob.m_plotters[0]->clear();
  interactiveTrajPlot(prob.m_currentTraj, rarm, &brs, &scene);

}
