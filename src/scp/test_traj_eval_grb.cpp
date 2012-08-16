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
	if (scenePtr->loopState.looping) scenePtr->startFixedTimestepLoop(.05);
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


#if 0
  float boxSize=.1;
	btVector3 loc0 = btVector3(1, 0, table_height + boxSize/2 + .2);
	BoxObject::Ptr box0(new BoxObject(0, GeneralConfig::scale * btVector3(boxSize, boxSize, boxSize), btTransform(btQuaternion(0, 0, 0, 1), METERS*loc0)));
	BoxObject::Ptr box1(new BoxObject(0, GeneralConfig::scale * btVector3(boxSize, boxSize, boxSize), btTransform(btQuaternion(0, 0, 0, 1), METERS*(loc0 + btVector3(2*boxSize, 0, 0)))));
	scene.env->add(box0);
	scene.env->add(box1);
	CollisionCollector cc;
	scene.env->bullet->dynamicsWorld->contactTest(box0->rigidBody.get(), cc);
	cout << "margin: " << box0->rigidBody->getCollisionShape()->getMargin() << endl;
	if (cc.m_collisions.size() > 0) {
		Collision coll0 = cc.m_collisions[0];
		cout << "distance: " << coll0.m_distance;
	}
	else cout << "no collisions" << endl;
	scene.startViewer();
	scene.idle(true);
	return 0;
#endif

	vector<KinBody::JointPtr> armJoints;
	vector<KinBody::LinkPtr> armLinks;
	vector<int> chainDepthOfBodies;

	getArmKinInfo(pr2->robot, rarm->manip, armLinks, armJoints, chainDepthOfBodies);

	vector<btRigidBody*> armBodies;
	BOOST_FOREACH(KinBody::LinkPtr& link, armLinks) {
		armBodies.push_back(pr2->associatedObj(link)->rigidBody.get());
	}


	int nSteps = LocalConfig::nSteps;
	int nJoints = armJoints.size();
	MatrixXf startEndJoints(2, nJoints);
	startEndJoints.row(0) = Map<const VectorXf>(postures[LocalConfig::startPosture], nJoints);
	startEndJoints.row(1) = Map<const VectorXf>(postures[LocalConfig::endPosture], nJoints);
	MatrixXf jointTraj = interp2d(VectorXf::LinSpaced(nSteps, 0, 1), VectorXf::LinSpaced(2,0,1), startEndJoints);

	VectorXf maxStepMvmt = 2*(startEndJoints.row(1) - startEndJoints.row(0)).array().abs() / nSteps;
	maxStepMvmt = maxStepMvmt.cwiseMax(VectorXf::Constant(nJoints, 2*SIMD_PI/nSteps));

	GRBEnv env;
	if (GeneralConfig::verbose > 0) env.set(GRB_IntParam_OutputFlag, 0);
	GRBModel model(env);
	BasicArray<GRBVar> trajIncs(jointTraj.rows(),jointTraj.cols());
	for (int iRow=1; iRow<jointTraj.rows()-1; iRow++)
		for (int iCol=0; iCol<jointTraj.cols(); iCol++) {
//			straightLineDist = fabs(startEndJoints(0,iCol) - startEndJoints(1,iCol));
			trajIncs.at(iRow, iCol) = model.addVar(0,0,0,GRB_CONTINUOUS);
		}
	model.update();

	vector<FakeGripper::Ptr> plotGrippers;
	for (int iStep=0; iStep<nSteps; iStep += LocalConfig::plotDecimation) {
		FakeGripper::Ptr fakeGripper(new FakeGripper(rarm));
		plotGrippers.push_back(fakeGripper);
		scene.env->osg->root->addChild(fakeGripper->m_node);
	}
	PlotCurve::Ptr trajCurve(new PlotCurve(3));
	trajCurve->m_defaultColor = osg::Vec4f(0,1,0,1);
	scene.env->osg->root->addChild(trajCurve);

	TrajectoryCost trajCost(pr2->robot, scene.env->bullet->dynamicsWorld, armLinks, armBodies, armJoints, chainDepthOfBodies);
	scene.startViewer();

	scene.addVoidKeyCallback('z', &togglePlaytime);


	for (int iter=0; iter < LocalConfig::nIter; ++iter) {
		LOG_INFO("iteration " << iter);


		if (LocalConfig::plotDecimation) {
			vector<btTransform> transforms(plotGrippers.size());
			vector<btVector3> origins(plotGrippers.size());
			for (int iPlot=0; iPlot<plotGrippers.size(); ++iPlot) {
				rarm->setDOFValues(toDoubleVec(jointTraj.row(iPlot * LocalConfig::plotDecimation)));
				transforms[iPlot] = rarm->getTransform();
				origins[iPlot] = transforms[iPlot].getOrigin();
			}
			rarm->setDOFValues(toDoubleVec(jointTraj.row(0)));
			trajCurve->setPoints(origins);
			for (int iPlot = 0; iPlot<plotGrippers.size(); ++iPlot)
				plotGrippers[iPlot]->setTransform(transforms[iPlot]);
			scene.step(0);
//			scene.idle(true);
		}

		StartClock();
		MatrixXf trajCostGrad = Eigen::Map<MatrixXf>(trajCost.calcDeriv(Map<VectorXf>(jointTraj.data(), jointTraj.rows()*jointTraj.cols())).data(), nSteps, nJoints);
		LOG_INFO("gradient time: " << GetClock());


		try	{
			StartClock();
			GRBQuadExpr pathLengthCost;
			GRBLinExpr linearizedCollisionCost;
			MatrixXd trajCostCoefs= trajCostGrad.cast<double>();
			linearizedCollisionCost.addTerms(trajCostCoefs.block(1,0,nSteps-2,nJoints).data(), trajIncs.m_data.data()+nJoints, (nSteps-2)*nJoints);

			for (int iStep=1; iStep < nSteps; ++iStep) {
				for (int iJoint=0; iJoint < 7; ++iJoint) {
					if (iStep < nSteps-1) {
						trajIncs.at(iStep, iJoint).set(GRB_DoubleAttr_LB, jointTraj(iStep, iJoint)-maxStepMvmt(iJoint)/5);
						trajIncs.at(iStep, iJoint).set(GRB_DoubleAttr_UB, jointTraj(iStep, iJoint)+maxStepMvmt(iJoint)/5);
					}
					GRBLinExpr diff;
					if (iStep  == 1) diff = trajIncs.at(iStep,iJoint) - jointTraj(iStep-1, iJoint);
					else if (iStep < nSteps - 1) diff = trajIncs.at(iStep, iJoint) - trajIncs.at(iStep-1,iJoint);
					else if (iStep == nSteps - 1) diff = jointTraj(iStep, iJoint) - trajIncs.at(iStep-1, iJoint);
					else assert(0);
//					float curdiff = jointTraj(iStep, iJoint) - jointTraj(iStep-1, iJoint);
//					cout << iStep << " " << iJoint << " curdiff " << curdiff << endl;
					model.addConstr(diff <= maxStepMvmt(iJoint));
					model.addConstr(diff >= -maxStepMvmt(iJoint));
					pathLengthCost += .5 * nSteps * diff * diff;
				}
			}
			model.setObjective(linearizedCollisionCost + pathLengthCost);


			LOG_INFO("optimization setup time: " << GetClock());
			StartClock();
			model.optimize();
			LOG_INFO("optimization time: " << GetClock());
			for (int iStep=1; iStep < nSteps-1; ++iStep) {
				for (int iJoint=0; iJoint < 7; ++iJoint) {
					jointTraj(iStep, iJoint) = trajIncs.at(iStep, iJoint).get(GRB_DoubleAttr_X);
				}
			}
		}
		catch (GRBException e) {
			cout << "GRB error: " << e.getMessage() << endl;
			throw;
		}

	}
}
