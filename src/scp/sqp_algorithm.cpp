#include "sqp_algorithm.h"
using namespace Eigen;
using namespace std;

void GetArmToJointGoal::setProblem(const VectorXf& startJoints, const VectorXf& endJoints, int nSteps) {
  assert(startJoints.size() == m_nJoints);
  assert(endJoints.size() == m_nJoints);
	m_currentTraj = interp2d(VectorXf::LinSpaced(nSteps, 0, 1), VectorXf::LinSpaced(2,0,1), startEndJoints);
	m_maxStepMvmt = MAX_LENGTH_RATIO*(startEndJoints.row(1) - startEndJoints.row(0)).array().abs() / nSteps;
	m_maxStepMvmt = m_maxStepMvmt.cwiseMax(VectorXf::Constant(nJoints, 2*SIMD_PI/nSteps));
  m_maxDiffPerIter = m_maxStepMvmt / 5;  /// xxx param
  m_nSteps = nSteps;
  m_model->reset();  
  m_trajVars = BasicArray<GRBVar>(m_nSteps, m_nJoints);
	for (int iRow=1; iRow < m_currentTraj.rows()-1; ++iRow) {
		for (int iCol=0; iCol< m_currentTraj.cols(); ++iCol) {
			m_trajVars.at(iRow, iCol) = model.addVar(0,0,0,GRB_CONTINUOUS);
		}
	}
  model.update();  
}

ArmPlanningProblem::ArmPlanningProblem() {
  m_grbenv = new GRBEnvironment();
  m_model = new GRBModel(*m_grbenv);
}

ArmPlanningProblem::~ArmPlanningProblem() {
  if (m_cce) delete m_cce;
  delete m_model;
  delete m_grbenv;
}

void ArmPlanningProblem::setup(RaveRobotObject::Manipulator::Ptr rrom, RaveRobotObject::Ptr rro, btCollisionWorld* world) {
	vector<KinBody::JointPtr> armJoints;
	vector<KinBody::LinkPtr> armLinks;
	vector<int> chainDepthOfBodies;
	getArmKinInfo(rro->robot, rrom->manip, armLinks, armJoints, chainDepthOfBodies);
	vector<btRigidBody*> armBodies;
	BOOST_FOREACH(KinBody::LinkPtr& link, armLinks) {
		armBodies.push_back(rro->associatedObj(link)->rigidBody.get());
	}
  m_cce = new CollisionCostEvaluator(rrom->robot, world, armLinks, armBodies, armJoints, chainDepthOfBodies);
}

void GetArmToJointGoal::doIteration() {

	StartClock();
  MatrixXf collGrad;
  float collCost;
  
  if (m_cce == NULL) throw std::runtime_error("you forgot to call setProblem");
  m_cce->calcCostAndGrad(m_currentTraj, collCost, collGrad)
	LOG_INFO("gradient time: " << GetClock());  
  
	try	{
		StartClock();
		GRBQuadExpr pathLengthCost;
		GRBLinExpr linearizedCollisionCost;
		MatrixXd trajCostCoefs= collCost.cast<double>();
		linearizedCollisionCost.addTerms(trajCostCoefs.block(1,0,nSteps-2,nJoints).data(), m_trajVars.m_data.data()+m_nJoints, (nSteps-2)*nJoints);

		for (int iStep=1; iStep < nSteps; ++iStep) {
			for (int iJoint=0; iJoint < 7; ++iJoint) {
				if (iStep < nSteps-1) {
					m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_LB, currentTraj(iStep, iJoint)-m_maxDiffPerIter(iJoint));
					m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_UB, currentTraj(iStep, iJoint)+m_maxDiffPerIter(iJoint));
				}
				GRBLinExpr diff;
				if (iStep  == 1) diff = m_trajVars.at(iStep,iJoint) - m_trajVars(iStep-1, iJoint);
				else if (iStep < nSteps - 1) diff = m_trajVars.at(iStep, iJoint) - m_trajVars.at(iStep-1,iJoint);
				else if (iStep == nSteps - 1) diff = m_trajVars(iStep, iJoint) - m_trajVars.at(iStep-1, iJoint);
				else assert(0);
				model.addConstr(diff <= maxStepMvmt(iJoint));
				model.addConstr(diff >= -maxStepMvmt(iJoint));
				pathLengthCost += .5 * nSteps * diff * diff;
			}
		}
		model.setObjective(linearizedCollisionCost + pathLengthCost);
		LOG_INFO("optimization setup time: " << GetClock());

		StartClock();
		m_model->optimize();
		LOG_INFO("optimization time: " << GetClock());

    m_currentObjective = m_model->XXX;
		for (int iStep=1; iStep < nSteps-1; ++iStep) {
			for (int iJoint=0; iJoint < 7; ++iJoint) {
				m_currentTraj(iStep, iJoint) = m_trajVars.at(iStep, iJoint).get(GRB_DoubleAttr_X);
			}
		}
	}
	catch (GRBException e) {
		cout << "GRB error: " << e.getMessage() << endl;
		throw;
	}  	
}

void GetArmToJointGoal::optimize(int maxIter) {
  if (m_atp) m_atp->plotTraj(m_currentTraj);
  for (int iter=0; iter < maxIter; ++iter) {
    doIteration();
    if (m_atp) m_atp->plotTraj(m_currentTraj);    
    LOG_INFO("iteration " << iter << " objective" << m_currentObjective);
  }
}

ArmTrajPlotter::ArmTrajPlotter(RaveRobotObject::Manipulator::Ptr rrom, osg::Group* osgRoot, int decimation) :
  m_osgRoot(osgRoot),
  m_rrom(rrom),
  m_decimation(decimation),
  m_curve(new PlotCurve(3))
{
  osgRoot->addChild(m_curve.get());
}

ArmTrajPlotter::setNumGrippers(int n) {
  if (n == m_grippers.size()) return;
  clear();
  m_grippers.resize(n);
  for (int i=0; i < n; ++i) {
		FakeGripper::Ptr fakeGripper(new FakeGripper(rarm));
		m_grippers[i] = fakeGripper;
		m_osgRoot->addChild(fakeGripper->m_node);
  }
}


ArmTrajPlotter::clear() {
  
  for (int i=0; i < m_grippers.size(); ++i) {
    osgRoot->removeChild(m_grippers[i]->m_node);		
  }
  osgRoot->removeChild(m_curve);
  m_grippers.clear();
}


void ArmTrajPlotter::plotTraj(const MatrixXf& traj) {
  setNumGrippers(traj.rows());
	vector<btTransform> transforms(m_.size());
	vector<btVector3> origins(m_.size());
	
  VectorXf curDOFVals = rrom->getDOFValues();
	for (int iPlot=0; iPlot< m_grippers.size(); ++iPlot) {
		rarm->setDOFValues(toDoubleVec(traj.row(iPlot * m_decimation)));
		transforms[iPlot] = rarm->getTransform();
		origins[iPlot] = transforms[iPlot].getOrigin();
	}
	m_rrom->setDOFValues(curDOFVals);
	
	m_curve->setPoints(origins);
	for (int iPlot = 0; iPlot < m_gripper.size(); ++iPlot)
		m_grippers[iPlot]->setTransform(transforms[iPlot]);
	scene.step(0);
}

ArmTrajPlotter::~ArmTrajPlotter() {  
  clear();
}