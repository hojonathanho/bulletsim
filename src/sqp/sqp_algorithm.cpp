#include "sqp_algorithm.h"
#include "utils/interpolation.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "simulation/simplescene.h"
#include "utils/conversions.h"
#include <typeinfo>
using namespace Eigen;
using namespace std;

static const float PATH_LENGTH_RATIO = 3;
static const float MAX_DIST_OVER_TIMESTEP_DIST = 5;

template <class T>
string getClassName(T& x) {
  string struct_classname = typeid(x).name();
  return std::string(&struct_classname[7]);
}

// http://www.gurobi.com/documentation/5.0/reference-manual/node740
static const char* grb_statuses[] = {
    "XXX", //0
     "LOADED",//1
     "OPTIMAL",//2
     "INFEASIBLE",//3
     "INF_OR_UNBD",//4
     "UNBOUDNED",//5
     "CUTOFF",//6
     "ITERATION LIMIT",//7
     "NODE_LIMIT",//8
     "TIME_LIMIT",//9
     "SOLUTION_LIMIT",//10
     "INTERRUPTED",//11
     "NUMERIC",//12
     "SUBOPTIMAL"//13
};

static GRBEnv* grbEnv = new GRBEnv();
GRBEnv* getGRBEnv() {
  return grbEnv;
}
static const GRBVar nullvar;
bool isValidVar(GRBVar& var) {return !var.sameAs(nullvar);}

Eigen::VectorXd defaultMaxStepMvmt(const Eigen::MatrixXd& traj) {
  VectorXd maxStepMvmt = PATH_LENGTH_RATIO * (traj.row(traj.rows()-1) - traj.row(0)).array().abs() / traj.rows();
  maxStepMvmt = maxStepMvmt.cwiseMax(VectorXd::Constant(traj.cols(), 2 * SIMD_PI / traj.rows()));
  return maxStepMvmt;
}

Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, int nSteps) {
  assert(startJoints.size() == endJoints.size());
  Eigen::MatrixXd startEndJoints(2, startJoints.size());
  startEndJoints.row(0) = startJoints;
  startEndJoints.row(1) = endJoints;
  return interp2d(VectorXd::LinSpaced(nSteps, 0, 1), VectorXd::LinSpaced(2, 0, 1), startEndJoints);

}

void updateTraj(const VarArray& trajVars, const VectorXb& optmask, Eigen::MatrixXd& traj) {
  for (int i=0; i < traj.rows(); ++i)
    if (optmask(i))
      for (int j=0; j < traj.cols(); ++j)
        traj(i,j) = trajVars.at(i, j).get(GRB_DoubleAttr_X);
}

void setVarsToTraj(const Eigen::MatrixXd traj, const VectorXb& optmask, VarArray& trajVars) {
  for (int i = 0; i < traj.rows(); ++i)
    if (optmask(i))
      for (int j = 0; j < traj.cols(); ++j)
        trajVars.at(i, j).set(GRB_DoubleAttr_X, traj(i, j));
}

ArmCCEPtr makeArmCCE(RaveRobotObject::Manipulator::Ptr rrom, RaveRobotObject::Ptr rro, btDynamicsWorld* world) {
  vector<KinBody::JointPtr> armJoints;
  vector<KinBody::LinkPtr> armLinks;
  vector<int> chainDepthOfBodies;
  getArmKinInfo(rro->robot, rrom->manip, armLinks, armJoints, chainDepthOfBodies);
  vector<btRigidBody*> armBodies;
  BOOST_FOREACH(KinBody::LinkPtr& link, armLinks){
    armBodies.push_back(rro->associatedObj(link)->rigidBody.get());
  }
  return ArmCCEPtr(new ArmCCE(rro->robot, world, armLinks, armBodies, armJoints, chainDepthOfBodies));
}

PlanningProblem::PlanningProblem() :
    m_model(new GRBModel(*grbEnv)) {}

void PlanningProblem::addComponent(ProblemComponent::Ptr comp) {
  assert (m_initialized);
  m_comps.push_back(comp);
  comp->m_problem = this;
  comp->onAdd();
}

void PlanningProblem::removeComponent(ProblemComponent::Ptr comp) {
  typedef vector<ProblemComponent::Ptr> ComponentList;
   ComponentList::iterator it = std::find(m_comps.begin(), m_comps.end(), comp);
   assert(it != m_comps.end());
   m_comps.erase(it);
   (*it)->onRemove();
 }


void CollisionCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  // Remove added constraints
  for (int iCnt = 0; iCnt < m_cnts.size(); ++iCnt)
    m_problem->m_model->remove(m_cnts[iCnt]);
  for (int iVar = 0; iVar < m_vars.size(); ++iVar)
    m_problem->m_model->remove(m_vars[iVar]);
  m_vars.clear();
  m_cnts.clear();

  // Actually run through trajectory and find collisions
  TrajCollisionInfo trajCollInfo = m_cce->collectCollisionInfo(traj);

  int nNear, nUnsafe, nColl;
  countCollisions(trajCollInfo, m_safeDistMinusPadding, nNear, nUnsafe, nColl);
  int nTotalProximity = nNear + nUnsafe + nColl;
  LOG_INFO_FMT("near: %i, unsafe: %i, collision: %i, total: %i", nNear, nUnsafe, nColl, nTotalProximity);


  for (int iStep = 0; iStep < traj.rows(); ++iStep)
    if (m_problem->m_optMask(iStep))
      for (int iColl = 0; iColl < trajCollInfo[iStep].first.size(); ++iColl) {
        m_vars.push_back(m_problem->m_model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,"hinge"));
  }
  m_problem->m_model->update();

  int varCount = 0;
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    if (m_problem->m_optMask(iStep)) {
      std::vector<Eigen::VectorXd>& jacs = trajCollInfo[iStep].first;
      std::vector<double>& dists = trajCollInfo[iStep].second;
      VarVector jointVars = m_problem->m_trajVars.row(iStep);
      for (int iColl = 0; iColl < trajCollInfo[iStep].first.size(); ++iColl) {
        GRBVar& hinge = m_vars[varCount];
        ++varCount;
        GRBLinExpr jacDotTheta;
        jacDotTheta.addTerms(jacs[iColl].data(), jointVars.data(), traj.cols());
        GRBConstr hingeCnt = m_problem->m_model->addConstr(hinge >= -dists[iColl] + jacDotTheta + m_safeDistMinusPadding - jacs[iColl].dot(traj.row(iStep)));
        m_cnts.push_back(hingeCnt);
      }
    }
  }
  assert(varCount == m_vars.size());

  // Create this part of the cost
  m_obj = GRBLinExpr(0);
  VectorXd coeffs = VectorXd::Constant(m_vars.size(), m_coeff);
  m_obj.addTerms(coeffs.data(), m_vars.data(), m_vars.size());

  objective += m_obj;
}

void CollisionCost::onRemove() {
  BOOST_FOREACH(GRBConstr& constr, m_cnts) m_problem->m_model->remove(constr);
  BOOST_FOREACH(GRBVar var, m_vars) m_problem->m_model->remove(var);
}

void CollisionConstraint::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  // Remove added constraints
  for (int iCnt = 0; iCnt < m_cnts.size(); ++iCnt)
    m_problem->m_model->remove(m_cnts[iCnt]);
  m_cnts.clear();

  // Actually run through trajectory and find collisions
  TrajCollisionInfo trajCollInfo = m_cce->collectCollisionInfo(traj);

  // Make a bunch of variables for hinge costs
  int nNear, nUnsafe, nColl;
  countCollisions(trajCollInfo, m_safeDistMinusPadding,nNear, nUnsafe, nColl);
  int nTotalProximity = nNear + nUnsafe + nColl;
  LOG_INFO_FMT("near: %i, unsafe: %i, collision: %i, total: %i", nNear, nUnsafe, nColl, nTotalProximity);

  // Create variables that will form the cost
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    std::vector<Eigen::VectorXd>& jacs = trajCollInfo[iStep].first;
    std::vector<double>& dists = trajCollInfo[iStep].second;
    VarVector jointVars = m_problem->m_trajVars.row(iStep);
    for (int iColl = 0; iColl < trajCollInfo[iStep].first.size(); ++iColl) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(jacs[iColl].data(), jointVars.data(), traj.rows());
      GRBConstr cnt = m_problem->m_model->addConstr(0 >= -dists[iColl] + jacDotTheta + m_safeDistMinusPadding - jacs[iColl].dot(traj.row(iStep)));
      m_cnts.push_back(cnt);
    }
  }

//  assert (m_cnts.size() == nTotalProximity);
}

void CollisionConstraint::onRemove() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts) m_problem->m_model->remove(cnt);
}

void CollisionConstraint::relax() {
//  double feasRelax(int relaxobjtype, bool minrelax, int vlen,
//                 const GRBVar* vars, const double* lbpen,
//                 const double* ubpen, int clen, const GRBConstr* constrs,
//                 const double* rhspen);

  VectorXd cntPen = VectorXd::Ones(m_cnts.size());
  m_problem->m_model->feasRelax(0, // sum of violations
  true, // minimize original objective among ones minimizing violation
  0, // vlen
  NULL, //vars
  NULL, //lbpen
  NULL, //ubpen
  m_cnts.size(), //clen
  m_cnts.data(), // constr
  cntPen.data());
}

void LengthConstraintAndCost::onAdd() {
  MatrixXd& traj = m_problem->m_currentTraj;
  for (int iStep = 1; iStep < traj.rows(); ++iStep)
    for (int iJoint = 0; iJoint < traj.cols(); ++iJoint) {
      GRBLinExpr diff;
      if (m_problem->m_optMask(iStep)) {
        diff += m_problem->m_trajVars.at(iStep, iJoint);
        assert(isValidVar(m_problem->m_trajVars.at(iStep,iJoint)));
      } else diff += traj(iStep, iJoint);
      if (m_problem->m_optMask(iStep - 1)) {
        diff -= m_problem->m_trajVars.at(iStep - 1, iJoint);
        assert(isValidVar(m_problem->m_trajVars.at(iStep-1,iJoint)));
      } else diff -= traj(iStep-1, iJoint);
      m_cnts.push_back(m_problem->m_model->addConstr(diff <= m_maxStepMvmt(iJoint)));
      m_cnts.push_back(m_problem->m_model->addConstr(diff >= -m_maxStepMvmt(iJoint)));
      m_obj += traj.rows() * (diff * diff);
    }
}

void LengthConstraintAndCost::onRemove() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts) m_problem->m_model->remove(cnt);
}

void JointBounds::onAdd() {
  MatrixXd& traj = m_problem->m_currentTraj;
  VectorXd maxStepMvmt = PATH_LENGTH_RATIO * (traj.row(traj.rows()-1) - traj.row(0)).array().abs() / traj.rows();
  maxStepMvmt = maxStepMvmt.cwiseMax(VectorXd::Constant(traj.cols(), 2 * SIMD_PI / traj.rows()));

}

void JointBounds::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    if (m_problem->m_optMask(iStep))
      for (int iJoint = 0; iJoint < traj.cols(); ++iJoint) {
        m_problem->m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_LB, fmax(traj(iStep, iJoint) - m_maxDiffPerIter(iJoint), m_jointLowerLimit(iJoint)));
        m_problem->m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_UB, fmin(traj(iStep, iJoint) + m_maxDiffPerIter(iJoint), m_jointUpperLimit(iJoint)));
      }
  }

}

#if 0
void CartesianPoseCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
  m_robot->SetActiveDOFs(m_manip->GetArmIndices());
  vector<double> prevVals;
  m_robot->GetActiveDOFValues(prevVals);
  VectorXd curVals = traj.row(m_timestep);
  m_robot->SetActiveDOFValues(toDoubleVec(curVals));
  boost::multi_array<dReal,2> jac0;
  boost::multi_array<dReal,2> rotjac0;
  m_manip->CalculateJacobian(jac0);
  m_manip->CalculateRotationJacobian(rotjac0);
  MatrixXd jac = Eigen::Map<MatrixXd>(jac0.data(), 3, m_nJoints);
  MatrixXd rotjac = Eigen::Map<MatrixXd>(rotjac0.data(), 4, m_nJoints);
  btTransform tf = util::toBtTransform(m_manip->GetTransform());
  m_robot->SetActiveDOFValues(prevVals);

  m_obj = GRBQuadExpr(0);

  if (m_posCoeff > 0) {
    Vector3d posCur = toVector3d(tf.getOrigin());
    for (int i = 0; i < 3; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(curVals.data(), m_timestepVars.data(), m_nJoints);
      GRBLinExpr erri = jac.row(i).dot(curVals) + posCur(i) - jacDotTheta - m_posTarg(i);
      m_obj += m_posCoeff * (erri * erri);
    }
  }

  if (m_rotCoeff > 0) {
    Vector4d rotCur = toVector4d(tf.getRotation());
    for (int i = 0; i < 4; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(curVals.data(), m_timestepVars.data(), m_nJoints);
      GRBLinExpr erri = rotjac.row(i).dot(curVals) + rotCur(i) - jacDotTheta - m_rotTarg(i);
      m_obj += m_rotCoeff * (erri * erri);
    }
  }


  objective += m_obj;
}


#endif



#if 0
void ArmPlanningProblem::setup(RaveRobotObject::Manipulator::Ptr rrom, RaveRobotObject::Ptr rro, btCollisionWorld* world) {
  vector<KinBody::JointPtr> armJoints;
  vector<KinBody::LinkPtr> armLinks;
  vector<int> chainDepthOfBodies;
  getArmKinInfo(rro->robot, rrom->manip, armLinks, armJoints, chainDepthOfBodies);
  vector<btRigidBody*> armBodies;
  BOOST_FOREACH(KinBody::LinkPtr& link, armLinks){  
    armBodies.push_back(rro->associatedObj(link)->rigidBody.get());
  }
  m_cce.reset(new CollisionCostEvaluator(rro->robot, world, armLinks, armBodies, armJoints, chainDepthOfBodies));
  m_nJoints = armJoints.size();
}
#endif




void PlanningProblem::doIteration() {
  GRBQuadExpr objective(0);
  TIC();
  for (int i=0; i < m_comps.size(); ++i) m_comps[i]->updateModel(m_currentTraj, objective);
  m_model->setObjective(objective);
  LOG_INFO_FMT("total problem construction time: %.2f", TOC());

  TIC1();
  m_model->optimize();
  LOG_INFO_FMT("optimization time: %.2f", TOC());

  int status = m_model->get(GRB_IntAttr_Status);
  if (status != GRB_OPTIMAL) {
    LOG_ERROR("bad grb status: " << grb_statuses[status]);
  }
  else {
    updateTraj(m_trajVars, m_optMask, m_currentTraj);
  }
}

void PlanningProblem::optimize(int maxIter) {
  BOOST_FOREACH(TrajPlotterPtr plotter, m_plotters) plotter->plotTraj(m_currentTraj);
  for (int iter = 0; iter < maxIter; ++iter) {
    doIteration();
    BOOST_FOREACH(TrajPlotterPtr plotter, m_plotters) plotter->plotTraj(m_currentTraj);
    LOG_INFO_FMT("iteration: %i, objective: %.3f",iter,m_model->get(GRB_DoubleAttr_ObjVal));
  }
}

void PlanningProblem::initialize(const Eigen::MatrixXd& initTraj, bool endFixed) {
  m_currentTraj = initTraj;
  m_trajVars = VarArray(initTraj.rows(), initTraj.cols());
  m_optMask = VectorXb::Ones(initTraj.rows());
  m_optMask(0) = false;
  if (endFixed) m_optMask(initTraj.rows()-1) = false;
  for (int iRow = 0; iRow < m_currentTraj.rows(); ++iRow) {
    if (m_optMask(iRow)) {
      for (int iCol = 0; iCol < m_currentTraj.cols(); ++iCol) {
        char namebuf[10];
        sprintf(namebuf, "j_%i_%i",iRow,iCol);
        m_trajVars.at(iRow, iCol) = m_model->addVar(0, 0, 0, GRB_CONTINUOUS,namebuf);
      }
    }
  }
  m_model->update();
  m_initialized=true;
}


GripperPlotter::GripperPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, int decimation) :
  m_scene(scene), m_osgRoot(scene->env->osg->root.get()), m_rrom(rrom), m_decimation(decimation), m_curve(new PlotCurve(3)) {
  m_osgRoot->addChild(m_curve.get());
}

void GripperPlotter::setNumGrippers(int n) {
  if (n != m_grippers.size()) {
    clear();
    m_grippers.resize(n);
    for (int i = 0; i < n; ++i) {
      FakeGripper::Ptr fakeGripper(new FakeGripper(m_rrom));
      m_grippers[i] = fakeGripper;
      m_osgRoot->addChild(fakeGripper->m_node);
    }
  }
}

void GripperPlotter::clear() {

  for (int i = 0; i < m_grippers.size(); ++i) {
    m_osgRoot->removeChild(m_grippers[i]->m_node);
  }
  m_grippers.clear();
}

void GripperPlotter::plotTraj(const MatrixXd& traj) {
  setNumGrippers(traj.rows() / m_decimation);
  vector<btTransform> transforms(m_grippers.size());
  vector<btVector3> origins(m_grippers.size());

  vector<double> curDOFVals = m_rrom->getDOFValues();
  for (int iPlot = 0; iPlot < m_grippers.size(); ++iPlot) {
    m_rrom->setDOFValues(toDoubleVec(traj.row(iPlot * m_decimation)));
    transforms[iPlot] = m_rrom->getTransform();
    origins[iPlot] = transforms[iPlot].getOrigin();
  }
  m_rrom->setDOFValues(curDOFVals);

  m_curve->setPoints(origins);
  for (int iPlot = 0; iPlot < m_grippers.size(); ++iPlot)
    m_grippers[iPlot]->setTransform(transforms[iPlot]);
  m_scene->step(0);
}

GripperPlotter::~GripperPlotter() {
  clear();
  m_osgRoot->removeChild(m_curve);
}

ArmPlotter::ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, vector<KinBody::LinkPtr>& links, BulletRaveSyncher& syncher, int decimation) {
  vector<BulletObject::Ptr> armObjs;
  BOOST_FOREACH(KinBody::LinkPtr link, links) armObjs.push_back(rrom->robot->associatedObj(link));
  init(rrom, armObjs, scene, &syncher, decimation);
}
ArmPlotter::ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, const std::vector<BulletObject::Ptr>& origs, Scene* scene, BulletRaveSyncher*syncher, int decimation) {
  init(rrom, origs, scene, syncher, decimation);
}

void ArmPlotter::init(RaveRobotObject::Manipulator::Ptr rrom, const std::vector<BulletObject::Ptr>& origs, Scene* scene, BulletRaveSyncher* syncher, int decimation) {
  m_rrom = rrom;
  m_scene = scene;
  m_osgRoot = scene->env->osg->root.get();
  m_origs = origs;
  m_syncher = syncher;
  m_decimation = decimation;
  m_curve = new PlotCurve(3);
  m_curve->m_defaultColor=osg::Vec4f(0,1,0,1);
  m_axes.reset(new PlotAxes());
  m_curve->m_defaultColor=osg::Vec4f(0,1,0,1);
  m_scene->env->add(m_axes);
  m_osgRoot->addChild(m_curve.get());
}

ArmPlotter::~ArmPlotter() {
  m_osgRoot->removeChild(m_curve);
  m_scene->env->remove(m_axes);
}

void ArmPlotter::setLength(int nPlots) {
  if (m_fakes.m_nRow == nPlots) return;
  m_fakes.resize(nPlots, m_origs.size());
  for (int iPlot = 0; iPlot < nPlots; ++iPlot) {
    for (int iObj = 0; iObj < m_origs.size(); ++iObj) {
      FakeObjectCopy::Ptr& fake = m_fakes.at(iPlot, iObj);
      fake.reset(new FakeObjectCopy(m_origs[iObj]));
      fake->makeChildOf(m_osgRoot);
      float frac = (float) iPlot / nPlots;
      fake->setColor(osg::Vec4f(frac, 0, 1 - frac, .35));
    }
  }
}

vector<btVector3> getGripperPositions(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr rrom) {
  vector<btVector3> out;
  for (int i=0; i < traj.rows(); ++i) {
    rrom->setDOFValues(toDoubleVec(traj.row(i)));
    out.push_back(rrom->getTransform().getOrigin());
  }
  return out;
}

void ArmPlotter::plotTraj(const MatrixXd& traj) {
  setLength(traj.rows() / m_decimation);

  vector<double> curDOFVals = m_rrom->getDOFValues();
  for (int iPlot = 0; iPlot < m_fakes.m_nRow; ++iPlot) {
    m_rrom->setDOFValues(toDoubleVec(traj.row(iPlot * m_decimation)));
    m_syncher->updateBullet();
    for (int iObj = 0; iObj < m_origs.size(); ++iObj) {
      m_fakes.at(iPlot, iObj)->setTransform(m_origs[iObj]->rigidBody->getCenterOfMassTransform());
    }
  }

  m_rrom->setDOFValues(toDoubleVec(traj.row(traj.rows()-1)));
  m_axes->setup(m_rrom->getTransform(), .1*METERS);

  vector<btVector3> gripperPositions = getGripperPositions(traj, m_rrom);
  m_curve->setPoints(gripperPositions);


  m_rrom->setDOFValues(curDOFVals);
  m_syncher->updateBullet();


  TIC();
  m_scene->step(0);
  LOG_INFO_FMT("draw time: %.3f", TOC());
}

void interactiveTrajPlot(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, BulletRaveSyncher* syncher, Scene* scene) {

  vector<double> curDOFVals = arm->getDOFValues();
  for (int iStep=0; iStep < traj.rows(); ++iStep) {
    arm->setDOFValues(toDoubleVec(traj.row(iStep)));
    syncher->updateBullet();
    printf("press p to continue\n");
    scene->step(0);
    scene->idle(true);
  }
  arm->setDOFValues(curDOFVals);
  syncher->updateBullet();
}



