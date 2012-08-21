#include "sqp_algorithm.h"
#include "utils/interpolation.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "simulation/simplescene.h"
#include "utils/conversions.h"
using namespace Eigen;
using namespace std;

static const float PATH_LENGTH_RATIO = 3;
static const float MAX_DIST_OVER_TIMESTEP_DIST = 5;

static GRBEnv* grbEnv = new GRBEnv();
GRBEnv* getGRBEnv() {
  return grbEnv;
}

void ProblemComponent::init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model) {
  assert(trajVars.rows() == traj.rows());
  assert(trajVars.cols() == traj.cols());
  m_trajVars = trajVars;
  m_model = model;
  m_nSteps = traj.rows();
  m_nJoints = traj.cols();
}

void CollisionCost::update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  // Remove added constraints
  for (int iCnt = 0; iCnt < m_cnts.size(); ++iCnt)
    m_model->remove(m_cnts[iCnt]);
  for (int iVar = 0; iVar < m_vars.size(); ++iVar)
    m_model->remove(m_vars[iVar]);
  m_vars.clear();
  m_cnts.clear();

  // Actually run through trajectory and find collisions
  TrajCollisionInfo trajCollInfo = m_cce->collectCollisionInfo(traj);

  int startStep = (m_startFixed ? 1 : 0);
  int endStep = (m_endFixed ? m_nSteps - 1 : m_nSteps);

  // Make a bunch of variables for hinge costs
  int nNear(0), nUnsafe(0), nColl(0);
  for (int iStep = startStep; iStep < m_nSteps; ++iStep) {
    vector<double>& dists = trajCollInfo[iStep].second;
    for (int iColl=0; iColl < trajCollInfo[iStep].first.size(); ++iColl) {
      if (dists[iColl] < -BulletConfig::linkPadding) ++nColl;
      else if (dists[iColl] < m_safeDistMinusPadding) ++nUnsafe;
      else if (dists[iColl] < 0) ++nNear;
    }
  }
  int nTotalProximity = nNear + nUnsafe + nColl;
  LOG_INFO_FMT("near: %i, unsafe: %i, collision: %i, total: %i", nNear, nUnsafe, nColl, nTotalProximity);

  for (int iStep = startStep; iStep < endStep; ++iStep) for (int iColl = 0; iColl < trajCollInfo[iStep].first.size(); ++iColl) {
    m_vars.push_back(m_model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,"hinge"));
  }
  m_model->update();

  int varCount = 0;
  // Create variables that will form the cost
  for (int iStep = startStep; iStep < endStep; ++iStep) {
    std::vector<Eigen::VectorXd>& jacs = trajCollInfo[iStep].first;
    std::vector<double>& dists = trajCollInfo[iStep].second;
    VarVector jointVars = m_trajVars.row(iStep);
    for (int iColl = 0; iColl < trajCollInfo[iStep].first.size(); ++iColl) {
      GRBVar& hinge = m_vars[varCount];
      ++varCount;
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(jacs[iColl].data(), jointVars.data(), m_nJoints);
      GRBConstr hingeCnt = m_model->addConstr(hinge >= -dists[iColl] + jacDotTheta + m_safeDistMinusPadding - jacs[iColl].dot(traj.row(iStep)));
      m_cnts.push_back(hingeCnt);
    }
  }

  // Create this part of the cost
  m_obj = GRBLinExpr(0);
  VectorXd coeffs = m_coeff * VectorXd::Ones(m_vars.size());
  m_obj.addTerms(coeffs.data(), m_vars.data(), m_vars.size());

  objective += m_obj;
}

void LengthConstraintAndCost::init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model) {
  ProblemComponent::init(traj, trajVars, model);

  VectorXb varMask = VectorXb::Ones(m_nSteps);
  if (m_startFixed) varMask(0) = false;
  if (m_endFixed) varMask(varMask.size()-1) = false;

  for (int iStep = 1; iStep < m_nSteps; ++iStep) {
    for (int iJoint = 0; iJoint < m_nJoints; ++iJoint) {
      GRBLinExpr diff;
      if (varMask(iStep)) diff += trajVars.at(iStep, iJoint);
      else diff += traj(iStep, iJoint);
      if (varMask(iStep - 1)) diff -= trajVars.at(iStep - 1, iJoint);
      else diff -= traj(iStep, iJoint);
      m_model->addConstr(diff <= m_maxStepMvmt(iJoint));
      m_model->addConstr(diff >= -m_maxStepMvmt(iJoint));
      m_obj += (m_coeff * m_nSteps) * (diff * diff);

    }
  }
}

void JointBounds::init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model) {
  ProblemComponent::init(traj, trajVars, model);
  VectorXd maxStepMvmt = PATH_LENGTH_RATIO * (traj.row(traj.rows()-1) - traj.row(0)).array().abs() / m_nSteps;
  maxStepMvmt = maxStepMvmt.cwiseMax(VectorXd::Constant(m_nJoints, 2 * SIMD_PI / m_nSteps));

}

void JointBounds::update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  for (int iStep = (m_startFixed ? 1 : 0); iStep < (m_endFixed ? m_nSteps-1 : m_nSteps); ++iStep) {
    for (int iJoint = 0; iJoint < m_nJoints; ++iJoint) {
      m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_LB, fmax(traj(iStep, iJoint) - m_maxDiffPerIter(iJoint), m_jointLowerLimit(iJoint)));
      m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_UB, fmin(traj(iStep, iJoint) + m_maxDiffPerIter(iJoint), m_jointUpperLimit(iJoint)));
    }
  }

}


void CartesianPoseCost::update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
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



ArmPlanningProblem::ArmPlanningProblem() :
  m_model(new GRBModel(*grbEnv)) {
}

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

void ArmPlanningProblem::optimize(int maxIter) {
  if (m_atp) m_atp->plotTraj(m_currentTraj);
  for (int iter = 0; iter < maxIter; ++iter) {
    doIteration();
    if (m_atp) m_atp->plotTraj(m_currentTraj);
    LOG_INFO_FMT("iteration: %i, objective: %.3f",iter,m_model->get(GRB_DoubleAttr_ObjVal));
  }
}

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


void updateTraj(const VarArray& trajVars, Eigen::MatrixXd& traj) {
  for (int i=1; i < traj.rows()-1; ++i)
    for (int j=0; j < traj.cols(); ++j)
      traj(i,j) = trajVars.at(i, j).get(GRB_DoubleAttr_X);
}

void setVarsToTraj(const Eigen::MatrixXd traj, VarArray& trajVars) {
  for (int i=1; i < traj.rows()-1; ++i)
    for (int j=0; j < traj.cols(); ++j)
      trajVars.at(i, j).set(GRB_DoubleAttr_X, traj(i,j));
}



void GetArmToJointGoal::setProblem(const VectorXd& startJoints, const VectorXd& endJoints, int nSteps) {
  assert(startJoints.size() == m_nJoints);
  assert(endJoints.size() == m_nJoints);
  Eigen::MatrixXd startEndJoints(2, m_nJoints);
  startEndJoints.row(0) = startJoints;
  startEndJoints.row(1) = endJoints;
  m_currentTraj = interp2d(VectorXd::LinSpaced(nSteps, 0, 1), VectorXd::LinSpaced(2, 0, 1), startEndJoints);
  m_maxStepMvmt = PATH_LENGTH_RATIO * (endJoints - startJoints).array().abs() / nSteps;
  m_maxStepMvmt = m_maxStepMvmt.cwiseMax(VectorXd::Constant(m_nJoints, 2 * SIMD_PI / nSteps));
  m_maxDiffPerIter = m_maxStepMvmt / MAX_DIST_OVER_TIMESTEP_DIST;
  m_model->reset();
  m_trajVars = VarArray(nSteps, m_nJoints);

  for (int iRow = 1; iRow < m_currentTraj.rows() - 1; ++iRow) {
    for (int iCol = 0; iCol < m_currentTraj.cols(); ++iCol) {
      m_trajVars.at(iRow, iCol) = m_model->addVar(0, 0, 0, GRB_CONTINUOUS);
    }
  }
  m_model->update();

  for (int iStep = 1; iStep < nSteps; ++iStep) {
    for (int iJoint = 0; iJoint < m_nJoints; ++iJoint) {
      GRBLinExpr diff;
      if (iStep == 1) diff = m_trajVars.at(iStep, iJoint) - m_currentTraj(iStep - 1, iJoint);
      else if (iStep < nSteps - 1) diff = m_trajVars.at(iStep, iJoint) - m_trajVars.at(iStep - 1, iJoint);
      else if (iStep == nSteps - 1) diff = m_currentTraj(iStep, iJoint) - m_trajVars.at(iStep - 1, iJoint);
      else assert(0);
      m_model->addConstr(diff <= m_maxStepMvmt(iJoint));
      m_model->addConstr(diff >= -m_maxStepMvmt(iJoint));
      m_pathLengthCost += .5 * nSteps * diff * diff;
    }
  }

}

void GetArmToJointGoal::doIteration() {

  MatrixXd collGrad;
  double collCost;

  if (m_cce == NULL) throw std::runtime_error("you forgot to call setProblem");
  TIC();
  m_cce->calcCostAndGrad(m_currentTraj, collCost, collGrad);
  LOG_INFO("gradient time: " << TOC());

  int nSteps = m_currentTraj.rows();

  try {

    TIC1();
    MatrixXd trajCostCoefs = collGrad.cast<double> ();
    GRBLinExpr linearizedCollisionCost;
    linearizedCollisionCost.addTerms(trajCostCoefs.block(1, 0, nSteps - 2, m_nJoints).data(), m_trajVars.m_data.data() + m_nJoints, (nSteps - 2) * m_nJoints);

    for (int iStep = 1; iStep < nSteps; ++iStep) {
      for (int iJoint = 0; iJoint < m_nJoints; ++iJoint) {
        if (iStep < nSteps - 1) {
          vector<double> ll, ul;
          m_cce->m_joints[iJoint]->GetLimits(ll, ul);
          m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_LB, fmax(m_currentTraj(iStep, iJoint) - m_maxDiffPerIter(iJoint), ll[0]));
          m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_UB, fmin(m_currentTraj(iStep, iJoint) + m_maxDiffPerIter(iJoint), ul[0]));
        }
      }
    }
    m_model->setObjective(linearizedCollisionCost + m_pathLengthCost);
    LOG_INFO("optimization setup time: " << TOC());

    TIC1();
    m_model->optimize();
    LOG_INFO("optimization time: " << TOC());

    for (int iStep = 1; iStep < nSteps - 1; ++iStep) {
      for (int iJoint = 0; iJoint < 7; ++iJoint) {
        m_currentTraj(iStep, iJoint) = m_trajVars.at(iStep, iJoint).get(GRB_DoubleAttr_X);
      }
    }
  } catch (GRBException e) {
    cout << "GRB error: " << e.getMessage() << endl;
    throw;
  }
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

ArmPlotter::ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, CollisionCostEvaluator& cce, int decimation) {
  vector<BulletObject::Ptr> armObjs;
  BOOST_FOREACH(KinBody::LinkPtr link, cce.m_links)
armObjs  .push_back(rrom->robot->associatedObj(link));
  init(rrom, armObjs, scene, &cce.m_syncher, decimation);
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


void ComponentizedArmPlanningProblem::doIteration() {
  GRBQuadExpr objective(0);
  TIC();
  for (int i=0; i < m_comps.size(); ++i) m_comps[i]->update(m_currentTraj, objective);
  m_model->setObjective(objective);
  LOG_INFO_FMT("total problem construction time: %.2f", TOC());

  TIC1();
  m_model->optimize();
  LOG_INFO_FMT("optimization time: %.2f", TOC());

  if (m_atp) m_atp->plotTraj(m_currentTraj);

  updateTraj(m_trajVars, m_currentTraj);
}

void ComponentizedArmPlanningProblem::initialize(Eigen::MatrixXd& initTraj) {
  m_currentTraj = initTraj;
  m_trajVars = VarArray(initTraj.rows(), m_nJoints);
  for (int iRow = 1; iRow < m_currentTraj.rows() - 1; ++iRow) {
    for (int iCol = 0; iCol < m_currentTraj.cols(); ++iCol) {
      char namebuf[10];
      sprintf(namebuf, "j_%i_%i",iRow,iCol);
      m_trajVars.at(iRow, iCol) = m_model->addVar(0, 0, 0, GRB_CONTINUOUS,namebuf);
    }
  }
  m_model->update();

  for (int i=0; i < m_comps.size(); ++i) m_comps[i]->init(m_currentTraj, m_trajVars, m_model.get());

}


