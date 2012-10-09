#include "sqp_algorithm.h"
#include "utils/interpolation.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "simulation/simplescene.h"
#include "utils/conversions.h"
#include <typeinfo>
#include "config_sqp.h"
using namespace Eigen;
using namespace std;

static const float PATH_LENGTH_RATIO = 3;
static const float MAX_DIST_OVER_TIMESTEP_DIST = 5;

template<class T>
string getClassName(T& x) {
  string struct_classname = typeid(x).name();
  return std::string(&struct_classname[2]); // not sure why 2 digit number in front of name
}

// http://www.gurobi.com/documentation/5.0/reference-manual/node740
static const char* grb_statuses[] = { "XXX", //0
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
    "SUBOPTIMAL" //13
    };

static GRBEnv* grbEnv = new GRBEnv();
GRBEnv* getGRBEnv() {
  return grbEnv;
}
static const GRBVar nullvar;
bool isValidVar(GRBVar& var) {
  return !var.sameAs(nullvar);
}

Eigen::VectorXd defaultMaxStepMvmt(const Eigen::MatrixXd& traj) {
  VectorXd maxStepMvmt = PATH_LENGTH_RATIO
      * (traj.row(traj.rows() - 1) - traj.row(0)).array().abs() / traj.rows();
  maxStepMvmt = maxStepMvmt.cwiseMax(VectorXd::Constant(traj.cols(), 2 * SIMD_PI / traj.rows()));
  return maxStepMvmt;
}

Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints,
    int nSteps) {
  assert(startJoints.size() == endJoints.size());
  Eigen::MatrixXd startEndJoints(2, startJoints.size());
  startEndJoints.row(0) = startJoints;
  startEndJoints.row(1) = endJoints;
  return interp2d(VectorXd::LinSpaced(nSteps, 0, 1), VectorXd::LinSpaced(2, 0, 1), startEndJoints);

}


void updateTraj(const VarArray& trajVars, const VectorXb& optmask, Eigen::MatrixXd& traj) {
  for (int i = 0; i < traj.rows(); ++i)
    if (optmask(i)) for (int j = 0; j < traj.cols(); ++j) {
      traj(i, j) = trajVars.at(i, j).get(GRB_DoubleAttr_X);
    }
}

void setVarsToTraj(const Eigen::MatrixXd traj, const VectorXb& optmask, VarArray& trajVars) {
  for (int i = 0; i < traj.rows(); ++i)
    if (optmask(i)) for (int j = 0; j < traj.cols(); ++j)
      trajVars.at(i, j).set(GRB_DoubleAttr_X, traj(i, j));
}

ArmCCEPtr makeArmCCE(RaveRobotObject::Manipulator::Ptr rrom, RaveRobotObject::Ptr rro,
    btDynamicsWorld* world) {
  vector<KinBody::JointPtr> armJoints;
  vector<KinBody::LinkPtr> armLinks;
  vector<int> chainDepthOfBodies;
  getArmKinInfo(rro->robot, rrom->manip, armLinks, armJoints, chainDepthOfBodies);
  vector<btRigidBody*> armBodies;
  BOOST_FOREACH(KinBody::LinkPtr& link, armLinks) {
    armBodies.push_back(rro->associatedObj(link)->rigidBody.get());
  }
  return ArmCCEPtr(
      new ArmCCE(rro->robot, world, armLinks, armBodies, armJoints, chainDepthOfBodies));
}

BulletRaveSyncher syncherFromArm(RaveRobotObject::Manipulator::Ptr rrom) {
  vector<KinBody::LinkPtr> armLinks = getArmLinks(rrom->manip);
  vector<btRigidBody*> armBodies;
  BOOST_FOREACH(KinBody::LinkPtr& link, armLinks) {
    armBodies.push_back(rrom->robot->associatedObj(link)->rigidBody.get());
  }
  return BulletRaveSyncher(armLinks, armBodies);
}

PlanningProblem::PlanningProblem() :
  m_model(new GRBModel(*grbEnv)) {
}

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

void PlanningProblem::addTrustRegionAdjuster(TrustRegionAdjusterPtr tra) {
  m_tra=tra;
  addComponent(tra);
}

void CollisionCost::subdivide(const std::vector<double>& insertTimes, const VectorXd& oldTimes, const VectorXd& newTimes) {
  if (m_coeffVec.size()>0) m_coeffVec = interp2d(newTimes, oldTimes, m_coeffVec);
}

void CollisionCost::removeVariablesAndConstraints() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts)
    m_problem->m_model->remove(cnt);
  BOOST_FOREACH(GRBVar& var, m_vars)
    m_problem->m_model->remove(var);
  m_vars.clear();
  m_cnts.clear();
}

void printCollisionReport(const TrajJointCollInfo& trajCollInfo, double safeDist) {
  int nNear, nUnsafe, nColl;
  countCollisions(trajCollInfo, safeDist, nNear, nUnsafe, nColl);
  int nTotalProximity = nNear + nUnsafe + nColl;
  LOG_INFO_FMT("near: %i, unsafe: %i, collision: %i, total: %i", nNear, nUnsafe, nColl, nTotalProximity);
}

void plotCollisions(const TrajCartCollInfo& trajCartInfo, double safeDist) {
  static PlotPoints::Ptr collisions(new PlotPoints(10));
  static PlotLines::Ptr escapes(new PlotLines(5));
  bool setupDone(false);
  if (!setupDone) {
    setupDone = true;
    util::getGlobalEnv()->add(collisions);
    util::getGlobalEnv()->add(escapes);
  }

  const osg::Vec4 GREEN(0, 1, 0, 1), YELLOW(1, 1, 0, 1), RED(1, 0, 0, 1);
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  osg::ref_ptr<osg::Vec3Array> escPts = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array> collPts = new osg::Vec3Array;
  for (int iStep = 0; iStep < trajCartInfo.size(); ++iStep) {
    if (iStep % SQPConfig::plotDecimation != 0) continue;
    for (int iColl = 0; iColl < trajCartInfo[iStep].size(); ++iColl) {
      const LinkCollision& lc = trajCartInfo[iStep][iColl];
      collPts->push_back(toOSGVector(lc.point * METERS));
      escPts->push_back(toOSGVector(lc.point * METERS));
      escPts->push_back(toOSGVector(lc.point * METERS - lc.normal * (lc.dist
          - BulletConfig::linkPadding) * METERS));
      if (lc.dist < 0) colors->push_back(RED);
      else if (lc.dist < safeDist) colors->push_back(YELLOW);
      else colors->push_back(GREEN);
    }
  }
  assert(collPts->size() == colors->size());
  assert(escPts->size() == 2*colors->size());
  collisions->setPoints(collPts, colors);
  escapes->setPoints(escPts, colors);
}

inline double pospart(double x) {
  return (x > 0) ? x : 0;
}

void CollisionCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  removeVariablesAndConstraints();
  // Actually run through trajectory and find collisions


  m_cartCollInfo = collectTrajCollisions(traj, m_robot, m_brs, m_world, m_dofInds);
  TrajJointCollInfo trajJointInfo = trajCartToJointCollInfo(m_cartCollInfo, traj, m_robot,
      m_dofInds);

#if 0
  TrajJointCollInfo trajJointInfo = m_cce->collectCollisionInfo(traj);
  {
    ArmCCE::Ptr cce = boost::dynamic_pointer_cast<ArmCCE>(m_cce);
    vector<int> dofInds;
    BOOST_FOREACH(KinBody::JointPtr joint, cce->m_joints) dofInds.push_back(joint->GetDOFIndex());
    TrajCartCollInfo trajCartInfo = collectTrajCollisions(traj, cce->m_robot, cce->m_syncher, cce->m_world, dofInds);
    TrajJointCollInfo trajJointInfo2 = trajCartToJointCollInfo(trajCartInfo, traj, cce->m_robot, dofInds);
    assert(trajJointInfo.size() == trajJointInfo2.size());
    for (int i=0; i < trajJointInfo.size(); ++i) {
      assert(trajJointInfo[i].dists.size() == trajJointInfo2[i].dists.size());
      assert(trajJointInfo[i].jacs.size() == trajJointInfo2[i].jacs.size());
      //      if (trajJointInfo[i].dists.size() > 0) {
      //        cout << trajJointInfo[i].dists << endl;
      //        cout << trajJointInfo2[i].dists << endl;
      //      }

      for (int j=0; j < trajJointInfo[i].dists.size(); ++j) {
        assert(fabs(trajJointInfo[i].dists[j] - trajJointInfo2[i].dists[j]) < 1e-6);
        assert((trajJointInfo[i].jacs[j] - trajJointInfo2[i].jacs[j]).norm() < 1e-6);
      }
    }
  }
#endif

  plotCollisions(m_cartCollInfo, SQPConfig::distDiscSafe);
  printCollisionReport(trajJointInfo, SQPConfig::distDiscSafe);

  for (int iStep = 0; iStep < traj.rows(); ++iStep)
    if (m_problem->m_optMask(iStep)) for (int iColl = 0; iColl < trajJointInfo[iStep].jacs.size(); ++iColl) {
      m_vars.push_back(m_problem->m_model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "hinge"));
    }
  m_problem->m_model->update();

  VectorXd coeffs_t;
  if (m_coeffVec.size() > 0) {
    ENSURE(m_coeffVec.size() == traj.rows());
    coeffs_t = m_coeffVec;
  }
  else coeffs_t = VectorXd::Constant(traj.rows(), m_coeff);

  m_exactObjective = 0;
  vector<double> coeffs;

  int varCount = 0;
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    if (m_problem->m_optMask(iStep)) {
      std::vector<Eigen::VectorXd>& jacs = trajJointInfo[iStep].jacs;
      std::vector<double>& dists = trajJointInfo[iStep].dists;
      VarVector jointVars = m_problem->m_trajVars.row(iStep);
      for (int iColl = 0; iColl < trajJointInfo[iStep].jacs.size(); ++iColl) {
        GRBVar& hinge = m_vars[varCount];
        //        hinge.set(GRB_DoubleAttr_X, dists[iColl]);
        ++varCount;
        GRBLinExpr jacDotTheta;
        jacDotTheta.addTerms(jacs[iColl].data(), jointVars.data(), traj.cols());
        GRBConstr hingeCnt = m_problem->m_model->addConstr(hinge >= -dists[iColl] + jacDotTheta
            + m_distPen - jacs[iColl].dot(traj.row(iStep)),"hinge");
        m_cnts.push_back(hingeCnt);
        coeffs.push_back(coeffs_t[iStep]);
        m_exactObjective += coeffs_t[iStep] * pospart(-dists[iColl] + m_distPen);
        //        cout << "asdf: " << coeffs_t[iStep] * pospart(-dists[iColl]+m_safeDist);
      }
    }
  }
  assert(varCount == m_vars.size());

  // Create this part of the cost
  m_obj = GRBLinExpr(0);
  m_obj.addTerms(coeffs.data(), m_vars.data(), m_vars.size());

  objective += m_obj;
}

void CollisionCost::onRemove() {
  removeVariablesAndConstraints();
}

#if 0
void VelScaledCollisionCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  removeVariablesAndConstraints();
  // Actually run through trajectory and find collisions
  TrajJointCollInfo trajCollInfo = m_cce->collectCollisionInfo(traj);
  printCollisionReport(trajCollInfo, m_distPen);

  for (int iStep = 0; iStep < traj.rows(); ++iStep)
  if (m_problem->m_optMask(iStep))
  for (int iColl = 0; iColl < trajCollInfo[iStep].jacs.size(); ++iColl) {
    m_vars.push_back(m_problem->m_model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS,"hinge"));
  }
  m_problem->m_model->update();

  int varCount = 0;
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    if (m_problem->m_optMask(iStep)) {
      std::vector<Eigen::VectorXd>& jacs = trajCollInfo[iStep].jacs;
      std::vector<double>& dists = trajCollInfo[iStep].dists;
      VarVector jointVars = m_problem->m_trajVars.row(iStep);
      for (int iColl = 0; iColl < trajCollInfo[iStep].jacs.size(); ++iColl) {
        GRBVar& hinge = m_vars[varCount];
        ++varCount;
        GRBLinExpr jacDotTheta;
        jacDotTheta.addTerms(jacs[iColl].data(), jointVars.data(), traj.cols());
        GRBConstr hingeCnt = m_problem->m_model->addConstr(hinge >= -dists[iColl] + jacDotTheta + m_distPen - jacs[iColl].dot(traj.row(iStep)));
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

void CollisionConstraint::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  // Remove added constraints
  for (int iCnt = 0; iCnt < m_cnts.size(); ++iCnt)
  m_problem->m_model->remove(m_cnts[iCnt]);
  m_cnts.clear();

  // Actually run through trajectory and find collisions
  TrajJointCollInfo trajCollInfo = m_cce->collectCollisionInfo(traj);

  // Make a bunch of variables for hinge costs
  int nNear, nUnsafe, nColl;
  countCollisions(trajCollInfo, m_distPen,nNear, nUnsafe, nColl);
  int nTotalProximity = nNear + nUnsafe + nColl;
  LOG_INFO_FMT("near: %i, unsafe: %i, collision: %i, total: %i", nNear, nUnsafe, nColl, nTotalProximity);

  // Create variables that will form the cost
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    std::vector<Eigen::VectorXd>& jacs = trajCollInfo[iStep].jacs;
    std::vector<double>& dists = trajCollInfo[iStep].dists;
    VarVector jointVars = m_problem->m_trajVars.row(iStep);
    for (int iColl = 0; iColl < trajCollInfo[iStep].jacs.size(); ++iColl) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(jacs[iColl].data(), jointVars.data(), traj.rows());
      GRBConstr cnt = m_problem->m_model->addConstr(0 >= -dists[iColl] + jacDotTheta + m_distPen - jacs[iColl].dot(traj.row(iStep)));
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
#endif

void LengthConstraintAndCost::onAdd() {
  m_obj = GRBQuadExpr(0);
  MatrixXd& traj = m_problem->m_currentTraj;
  for (int iStep = 1; iStep < traj.rows(); ++iStep)
    for (int iJoint = 0; iJoint < traj.cols(); ++iJoint) {
      GRBLinExpr vel;
      if (m_problem->m_optMask(iStep)) {
        vel += m_problem->m_trajVars.at(iStep, iJoint);
        assert(isValidVar(m_problem->m_trajVars.at(iStep,iJoint)));
      }
      else vel += traj(iStep, iJoint);
      if (m_problem->m_optMask(iStep - 1)) {
        vel -= m_problem->m_trajVars.at(iStep - 1, iJoint);
        assert(isValidVar(m_problem->m_trajVars.at(iStep-1,iJoint)));
      }
      else vel -= traj(iStep - 1, iJoint);
      double dt = m_problem->m_times(iStep) - m_problem->m_times(iStep - 1);
      vel = vel / dt;
      m_cnts.push_back(m_problem->m_model->addConstr(vel <= m_maxStepMvmt(iJoint),"vel"));
      m_cnts.push_back(m_problem->m_model->addConstr(vel >= -m_maxStepMvmt(iJoint),"vel"));
      m_obj += traj.rows() * dt * (vel * vel);
    }
}

void LengthConstraintAndCost::onRemove() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts)
    m_problem->m_model->remove(cnt);
  m_cnts.clear();
}

void LengthConstraintAndCost::subdivide(const std::vector<double>& insertTimes,const VectorXd& oldTimes, const VectorXd& newTimes) {
  reset();
}

double LengthConstraintAndCost::calcExactObjective() {
  MatrixXd& traj = m_problem->m_currentTraj;
  VectorXd& times = m_problem->m_times;
  MatrixXd diffs = traj.middleRows(1, traj.rows() - 1) - traj.middleRows(0, traj.rows() - 1);
  VectorXd dts = times.middleRows(1, traj.rows() - 1) - times.middleRows(0, traj.rows() - 1);
  double obj = 0;
  for (int i = 0; i < dts.size(); ++i)
    obj += traj.rows() * diffs.row(i).squaredNorm() / dts(i);
  return m_coeff * obj;
}

void JointBounds::onAdd() {
  MatrixXd& traj = m_problem->m_currentTraj;
  VectorXd maxStepMvmt = PATH_LENGTH_RATIO
      * (traj.row(traj.rows() - 1) - traj.row(0)).array().abs() / traj.rows();
  maxStepMvmt = maxStepMvmt.cwiseMax(VectorXd::Constant(traj.cols(), 2 * SIMD_PI / traj.rows()));

}

void JointBounds::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    if (m_problem->m_optMask(iStep)) for (int iJoint = 0; iJoint < traj.cols(); ++iJoint) {
      m_problem->m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_LB, fmax(traj(iStep, iJoint)
          - m_maxDiffPerIter(iJoint), m_jointLowerLimit(iJoint)));
      m_problem->m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_UB, fmin(traj(iStep, iJoint)
          + m_maxDiffPerIter(iJoint), m_jointUpperLimit(iJoint)));
    }
  }

}

void JointBounds::adjustTrustRegion(double ratio) {
  m_maxDiffPerIter *= ratio;
}

void getGripperTransAndJac(RaveRobotObject::Manipulator::Ptr manip, const VectorXd& dofVals,
    btTransform& tf, Eigen::MatrixXd& jac, Eigen::MatrixXd& rotJac) {
  RobotBasePtr robot = manip->robot->robot;
  ScopedRobotSave srs(robot);
  robot->SetActiveDOFs(manip->manip->GetArmIndices());
  robot->SetActiveDOFValues(toDoubleVec(dofVals));

  boost::multi_array<dReal, 2> jac0;
  manip->manip->CalculateJacobian(jac0);
  jac = Eigen::Map<MatrixXd>(jac0.data(), 3, jac0.shape()[1]);



#if 0  /// openrave rotation jacobian calculation is screwy, so i do it numerically
  boost::multi_array<dReal, 2> rotjac0;
  manip->manip->CalculateRotationJacobian(rotjac0);
  MatrixXd rotJacRave = Eigen::Map<MatrixXd>(rotjac0.data(), 4, jac0.shape()[1]);
  rotJac = MatrixXd(4, jac0.shape()[1]);
  rotJac.row(0) = rotJacRave.row(1);
  rotJac.row(1) = rotJacRave.row(2);
  rotJac.row(2) = rotJacRave.row(3);
  rotJac.row(3) = rotJacRave.row(0);

  rotJac *= -1;
#endif
  // the following code seems to reveal a sign error in openrave or our code's interface with it
#if 1



  MatrixXd numericalJac(4,7);
  double eps = 1e-5;
  Vector4d curQuat = toVector4d(manip->getTransform().getRotation());
  for (int i=0; i < 7; ++i) {
    VectorXd dofValsNew = dofVals;
    dofValsNew[i] += eps;
    robot->SetActiveDOFValues(toDoubleVec(dofValsNew));
    numericalJac.col(i) = (toVector4d(manip->getTransform().getRotation()) - curQuat)/eps;
  }

  rotJac = numericalJac;
//  cout << "numerical jacobian: " << endl << numericalJac << endl;
//  cout << "analytic jacobian: " << endl << rotJac << endl;;
#endif

#if 0
  RaveVector<double> raveQuat = manip->manip->GetTransform().rot;
  cout << "rave/bullet quats" << endl;
  cout << raveQuat[0] << " " << raveQuat[1] << " " << raveQuat[2] << " " << raveQuat[3] << endl;
  cout << manip->getTransform().getRotation() << endl;
  exit(0);
#endif
#if 0
  cout << "multi_array:"<<endl;
  for (int i=0; i < jac0.shape()[0]; ++i) {
    for (int j=0; j < jac0.shape()[1]; ++j) {
      cout << jac0[i][j] << " ";
    }
    cout << endl;
  }
  cout << "Eigen: " << endl;
  cout << jac << endl;
  exit(0);
#endif
  tf = util::toBtTransform(manip->manip->GetTransform());
}

void CartesianPoseCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  VectorXd curVals = traj.row(m_timestep);
  // collect jacobian info --------
  btTransform tf;
  MatrixXd jac, rotjac;
  getGripperTransAndJac(m_manip, curVals, tf, jac, rotjac);
  // ---------

  m_obj = GRBQuadExpr(0);
  VarVector timestepVars = m_problem->m_trajVars.row(m_timestep);

  if (m_posCoeff > 0) {
    Vector3d posCur = toVector3d(tf.getOrigin());
    //    cout << "position error: " << (m_posTarg - posCur).transpose() << endl;
    for (int i = 0; i < 3; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(jac.row(i).data(), timestepVars.data(), traj.cols());
      GRBLinExpr erri = posCur(i) + jacDotTheta - jac.row(i).dot(curVals) - m_posTarg(i);
      m_obj += m_posCoeff * (erri * erri);
    }
  }

  if (m_rotCoeff > 0) {
    Vector4d rotCur = toVector4d(tf.getRotation());
    if ((rotCur.dot(m_rotTarg)) < 0) m_rotTarg *= -1;
    //    cout << "cur/targ: " << rotCur.transpose() << ", " << m_rotTarg.transpose() << endl;
    for (int i = 0; i < 4; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(rotjac.row(i).data(), timestepVars.data(), traj.cols());
      GRBLinExpr erri = rotCur(i) + jacDotTheta - rotjac.row(i).dot(curVals) - m_rotTarg(i);
      m_obj += m_rotCoeff * (erri * erri);
    }
  }

  objective += m_obj;
}

void CartesianPoseCost::subdivide(const std::vector<double>& insertTimes,const VectorXd& oldTimes, const VectorXd& newTimes) {
  BOOST_FOREACH(double t, insertTimes) {
    if (t < m_timestep) {
      ++m_timestep;
    }
  }
  LOG_INFO_FMT("CartPoseCost subdiv: new traj len: %i. cnt timestep: %i", m_problem->m_times.size(), m_timestep);
}

double CartesianPoseCost::calcExactObjective() {
  ScopedRobotSave srs(m_manip->robot->robot);
  m_manip->setDOFValues(toDoubleVec(m_problem->m_currentTraj.row(m_timestep)));
  btTransform tf = util::toBtTransform(m_manip->manip->GetTransform());
  return m_posCoeff * (toVector3d(tf.getOrigin()) - m_posTarg).squaredNorm() + m_rotCoeff
      * (toVector4d(tf.getRotation()) - m_rotTarg).squaredNorm();
}

void CartesianPoseConstraint::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  removeVariablesAndConstraints();

  VectorXd curVals = traj.row(m_timestep);
  // collect jacobian info --------
  btTransform tf;
  MatrixXd jac, rotjac;
  getGripperTransAndJac(m_manip, curVals, tf, jac, rotjac);
  // ---------

  VarVector timestepVars = m_problem->m_trajVars.row(m_timestep);

  if (m_posTol > 0) {
    Vector3d posCur = toVector3d(tf.getOrigin());
    //    cout << "position error: " << (m_posTarg - posCur).transpose() << endl;
    for (int i = 0; i < 3; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(jac.row(i).data(), timestepVars.data(), traj.cols());
      GRBLinExpr erri = posCur(i) + jacDotTheta - jac.row(i).dot(curVals) - m_posTarg(i);
      m_cnts.push_back(m_problem->m_model->addConstr(erri <= m_posTol,"pos"));
    }
  }

  if (m_rotTol > 0) {
    Vector4d rotCur = toVector4d(tf.getRotation());
    if ((rotCur.dot(m_rotTarg)) < 0) m_rotTarg *= -1;
    //    cout << "cur/targ: " << rotCur.transpose() << ", " << m_rotTarg.transpose() << endl;
    for (int i = 0; i < 4; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(rotjac.row(i).data(), timestepVars.data(), traj.cols());
      GRBLinExpr erri = rotCur(i) + jacDotTheta - rotjac.row(i).dot(curVals) - m_rotTarg(i);
      m_cnts.push_back(m_problem->m_model->addConstr(erri <= m_rotTol,"rot"));
    }
  }

}

void CartesianPoseConstraint::removeVariablesAndConstraints() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts)
    m_problem->m_model->remove(cnt);
  m_cnts.clear();
}

void CartesianPoseConstraint::onRemove() {
  removeVariablesAndConstraints();
}

void CartesianPoseConstraint::subdivide(const std::vector<double>& insertTimes,const VectorXd& oldTimes, const VectorXd& newTimes) {
  BOOST_FOREACH(double t, insertTimes) {
    if (t < m_timestep) {
      ++m_timestep;
    }
  }
  LOG_INFO_FMT("CartPoseConstraint subdiv: new traj len: %i. cnt timestep: %i", m_problem->m_times.size(), m_timestep);
}

void CartesianVelConstraint::removeVariablesAndConstraints() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts)
    m_problem->m_model->remove(cnt);
  m_cnts.clear();
}

void CartesianVelConstraint::onRemove() {
  removeVariablesAndConstraints();
}

void updateConstraint(GRBModel* model, GRBConstr& cnt, GRBLinExpr& expr) {
  vector<GRBVar> vars;
  vector<GRBConstr> cnts;
  vector<double> coeffs;
  for (int i=0; i < expr.size(); ++i) {
    vars.push_back(expr.getVar(i));
    cnts.push_back(cnt);
    coeffs.push_back(expr.getCoeff(i));
  }
  model->chgCoeffs(cnts.data(), vars.data(), coeffs.data(), vars.size());
  cnt.set(GRB_DoubleAttr_RHS, -expr.getConstant());
}

void CartesianVelConstraint::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
#define RESET_CONSTRAINTS // I thought it might speed things up to change the constraints in place
  // but actually it doesn't make a difference
#ifdef RESET_CONSTRAINTS
  removeVariablesAndConstraints();
#endif

  vector<MatrixXd> jacs(m_stop - m_start), rotjacs(m_stop - m_start);
  vector<btTransform> tfs(m_stop - m_start);

  for (int i = 0; i < m_stop - m_start; ++i) {
    getGripperTransAndJac(m_manip, traj.row(i + m_start), tfs[i], jacs[i], rotjacs[i]);
  }
  // ---------
#ifndef RESET_CONSTRAINTS
  vector<GRBLinExpr> exprs;
#endif

  for (int iStep = 0; iStep < m_stop - m_start - 1; ++iStep) {
    VarVector fromVars = m_problem->m_trajVars.row(iStep + m_start);
    VarVector toVars = m_problem->m_trajVars.row(iStep + 1 + m_start);

    VectorXd diff = toVector3d(tfs[iStep + 1].getOrigin() - tfs[iStep].getOrigin());
    GRBQuadExpr sqdist(0);
    for (int iDim = 0; iDim < 3; ++iDim) {
      GRBLinExpr jacDotThetaFrom, jacDotThetaTo;
      jacDotThetaFrom.addTerms(jacs[iStep].row(iDim).data(), fromVars.data(), traj.cols());
      jacDotThetaTo.addTerms(jacs[iStep + 1].row(iDim).data(), toVars.data(), traj.cols());
      GRBLinExpr diffi = diff[iDim] +
          (jacDotThetaTo - jacs[iStep + 1].row(iDim).dot(traj.row(iStep+ 1 + m_start)))
          - (jacDotThetaFrom- jacs[iStep].row(iDim).dot(traj.row(iStep + m_start)));
//      sqdist += diffi * diffi;

#ifdef RESET_CONSTRAINTS
      m_cnts.push_back(m_problem->m_model->addConstr(diffi <= m_maxDist,"eevel"));
      m_cnts.push_back(m_problem->m_model->addConstr(diffi >= -m_maxDist,"eevel"));
#else
      exprs.push_back(diffi - m_maxDist);
      exprs.push_back(-m_maxDist - diffi);
#endif
    }
//    m_cnts.push_back(m_problem->m_model->addQConstr(sqdist <= m_maxDist * m_maxDist));
  }

#ifndef RESET_CONSTRAINTS
  if (m_cnts.size() == 0) {
    BOOST_FOREACH(GRBLinExpr& expr, exprs) {
      m_cnts.push_back(m_problem->m_model->addConstr(expr <= 0));
    }
  }
  else {
    assert (m_cnts.size() == exprs.size());
    for (int i=0; i < m_cnts.size(); ++i) {
      updateConstraint(m_problem->m_model.get(), m_cnts[i], exprs[i]);
    }
  }
#endif

}

void CartesianVelConstraint::subdivide(const std::vector<double>& insertTimes,const VectorXd& oldTimes, const VectorXd& newTimes) {
  BOOST_FOREACH(double t, insertTimes) {
    if (t < m_start) {
      ++m_start;
      ++m_stop;
    }
    else if (t < m_stop) {
      throw std::runtime_error("don't know how to subdivide CartesianVelConstraint");
    }
  }
}

void PlanningProblem::updateModel() {
  // todo: they should directly modify the objective, since they directly modify the model thru constraints
  GRBQuadExpr objective(0);
  for (int i = 0; i < m_comps.size(); ++i)
    m_comps[i]->updateModel(m_currentTraj, objective);
  m_model->update();
  m_model->setObjective(objective);
}

double PlanningProblem::calcApproxObjective() {
  double out = 0;
  BOOST_FOREACH(ProblemComponent::Ptr comp, m_comps) {
    if (comp->m_attrs & ProblemComponent::HAS_COST) {
      out += comp->calcApproxObjective();
      LOG_INFO(getClassName(*comp) << " approx after: " << comp->calcApproxObjective());
    }
  }
  //  ASSERT_ALMOST_EQUAL(out,m_model->get(GRB_DoubleAttr_ObjVal), 1e-5);
  return out;
}

double PlanningProblem::calcExactObjective() {
  double out = 0;
  BOOST_FOREACH(ProblemComponent::Ptr comp, m_comps) {
    if (comp->m_attrs & ProblemComponent::HAS_COST) {
      out += comp->calcExactObjective();
      LOG_INFO(getClassName(*comp) << " exact before: " << comp->calcExactObjective());
    }
  }
  return out;
}

void PlanningProblem::testObjectives() {

  printf("testing objectives\n");
  m_model->setObjective(GRBQuadExpr(0));
  updateModel();

  vector<GRBConstr> cnts;
  for (int i = 0; i < m_currentTraj.rows(); ++i) {
    for (int j = 0; j < m_currentTraj.cols(); ++j) {
      if (m_optMask(i)) cnts.push_back(m_model->addConstr(m_trajVars.at(i, j)
          == m_currentTraj(i, j)));
    }
  }

  m_model->optimize();

  int status = m_model->get(GRB_IntAttr_Status);
  if (status != GRB_OPTIMAL) {
    LOG_ERROR("bad grb status: " << grb_statuses[status]);
    throw;
  }

  BOOST_FOREACH(ProblemComponentPtr comp, m_comps) {
    if (comp->m_attrs & ProblemComponent::HAS_COST) {
      double approxObj = comp->calcApproxObjective();
      double exactObj = comp->calcExactObjective();
      LOG_INFO_FMT("%s: exact: %.3f, approx: %.3f", getClassName(*comp).c_str(), exactObj, approxObj);
      ASSERT_ALMOST_EQUAL(approxObj, exactObj, 1e-5);
    }
  }

  BOOST_FOREACH(GRBConstr& cnt, cnts)
    m_model->remove(cnt);

}

void PlanningProblem::doIteration() {
}

void PlanningProblem::optimize(int maxIter) {
  assert(m_tra);
  BOOST_FOREACH(TrajPlotterPtr plotter, m_plotters)
    plotter->plotTraj(m_currentTraj);

  MatrixXd prevTraj;
  vector<double> trueObjBefore, approxObjAfter;
  double trShrinkage=1;

  TIC();
  updateModel();
  LOG_INFO_FMT("total convexification time: %.2f", TOC());
  double trueObj = calcExactObjective();
  trueObjBefore.push_back(trueObj);
  LOG_INFO_FMT("Total exact objective before: %.2f", trueObj);


  for (int iter = 0; iter < maxIter; ) {

    assert(trueObjBefore.size() == iter+1);
    assert(approxObjAfter.size() == iter);

    LOG_INFO_FMT("iteration: %i",iter);

    TIC1();
    m_model->optimize();
    LOG_INFO_FMT("optimization time: %.2f", TOC());
    double approxObj = calcApproxObjective();
    prevTraj = m_currentTraj;
    int status = m_model->get(GRB_IntAttr_Status);
    assert(status == GRB_OPTIMAL);
    updateTraj(m_trajVars, m_optMask, m_currentTraj);

    BOOST_FOREACH(Callback& cb, m_callbacks)
      cb(this);
    BOOST_FOREACH(TrajPlotterPtr plotter, m_plotters)
      plotter->plotTraj(m_currentTraj);


    TIC();
    updateModel();
    m_model->update();
    LOG_INFO_FMT("total convexification time: %.2f", TOC());
    double trueObj = calcExactObjective();

    if (trueObj > trueObjBefore.back()) {
      LOG_INFO("objective got worse! rolling back and shrinking trust region");
      m_currentTraj = prevTraj;
      m_tra->adjustTrustRegion(SQPConfig::trShrink);
      trShrinkage *= SQPConfig::trShrink;
      continue;
    }

    LOG_INFO_FMT("Total exact objective: %.2f", trueObj);
    LOG_INFO("accepting traj modification");
    trueObjBefore.push_back(trueObj);
    approxObjAfter.push_back(approxObj);


    double trueImprove = trueObjBefore[trueObjBefore.size()-2] - trueObjBefore.back();
    double approxImprove = trueObjBefore[trueObjBefore.size()-2] - approxObj;
    double improveRatio = trueImprove/approxImprove;
    LOG_INFO_FMT("true improvement: %.2e.  approx improvement: %.2e.  ratio: %.2f", trueImprove, approxImprove, improveRatio);
    if (improveRatio < SQPConfig::trThresh) {
      LOG_INFO_FMT("shrinking trust region by %.2f", SQPConfig::trShrink);
      m_tra->adjustTrustRegion(SQPConfig::trShrink);
      trShrinkage *= SQPConfig::trShrink;
    }
    else {
      LOG_INFO_FMT("expanding trust region by %.2f", SQPConfig::trExpand);
      m_tra->adjustTrustRegion(SQPConfig::trExpand);
      trShrinkage *= SQPConfig::trExpand;
    }

    if (trShrinkage < 1e-3) {
      LOG_INFO("trust region got shrunk too much. stopping");
      break;
    }
    if (trueImprove < SQPConfig::doneIterThresh) {
      LOG_INFO("cost didn't improve much. stopping iteration");
      break;
    }

    ++iter;

  }

}

void PlanningProblem::initialize(const Eigen::MatrixXd& initTraj, bool endFixed) {
  Eigen::VectorXd times = VectorXd::LinSpaced(initTraj.rows(), 0, initTraj.rows() - 1);
  initialize(initTraj, endFixed, times);
}

void merged(const vector<double>& v0, const vector<double>& v1, vector<double>& out,
    vector<int>& inds0, vector<int>& inds1) {
  int nNew = v0.size() + v1.size();
  out.resize(nNew);
  inds0.resize(v0.size());
  inds1.resize(v1.size());

  int i0(0), i1(0);
  bool done0(false), done1(false);
  for (int i = 0; i < nNew; ++i) {
    if (i0 == v0.size()) done0 = true;
    if (i1 == v1.size()) done1 = true;
    assert(!done0 || !done1);
    if (!done0 && (done1 || v0[i0] < v1[i1])) {
      out[i] = v0[i0];
      inds0[i0] = i;
      ++i0;
    }
    else {
      out[i] = v1[i1];
      inds1[i1] = i;
      ++i1;
    }
  }
}

void PlanningProblem::subdivide(const std::vector<double>& insertTimes) {
  int nNew = insertTimes.size() + m_times.size();
  int nCol = m_currentTraj.cols();
  vector<int> oldInds, newInds;
  vector<double> newTimesVec;
  merged(toDoubleVec(m_times), insertTimes, newTimesVec, oldInds, newInds);
  for (int i = 0; i < newTimesVec.size() - 1; ++i)
    assert(newTimesVec[i] != newTimesVec[i+1]);
  VectorXd newTimes = toVectorXd(newTimesVec);
  MatrixXd newTraj = interp2d(newTimes, m_times, m_currentTraj);
  VarArray newTrajVars(nNew, nCol);
  VectorXd oldTimes = m_times;

  for (int i = 0; i < oldInds.size(); ++i) {
    for (int j = 0; j < nCol; ++j) {
      newTrajVars.at(oldInds[i], j) = m_trajVars.at(i, j);
    }
  }
  for (int i = 0; i < newInds.size(); ++i) {
    for (int j = 0; j < nCol; ++j) {
      char namebuf[10];
      sprintf(namebuf, "j_%.2f_%i", insertTimes[i], j);
      newTrajVars.at(newInds[i], j) = m_model->addVar(0, 0, 0, GRB_CONTINUOUS, namebuf);
    }
  }
  m_times = newTimes;
  m_trajVars = newTrajVars;
  m_currentTraj = newTraj;
  VectorXb newOptMask = VectorXb::Ones(nNew);
  newOptMask(0) = m_optMask(0);
  newOptMask(nNew - 1) = m_optMask(m_optMask.size() - 1);
  m_optMask = newOptMask;
  //  cout << m_optMask.transpose() << endl;
  m_model->update();
  BOOST_FOREACH(ProblemComponentPtr comp, m_comps) {
    comp->subdivide(insertTimes, oldTimes, newTimes);
  }
}

void PlanningProblem::initialize(const Eigen::MatrixXd& initTraj, bool endFixed,
    const Eigen::VectorXd& times) {
  assert (times.size() == initTraj.rows());
  m_times = times;
  m_currentTraj = initTraj;
  m_trajVars = VarArray(initTraj.rows(), initTraj.cols());
  m_optMask = VectorXb::Ones(initTraj.rows());
  m_optMask(0) = false;
  if (endFixed) m_optMask(initTraj.rows() - 1) = false;
  for (int iRow = 0; iRow < m_currentTraj.rows(); ++iRow) {
    if (m_optMask(iRow)) {
      for (int iCol = 0; iCol < m_currentTraj.cols(); ++iCol) {
        char namebuf[10];
        sprintf(namebuf, "j_%i_%i", iRow, iCol);
        m_trajVars.at(iRow, iCol) = m_model->addVar(0, 0, 0, GRB_CONTINUOUS, namebuf);
      }
    }
  }
  m_model->update();
  //  setVarsToTraj(m_currentTraj,m_optMask, m_trajVars);
  m_initialized = true;
}

GripperPlotter::GripperPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, int decimation) :
  m_scene(scene), m_osgRoot(scene->env->osg->root.get()), m_rrom(rrom), m_decimation(decimation),
      m_curve(new PlotCurve(3)) {
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

ArmPlotter::ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, int decimation) :
  m_syncher(syncherFromArm(rrom)) {
  vector<BulletObject::Ptr> armObjs;
  BOOST_FOREACH(KinBody::LinkPtr link, m_syncher.m_links)
    armObjs.push_back(rrom->robot->associatedObj(link));
  init(rrom, armObjs, scene, decimation);
}

void ArmPlotter::init(RaveRobotObject::Manipulator::Ptr rrom,
    const std::vector<BulletObject::Ptr>& origs, Scene* scene, int decimation) {
  m_rrom = rrom;
  m_scene = scene;
  m_osgRoot = scene->env->osg->root.get();
  m_origs = origs;
  m_decimation = decimation;
  m_curve = new PlotCurve(3);
  m_curve->m_defaultColor = osg::Vec4f(0, 1, 0, 1);
  m_axes.reset(new PlotAxes());
  m_curve->m_defaultColor = osg::Vec4f(0, 1, 0, 1);
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
  vector<double> dofOrig = rrom->getDOFValues();
  vector<btVector3> out;
  for (int i = 0; i < traj.rows(); ++i) {
    rrom->setDOFValues(toDoubleVec(traj.row(i)));
    out.push_back(rrom->getTransform().getOrigin());
  }
  rrom->setDOFValues(dofOrig);
  return out;
}

vector<btTransform> getGripperPoses(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr rrom) {
  vector<double> dofOrig = rrom->getDOFValues();
  vector<btTransform> out;
  for (int i = 0; i < traj.rows(); ++i) {
    rrom->setDOFValues(toDoubleVec(traj.row(i)));
    out.push_back(rrom->getTransform());
  }
  rrom->setDOFValues(dofOrig);
  return out;
}

void ArmPlotter::plotTraj(const MatrixXd& traj) {
  setLength(traj.rows() / m_decimation);

  vector<double> curDOFVals = m_rrom->getDOFValues();
  for (int iPlot = 0; iPlot < m_fakes.m_nRow; ++iPlot) {
    m_rrom->setDOFValues(toDoubleVec(traj.row(iPlot * m_decimation)));
    m_syncher.updateBullet();
    for (int iObj = 0; iObj < m_origs.size(); ++iObj) {
      m_fakes.at(iPlot, iObj)->setTransform(m_origs[iObj]->rigidBody->getCenterOfMassTransform());
    }
  }

  m_rrom->setDOFValues(toDoubleVec(traj.row(traj.rows() - 1)));
  m_axes->setup(m_rrom->getTransform(), .1 * METERS);

  vector<btVector3> gripperPositions = getGripperPositions(traj, m_rrom);
  m_curve->setPoints(gripperPositions);

  m_rrom->setDOFValues(curDOFVals);
  m_syncher.updateBullet();

  TIC();
  m_scene->step(0);
  LOG_INFO_FMT("draw time: %.3f", TOC());
}

void interactiveTrajPlot(const MatrixXd& traj, RaveRobotObject::Manipulator::Ptr arm, Scene* scene) {
  BulletRaveSyncher syncher = syncherFromArm(arm);
  vector<double> curDOFVals = arm->getDOFValues();
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    arm->setDOFValues(toDoubleVec(traj.row(iStep)));
    syncher.updateBullet();
    printf("step %i. press p to continue\n", iStep);
    scene->step(0);
    scene->idle(true);
  }
  arm->setDOFValues(curDOFVals);
  syncher.updateBullet();
}

