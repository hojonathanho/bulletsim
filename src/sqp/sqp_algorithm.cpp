#include "sqp_algorithm.h"
#include "utils/interpolation.h"
#include "utils/logging.h"
#include "utils/clock.h"
#include "simulation/simplescene.h"
#include "utils/conversions.h"
#include <typeinfo>
#include "config_sqp.h"
#include <json/json.h>
#include "plotters.h"
#include "kinematics_utils.h"

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

void updateTraj(const VarArray& trajVars, Eigen::MatrixXd& traj) {
  for (int i = 0; i < traj.rows(); ++i)
    for (int j = 0; j < traj.cols(); ++j) {
      traj(i, j) = trajVars.at(i, j).get(GRB_DoubleAttr_X);
    }
}

void setVarsToTraj(const Eigen::MatrixXd traj, VarArray& trajVars) {
  for (int i = 0; i < traj.rows(); ++i)
    for (int j = 0; j < traj.cols(); ++j)
      trajVars.at(i, j).set(GRB_DoubleAttr_X, traj(i, j));
}

void CollisionCost::subdivide(const std::vector<double>& insertTimes, const VectorXd& oldTimes,
    const VectorXd& newTimes) {

  if (m_coeffVec.size() > 0) m_coeffVec = interp2d(newTimes, oldTimes, m_coeffVec);
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

void CollisionCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  removeVariablesAndConstraints();
  // Actually run through trajectory and find collisions


  m_cartCollInfo = collectTrajCollisions(traj, m_robot, *m_brs, m_world, m_dofInds, m_useAffine);
  TrajJointCollInfo trajJointInfo = trajCartToJointCollInfo(m_cartCollInfo, traj, m_robot,
      m_dofInds, m_useAffine);

#if 0
  TrajJogetCostInfo trajJointInfo = m_cce->collectCollisionInfo(traj);
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

	// static so it won't get destroyed at the end of the function
  if (SQPConfig::enablePlot) plotCollisions(m_cartCollInfo, SQPConfig::distDiscSafe);
  printCollisionReport(trajJointInfo, SQPConfig::distDiscSafe);

  for (int iStep = 0; iStep < traj.rows(); ++iStep)
    for (int iColl = 0; iColl < trajJointInfo[iStep].jacs.size(); ++iColl) {
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
            + m_distPen - jacs[iColl].dot(traj.row(iStep)), "hinge");
      m_cnts.push_back(hingeCnt);
      coeffs.push_back(coeffs_t[iStep]);
      m_exactObjective += coeffs_t[iStep] * pospart(-dists[iColl] + m_distPen);
      //        cout << "asdf: " << coeffs_t[iStep] * pospart(-dists[iColl]+m_safeDist);
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
void VelScalgetCostsionCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

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
      vel += m_problem->m_trajVars.at(iStep, iJoint);
      assert(isValidVar(m_problem->m_trajVars.at(iStep,iJoint)));

      vel -= m_problem->m_trajVars.at(iStep - 1, iJoint);
      assert(isValidVar(m_problem->m_trajVars.at(iStep-1,iJoint)));
      double dt = m_problem->m_times(iStep) - m_problem->m_times(iStep - 1);
      vel = vel / dt;
      m_cnts.push_back(m_problem->m_model->addConstr(vel <= m_maxStepMvmt(iJoint), "vel"));
      m_cnts.push_back(m_problem->m_model->addConstr(vel >= -m_maxStepMvmt(iJoint), "vel"));
      m_obj += traj.rows() * dt * (vel * vel);
    }
}

void LengthConstraintAndCost::onRemove() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts)
    m_problem->m_model->remove(cnt);
  m_cnts.clear();
}

void LengthConstraintAndCost::subdivide(const std::vector<double>& insertTimes,
    const VectorXd& oldTimes, const VectorXd& newTimes) {
  reset();
}

double LengthConstraintAndCost::getCost() {
  MatrixXd& traj = m_problem->m_currentTraj;
  VectorXd& times = m_problem->m_times;
  MatrixXd diffs = traj.middleRows(1, traj.rows() - 1) - traj.middleRows(0, traj.rows() - 1);
  VectorXd dts = times.middleRows(1, traj.rows() - 1) - times.middleRows(0, traj.rows() - 1);
  double obj = 0;
  for (int i = 0; i < dts.size(); ++i)
    obj += traj.rows() * diffs.row(i).squaredNorm() / dts(i);
  return m_coeff * obj;
}

JointBounds::JointBounds(bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter,
    OpenRAVE::RobotBase::ManipulatorPtr manip, int extraDofs) {
  construct(manip->GetRobot(), startFixed, endFixed, maxDiffPerIter, manip->GetArmIndices(), extraDofs);
}
JointBounds::JointBounds(bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter,
    OpenRAVE::RobotBase::ManipulatorPtr manip) {
  construct(manip->GetRobot(), startFixed, endFixed, maxDiffPerIter, manip->GetArmIndices(), 0);
}
JointBounds::JointBounds(OpenRAVE::RobotBasePtr robot, bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter,
    const std::vector<int>& dofInds) {
  construct(robot, startFixed, endFixed, maxDiffPerIter, dofInds, 0);
}
void JointBounds::construct(RobotBasePtr robot, bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter,
    const std::vector<int>& dofInds, int extraDofs) {
  m_startFixed = startFixed;
  m_endFixed = endFixed;
  m_maxDiffPerIter = maxDiffPerIter;

  m_jointLowerLimit.resize(dofInds.size() + extraDofs);
  m_jointLowerLimit.setConstant(-GRB_INFINITY);
  m_jointUpperLimit.resize(dofInds.size() + extraDofs);
  m_jointUpperLimit.setConstant(GRB_INFINITY);

  vector<double> ul, ll;
  for (int i = 0; i < dofInds.size(); ++i) {
    robot->GetJointFromDOFIndex(dofInds[i])->GetLimits(ll, ul);
    m_jointLowerLimit(i) = ll[0];
    m_jointUpperLimit(i) = ul[0];
  }
  m_useBall=false;
}

void JointBounds::onAdd() {
  MatrixXd& traj = m_problem->m_currentTraj;
  VectorXd maxStepMvmt = PATH_LENGTH_RATIO
      * (traj.row(traj.rows() - 1) - traj.row(0)).array().abs() / traj.rows();
  maxStepMvmt = maxStepMvmt.cwiseMax(VectorXd::Constant(traj.cols(), 2 * SIMD_PI / traj.rows()));

}

void JointBounds::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
  if (m_useBall) {
    m_problem->m_model->remove(m_cnt);
    GRBQuadExpr chgnorm(0);
    for (int i=0; i < traj.rows(); ++i)
      for (int j=0; j < traj.cols(); ++j) {
        GRBLinExpr diff = (traj(i, j) - m_problem->m_trajVars(i,j));
        chgnorm += (diff*diff) / (m_maxDiffPerIter(j)*m_maxDiffPerIter(j));

    }
    m_cnt = m_problem->m_model->addQConstr(chgnorm <= traj.rows() * traj.cols());
  }



  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    for (int iJoint = 0; iJoint < traj.cols(); ++iJoint) {
      m_problem->m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_LB, fmax(traj(iStep, iJoint)
          - m_maxDiffPerIter(iJoint), m_jointLowerLimit(iJoint)));
      m_problem->m_trajVars.at(iStep, iJoint).set(GRB_DoubleAttr_UB, fmin(traj(iStep, iJoint)
          + m_maxDiffPerIter(iJoint), m_jointUpperLimit(iJoint)));
    }
  }

}

void JointBounds::adjustTrustRegion(double ratio) {
  m_maxDiffPerIter *= ratio;
  m_shrinkage *= ratio;
  LOG_INFO("new trust region: " << m_maxDiffPerIter.transpose());
}


void getGripperTransAndJac(RaveRobotObject::Manipulator::Ptr manip, const VectorXd& dofVals, const vector<int>& dofInds,
    btTransform& tf, Eigen::MatrixXd& jac, Eigen::MatrixXd& rotJac) {
  RobotBasePtr robot = manip->robot->robot;
  ScopedRobotSave srs(robot);

  if (dofInds.size() == dofVals.size()) {
    robot->SetActiveDOFs(dofInds);
  }
  else {
    robot->SetActiveDOFs(dofInds, DOF_X | DOF_Y | DOF_RotationAxis, OpenRAVE::RaveVector<double>(0,0,1));
  }
  robot->SetActiveDOFValues(toDoubleVec(dofVals),false);

  int linkInd = manip->manip->GetEndEffector()->GetIndex();

  std::vector<double> jacvec(3*dofVals.size());
  robot->CalculateActiveJacobian(linkInd, manip->manip->GetEndEffectorTransform().trans, jacvec);

  jac = Eigen::Map<MatrixXd>(jacvec.data(), 3, dofVals.size());

  if (dofInds.size() != dofVals.size()) jac.col(jac.cols()-1) *= -1; // numerical jac didn't match with analytic

#if 0
  {
  MatrixXd numericalJac(3, dofVals.size());
  double eps = 1e-5;
  vector<double> d;
  Vector3d curPos = toVector3d(manip->getTransform().getOrigin())/METERS;
  robot->GetActiveDOFValues(d);
  VectorXd dofValsPert = dofVals;
  for (int i = 0; i < dofVals.size(); ++i) {
    dofValsPert[i]  = dofVals[i] + eps;
    robot->SetActiveDOFValues(toDoubleVec(dofValsPert));
    numericalJac.col(i) = (toVector3d(manip->getTransform().getOrigin())/METERS - curPos) / eps;
    dofValsPert[i] = dofVals[i];
  }

  cout << "numerical jac" << endl << numericalJac << endl;
  cout << "analytic jac" << endl << jac << endl;
  }
#endif

  MatrixXd numericalJac(4, dofVals.size());
  double eps = 1e-5;
  Vector4d curQuat = toVector4d(manip->getTransform().getRotation());
  VectorXd dofValsPert = dofVals;
  for (int i = 0; i < dofVals.size(); ++i) {
    dofValsPert[i]  = dofVals[i] + eps;
    robot->SetActiveDOFValues(toDoubleVec(dofValsPert),false);
    numericalJac.col(i) = (toVector4d(manip->getTransform().getRotation()) - curQuat) / eps;
    dofValsPert[i] = dofVals[i];
  }

  rotJac = numericalJac;
  //  cout << "numerical jacobian: " << endl << numericalJac << endl;
  //  cout << "analytic jacobian: " << endl << rotJac << endl;;


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



void getGripperTransAndJac(RaveRobotObject::Manipulator::Ptr manip, const VectorXd& dofVals,
    btTransform& tf, Eigen::MatrixXd& jac, Eigen::MatrixXd& rotJac) {
  const vector<int>& armInds = manip->manip->GetArmIndices();
  getGripperTransAndJac(manip, dofVals, armInds, tf, jac, rotJac);
}


void CartesianPoseCost::removeConstraints() {
  BOOST_FOREACH(GRBConstr& cnt, m_cnts)
    m_problem->m_model->remove(cnt);
  m_cnts.clear();
}


void CartesianPoseCost::updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {

  if (m_l1) {
    removeConstraints();
    if (m_vars.size() == 0) {
      for (int i=0; i < 7; ++i) m_vars.push_back(m_problem->m_model->addVar(0, GRB_INFINITY,0, GRB_CONTINUOUS, "pose_hinge"));
      m_problem->m_model->update();
    }
  }

  VectorXd curVals = traj.row(m_timestep);
  // collect jacobian info --------
  btTransform tf;
  MatrixXd jac, rotjac;
  getGripperTransAndJac(m_manip, curVals, m_dofInds, tf, jac, rotjac);
  // ---------

  m_obj = GRBQuadExpr(0);
  m_exactObjective = 0;
  VarVector timestepVars = m_problem->m_trajVars.row(m_timestep);

  if (m_posCoeff > 0) {
    Vector3d posCur = toVector3d(tf.getOrigin());
    //    cout << "position error: " << (m_posTarg - posCur).transpose() << endl;
    for (int i = 0; i < 3; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(jac.row(i).data(), timestepVars.data(), traj.cols());
      GRBLinExpr erri = posCur(i) - m_posTarg(i) + jacDotTheta - jac.row(i).dot(curVals);

      if (m_l1) {
        GRBVar abserr = m_vars[i];
        m_cnts.push_back(m_problem->m_model->addConstr(abserr >= erri));
        m_cnts.push_back(m_problem->m_model->addConstr(abserr >= -erri));
        m_obj += m_posCoeff * abserr;
        m_exactObjective += m_posCoeff * fabs(posCur(i) - m_posTarg(i));
      }
      else {
        m_obj += m_posCoeff * (erri * erri);
        m_exactObjective += m_posCoeff * sq(posCur(i) - m_posTarg(i));
      }
    }
  }

  if (m_rotCoeff > 0) {
    Vector4d rotCur = toVector4d(tf.getRotation());
    if ((rotCur.dot(m_rotTarg)) < 0) m_rotTarg *= -1;
    //    cout << "cur/targ: " << rotCur.transpose() << ", " << m_rotTarg.transpose() << endl;
    for (int i = 0; i < 4; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(rotjac.row(i).data(), timestepVars.data(), traj.cols());
      GRBLinExpr erri = rotCur(i) - m_rotTarg(i) + jacDotTheta - rotjac.row(i).dot(curVals);


      if (m_l1) {
        GRBVar abserr = m_vars[i+3];
        m_cnts.push_back(m_problem->m_model->addConstr(abserr >= erri));
        m_cnts.push_back(m_problem->m_model->addConstr(abserr >= -erri));
        m_obj += m_rotCoeff * abserr;
        m_exactObjective += m_posCoeff * fabs(rotCur(i) - m_rotTarg(i));
      }
      else {
        m_obj += m_posCoeff * (erri * erri);
        m_exactObjective += m_posCoeff * sq(rotCur(i) - m_rotTarg(i));
      }


    }
  }

  objective += m_obj;
}

void CartesianPoseCost::subdivide(const std::vector<double>& insertTimes, const VectorXd& oldTimes,
    const VectorXd& newTimes) {
  BOOST_FOREACH(double t, insertTimes) {
    if (t < m_timestep) {
      ++m_timestep;
    }
  }
  LOG_INFO_FMT("CartPoseCost subdiv: new traj len: %i. cnt timestep: %i", m_problem->m_times.size(), m_timestep);
}

double CartesianPoseCost::getCost() {
  // xxx this just calculates the current cost
  ScopedRobotSave srs(m_manip->robot->robot);
  m_manip->setDOFValues(toDoubleVec(m_problem->m_currentTraj.row(m_timestep)));
  btTransform tf = util::toBtTransform(m_manip->manip->GetTransform());
  if (m_l1) {
    return m_posCoeff * (toVector3d(tf.getOrigin()) - m_posTarg).lpNorm<1>() + m_rotCoeff
          * (toVector4d(tf.getRotation()) - m_rotTarg).lpNorm<1>();
  }
  else {
    return m_posCoeff * (toVector3d(tf.getOrigin()) - m_posTarg).squaredNorm() + m_rotCoeff
        * (toVector4d(tf.getRotation()) - m_rotTarg).squaredNorm();
  }
}

#if 0
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
      m_cnts.push_back(m_problem->m_model->addConstr(erri <= m_posTol, "pos"));
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
      m_cnts.push_back(m_problem->m_model->addConstr(erri <= m_rotTol, "rot"));
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

void CartesianPoseConstraint::subdivide(const std::vector<double>& insertTimes,
    const VectorXd& oldTimes, const VectorXd& newTimes) {
  BOOST_FOREACH(double t, insertTimes) {
    if (t < m_timestep) {
      ++m_timestep;
    }
  }
  LOG_INFO_FMT("CartPoseConstraint subdiv: new traj len: %i. cnt timestep: %i", m_problem->m_times.size(), m_timestep);
}
#endif





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
  for (int i = 0; i < expr.size(); ++i) {
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
      GRBLinExpr diffi = diff[iDim] + (jacDotThetaTo - jacs[iStep + 1].row(iDim).dot(traj.row(iStep
          + 1 + m_start))) - (jacDotThetaFrom
          - jacs[iStep].row(iDim).dot(traj.row(iStep + m_start)));
      //      sqdist += diffi * diffi;

#ifdef RESET_CONSTRAINTS
      m_cnts.push_back(m_problem->m_model->addConstr(diffi <= m_maxDist, "eevel"));
      m_cnts.push_back(m_problem->m_model->addConstr(diffi >= -m_maxDist, "eevel"));
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

void CartesianVelConstraint::subdivide(const std::vector<double>& insertTimes,
    const VectorXd& oldTimes, const VectorXd& newTimes) {
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

PlanningProblem::PlanningProblem() :
  m_model(new GRBModel(*grbEnv)), m_exactObjectiveReady(false), m_approxObjectiveReady(false) {
}

void PlanningProblem::addComponent(ProblemComponentPtr comp) {
  assert (m_initialized);
  m_comps.push_back(comp);
  comp->m_problem = this;
  comp->onAdd();
}

void PlanningProblem::removeComponent(ProblemComponentPtr comp) {
  typedef vector<ProblemComponentPtr> ComponentList;
  ComponentList::iterator it = std::find(m_comps.begin(), m_comps.end(), comp);
  assert(it != m_comps.end());
  m_comps.erase(it);
  (*it)->onRemove();
}

void PlanningProblem::addTrustRegionAdjuster(TrustRegionAdjusterPtr tra) {
  m_tra = tra;
  addComponent(tra);
}

void PlanningProblem::updateModel() {
  // todo: they should directly modify the objective, since they directly modify the model thru constraints
  GRBQuadExpr objective(0);
  for (int i = 0; i < m_comps.size(); ++i)
    m_comps[i]->updateModel(m_currentTraj, objective);
  m_model->update();
  m_model->setObjective(objective);
  m_exactObjectiveReady = true;
}

double PlanningProblem::getApproxCost() {
	#if 0
  ENSURE(m_approxObjectiveReady);
  double out = 0;
  BOOST_FOREACH(CostFuncPtr comp, getCostComponents()) {
      out += comp->getApproxCost();
      LOG_INFO(getClassName(*comp) << " approx after: " << comp->getApproxCost());
  }
    ASSERT_ALMOST_EQUAL2(out,m_model->get(GRB_DoubleAttr_ObjVal), 1e-5, 1e-8);
	#endif
  return m_model->get(GRB_DoubleAttr_ObjVal);
}

double PlanningProblem::getCachedCost() {
  ENSURE(m_exactObjectiveReady);
  double out = 0;
  BOOST_FOREACH(CostFuncPtr comp, getCostComponents()) {
      out += comp->getCachedCost();
//      ASSERT_ALMOST_EQUAL2(comp->getCost(),comp->getCachedCost(), 1e-5, 1e-7);
      LOG_INFO(getClassName(*comp) << " exact before: " << comp->getCachedCost());
  }
  return out;
}

void PlanningProblem::forceOptimizeHere() {
  vector<GRBConstr> cnts;
  for (int i = 0; i < m_currentTraj.rows(); ++i) {
    for (int j = 0; j < m_currentTraj.cols(); ++j) {
      cnts.push_back(m_model->addConstr(m_trajVars.at(i, j)
          == m_currentTraj(i, j)));
    }
  }
  optimizeModel();
  BOOST_FOREACH(GRBConstr& cnt, cnts)
    m_model->remove(cnt);
}

std::vector<CostFuncPtr> PlanningProblem::getCostComponents() {
  vector<CostFuncPtr> out;
  BOOST_FOREACH(ProblemComponentPtr comp, m_comps) {
    CostFuncPtr costComp = boost::dynamic_pointer_cast<CostFunc>(comp);
    if (costComp) out.push_back(costComp);
  }
  return out;
}

std::vector<ProblemComponentPtr> PlanningProblem::getConstraintComponents() {
  vector<ProblemComponentPtr> out;
  BOOST_FOREACH(ProblemComponentPtr comp, m_comps)
    if (comp->getAttrs() & ProblemComponent::HAS_CONSTRAINT) out.push_back(comp);
  return out;
}

void PlanningProblem::testObjectives() {

  LOG_INFO("evaluating constant part of linearization");

  map<ProblemComponentPtr, double> comp2exact, comp2approx;
  updateModel();
  forceOptimizeHere();

  BOOST_FOREACH(CostFuncPtr comp, getCostComponents()) {
      double approxObj = comp->getApproxCost();
      double exactObj = comp->getCachedCost();
      comp2approx[comp] = approxObj;
      comp2exact[comp] = exactObj;
      LOG_INFO_FMT("%s: exact: %.3f, approx: %.3f", getClassName(*comp).c_str(), exactObj, approxObj);
      ASSERT_ALMOST_EQUAL2(approxObj, exactObj, 1e-5,1e-7);
  }
  // GRB requires you to call optimize before evaluating expr
  LOG_INFO("Orig objective: " << getApproxCost());

  double epsilon = 1e-5;
  MatrixXd savedTraj = m_currentTraj;

  map<ProblemComponentPtr, MatrixXd> prob2linGrad, prob2numGrad;
  // gradient of linearization, numerical gradient
  BOOST_FOREACH(ProblemComponentPtr prob, getCostComponents()) {
    prob2linGrad[prob] = NAN * MatrixXd::Ones(savedTraj.rows(), savedTraj.cols());
    prob2numGrad[prob] = NAN * MatrixXd::Ones(savedTraj.rows(), savedTraj.cols());
  }

  for (int i = 0; i < m_currentTraj.rows(); ++i){
    for (int j = 0; j < m_currentTraj.cols(); ++j) {
      m_currentTraj(i, j) = savedTraj(i, j) + epsilon;
      BOOST_FOREACH(CostFuncPtr comp, getCostComponents()) {
        forceOptimizeHere();// XXX might be infeasible!!!
        int status = m_model->get(GRB_IntAttr_Status);
        if (status == GRB_OPTIMAL) {
//            LOG_INFO_FMT("gradient test %s %i %i", getClassName(*comp).c_str(), i, j);
          double pertApproxObj = comp->getApproxCost();
          double approxGrad = (pertApproxObj - comp2approx[comp]) / (epsilon);
          prob2linGrad[comp](i, j) = approxGrad;
        }
      }
      m_currentTraj(i, j) = savedTraj(i, j);
    }
  }

  for (int i = 0; i < m_currentTraj.rows(); ++i){
    for (int j = 0; j < m_currentTraj.cols(); ++j) {
      m_currentTraj(i, j) = savedTraj(i, j) + epsilon;
      updateModel();
      BOOST_FOREACH(CostFuncPtr comp, getCostComponents()) {
//      LOG_INFO_FMT("gradient test %s %i %i", getClassName(*comp).c_str(), i, j);
        double pertExactObj = comp->getCachedCost();
        double exactGrad = (pertExactObj - comp2approx[comp]) / (epsilon);
        prob2numGrad[comp](i, j) = exactGrad;
      }
      m_currentTraj(i, j) = savedTraj(i, j);
    }
  }

  BOOST_FOREACH(ProblemComponentPtr comp, getCostComponents()) {
    cout << "grad check: " << getClassName(*comp) << "-------------" << endl;
    cout << "convexification / numerical" << endl;
    for (int i = 0; i < m_currentTraj.rows(); ++i)
      for (int j = 0; j < m_currentTraj.cols(); ++j) {
        printf("%i %i %.3e %.3e\n",i,j,prob2linGrad[comp](i,j), prob2numGrad[comp](i,j) );
      }
  }

  exit(0);
}

void PlanningProblem::optimizeModel() {
  m_model->optimize();
  m_approxObjectiveReady = true;
}

void PlanningProblem::optimize(int maxIter) {

  assert(m_tra);

  MatrixXd prevTraj;
  vector<double> trueObjBefore, approxObjAfter;

  TIC();
  updateModel();
  LOG_INFO_FMT("total convexification time: %.2f", TOC());

  BOOST_FOREACH(TrajPlotterPtr plotter, m_plotters)
    plotter->plotTraj(m_currentTraj);
  if (SQPConfig::pauseEachIter) {
    util::getGlobalScene()->step(0);
    util::getGlobalScene()->idle(true);
  }


  double trueObj = getCachedCost();
  trueObjBefore.push_back(trueObj);
  LOG_INFO_FMT("Total exact objective before: %.2f", trueObj);

  for (int iter = 0; iter < maxIter;) {


    assert(trueObjBefore.size() == iter+1);
    assert(approxObjAfter.size() == iter);

    LOG_INFO_FMT("iteration: %i",iter);
// #define DEBUG_ALL_COSTS
#ifdef DEBUG_ALL_COSTS
    vector<double> costs_before, acosts_before;
    BOOST_FOREACH(CostFuncPtr pc, getCostComponents()) {
      CostFuncPtr pc = boost::dynamic_pointer_cast<CostFunc>(m_comps[0]);
      costs_before.push_back(pc->getCost());
      forceOptimizeHere();
      acosts_before.push_back(pc->getApproxCost());
    }
#endif

    TIC1();
    optimizeModel();
    LOG_INFO_FMT("optimization time: %.2f", TOC());
    int status = m_model->get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL) {
      LOG_FATAL_FMT("Bad GRB status: %s", grb_statuses[status]);
      throw;
    }
    printAllConstraints(*m_model);

    double approxObj = getApproxCost();
    prevTraj = m_currentTraj;
    updateTraj(m_trajVars, m_currentTraj);

#ifdef DEBUG_ALL_COSTS
    vector<double> costs_after, acosts_after;
    int i=0;
    BOOST_FOREACH(CostFuncPtr pc, getCostComponents()) {
      costs_after.push_back(pc->getCost());
      acosts_after.push_back(pc->getApproxCost());
      LOG_ERROR_FMT("exact before/after/diff: %.3e %.3e %.3e", costs_before.back(), costs_after.back(), costs_before.back()-costs_after.back());
      LOG_ERROR_FMT("approx before/after/diff: %.3e %.3e %.3e", acosts_before.back(), acosts_after.back(), acosts_before.back()-acosts_after.back());
      ++i;
    }
#endif


    BOOST_FOREACH(TrajChangePlotterPtr plotter, m_changePlotters)
      plotter->plotTrajs(prevTraj, m_currentTraj);



    m_approxObjectiveReady = false;
    m_exactObjectiveReady = false;
    TIC();
    updateModel();

    BOOST_FOREACH(TrajPlotterPtr plotter, m_plotters)
      plotter->plotTraj(m_currentTraj);
    if (SQPConfig::pauseEachIter) {
      util::getGlobalScene()->step(0);
      util::getGlobalScene()->idle(true);
    }

    LOG_INFO_FMT("total convexification time: %.2f", TOC());
    LOG_DEBUG("current traj:"<<endl<<m_currentTraj);

    double trueObj = getCachedCost();

    LOG_DEBUG("trajectory increment" << endl << m_currentTraj - prevTraj);

    if (trueObj > trueObjBefore.back()) {
      m_currentTraj = prevTraj;
      LOG_INFO("objective got worse! rolling back and shrinking trust region.");
      m_tra->adjustTrustRegion(SQPConfig::trShrink);
			updateModel();
    }
    else {
    LOG_INFO_FMT("Total exact objective: %.2f", trueObj);
    LOG_INFO("accepting traj modification");
    trueObjBefore.push_back(trueObj);
    approxObjAfter.push_back(approxObj);
    LOG_DEBUG("approx obj history:" << approxObjAfter);
    LOG_DEBUG("exact obj history:" << trueObjBefore);


    double trueImprove = trueObjBefore[trueObjBefore.size() - 2] - trueObjBefore.back();
    double approxImprove = trueObjBefore[trueObjBefore.size() - 2] - approxObj;
    if (approxImprove < 0) LOG_ERROR("wtf approxImprove < 0");
    double improveRatio = trueImprove / approxImprove;
    LOG_INFO_FMT("true improvement: %.2e.  approx improvement: %.2e.  ratio: %.2f", trueImprove, approxImprove, improveRatio);
    if (improveRatio < SQPConfig::trThresh) {
      m_tra->adjustTrustRegion(SQPConfig::trShrink);
    }
    else {
      m_tra->adjustTrustRegion(SQPConfig::trExpand);
    }

    if (trueImprove < SQPConfig::doneIterThresh) {
      LOG_INFO("cost didn't improve much. stopping iteration");
      break;
    }
    ++iter;
    }

    if (m_tra->m_shrinkage < SQPConfig::shrinkLimit) {
      LOG_INFO("trust region got shrunk too much. stopping");
      break;
    }



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
  int timesteps = m_currentTraj.rows();
//  m_optMask(0) = false;
//  if (endFixed) m_optMask(initTraj.rows() - 1) = false;
  for (int iRow = 0; iRow < timesteps; ++iRow) {
    for (int iCol = 0; iCol < m_currentTraj.cols(); ++iCol) {
      char namebuf[10];
      sprintf(namebuf, "j_%i_%i", iRow, iCol);
      m_trajVars.at(iRow, iCol) = m_model->addVar(0, 0, 0, GRB_CONTINUOUS, namebuf);
    }
  }
  m_model->update();
  // Add equality constraints for endpoints
  for(int j=0; j < m_currentTraj.cols(); j++){
    m_model->addConstr(m_trajVars.at(0, j), GRB_EQUAL, m_currentTraj(0, j));
  }
  if(endFixed){
    for(int j=0; j < m_currentTraj.cols(); j++){
      m_model->addConstr(m_trajVars.at(timesteps-1, j) == m_currentTraj(timesteps-1, j));
    }
  }

  m_model->update();
  int nRow = m_currentTraj.rows();
  for (int i=0; i < initTraj.cols(); ++i) {
    if (true) m_model->addConstr(m_trajVars(0,i) == m_currentTraj(0,i),"startfixed");
    if (endFixed) m_model->addConstr(m_trajVars(nRow-1,i) == m_currentTraj(nRow- 1,i),"endfixed");
  }
  //  setVarsToTraj(m_currentTraj,m_optMask, m_trajVars);
  m_initialized = true;
}

void PlanningProblem::writeTrajToJSON(std::string filename){
  Json::Value outputData(Json::objectValue);
  outputData["trajectory"] = Json::Value(Json::objectValue);
  int rowCount = m_currentTraj.rows();
  outputData["trajectory"]["length"] = Json::Value(rowCount);
  outputData["trajectory"]["indices"] = Json::Value(Json::arrayValue);
  outputData["trajectory"]["values"] = Json::Value(Json::arrayValue);
  for(int i = 0; i< m_currentTraj.rows(); i++){
    Json::Value row = Json::Value(Json::arrayValue);
    for(int j = 0; j < m_currentTraj.cols(); j++){
      row.append(m_currentTraj(i,j));
    }
    outputData["trajectory"]["values"].append(row);
  }
  std::ofstream outputFile;
  outputFile.open(filename.c_str());
  Json::StyledWriter writer;
  outputFile << writer.write(outputData);
  outputFile.close();
}

Eigen::MatrixXd loadTrajFromJSON(std::string filename){
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(filename.c_str(), root);
  if(!success){
    LOG_ERROR_FMT("Failed to load trajectory data from %s", filename.c_str());
    return MatrixXd();
  }

  int length = root["trajetory"]["length"].asInt();
  int width = root["trajectory"]["values"][0].size();
  MatrixXd trajData(length, width);
  for(int i = 0; i < length; i++){
    for(int j = 0; j < width; j++){
      trajData(i,j) = root["trajectory"]["values"][i][j].asDouble();
    }
  }
  return trajData;
}
