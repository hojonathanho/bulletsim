#include "traj_sqp.h"
#include <boost/foreach.hpp>
#include "config_sqp.h"
#include "utils/logging.h"
#include "collisions.h"
#include <map>
#include "simulation/openravesupport.h"
#include "plotters.h"
#include "functions.h"
#include "kinematics_utils.h"
using std::map;

TrajOptimizer::TrajOptimizer() : m_initialized(false) {}

void TrajOptimizer::updateValues() {
  for (int i = 0; i < m_traj.rows(); ++i) {
    for (int j = 0; j < m_traj.cols(); ++j) {
      m_traj(i, j) = m_vars.at(i, j).get(GRB_DoubleAttr_X);
    }
	}
}

void TrajOptimizer::storeValues() {
	m_traj_backup = m_traj;
}

void TrajOptimizer::rollbackValues() {
	m_traj = m_traj_backup;
}

void TrajOptimizer::initialize(const MatrixXd& traj, const VectorXd& times) {
  assert(!m_initialized);
	m_traj = traj;
	m_times = times;
	m_vars.resize(traj.rows(), traj.cols());
	
  for (int iRow = 0; iRow < traj.rows(); ++iRow) {
    for (int iCol = 0; iCol < traj.cols(); ++iCol) {
      char namebuf[10];
      sprintf(namebuf, "j_%i_%i", iRow, iCol);
      m_vars.at(iRow, iCol) = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, namebuf);
    }
  }
  m_model->update();	
  m_initialized = true;
}

void TrajOptimizer::preOptimize() {
  if (SQPConfig::enablePlot) {
    BOOST_FOREACH(TrajPlotterPtr& plotter, m_plotters) plotter->plotTraj(m_traj);
  }
	if (SQPConfig::pauseEachIter) {
	  pauseScene();
	}

}

void TrajOptimizer::postOptimize() {
	LOG_INFO("todo: postOptimize() plots");
}

void TrajOptimizer::setStartFixed() {
  assert(m_initialized);
  int row = 0;
  for (int col=0; col < m_vars.cols(); ++col) {
    m_model->addConstr(m_vars(row, col) == m_traj(row,col));
  }
}
void TrajOptimizer::setEndFixed() {
  assert(m_initialized);
  int row = m_vars.rows() - 1;
  for (int col=0; col < m_vars.cols(); ++col) {
    m_model->addConstr(m_vars(row, col) == m_traj(row,col));
  }
}

TrajComponent::TrajComponent(TrajOptimizer* opt) : m_opt(opt) {}

#define MEMO_CALC(lhs, rhs, key, cache) do {\
	typeof(cache)::iterator it = cache.find(key);\
	if (it != cache.end()) lhs = it->second;\
	else cache[key] = lhs = rhs;\
} while (0)		


static map<double, TrajCartCollInfo> collisionCache;

CollisionCost::CollisionCost(TrajOptimizer* opt, RaveRobotObject* robot, const vector<int>& dofInds, bool useAffine, double coeff) :
    TrajComponent(opt), m_robot(robot), m_dofInds(dofInds), m_useAffine(useAffine), m_coeff(coeff) {}

double CollisionCost::evaluate() {
	TrajCartCollInfo tcci = collectTrajCollisions(getTraj(), m_robot, m_dofInds, m_useAffine);
	double out = 0;
	BOOST_FOREACH(CartCollInfo& cci, tcci) {
		BOOST_FOREACH(LinkCollision& lc, cci) {
			out += m_coeff * pospart(SQPConfig::distPen - lc.dist);
		}
	}
	return out;
}

GRBLinExpr varDot(const VectorXd& x, const VarVector& v) {
  assert(x.size() == v.size());
  GRBLinExpr out;
  out.addTerms(x.data(), v.data(), x.size());
	return out;
}

ConvexObjectivePtr CollisionCost::convexify(GRBModel* model) {
	ConvexObjectivePtr out(new ConvexObjective());
	TrajCartCollInfo tcci = collectTrajCollisions(getTraj(), m_robot, m_dofInds, m_useAffine);
  TrajJointCollInfo trajJointInfo = trajCartToJointCollInfo(tcci, getTraj(), m_robot->robot,
      m_dofInds, m_useAffine);
	
  if (SQPConfig::enablePlot) plotCollisions(tcci, SQPConfig::distDiscSafe);

  for (int iStep = 0; iStep < getTraj().rows(); ++iStep) {
    vector<VectorXd>& jacs = trajJointInfo[iStep].jacs;
    vector<double>& dists = trajJointInfo[iStep].dists;
    for (int iColl = 0; iColl < trajJointInfo[iStep].jacs.size(); ++iColl) {
      GRBVar hinge = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "coll_hinge");
			out->m_vars.push_back(hinge);
      out->m_exprs.push_back(-hinge + SQPConfig::distPen - dists[iColl]
             + varDot(jacs[iColl], getVars().row(iStep)) - jacs[iColl].dot(getTraj().row(iStep)));
			out->m_cntNames.push_back("coll_hinge");			
    }
  }
	out->m_objective = varDot(VectorXd::Constant(out->m_vars.size(), out->m_vars.size()), out->m_vars);
	return out;
}


CartPoseCost::CartPoseCost(TrajOptimizer* opt, RobotManipulatorPtr arm, void* link, const vector<int>& dofInds,
             bool useAffine, const btTransform& goal, double posCoeff, double rotCoeff, bool l1) :
             TrajComponent(opt),
             m_arm(arm),
             m_link(link),
             m_dofInds(dofInds),
             m_useAffine(useAffine),
             m_posTarg(toVector3d(goal.getOrigin())),
             m_rotTarg(toVector4d(goal.getRotation())),
             m_posCoeff(posCoeff),
             m_rotCoeff(rotCoeff),
             m_l1(l1)
{}


double CartPoseCost::evaluate() {
  ScopedRobotSave(m_arm->robot->robot);
  VectorXd dofvals = getTraj().row(getLength()-1);
  if (m_useAffine) setDofVals(m_arm->robot->robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
  else setDofVals(m_arm->robot->robot, m_dofInds, dofvals);
  OpenRAVE::Transform tfCur = static_cast<KinBody::Link*>(m_link)->GetTransform();
  Vector3d posCur = toVector3d(tfCur.trans);
  Vector4d rotCur = toQuatVector4d(tfCur.rot);
  if (rotCur.dot(m_rotTarg) < 0) {
    m_rotTarg *= -1;
  }
  if (m_l1) {
    return m_rotCoeff * (m_rotTarg - rotCur).lpNorm<1>() + m_posCoeff * (m_posTarg - posCur).lpNorm<1>();
  }
  else {
    return m_rotCoeff * (m_rotTarg - rotCur).squaredNorm() + m_posCoeff * (m_posTarg - posCur).squaredNorm();
  }
}


ConvexObjectivePtr CartPoseCost::convexify(GRBModel* model) {
  ScopedRobotSave(m_arm->robot->robot);
  VectorXd dofvals = getTraj().row(getLength()-1);
  if (m_useAffine) setDofVals(m_arm->robot->robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
  else setDofVals(m_arm->robot->robot, m_dofInds, dofvals);
  MatrixXd posjac, rotjac;
  calcActiveLinkJac(static_cast<KinBody::Link*>(m_link), m_arm->robot->robot,  posjac, rotjac);
  OpenRAVE::Transform tfCur = static_cast<KinBody::Link*>(m_link)->GetTransform();
  Vector3d posCur = toVector3d(tfCur.trans);
  Vector4d rotCur = toQuatVector4d(tfCur.rot);
  if (rotCur.dot(m_rotTarg) < 0) {
    m_rotTarg *= -1;
  }

  VarVector vars = getVars().row(getLength()-1);

  ConvexObjectivePtr out(new ConvexObjective());
  // TODO: use makeDerivExpr
  // TODO: switch sign of all inequalities

  if (m_posCoeff > 0) {
    for (int i=0; i < 3; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(posjac.row(i).data(), vars.data(), getDof());
      GRBLinExpr erri = posCur(i) - m_posTarg(i) + jacDotTheta - posjac.row(i).dot(dofvals);
      if (m_l1) addHingeCost(out, m_posCoeff, erri, model, "pos_l1");
      else  out->m_objective += m_posCoeff * (erri * erri);
    }
  }

  if (m_rotCoeff > 0) {
    for (int i=0; i < 4; ++i) {
      GRBLinExpr jacDotTheta;
      jacDotTheta.addTerms(rotjac.row(i).data(), vars.data(), getDof());
      GRBLinExpr erri = posCur(i) - m_rotTarg(i) + jacDotTheta - rotjac.row(i).dot(dofvals);
      if (m_l1) addHingeCost(out, m_rotCoeff, erri, model, "rot_l1");
      else  out->m_objective += m_rotCoeff * (erri * erri);
    }
  }

  return out;

}



JntLenCost::JntLenCost(TrajOptimizer* opt, double coeff) : TrajComponent(opt), m_coeff(coeff) {}

ConvexObjectivePtr JntLenCost::convexify(GRBModel* model) {
	VarArray& vars = getVars();

	ConvexObjectivePtr out(new ConvexObjective());	
	
  for (int iStep = 1; iStep < vars.rows(); ++iStep) {
    for (int iJoint = 0; iJoint < vars.cols(); ++iJoint) {
      GRBLinExpr vel = vars(iStep, iJoint) - vars(iStep-1, iJoint);
      double dt = getTimes()(iStep) - getTimes()(iStep - 1);
      out->m_objective += m_coeff * vars.rows() * dt * (vel * vel); 
			// xxx should scale other costs to make them invariant to sampling
			// instead of making this proportional to sampling
    }
	}
	
	return out;

}

double JntLenCost::evaluate() {
  MatrixXd& vars = getTraj();
  double out = 0;
  for (int iStep = 1; iStep < vars.rows(); ++iStep) {
    for (int iJoint = 0; iJoint < vars.cols(); ++iJoint) {
      double vel = vars(iStep, iJoint) - vars(iStep-1, iJoint);
      double dt = getTimes()(iStep) - getTimes()(iStep - 1);
      out += m_coeff * vars.rows() * dt * (vel * vel);
      // xxx should scale other costs to make them invariant to sampling
      // instead of making this proportional to sampling
    }
  }
  return out;
}

JointBounds::JointBounds(TrajOptimizer* opt, const VectorXd& maxDiffPerIter, const VectorXd& jointLowerLimit, const VectorXd& jointUpperLimit) :
    TrajComponent(opt), m_maxDiffPerIter(maxDiffPerIter), m_jointLowerLimit(jointLowerLimit), m_jointUpperLimit(jointUpperLimit) {}

void JointBounds::adjustTrustRegion(double ratio) {
  m_maxDiffPerIter *= ratio;
  m_shrinkage *= ratio;
}

ConvexConstraintPtr JointBounds::convexify(GRBModel* model) {
	VarArray& vars = getVars();
	MatrixXd& traj = getTraj();
	
  for (int iStep = 0; iStep < traj.rows(); ++iStep) {
    for (int iJoint = 0; iJoint < traj.cols(); ++iJoint) {
      vars(iStep, iJoint).set(GRB_DoubleAttr_LB, fmax(traj(iStep, iJoint)
          - m_maxDiffPerIter(iJoint), m_jointLowerLimit(iJoint)));
      vars(iStep, iJoint).set(GRB_DoubleAttr_UB, fmin(traj(iStep, iJoint)
          + m_maxDiffPerIter(iJoint), m_jointUpperLimit(iJoint)));
    }
  }
	
	ConvexConstraintPtr out(new ConvexConstraint());
	return out;
}
