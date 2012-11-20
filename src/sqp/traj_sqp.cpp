#include "traj_sqp.h"
#include <boost/foreach.hpp>
#include "config_sqp.h"
#include "utils/logging.h"
#include "collisions.h"
#include <map>
#include "simulation/openravesupport.h"
#include "simulation/bullet_io.h"
#include "plotters.h"
#include "functions.h"
#include "kinematics_utils.h"
#include "utils/clock.h"
#include "utils/interpolation.h"
#include "expr_ops.h"
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
  LOG_DEBUG("traj increment:\n" << m_traj - m_traj_backup);
}

TimestepFixed::TimestepFixed(TrajOptimizer* opt, int step) : TrajComponent(opt), m_step(step) {}

ConvexConstraintPtr TimestepFixed::convexify(GRBModel* model) {
  int row = (m_step >= 0) ? m_step : (getLength()+m_step);
  ConvexConstraintPtr out(new ConvexConstraint());
  MatrixXd& traj = getTraj();
  VarArray& vars = getVars();
  for (int col=0; col < getDof(); ++col) {
    out->m_eqcnts.push_back(model->addConstr(vars(row, col) == traj(row, col)));
  }
  return out;
}

void setStartFixed(TrajOptimizer& opt) {
  opt.m_cnts.push_back(TimestepFixedPtr(new TimestepFixed(&opt, 0)));
}
void setEndFixed(TrajOptimizer& opt) {
  opt.m_cnts.push_back(TimestepFixedPtr(new TimestepFixed(&opt, -1)));
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


void TrajOptimizer::subdivide(const vector<double>& insertTimes) {
  vector<int> oldInds, newInds;
  vector<double> newTimesVec;
  merged(toDoubleVec(m_times), insertTimes, newTimesVec, oldInds, newInds);
  for (int i = 0; i < newTimesVec.size() - 1; ++i) assert(newTimesVec[i] != newTimesVec[i+1]);
  VectorXd oldTimes = m_times;
  VectorXd newTimes = toVectorXd(newTimesVec);
  MatrixXd newTraj = interp2d(newTimes, m_times, m_traj);
  LOG_INFO("new times: " << newTimes.transpose());

  m_initialized = false;
  m_model->reset();
  initialize(newTraj, newTimes);
  
  BOOST_FOREACH(CostPtr comp, m_costs) {
    boost::dynamic_pointer_cast<TrajComponent>(comp)->subdivide(insertTimes, oldTimes, newTimes);
  }
  BOOST_FOREACH(ConstraintPtr comp, m_cnts) {
    boost::dynamic_pointer_cast<TrajComponent>(comp)->subdivide(insertTimes, oldTimes, newTimes);
  }
    
}

TrajComponent::TrajComponent(TrajOptimizer* opt) : m_opt(opt) {}
TrajCost::TrajCost(TrajOptimizer* opt) : TrajComponent(opt) {}
TrajComponent::~TrajComponent() {}
TrajConstraint::TrajConstraint(TrajOptimizer* opt) : TrajComponent(opt) {}

ConvexObjectivePtr TrajConstraint::getViolExpr(GRBModel* model) {
  ConvexObjectivePtr cost(new ConvexObjective());
  ConvexConstraintPtr cnt = convexify(model);
  cost->m_vars = cnt->m_vars;
  BOOST_FOREACH(GRBLinExpr& expr, cnt->m_exprs) {
    addHingeCost(cost, 1, expr, model, "viol_hinge");
  }
  return cost;
}

TrajCostPtr TrajConstraint::asCost() {
  struct ThisAsCost : public TrajCost {
    TrajConstraint* m_cnt;
    ThisAsCost(TrajConstraint* cnt) : TrajCost(cnt->m_opt), m_cnt(cnt) {}
    double evaluate(const MatrixXd& traj) {
      return m_cnt->getViolVal(traj);
    }
    ConvexObjectivePtr convexify(GRBModel* model) {
      return m_cnt->getViolExpr(model);
    }
  };
  TrajCost* thisAsCost = new ThisAsCost(this);
  return TrajCostPtr(thisAsCost);
}

#define MEMO_CALC(lhs, rhs, key, cache) do {\
	typeof(cache)::iterator it = cache.find(key);\
	if (it != cache.end()) lhs = it->second;\
	else cache[key] = lhs = rhs;\
} while (0)		



CollisionCost::CollisionCost(TrajOptimizer* opt, RaveRobotObject* robot, const vector<int>& dofInds, bool useAffine, double coeff) :
    TrajCost(opt), m_robot(robot), m_dofInds(dofInds), m_useAffine(useAffine), m_coeff(coeff) {}

double CollisionCost::evaluate(const MatrixXd& traj) {
	TrajCartCollInfo tcci = collectTrajCollisions(traj, m_robot, m_dofInds, m_useAffine);
	double out = 0;
	BOOST_FOREACH(CartCollInfo& cci, tcci) {
		BOOST_FOREACH(LinkCollision& lc, cci) {
			out += m_coeff * pospart(SQPConfig::distPen - lc.dist) * lc.frac;
		}
	}
	return out;
}

ConvexObjectivePtr CollisionCost::convexify(GRBModel* model) {
	ConvexObjectivePtr out(new ConvexObjective());
	TrajCartCollInfo tcci = discreteCollisionCheck();
  TrajJointCollInfo trajJointInfo = trajCartToJointCollInfo(tcci, getTraj(), m_robot->robot,
      m_dofInds, m_useAffine);
	
  if (SQPConfig::enablePlot) plotCollisions(tcci, SQPConfig::distDiscSafe);

  vector<double> coeffVec;

  for (int iStep = 0; iStep < getTraj().rows(); ++iStep) {
    vector<VectorXd>& jacs = trajJointInfo[iStep].jacs;
    vector<double>& dists = trajJointInfo[iStep].dists;
    for (int iColl = 0; iColl < trajJointInfo[iStep].jacs.size(); ++iColl) {
      GRBVar hinge = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "coll_hinge");
			out->m_vars.push_back(hinge);
      out->m_exprs.push_back(-hinge + SQPConfig::distPen - dists[iColl]
             + varDot(jacs[iColl], getVars().row(iStep)) - jacs[iColl].dot(getTraj().row(iStep)));
			out->m_cntNames.push_back("coll_hinge");
			coeffVec.push_back(m_coeff*trajJointInfo[iStep].weights[iColl]);
    }
  }
	out->m_objective = varDot(toVectorXd(coeffVec), out->m_vars);
	return out;
}

TrajCartCollInfo CollisionCost::discreteCollisionCheck() {
  return collectTrajCollisions(getTraj(), m_robot, m_dofInds, m_useAffine);  
}
TrajCartCollInfo CollisionCost::continuousCollisionCheck() {
  return  continuousTrajCollisions(getTraj(), m_robot, m_dofInds, m_useAffine, SQPConfig::distContSafe);
}

CartPoseCost::CartPoseCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link, const vector<int>& dofInds,
             bool useAffine, const btTransform& goal, double posCoeff, double rotCoeff, bool l1) :
             TrajCost(opt),
             m_robot(robot),
             m_link(link),
             m_dofInds(dofInds),
             m_useAffine(useAffine),
             m_posTarg(toVector3d(goal.getOrigin()).transpose()),
             m_rotTarg(toVector4d(goal.getRotation()).transpose()),
             m_posCoeff(VectorXd::Ones(1)*posCoeff),
             m_rotCoeff(VectorXd::Ones(1)*rotCoeff),
             m_l1(l1),
             m_justEnd(true)
{}

CartPoseCost::CartPoseCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link, const vector<int>& dofInds,
             bool useAffine, const vector<btTransform>& goal, const VectorXd& posCoeff, const VectorXd& rotCoeff, bool l1) :
             TrajCost(opt),
             m_robot(robot),
             m_link(link),
             m_dofInds(dofInds),
             m_useAffine(useAffine),
             m_posTarg(goal.size(), 3),
             m_rotTarg(goal.size(), 4),
             m_posCoeff(posCoeff),
             m_rotCoeff(rotCoeff),
             m_l1(l1),
             m_justEnd(false)
{
	assert (goal.size() == posCoeff.size());
	assert(goal.size() == rotCoeff.size());
  for (int i=0; i < goal.size(); ++i) {
    m_posTarg.row(i) = toVector3d(goal[i].getOrigin()).transpose();
    m_rotTarg.row(i) = toVector4d(goal[i].getRotation()).transpose();
  }
}



double CartPoseCost::evaluate(const MatrixXd& traj) {
  ScopedRobotSave srs(m_robot);
  m_robot->SetActiveDOFs(m_dofInds);
  double out = 0;
  
  int tStart = m_justEnd ? getLength()-1 : 0;
  int tEnd = getLength();
  
  for (int iStep=tStart; iStep < tEnd; ++iStep) {
    int row = m_justEnd ? 0 : iStep;
    VectorXd dofvals = traj.row(iStep);
    if (m_useAffine) setDofVals(m_robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
    else setDofVals(m_robot, m_dofInds, dofvals);
    OpenRAVE::Transform tfCur = m_link->GetTransform();
    VectorXd posCur = toVector3d(tfCur.trans);
    LOG_DEBUG("pos cur: " << posCur.transpose() << " pos targ: " << m_posTarg.row(row).transpose());
    VectorXd rotCur = toQuatVector4d(tfCur.rot);
    if (rotCur.dot(m_rotTarg.row(row)) < 0) {
      m_rotTarg.row(row) *= -1;
    }
    LOG_DEBUG("quat cur: " << rotCur.transpose() << " quat targ: " << m_rotTarg.row(row).transpose());
    if (m_l1) {
      out += m_rotCoeff(row) * (m_rotTarg.row(row).transpose() - rotCur).lpNorm<1>() + m_posCoeff(row) * (m_posTarg.row(row).transpose() - posCur).lpNorm<1>();
    }
    else {
      out += m_rotCoeff(row) * (m_rotTarg.row(row).transpose() - rotCur).squaredNorm() + m_posCoeff(row) * (m_posTarg.row(row).transpose() - posCur).squaredNorm();
    }
  }
  return out;
}



ConvexObjectivePtr CartPoseCost::convexify(GRBModel* model) {
  ScopedRobotSave srs(m_robot);
  m_robot->SetActiveDOFs(m_dofInds);
  ConvexObjectivePtr out(new ConvexObjective());

  int tStart = m_justEnd ? getLength()-1 : 0;
  int tEnd = getLength();
  
  for (int iStep=tStart; iStep < tEnd; ++iStep) {
    int row = m_justEnd ? 0 : iStep;
    VectorXd dofvals = getTraj().row(iStep);
    if (m_useAffine) setDofVals(m_robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
    else setDofVals(m_robot, m_dofInds, dofvals);
    MatrixXd posjac, rotjac;
    calcActiveLinkJac(dofvals, m_link.get(), m_robot,  posjac, rotjac, m_useAffine);
    OpenRAVE::Transform tfCur = m_link->GetTransform();
    Vector3d posCur = toVector3d(tfCur.trans);
    Vector4d rotCur = toQuatVector4d(tfCur.rot);
    if (rotCur.dot(m_rotTarg.row(row)) < 0) {
      m_rotTarg.row(row) *= -1;
    }

    VarVector vars = getVars().row(iStep);


    if (m_posCoeff(row) > 0) {
      for (int i=0; i < 3; ++i) {
        GRBLinExpr erri = posCur(i) - m_posTarg(row, i) + makeDerivExpr(posjac.row(i), vars, dofvals);
        if (m_l1) addAbsCost(out, m_posCoeff(row), erri, model, "pos_l1");
        else  out->m_objective += m_posCoeff(row) * (erri * erri);
      }
    }

    if (m_rotCoeff(row) > 0) {
      for (int i=0; i < 4; ++i) {
        GRBLinExpr erri = rotCur(i) - m_rotTarg(row, i) + makeDerivExpr(rotjac.row(i), vars, dofvals);
        if (m_l1) addAbsCost(out, m_rotCoeff(row), erri, model, "rot_l1");
        else  out->m_objective += m_rotCoeff(row) * (erri * erri);
      }
    }
    
  }
  return out;
}



ThisSideUpCost::ThisSideUpCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link, const vector<int>& dofInds,
             bool useAffine, double coeff, bool l1) :
             TrajCost(opt),
             m_robot(robot),
             m_link(link),
             m_dofInds(dofInds),
             m_useAffine(useAffine),
             m_coeff(coeff),
             m_l1(l1)
{
}


double ThisSideUpCost::evaluate(const MatrixXd& traj) {
  ScopedRobotSave srs(m_robot);
  double out = 0;

  btMatrix3x3 origRot;
  origRot.setValue(NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN,NAN);
  for (int t=0; t < traj.rows(); ++t) {
    VectorXd dofvals = traj.row(t);
    if (m_useAffine) setDofVals(m_robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
    else setDofVals(m_robot, m_dofInds, dofvals);
    btMatrix3x3 rot = util::toBtTransform(m_link->GetTransform()).getBasis();

    if (t == 0) {
      origRot = rot;
    }
    else {
      btVector3 up = (origRot * rot.transpose()).getColumn(2);
      Eigen::Vector3d err(up.x(), up.y(), up.z()-1);

      if (m_l1) out +=  m_coeff * (err).lpNorm<1>();
      else out += m_coeff * err.squaredNorm();

    }
  }
  return out;
}



ConvexObjectivePtr ThisSideUpCost::convexify(GRBModel* model) {
  ScopedRobotSave srs(m_robot);
  ConvexObjectivePtr out(new ConvexObjective());

  MatrixXd& traj = getTraj();
  VarArray& vars = getVars();

  for (int t=0; t < traj.rows(); ++t) {
    VectorXd dofvals = traj.row(t);
    if (m_useAffine) setDofVals(m_robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
    else setDofVals(m_robot, m_dofInds, dofvals);

    btMatrix3x3 origRot;
    if (t == 0) {
      origRot = util::toBtTransform(m_link->GetTransform()).getBasis();
    }
    else {

      struct F : public fVectorOfVector {
        KinBody::LinkPtr m_link;
        RobotBasePtr m_robot;
        bool m_useAffine;
        btMatrix3x3 m_origRot;
        F(KinBody::LinkPtr link, RobotBasePtr robot, bool useAffine, const btMatrix3x3& origRot) : m_link(link), m_robot(robot), m_useAffine(useAffine), m_origRot(origRot) {}
        VectorXd operator()(const VectorXd& vals) const {
          if (m_useAffine) setDofVals(m_robot, vector<int>(), vals.topRows(vals.size()-3), vals.bottomRows(3));
          else setDofVals(m_robot, vector<int>(), vals);
          btMatrix3x3 rot = util::toBtTransform(m_link->GetTransform()).getBasis();
          btVector3 up = (m_origRot * rot.transpose()).getColumn(2);
          return Eigen::Vector3d(up.x(), up.y(), up.z()-1);
        }
      };

      F f(m_link, m_robot, m_useAffine, origRot);
      MatrixXd jac = calcJacobian(f, dofvals, 1e-5);
      VectorXd errcur = f(dofvals);


      for (int i=0; i < errcur.size(); ++i) {
        GRBLinExpr erri =  errcur(i) + makeDerivExpr(jac.row(i), vars.row(t), dofvals);
        if (m_l1) addAbsCost(out, m_coeff, erri, model, "tsi_l1");
        else  out->m_objective += m_coeff * (erri * erri);
      }
    }
  }

  return out;

}





JntLenCost::JntLenCost(TrajOptimizer* opt, double coeff) : TrajCost(opt), m_coeff(coeff) {}

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

double JntLenCost::evaluate(const MatrixXd& traj) {
  double out = 0;
  for (int iStep = 1; iStep < traj.rows(); ++iStep) {
    for (int iJoint = 0; iJoint < traj.cols(); ++iJoint) {
      double vel = traj(iStep, iJoint) - traj(iStep-1, iJoint);
      double dt = getTimes()(iStep) - getTimes()(iStep - 1);
      out += m_coeff * traj.rows() * dt * (vel * vel);
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
  LOG_INFO("new trust region: " << m_maxDiffPerIter.transpose());
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

void getTrajPositionJacobians(RobotBasePtr robot, KinBody::LinkPtr link, const MatrixXd& traj, const vector<int>& dofInds, bool useAffine, 
	vector<MatrixXd>& jacs, vector<VectorXd>& positions) {
	ScopedRobotSave srs(robot);
	jacs.resize(traj.rows());
	positions.resize(traj.rows());
  int linkInd = getRobotLinkIndex(robot,  link);
	for (int i=0; i < traj.rows(); ++i) {
		VectorXd dofvals = traj.row(i);
		if (useAffine) setDofVals(robot, dofInds, dofvals.topRows(dofInds.size()), dofvals.bottomRows(3));
		else setDofVals(robot, dofInds, dofvals);
		positions[i] = toVector3d(link->GetTransform().trans);
		jacs[i] = calcPointJacobian(robot, linkInd, util::toBtVector(link->GetTransform().trans), useAffine);
	}
}


void getIndInterval(const VectorXd& times, double tStart, double tEnd, int& iStart, int& nSteps) {
  // block of indices i such that t(i) >= tstart and t(i) <= tend
  for (int i=0; i < times.size(); ++i) {
    if (times(i) >= tStart) iStart = i;
    break;
  }
  nSteps = -1;
  for (int i=iStart+1; i < times.size(); ++i) {
    if (times(i) > tEnd) {
      nSteps = i - iStart - 1;
      break;
    }
  }
  if (nSteps == -1) {
    nSteps = times.size() - iStart - 1;
  }
}

CartVelCnt::CartVelCnt(TrajOptimizer* opt, int tStart, int tEnd, double maxSpeed, RobotBasePtr robot,
                       KinBody::LinkPtr link, const vector<int>& dofInds, bool useAffine)
: TrajConstraint(opt),
  m_tStart(tStart),
  m_tEnd(tEnd),
  m_maxSpeed(maxSpeed),
  m_robot(robot),
  m_link(link),
  m_dofInds(dofInds),
  m_useAffine(useAffine)
{}

ConvexConstraintPtr CartVelCnt::convexify(GRBModel* model) {
  int iStart, nSteps;
	getIndInterval(getTimes(), m_tStart, m_tEnd, iStart, nSteps);

	MatrixXd traj = getTraj().middleRows(iStart, nSteps);
	VarArray vars = getVars().middleRows(iStart, nSteps);
	VectorXd times = getTimes().middleRows(iStart, nSteps);
		
	ConvexConstraintPtr out(new ConvexConstraint());

	vector<MatrixXd> jacs;
	vector<VectorXd> positions;
	getTrajPositionJacobians(m_robot, m_link, traj, m_dofInds, m_useAffine, jacs, positions);
	
	// make constraints
	for (int i=0; i < nSteps-1; ++i) {
		VectorXd disp = positions[i+1] - positions[i];
		double dt = times(i+1) - times(i);
		for (int dim = 0; dim < 3; ++dim) {
			GRBLinExpr diffi = (disp(dim) + makeDerivExpr(jacs[i+1].row(dim), vars.row(i+1), traj.row(i+1))
					 + makeDerivExpr(-jacs[i].row(dim), vars.row(i), traj.row(i)))/dt;
			out->m_exprs.push_back(diffi - m_maxSpeed);
			out->m_cntNames.push_back((boost::format("vel%i_%i<")%i%dim).str());
			out->m_exprs.push_back(- diffi - m_maxSpeed);
			out->m_cntNames.push_back((boost::format("vel%i_%i>")%i%dim).str());
		}
	}
	
	return out;
		
}

double CartVelCnt::getViolVal(const MatrixXd& newTraj) {
  int iStart, nSteps;
  getIndInterval(getTimes(), m_tStart, m_tEnd, iStart, nSteps);

  MatrixXd traj = newTraj.middleRows(iStart, nSteps);
  VectorXd times = getTimes().middleRows(iStart, nSteps);

  double out = 0;

  vector<MatrixXd> jacs;
  vector<VectorXd> positions;
  getTrajPositionJacobians(m_robot, m_link, traj, m_dofInds, m_useAffine, jacs, positions);

  // make constraints
  for (int i=0; i < nSteps-1; ++i) {
    out += (((positions[i+1] - positions[i])/(times[i+1]-times[i])).cwiseAbs() - VectorXd::Ones(3)*m_maxSpeed).lpNorm<1>();
  }

  return out;

}

CartAccCost::CartAccCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link,
                         const vector<int>& dofInds, bool useAffine, double coeff)
: TrajCost(opt),
  m_robot(robot),
  m_link(link),
  m_dofInds(dofInds),
  m_useAffine(useAffine),
  m_coeff(coeff)
{}

double CartAccCost::evaluate(const MatrixXd& traj) {
	double out = 0;
	VectorXd& times = getTimes();
  vector<MatrixXd> jacs;
  vector<VectorXd> positions;
  getTrajPositionJacobians(m_robot, m_link, traj, m_dofInds, m_useAffine, jacs, positions);

	for (int i=1; i < traj.rows()-1; ++i) {
    VectorXd a = ((positions[i+1] - positions[i])/(times(i+1) - times(i))
                - (positions[i] - positions[i-1])/(times(i) - times(i-1)))
                  /((times(i+1) - times(i-1))/2);
    out += m_coeff * a.squaredNorm();
	}
	return out;
}

ConvexObjectivePtr CartAccCost::convexify(GRBModel* model) {
	MatrixXd& traj = getTraj();
	VarArray& vars = getVars();
	VectorXd& times = getTimes();
	
	ConvexObjectivePtr out(new ConvexObjective());

	vector<MatrixXd> jacs;
	vector<VectorXd> positions;
	getTrajPositionJacobians(m_robot, m_link, traj, m_dofInds, m_useAffine, jacs, positions);
	
	for (int i=1; i < traj.rows()-1; ++i) {
	  VectorXd disp1 = positions[i+1] - positions[i];
	  VectorXd disp0 = positions[i] - positions[i-1];
		for (int dim = 0; dim < 3; ++dim) {
	    GRBLinExpr veli1 =
	        (disp1(dim)
              + makeDerivExpr(jacs[i+1].row(dim), vars.row(i+1), traj.row(i+1))
              + makeDerivExpr(-jacs[i].row(dim), vars.row(i), traj.row(i)))
                 /(times(i+1)-times(i));
      GRBLinExpr veli0 =
          (disp0(dim)
              + makeDerivExpr(jacs[i].row(dim), vars.row(i), traj.row(i))
              + makeDerivExpr(-jacs[i-1].row(dim), vars.row(i-1), traj.row(i-1)))
                /(times(i)-times(i-1));
      GRBLinExpr acci = (veli1-veli0) / ((times(i+1)-times(i-1))/2);
      out->m_objective += m_coeff * (acci*acci);
		}
	}
	
	return out;			
}

ClosedChainCost::ClosedChainCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link1, KinBody::LinkPtr link2,
  KinBody::LinkPtr heldLink, const vector<int>& dofInds, bool useAffine, double posCoeff, double rotCoeff, bool l1) :
	TrajCost(opt),
  m_robot(robot),
  m_link1(link1),
  m_link2(link2),
  m_heldLink(heldLink),
  m_dofInds(dofInds),
  m_useAffine(useAffine),
  m_posCoeff(posCoeff),
  m_rotCoeff(rotCoeff),
  m_l1(l1)
{}    


double ClosedChainCost::evaluate(const MatrixXd& traj) {

  ScopedRobotSave srs(m_robot);
  
  double out = 0;

  OpenRAVE::Transform heldFromL2;
  OpenRAVE::Transform heldFromL1;

	for (int i=0; i < traj.rows(); ++i) {    

    VectorXd dofvals = traj.row(i);
    if (m_useAffine) setDofVals(m_robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
    else setDofVals(m_robot, m_dofInds, dofvals);
    
    if (i==0) {
      // get heldFromL2
      OpenRAVE::Transform worldFromHeld = m_heldLink->GetTransform();
      OpenRAVE::Transform worldFromL1 = m_link1->GetTransform();
      OpenRAVE::Transform worldFromL2 = m_link2->GetTransform();
      
      heldFromL1 = worldFromHeld.inverse() * worldFromL1;
      heldFromL2 = worldFromHeld.inverse() * worldFromL2;
    }
    else {
      
      OpenRAVE::Transform worldFromHeld = m_heldLink->GetTransform();
      
      OpenRAVE::Transform worldFromHeld_1 = m_link1->GetTransform() * heldFromL1.inverse();
      OpenRAVE::Transform worldFromHeld_2 = m_link2->GetTransform() * heldFromL2.inverse();
      
      OpenRAVE::Transform err1 = worldFromHeld * worldFromHeld_1.inverse();
      assert(err1.trans.lengthsqr3() < 1e-6);
      assert(err1.rot.lengthsqr3() < 1e-6);
      
      OpenRAVE::Transform err2 = worldFromHeld * worldFromHeld_2.inverse();
      
      if (m_l1) {
        out += m_posCoeff * toVector3d(err2.trans).lpNorm<1>();        
        out += m_rotCoeff * toVector3d(err2.rot).lpNorm<1>();        
      }
      else {
        out += m_posCoeff * toVector3d(err2.trans).lpNorm<2>();        
        out += m_rotCoeff * toVector3d(err2.rot).lpNorm<2>();                
      }
                  
      // compare heldFromL2 to actual held transform      
    }        
	}

	return out;
}
ConvexObjectivePtr ClosedChainCost::convexify(GRBModel* model) {

  ScopedRobotSave srs(m_robot);  
  ConvexObjectivePtr out(new ConvexObjective());

  OpenRAVE::Transform heldFromL2;

  MatrixXd& traj = getTraj();

	for (int i=0; i < traj.rows(); ++i) {    

    VectorXd dofvals = traj.row(i);
    if (m_useAffine) setDofVals(m_robot, m_dofInds, dofvals.topRows(m_dofInds.size()), dofvals.bottomRows(3));
    else setDofVals(m_robot, m_dofInds, dofvals);
    
    OpenRAVE::Transform worldFromHeld = m_heldLink->GetTransform();
    OpenRAVE::Transform worldFromL2 = m_link2->GetTransform();      


    if (i==0) {
      // get heldFromL2
      heldFromL2 = worldFromHeld.inverse() * worldFromL2;
    }
    else {
      
      struct F : public fVectorOfVector {
        KinBody::LinkPtr m_heldLink, m_link2;
        F(KinBody::LinkPtr heldLink, KinBody::LinkPtr link2)
          : m_heldLink(heldLink), m_link2(link2)
        {}
        VectorXd operator()(const VectorXd& dofvals) const {
          OpenRAVE::Transform worldFromHeld = m_heldLink->GetTransform();
          OpenRAVE::Transform worldFromL2 = m_link2->GetTransform();      
          OpenRAVE::Transform err2 = worldFromHeld * worldFromHeld.inverse();
          VectorXd out(6);
          out.topRows(3) = toVector3d(err2.trans);
          out.bottomRows(3) = toQuatVector4d(err2.rot).topRows(3);
          return out;
        }
      };
      
      F f(m_heldLink, m_link2);
      MatrixXd errjac = calcJacobian(f, dofvals, 1e-5);
      VectorXd err = f(dofvals);
      

      for (int i=0; i < err.size(); ++i) {
        double coeff = (i < 3) ? m_posCoeff : m_rotCoeff;
        GRBLinExpr erri = err(i) + makeDerivExpr(errjac.row(i), getVars().row(i), dofvals);
        if (m_l1) addAbsCost(out, coeff, erri, model, "pos_l1");
        else  out->m_objective += coeff * (erri * erri);
      }                        
    }        
	}
  return out;
}



struct TrajCvxEvaluator : public fScalarOfMatrix {
  boost::shared_ptr<GRBModel> m_model;
  ConvexObjectivePtr m_cvxobj;
  VarArray m_vars;
  TrajCvxEvaluator(TrajCostPtr cost, int rows, int cols) : m_model(new GRBModel(*getGRBEnv())) {
    m_vars.resize(rows, cols);
    for (int i=0; i < rows; ++i) {
      for (int j=0; j < cols; ++j) {
        m_vars(i,j) = m_model->addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS);
      }
    }
    TIC();
    m_cvxobj = cost->convexify(m_model.get());
    printf("%s convexification time: %.3f\n", cost->getName().c_str(), TOC());
    m_model->update();
    m_cvxobj->addToModel(m_model.get());
    m_model->setObjective(m_cvxobj->m_objective);
  }
  double operator()(const MatrixXd& traj) const {
    vector<GRBConstr> cnts;
    for (int i=0; i < traj.rows(); ++i) {
      for (int j=0; j < traj.cols(); ++j) {
        cnts.push_back(m_model->addConstr(m_vars(i,j) == traj(i,j)));
      }
    }
    m_model->optimize();
    int status = m_model->get(GRB_IntAttr_Status);
    assert(status == GRB_OPTIMAL);
    double out = m_model->get(GRB_DoubleAttr_ObjVal);
    BOOST_FOREACH(GRBConstr cnt, cnts) {
      m_model->remove(cnt);
    }
    return out;
  }
  ~TrajCvxEvaluator() {
    m_cvxobj->removeFromModel(); // (so we don't get a warning)
  }
};

struct TrajCostEvaluator : public fScalarOfMatrix {
  TrajCostPtr m_cost;
  ConvexObjectivePtr m_obj;
  TrajCostEvaluator(TrajCostPtr cost) : m_cost(cost) {}
  double operator()(const MatrixXd& traj) const {
    return m_cost->evaluate(traj);
  }
};


void checkLinearization(TrajCostPtr cost) {
  TrajCvxEvaluator ce0(cost, cost->getLength(), cost->getDof());
  TrajCostEvaluator ce1(cost);
  MatrixXd grad0 = matGrad(ce0, cost->getTraj());
  MatrixXd grad1 = matGrad(ce1, cost->getTraj());
  printf("Testing cost: %s\n", cost->getName().c_str());
  printf("row col approx exact\n");
  printf("--------------------------------\n");
  for (int i=0; i < grad0.rows(); ++i) {
    for (int j=0; j < grad0.cols(); ++j) {
      printf("%i %i %.3e %.3e\n", i, j, grad0(i,j), grad1(i,j));
    }
  }

}

void checkAllLinearizations(TrajOptimizer& opt) {
  BOOST_FOREACH(CostPtr cost, opt.m_costs) {
    checkLinearization(boost::dynamic_pointer_cast<TrajCost>(cost));
  }
}
void checkAllConvexifications(TrajOptimizer&);
void checkConvexification(TrajCost&);

