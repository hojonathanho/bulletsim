#pragma once
#include "utils_sqp.h"
#include "traj_costs.h"
#include "simulation/openravesupport.h"
#include "simulation/fake_gripper.h"
#include "simulation/plotting.h"
#include "gurobi_c++.h"
#include "sqp_fwd.h"
#include "functions.h"

GRBEnv* getGRBEnv();

class Scene;



class PlanningProblem {
public:
  typedef boost::function<void(PlanningProblem*)> Callback;
  VarArray m_trajVars; // trajectory variables in optimization. sometimes the endpoints are not actually added to Gurobi model
  Eigen::MatrixXd m_currentTraj;
  VectorXb m_optMask; // mask that indicates which timesteps are in the optimization
  boost::shared_ptr<GRBModel> m_model;
  std::vector<TrajPlotterPtr> m_plotters;
	std::vector<TrajChangePlotterPtr> m_changePlotters;
  std::vector<ProblemComponentPtr> m_comps;
  bool m_initialized;
  std::vector<Callback> m_callbacks;
  Eigen::VectorXd m_times; // note that these aren't times in meaningful. they're just the integer indices
  TrustRegionAdjusterPtr m_tra; // this is the problem component that determines how much you're allowed to chnage at each iteration
  bool m_exactObjectiveReady, m_approxObjectiveReady;

  PlanningProblem();
  void addComponent(ProblemComponentPtr comp);
  void removeComponent(ProblemComponentPtr comp);
  std::vector<CostFuncPtr> getCostComponents();
  std::vector<ProblemComponentPtr> getConstraintComponents();
  void initialize(const Eigen::MatrixXd& initTraj, bool endFixed);
  void initialize(const Eigen::MatrixXd& initTraj, bool endFixed, const Eigen::VectorXd& times);
  void optimize(int maxIter);
  void addPlotter(TrajPlotterPtr plotter) {
    m_plotters.push_back(plotter);
  }
	void addChangePlotter(TrajChangePlotterPtr plotter) {
		m_changePlotters.push_back(plotter);
	}
  double getApproxCost();
  double getCachedCost();
  void updateModel();
  void optimizeModel();
  void forceOptimizeHere();
  void subdivide(const std::vector<double>& insertTimes); // resample in time (resamples all subproblems)
  void testObjectives(); // checks that the linearizations computed by all of the objectives is correct by comparing getApproxCost and getCachedCost
  void addTrustRegionAdjuster(TrustRegionAdjusterPtr tra); // actually there's only one right now so "add" is wrong
};

class ProblemComponent {
public:
  enum CompAttrs {
    RELAXABLE = 1,
    HAS_CONSTRAINT = 2,
    HAS_COST = 4
  };

  PlanningProblem* m_problem;

  ProblemComponent() {
  }
  virtual ~ProblemComponent() {
  }
  virtual void onAdd() {
  }
  virtual void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) = 0;
  virtual void relax() {
  }
  virtual void onRemove() {
  }
  virtual void reset() {
    onRemove();
    onAdd();
  }
  virtual void subdivide(const std::vector<double>& insertTimes, const Eigen::VectorXd& oldTimes,
                         const Eigen::VectorXd& newTimes) {
  }
  virtual int getAttrs()=0;
};

class CostFunc : public ProblemComponent {
public:
  virtual double getApproxCost()=0; // get convexified objective at current grb variables
	virtual double getCost() {ASSERT_FAIL();}
  virtual double getCachedCost()=0;  // should have been stored by updateModel
  virtual int getAttrs() {
    return HAS_COST;
  }
};

class LinearizedCostFunc : public CostFunc {
public:
  virtual Eigen::MatrixXd getGradient() {
    Eigen::MatrixXd grad;
    double cost;
    getCostAndGradient(cost, grad);
    return grad;
  }
  virtual void getCostAndGradient(double& cost, Eigen::MatrixXd& gradient)=0;
};

class CollisionCost : public CostFunc {
public:
  std::vector<GRBVar> m_vars;
  std::vector<GRBConstr> m_cnts;
  GRBLinExpr m_obj;

  OpenRAVE::RobotBasePtr m_robot;
  btDynamicsWorld* m_world;
  BulletRaveSyncherPtr m_brs;
  vector<int> m_dofInds;

  int m_start, m_end;
  double m_distPen;
  double m_coeff;
  double m_exactObjective;
  TrajCartCollInfo m_cartCollInfo;
  Eigen::VectorXd m_coeffVec;
  void removeVariablesAndConstraints();
  CollisionCost(OpenRAVE::RobotBasePtr robot, btDynamicsWorld* world, BulletRaveSyncherPtr brs,
                const vector<int>& dofInds, double distPen, double coeff) :
    m_robot(robot), m_world(world), m_brs(brs), m_dofInds(dofInds), m_distPen(distPen), m_coeff(coeff) {
  }
  virtual int getAttrs() {
    return HAS_COST;
  }
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getApproxCost() {
    return m_obj.getValue();
  }
  double getCachedCost() {
    return m_exactObjective;
  }
  void onRemove();
  void setCoeff(double coeff) {
    m_coeff = coeff;
  }
  void setCoeffVec(const Eigen::VectorXd& coeffVec) {
    m_coeffVec = coeffVec;
  }
  void subdivide(const std::vector<double>& insertTimes, const Eigen::VectorXd& oldTimes,
                 const Eigen::VectorXd& newTimes);

};

#if 0
class VelScaledCollisionCost : public CollisionCost {
public:
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
};
#endif

#if 0
class CollisionConstraint: public ProblemComponent {
protected:
  std::vector<GRBConstr> m_cnts;
  CollisionCostEvaluatorPtr m_cce;
  double m_distPen;
  double m_coeff;
  bool m_startFixed, m_endFixed;
public:
  CollisionConstraint(bool startFixed, bool endFixed, CollisionCostEvaluatorPtr cce, double safeDist) :
  m_startFixed(startFixed), m_endFixed(endFixed), m_cce(cce), m_distPen(safeDist) {
    m_attrs = RELAXABLE + HAS_CONSTRAINT;
  }
  void onAdd();
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  void onRemove();
  void relax();
};
#endif

class TrustRegionAdjuster : public ProblemComponent {
public:
  int m_shrinkage;
  TrustRegionAdjuster() : m_shrinkage(1) {}
  virtual int getAttrs() {
    return HAS_CONSTRAINT;
  }
  virtual void adjustTrustRegion(double ratio)=0;
};

class LengthConstraintAndCost : public CostFunc {
  // todo: set this
  GRBQuadExpr m_obj;//UNSCALED OBJECTIVE
  std::vector<GRBConstr> m_cnts;
  bool m_startFixed, m_endFixed;
  Eigen::VectorXd m_maxStepMvmt;
  double m_coeff;
  double m_exactObjective;
public:
  LengthConstraintAndCost(bool startFixed, bool endFixed, const Eigen::VectorXd& maxStepMvmt, double coeff) :
    m_startFixed(startFixed), m_endFixed(endFixed), m_maxStepMvmt(maxStepMvmt), m_coeff(coeff) {
  }

  void onAdd();
  double getCachedCost() {return m_exactObjective;}
  double getApproxCost() {return m_coeff * m_obj.getValue();}
  double getCost();
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
    m_exactObjective = getCost();
    objective += m_coeff * m_obj;
  }
  void onRemove();
  void setCoeff(double coeff) {
    m_coeff = coeff;
  }
  void subdivide(const std::vector<double>& insertTimes, const Eigen::VectorXd& oldTimes,
                 const Eigen::VectorXd& newTimes);
  int getAttrs() {
    return HAS_CONSTRAINT + HAS_COST;
  }
};

class JointBounds : public TrustRegionAdjuster {
  // todo: set these in constructor
  Eigen::VectorXd m_jointLowerLimit;
  Eigen::VectorXd m_jointUpperLimit;
  Eigen::VectorXd m_maxDiffPerIter;
  bool m_startFixed, m_endFixed;
  GRBQConstr m_cnt;
  void construct(bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter,
                 OpenRAVE::RobotBase::ManipulatorPtr manip, int extraDofs);

public:
  JointBounds(bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter,
              OpenRAVE::RobotBase::ManipulatorPtr manip);
  JointBounds(bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter,
              OpenRAVE::RobotBase::ManipulatorPtr manip, int extraDofs);

  void onAdd();
  void onRemove() {
  }
  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  void adjustTrustRegion(double ratio);
  int getAttrs() {return HAS_CONSTRAINT;}
  bool m_useBall;
};



class CartesianPoseCost : public CostFunc {
  GRBQuadExpr m_obj;
  RaveRobotObject::Manipulator::Ptr m_manip;
  Eigen::Vector3d m_posTarg;
  Eigen::Vector4d m_rotTarg;
  double m_posCoeff, m_rotCoeff;
  int m_timestep;
public:
  CartesianPoseCost(RaveRobotObject::Manipulator::Ptr manip, const btTransform& target, int timestep, double posCoeff,
                    double rotCoeff) :
    m_manip(manip), m_posTarg(toVector3d(target.getOrigin())), m_rotTarg(toVector4d(target.getRotation())),
        m_posCoeff(posCoeff), m_rotCoeff(rotCoeff), m_timestep(timestep) {
  }

  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getApproxCost() {
    return m_obj.getValue();
  }
  double getCachedCost();
  void subdivide(const std::vector<double>& insertTimes, const Eigen::VectorXd& oldTimes,
                 const Eigen::VectorXd& newTimes);
};

#if 0
class CartesianPoseConstraint : public ProblemComponent {
  std::vector<GRBConstr> m_cnts;
  RaveRobotObject::Manipulator::Ptr m_manip;
  Eigen::Vector3d m_posTarg;
  Eigen::Vector4d m_rotTarg;
  double m_posTol, m_rotTol;
  int m_timestep;
  void removeVariablesAndConstraints();
public:
  CartesianPoseConstraint(RaveRobotObject::Manipulator::Ptr manip, const btTransform& target, int timestep,
                          double posTol, double rotTol) :
    m_manip(manip), m_posTarg(toVector3d(target.getOrigin())), m_rotTarg(toVector4d(target.getRotation())),
        m_posTol(posTol), m_rotTol(rotTol), m_timestep(timestep) {
  }

  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  void onRemove();
  void subdivide(const std::vector<double>& insertTimes, const Eigen::VectorXd& oldTimes,
                 const Eigen::VectorXd& newTimes);
  int getAttrs() {return HAS_CONSTRAINT;}

};
#endif

class CartesianVelConstraint : public ProblemComponent {
  GRBQuadExpr m_obj;
  std::vector<GRBConstr> m_cnts;
  RaveRobotObject::Manipulator::Ptr m_manip;
  Eigen::Vector3d m_posTarg;
  Eigen::Vector4d m_rotTarg;
  int m_start, m_stop;
  double m_maxDist;
  void removeVariablesAndConstraints();
public:
  CartesianVelConstraint(RaveRobotObject::Manipulator::Ptr manip, int start, int stop, double maxDist) :
    m_manip(manip), m_start(start), m_stop(stop), m_maxDist(maxDist) {
  }

  void updateModel(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getApproxCost() {
    return m_obj.getValue();
  }
  void onRemove();
  void subdivide(const std::vector<double>& insertTimes, const Eigen::VectorXd& oldTimes,
                 const Eigen::VectorXd& newTimes);
  int getAttrs() {return HAS_CONSTRAINT;}
};

Eigen::VectorXd defaultMaxStepMvmt(const Eigen::MatrixXd& traj);
Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, int nSteps);
Eigen::MatrixXd makeTraj(RaveRobotObject::Manipulator::Ptr manip, const Eigen::VectorXd& startJoints,
                         const btTransform endTransform, int nSteps);
Eigen::MatrixXd makeTraj(RaveRobotObject::Manipulator::Ptr manip, const std::vector<btTransform>& transforms);
void updateTraj(const VarArray& trajVars, const VectorXb& optmask, Eigen::MatrixXd& traj);



/*
Utilities for gradient testing
*/

struct CostFuncEvaluator : public fScalarOfMatrix {
  CostFuncPtr m_comp;
	CostFuncEvaluator(CostFuncPtr comp) : m_comp(comp) {}
	CostFuncEvaluator(CostFuncEvaluator& other) : m_comp(other.m_comp) {}
	double operator()(const Eigen::MatrixXd& in) const {
		Eigen::MatrixXd& traj = m_comp->m_problem->m_currentTraj;
		Eigen::MatrixXd savedTraj = traj;
		traj = in;
		double out =  m_comp->getCost();
		traj = savedTraj;
		return out;
	}
};

struct CostGradEvaluator : public fMatrixOfMatrix {
  LinearizedCostFuncPtr m_comp;
  CostGradEvaluator(LinearizedCostFuncPtr comp) : m_comp(comp) {}
  CostGradEvaluator(CostGradEvaluator& other) : m_comp(other.m_comp) {}
  Eigen::MatrixXd operator()(const Eigen::MatrixXd& in) const {
    Eigen::MatrixXd& traj = m_comp->m_problem->m_currentTraj;
    Eigen::MatrixXd savedTraj = traj;
    traj = in;
    Eigen::MatrixXd out =  m_comp->getGradient();
    traj = savedTraj;
    return out;
  }
};

