#pragma once
#include "sqp.h"
#include "utils_sqp.h"
#include "simulation/simulation_fwd.h"
#include "sqp/sqp_fwd.h"
#include "simulation/openrave_fwd.h"
#include <openrave/openrave.h>
#include "sqp/collisions.h"

class btDynamicsWorld;
class TrajPlotter;
typedef boost::shared_ptr<TrajPlotter> TrajPlotterPtr;

class TrajOptimizer : public Optimizer {
public:
	VarArray m_vars;
	MatrixXd m_traj;
	MatrixXd m_traj_backup;
	VectorXd m_times;	
	vector<TrajPlotterPtr> m_plotters;
	bool m_initialized;
	
	TrajOptimizer();	
	void updateValues();
	void storeValues();
	void rollbackValues();
	void initialize(const MatrixXd& traj, const VectorXd& times);
	void preOptimize();
	void postOptimize();
  void subdivide(const vector<double>& insertTimes);
};

void setStartFixed(TrajOptimizer&);
void setEndFixed(TrajOptimizer&);

class TrajComponent {
public:
  TrajOptimizer* m_opt;

  virtual void subdivide(const vector<double>& insertTimes, const VectorXd& oldTimes, const VectorXd& newTimes) {}
	
	TrajComponent(TrajOptimizer*);
	virtual ~TrajComponent();

	MatrixXd& getTraj() {return m_opt->m_traj;}
	VarArray& getVars() {return m_opt->m_vars;}
	int getDof() {return m_opt->m_traj.cols();}
	int getLength() {return m_opt->m_traj.rows();}
	VectorXd& getTimes() {return m_opt->m_times;}
};

class TrajCost : public Cost, public TrajComponent {
public:
  TrajCost(TrajOptimizer*);
  virtual double evaluate(const MatrixXd&)=0;
  double evaluate() {return evaluate(getTraj());}
};

class TrajConstraint : public Constraint, public TrajComponent {
public:
  TrajConstraint(TrajOptimizer*);
  virtual double getViolVal(const MatrixXd&)=0;
  ConvexObjectivePtr getViolExpr(GRBModel* model);
  TrajCostPtr asCost();
};

class TimestepFixed : public Constraint, public TrajComponent {
public:
  int m_step; // if it's negative, then take that many steps from the end
  TimestepFixed(TrajOptimizer* opt, int step);
  ConvexConstraintPtr convexify(GRBModel* model);
};

class CollisionCost : public TrajCost {
public:
  RaveRobotObject* m_robot;
	vector<int> m_dofInds;
  bool m_useAffine;
  double m_coeff;
	
	CollisionCost(TrajOptimizer* opt, RaveRobotObject* robot, const vector<int>& dofInds, bool useAffine, double coeff);
	double evaluate(const MatrixXd&);
  ConvexObjectivePtr convexify(GRBModel* model);	
	string getName() {return "CollisionCost";}
	TrajCartCollInfo discreteCollisionCheck();
	TrajCartCollInfo continuousCollisionCheck();
};

class CartPoseCost : public TrajCost {
public:
  RobotBasePtr m_robot;
  KinBody::LinkPtr m_link;
  vector<int> m_dofInds;
  bool m_useAffine;
  MatrixXd m_posTarg, m_rotTarg;
  VectorXd m_posCoeff, m_rotCoeff;
  bool m_l1;
  bool m_justEnd;

  /* for single timestep */
  CartPoseCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link, const vector<int>& dofInds,
               bool useAffine, const btTransform& goal, double posCoeff, double rotCoeff, bool l1);
  /* for multiple timesteps */
  CartPoseCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link, const vector<int>& dofInds,
               bool useAffine, const vector<btTransform>& goal, const VectorXd& posCoeff, const VectorXd& rotCoeff, bool l1);
  double evaluate(const MatrixXd&);
  ConvexObjectivePtr convexify(GRBModel* model);	
	string getName() {return "CartPoseCost";}
};

class ThisSideUpCost : public TrajCost {
public:
  RobotBasePtr m_robot;
  KinBody::LinkPtr m_link;
  vector<int> m_dofInds;
  bool m_useAffine;
  double m_coeff;
  bool m_l1;

  ThisSideUpCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link, const vector<int>& dofInds,
               bool useAffine, double coeff, bool l1);
  double evaluate(const MatrixXd&);
  ConvexObjectivePtr convexify(GRBModel* model);
  string getName() {return "ThisSideUp";}
};

class JntLenCost : public TrajCost {
public:
  double m_coeff;

  JntLenCost(TrajOptimizer* opt, double coeff);

  double evaluate(const MatrixXd&);
  ConvexObjectivePtr convexify(GRBModel* model);	
	string getName() {return "JntLenCost";}
};

class JointBounds : public TrustRegion, public TrajComponent {
public:
  VectorXd m_maxDiffPerIter;
  VectorXd m_jointLowerLimit;
  VectorXd m_jointUpperLimit;

  JointBounds(TrajOptimizer* opt, const VectorXd& maxDiffPerIter, const VectorXd& jointLowerLimit, const VectorXd& jointUpperLimit);
  ConvexConstraintPtr convexConstraint(GRBModel* model);
  ConvexObjectivePtr convexObjective(GRBModel* model);
	void adjustTrustRegion(double ratio);
};

class SoftTrustRegion : public TrustRegion, public TrajComponent {
public:
  double m_coeff;
  SoftTrustRegion(TrajOptimizer* opt, double coeff);
  ConvexConstraintPtr convexConstraint(GRBModel* model);
  ConvexObjectivePtr convexObjective(GRBModel* model);
	void adjustTrustRegion(double ratio);
};

class CartVelCnt : public TrajConstraint {
public:
  int m_tStart, m_tEnd;
  double m_maxSpeed;
  RobotBasePtr m_robot;
  KinBody::LinkPtr m_link;
  vector<int> m_dofInds;
  bool m_useAffine;
  CartVelCnt(TrajOptimizer* opt, int tStart, int tEnd, double maxSpeed, RobotBasePtr robot,
                         KinBody::LinkPtr link, const vector<int>& dofInds, bool useAffine);
  ConvexConstraintPtr convexify(GRBModel* model);
  double getViolVal(const MatrixXd&);
};

class CartAccCost : public TrajCost {
public:
  RobotBasePtr m_robot;
  KinBody::LinkPtr m_link;
  vector<int> m_dofInds;
  bool m_useAffine;
  double m_coeff;

  CartAccCost(TrajOptimizer* opt, RobotBasePtr robot, KinBody::LinkPtr link, const vector<int>& dofInds,
               bool useAffine, double coeff);
  double evaluate(const MatrixXd&);
  ConvexObjectivePtr convexify(GRBModel* model);
  string getName() {return "CartAccCost";}
};

class ClosedChainCost : public TrajCost {
	RobotBasePtr m_robot;
	KinBody::LinkPtr m_link1, m_link2, m_heldLink;
	vector<int> m_dofInds;
	bool m_useAffine;
	double m_posCoeff, m_rotCoeff;
	bool m_l1;
	
	ClosedChainCost(TrajOptimizer* opt, RobotBasePtr, KinBody::LinkPtr link1, KinBody::LinkPtr link2, KinBody::LinkPtr heldLink,
	  const vector<int>& dofInds, bool useAffine, double posCoeff, double rotCoeff, bool l1);
	double evaluate(const MatrixXd&);
	ConvexObjectivePtr convexify(GRBModel* model);
	string getName() {return "ClosedChainCost";}
};

void checkLinearization(TrajCostPtr);
void checkConvexification(TrajCostPtr);
void checkAllLinearizations(TrajOptimizer&);
void checkAllConvexifications(TrajOptimizer&);
