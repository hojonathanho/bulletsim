#pragma once
#include "utils_sqp.h"
#include "traj_costs.h"
#include "kinematics_utils.h"
#include "simulation/openravesupport.h"
#include "simulation/fake_gripper.h"
#include "simulation/plotting.h"
#include "gurobi_c++.h"

class Scene;

typedef BasicArray<GRBVar> VarArray;
typedef std::vector<GRBVar> VarVector;

class TrajPlotter {
public:
  typedef boost::shared_ptr<TrajPlotter> Ptr;
	virtual void plotTraj(const Eigen::MatrixXd& traj) = 0;
};

class GripperPlotter : public TrajPlotter {
public:
  std::vector<FakeGripper::Ptr> m_grippers;
  PlotCurve::Ptr m_curve;
  RaveRobotObject::Manipulator::Ptr m_rrom;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  int m_decimation;
  void plotTraj(const Eigen::MatrixXd& traj);
  void clear();
  GripperPlotter(RaveRobotObject::Manipulator::Ptr, Scene*, int decimation=1);
  ~GripperPlotter();
  void setNumGrippers(int n);
};

class ArmPlotter : public TrajPlotter {
public:
  typedef boost::shared_ptr<ArmPlotter> Ptr;
  RaveRobotObject::Manipulator::Ptr m_rrom;
  std::vector<BulletObject::Ptr> m_origs;
  BasicArray<FakeObjectCopy::Ptr> m_fakes;
  Scene* m_scene;
  osg::Group* m_osgRoot;
  int m_decimation;
  BulletRaveSyncher* m_syncher;
  void plotTraj(const Eigen::MatrixXd& traj);
  ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, Scene* scene, CollisionCostEvaluator& cce, int decimation=1);
  ArmPlotter(RaveRobotObject::Manipulator::Ptr rrom, const std::vector<BulletObject::Ptr>& origs, Scene* scene, BulletRaveSyncher*syncher, int decimation);
  void init(RaveRobotObject::Manipulator::Ptr, const std::vector<BulletObject::Ptr>&, Scene*, BulletRaveSyncher*, int decimation);
  void setLength(int n);
};

GRBEnv* getGRBEnv();


typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;

class ProblemComponent {
protected:
  VarArray m_trajVars;
  GRBModel* m_model;
  int m_nSteps, m_nJoints;
public:
  typedef boost::shared_ptr<ProblemComponent> Ptr;
  virtual void init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model);
  virtual void update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) = 0;
  virtual double getValue() {
    return 0;
  }
};

class CollisionCost: public ProblemComponent {
protected:
  std::vector<GRBVar> m_vars;
  std::vector<GRBConstr> m_cnts;
  GRBLinExpr m_obj;
  CollisionCostEvaluator* m_cce;
  double m_safeDistMinusPadding;
  double m_coeff;
  bool m_startFixed, m_endFixed;
public:
  CollisionCost(bool startFixed, bool endFixed, CollisionCostEvaluator* cce, double safeDistMinusPadding, double coeff) :
    m_startFixed(startFixed), m_endFixed(endFixed), m_cce(cce), m_safeDistMinusPadding(safeDistMinusPadding), m_coeff(coeff) {}
  void init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model) {
    ProblemComponent::init(traj, trajVars, model);
  }
  void update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getValue() {
    return m_obj.getValue();
  }
};

class LengthConstraintAndCost : public ProblemComponent {
  // todo: set this
  GRBQuadExpr m_obj;
  bool m_startFixed, m_endFixed;
  Eigen::VectorXd m_maxStepMvmt;
  double m_coeff;
public:
  LengthConstraintAndCost(bool startFixed, bool endFixed, const Eigen::VectorXd& maxStepMvmt, double coeff) :
    m_startFixed(startFixed), m_endFixed(endFixed), m_maxStepMvmt(maxStepMvmt), m_coeff(coeff) {}

  void init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model);
  void update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective) {
    objective += m_obj;
  }
  double getValue() {
    return m_obj.getValue();
  }
};

class JointBounds: public ProblemComponent {
  // todo: set these in constructor
  Eigen::VectorXd m_jointLowerLimit;
  Eigen::VectorXd m_jointUpperLimit;
  Eigen::VectorXd m_maxDiffPerIter;
  bool m_startFixed, m_endFixed;
public:
  JointBounds(bool startFixed, bool endFixed, const Eigen::VectorXd& maxDiffPerIter, OpenRAVE::RobotBase::ManipulatorPtr manip) :
    m_startFixed(startFixed), m_endFixed(endFixed), m_maxDiffPerIter(maxDiffPerIter) {
    vector<int> armInds = manip->GetArmIndices();
    m_jointLowerLimit.resize(armInds.size());
    m_jointUpperLimit.resize(armInds.size());
    vector<double> ul, ll;
    for (int i=0; i < armInds.size(); ++i) {
      manip->GetRobot()->GetJointFromDOFIndex(armInds[i])->GetLimits(ll, ul);
      m_jointLowerLimit(i) = ll[0];
      m_jointUpperLimit(i) = ul[0];
    }
  }


  void init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model);
  void update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
};

class CartesianPoseCost: public ProblemComponent {
  GRBQuadExpr m_obj;
  OpenRAVE::RobotBasePtr m_robot;
  OpenRAVE::RobotBase::ManipulatorPtr m_manip;
  Eigen::Vector3d m_posTarg;
  Eigen::Vector4d m_rotTarg;
  VarVector m_timestepVars;
  double m_posCoeff, m_rotCoeff;
  int m_timestep;
public:
  CartesianPoseCost(OpenRAVE::RobotBasePtr robot, OpenRAVE::RobotBase::ManipulatorPtr manip,
      const btTransform& target, int timestep, double posCoeff, double rotCoeff) :
        m_robot(robot),
        m_manip(manip),
        m_posTarg(toVector3d(target.getOrigin())),
        m_rotTarg(toVector4d(target.getRotation())),
        m_posCoeff(posCoeff),
        m_rotCoeff(rotCoeff),
        m_timestep(timestep) {}

  void init(const Eigen::MatrixXd& traj, const VarArray& trajVars, GRBModel* model) {
    ProblemComponent::init(traj, trajVars, model);
  }
  void update(const Eigen::MatrixXd& traj, GRBQuadExpr& objective);
  double getValue() {
    return m_obj.getValue();
  }

};


Eigen::VectorXd defaultMaxStepMvmt(const Eigen::MatrixXd& traj);
Eigen::MatrixXd makeTraj(const Eigen::VectorXd& startJoints, const Eigen::VectorXd& endJoints, int nSteps);
Eigen::MatrixXd makeTraj(RaveRobotObject::Manipulator::Ptr manip, const Eigen::VectorXd& startJoints, const btTransform endTransform, int nSteps);
Eigen::MatrixXd makeTraj(RaveRobotObject::Manipulator::Ptr manip, const std::vector<btTransform>& transforms);
void updateTraj(const VarArray& trajVars, Eigen::MatrixXd& traj);


class ArmPlanningProblem {
  // represents a single optimization problem and stores the solution
public:
  VarArray m_trajVars;  
  Eigen::MatrixXd m_currentTraj;
  boost::shared_ptr<GRBModel> m_model;
  GRBQuadExpr m_pathLengthCost;
  CollisionCostEvaluator::Ptr m_cce;
  TrajPlotter::Ptr m_atp;
  int m_nJoints;
  ArmPlanningProblem();
  void setPlotter(TrajPlotter::Ptr atp) {m_atp = atp;}
  void setup(RaveRobotObject::Manipulator::Ptr rrom, RaveRobotObject::Ptr rro, btCollisionWorld* world);
  void optimize(int maxIter=100);
  virtual void doIteration() = 0;
  Eigen::MatrixXd& getTraj() {return m_currentTraj;}
};

class GetArmToJointGoal : public ArmPlanningProblem {
  Eigen::VectorXd m_maxStepMvmt;
  Eigen::VectorXd m_maxDiffPerIter;
  TrajPlotter::Ptr m_trajPlotter;
public:
  void setProblem(const Eigen::VectorXd& start, const Eigen::VectorXd& end, int nSteps);
  virtual void doIteration();
};


class GetArmToCartGoal : public ArmPlanningProblem {
  Eigen::MatrixXd m_currentTraj;
  float m_currentObjective;
  Eigen::VectorXd m_maxStepMvmt;
  Eigen::VectorXd m_maxDiffPerIter;
  VarArray m_trajVars;
  TrajPlotter::Ptr m_trajPlotter;
public:
  void setProblem(const Eigen::VectorXd& start, const btTransform& end, int nSteps);
  virtual void doIteration();
};


class ComponentizedArmPlanningProblem : public ArmPlanningProblem {
  std::vector<ProblemComponent::Ptr> m_comps;
public:
  void addComponent(ProblemComponent::Ptr comp) {
    m_comps.push_back(comp);
  }
  void initialize(Eigen::MatrixXd& initTraj);

  void doIteration();

};

